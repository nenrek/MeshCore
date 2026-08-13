# Remote Fleet Control Plane

**Status:** Proposal · **Scope:** 6 MQTT observers (cellular + WiFi) · **Base:** agessaman 1.17 (`cellular-mqtt-uplink-117`, `rak2305-observer-117`)

Managing geographically-distributed MeshCore observers over the links they already
hold open — turning the MQTT broker from a data sink into a bidirectional control plane.

> Rendered version with diagrams: the design artifact (control-plane topology + command
> round-trip sequence). This doc is the code-grounded companion, and carries the Phase 1
> implementation spec the artifact doesn't.

---

## The idea

The hard constraint is topology, not software. The cellular nodes sit behind carrier NAT;
the field nodes are miles apart. **Nothing can reach in.** But every observer already reaches
*out* to the broker and keeps that socket open — so the broker is the one place every node is
simultaneously reachable.

Each node **subscribes** to a per-node command topic. A control service on the shop swarm
publishes commands there; the node routes them through its **existing CLI**, executes, and
publishes a result. Because the CLI is already the configuration surface, every `set`/`get`
becomes remotely callable the moment subscribe works.

This kills the two field pains directly:
- Config can't silently drift — the reconciler re-applies it.
- A post-reflash config wipe **self-heals** — the node reports defaults, the reconciler pushes the deltas back. No USB, no site visit.

## Topic scheme

Everything stays under each node's existing subtree, so one ACL (`meshcore/<IATA>/<DEVICE_ID>/#`)
covers uplink and downlink. `DEVICE_ID` = the node's public-key hex (already keys the status topics).

```
meshcore/<IATA>/<DEVICE_ID>/status     # existing · retained · QoS1 · health + config snapshot
meshcore/<IATA>/<DEVICE_ID>/packets    # existing · QoS1
meshcore/<IATA>/<DEVICE_ID>/cmd        # NEW · node SUBSCRIBES · one command per message
meshcore/<IATA>/<DEVICE_ID>/ack        # NEW · node PUBLISHES · {id, cmd, result, ts}
```

Command payload — a small JSON envelope:

```json
{ "id": "a1f2", "seq": 4127, "cmd": "set cell.gps on", "sig": "<ed25519>" }
```

`seq` = monotonic replay guard; `sig` = optional signature over `id|seq|cmd`.

## Security (defense-in-depth — a compromised broker must not brick a node)

| Layer | Mechanism | Stops |
|---|---|---|
| Broker ACL | only the control-service user may publish `…/cmd`; a node subscribes only to its own subtree | other clients issuing commands |
| Allowlist | firmware refuses commands outside a safe set (config get/set, reboot, advert, gps); no firmware/identity/key writes | bricking, key exfiltration |
| Replay guard | monotonic persisted `seq`; reject stale/duplicate | captured commands re-fired |
| Ed25519 sig (optional) | control service signs; node verifies vs a baked control-plane pubkey | a compromised broker forging commands |

Destructive/fleet-wide commands additionally route through the existing **approval broker**
(submit → ntfy tap → publish).

## Control service & desired state

A small service on the shop swarm (Python + Docker), fed by git-versioned `fleet.yaml`:

- **Inventory** — subscribe every `…/status` + `…/ack`; track firmware, live config, reboot count, last-seen (SQLite).
- **Reconcile** — diff reported vs desired; publish only the deltas as signed commands; retry on next status.
- **Dispatch** — `fleetctl cmd NM_R01 "get cell"`.
- **Alert** — version/config drift, offline, reboot-storm → ntfy.

## Firmware updates (staged)

- **ESP32 web-OTA** already works on the WiFi observers (remote today if the site net is reachable).
- **DFU-over-cellular** (BG77 downloads a signed blob → nRF52 self-DFU) is the only true remote path for the cellular fleet — a milestone of its own (Phase 7).
- **DFU-over-LoRa is off the table** (hundreds of KB over a duty-cycle-limited link).

## Phases

1. Command downlink — **cellular** (firmware, M) ← *spec below*
2. Command downlink — **WiFi** (firmware, S)
3. Security hardening — ACLs + allowlist + replay + signing (firmware+broker, M)
4. Control service — inventory + `fleetctl` dispatch (service, M)
5. Desired-state reconciler + `fleet.yaml` (service, M)
6. Fleet observability — drift alerts, inventory panel (service, S)
7. DFU-over-cellular (future, L)

## Out of scope

The **weather & asset nodes are LoRa-only** — no IP uplink. Managed via LoRa mesh admin or a
co-located gateway, not this control plane.

---

# Appendix A — Phase 1 spec (cellular subscribe path)

Grounded in the current `cellular-mqtt-uplink-117` code. The BG77 path is small because the
`ST_READY` tick already reads lines watching for a URC (`+QMTSTAT:`), and there's an established
callback-with-ctx idiom (`TimeSyncCb`) to mirror.

## A.1 — `BG77Modem` (subscribe + receive)

**Subscribe.** `openConnectTick()` (`src/helpers/cellular/BG77Modem.cpp`) is the phase sequence
that ends in `enterState(ST_READY)` (`BG77Modem.cpp:448`). Append two phase steps after the
`QMTCONN` step, so the subscription is live before the modem reports ready — and, because
`openConnectTick()` re-runs on every reconnect, it re-subscribes automatically after a drop:

```cpp
// … after QMTCONN succeeds, before returning AT_OK:
case N:   // URC push mode: incoming messages arrive as +QMTRECV with the payload inline
  atTick("AT+QMTCFG=\"recv/mode\"," STR(MQTT_CLIENT_IDX) ",0,1", "OK", T_CFG);  // -> step N+1
case N+1: // subscribe to the per-node command topic (QoS1)
  snprintf(cmd, sizeof(cmd), "AT+QMTSUB=%u,1,\"%s\",1", MQTT_CLIENT_IDX, _cmd_topic);
  atTick(cmd, "+QMTSUB:", T_CFG);  // -> AT_OK
```

**Receive.** The `ST_READY` case already does `readLine(5)` + `strstr(_line, "+QMTSTAT:")`
(`BG77Modem.cpp:454-457`). Add one branch — parse `+QMTRECV: <idx>,<mid>,<topic>,<len>,<payload>`
and fire a callback (no CLI work here; this runs inside the modem tick):

```cpp
case ST_READY: {
  int n = readLine(5);
  if (n > 0) {
    if (strstr(_line, "+QMTSTAT:"))      enterBackoff("broker disconnect");
    else if (strstr(_line, "+QMTRECV:")) parseAndDispatchRecv(_line);   // NEW
  }
  break;
}
```

**Callback** — mirror `TimeSyncCb` / `setTimeSyncCallback` exactly (`BG77Modem.h:153`):

```cpp
typedef void (*RecvCb)(void* ctx, const char* topic, const char* payload, int len);
void setRecvCallback(RecvCb cb, void* ctx) { _recv_cb = cb; _recv_ctx = ctx; }
void setCommandTopic(const char* t);   // stored in _cmd_topic, QMTSUB'd in openConnectTick
```

Notes:
- `+QMTRECV` payload length is bounded; commands are tiny — no `PACKET_BUF` concern.
- Parsing stays inside the non-blocking tick — **never block on receive** (the 07-14 mesh-loop-starvation lesson).

## A.2 — `CellularMQTTBridge` (route + ack)

Mirror the time-sync wiring in `begin()` (`CellularMQTTBridge.cpp:113`
`_modem.setTimeSyncCallback(&CellularMQTTBridge::onModemTime, this)`):

```cpp
// begin():
buildTopic(MSG_CMD, cmd_topic, sizeof(cmd_topic));   // meshcore/<iata>/<devid>/cmd
_modem.setCommandTopic(cmd_topic);
_modem.setRecvCallback(&CellularMQTTBridge::onModemRecv, this);

// static — fires from the modem tick; stage only, do NOT run the CLI here
static void onModemRecv(void* ctx, const char* topic, const char* payload, int len) {
  auto* self = (CellularMQTTBridge*)ctx;
  self->_inbox_len = min(len, (int)sizeof(self->_inbox)-1);
  memcpy(self->_inbox, payload, self->_inbox_len);
  self->_inbox[self->_inbox_len] = 0;
  self->_inbox_ready = true;
}
```

Drain in `loop()` (alongside the existing publish drain): if a command is staged, hand it to the
command sink, then enqueue the ack via the existing `enqueue()` / `drainOne()` path:

```cpp
if (_inbox_ready) {
  char reply[160];
  if (_cmd_sink) _cmd_sink->runRemoteCommand(_inbox, reply, sizeof(reply));
  char ack_topic[128]; buildTopic(MSG_ACK, ack_topic, sizeof(ack_topic));
  enqueue(ack_topic, reply, strlen(reply), 1, false);   // QoS1, not retained
  _inbox_ready = false;
}
```

Add `MSG_CMD` / `MSG_ACK` to the `MsgType` enum + `buildTopic()` (parity with `status`/`packets`).

## A.3 — `MyMesh` (execute via the existing CLI)

The bridge routes to a sink implemented by `MyMesh` — reuse `handleCommand()`
(`MyMesh.cpp:859`), the *same* entry point the mesh remote-admin path already uses, so an
MQTT command and a LoRa admin command execute identically:

```cpp
// interface the bridge holds:  void setCommandSink(RemoteCommandSink* s);
int MyMesh::runRemoteCommand(const char* json, char* reply, size_t n) override {
  // 1. parse envelope {id, seq, cmd, sig}
  // 2. verify seq monotonic (persisted) + optional sig  ── Phase 3 hardening
  // 3. allowlist-check cmd                                ── Phase 3 hardening
  // 4. handleCommand(0, cmd, reply);   // sender_timestamp 0 = local/trusted console
  // 5. format {id, result:reply} into reply
}
```

Register it in `setBridgeState()` under `WITH_CELLULAR_MQTT_BRIDGE`, next to the existing
`setStatsSources` wiring.

## A.4 — The one ordering subtlety

A command that restarts the bridge (`set cell.server`, `set cell.tls…`) drops the very
connection the ack travels on. So for the disruptive-config subset: **ack first, apply after.**
Simplest: `runRemoteCommand` returns the reply, the bridge enqueues + flushes the ack, *then*
`restartBridge` fires on the next loop. The reconciler confirms the change on the node's next
status regardless.

## A.5 — Bench validation

1. USB cell node, `MQTT_DEBUG` on.
2. Reach `mqtt=up`; confirm `+QMTSUB` success in the AT echo.
3. `mosquitto_pub -t meshcore/ATW/<devid>/cmd -m '{"id":"t1","seq":1,"cmd":"get cell"}'`.
4. Expect the node to execute and publish to `…/ack`; confirm via `mosquitto_sub`.
5. Confirm the LoRa RX cadence / advert timing never stalls while commands flow (the whole point).
