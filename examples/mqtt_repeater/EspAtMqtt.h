#pragma once
// ---------------------------------------------------------------------------
// EspAtMqtt — drive a RAK2305 (ESP32-WROVER running ESP-AT) over UART to
// publish MeshCore packet events to an MQTT broker.
//
// WHY this exists (see memory project_mqtt_repeater): the RAK3401's internal
// SX1262 owns both chip-select lines the RAK13800 could use, so on-board
// Ethernet (SPI) is impossible on that Core. The RAK2305 talks over UART1
// instead — no SPI, no CS collision — and its ESP-AT firmware can do
// MQTT-over-TLS *and* MQTT-over-WebSocket-Secure (scheme 7-10), so the ESP32
// carries the whole TLS/WSS stack the nRF52 can't fit. The nRF52 only ever
// emits AT command strings on Serial1 (P0.15 RX / P0.16 TX on the RAK3401).
//
// DESIGN — non-blocking. A repeater must never stall LoRa RX, but an AT round
// trip over WiFi/WSS can take hundreds of ms. So the packet hook only ENQUEUES
// a pre-formatted JSON record (microseconds); loop() drains the queue through a
// non-blocking AT state machine, one publish at a time, with a '>' prompt /
// +MQTTPUB:OK handshake (AT+MQTTPUBRAW — raw bytes, no JSON escaping needed).
//
// Topic / payload layout matches Cisien/meshcoretomqtt (LetsMesh observers):
//   <prefix>/<IATA>/<PUBKEY>/status    retained "online"/"offline" (+ LWT)
//   <prefix>/<IATA>/<PUBKEY>/packets   one JSON event per unique RX packet
//   <prefix>/<IATA>/<PUBKEY>/debug     periodic heartbeat / "mqtt test"
//
// Persistence: /mqtt.cfg   CLI: handleCommand() at the bottom.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <RTClib.h>
#include <Mesh.h>
#include <target.h>     // global radio_driver (getLastRSSI), board, rtc_clock

#ifndef MQTT_AT_BAUD
  #define MQTT_AT_BAUD            115200      // RAK2305 ESP-AT default
#endif
#ifndef MQTT_DEFAULT_PORT
  #define MQTT_DEFAULT_PORT       443         // LetsMesh WSS endpoint
#endif
#ifndef MQTT_DEFAULT_SCHEME
  // 1=TCP 2-5=TLS 6=WS 7=WSS(no verify) 8=WSS(verify CA) ...
  #define MQTT_DEFAULT_SCHEME     7
#endif
#ifndef MQTT_KEEPALIVE_S
  #define MQTT_KEEPALIVE_S        120
#endif
#ifndef MQTT_STATUS_INTERVAL_MS
  // Periodic /status keepalive. Beacon marks an observer online only if a /status
  // OR /packets message arrived within the last 5 min (hardcoded server-side;
  // /debug does not count). Refresh well inside that window so quiet gaps between
  // bursty mesh packets don't flip the observer offline.
  #define MQTT_STATUS_INTERVAL_MS 120000UL
#endif
#ifndef MQTT_BACKOFF_MIN_MS
  #define MQTT_BACKOFF_MIN_MS     5000UL
#endif
#ifndef MQTT_ONLINE_SETTLE_MS
  // Hold the first publish briefly after +MQTTCONNECTED so AT+MQTTPUBRAW doesn't
  // race the tail of the connect response (which yields "busy p..." + a failed
  // publish, and can wedge/reset the ESP-AT stack).
  #define MQTT_ONLINE_SETTLE_MS   1000UL
#endif
#ifndef MQTT_PUBFAIL_RECONNECT
  // Consecutive publish failures that force a reconnect (stale-link backstop).
  #define MQTT_PUBFAIL_RECONNECT  5
#endif
#ifndef MQTT_RSSI_FEM_OFFSET
  // dB subtracted from the SX1262's reported RSSI to recover antenna-referred
  // power on the RAK13302 1W module: its SKY66122 LNA adds ~13-15 dB of RX gain
  // that the radio doesn't account for. Starting estimate — calibrate empirically
  // against a non-FEM reference node and adjust with `mqtt rssioffset <dB>`.
  // 0 = raw RSSI (for non-FEM boards).
  #define MQTT_RSSI_FEM_OFFSET    13
#endif
#ifndef MQTT_RFPOWER_DEFAULT
  // ESP-AT WiFi TX power cap, in 0.25 dBm units (module default is 78 = 19.5 dBm).
  // Lowering it cuts the WiFi current spikes that brown out the RAK2305 on a weak
  // rail. 52 = 13 dBm — a solid reduction that still associates from in-building.
  // Range ~8-84; set 78 to disable the reduction. Tune with `mqtt rfpower <n>`.
  #define MQTT_RFPOWER_DEFAULT    52
#endif
#ifndef MQTT_BACKOFF_MAX_MS
  #define MQTT_BACKOFF_MAX_MS     60000UL
#endif
#ifndef MQTT_MAX_PAYLOAD
  // Must hold a full packet's `raw` hex (up to 2*MAX_TRANS_UNIT = 510 chars)
  // plus the surrounding meshcoretomqtt JSON envelope (~280 chars).
  #define MQTT_MAX_PAYLOAD        1024
#endif
#ifndef MQTT_QUEUE_LEN
  #define MQTT_QUEUE_LEN          8
#endif
#define MQTT_RX_BUF               320

// Leaf topics
#define LEAF_STATUS   0
#define LEAF_PACKETS  1
#define LEAF_DEBUG    2

class EspAtMqtt {
  Stream*      _at;          // UART to the RAK2305 (Serial1)
  FILESYSTEM*  _fs;
  mesh::RTCClock* _rtc;

  // ---- persisted config (/mqtt.cfg) ----
  bool     _enabled;
  char     _ssid[33];
  char     _wifi_pass[64];
  char     _host[80];
  uint16_t _port;
  uint8_t  _scheme;
  char     _user[64];
  char     _pass[64];
  char     _ws_path[40];     // websocket resource path (scheme >= 6)
  char     _prefix[24];      // default "meshcore"
  char     _iata[8];         // 3-letter location code

  // ---- runtime identity ----
  const char* _node_name;
  float _freq, _bw; uint8_t _sf, _cr;   // radio params for /status
  uint16_t _batt_mv; int16_t _noise;    // observer telemetry (fed from main.cpp)
  uint32_t _tx_air, _rx_air, _recv_err;
  // Full radio/mesh stats snapshot (fed from main.cpp; pushed to ESP32 via MCSTA
  // so the observer's /status carries the REAL nRF52 stats, not the ESP32's blanks).
  uint32_t _stat_queue, _stat_uptime, _stat_pkts_sent, _stat_pkts_recv;
  uint16_t _stat_errflags;
  int8_t   _rssi_offset;                 // dB subtracted from reported RSSI (FEM/LNA gain)
  uint8_t  _rfpower;                      // ESP-AT WiFi TX power cap (0.25dBm units)
  char     _pubkey_hex[2 * PUB_KEY_SIZE + 1];
  char     _privkey_hex[2 * PRV_KEY_SIZE + 1];   // for UART_UPLINK identity push
  char     _client_id[40];

  // ---- connection state machine ----
  enum CState { CS_OFF, CS_BRINGUP, CS_ONLINE, CS_BACKOFF };
  CState   _cstate;
  uint8_t  _step;
  bool     _step_sent;
  unsigned long _step_deadline;
  unsigned long _backoff_until;
  uint32_t _backoff_ms;

  // ---- publish sub-state ----
  enum PState { PUB_IDLE, PUB_WAITPROMPT, PUB_WAITRESULT };
  PState   _pstate;
  unsigned long _pub_deadline;
  unsigned long _next_status;   // next periodic /status keepalive
  unsigned long _online_settle;   // hold first publish until this time after connect

  // ---- AT rx accumulator ----
  char     _rx[MQTT_RX_BUF];
  int      _rxlen;
  bool     _verbose;
  bool     _wifi_got_ip;
  bool     _hw_seen;          // any byte ever received from the ESP-AT module
  bool     _no_hw;            // gave up — no RAK2305 detected, repeater-only
  uint8_t  _bringup_fails;    // consecutive failed bring-up cycles

  // ---- publish queue (ring) ----
  struct PubItem { uint8_t leaf; bool retain; uint16_t len; char payload[MQTT_MAX_PAYLOAD]; };
  PubItem  _q[MQTT_QUEUE_LEN];
  uint8_t  _qhead, _qtail, _qcount;

  // ---- counters ----
  uint32_t _pub_ok, _pub_fail, _pkts_seen, _q_drops, _conn_attempts;
  uint16_t _pub_fail_run;   // consecutive publish failures -> reconnect backstop

  // bring-up steps
  enum { ST_ATE0, ST_CWMODE, ST_RFPOWER, ST_CWJAP, ST_USERCFG, ST_CONNCFG, ST_CONN, ST_PUBSTATUS, ST_DONE };

  /* ===================== small helpers ===================== */
  static char hexNib(uint8_t n) { return n < 10 ? ('0' + n) : ('A' + n - 10); }
  static void toHex(char* dst, const uint8_t* src, int len) {
    for (int i = 0; i < len; i++) { dst[2*i] = hexNib(src[i] >> 4); dst[2*i+1] = hexNib(src[i] & 0xF); }
    dst[2*len] = 0;
  }
  void clearRx() { _rxlen = 0; _rx[0] = 0; }
  bool seen(const char* tok) { return strstr(_rx, tok) != nullptr; }
  bool sawOK()    { return seen("OK\r\n"); }
  bool sawError() { return seen("ERROR"); }

  void buildClientId() {
    // <prefix>_<first 3 bytes of pubkey>
    snprintf(_client_id, sizeof(_client_id), "%s_%c%c%c%c%c%c",
             _prefix[0] ? _prefix : "meshcore",
             _pubkey_hex[0], _pubkey_hex[1], _pubkey_hex[2],
             _pubkey_hex[3], _pubkey_hex[4], _pubkey_hex[5]);
  }

  // The RAK2305 ESP-AT build (ESP-IDF v4.0) caps MQTT topic strings at 64 chars
  // (measured: 62 OK, 83 ERROR, on both MQTTPUB and MQTTCONNCFG). The full 64-hex
  // pubkey makes an 83-char topic that the firmware rejects outright, so the key
  // segment MUST be truncated — the full key cannot live in the topic on this HW.
  // The truncation is forced to an EVEN length so the consumer's hex-decode of the
  // topic pubkey succeeds (odd-length hex fails to parse). The full untruncated key
  // is carried in the payload's origin_id instead. Budget uses the LONGEST leaf
  // ("packets"=7) so the key is identical across status/packets/debug.
  #define MQTT_TOPIC_MAX 64
  void topicFor(uint8_t leaf, char* buf, int n) {
    const char* l = (leaf == LEAF_STATUS) ? "status" : (leaf == LEAF_PACKETS) ? "packets" : "debug";
    const char* prefix = _prefix[0] ? _prefix : "meshcore";
    const char* iata   = _iata[0]   ? _iata   : "XXX";
    const char* key    = _pubkey_hex[0] ? _pubkey_hex : "00";
    const int LONGEST_LEAF = 7;  // "packets"
    int fixed  = (int)strlen(prefix) + 1 + (int)strlen(iata) + 1 + 1 + LONGEST_LEAF;
    int keymax = MQTT_TOPIC_MAX - fixed; if (keymax < 0) keymax = 0;
    keymax &= ~1;                                  // even budget -> valid hex
    int keylen = (int)strlen(key);   if (keylen > keymax) keylen = keymax;
    keylen &= ~1;                                  // never emit an odd-length key
    snprintf(buf, n, "%s/%s/%.*s/%s", prefix, iata, keylen, key, l);
  }

  int isoTime(char* buf, int n) {
    uint32_t now = _rtc ? _rtc->getCurrentTime() : 0;
    DateTime dt(now);
    return snprintf(buf, n, "%04u-%02u-%02uT%02u:%02u:%02uZ",
                    dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
  }

  // send "<cmd>\r\n" to the ESP-AT and reset the rx accumulator
  void sendLine(const char* cmd) {
    if (_verbose) { Serial.print("[AT>] "); Serial.println(cmd); }
    _at->print(cmd);
    _at->print("\r\n");
    clearRx();
  }

  /* ===================== queue ===================== */
  bool enqueue(uint8_t leaf, const char* payload, bool retain) {
    if (_qcount >= MQTT_QUEUE_LEN) { _q_drops++; return false; }
    PubItem& it = _q[_qhead];
    it.leaf = leaf; it.retain = retain;
    strncpy(it.payload, payload, MQTT_MAX_PAYLOAD - 1);
    it.payload[MQTT_MAX_PAYLOAD - 1] = 0;
    it.len = (uint16_t)strlen(it.payload);
    _qhead = (_qhead + 1) % MQTT_QUEUE_LEN;
    _qcount++;
    return true;
  }
  PubItem* qfront() { return _qcount ? &_q[_qtail] : nullptr; }
  void qpop() { if (_qcount) { _qtail = (_qtail + 1) % MQTT_QUEUE_LEN; _qcount--; } }

  /* ===================== AT rx pump ===================== */
  void pump() {
    while (_at->available()) {
      char c = (char)_at->read();
      _hw_seen = true;   // something is answering on the UART → a module is present
      if (_verbose) Serial.write(c);
      if (_rxlen >= MQTT_RX_BUF - 1) {
        // keep the tail half so long responses / prompts aren't lost
        memmove(_rx, _rx + MQTT_RX_BUF / 2, MQTT_RX_BUF / 2);
        _rxlen = MQTT_RX_BUF / 2;
      }
      _rx[_rxlen++] = c;
      _rx[_rxlen] = 0;
    }
  }

  /* ===================== bring-up ===================== */
  void issueStep(unsigned long now) {
    char cmd[200];
    char topic[140];
    unsigned long to = 5000;  // default per-step timeout
    switch (_step) {
      case ST_ATE0:    strcpy(cmd, "ATE0"); to = 2000; break;
      case ST_CWMODE:  strcpy(cmd, "AT+CWMODE=1"); break;
      case ST_RFPOWER: // cap WiFi TX power (0.25dBm units) to limit current spikes
        snprintf(cmd, sizeof(cmd), "AT+RFPOWER=%u", (unsigned)_rfpower); break;
      case ST_CWJAP:
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", _ssid, _wifi_pass);
        to = 20000;  // association + DHCP
        _wifi_got_ip = false;
        break;
      case ST_USERCFG:
        buildClientId();
        snprintf(cmd, sizeof(cmd),
                 "AT+MQTTUSERCFG=0,%u,\"%s\",\"%s\",\"%s\",0,0,\"%s\"",
                 (unsigned)_scheme, _client_id, _user, _pass, _ws_path);
        break;
      case ST_CONNCFG:
        topicFor(LEAF_STATUS, topic, sizeof(topic));
        // LWT publishes to /status on unexpected drop, so it must be valid JSON
        // too (Beacon json.Unmarshals /status). "{}" is the minimal valid object
        // and needs no AT-string quote-escaping.
        snprintf(cmd, sizeof(cmd),
                 "AT+MQTTCONNCFG=0,%u,0,\"%s\",\"{}\",0,1",
                 (unsigned)MQTT_KEEPALIVE_S, topic);
        break;
      case ST_CONN:
        snprintf(cmd, sizeof(cmd),
                 "AT+MQTTCONN=0,\"%s\",%u,1", _host, (unsigned)_port);
        to = 20000;  // TLS/WSS handshake
        _conn_attempts++;
        break;
      default: return;
    }
    sendLine(cmd);
    _step_sent = true;
    _step_deadline = now + to;
  }

  void advanceStep() { _step++; _step_sent = false; }

  void failBringup(const char* why) {
    Serial.printf("[MQTT] bring-up failed at step %u (%s)\n", _step, why);
    // If the module never answered a single byte across a couple of cycles,
    // there's no RAK2305 attached — stop hammering the UART and let the board
    // run as a plain repeater. 'mqtt retry' (or 'mqtt on') re-probes.
    if (!_hw_seen && ++_bringup_fails >= 2) {
      Serial.println("[MQTT] no RAK2305 detected on UART1 — running repeater-only. Use 'mqtt retry' to re-probe.");
      _no_hw = true;
      _cstate = CS_BACKOFF; _backoff_until = millis() + 0xFFFFFFF;  // dormant until retry
      _step = ST_ATE0; _step_sent = false;
      return;
    }
    if (_backoff_ms < MQTT_BACKOFF_MAX_MS) _backoff_ms *= 2;
    _backoff_until = millis() + _backoff_ms;
    _cstate = CS_BACKOFF;
    _step = ST_ATE0; _step_sent = false;
  }

  void serviceBringup(unsigned long now) {
    // ST_PUBSTATUS is a virtual finalize step that sends no AT command, so it
    // must run BEFORE the issueStep guard below: issueStep() has no case for it
    // and its default-return never sets _step_sent, which would otherwise loop
    // here forever — connected to the broker but never transitioning to online.
    // (Latent until the first successful MQTTCONN.)
    if (_step == ST_PUBSTATUS) {
      _backoff_ms = MQTT_BACKOFF_MIN_MS;
      clearRx();                              // drop the connect-response residue
      queueStatus();                          // retained JSON /status on connect
      _online_settle = now + MQTT_ONLINE_SETTLE_MS;
      _next_status = now + MQTT_STATUS_INTERVAL_MS;
      _pstate = PUB_IDLE;
      _cstate = CS_ONLINE;
      Serial.printf("[MQTT] online: %s:%u scheme=%u as %s\n",
                    _host, (unsigned)_port, (unsigned)_scheme, _client_id);
      return;
    }
    if (!_step_sent) { issueStep(now); return; }
    bool timeout = (long)(now - _step_deadline) >= 0;

    switch (_step) {
      case ST_ATE0:   // tolerate failure (echo may already be off / ESP booting)
        if (sawOK() || sawError() || timeout) advanceStep();
        break;
      case ST_CWMODE:
        if (sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("CWMODE");
        break;
      case ST_RFPOWER:  // non-critical: continue even if the build rejects it
        if (sawOK() || sawError() || timeout) advanceStep();
        break;
      case ST_CWJAP:
        if (seen("WIFI GOT IP")) _wifi_got_ip = true;
        if (_wifi_got_ip && sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("CWJAP (wifi?)");
        break;
      case ST_USERCFG:
        if (sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("MQTTUSERCFG");
        break;
      case ST_CONNCFG:
        if (sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("MQTTCONNCFG");
        break;
      case ST_CONN:
        // ESP-AT can emit "+MQTTCONNECTED" then "ERROR" (e.g. CONNECT URC plus a
        // benign command-level error); treat the URC as authoritative success.
        if (seen("+MQTTCONNECTED") || sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("MQTTCONN (broker?)");
        break;
      // ST_PUBSTATUS handled at top of serviceBringup (no AT command).
    }
  }

  /* ===================== steady-state publish ===================== */
  void serviceOnline(unsigned long now) {
    if (seen("+MQTTDISCONNECTED")) { failBringup("disconnected"); return; }
    // The ESP can brown-out/reset under WiFi load and silently drop MQTT without
    // a +MQTTDISCONNECTED — leaving us "online" but publishing into a dead socket
    // forever. Detect a reset (boot banner) or an unsolicited WiFi re-association,
    // and reconnect. The publish-failure run is the backstop for any other stall
    // (covers the active case; the banner/re-assoc covers the idle case).
    if (seen("ready\r\n") || seen("WIFI GOT IP")) { failBringup("esp reset"); return; }
    if (_pub_fail_run >= MQTT_PUBFAIL_RECONNECT) {
      Serial.println("[MQTT] publish stall — forcing reconnect");
      _pub_fail_run = 0; failBringup("pub stall"); return;
    }
    // Post-connect settle: don't start the first publish (or a heartbeat) until
    // the ESP has finished the connect handshake — avoids the busy/race.
    if (_pstate == PUB_IDLE && (long)(now - _online_settle) < 0) return;

    switch (_pstate) {
      case PUB_IDLE: {
        PubItem* it = qfront();
        if (!it) {  // nothing queued — heartbeat if due
          if ((long)(now - _next_status) >= 0) {
            queueStatus();
            _next_status = now + MQTT_STATUS_INTERVAL_MS;
          }
          return;
        }
        char topic[140];
        topicFor(it->leaf, topic, sizeof(topic));
        char cmd[200];
        snprintf(cmd, sizeof(cmd), "AT+MQTTPUBRAW=0,\"%s\",%u,0,%u",
                 topic, (unsigned)it->len, it->retain ? 1u : 0u);
        sendLine(cmd);
        _pstate = PUB_WAITPROMPT;
        _pub_deadline = now + 8000;
        break;
      }
      case PUB_WAITPROMPT: {
        if (seen(">")) {
          PubItem* it = qfront();
          if (it) { _at->write((const uint8_t*)it->payload, it->len);  // raw, no CRLF
                    if (_verbose) { Serial.print("[AT>] <"); Serial.print(it->len); Serial.println(" raw bytes>"); } }
          clearRx();
          _pstate = PUB_WAITRESULT;
          _pub_deadline = now + 10000;
        } else if (sawError() || (long)(now - _pub_deadline) >= 0) {
          qpop(); _pub_fail++; _pub_fail_run++; _pstate = PUB_IDLE;
        }
        break;
      }
      case PUB_WAITRESULT: {
        if (seen("+MQTTPUB:OK"))        { qpop(); _pub_ok++; _pub_fail_run = 0; _pstate = PUB_IDLE; }
        else if (seen("+MQTTPUB:FAIL")) { qpop(); _pub_fail++; _pub_fail_run++; _pstate = PUB_IDLE; }
        else if ((long)(now - _pub_deadline) >= 0) { qpop(); _pub_fail++; _pub_fail_run++; _pstate = PUB_IDLE; }
        break;
      }
    }
  }

  // Build + enqueue a retained JSON /status. Published on connect and every
  // MQTT_STATUS_INTERVAL_MS — this is the application-level keepalive that holds
  // the observer "online" in Beacon between bursty mesh packets. (The MQTT-level
  // keepalive only keeps the broker connection up; Beacon ignores it.)
  void queueStatus() {
    static char sj[512];
    // "radio" must be exactly "freq,bw,sf,cr" (MHz,kHz,sf,cr) or Beacon skips it.
    // "stats" carries the observer telemetry Beacon charts (battery, noise floor,
    // airtime TX/RX, receive errors, queue) — fed from main.cpp via setStats().
    snprintf(sj, sizeof(sj),
      "{\"source\":\"meshcoretomqtt\",\"model\":\"RAK3401\",\"origin\":\"%s\","
      "\"origin_id\":\"%s\",\"radio\":\"%.3f,%.1f,%u,%u\",\"stats\":{"
      "\"uptime_secs\":%lu,\"battery_mv\":%u,\"noise_floor\":%d,\"queue_len\":%u,"
      "\"tx_air_secs\":%lu,\"rx_air_secs\":%lu,\"recv_errors\":%lu,"
      "\"pkts_seen\":%lu,\"pub_ok\":%lu,\"pub_fail\":%lu}}",
      _node_name ? _node_name : "node", _pubkey_hex,
      (double)_freq, (double)_bw, (unsigned)_sf, (unsigned)_cr,
      (unsigned long)(millis() / 1000), (unsigned)_batt_mv, (int)_noise,
      (unsigned)_qcount, (unsigned long)_tx_air, (unsigned long)_rx_air,
      (unsigned long)_recv_err, (unsigned long)_pkts_seen,
      (unsigned long)_pub_ok, (unsigned long)_pub_fail);
    enqueue(LEAF_STATUS, sj, true);
  }

public:
  EspAtMqtt()
    : _at(nullptr), _fs(nullptr), _rtc(nullptr),
      _enabled(false), _port(MQTT_DEFAULT_PORT), _scheme(MQTT_DEFAULT_SCHEME),
      _node_name(nullptr), _freq(0), _bw(0), _sf(0), _cr(0),
      _batt_mv(0), _noise(0), _tx_air(0), _rx_air(0), _recv_err(0),
      _stat_queue(0), _stat_uptime(0), _stat_pkts_sent(0), _stat_pkts_recv(0),
      _stat_errflags(0),
      _rssi_offset(MQTT_RSSI_FEM_OFFSET), _rfpower(MQTT_RFPOWER_DEFAULT),
      _cstate(CS_OFF), _step(ST_ATE0), _step_sent(false),
      _step_deadline(0), _backoff_until(0), _backoff_ms(MQTT_BACKOFF_MIN_MS),
      _pstate(PUB_IDLE), _pub_deadline(0), _next_status(0), _online_settle(0),
      _rxlen(0), _verbose(false), _wifi_got_ip(false),
      _hw_seen(false), _no_hw(false), _bringup_fails(0),
      _qhead(0), _qtail(0), _qcount(0),
      _pub_ok(0), _pub_fail(0), _pkts_seen(0), _q_drops(0), _conn_attempts(0),
      _pub_fail_run(0)
  {
    _ssid[0] = _wifi_pass[0] = _host[0] = _user[0] = _pass[0] = 0;
    _ws_path[0] = _iata[0] = _pubkey_hex[0] = _client_id[0] = _privkey_hex[0] = 0;
    strncpy(_prefix, "meshcore", sizeof(_prefix));
  }

  void setStream(Stream* s)        { _at = s; }
  void setRTC(mesh::RTCClock* r)   { _rtc = r; }
  void setNodeName(const char* n)  { _node_name = n; }
  // Fire-and-forget relay of a raw command to the ESP32 (framed as MCCMD, same as
  // the `esp <cmd>` console path but without blocking for the reply). Used by the
  // load-shed state machine to push `net shed` / `net resume` on battery events.
  void sendEspCommand(const char* cmd) {
    if (!_at) return;
    _at->print("MCCMD "); _at->print(cmd); _at->print("\r\n");
  }
  // freq in MHz, bw in kHz, sf, cr — for the /status "radio" field (Beacon
  // wants exactly "freq,bw,sf,cr" or it skips the radio info).
  void setRadio(float freq, float bw, uint8_t sf, uint8_t cr) {
    _freq = freq; _bw = bw; _sf = sf; _cr = cr;
  }
  // Observer telemetry snapshot for the /status "stats" block. Pushed from the
  // main loop (throttled) since the bridge can't reach board/mesh/radio directly.
  void setStats(uint16_t batt_mv, int16_t noise, uint32_t tx_air_s,
                uint32_t rx_air_s, uint32_t recv_err,
                uint32_t queue_len, uint32_t uptime_s, uint16_t err_flags,
                uint32_t pkts_sent, uint32_t pkts_recv) {
    _batt_mv = batt_mv; _noise = noise;
    _tx_air = tx_air_s; _rx_air = rx_air_s; _recv_err = recv_err;
    _stat_queue = queue_len; _stat_uptime = uptime_s; _stat_errflags = err_flags;
    _stat_pkts_sent = pkts_sent; _stat_pkts_recv = pkts_recv;
  }
  void setPubKey(const uint8_t* k, int len) {
    int n = len > PUB_KEY_SIZE ? PUB_KEY_SIZE : len;
    toHex(_pubkey_hex, k, n);
  }
  // The node's private key, pushed to the ESP32 over UART (UART_UPLINK) so it
  // adopts this node's identity and publishes as one logical device.
  void setPrivKey(const uint8_t* k, int len) {
    int n = len > PRV_KEY_SIZE ? PRV_KEY_SIZE : len;
    toHex(_privkey_hex, k, n);
  }
  bool isEnabled()  const { return _enabled; }
  bool isOnline()   const { return _cstate == CS_ONLINE; }

  /* ===================== persistence ===================== */
  void load(FILESYSTEM* fs) {
    _fs = fs;
    if (!_fs || !_fs->exists("/mqtt.cfg")) return;
    File f = _fs->open("/mqtt.cfg");
    if (!f) return;
    char buf[512];
    int len = f.read((uint8_t*)buf, sizeof(buf) - 1);
    f.close();
    if (len <= 0) return;
    buf[len] = 0;
    auto val = [](const char* src, const char* key, char* dst, int dn) {
      const char* p = strstr(src, key); if (!p) return;
      p += strlen(key); int i = 0;
      while (*p && *p != '\n' && *p != '\r' && i < dn - 1) dst[i++] = *p++;
      dst[i] = 0;
    };
    char tmp[16];
    tmp[0] = 0;  // val() leaves dst untouched when the key is missing
    val(buf, "enabled=", tmp, sizeof(tmp)); if (tmp[0]) _enabled = atoi(tmp);
    val(buf, "ssid=",    _ssid, sizeof(_ssid));
    val(buf, "wpass=",   _wifi_pass, sizeof(_wifi_pass));
    val(buf, "host=",    _host, sizeof(_host));
    tmp[0] = 0;
    val(buf, "port=",    tmp, sizeof(tmp)); if (tmp[0]) _port = (uint16_t)atoi(tmp);
    tmp[0] = 0;
    val(buf, "scheme=",  tmp, sizeof(tmp)); if (tmp[0]) _scheme = (uint8_t)atoi(tmp);
    val(buf, "user=",    _user, sizeof(_user));
    val(buf, "mpass=",   _pass, sizeof(_pass));
    val(buf, "wspath=",  _ws_path, sizeof(_ws_path));
    val(buf, "prefix=",  _prefix, sizeof(_prefix));
    val(buf, "iata=",    _iata, sizeof(_iata));
    tmp[0] = 0;
    val(buf, "rssioff=", tmp, sizeof(tmp)); if (tmp[0]) _rssi_offset = (int8_t)atoi(tmp);
    tmp[0] = 0;
    val(buf, "rfpow=", tmp, sizeof(tmp)); if (tmp[0]) _rfpower = (uint8_t)atoi(tmp);
    Serial.printf("[MQTT] cfg: en=%d ssid=%s host=%s:%u scheme=%u iata=%s\n",
                  (int)_enabled, _ssid, _host, (unsigned)_port, (unsigned)_scheme,
                  _iata[0] ? _iata : "(unset)");
  }

  void save() {
    if (!_fs) { Serial.println("[MQTT] save FAILED: no filesystem"); return; }
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    _fs->remove("/mqtt.cfg");
    File f = _fs->open("/mqtt.cfg", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = _fs->open("/mqtt.cfg", "w");
#else
    File f = _fs->open("/mqtt.cfg", "w", true);
#endif
    if (!f) { Serial.println("[MQTT] save FAILED: open /mqtt.cfg"); return; }
    char buf[512];
    int n = snprintf(buf, sizeof(buf),
      "enabled=%d\nssid=%s\nwpass=%s\nhost=%s\nport=%u\nscheme=%u\n"
      "user=%s\nmpass=%s\nwspath=%s\nprefix=%s\niata=%s\nrssioff=%d\nrfpow=%u\n",
      (int)_enabled, _ssid, _wifi_pass, _host, (unsigned)_port, (unsigned)_scheme,
      _user, _pass, _ws_path, _prefix, _iata, (int)_rssi_offset, (unsigned)_rfpower);
    int w = f.write((const uint8_t*)buf, n);
    f.close();
    if (w != n) Serial.printf("[MQTT] save FAILED: short write %d/%d\n", w, n);
  }

  /* ===================== packet hook (fast, non-blocking) ===================== */
  // Called once per unique received packet from MqttMeshTables::hasSeen().
  // Only formats + enqueues — never touches the UART.
  void onPacketReceived(const mesh::Packet* pkt) {
    if (!_enabled || !pkt) return;
    // Skip internally-generated packets that never came off the air with real
    // radio metrics — the dispatcher leaves _snr==0 for these (multipart-ACK
    // reconstruction, loopback). Publishing them pollutes the feed with bogus
    // SNR 0.00 / stale-RSSI observations. Genuine receptions set _snr (Dispatcher
    // sets pkt->_snr = getLastSNR()*4 on rx); a real SNR of exactly 0 is vanishingly
    // rare and harmless to drop.
    if (pkt->getSNR() == 0.0f) return;
    _pkts_seen++;
    if (_cstate != CS_ONLINE && _qcount >= MQTT_QUEUE_LEN) return;  // avoid churn while offline

    // The full over-the-air frame, hex-encoded, is the `raw` field the map
    // decoders (meshcore-decoder / meshcore-go) need to recover node identity,
    // position, adverts and routes. Without it the packet cannot be decoded.
    // Buffers are static (single-threaded packet hook) to keep this off the stack.
    static uint8_t raw[MAX_TRANS_UNIT];
    static char    raw_hex[2 * MAX_TRANS_UNIT + 1];
    static char    p[MQTT_MAX_PAYLOAD];
    uint8_t raw_len = pkt->writeTo(raw);
    toHex(raw_hex, raw, raw_len);

    // Correct for the RAK13302 FEM/LNA gain so RSSI is antenna-referred (see
    // _rssi_offset / `mqtt rssioffset`). SNR is unaffected by the LNA.
    int rssi = (int)radio_driver.getLastRSSI() - _rssi_offset;
    float snr = pkt->getSNR();

#ifdef UART_UPLINK
    // UART-uplink mode: the RAK2305 now runs native MeshCore MQTT firmware and
    // owns WiFi/NTP/MQTT/JWT, the observer identity, the topic, and the timestamp.
    // The nRF52 just ships the raw frame + radio metrics in a minimal line:
    //     MCPKT <raw_hex> <SNR f.2> <RSSI int>\r\n
    // The ESP32 re-parses <raw_hex> into a packet and feeds storeRawRadioData().
    // No origin_id/timestamp here — the ESP32 supplies its own observer identity
    // and NTP-sourced time, and (unlike ESP-AT) carries the full untruncated key.
    snprintf(p, sizeof(p), "MCPKT %s %.2f %d", raw_hex, (double)snr, rssi);
    enqueue(LEAF_PACKETS, p, false);
#else
    char ts[40]; isoTime(ts, sizeof(ts));
    // Minimal meshcoretomqtt packet envelope (Beacon / meshmapper). `raw` is the
    // ONLY required field — the broker-side decoder rebuilds hash/type/path/etc.
    // from it. Earlier we also sent numeric len/payload_len/packet_type; Beacon's
    // struct types those differently (e.g. len is a string), so json.Unmarshal
    // failed with "malformed packet envelope". Keys must be exactly these, and
    // SNR/RSSI MUST be upper-case (lower-case is silently ignored -> 0).
    snprintf(p, sizeof(p),
      "{\"raw\":\"%s\",\"origin_id\":\"%s\",\"timestamp\":\"%s\",\"SNR\":%.2f,\"RSSI\":%d}",
      raw_hex, _pubkey_hex, ts, (double)snr, rssi);
    enqueue(LEAF_PACKETS, p, false);
#endif
  }

  /* ===================== main loop ===================== */
  void loop() {
#ifdef UART_UPLINK
    // No AT handshake in uplink mode — the RAK2305 runs native MQTT firmware and
    // reads our framed lines off Serial1. Just drain the queue, one frame per
    // call so a burst can't stall LoRa RX, newline-terminated for line framing.
    // (The ESP32 publishes its own /status heartbeat, so none is sent here.)
    if (!_enabled || !_at) return;
    unsigned long now = millis();
    // Receive UTC time from the ESP32 (it has NTP; this node has no RTC):
    // "MCTIME <utc_epoch>" -> set our clock so adverts/packets are stamped right.
    {
      static char trl[48]; static int trli = 0;
      while (_at->available()) {
        char c = (char)_at->read();
        if (c == '\n' || c == '\r') {
          if (trli > 0) {
            trl[trli] = 0;
            if (_rtc && strncmp(trl, "MCTIME ", 7) == 0) {
              uint32_t epoch = (uint32_t)strtoul(trl + 7, nullptr, 10);
              if (epoch > 1735689600UL) _rtc->setCurrentTime(epoch);
            }
            trli = 0;
          }
        } else if (trli < (int)sizeof(trl) - 1) {
          trl[trli++] = c;
        } else { trli = 0; }
      }
    }
    // Push this node's identity + info to the ESP32 so the two act as ONE logical
    // device: the ESP32 publishes AS this node (same key/name), with this node's
    // real radio config and battery. nRF52 is the source of truth; sent
    // periodically so it survives an ESP32 reboot and tracks any change.
    static unsigned long next_sync = 0;
    if ((long)(now - next_sync) >= 0) {
      next_sync = now + 15000UL;
      char ln[160];
      if (_privkey_hex[0]) {                       // identity (so ESP32 = this node)
        snprintf(ln, sizeof(ln), "MCIDENT %s", _privkey_hex); _at->print(ln); _at->print("\r\n");
      }
      if (_node_name && _node_name[0]) {           // observer name
        snprintf(ln, sizeof(ln), "MCNAME %s", _node_name); _at->print(ln); _at->print("\r\n");
      }
      snprintf(ln, sizeof(ln), "MCRADIO %.3f %.1f %u %u",   // radio config
               (double)_freq, (double)_bw, (unsigned)_sf, (unsigned)_cr); _at->print(ln); _at->print("\r\n");
      snprintf(ln, sizeof(ln), "MCBATT %u", (unsigned)_batt_mv);  // battery mV
      _at->print(ln); _at->print("\r\n");
      // Full stats snapshot so the ESP32 /status reports the REAL radio/mesh stats
      // (it has no radio of its own). Order: noise tx_air rx_air recv_err queue
      // uptime errflags pkts_sent pkts_recv. Battery goes via MCBATT (above).
      snprintf(ln, sizeof(ln), "MCSTA %d %lu %lu %lu %lu %lu %u %lu %lu",
               (int)_noise,
               (unsigned long)_tx_air, (unsigned long)_rx_air, (unsigned long)_recv_err,
               (unsigned long)_stat_queue, (unsigned long)_stat_uptime,
               (unsigned)_stat_errflags,
               (unsigned long)_stat_pkts_sent, (unsigned long)_stat_pkts_recv);
      _at->print(ln); _at->print("\r\n");
    }
    PubItem* it = qfront();
    if (it) {
      _at->write((const uint8_t*)it->payload, it->len);
      _at->print("\r\n");
      qpop();
      _pub_ok++;
    }
#else
    if (!_enabled || !_at || _no_hw) return;
    pump();
    unsigned long now = millis();
    switch (_cstate) {
      case CS_OFF:     _step = ST_ATE0; _step_sent = false; _wifi_got_ip = false;
                       _cstate = CS_BRINGUP; break;
      case CS_BRINGUP: serviceBringup(now); break;
      case CS_ONLINE:  serviceOnline(now); break;
      case CS_BACKOFF: if ((long)(now - _backoff_until) >= 0) _cstate = CS_OFF; break;
    }
#endif
  }

  /* ===================== CLI ===================== */
  // Returns true if the command was one of ours.
  bool handleCommand(const char* cmd, char* reply) {
#ifdef UART_UPLINK
    // Relay a CLI command to the RAK2305 observer over UART1 and surface its
    // reply here — so the USB-less ESP32 is configurable from this MeshCore CLI
    // (and, via the command hook, the remote-admin interface). Examples:
    //   esp set wifi.ssid <s>   /  esp set mqtt2.preset rflab  /  esp get mqtt.status
    if (strncmp(cmd, "esp ", 4) == 0) {
      if (!_at) { strcpy(reply, "no UART"); return true; }
      while (_at->available()) _at->read();              // drop stale rx
      _at->print("MCCMD "); _at->print(cmd + 4); _at->print("\r\n");
      char line[200]; int li = 0; bool got = false;
      reply[0] = 0;
      unsigned long until = millis() + 2500;             // wait for the reply
      while ((long)(millis() - until) < 0) {
        while (_at->available()) {
          char c = (char)_at->read();
          if (c == '\n' || c == '\r') {
            if (li > 0) {
              line[li] = 0;
              if (strncmp(line, "MCRSP ", 6) == 0) {
                Serial.print("  [esp] "); Serial.println(line + 6);
                if (!got) { strncpy(reply, line + 6, 159); reply[159] = 0; got = true; }
                until = millis() + 300;                  // brief tail for extra lines
              }
              li = 0;
            }
          } else if (li < (int)sizeof(line) - 1) {
            line[li++] = c;
          } else {
            li = 0;
          }
        }
      }
      if (!got) strcpy(reply, "(no response from RAK2305)");
      return true;
    }
#endif
    if (strcmp(cmd, "mqtt status") == 0) {
      const char* st = _no_hw ? "no-2305" :
                       _cstate == CS_ONLINE ? "online" :
                       _cstate == CS_BRINGUP ? "connecting" :
                       _cstate == CS_BACKOFF ? "backoff" : "off";
      snprintf(reply, 160,
        "%s [%s] ssid=%s %s:%u scheme=%u iata=%s q=%u/%u pkts=%lu ok=%lu fail=%lu drop=%lu",
        _enabled ? "ON" : "OFF", st, _ssid[0] ? _ssid : "?", _host[0] ? _host : "?",
        (unsigned)_port, (unsigned)_scheme, _iata[0] ? _iata : "?",
        (unsigned)_qcount, (unsigned)MQTT_QUEUE_LEN,
        (unsigned long)_pkts_seen, (unsigned long)_pub_ok, (unsigned long)_pub_fail,
        (unsigned long)_q_drops);
      return true;
    }
    if (strcmp(cmd, "mqtt cfg") == 0) {  // debug: ground truth on persistence
      if (!_fs) { strcpy(reply, "no filesystem"); return true; }
      if (!_fs->exists("/mqtt.cfg")) { strcpy(reply, "/mqtt.cfg MISSING"); return true; }
      File f = _fs->open("/mqtt.cfg");
      if (!f) { strcpy(reply, "/mqtt.cfg exists but open failed"); return true; }
      char buf[512];
      int len = f.read((uint8_t*)buf, sizeof(buf) - 1);
      f.close();
      if (len < 0) len = 0;
      buf[len] = 0;
      Serial.printf("[MQTT] /mqtt.cfg (%d bytes):\n%s", len, buf);
      snprintf(reply, 160, "%d bytes", len);
      return true;
    }
    if (strncmp(cmd, "mqtt at ", 8) == 0) {  // debug: raw AT passthrough, blocks ~3s
      if (!_at) { strcpy(reply, "no UART"); return true; }
      while (_at->available()) _at->read();   // drop stale rx
      _at->print(cmd + 8); _at->print("\r\n");
      unsigned long until = millis() + 3000;
      int n = 0;
      while ((long)(millis() - until) < 0) {
        while (_at->available()) {
          char c = (char)_at->read();
          Serial.write(c);
          n++; _hw_seen = true;
        }
      }
      snprintf(reply, 160, "%s%d bytes in 3s", n ? "\n  -> " : "", n);
      return true;
    }
    if (strcmp(cmd, "mqtt on") == 0)  { _enabled = true;  save();
                                        _no_hw = false; _hw_seen = false; _bringup_fails = 0;
                                        _backoff_ms = MQTT_BACKOFF_MIN_MS; _cstate = CS_OFF;
                                        strcpy(reply, "MQTT enabled"); return true; }
    if (strcmp(cmd, "mqtt retry") == 0){ _no_hw = false; _hw_seen = false; _bringup_fails = 0;
                                        _backoff_ms = MQTT_BACKOFF_MIN_MS; _cstate = CS_OFF;
                                        strcpy(reply, _enabled ? "Re-probing RAK2305..." : "MQTT is off (use 'mqtt on')"); return true; }
    if (strcmp(cmd, "mqtt off") == 0) { _enabled = false; save();
                                        if (_at) sendLine("AT+MQTTCLEAN=0");
                                        _cstate = CS_OFF; strcpy(reply, "MQTT disabled"); return true; }
    if (strncmp(cmd, "wifi ssid ", 10) == 0) { strncpy(_ssid, cmd + 10, sizeof(_ssid)-1); _ssid[sizeof(_ssid)-1]=0; save(); snprintf(reply,160,"SSID: %s",_ssid); return true; }
    if (strncmp(cmd, "wifi pass ", 10) == 0) { strncpy(_wifi_pass, cmd + 10, sizeof(_wifi_pass)-1); _wifi_pass[sizeof(_wifi_pass)-1]=0; save(); strcpy(reply,"WiFi password set"); return true; }
    if (strncmp(cmd, "mqtt host ", 10) == 0) { strncpy(_host, cmd + 10, sizeof(_host)-1); _host[sizeof(_host)-1]=0; save(); snprintf(reply,160,"Host: %s",_host); return true; }
    if (strncmp(cmd, "mqtt port ", 10) == 0) { _port = (uint16_t)atoi(cmd + 10); save(); snprintf(reply,160,"Port: %u",(unsigned)_port); return true; }
    if (strncmp(cmd, "mqtt scheme ", 12) == 0){ _scheme = (uint8_t)atoi(cmd + 12); save(); snprintf(reply,160,"Scheme: %u (1=TCP 7/8=WSS)",(unsigned)_scheme); return true; }
    if (strncmp(cmd, "mqtt user ", 10) == 0) { strncpy(_user, cmd + 10, sizeof(_user)-1); _user[sizeof(_user)-1]=0; save(); snprintf(reply,160,"User: %s",_user); return true; }
    if (strncmp(cmd, "mqtt mpass ", 11) == 0) { strncpy(_pass, cmd + 11, sizeof(_pass)-1); _pass[sizeof(_pass)-1]=0; save(); strcpy(reply,"MQTT password set"); return true; }
    if (strncmp(cmd, "mqtt path ", 10) == 0) { strncpy(_ws_path, cmd + 10, sizeof(_ws_path)-1); _ws_path[sizeof(_ws_path)-1]=0; save(); snprintf(reply,160,"WS path: %s",_ws_path); return true; }
    if (strncmp(cmd, "mqtt prefix ", 12) == 0){ strncpy(_prefix, cmd + 12, sizeof(_prefix)-1); _prefix[sizeof(_prefix)-1]=0; save(); snprintf(reply,160,"Prefix: %s",_prefix); return true; }
    if (strncmp(cmd, "mqtt iata ", 10) == 0) { strncpy(_iata, cmd + 10, sizeof(_iata)-1); _iata[sizeof(_iata)-1]=0; for(char*p=_iata;*p;p++)*p=toupper((int)*p); save(); snprintf(reply,160,"IATA: %s",_iata); return true; }
    if (strncmp(cmd, "mqtt rssioffset ", 16) == 0){ _rssi_offset = (int8_t)atoi(cmd + 16); save(); snprintf(reply,160,"RSSI offset: %d dB (reported - offset)",(int)_rssi_offset); return true; }
    if (strncmp(cmd, "mqtt rfpower ", 13) == 0){ int v=atoi(cmd+13); if(v<8)v=8; if(v>84)v=84; _rfpower=(uint8_t)v; save(); snprintf(reply,160,"WiFi TX power: %u (%.2f dBm) - reconnect to apply",(unsigned)_rfpower,_rfpower*0.25); return true; }
    if (strcmp(cmd, "mqtt rfpower") == 0)        { snprintf(reply,160,"WiFi TX power: %u (%.2f dBm)",(unsigned)_rfpower,_rfpower*0.25); return true; }
    if (strcmp(cmd, "mqtt rssioffset") == 0)       { snprintf(reply,160,"RSSI offset: %d dB",(int)_rssi_offset); return true; }
    if (strcmp(cmd, "mqtt pubkey") == 0)    { snprintf(reply,160,"PubKey: %s",_pubkey_hex); return true; }
    if (strcmp(cmd, "mqtt verbose on") == 0){ _verbose = true;  strcpy(reply,"AT trace ON"); return true; }
    if (strcmp(cmd, "mqtt verbose off")== 0){ _verbose = false; strcpy(reply,"AT trace OFF"); return true; }
    if (strcmp(cmd, "mqtt test") == 0) {
      char ts[40]; isoTime(ts, sizeof(ts));
      static char p[MQTT_MAX_PAYLOAD];
      snprintf(p, sizeof(p), "{\"origin\":\"%s\",\"origin_id\":\"%s\",\"timestamp\":\"%s\",\"type\":\"TEST\"}",
               _node_name ? _node_name : "node", _pubkey_hex, ts);
      bool ok = enqueue(LEAF_DEBUG, p, false);
      strcpy(reply, ok ? "Test queued" : "Queue full");
      return true;
    }
    if (strcmp(cmd, "mqtt help") == 0) {
      Serial.println("--- MQTT/WiFi (RAK2305 ESP-AT) ---");
      Serial.println("wifi ssid <s> / wifi pass <p>");
      Serial.println("mqtt host <h> / port <n> / scheme <1|7|8>");
      Serial.println("mqtt user <u> / mpass <p> / path <wspath>");
      Serial.println("mqtt prefix <p> / iata <CODE> / pubkey / rssioffset <dB> / rfpower <8-84>");
      Serial.println("mqtt on / off / retry / status / test / verbose on|off");
      Serial.println("debug: mqtt cfg / mqtt at <raw AT cmd> / mqtt baud <n>  (use with mqtt off)");
      strcpy(reply, "See Serial output");
      return true;
    }
    return false;
  }
};
