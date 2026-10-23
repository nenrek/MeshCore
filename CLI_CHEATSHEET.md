# MeshCore CLI cheat sheet — Meshwerks fleet

Commands across all custom builds, grouped by firmware. Base: **MeshCore v1.18.0** (fleet builds `cell.15` / `uart.11` / `wifiobs.17` / `asset.3`).
Convention: `get <key>` reads a value, `set <key> <value>` writes it. Passwords are shown as
`<placeholders>` — fill in your own.

> **Builds in this repo (`meshcore-rak4631`, branch `uart-uplink`):**
> `uart` (RAK_3401_uart_uplink) · `asset` (RAK_3401_asset_beacon_ble) · `weather` (RAK_4631_weather_node)
> The WiFi observer's ESP32 half (`wifiobs`) lives in `agessaman-MeshCore`.

## Getting a prompt

| Method | How |
|---|---|
| **USB serial** | 115200 baud, DTR + RTS asserted; commands must end in **CRLF** (LF-only is echoed with no reply) |
| **Bluetooth** | Connect in the MeshCore companion app, use the node's CLI console |
| **Over the mesh** | Repeater-login to the node with its admin password, then send commands as text |
| **WiFi observer's ESP32** | Prefix with `esp` — the nRF52 relays it to the RAK2305 (e.g. `esp ver`) |

## Build families

- **Common** — every build
- **Cellular** (`cell`) — RAK3401 + BG77 · APL_R02, NM_R01, GC_R01
- **WiFi observer** (`uart` nRF52 + `wifiobs` ESP32) — APL_R01, LC_R01, OSH_R01
- **Weather** (`weather`) — RAK4631 · MESHWERKS_WEATHER
- **Asset beacon** (`asset`) — WD01

---

## Common (all builds)

| Command | Description |
|---|---|
| `ver` | Firmware version + build date; nRF52 builds also show `reboots=N wdt=armed/off` |
| `reboot` / `clkreboot` | Restart the node (`clkreboot` also re-syncs the clock) |
| `set wdt <on\|off>` (or `set wdt` to query) | Hardware watchdog — OFF by default, **effect after reboot**; auto-resets a hung node. On the WiFi observer this is the **nRF52** WDT (use `esp set wdt` for the ESP). `uart.2`/`cell`/`wifiobs.3`+ |
| `wdt test` | **Bench only:** deliberately hang the loop to prove the watchdog resets the node |
| `advert` | Flood advert (announce to the whole mesh) |
| `advert.zerohop` | Advert to direct neighbors only |
| `get name` / `set name <name>` | Read / set the node name (`MESHWERKS_…`) |
| `get radio` / `set radio <freq,bw,sf,cr>` | LoRa params, e.g. `910.525,62.5,7,5` |
| `set tx <dBm>` | LoRa transmit power |
| `set lat <deg>` / `set lon <deg>` | Manual node location for the map pin |
| `set repeat <on\|off>` | Enable/disable packet repeating |
| `password <pw>` / `set guest.password <pw>` | Admin / guest password for remote login |
| `get public.key` | Node public key (identity) |
| `get prv.key` / `set prv.key <hex>` | Back up / restore the full identity key (then reboot) |
| `neighbors` / `neighbor.remove <n>` | List heard neighbors / drop one |
| `clock` / `clock sync` | Show / re-sync the RTC |
| `set timezone <tz>` / `set timezone.offset <h>` | Timezone string / fallback offset |
| `stats-core` / `stats-radio` / `stats-packets` | Runtime, radio, packet counters |
| `log start` / `log stop` / `log erase` | Packet logging to flash |
| `set advert.interval <min>` / `set flood.advert.interval <min>` | Advert cadence |
| `set dutycycle <pct>` / `set txdelay <n>` / `set rxdelay <n>` | Airtime + timing tuning |
| `powersaving <on\|off>` | CPU/radio power-saving |
| `gps <on\|off>` / `gps setloc` / `gps sync` / `gps advert` | On-board GPS (GPS-equipped nodes) |
| `sensor list` / `sensor get <key>` | Read attached sensors |
| `poweroff` / `shutdown` | Power down |

More advanced radio/routing keys exist (`flood.max`, `agc.reset.interval`, `radio.watchdog`,
`path.hash.mode`, `snmp`, …). Run `get` or `set` with no args on the device to list them.

## Cellular observer (`cell`)

| Command | Description |
|---|---|
| `get cell` | Full config in one line (host, tls, apn, iata, **user**, **pass**, origin, toggles, keepalive, gps) |
| `cell.status` | Live modem state: `modem= reg= csq= net= mqtt= pub= drop= err= gps= rst= cnt=` — main health check. `net=TMO/b12` = carrier + LTE band of the last registration *(cell.14+)*, `pref=` shown when `cell.plmn` is set; `rst`/`cnt` = last reboot reason + count |
| `set cell.server <ip>` | Broker address — use the **IP** (SIM DNS is unreliable) |
| `set cell.port <n>` | Broker port (8883 for mqtts) |
| `set cell.user <user>` | MQTT username (`observer`) |
| `set cell.pass <password>` | MQTT password (shown by `get cell` only as `pass=set/none`) |
| `set cell.iata <code>` | IATA topic segment (`ATW`) — drives the map region |
| `set cell.origin <name>` | MQTT display name (empty = node name) |
| `set cell.apn <apn>` | Cellular APN (`hologram`) |
| `set cell.plmn <PLMN>` / `auto` | *(cell.14+)* Prefer one carrier, with automatic fallback (`AT+COPS=4`). PLMNs: T-Mobile `310260`, AT&T `310410`, Verizon `311480`. `get cell.plmn` shows it. Allowed over the MQTT downlink |
| `set cell.tls <on\|off>` / `set cell.tls.verify <on\|off>` | TLS on/off / validate the cert |
| `set cell.keepalive <10-3600>` | MQTT keepalive seconds (60 keeps it fresh on beacon) |
| `set cell.interval <min>` | Status publish interval |
| `set cell.gps <on\|off>` | GNSS self-location — acquires with a one-shot MQTT pause (won't drop the uplink) |
| `set cell.packets <on\|off>` | Publish heard packets; `cell.rx` / `cell.status` / `cell.tx` gate the others |
| `cell.ping <host>` / `cell.ping` | *(cell.13+)* Queue a data-path ping (IP = data only, name = DNS + data; ≤90 s, re-opens the data link) / show the last result + age. Works over **mesh admin** — the way in when a node is registered but its data/MQTT is down. Use `8.8.8.8`; the broker IP doesn't answer pings |
| `cell.ufs` | *(cell.13+)* Modem file storage: free space + files (e.g. a staged `otaapp.bin`). First run queues, run again to read. Read-only; works over mesh admin |
| `cell.otaget https://ota.meshwerks.org/<path>` | Download `<path>.bin/.dat` into the modem + CRC-verify (ack `id=ota`). Host-locked. *(cell.12+: 300 s download budget)* |
| `reboot celldfu` | Re-verify the staged image, then flash it from the modem (ack `id=ota`, ~90 s hold + ~40 s flash) |
| `at <AT command>` | Raw BG77 modem passthrough, e.g. `at AT+CSQ` — **USB console only** (refused over MQTT since cell.12; not available over mesh) |

**Over the MQTT downlink** (cell.11+ / wifiobs.16+), commands touching secrets (`prv`, `pass`, `pwd`, `psk`, `secret`, `token`, `identity`) or connection redirects (`cell.server/port/user/tls*`; WiFi: `wifi.ssid`, `mqttN.*`) are refused — do those over USB. Only the `fleet` broker user can send commands.

## WiFi observer (`uart` nRF52 + `wifiobs` ESP32)

nRF52 side (flashed by BLE DFU with the `uart` .zip):

| Command | Description |
|---|---|
| `mqtt on` / `mqtt off` | Enable the device-info push to the ESP32 — **must be ON** or identity/name won't sync |
| `mqtt status` / `mqtt cfg` | nRF52↔ESP32 link status / dump the persisted config (debug) |

ESP32 side via the `esp` relay (flashed by web-OTA with the `wifiobs` .bin):

| Command | Description |
|---|---|
| `esp ver` | ESP32 firmware version (should read `mw.wifiobs.<rev>`) |
| `esp start ota` | Launch the ElegantOTA web UI → `Started: http://<ip>/update`; upload the `wifiobs` **.bin** there |
| `esp get mqtt.status` | Broker connection state for all slots (custom / rflab / meshmapper) |
| `esp set wifi.ssid <ssid>` / `esp set wifi.pwd <pass>` | WiFi credentials |
| `esp get wifi.status` | Connection state, IP, RSSI, disconnect reason |
| `esp set wifi.txpower <dBm>` / `esp get wifi.txpower` | **TX power** — 0 = default, 2–20 → nearest step (crank a weak node to `19`). *wifiobs.2+* |
| `esp set wifi.powersave <none\|min\|max>` | WiFi power-save (`none` for best reception) |
| `esp set wdt <on\|off>` (or `esp set wdt`) | Arm/disarm the **ESP32** loop watchdog (separate from the nRF52 `set wdt`; OFF by default, effect after reboot). Reboot reason + count also appear in the MQTT status. *wifiobs.3+* |
| `esp wdt test` | **Bench only:** hang the ESP loop to prove its watchdog bites |
| `esp set mqtt.iata <code>` / `esp set mqtt.origin <name>` | MQTT identity |
| `esp set mqtt1.preset <custom\|rflab\|meshmapper>` | Slot 1 preset (also `mqtt2`, `mqtt3`) |
| `esp set mqtt1.server/username/password <v>` | Custom-slot broker settings |
| `esp set bridge.enabled <on\|off>` | Turn the MQTT bridge on — do this at deploy |
| `esp ota check` / `esp ota update` | Pull-OTA from a manifest, if configured (separate from `start ota`) |
| `esp ota branch [prod\|beta\|default]` | *(wifiobs.17+, from 1.18)* Show / pick the OTA release channel. On fleet builds every channel resolves to `ota.meshwerks.org`, so it changes nothing today |

## Weather node (`weather`)

| Command | Description |
|---|---|
| `nws status` | Node status: last poll, active alerts, zones, proxy |
| `nws poll` | Force an NWS poll now |
| `nws test` | Broadcast a test alert (tagged `[TEST]`) |
| `nws alerts` | List active alerts |
| `nws zone <WIZxxx>` | Add / set a forecast zone (e.g. `WIZ038`) |
| `nws hashtag <#tag>` / `nws channel` | Mesh alert channel hashtag / show it |
| `nws severity <level>` | Minimum severity that triggers a broadcast |
| `nws proxy <ip:port>` | NWS proxy address (literal IP on the shop net) |
| `nws utcoffset <h>` | UTC offset for alert timestamps |
| `nws announce` / `nws clear` / `nws ping` | Manual announce / clear alerts / connectivity ping |
| `uptime <on\|off\|status\|test>` | Periodic uptime reporting |
| `set display timeout <s>` | OLED blank timeout |

## Ethernet companion (`companion-eth`) — KE9NEN-GW01

| Command | Description |
|---|---|
| `start ota` | *(companion-eth.1+)* Local links only (USB, BLE, or the TCP companion CLI frame over Ethernet; refused from mesh peers). Reboots into a one-shot BLE OTA mode advertising `RAK4631_OTA` for 10 min (Ethernet/USB stay up), then back to normal. Flash from the desktop with `fleet-control/nrf-ota/ble_dfu.py <MAC> firmware.zip` |
| `ver` / `get name` | Over the companion CLI frame (CMD 66) the full version string is returned (the device-info frame truncates it to 20 chars) |

## Asset beacon (`asset`)

| Command | Description |
|---|---|
| `set ab <1\|0>` | Enable / disable the asset-beacon engine (movement-gated GPS push) |
| `set ab_move <n>` | Movement threshold that triggers a beacon |
| `set ab_flood <sec>` | Flood-advert interval while moving, e.g. `set ab_flood 60` |
| `set ab_idle <sec>` | Advert interval while stationary |
| `set ab_zh <sec>` | Zero-hop advert interval, e.g. `set ab_zh 20` |

The asset beacon is a companion-app build, so it also takes the full **Common** set over USB/BLE.

---

## Reading a version

`ver` → `v1.18.0-mw.cell.15+g23ad2cf`

| Part | Meaning |
|---|---|
| `v1.18.0` | Upstream MeshCore base |
| `mw` | Fleet (meshwerks) |
| `cell` | Which firmware |
| `.1` | Your revision (bump per change) |
| `+g744b985` | Exact commit it was built from |
| `.dirty` | Built with uncommitted changes |

Per-firmware rev history lives in [`VERSIONS.md`](VERSIONS.md). **Keep this cheat sheet in sync
whenever a CLI command is added, renamed, or removed.**
