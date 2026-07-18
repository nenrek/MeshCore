# Custom firmware versions

Every custom build embeds a fleet version string reported by the `ver` CLI command,
the companion app's version field, and the MQTT `firmware_version` status field (so it's
visible per-node on the beacon/CoreScope map too).

## Scheme

```
v<upstream>-<fleet>.<variant>.<rev>+g<githash>[.dirty]
        │        │        │      │      │        └─ uncommitted changes at build time
        │        │        │      │      └─ short commit the build came from (auto)
        │        │        │      └─ our revision — bump on each meaningful change
        │        │        └─ which firmware: cell | wifiobs | wroom
        │        └─ fleet tag (meshwerks)
        └─ upstream MeshCore base (bump when we rebase onto a new upstream release)
```

Example: `v1.16.0-mw.cell.3+g1a2b3c` — meshwerks cellular observer, our rev 3, built from
commit `1a2b3c` on upstream MeshCore v1.16.0.

## How it works

- The rev base lives per-env as `custom_fw_version` in the env's `platformio.ini`.
- `fw_version.py` (pre-build script) appends `+g<githash>` (and `.dirty` if the tree has
  uncommitted changes) and defines `FIRMWARE_VERSION`, overriding the `#ifndef` fallback in
  the example headers. Works on a plain `pio run` — no `build.sh` needed.
- **To cut a new rev:** bump the `.rev` in that env's `custom_fw_version`, add a row below,
  commit, build, flash.

## Registry — branch `cellular-mqtt-uplink`

### cell — `RAK_3401_cellular_observer` (nRF52 RAK3401 + RAK5860/BG77, LTE-M)
| rev | date | notes |
|-----|------|-------|
| 1 | 2026-07-15 | First versioned build. Non-blocking modem bring-up, connect-by-IP + HostSNI(*), QMTPUB 4 KB, RX packet-feed fix, weak-signal connect timeouts, GPS one-shot MQTT pause, `get cell` shows user/origin/pass-presence. |

### wifiobs — `RAK2305_uplink_observer` (ESP32-WROVER, PSRAM, 5 broker slots)
| rev | date | notes |
|-----|------|-------|
| 1 | 2026-07-15 | First versioned build. Native multi-broker (meshwerks + rflab + meshmapper JWT), MCSTA stats push from nRF52, DHCP hostname, load-shed hook. |

### wroom — `WROOM_uplink_observer_TEMP` (bare ESP-WROOM-32, no PSRAM, 2 slots, OSH stopgap)
| rev | date | notes |
|-----|------|-------|
| 1 | 2026-07-15 | First versioned build. 2-slot no-PSRAM stopgap for a dead RAK2305. |
