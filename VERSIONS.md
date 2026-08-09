# Custom firmware versions

Every custom build embeds a fleet version string reported by the `ver` CLI command,
the companion app's version field, and (where applicable) the MQTT status feed.

## Scheme

```
v<upstream>-<fleet>.<variant>.<rev>+g<githash>[.dirty]
        │        │        │      │      │        └─ uncommitted changes at build time
        │        │        │      │      └─ short commit the build came from (auto)
        │        │        │      └─ our revision — bump on each meaningful change
        │        │        └─ which firmware: uart | asset | weather
        │        └─ fleet tag (meshwerks)
        └─ upstream MeshCore base (bump when we rebase onto a new upstream release)
```

Example: `v1.17.0-mw.weather.1+g1a2b3c`.

## Upstream base

| base | date | notes |
|------|------|-------|
| v1.16.0 | 2026-07-15 | Initial versioned fleet. |
| **v1.17.0** | **2026-08-09** | Rebased all custom firmwares onto `repeater-v1.17.0`. Adapted to three 1.17 API changes (UITask `(board,display)` ctor, MeshTables `wasSeen`/`markSeen` split, `UIColor` semantic palette). Companion prefs now JSON `ConfigSerializer` (`/prefs.json`) — asset-beacon `ab_*` fields ported to an `AssetBeaconPrefs` group; legacy binary `/new_prefs` still migrated on first boot. Upstream added official RAK13800 Ethernet (SPI1 pins 29/30/3 — same split the weather node uses). |

## How it works

- The rev base lives per-env as `custom_fw_version` in the env's `platformio.ini`.
- `fw_version.py` (pre-build script) appends `+g<githash>` (`.dirty` if the tree has
  uncommitted changes) and defines `FIRMWARE_VERSION`, overriding the `#ifndef` fallback in
  the example headers. Works on a plain `pio run`.
- **To cut a new rev:** bump the `.rev` in that env's `custom_fw_version`, add a row, commit.

## Registry — branch `uart-uplink`

### uart — `RAK_3401_uart_uplink` (nRF52 RAK3401; feeds a native-flex RAK2305 over UART)
| rev | date | notes |
|-----|------|-------|
| 1 | 2026-07-15 | First versioned build. UART_UPLINK packet forwarding (MCPKT/MCSTA framing), load-shed, runtime LV cutoff. |
| 2 | 2026-07-19 | Reliability: flag-gated nRF52 hardware watchdog (`set wdt on`, OFF by default, ~120s CRV, armed after bring-up) + flash-persisted `reboot_count` (survives the Adafruit bootloader that clears RESETREAS). `wdt test` bench hook; `ver` now reports `reboots=N wdt=armed/off`. Cellular-parity port (from cell `6763fef0`/`348b56cf`). NOT yet hardware-validated. |

### asset — `RAK_3401_asset_beacon_ble` (nRF52 RAK3401; anti-theft / asset locator)
| rev | date | notes |
|-----|------|-------|
| 1 | 2026-07-15 | First versioned build. `-D AB` movement-gated periodic self-advert GPS push beacon. |

### weather — `RAK_4631_weather_node` (nRF52 RAK4631 + W5100S ethernet)
| rev | date | notes |
|-----|------|-------|
| 1 | 2026-07-15 | First versioned build. NWS-proxy severe-alert polling → mesh broadcast, hashtag channel, ATW zones. (Replaces the old ad-hoc `v1.0.0-nws` version label.) |
