# Flashing a RAK2305 with the MQTT ESP-AT firmware

The RAK2305 (ESP32-WROVER) ships with RAK's factory **WIFI-BLE** ESP-AT image,
which has **no MQTT command set** — `AT+MQTTUSERCFG` returns `ERROR`. Before it can
be used as the WiFi/MQTT bridge for the `mqtt_repeater` firmware, it must be
reflashed with RAK's **WIFI-HTTP-MQTT** ESP-AT build.

This is a one-time, per-module provisioning step done over the ESP32's UART0
download pins with a USB-serial adapter — the WisConnector only carries UART1
(the AT interface), not the download lines.

> Verified end-to-end on the first unit (commits `3177b867`…). The whole observer
> then reaches `[online]` and publishes decoded packets to Beacon.

---

## What you need

- **RAK2305** module (the one to flash).
- **3.3 V USB-to-serial adapter** — e.g. an **HW-417 (CP2102)**. Must be 3.3 V
  logic; the ESP32 is not 5 V tolerant.
- A way to connect to the 2305's broken-out **download holes** (the row separate
  from the board-to-board WisConnector). Solder a 2.54 mm header into them, or tack
  wires directly. Don't rely on a press-fit for a 1–2 min flash — it drops sync.
- The firmware image: **`RAK2305-Basic-WIFI-HTTP-MQTT-AT.bin`** (a 4 MB full-flash
  ESP32 image). Already downloaded locally at `C:\Users\Josh\projects\RAK2305-MQTT.bin`.
  To re-fetch:
  ```bash
  curl -L -o RAK2305-MQTT.bin \
    https://github.com/RAKWireless/WisBlock/raw/master/bootloader/RAK2305/RAK2305-Basic-WIFI-HTTP-MQTT-AT.bin
  ```
- **esptool** (already installed in PlatformIO's Python):
  ```bash
  ~/.platformio/penv/Scripts/python.exe -m esptool version   # expect v5.x
  ```

---

## Wiring (standalone, off the base board)

Power the module from the adapter's **3V3** pin (never 5 V). Tie **IO0 → GND** to
force download mode.

| HW-417 pin | → RAK2305 pad | purpose |
|---|---|---|
| 3V3 | 3V3 | power (flashing only — see note) |
| GND | GND | ground |
| TXD | RXD0 (IO3) | data (crossed) |
| RXD | TXD0 (IO1) | data (crossed) |
| GND | IO0 (BOOT) | **strap low = download mode** |

**Power note:** the adapter's 3V3 is fine for *flashing* (WiFi is off, low draw),
but **not** for *running* the AT firmware — the ESP32's WiFi current spikes will
brown it out (you'll see it loop `ready…` and never respond to `AT`). So validate
on the base board, not standalone.

Because IO0 is tied low, the module boots straight into download mode the moment
it's powered — **no DTR/RTS auto-reset is needed** (the HW-417 doesn't have it).

---

## Flash

1. Find the adapter's COM port (Device Manager, or it's the only new USB-serial).
   Call it `COM_N`. If it doesn't appear, install the Silabs CP210x VCP driver.

2. **Confirm the connection (read-only)** before writing — this proves the port,
   wiring, and download-mode strap all at once:
   ```bash
   ~/.platformio/penv/Scripts/python.exe -m esptool --port COM_N --baud 115200 flash-id
   ```
   Expect: `Detecting chip type... ESP32`, `ESP32-D0WD-V3`, `Detected flash size: 4MB`.
   If it hangs at `Connecting…`, TX/RX are swapped or IO0 isn't actually low.

3. **Write the image** (full-flash image → offset `0x0`):
   ```bash
   cd /c/Users/Josh/projects
   ~/.platformio/penv/Scripts/python.exe -m esptool --port COM_N --baud 115200 \
     write-flash 0x0 RAK2305-MQTT.bin
   ```
   Takes ~70 s (the 4 MB image compresses to ~677 KB). Ends with
   `Hash of data verified.` Bump `--baud 460800` on later runs once a 115200
   connect is confirmed good.

---

## Verify

Quick optional check while still on the adapter (it'll brown-out-loop on WiFi,
but `AT+GMR` answers before that): open a 115200 terminal on `COM_N` and send
`AT+GMR` — the build string should now mention MQTT, not the old `WIFI-BLE`.

The real verification is on the base board:

1. Remove the **IO0 → GND** strap, unplug the adapter.
2. Seat the 2305 on the WisBlock base board (Slot B), powered by battery/USB.
3. On the node console (USB serial, 115200), bring MQTT up — config persists in
   `/mqtt.cfg`, so a previously-configured node just needs:
   ```
   mqtt retry
   mqtt status     # expect: [online], ok climbing
   ```
   For a fresh node, set credentials first (see `FIRMWARE.md`): `wifi ssid/pass`,
   `mqtt host/port/scheme/user/mpass/iata`, then `mqtt on`.

`AT+MQTTUSERCFG` returning `OK` (instead of `ERROR`) is the proof the MQTT image
took. From there the bring-up runs to `[online]` and packets flow to the broker.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| esptool hangs at `Connecting…` | TX/RX swapped, or IO0 not low. Re-check wiring; ensure IO0→GND held through power-up. |
| `ready` loops, no `AT` reply when running | Power brown-out on the adapter's 3V3 — normal standalone; verify on the base board instead. |
| `AT+MQTTUSERCFG` still `ERROR` after flash | Wrong/old bin. Confirm you flashed `…WIFI-HTTP-MQTT-AT.bin`, not the factory WIFI-BLE image. |
| No COM port for the adapter | Install Silabs CP210x VCP driver (CP2102) or the FTDI driver, per adapter chip. |
