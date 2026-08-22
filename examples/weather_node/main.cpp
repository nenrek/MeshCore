#include <Arduino.h>
#include <Mesh.h>

#include "NWSClient.h"
#include "MyMesh.h"

// Ethernet bring-up + link management via the upstream RAK13800 interface (PoE-stable).
// NWSClient just layers HTTPS on the shared global Ethernet this sets up.
#ifdef ETHERNET_ENABLED
  #include <helpers/ethernet/EthernetInterface.h>
  static ETHERNET_CLASS ethernet_interface;
#endif

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);
NWSClient nws_client;
static char command[160];
#ifdef ETHERNET_ENABLED
static char eth_command[160];   // line buffer for the network (TCP:5000) CLI
#endif

void halt() { while (1); }

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n--- FOX VALLEY NWS ALERT NODE BOOT ---");

  // STEP 1: Filesystem Initialization FIRST
  FILESYSTEM* fs = nullptr;
  #if defined(NRF52_PLATFORM)
    InternalFS.begin();
    fs = &InternalFS;
    
    // WORKAROUND: Explicitly disable the QSPI peripheral.
    // Releases pins 3, 26, 29, 30 for Ethernet SPI1.
    NRF_QSPI->ENABLE = 0; 
  #endif

  IdentityStore store(*fs, "");

  // STEP 2: Identity Loading
  if (!store.load("_main", the_mesh.self_id)) {
    Serial.println("[SETUP] Generating new identity...");
    the_mesh.self_id = radio_new_identity();
    store.save("_main", the_mesh.self_id);
  }

  // STEP 3: Power and Pins
  board.begin();
  pinMode(34, OUTPUT);
  digitalWrite(34, HIGH); // Enable 3.3V power for RAK13800
  delay(500);

  pinMode(ETH_CS_PIN, OUTPUT);
  digitalWrite(ETH_CS_PIN, HIGH); // Ensure Ethernet is deselected initially.

  // STEP 4: Radio Initialization (SPI0)
  if (!radio_init()) {
    Serial.println("[SETUP] Radio failed!");
    halt();
  }
  Serial.println("[SETUP] Radio OK");
  fast_rng.begin(radio_driver.getRngSeed());

  // STEP 5: Ethernet Setup (SPI1) — upstream RAK13800 interface. begin() always returns true so
  // the node boots even with no cable; the link + DHCP come up (and retry) in its loop(). This
  // is the PoE-stable path: reset is never toggled and the chip is never re-inited on retry, so
  // the PHY stays powered (keeps the 802.3af power signature alive).
#ifdef ETHERNET_ENABLED
  ethernet_interface.begin();
#endif

  // STEP 6: Start Node
  sensors.begin();
  the_mesh.setNWSClient(&nws_client);
  the_mesh.begin(fs);
  the_mesh.loadNWSPrefs(fs);

  // TLS cert-date validation needs a real clock. No NTP here and mesh time-sync may be absent,
  // so floor the RTC to a recent build-time epoch if it looks unset/stale. (Once the node gets
  // real time from the mesh, that's newer and wins.)
  if (the_mesh.getRTCClock()->getCurrentTime() < 1787000000UL) {   // < ~2026-08-17
    the_mesh.getRTCClock()->setCurrentTime(1787270400UL);          // ~2026-08-20
    Serial.println("[TIME] RTC floored to build-time epoch for TLS cert validation");
  }

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    Serial.println("[DISPLAY] SSD1306 initialized OK");
  } else {
    Serial.println("[DISPLAY] SSD1306 init FAILED — check I2C wiring and address");
  }
  ui_task.begin(the_mesh.getNodePrefs(), the_mesh.getDisplayData(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  Serial.println("[SETUP] Fox Valley NWS Node ready.");
  command[0] = 0;
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    int len = strlen(command);
    
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        char reply[160];
        reply[0] = 0;
        the_mesh.handleCommand(0, command, reply);
        if (reply[0]) {
          Serial.print("  -> ");
          Serial.println(reply);
        }
        command[0] = 0;
      }
    } else if (len < sizeof(command) - 1) {
      command[len] = c;
      command[len + 1] = 0;
    }
  }

  the_mesh.loop();
  sensors.loop();
  rtc_clock.tick();
#ifdef ETHERNET_ENABLED
  ethernet_interface.loop();   // link management: brings DHCP up, retries, Ethernet.maintain()

  // Network CLI: a plain-text admin channel over TCP:5000 (connect with `nc <ip> 5000` or a
  // telnet client and type the same commands as the USB serial CLI: ver, get radio, nws poll...).
  // The interface's loop() above accepts the client; we read raw bytes and dispatch to handleCommand.
  if (ethernet_interface.isConnected()) {
    while (ethernet_interface.available()) {
      char c = (char)ethernet_interface.read();
      int len = strlen(eth_command);
      if (c == '\n' || c == '\r') {
        if (len > 0) {
          char reply[160];
          reply[0] = 0;
          the_mesh.handleCommand(0, eth_command, reply);
          if (reply[0]) {
            const char* pre = "  -> ";
            ethernet_interface.write((const uint8_t*)pre, 5);
            ethernet_interface.write((const uint8_t*)reply, strlen(reply));
            ethernet_interface.write((const uint8_t*)"\r\n", 2);
          }
          eth_command[0] = 0;
        }
      } else if (len < (int)sizeof(eth_command) - 1) {
        eth_command[len] = c;
        eth_command[len + 1] = 0;
      }
    }
  }
#endif
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
}