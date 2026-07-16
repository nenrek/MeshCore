#include <Arduino.h>
#include <Mesh.h>

#include "NWSClient.h"
#include "MyMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);
NWSClient nws_client;
static char command[160];

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

  // STEP 5: Ethernet Setup (SPI1)
  if (!nws_client.begin()) {
    Serial.println("[SETUP] Ethernet Failed.");
  } else {
    // DIAGNOSTIC: Verify local network path
    nws_client.testGateway();
    delay(2000);
  }

  // STEP 6: Start Node
  sensors.begin();
  the_mesh.setNWSClient(&nws_client);
  the_mesh.begin(fs);
  the_mesh.loadNWSPrefs(fs);

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
  nws_client.maintain();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
}