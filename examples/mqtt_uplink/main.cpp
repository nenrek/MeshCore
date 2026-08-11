// RAK2305 (ESP32) MQTT uplink observer — main.
// Identical to examples/simple_repeater/main.cpp except:
//   (a) it reuses simple_repeater's MyMesh (via -I examples/simple_repeater), and
//   (b) loop() relays CLI commands that arrive from the companion nRF52 over the
//       UART link (UartRadio) to our MeshCore CLI, sending the reply back — so
//       this USB-less ESP32 is fully configurable from the nRF52's MeshCore CLI
//       (type `esp <cmd>` there).
#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"
#if defined(ESP32)
  #include "esp_task_wdt.h"   // flag-gated loop watchdog (`set wdt on`)
#endif

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

unsigned long POWERSAVING_FIRSTSLEEP_SECS = 120;

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#if defined(MESH_DEBUG) && defined(NRF52_PLATFORM)
  delay(5000);
#endif

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.setCursor(0, 0);
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) {   // for us: starts Serial1 (UART link) — see variants/rak2305/target.cpp
    MESH_DEBUG_PRINTLN("Radio init failed!");
    halt();
  }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // StdRNG/esp_random — no radio entropy
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Firmware: "); Serial.print(FIRMWARE_VERSION);
  Serial.print(" (built "); Serial.print(FIRMWARE_BUILD_DATE); Serial.println(")");

  Serial.print("Observer ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  // NOTE: our old ESP loop-watchdog (`set wdt on`) was dropped on the 1.17 rebase —
  // agessaman's observer-firmware provides its own `radio.watchdog` + ExternalWatchdogManager.

  board.onBootComplete();
}

void loop() {
  // Local console (UART0 / HW-417 on the bench) — still works for direct config.
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
      Serial.print(c);
    }
    if (c == '\r') break;
  }
  if (len == sizeof(command)-1) {
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {
    Serial.print('\n');
    command[len - 1] = 0;
    char reply[160];
    the_mesh.handleCommand(0, command, reply);
    if (reply[0]) {
      Serial.print("  -> "); Serial.println(reply);
    }
    command[0] = 0;
  }

  the_mesh.loop();

  // CLI relay: a config command arrived from the nRF52 over the UART link
  // (`esp <cmd>` typed on the nRF52). Run it on our CLI and send the reply back
  // so it surfaces on the nRF52's USB console. This is how the USB-less ESP32 is
  // managed in normal (assembled) operation.
  if (radio_driver.hasHostCommand()) {
    char hcmd[160];
    radio_driver.takeHostCommand(hcmd, sizeof(hcmd));
    char hreply[160]; hreply[0] = 0;
    the_mesh.handleCommand(0, hcmd, hreply);
    radio_driver.sendHostReply(hreply[0] ? hreply : "OK");
  }

  // Radio config pushed from the nRF52 (it owns the radio). Drop it into the
  // prefs the bridge reads, so /status reports the real freq/bw/sf/cr instead of
  // this firmware's region default. In-memory only — refreshed periodically.
  if (radio_driver.hasRadioConfig()) {
    float f, b; uint8_t s, c;
    radio_driver.takeRadioConfig(f, b, s, c);
    NodePrefs* pr = the_mesh.getNodePrefs();
    if (pr) { pr->freq = f; pr->bw = b; pr->sf = s; pr->cr = c; }
  }

  // ONE DEVICE: adopt the nRF52's identity so the ESP32 publishes AS that node
  // (same observer key on all maps). Only re-keys on an actual change, then
  // persists and restarts the bridge so the topic + JWT use the adopted key.
  if (radio_driver.hasIdentity()) {
    uint8_t prv[PRV_KEY_SIZE];
    radio_driver.takeIdentity(prv);
    mesh::LocalIdentity nid;
    nid.readFrom(prv, PRV_KEY_SIZE);
    if (memcmp(nid.pub_key, the_mesh.self_id.pub_key, PUB_KEY_SIZE) != 0) {
      the_mesh.self_id = nid;
      IdentityStore st(SPIFFS, "/identity");
      st.save("_main", the_mesh.self_id);
      the_mesh.restartBridge();   // re-init device_id + JWT with the adopted identity
    }
  }

  // Node name from the nRF52 -> node_name (used in the ADVERT the maps display).
  // The /status origin field is derived by their bridge via getEffectiveMqttOrigin(),
  // which falls back to node_name when mqtt_origin (now in MQTTPrefs) is empty — so
  // setting node_name alone shows the observer as the node's name everywhere.
  if (radio_driver.hasName()) {
    char nm[32]; radio_driver.takeName(nm, sizeof(nm));
    NodePrefs* pr = the_mesh.getNodePrefs();
    if (pr) {
      strncpy(pr->node_name,   nm, sizeof(pr->node_name)   - 1); pr->node_name[sizeof(pr->node_name)   - 1] = 0;
    }
  }

  // Battery (mV) from the node -> reported in /status.
  if (radio_driver.hasBatt()) {
    board.setBattMilliVolts(radio_driver.takeBatt());
  }

  // TODO(1.17): companion-stats override — feed the nRF52's real radio/mesh stats
  // (MCSTA: airtime/queue/uptime/errors/packet-counts) into the bridge /status so a
  // radio-less ESP32 reports the REAL values, not its own blanks. agessaman's
  // MQTTBridge has no external-stats injection yet (MQTTExternalStats /
  // setBridgeExternalStats were ours). Port that hook to their bridge, then re-enable.
  // Until then /status radio stats read blank on the 2-chip observer.
  if (radio_driver.hasStats()) {
    uint32_t tx_air, rx_air, queue, uptime, pkts_sent, pkts_recv; uint16_t errflags;
    radio_driver.takeStats(tx_air, rx_air, queue, uptime, errflags, pkts_sent, pkts_recv);
    (void)tx_air; (void)rx_air; (void)queue; (void)uptime; (void)errflags; (void)pkts_sent; (void)pkts_recv;
  }

  // Push NTP-synced UTC time DOWN to the nRF52 (it has no RTC) so its adverts and
  // packet timestamps are correct. We're the only internet-connected half — one
  // device. Time stays UTC; DST is a display concern handled by the maps.
  {
    static unsigned long next_time_push = 0;
    if ((long)(millis() - next_time_push) >= 0) {
      time_t nowt = time(nullptr);
      if (nowt > 1735689600) {            // SNTP has synced (after 2025-01-01)
        Serial1.printf("MCTIME %lu\r\n", (unsigned long)nowt);
        next_time_push = millis() + 60000UL;
      } else {
        next_time_push = millis() + 5000UL;   // not synced yet — retry soon
      }
    }
  }

  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();

  // No powersaving: WiFi + MQTT + the UART link need the CPU serviced continuously.
}
