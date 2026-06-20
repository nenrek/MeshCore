#include <Arduino.h>   // needed for PlatformIO
#include <Mesh.h>

#include "MyMesh.h"            // standard repeater behaviour (examples/simple_repeater)
#include "EspAtMqtt.h"         // RAK2305 ESP-AT WiFi/MQTT bridge (UART1)
#include "MqttMeshTables.h"    // per-packet publish hook

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

StdRNG fast_rng;

// MqttMeshTables replaces the stock SimpleMeshTables so each unique received
// packet is fanned out to MQTT exactly once.
MqttMeshTables tables;
EspAtMqtt      mqtt;

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

  if (!radio_init()) {
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
    the_mesh.self_id = radio_new_identity();
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Repeater ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

  // --- RAK2305 ESP-AT bridge bring-up ---
  // Power the IO-slot 3V3_S rail (also feeds the RAK13302 FEM). Safe to re-assert.
#ifdef PIN_3V3_EN
  pinMode(PIN_3V3_EN, OUTPUT);
  digitalWrite(PIN_3V3_EN, HIGH);
#endif
  // UART1 talks to the RAK2305. NOTE: this is the same UART the optional
  // serial GPS would use — do not enable GPS.
  Serial1.begin(MQTT_AT_BAUD);
  Serial.printf("[MQTT] UART1 to RAK2305: RX=%d TX=%d baud=%d\n",
                (int)PIN_SERIAL1_RX, (int)PIN_SERIAL1_TX, (int)MQTT_AT_BAUD);
  mqtt.setStream(&Serial1);
  mqtt.setRTC(&rtc_clock);
  mqtt.setNodeName(the_mesh.getNodeName());
  mqtt.setPubKey(the_mesh.self_id.pub_key, PUB_KEY_SIZE);
  mqtt.load(fs);

  tables.setBridge(&mqtt);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  board.onBootComplete();
}

void loop() {
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
    reply[0] = 0;
    // Intercept wifi/mqtt commands before handing off to the mesh CLI.
    if (strcmp(command, "uart scan") == 0) {
      // Empirically find which GPIOs reach the base-board header/slot UART nets.
      // With a jumper across the header TXD1<->RXD1, the connected pair reports.
      // Without a jumper, 'uart idle' (below) finds the ESP's idle-high TX line.
      // Candidates exclude everything the variant claims: radio SPI/CTL
      // (3,4,9,10,26,29,30), FEM (21), I2C1/display (13,14), LEDs (35,36),
      // 3V3_EN (34), battery (5), AREF (2), and the 32k crystal (0,1).
      static const uint8_t cands[] = {6,7,8,11,12,15,16,18,19,20,22,23,24,25,
                                      27,33,37,38,39,40,41,42,46,47};
      const int N = sizeof(cands);
      Serial.println("  -> scanning pin pairs (expects jumper on TXD1<->RXD1)...");
      Serial1.end();
      int found = 0;
      for (int t = 0; t < N; t++) {
        pinMode(cands[t], OUTPUT);
        for (int r = 0; r < N; r++) {
          if (r == t) continue;
          pinMode(cands[r], INPUT_PULLDOWN);
          bool match = true;
          for (int k = 0; k < 8 && match; k++) {
            digitalWrite(cands[t], k & 1);
            delayMicroseconds(50);
            if (digitalRead(cands[r]) != (k & 1)) match = false;
          }
          digitalWrite(cands[t], LOW);
          pinMode(cands[r], INPUT);
          if (match) {
            uint8_t T = cands[t], R = cands[r];
            Serial.printf("  -> CONNECTED: drive %d (P%d.%02d) -> sense %d (P%d.%02d)\n",
                          T, T / 32, T % 32, R, R / 32, R % 32);
            found++;
          }
        }
        pinMode(cands[t], INPUT);
      }
      if (!found) Serial.println("  -> no connected pair found");
      Serial1.begin(MQTT_AT_BAUD);
    } else if (strcmp(command, "uart idle") == 0) {
      // With NO jumper and the RAK2305 installed: its UART TX idles high, so a
      // candidate that reads HIGH against a pulldown is externally driven —
      // that net is our RX (core RXD <- ESP TXD).
      static const uint8_t cands[] = {6,7,8,11,12,15,16,18,19,20,22,23,24,25,
                                      27,33,37,38,39,40,41,42,46,47};
      const int N = sizeof(cands);
      Serial1.end();
      Serial.println("  -> pins reading HIGH against pulldown (externally driven):");
      int found = 0;
      for (int i = 0; i < N; i++) {
        pinMode(cands[i], INPUT_PULLDOWN);
        delayMicroseconds(100);
        int hi = 0;
        for (int k = 0; k < 5; k++) { hi += digitalRead(cands[i]); delayMicroseconds(50); }
        pinMode(cands[i], INPUT);
        if (hi == 5) {
          uint8_t P = cands[i];
          Serial.printf("  ->   %d (P%d.%02d)\n", P, P / 32, P % 32);
          found++;
        }
      }
      if (!found) Serial.println("  ->   none");
      Serial1.begin(MQTT_AT_BAUD);
    } else if (strcmp(command, "uart probe") == 0) {
      // RX fixed on P0.15 (the externally-driven-high net = ESP-AT TX idle).
      // For each candidate TX pin, send "AT\r\n" at 115200 using UARTE1 directly
      // (registers, polled — Serial1/UARTE0 untouched) and count reply bytes.
      // The candidate that draws a reply is the real TXD1 net.
      static const uint8_t txc[] = {16,17,19,20,2,6,7,8,11,12,22,23,25,27,28,
                                    31,32,33,37,38,39,40,41,42,46,47};
      static uint8_t rxbuf[64];
      static char txb[8];
      strcpy(txb, "AT\r\n");                  // EasyDMA needs a RAM source
      Serial.println("  -> probing TX candidates against RX=P0.15 @115200...");
      pinMode(15, INPUT);
      int hits = 0;
      for (unsigned i = 0; i < sizeof(txc); i++) {
        uint8_t tx = txc[i];
        pinMode(tx, OUTPUT);
        digitalWrite(tx, HIGH);               // UART idle level
        delay(5);
        NRF_UARTE1->ENABLE = 0;
        NRF_UARTE1->PSEL.TXD = tx;            // logical pin == PSEL encoding here
        NRF_UARTE1->PSEL.RXD = 15;
        NRF_UARTE1->PSEL.RTS = 0xFFFFFFFF;
        NRF_UARTE1->PSEL.CTS = 0xFFFFFFFF;
        NRF_UARTE1->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud115200;
        NRF_UARTE1->CONFIG = 0;               // 8N1, no flow control
        NRF_UARTE1->EVENTS_ENDTX = 0;
        NRF_UARTE1->EVENTS_ENDRX = 0;
        NRF_UARTE1->EVENTS_RXTO = 0;
        NRF_UARTE1->ENABLE = 8;
        NRF_UARTE1->RXD.PTR = (uint32_t)rxbuf;
        NRF_UARTE1->RXD.MAXCNT = sizeof(rxbuf);
        NRF_UARTE1->TASKS_STARTRX = 1;
        NRF_UARTE1->TXD.PTR = (uint32_t)txb;
        NRF_UARTE1->TXD.MAXCNT = 4;
        NRF_UARTE1->TASKS_STARTTX = 1;
        unsigned long t0 = millis();
        while (!NRF_UARTE1->EVENTS_ENDTX && millis() - t0 < 20) ;
        delay(150);                           // reply window
        NRF_UARTE1->TASKS_STOPRX = 1;
        t0 = millis();
        while (!NRF_UARTE1->EVENTS_RXTO && !NRF_UARTE1->EVENTS_ENDRX
               && millis() - t0 < 20) ;
        uint32_t n = NRF_UARTE1->RXD.AMOUNT;
        NRF_UARTE1->ENABLE = 0;
        pinMode(tx, INPUT);
        if (n > 0) {
          Serial.printf("  -> TX=%u (P%u.%02u): %lu bytes: ", tx, tx / 32, tx % 32,
                        (unsigned long)n);
          for (uint32_t k = 0; k < n && k < 24; k++) {
            char c = (char)rxbuf[k];
            Serial.write((c >= 32 && c < 127) ? c : '.');
          }
          Serial.println();
          hits++;
        }
      }
      NRF_UARTE1->PSEL.TXD = 0xFFFFFFFF;
      NRF_UARTE1->PSEL.RXD = 0xFFFFFFFF;
      if (!hits) Serial.println("  -> no reply on any TX candidate");
    } else if (strncmp(command, "mqtt baud ", 10) == 0) {  // debug: UART1 baud sweep, not persisted
      uint32_t b = (uint32_t)atoi(command + 10);
      if (b >= 300 && b <= 921600) {
        Serial1.end();
        Serial1.begin(b);
        Serial.printf("  -> UART1 at %lu baud (until reboot)\n", (unsigned long)b);
      } else {
        Serial.println("  -> bad baud");
      }
    } else if (mqtt.handleCommand(command, reply)) {
      if (reply[0]) { Serial.print("  -> "); Serial.println(reply); }
    } else {
      the_mesh.handleCommand(0, command, reply);
      if (reply[0]) { Serial.print("  -> "); Serial.println(reply); }
    }
    command[0] = 0;
  }

  the_mesh.loop();
  mqtt.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();

  // NOTE: powersaving/sleep is intentionally omitted — the MQTT bridge needs the
  // CPU and UART continuously serviced, so this build stays awake.
}
