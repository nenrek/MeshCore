#include "target.h"
#include <Arduino.h>
#include <esp_system.h>
#include <helpers/ArduinoHelpers.h>   // StdRNG

// UART link to the companion nRF52 (RAK3401), via the RAK2305 IO-Slot UART1.
// GROUND TRUTH from the RAK2305 schematic (Figure 8, U6 ESP32-WROVER-B):
//   RXD1 = IO19 (WROVER pin 31) -> slot pin 33 -> (crossover) nRF52 TX P0.16
//   TXD1 = IO21 (WROVER pin 33) -> slot pin 34 -> (crossover) nRF52 RX P0.15
// NOTE: the RAK datasheet text/blog claim "RX1=GPIO20" is WRONG — IO20 isn't even
// bonded on the WROVER. RX must be GPIO19. (TX=GPIO21 is correct.)
#ifndef UART_UPLINK_BAUD
  #define UART_UPLINK_BAUD 115200
#endif
#ifndef UART_UPLINK_RX
  #define UART_UPLINK_RX 19   // ESP32 RX (RXD1/IO19) <- nRF52 TX (P0.16)
#endif
#ifndef UART_UPLINK_TX
  #define UART_UPLINK_TX 21   // ESP32 TX (TXD1/IO21) -> nRF52 RX (P0.15)
#endif

RAK2305Board board;
UartRadio radio_driver;

ESP32RTCClock rtc_clock;   // = system time(); NTP-synced via configTime() in setup

SensorManager sensors;

bool radio_init() {
  rtc_clock.begin();

  // NTP-sync the ESP32 system clock so MeshCore packet/advert timestamps are
  // correct (the node has no RTC; SNTP syncs once WiFi is up).
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  // The "radio" is the UART from the nRF52.
  Serial1.begin(UART_UPLINK_BAUD, SERIAL_8N1, UART_UPLINK_RX, UART_UPLINK_TX);
  radio_driver.setStream(&Serial1);
  return true;
}

mesh::LocalIdentity radio_new_identity() {
  // No radio noise source — seed from the ESP32 hardware RNG instead.
  static StdRNG rng;
  rng.begin(esp_random());
  return mesh::LocalIdentity(&rng);
}
