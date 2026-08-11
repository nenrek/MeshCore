#include <Arduino.h>
#include "target.h"
#include <helpers/ArduinoHelpers.h>

RAK3401Board board;

#ifndef PIN_USER_BTN
  #define PIN_USER_BTN (-1)
#endif

#ifdef DISPLAY_CLASS
  DISPLAY_CLASS display;
  MomentaryButton user_btn(PIN_USER_BTN, 1000, true, true);

  #if defined(PIN_USER_BTN_ANA)
  MomentaryButton analog_btn(PIN_USER_BTN_ANA, 1000, 20);
  #endif
#endif

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);

#if ENV_INCLUDE_GPS
  #include <helpers/sensors/MicroNMEALocationProvider.h>
  MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1);
  EnvironmentSensorManager sensors = EnvironmentSensorManager(nmea);
#else
  // GPS disabled on the cellular variant so Serial1 is free for the RAK5860/BG77 modem.
  EnvironmentSensorManager sensors;
#endif

bool radio_init() {
  rtc_clock.begin(Wire);
  return radio.std_init(&SPI);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

// ---------------------------------------------------------------------------
// nRF52 wall clock for MQTTMessageBuilder
// ---------------------------------------------------------------------------
// CellularMQTTBridge reuses MQTTMessageBuilder, which reads gettimeofday() for every JSON
// timestamp. That's SNTP-backed on ESP32, but on the nRF52 libc has no backing clock, so
// route _gettimeofday() to the MeshCore RTC (rtc_clock) — which the BG77 modem seeds from
// LTE network time (AT+QLTS). The Adafruit nRF52 core provides a weak _gettimeofday stub;
// this strong definition overrides it. Only compiled into the cellular variant.
#include <sys/time.h>
extern "C" int _gettimeofday(struct timeval* tv, void* /*tz*/) {
  if (tv) {
    tv->tv_sec = (time_t)rtc_clock.getCurrentTime();
    tv->tv_usec = 0;
  }
  return 0;
}

