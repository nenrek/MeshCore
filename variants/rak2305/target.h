#pragma once

#include <helpers/ESP32Board.h>
#include <helpers/AutoDiscoverRTCClock.h>
#include <helpers/SensorManager.h>
#include "UartRadio.h"
#include "RAK2305Board.h"

// Bare RAK2305 (ESP32-WROVER) observer: generic ESP32 board (reports as RAK3401),
// no LoRa radio — packets arrive over UART from the companion nRF52 via UartRadio.
extern RAK2305Board board;
extern UartRadio radio_driver;
extern ESP32RTCClock rtc_clock;   // system clock (NTP-synced via configTime) — drives advert timestamps
extern SensorManager sensors;

bool radio_init();
mesh::LocalIdentity radio_new_identity();
