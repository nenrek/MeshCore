#pragma once
#include <helpers/ESP32Board.h>

// Generic ESP32 board for the RAK2305 observer, but reports as "RAK3401" so the
// observer's Hardware/model on the maps matches the rest of the OSH_R01 node
// (the RAK3401 system this ESP32 is the uplink for) rather than "Generic ESP32".
class RAK2305Board : public ESP32Board {
  uint16_t _batt_mv = 0;   // pushed from the nRF52 over UART (MCBATT)
public:
  const char* getManufacturerName() const override { return "RAK3401"; }
  void setBattMilliVolts(uint16_t mv) { _batt_mv = mv; }
  uint16_t getBattMilliVolts() override { return _batt_mv; }
};
