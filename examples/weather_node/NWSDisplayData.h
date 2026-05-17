#pragma once
#include <stdint.h>

struct NWSDisplayData {
  bool     eth_ready;
  char     zone[64];
  uint32_t polls_total;
  uint32_t alerts_sent_total;
  int      active_alerts;
  char     last_header[80];   // e.g. "[Severe] Winter Storm Warning"
  bool     uk_enabled;
  bool     uk_last_ok;
  char     uk_host[40];
  uint16_t uk_port;
  uint32_t display_timeout_secs;  // 0 = always on
};
