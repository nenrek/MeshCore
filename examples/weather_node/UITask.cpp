#include "UITask.h"
#include <Arduino.h>
#include <ctype.h>

#define AUTO_OFF_MILLIS     60000
#define BOOT_SCREEN_MILLIS   4000
#define REFRESH_MILLIS        1000
#define PAGE_FLIP_MILLIS     10000

// 'meshcore', 128x13px
static const uint8_t meshcore_logo[] PROGMEM = {
    0x3c, 0x01, 0xe3, 0xff, 0xc7, 0xff, 0x8f, 0x03, 0x87, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe, 0x1f, 0xfe,
    0x3c, 0x03, 0xe3, 0xff, 0xc7, 0xff, 0x8e, 0x03, 0x8f, 0xfe, 0x3f, 0xfe, 0x1f, 0xff, 0x1f, 0xfe,
    0x3e, 0x03, 0xc3, 0xff, 0x8f, 0xff, 0x0e, 0x07, 0x8f, 0xfe, 0x7f, 0xfe, 0x1f, 0xff, 0x1f, 0xfc,
    0x3e, 0x07, 0xc7, 0x80, 0x0e, 0x00, 0x0e, 0x07, 0x9e, 0x00, 0x78, 0x0e, 0x3c, 0x0f, 0x1c, 0x00,
    0x3e, 0x0f, 0xc7, 0x80, 0x1e, 0x00, 0x0e, 0x07, 0x1e, 0x00, 0x70, 0x0e, 0x38, 0x0f, 0x3c, 0x00,
    0x7f, 0x0f, 0xc7, 0xfe, 0x1f, 0xfc, 0x1f, 0xff, 0x1c, 0x00, 0x70, 0x0e, 0x38, 0x0e, 0x3f, 0xf8,
    0x7f, 0x1f, 0xc7, 0xfe, 0x0f, 0xff, 0x1f, 0xff, 0x1c, 0x00, 0xf0, 0x0e, 0x38, 0x0e, 0x3f, 0xf8,
    0x7f, 0x3f, 0xc7, 0xfe, 0x0f, 0xff, 0x1f, 0xff, 0x1c, 0x00, 0xf0, 0x1e, 0x3f, 0xfe, 0x3f, 0xf0,
    0x77, 0x3b, 0x87, 0x00, 0x00, 0x07, 0x1c, 0x0f, 0x3c, 0x00, 0xe0, 0x1c, 0x7f, 0xfc, 0x38, 0x00,
    0x77, 0xfb, 0x8f, 0x00, 0x00, 0x07, 0x1c, 0x0f, 0x3c, 0x00, 0xe0, 0x1c, 0x7f, 0xf8, 0x38, 0x00,
    0x73, 0xf3, 0x8f, 0xff, 0x0f, 0xff, 0x1c, 0x0e, 0x3f, 0xf8, 0xff, 0xfc, 0x70, 0x78, 0x7f, 0xf8,
    0xe3, 0xe3, 0x8f, 0xff, 0x1f, 0xfe, 0x3c, 0x0e, 0x3f, 0xf8, 0xff, 0xfc, 0x70, 0x3c, 0x7f, 0xf8,
    0xe3, 0xe3, 0x8f, 0xff, 0x1f, 0xfc, 0x3c, 0x0e, 0x1f, 0xf8, 0xff, 0xf8, 0x70, 0x3c, 0x7f, 0xf8,
};

// Strip alphabetic zone prefix for display: "WIZ038,WIZ039,WIZ040" -> "038,039,040"
static void abbreviateZones(const char* zones, char* out, int out_size) {
  int j = 0;
  bool in_prefix = true;
  for (int i = 0; zones[i] && j < out_size - 1; i++) {
    char c = zones[i];
    if (c == ',') {
      if (j < out_size - 1) out[j++] = ',';
      in_prefix = true;
    } else if (in_prefix && isalpha((unsigned char)c)) {
      // skip alphabetic prefix
    } else {
      in_prefix = false;
      out[j++] = c;
    }
  }
  out[j] = 0;
}

// Determine alert color from header string (e.g. "[Severe] Winter Storm Warning")
static DisplayDriver::Color alertColor(const char* header) {
  if (!header || !header[0])            return DisplayDriver::LIGHT;
  if (strstr(header, "Extreme"))        return DisplayDriver::RED;
  if (strstr(header, "Severe"))         return DisplayDriver::YELLOW;
  if (strstr(header, "Test"))           return DisplayDriver::BLUE;
  return DisplayDriver::LIGHT;
}

void UITask::begin(NodePrefs* node_prefs, NWSDisplayData* nws_data,
                   const char* build_date, const char* firmware_version) {
  _node_prefs  = node_prefs;
  _nws_data    = nws_data;
  _prevBtnState = HIGH;
  uint32_t timeout_ms = (_nws_data && _nws_data->display_timeout_secs > 0)
    ? _nws_data->display_timeout_secs * 1000 : AUTO_OFF_MILLIS;
  _auto_off       = millis() + timeout_ms;
  _next_page_flip = millis() + BOOT_SCREEN_MILLIS + PAGE_FLIP_MILLIS;

  char ver[32];
  strncpy(ver, firmware_version, sizeof(ver) - 1);
  ver[sizeof(ver) - 1] = 0;
  char* dash = strchr(ver, '-');
  if (dash) *dash = 0;
  snprintf(_version_info, sizeof(_version_info), "%s (%s)", ver, build_date);

  _display->turnOn();
}

void UITask::renderBootScreen() {
  int cx = _display->width() / 2;

  _display->setColor(DisplayDriver::BLUE);
  _display->drawXbm(((_display->width() - 128) / 2), 3, meshcore_logo, 128, 13);

  _display->setColor(DisplayDriver::LIGHT);
  _display->setTextSize(1);
  _display->drawTextCentered(cx, 22, _version_info);

  _display->setColor(DisplayDriver::YELLOW);
  _display->drawTextCentered(cx, 35, "< NWS Weather Node >");
}

void UITask::renderHomeScreen() {
  if (!_nws_data || !_node_prefs) return;

  const int W   = _display->width();
  const int L1  = 0;
  const int L2  = 12;
  const int L3  = 24;
  const int L4  = 36;
  const int L5  = 52;

  _display->setTextSize(1);

  // ── Line 1: node name ──────────────────────────────────────────────
  _display->setColor(DisplayDriver::GREEN);
  const char* name = (_node_prefs->node_name[0]) ? _node_prefs->node_name : "NWS Alerts";
  _display->drawTextEllipsized(0, L1, W, name);

  // thin divider
  _display->setColor(DisplayDriver::LIGHT);
  _display->fillRect(0, L1 + 9, W, 1);

  // ── Line 2: ETH status + zone ──────────────────────────────────────
  _display->setColor(DisplayDriver::YELLOW);
  char zone_abbr[24];
  abbreviateZones(_nws_data->zone, zone_abbr, sizeof(zone_abbr));
  char line2[32];
  snprintf(line2, sizeof(line2), "ETH:%s  %s",
    _nws_data->eth_ready ? "OK" : "DOWN", zone_abbr);
  _display->drawTextEllipsized(0, L2, W, line2);

  // ── Line 3: poll + alert counts ───────────────────────────────────
  _display->setColor(DisplayDriver::LIGHT);
  char line3[32];
  snprintf(line3, sizeof(line3), "Polls:%lu  Alerts:%lu",
    (unsigned long)_nws_data->polls_total,
    (unsigned long)_nws_data->alerts_sent_total);
  _display->drawTextEllipsized(0, L3, W, line3);

  // ── Line 4: last alert ─────────────────────────────────────────────
  if (_nws_data->last_header[0]) {
    _display->setColor(alertColor(_nws_data->last_header));
    _display->drawTextEllipsized(0, L4, W, _nws_data->last_header);
  } else {
    _display->setColor(DisplayDriver::LIGHT);
    _display->setCursor(0, L4);
    _display->print("No alerts yet");
  }

  // ── Line 5: Uptime Kuma or LoRa params ────────────────────────────
  if (_nws_data->uk_enabled) {
    char line5[32];
    snprintf(line5, sizeof(line5), "UK:%s  %s",
      _nws_data->uk_last_ok ? "OK" : "FAIL", _nws_data->uk_host);
    _display->setColor(_nws_data->uk_last_ok ? DisplayDriver::GREEN : DisplayDriver::RED);
    _display->drawTextEllipsized(0, L5, W, line5);
  } else {
    char line5[32];
    snprintf(line5, sizeof(line5), "%06.3f SF%d BW%.1f",
      _node_prefs->freq, _node_prefs->sf, _node_prefs->bw);
    _display->setColor(DisplayDriver::YELLOW);
    _display->setCursor(0, L5);
    _display->print(line5);
  }
}

void UITask::renderInfoScreen() {
  if (!_nws_data || !_node_prefs) return;

  const int W  = _display->width();
  const int L1 = 0;
  const int L2 = 12;
  const int L3 = 24;
  const int L4 = 36;
  const int L5 = 52;

  _display->setTextSize(1);

  // ── Line 1: node name ──────────────────────────────────────────────
  _display->setColor(DisplayDriver::GREEN);
  const char* name = (_node_prefs->node_name[0]) ? _node_prefs->node_name : "NWS Alerts";
  _display->drawTextEllipsized(0, L1, W, name);

  _display->setColor(DisplayDriver::LIGHT);
  _display->fillRect(0, L1 + 9, W, 1);

  // ── Line 2: LoRa params ────────────────────────────────────────────
  _display->setColor(DisplayDriver::YELLOW);
  char line2[32];
  snprintf(line2, sizeof(line2), "%06.3f SF%d BW%.0f",
    _node_prefs->freq, _node_prefs->sf, _node_prefs->bw);
  _display->drawTextEllipsized(0, L2, W, line2);

  // ── Line 3: active alerts + zone ──────────────────────────────────
  _display->setColor(DisplayDriver::LIGHT);
  char zone_abbr[24];
  abbreviateZones(_nws_data->zone, zone_abbr, sizeof(zone_abbr));
  char line3[32];
  snprintf(line3, sizeof(line3), "Zone:%s  Active:%d",
    zone_abbr, _nws_data->active_alerts);
  _display->drawTextEllipsized(0, L3, W, line3);

  // ── Line 4: uptime ────────────────────────────────────────────────
  _display->setColor(DisplayDriver::LIGHT);
  unsigned long up = millis() / 1000;
  char line4[32];
  snprintf(line4, sizeof(line4), "Up: %lud %02luh %02lum",
    up / 86400, (up % 86400) / 3600, (up % 3600) / 60);
  _display->drawTextEllipsized(0, L4, W, line4);

  // ── Line 5: UK or poll interval ───────────────────────────────────
  _display->setColor(DisplayDriver::LIGHT);
  if (_nws_data->uk_enabled) {
    char line5[32];
    snprintf(line5, sizeof(line5), "UK:%s  %s",
      _nws_data->uk_last_ok ? "OK" : "FAIL", _nws_data->uk_host);
    _display->setColor(_nws_data->uk_last_ok ? DisplayDriver::GREEN : DisplayDriver::RED);
    _display->drawTextEllipsized(0, L5, W, line5);
  } else {
    _display->setColor(DisplayDriver::LIGHT);
    _display->setCursor(0, L5);
    _display->print("UK: disabled");
  }
}

void UITask::loop() {
#ifdef PIN_USER_BTN
  if (millis() >= _next_read) {
    int btnState = digitalRead(PIN_USER_BTN);
    if (btnState != _prevBtnState) {
      if (btnState == LOW) {
        _display->turnOn();
        uint32_t t = (_nws_data && _nws_data->display_timeout_secs > 0)
          ? _nws_data->display_timeout_secs * 1000 : AUTO_OFF_MILLIS;
        _auto_off = millis() + t;
        _page = 0;
        _next_page_flip = millis() + PAGE_FLIP_MILLIS;
      }
      _prevBtnState = btnState;
    }
    _next_read = millis() + 200;
  }
#endif

  if (_display->isOn()) {
    // Page flip for burn-in prevention
    if (millis() >= _next_page_flip && millis() >= BOOT_SCREEN_MILLIS) {
      _page = (_page + 1) % 2;
      _next_page_flip = millis() + PAGE_FLIP_MILLIS;
      _next_refresh = 0;  // force immediate redraw on flip
    }

    if (millis() >= _next_refresh) {
      _display->startFrame();
      if (millis() < BOOT_SCREEN_MILLIS)
        renderBootScreen();
      else if (_page == 0)
        renderHomeScreen();
      else
        renderInfoScreen();
      _display->endFrame();
      _next_refresh = millis() + REFRESH_MILLIS;
    }

    bool always_on = (_nws_data && _nws_data->display_timeout_secs == 0);
    if (!always_on && millis() > _auto_off) {
      _display->turnOff();
    }
  }
}
