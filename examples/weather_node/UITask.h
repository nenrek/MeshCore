#pragma once

#include <helpers/ui/DisplayDriver.h>
#include <helpers/CommonCLI.h>
#include "NWSDisplayData.h"

class UITask {
  DisplayDriver*   _display;
  NWSDisplayData*  _nws_data;
  NodePrefs*       _node_prefs;
  char             _version_info[32];
  unsigned long    _next_read, _next_refresh, _auto_off, _next_page_flip;
  int              _prevBtnState;
  int              _page;

  void renderBootScreen();
  void renderHomeScreen();
  void renderInfoScreen();

public:
  UITask(DisplayDriver& display)
    : _display(&display), _nws_data(nullptr), _node_prefs(nullptr),
      _next_read(0), _next_refresh(0), _auto_off(0), _next_page_flip(0),
      _prevBtnState(1), _page(0) {}

  void begin(NodePrefs* node_prefs, NWSDisplayData* nws_data,
             const char* build_date, const char* firmware_version);
  void loop();
};
