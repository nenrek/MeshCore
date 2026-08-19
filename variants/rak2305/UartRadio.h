#pragma once
// ---------------------------------------------------------------------------
// UartRadio — a mesh::Radio whose "air" is a UART link to a companion nRF52.
//
// The RAK2305 (ESP32-WROVER) has no LoRa radio of its own. On the RAK3401 the
// nRF52840 owns the SX1262 and, built with -D UART_UPLINK (see the meshcore
// mqtt_repeater fork), forwards every heard packet to us over UART as a line:
//
//     MCPKT <raw_hex> <SNR f.2> <RSSI int>\r\n
//
// By presenting those frames through the standard mesh::Radio interface
// (recvRaw + getLastSNR/getLastRSSI), the entire existing MeshCore receive path
// runs unchanged: Dispatcher::checkRecv() -> logRxRaw() -> storeRawRadioData(),
// then parse -> logRx() -> MQTTBridge::onPacketReceived(). No injection hacks.
//
// We never transmit (there is no radio): startSendRaw() is a no-op that reports
// success so the repeater's forward logic completes harmlessly.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Mesh.h>
#include <esp_system.h>   // esp_random()

#ifndef UART_UPLINK_LINE_MAX
  // hex is <= 2*256 = 512, plus "MCPKT " + " <snr> <rssi>"
  #define UART_UPLINK_LINE_MAX 600
#endif

class UartRadio : public mesh::Radio {
  Stream*  _uart = nullptr;
  char     _line[UART_UPLINK_LINE_MAX];
  int      _linelen = 0;
  uint8_t  _raw[256];
  int      _rawlen = 0;
  float    _snr = 0.0f;
  float    _rssi = 0.0f;
  bool     _have = false;        // a parsed frame is waiting to be consumed
  uint32_t _rx_count = 0;        // frames handed to the stack (for stats)
  char     _cmd[160];            // a relayed CLI command from the nRF52 (MCCMD)
  bool     _have_cmd = false;
  char     _dfu_url[160];        // Phase 7: nRF52 OTA base URL (MCPULL)
  bool     _have_dfu = false;
  float    _r_freq = 0, _r_bw = 0;   // radio config pushed by the nRF52 (MCRADIO)
  uint8_t  _r_sf = 0, _r_cr = 0;
  bool     _have_radio = false;
  uint8_t  _id_prv[PRV_KEY_SIZE];    // node identity pushed by the nRF52 (MCIDENT)
  bool     _have_id = false;
  char     _r_name[32];              // node name (MCNAME)
  bool     _have_name = false;
  uint16_t _r_batt = 0;              // battery mV (MCBATT)
  bool     _have_batt = false;
  // Full radio/mesh stats pushed by the nRF52 (MCSTA) — the REAL stats for the
  // observer's /status (this ESP32 has no radio, so its own stats are all blank).
  int16_t  _s_noise = 0;
  uint32_t _s_tx_air = 0, _s_rx_air = 0, _s_recv_err = 0;
  uint32_t _s_queue = 0, _s_uptime = 0, _s_pkts_sent = 0, _s_pkts_recv = 0;
  uint16_t _s_errflags = 0;
  bool     _have_stats = false;      // a new snapshot since last taken
  bool     _stats_seen = false;      // ever received (gates the radio-getter overrides)

  static int hexNib(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  void parseLine() {
    // Identity pushed by the nRF52: "MCIDENT <prvkey_hex>" — the ESP32 adopts the
    // node's key so the two act as ONE device (same observer key on the maps).
    if (strncmp(_line, "MCIDENT ", 8) == 0) {
      const char* h = _line + 8;
      if ((int)strlen(h) >= 2 * PRV_KEY_SIZE) {
        bool ok = true;
        for (int i = 0; i < PRV_KEY_SIZE; i++) {
          int hi = hexNib(h[2*i]), lo = hexNib(h[2*i+1]);
          if (hi < 0 || lo < 0) { ok = false; break; }
          _id_prv[i] = (uint8_t)((hi << 4) | lo);
        }
        if (ok) _have_id = true;
      }
      return;
    }
    if (strncmp(_line, "MCNAME ", 7) == 0) {        // node name -> observer origin
      strncpy(_r_name, _line + 7, sizeof(_r_name) - 1);
      _r_name[sizeof(_r_name) - 1] = 0; _have_name = true; return;
    }
    if (strncmp(_line, "MCBATT ", 7) == 0) {        // battery mV
      _r_batt = (uint16_t)atoi(_line + 7); _have_batt = true; return;
    }
    // Full stats snapshot: "MCSTA <noise> <tx_air> <rx_air> <recv_err> <queue>
    // <uptime> <errflags> <pkts_sent> <pkts_recv>". Stored + surfaced in /status.
    if (strncmp(_line, "MCSTA ", 6) == 0) {
      long v[9]; int i = 0;
      char* tok = strtok(_line + 6, " ");
      while (tok && i < 9) { v[i++] = strtol(tok, nullptr, 10); tok = strtok(nullptr, " "); }
      if (i == 9) {
        _s_noise     = (int16_t)v[0];
        _s_tx_air    = (uint32_t)v[1];
        _s_rx_air    = (uint32_t)v[2];
        _s_recv_err  = (uint32_t)v[3];
        _s_queue     = (uint32_t)v[4];
        _s_uptime    = (uint32_t)v[5];
        _s_errflags  = (uint16_t)v[6];
        _s_pkts_sent = (uint32_t)v[7];
        _s_pkts_recv = (uint32_t)v[8];
        _have_stats = true; _stats_seen = true;
      }
      return;
    }
    // Radio config pushed by the nRF52 (source of truth — the ESP32 has no radio):
    // "MCRADIO <freq> <bw> <sf> <cr>". Applied to NodePrefs so /status is accurate.
    if (strncmp(_line, "MCRADIO ", 8) == 0) {
      char* p = _line + 8;
      float f = (float)atof(p);
      char* p2 = strchr(p, ' ');      if (!p2) return; float b = (float)atof(p2 + 1);
      char* p3 = strchr(p2 + 1, ' '); if (!p3) return; int   s = atoi(p3 + 1);
      char* p4 = strchr(p3 + 1, ' '); if (!p4) return; int   c = atoi(p4 + 1);
      _r_freq = f; _r_bw = b; _r_sf = (uint8_t)s; _r_cr = (uint8_t)c;
      _have_radio = true;
      return;
    }
    // CLI relay: a config command from the nRF52 (so the USB-less ESP32 is
    // manageable from the nRF52's MeshCore CLI). Stashed for the main loop.
    if (strncmp(_line, "MCCMD ", 6) == 0) {
      strncpy(_cmd, _line + 6, sizeof(_cmd) - 1);
      _cmd[sizeof(_cmd) - 1] = 0;
      _have_cmd = true;
      return;
    }
    // Phase 7 nRF52-OTA: the nRF52 asks us to fetch a firmware + host its UART DFU.
    // "MCPULL <baseUrl>" -> stashed; the main loop downloads, replies MCPULLED (which
    // makes the nRF52 enterUartDfu), then drives the DFU. nRF52-driven so the reset
    // timing is ours-then-theirs (no race with the MCCMD/MCRSP relay).
    if (strncmp(_line, "MCPULL ", 7) == 0) {
      strncpy(_dfu_url, _line + 7, sizeof(_dfu_url) - 1);
      _dfu_url[sizeof(_dfu_url) - 1] = 0;
      _have_dfu = true;
      return;
    }
    if (strncmp(_line, "MCPKT ", 6) != 0) return;     // ignore anything else
    char* hex = _line + 6;
    char* sp1 = strchr(hex, ' ');
    if (!sp1) return;
    *sp1 = 0;                                          // terminate hex token
    char* rest = sp1 + 1;                              // "<snr> <rssi>"

    int hl = (int)strlen(hex);
    if (hl < 2 || (hl & 1)) return;                    // need even-length hex
    int n = hl / 2;
    if (n > (int)sizeof(_raw)) n = (int)sizeof(_raw);
    for (int i = 0; i < n; i++) {
      int hi = hexNib(hex[2*i]), lo = hexNib(hex[2*i+1]);
      if (hi < 0 || lo < 0) return;                    // bad hex -> drop frame
      _raw[i] = (uint8_t)((hi << 4) | lo);
    }
    _rawlen = n;
    _snr = (float)atof(rest);
    char* sp2 = strchr(rest, ' ');
    _rssi = sp2 ? (float)atof(sp2 + 1) : 0.0f;
    _have = true;
  }

  void pump() {
    if (!_uart || _have) return;                       // hold until consumed
    while (_uart->available()) {
      char c = (char)_uart->read();
      if (c == '\n' || c == '\r') {
        if (_linelen > 0) {
          _line[_linelen] = 0;
          parseLine();
          _linelen = 0;
          if (_have) return;                           // one frame per recvRaw
        }
      } else if (_linelen < (int)sizeof(_line) - 1) {
        _line[_linelen++] = c;
      } else {
        _linelen = 0;                                  // overrun -> resync
      }
    }
  }

public:
  void setStream(Stream* s) { _uart = s; }

  // ---- mesh::Radio interface ----
  int recvRaw(uint8_t* bytes, int sz) override {
    pump();
    if (!_have) return 0;
    int n = _rawlen; if (n > sz) n = sz;
    memcpy(bytes, _raw, n);
    _have = false;                                     // keep _snr/_rssi for getLast*()
    _rx_count++;
    return n;
  }

  float getLastSNR()  const override { return _snr; }
  float getLastRSSI() const override { return _rssi; }

  // Real radio stats pushed from the nRF52 (MCSTA). The bridge's /status reads
  // noise floor + recv errors off _radio (this) — return the nRF52 values so they
  // aren't blank. Fall back to base (0) until the first snapshot arrives.
  int      getNoiseFloor() const override        { return _stats_seen ? (int)_s_noise : 0; }
  uint32_t getPacketsRecvErrors() const override { return _s_recv_err; }

  // Stats counters StatsFormatHelper reads off the concrete driver type.
  uint32_t getPacketsRecv() const { return _stats_seen ? _s_pkts_recv : _rx_count; }
  uint32_t getPacketsSent() const { return _s_pkts_sent; }   // from nRF52 (this ESP32 never TXes)

  // Concrete-driver API MyMesh calls directly on radio_driver — no-ops here
  // since there is no real radio to parameterize, power, or reset.
  void resetStats() { _rx_count = 0; }
  void setParams(float, float, uint8_t, uint8_t) { }
  void setTxPower(uint8_t) { }
  bool getRxBoostedGainMode() const { return false; }
  bool setRxBoostedGainMode(bool) { return false; }  // 1.17 MyMesh returns this; no real radio
  void powerOff() { }                                // 1.17 ESP32Board::enterDeepSleep calls radio_driver.powerOff()

  uint32_t getEstAirtimeFor(int) override { return 0; }
  float    packetScore(float, int) override { return 0.0f; }
  bool     startSendRaw(const uint8_t*, int) override { return true; }  // no radio: pretend sent
  bool     isSendComplete() override { return true; }
  void     onSendFinished() override { }
  bool     isInRecvMode() const override { return true; }

  // Used by main.cpp to seed the RNG (no radio entropy available).
  uint32_t getRngSeed() { return esp_random(); }

  // ---- CLI relay (config from the nRF52 over the UART link) ----
  bool hasHostCommand() const { return _have_cmd; }
  void takeHostCommand(char* dst, int n) {
    strncpy(dst, _cmd, n - 1); dst[n - 1] = 0; _have_cmd = false;
  }
  void sendHostReply(const char* s) {
    if (!_uart) return;
    _uart->print("MCRSP "); _uart->print(s); _uart->print("\r\n");
  }

  // ---- Phase 7 nRF52-OTA pull (nRF52-driven) ----
  bool hasDfuPull() const { return _have_dfu; }
  void takeDfuPull(char* dst, int n) {
    strncpy(dst, _dfu_url, n - 1); dst[n - 1] = 0; _have_dfu = false;
  }

  // ---- info pushed from the nRF52 (so the two act as one logical device) ----
  bool hasRadioConfig() const { return _have_radio; }
  void takeRadioConfig(float& f, float& b, uint8_t& s, uint8_t& c) {
    f = _r_freq; b = _r_bw; s = _r_sf; c = _r_cr; _have_radio = false;
  }
  bool hasIdentity() const { return _have_id; }
  void takeIdentity(uint8_t* dst) { memcpy(dst, _id_prv, PRV_KEY_SIZE); _have_id = false; }
  bool hasName() const { return _have_name; }
  void takeName(char* dst, int n) { strncpy(dst, _r_name, n - 1); dst[n - 1] = 0; _have_name = false; }
  bool hasBatt() const { return _have_batt; }
  uint16_t takeBatt() { _have_batt = false; return _r_batt; }

  // Dispatcher-sourced stats (airtime/queue/uptime/errflags/packets) that the
  // bridge can't get from a real radio here — main.cpp pushes these into the
  // bridge's external-stats override. (noise + recv_err come via the getters above.)
  bool hasStats() const { return _have_stats; }
  void takeStats(uint32_t& tx_air, uint32_t& rx_air, uint32_t& queue,
                 uint32_t& uptime, uint16_t& errflags,
                 uint32_t& pkts_sent, uint32_t& pkts_recv) {
    tx_air = _s_tx_air; rx_air = _s_rx_air; queue = _s_queue;
    uptime = _s_uptime; errflags = _s_errflags;
    pkts_sent = _s_pkts_sent; pkts_recv = _s_pkts_recv;
    _have_stats = false;
  }
};
