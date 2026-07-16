#pragma once

#include <Arduino.h>

/**
 * @brief Driver for the Quectel BG77 (RAK5860) LTE-M / NB-IoT modem over UART AT commands.
 *
 * This is the transport used by CellularMQTTBridge in place of the ESP32/WiFi + esp-mqtt
 * path. The BG77 has an ONBOARD TLS stack and MQTT client, so the nRF52 host never runs
 * mbedTLS — it only drives the modem with AT commands:
 *
 *   power-up (PWRKEY) -> ATE0/CMEE -> SIM (CPIN) -> LTE-M band/RAT lock -> APN (CGDCONT)
 *   -> wait registration (CEREG) -> network time (QLTS) -> [TLS: QSSLCFG] -> MQTT client
 *   (QMTCFG/QMTOPEN/QMTCONN) -> publish (QMTPUBEX) -> keepalive.
 *
 * Publishing uses AT+QMTPUBEX (4096-byte ceiling) rather than AT+QMTPUB (560-byte cap),
 * because MeshCore JSON payloads run ~0.9-2 KB.
 *
 * The driver is non-blocking where it matters: begin() kicks off the bring-up state
 * machine and loop() advances it, so the mesh loop is never stalled waiting on the radio
 * network. Individual AT command/response exchanges are bounded by short timeouts.
 *
 * NOTE (hardware): the PWRKEY / power-enable pin mapping for the RAK5860 in the WisBlock
 * IO slot must be confirmed against the RAK5860 docs before bring-up (see the plan's
 * "Open hardware detail"). The pins are passed to the constructor so only the variant
 * wiring changes, not this driver.
 */
class BG77Modem {
public:
  enum State {
    ST_OFF = 0,        // powered down / not started
    ST_POWERING,       // PWRKEY pulsed, waiting for the modem to boot (RDY)
    ST_INIT,           // ATE0 / CMEE / SIM checks
    ST_NET_WAIT,       // waiting for LTE-M registration
    ST_TIME_SYNC,      // reading network time (QLTS) -> RTC
    ST_MQTT_SETUP,     // TLS + QMTCFG + QMTOPEN + QMTCONN
    ST_READY,          // connected to the broker, ready to publish
    ST_BACKOFF,        // error; waiting before a full re-attempt
    ST_GNSS_HOLD       // MQTT torn down; GNSS engine owns the radio for a one-shot fix
  };

  /**
   * @param serial      UART wired to the BG77 (Serial1 on the WisBlock: P0.15/P0.16).
   * @param pwrkey_pin  GPIO driving the BG77 PWRKEY line (active pulse turns it on). -1 = none.
   * @param power_en_pin GPIO gating modem power/rail (e.g. an IO-slot enable). -1 = none.
   * @param status_pin  BG77 STATUS output (HIGH once running), -1 if not wired.
   */
  BG77Modem(HardwareSerial& serial, int8_t pwrkey_pin, int8_t power_en_pin = -1, int8_t status_pin = -1);

  // ---- configuration (call before begin(); host/creds may change at runtime) ----
  void setAPN(const char* apn);
  void setBroker(const char* host, uint16_t port);
  void setCredentials(const char* username, const char* password);
  void setClientId(const char* client_id);
  void setKeepAlive(uint16_t seconds) { _keepalive = seconds; }
  /** Enable TLS. verify=true validates the server cert against the uploaded CA. */
  void setTLS(bool enabled, bool verify) { _tls = enabled; _tls_verify = verify; }
  /** LTE-M band bitmap hint as a Quectel hex string (empty = leave modem default). */
  void setBand(const char* band_hex);
  void setDebugStream(Stream* dbg) { _dbg = dbg; }

  // ---- lifecycle ----
  void begin();                 // start bring-up (non-blocking)
  void end();                   // MQTT disconnect + power down
  void loop();                  // advance the bring-up / keepalive state machine
  State state() const { return _state; }
  bool isReady() const { return _state == ST_READY; }
  /** True once the modem is configured and its radio is on (past power-up/init) — safe to
   *  run GNSS even if it hasn't registered/connected to the broker yet. */
  bool isUp() const { return _state >= ST_NET_WAIT; }
  bool isRegistered() const { return _registered; }
  /** True while a non-blocking atTick() command is in flight. GNSS (blocking AT) must not
   *  inject while this is set, or it clobbers the pending bring-up response. */
  bool isBusy() const { return _at_active; }
  int  lastRssiDbm() const { return _rssi_dbm; }   // from CSQ (-113..-51), 0 = unknown
  int  ceregStat() const { return _cereg_stat; }   // last +CEREG stat (0/2 search,1/5 reg)
  const char* stateName() const;                   // short name of the current state
  const char* lastError() const { return _last_err; }  // last MQTT open/conn result
  const char* lastPubResult() const { return _last_pub; } // last publish outcome (diag)

  /**
   * Publish a payload via AT+QMTPUBEX. Blocks briefly for the data prompt and the
   * +QMTPUB result (bounded by timeout). Returns false if not ready or on modem error.
   */
  bool publish(const char* topic, const char* payload, size_t len, uint8_t qos, bool retain);

  /**
   * One-time provisioning: upload a PEM CA cert into the modem filesystem so TLS verify
   * can validate the broker. Safe to call again (overwrites). Blocking; use at setup.
   */
  bool uploadCACert(const char* name, const char* pem, size_t pem_len);

  // ---- GNSS (BG77 integrated GPS/GLONASS/Galileo/BeiDou) ----
  /** Turn the standalone GNSS engine on (AT+QGPS=1). Idempotent. */
  bool gnssEnable();
  /** Turn GNSS off (AT+QGPSEND). */
  bool gnssDisable();
  /**
   * Read the current GNSS fix (AT+QGPSLOC=2). Returns true and fills lat/lon (signed
   * decimal degrees) if a fix is available; false while still acquiring (CME 516).
   */
  bool gnssGetFix(float& lat, float& lon);

  /**
   * Pause the MQTT uplink for a one-shot GNSS acquisition. On the BG77 the GNSS engine
   * preempts the LTE data bearer, so acquiring while connected drops MQTT uncleanly. This
   * closes the MQTT session and parks the modem in ST_GNSS_HOLD; the caller then runs the
   * GNSS fix and calls resumeFromGnss() to reconnect. Call only from a connected state.
   */
  bool holdForGnss();
  /** Release the GNSS hold and reconnect MQTT (re-runs TLS setup + open + connect). */
  void resumeFromGnss();

  /**
   * Send a raw AT command and collect the response lines into `out` (newline-joined),
   * up to the next OK/ERROR or timeout. For bench/bring-up debugging via the `at <cmd>`
   * CLI command. Returns true on OK. Shares the UART with loop() — call between cycles.
   */
  bool sendRawAT(const char* cmd, char* out, size_t out_size, uint32_t timeout_ms = 5000);

private:
  // AT engine ---------------------------------------------------------------
  void   flushInput();
  void   sendRaw(const char* s);
  void   sendAT(const char* cmd);                         // writes "cmd\r\n"
  /** Read one line (CRLF-terminated) into _line, up to timeout_ms. Returns length or -1. */
  int    readLine(uint32_t timeout_ms);
  /** Send cmd, then wait for a line containing `expect` (default "OK") before timeout. */
  bool   sendExpect(const char* cmd, const char* expect, uint32_t timeout_ms);
  /** Wait (no send) for a line containing `expect` before timeout. */
  bool   waitFor(const char* expect, uint32_t timeout_ms);
  /** Wait for the raw '>' data prompt (QMTPUBEX). Char-level, since the prompt is NOT
   *  newline-terminated and the line-based readLine()/waitFor() would never see it. */
  bool   waitForPrompt(uint32_t timeout_ms);

  // bring-up steps ----------------------------------------------------------
  void   powerPulse();
  void   enterBackoff(const char* why);
  void   enterState(State s);    // transition + reset the per-phase tick state
  // Non-blocking, time-sliced bring-up: each *Tick() advances at most one AT step per call
  // and returns AT_BUSY/OK/FAIL, so loop() never blocks long enough to starve the mesh RX
  // (the old blocking path stalled loop() ~45-90s per failed MQTT cycle -> node went deaf).
  enum AtRes { AT_BUSY = 0, AT_OK = 1, AT_FAIL = 2 };
  AtRes  atTick(const char* cmd, const char* expect, uint32_t timeout_ms); // send once, poll non-blocking
  AtRes  configureModemTick();  // ATE0, CMEE, CPIN, RAT/band, CGDCONT, CFUN
  AtRes  pollRegistrationTick();// CEREG / CSQ  (AT_OK = a full cycle finished; check isRegistered())
  AtRes  syncNetworkTimeTick(); // QLTS -> epoch via callback (best-effort)
  AtRes  setupTLSTick();        // QSSLCFG for the MQTT SSL context
  AtRes  openConnectTick();     // QMTCFG + QICSGP/QIACT/QIDNSCFG + QMTOPEN + QMTCONN

public:
  /** Set by the owner (bridge) so the modem can push network time into the mesh clock. */
  typedef void (*TimeSyncCb)(void* ctx, uint32_t epoch);
  void setTimeSyncCallback(TimeSyncCb cb, void* ctx) { _time_cb = cb; _time_ctx = ctx; }

private:
  HardwareSerial& _ser;
  int8_t  _pwrkey, _power_en, _status_pin;
  Stream* _dbg = nullptr;

  char     _apn[40]      = "hologram";
  char     _host[80]     = {0};
  uint16_t _port         = 8883;
  char     _user[40]     = {0};
  char     _pass[64]     = {0};
  char     _client_id[40]= {0};
  char     _band[24]     = {0};
  uint16_t _keepalive    = 300;    // seconds; longer than the WiFi build's 45s to save airtime/power
  bool     _tls          = true;
  bool     _tls_verify   = true;

  static const uint8_t MQTT_CLIENT_IDX = 0;   // AT+QMTCFG client index (0..5)
  static const uint8_t SSL_CTX_IDX     = 2;   // AT+QSSLCFG context index (0..5)
  static const char*   CA_FILE_NAME;          // "cacert.pem"

  State    _state        = ST_OFF;
  bool     _registered   = false;
  int      _rssi_dbm     = 0;
  int      _cereg_stat   = -1;
  char     _last_err[48] = "none";
  char     _last_pub[20] = "none";   // diagnostic: outcome of the last publish() attempt
  uint16_t _msg_id       = 1;
  unsigned long _state_since = 0;   // millis() when we entered _state
  unsigned long _last_poll   = 0;
  unsigned long _backoff_until = 0;
  uint8_t  _backoff_step = 0;

  // non-blocking tick engine state
  bool     _at_active    = false;   // an AT command is in flight (sent, awaiting expect)
  uint32_t _at_deadline  = 0;
  char     _at_expect[16]= {0};
  uint8_t  _phase_step   = 0;       // sub-step within the current bring-up phase
  uint8_t  _at_retry     = 0;       // retry counter (e.g. the initial AT probe)
  uint8_t  _mqtt_stage   = 0;       // ST_MQTT_SETUP: 0 = TLS config, 1 = open/connect

  static const size_t LINE_MAX = 256;
  char     _line[LINE_MAX];
  size_t   _line_len = 0;    // persistent partial-line index: readLine() preserves an
                             // incomplete line across calls so short poll budgets (atTick)
                             // never truncate a response that straddles two reads.

  TimeSyncCb _time_cb = nullptr;
  void*      _time_ctx = nullptr;
};
