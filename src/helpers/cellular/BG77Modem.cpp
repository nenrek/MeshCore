#include "BG77Modem.h"
#include <string.h>
#include <stdlib.h>

const char* BG77Modem::CA_FILE_NAME = "cacert.pem";

const char* BG77Modem::stateName() const {
  switch (_state) {
    case ST_OFF:        return "off";
    case ST_POWERING:   return "powering";
    case ST_INIT:       return "init";
    case ST_NET_WAIT:   return "netwait";
    case ST_TIME_SYNC:  return "timesync";
    case ST_MQTT_SETUP: return "mqttsetup";
    case ST_READY:      return "ready";
    case ST_BACKOFF:    return "backoff";
    default:            return "?";
  }
}

// Timings (ms). Conservative; tune on hardware.
static const uint32_t T_AT      = 1000;    // simple AT/OK exchange
static const uint32_t T_CFG     = 3000;    // config commands
static const uint32_t T_BOOT    = 15000;   // modem power-up to RDY
static const uint32_t T_QMTOPEN = 45000;   // TCP + TLS handshake to broker (cellular TLS can be slow)
static const uint32_t T_QMTCONN = 15000;   // MQTT CONNECT
static const uint32_t T_PUB     = 15000;   // publish round trip
static const uint32_t REG_POLL_MS = 3000;  // registration poll cadence

BG77Modem::BG77Modem(HardwareSerial& serial, int8_t pwrkey_pin, int8_t power_en_pin, int8_t status_pin)
  : _ser(serial), _pwrkey(pwrkey_pin), _power_en(power_en_pin), _status_pin(status_pin) {}

// ---- config ---------------------------------------------------------------

void BG77Modem::setAPN(const char* apn) {
  if (apn) { strncpy(_apn, apn, sizeof(_apn) - 1); _apn[sizeof(_apn) - 1] = 0; }
}
void BG77Modem::setBroker(const char* host, uint16_t port) {
  if (host) { strncpy(_host, host, sizeof(_host) - 1); _host[sizeof(_host) - 1] = 0; }
  if (port) _port = port;
}
void BG77Modem::setCredentials(const char* username, const char* password) {
  if (username) { strncpy(_user, username, sizeof(_user) - 1); _user[sizeof(_user) - 1] = 0; }
  if (password) { strncpy(_pass, password, sizeof(_pass) - 1); _pass[sizeof(_pass) - 1] = 0; }
}
void BG77Modem::setClientId(const char* client_id) {
  if (client_id) { strncpy(_client_id, client_id, sizeof(_client_id) - 1); _client_id[sizeof(_client_id) - 1] = 0; }
}
void BG77Modem::setBand(const char* band_hex) {
  if (band_hex) { strncpy(_band, band_hex, sizeof(_band) - 1); _band[sizeof(_band) - 1] = 0; }
}

// ---- AT engine ------------------------------------------------------------

void BG77Modem::flushInput() {
  while (_ser.available()) _ser.read();
  _line_len = 0;   // drop any partial line too
}

void BG77Modem::sendRaw(const char* s) {
  _ser.print(s);
  if (_dbg) { _dbg->print("[BG77>] "); _dbg->println(s); }
}

void BG77Modem::sendAT(const char* cmd) {
  _ser.print(cmd);
  _ser.print("\r\n");
  if (_dbg) { _dbg->print("[BG77>] "); _dbg->println(cmd); }
}

int BG77Modem::readLine(uint32_t timeout_ms) {
  // Non-destructive on timeout: characters accumulate into the persistent _line/_line_len,
  // so a line that doesn't finish within `timeout_ms` is resumed on the next call rather than
  // lost. This lets atTick() poll with tiny budgets without corrupting straddling responses.
  unsigned long start = millis();
  for (;;) {
    while (_ser.available()) {
      char c = (char)_ser.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (_line_len == 0) continue;      // skip blank lines
        _line[_line_len] = 0;
        int len = (int)_line_len;
        _line_len = 0;
        if (_dbg) { _dbg->print("[BG77<] "); _dbg->println(_line); }
        return len;
      }
      if (_line_len < LINE_MAX - 1) _line[_line_len++] = c;
    }
    if ((millis() - start) >= timeout_ms) return -1;   // partial preserved in _line/_line_len
    yield();
  }
}

bool BG77Modem::waitFor(const char* expect, uint32_t timeout_ms) {
  unsigned long start = millis();
  while ((millis() - start) < timeout_ms) {
    int n = readLine(timeout_ms - (millis() - start));
    if (n < 0) break;
    if (strstr(_line, expect)) return true;
    if (strstr(_line, "ERROR")) return false;
  }
  return false;
}

bool BG77Modem::sendExpect(const char* cmd, const char* expect, uint32_t timeout_ms) {
  flushInput();
  sendAT(cmd);
  return waitFor(expect ? expect : "OK", timeout_ms);
}

bool BG77Modem::waitForPrompt(uint32_t timeout_ms) {
  // The QMTPUBEX data prompt arrives as "\r\n> " with NO trailing newline, so readLine()
  // (which only returns on '\n') can never see it. Scan raw bytes for '>' instead.
  unsigned long start = millis();
  while ((millis() - start) < timeout_ms) {
    while (_ser.available()) {
      char c = (char)_ser.read();
      if (c == '>') { _line_len = 0; return true; }   // drop any partial; prompt seen
    }
    yield();
  }
  return false;
}

// ---- non-blocking AT engine ----------------------------------------------
// atTick() sends `cmd` exactly once (on the first call, when no command is in flight),
// then on subsequent calls polls for `expect` / "ERROR" / timeout using a small (~20 ms)
// read budget and returns. So a long modem wait (e.g. the 45 s QMTOPEN TLS handshake)
// is spread across many loop() iterations instead of blocking the mesh loop in one call.
// On AT_OK the matching line is left in _line for the caller to parse.

BG77Modem::AtRes BG77Modem::atTick(const char* cmd, const char* expect, uint32_t timeout_ms) {
  if (!_at_active) {
    flushInput();
    if (cmd) sendAT(cmd);
    strncpy(_at_expect, expect ? expect : "OK", sizeof(_at_expect) - 1);
    _at_expect[sizeof(_at_expect) - 1] = 0;
    _at_deadline = millis() + timeout_ms;
    _at_active = true;
    return AT_BUSY;                       // response arrives on a later tick
  }
  unsigned long budget_end = millis() + 20;   // read a few lines, then yield back to loop()
  while ((long)(millis() - budget_end) < 0) {
    int n = readLine(6);
    if (n <= 0) break;                    // nothing buffered right now
    if (strstr(_line, _at_expect)) { _at_active = false; return AT_OK; }
    if (strstr(_line, "ERROR"))    { _at_active = false; return AT_FAIL; }
    // otherwise: command echo / unrelated URC -> keep polling
  }
  if ((long)(millis() - _at_deadline) >= 0) { _at_active = false; return AT_FAIL; }
  return AT_BUSY;
}

void BG77Modem::enterState(State s) {
  _state = s;
  _state_since = millis();
  _at_active = false;
  _phase_step = 0;
  _at_retry = 0;
  _mqtt_stage = 0;
}

// ---- power ----------------------------------------------------------------

void BG77Modem::powerPulse() {
  if (_power_en >= 0) { pinMode(_power_en, OUTPUT); digitalWrite(_power_en, HIGH); delay(100); }
  if (_pwrkey >= 0) {
    // BG77 PWRKEY: drive active for ~600 ms then release to boot the modem.
    pinMode(_pwrkey, OUTPUT);
    digitalWrite(_pwrkey, HIGH);
    delay(600);
    digitalWrite(_pwrkey, LOW);
  }
}

// ---- bring-up steps -------------------------------------------------------

BG77Modem::AtRes BG77Modem::configureModemTick() {
  char cmd[96];
  switch (_phase_step) {
    case 0: {  // modem alive? the modem may still be booting -> retry a few times
      AtRes r = atTick("AT", "OK", T_AT);
      if (r == AT_BUSY) return AT_BUSY;
      if (r == AT_OK)   { _phase_step = 1; _at_retry = 0; return AT_BUSY; }
      if (++_at_retry >= 6) return AT_FAIL;   // no modem
      return AT_BUSY;                          // atTick will re-send on the next call
    }
    // ATE0 / CMEE are best-effort: advance whether OK or ERROR.
    case 1: { AtRes r = atTick("ATE0", "OK", T_AT);      if (r == AT_BUSY) return AT_BUSY; _phase_step = 2; return AT_BUSY; }
    case 2: { AtRes r = atTick("AT+CMEE=2", "OK", T_AT); if (r == AT_BUSY) return AT_BUSY; _phase_step = 3; return AT_BUSY; }
    case 3: {  // SIM present? (must succeed)
      AtRes r = atTick("AT+CPIN?", "READY", T_CFG);
      if (r == AT_BUSY) return AT_BUSY;
      if (r == AT_FAIL) return AT_FAIL;
      _phase_step = 4; return AT_BUSY;
    }
    // LTE-M first (nwscanseq 020301), prefer LTE-M (iotopmode 0). Best-effort.
    case 4: { AtRes r = atTick("AT+QCFG=\"nwscanseq\",020301", "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 5; return AT_BUSY; }
    case 5: { AtRes r = atTick("AT+QCFG=\"iotopmode\",0,1", "OK", T_CFG);    if (r == AT_BUSY) return AT_BUSY; _phase_step = 6; return AT_BUSY; }
    case 6: {  // optional LTE-M band mask (2nd QCFG "band" field)
      if (!_band[0]) { _phase_step = 7; return AT_BUSY; }
      snprintf(cmd, sizeof(cmd), "AT+QCFG=\"band\",0,%s,0,1", _band);
      AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 7; return AT_BUSY;
    }
    case 7: {  // PDP context / APN
      snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", _apn);
      AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 8; return AT_BUSY;
    }
    case 8: {  // radio on
      AtRes r = atTick("AT+CFUN=1", "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; return AT_OK;
    }
  }
  return AT_OK;
}

BG77Modem::AtRes BG77Modem::pollRegistrationTick() {
  // Runs one CEREG then one CSQ, updating _cereg_stat/_registered/_rssi_dbm. Returns
  // AT_OK once the full cycle is done (caller inspects isRegistered()); AT_BUSY meanwhile.
  switch (_phase_step) {
    case 0: {  // +CEREG: <n>,<stat> — stat 1 (home) / 5 (roaming) = attached.
      AtRes r = atTick("AT+CEREG?", "+CEREG:", T_CFG);
      if (r == AT_BUSY) return AT_BUSY;
      if (r == AT_OK) {
        char* p = strstr(_line, "+CEREG:");
        char* comma = p ? strchr(p, ',') : nullptr;
        if (comma) { int stat = atoi(comma + 1); _cereg_stat = stat; _registered = (stat == 1 || stat == 5); }
      }
      _phase_step = 1; return AT_BUSY;      // advance whether parsed or timed out
    }
    case 1: {  // +CSQ: <rssi>,<ber>; rssi 0..31 -> -113..-51 dBm, 99=unknown.
      AtRes r = atTick("AT+CSQ", "+CSQ:", T_AT);
      if (r == AT_BUSY) return AT_BUSY;
      if (r == AT_OK) {
        char* p = strstr(_line, "+CSQ:");
        if (p) { int raw = atoi(p + 5); _rssi_dbm = (raw == 99) ? 0 : (-113 + 2 * raw); }
      }
      _phase_step = 0; return AT_OK;        // cycle complete
    }
  }
  return AT_OK;
}

// Convert a UTC calendar time to a Unix epoch (no libc/TZ dependency).
static uint32_t utc_to_epoch(int y, int mo, int d, int h, int mi, int s) {
  static const int mdays[] = {0,31,59,90,120,151,181,212,243,273,304,334};
  long days = (y - 1970) * 365L + (y - 1969) / 4 - (y - 1901) / 100 + (y - 1601) / 400;
  days += mdays[(mo - 1) % 12] + (d - 1);
  bool leap = ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0);
  if (leap && mo > 2) days += 1;
  return (uint32_t)(((days * 24L + h) * 60L + mi) * 60L + s);
}

BG77Modem::AtRes BG77Modem::syncNetworkTimeTick() {
  // AT+QLTS=2 -> +QLTS: "yyyy/mm/dd,hh:mm:ss+zz,d"  (GMT/UTC). Best-effort: on OK or FAIL
  // the caller proceeds either way (the RTC just stays unset if the network gave no time).
  AtRes r = atTick("AT+QLTS=2", "+QLTS:", T_CFG);
  if (r == AT_OK) {
    char* p = strstr(_line, "+QLTS:");
    char* q = p ? strchr(p, '"') : nullptr;
    if (q) {
      int Y, Mo, D, H, Mi, S;
      if (sscanf(q + 1, "%d/%d/%d,%d:%d:%d", &Y, &Mo, &D, &H, &Mi, &S) == 6 && Y > 2020) {
        uint32_t epoch = utc_to_epoch(Y, Mo, D, H, Mi, S);
        if (_time_cb) _time_cb(_time_ctx, epoch);
      }
    }
  }
  return r;   // AT_BUSY = keep ticking; AT_OK/AT_FAIL = done
}

BG77Modem::AtRes BG77Modem::setupTLSTick() {
  if (!_tls) return AT_OK;
  char cmd[96];
  // Bind an SSL context and configure it. seclevel: 0=none, 1=verify server, 2=mutual.
  switch (_phase_step) {
    case 0: { snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"sslversion\",%u,4", SSL_CTX_IDX);       // TLS 1.2
              AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 1; return AT_BUSY; }
    case 1: { snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"ciphersuite\",%u,0xFFFF", SSL_CTX_IDX);
              AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 2; return AT_BUSY; }
    case 2: {  // SNI = broker hostname in the ClientHello. REQUIRED for Traefik HostSNI routing;
               // without it QMTOPEN falls back to the default cert / no route -> open:timeout.
              snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"sni\",%u,1", SSL_CTX_IDX);
              AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 3; return AT_BUSY; }
    case 3: {  // upload-CA step only when verifying
              if (!_tls_verify) { _phase_step = 4; return AT_BUSY; }
              snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"cacert\",%u,\"%s\"", SSL_CTX_IDX, CA_FILE_NAME);
              AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 4; return AT_BUSY; }
    case 4: {  // seclevel: 1 = verify server cert, 0 = encrypt only
              if (_tls_verify) snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"seclevel\",%u,1", SSL_CTX_IDX);
              else             snprintf(cmd, sizeof(cmd), "AT+QSSLCFG=\"seclevel\",%u,0", SSL_CTX_IDX);
              AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; return AT_OK; }
  }
  return AT_OK;
}

BG77Modem::AtRes BG77Modem::openConnectTick() {
  char cmd[160];
  switch (_phase_step) {
    case 0: {  // Close any stale MQTT client FIRST. An nRF52 reboot/DFU does not power-cycle
               // the BG77, so a session opened before the reboot lingers and makes QMTOPEN
               // return open:2 ("identifier occupied"). QMTCLOSE frees it (ERROR if none = fine).
               // Also covers broker-drop -> reconnect, where the old client may be half-open.
      snprintf(cmd, sizeof(cmd), "AT+QMTCLOSE=%u", MQTT_CLIENT_IDX);
      AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 1; return AT_BUSY;
    }
    case 1: {  // bind MQTT client to the SSL context, or plain TCP if TLS off
      if (_tls) snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"ssl\",%u,1,%u", MQTT_CLIENT_IDX, SSL_CTX_IDX);
      else      snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"ssl\",%u,0", MQTT_CLIENT_IDX);
      AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 2; return AT_BUSY;
    }
    case 2: { snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"version\",%u,4", MQTT_CLIENT_IDX);        // MQTT 3.1.1
              AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 3; return AT_BUSY; }
    case 3: { snprintf(cmd, sizeof(cmd), "AT+QMTCFG=\"keepalive\",%u,%u", MQTT_CLIENT_IDX, _keepalive);
              AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 4; return AT_BUSY; }
    // Bring the TCP/IP data context up before opening the socket. When connecting by hostname,
    // QMTOPEN result 4 (= domain-name parse fail) means carrier DNS wasn't resolving; QICSGP +
    // QIACT activate context 1 and QIDNSCFG pins public DNS. Harmless when connecting by IP.
    case 4: { snprintf(cmd, sizeof(cmd), "AT+QICSGP=1,1,\"%s\",\"\",\"\",1", _apn);
              AtRes r = atTick(cmd, "OK", T_CFG); if (r == AT_BUSY) return AT_BUSY; _phase_step = 5; return AT_BUSY; }
    case 5: { AtRes r = atTick("AT+QIACT=1", "OK", 30000);   // idempotent; already-active returns ERROR (harmless)
              if (r == AT_BUSY) return AT_BUSY; _phase_step = 6; return AT_BUSY; }
    case 6: { AtRes r = atTick("AT+QIDNSCFG=1,\"8.8.8.8\",\"8.8.4.4\"", "OK", T_CFG);
              if (r == AT_BUSY) return AT_BUSY; _phase_step = 7; return AT_BUSY; }
    case 7: {  // open the socket to the broker. +QMTOPEN: <idx>,<ret> (ret 0 = ok)
      snprintf(cmd, sizeof(cmd), "AT+QMTOPEN=%u,\"%s\",%u", MQTT_CLIENT_IDX, _host, _port);
      AtRes r = atTick(cmd, "+QMTOPEN:", T_QMTOPEN);
      if (r == AT_BUSY) return AT_BUSY;
      if (r == AT_FAIL) { snprintf(_last_err, sizeof(_last_err), "open:timeout"); return AT_FAIL; }
      char* p = strstr(_line, "+QMTOPEN:");
      char* comma = p ? strchr(p, ',') : nullptr;
      int code = comma ? atoi(comma + 1) : -99;
      if (code != 0) { snprintf(_last_err, sizeof(_last_err), "open:%d", code); return AT_FAIL; }
      _phase_step = 8; return AT_BUSY;
    }
    case 8: {  // MQTT CONNECT. +QMTCONN: <idx>,<result>,<ret> (result 0 & ret 0 = accepted)
      const char* cid = _client_id[0] ? _client_id : "meshcore";
      if (_user[0]) snprintf(cmd, sizeof(cmd), "AT+QMTCONN=%u,\"%s\",\"%s\",\"%s\"", MQTT_CLIENT_IDX, cid, _user, _pass);
      else          snprintf(cmd, sizeof(cmd), "AT+QMTCONN=%u,\"%s\"", MQTT_CLIENT_IDX, cid);
      AtRes r = atTick(cmd, "+QMTCONN:", T_QMTCONN);
      if (r == AT_BUSY) return AT_BUSY;
      if (r == AT_FAIL) { snprintf(_last_err, sizeof(_last_err), "conn:timeout"); return AT_FAIL; }
      char* p = strstr(_line, "+QMTCONN:");
      char* c1 = p ? strchr(p, ',') : nullptr;      // -> ,<result>
      if (!c1) { snprintf(_last_err, sizeof(_last_err), "conn:noparse"); return AT_FAIL; }
      int result = atoi(c1 + 1);
      char* c2 = strchr(c1 + 1, ',');               // -> ,<ret_code>
      int ret = c2 ? atoi(c2 + 1) : -1;
      if (result != 0 || ret != 0) { snprintf(_last_err, sizeof(_last_err), "conn:%d/%d", result, ret); return AT_FAIL; }
      snprintf(_last_err, sizeof(_last_err), "ok");
      return AT_OK;
    }
  }
  return AT_OK;
}

void BG77Modem::enterBackoff(const char* why) {
  _backoff_step = (_backoff_step < 5) ? _backoff_step + 1 : 5;
  static const uint32_t table[] = {5000, 15000, 30000, 60000, 120000, 300000};
  _backoff_until = millis() + table[_backoff_step];
  _state = ST_BACKOFF;
  _state_since = millis();
  _at_active = false;          // abort any in-flight AT command; ST_INIT re-issues cleanly
  _phase_step = 0;
  if (_dbg) { _dbg->print("[BG77] backoff: "); _dbg->println(why ? why : "?"); }
}

// ---- lifecycle ------------------------------------------------------------

void BG77Modem::begin() {
  if (_status_pin >= 0) pinMode(_status_pin, INPUT);
  powerPulse();
  enterState(ST_POWERING);   // resets the tick/phase state as well
}

void BG77Modem::end() {
  if (_state == ST_READY) {
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+QMTDISC=%u", MQTT_CLIENT_IDX);
    sendExpect(cmd, "OK", T_CFG);
  }
  // Leave the modem powered (a re-begin is cheaper than a cold boot); just drop to OFF.
  _state = ST_OFF;
}

// Non-blocking: every case does at most one atTick() (a send, or a ~20 ms poll) and returns,
// so MyMesh::loop() keeps running and the LoRa RX / advert cadence is never starved — even
// while MQTT setup is failing and retrying. (The old blocking path stalled here for tens of
// seconds per cycle, which deafened the node and made it unadministrable over the mesh.)
void BG77Modem::loop() {
  switch (_state) {
    case ST_OFF:
      break;

    case ST_POWERING: {
      // Give the modem a moment to boot, then probe with AT until it answers (or give up).
      if ((millis() - _state_since) < 3000) break;
      AtRes r = atTick("AT", "OK", T_AT);
      if (r == AT_OK) enterState(ST_INIT);
      else if (r == AT_FAIL && (millis() - _state_since) > T_BOOT) enterBackoff("no boot");
      break;
    }

    case ST_INIT: {
      AtRes r = configureModemTick();
      if (r == AT_OK) { enterState(ST_NET_WAIT); _last_poll = 0; }
      else if (r == AT_FAIL) enterBackoff("init failed");
      break;
    }

    case ST_NET_WAIT: {
      // Keep ticking an in-progress CEREG/CSQ cycle; start a fresh one every REG_POLL_MS.
      bool in_progress = _at_active || _phase_step != 0;
      if (in_progress || (millis() - _last_poll) >= REG_POLL_MS) {
        AtRes r = pollRegistrationTick();
        if (r == AT_OK) {
          _last_poll = millis();
          if (_registered) { enterState(ST_TIME_SYNC); break; }
        }
      }
      // LTE-M cold attach can take several minutes; give it 5 before a full re-init.
      if ((millis() - _state_since) > 300000) enterBackoff("no registration");
      break;
    }

    case ST_TIME_SYNC: {
      AtRes r = syncNetworkTimeTick();   // best-effort; proceed on OK or FAIL
      if (r != AT_BUSY) enterState(ST_MQTT_SETUP);
      break;
    }

    case ST_MQTT_SETUP: {
      if (_mqtt_stage == 0) {            // TLS context config
        AtRes r = setupTLSTick();
        if (r == AT_OK) { _mqtt_stage = 1; _phase_step = 0; _at_active = false; }
        else if (r == AT_FAIL) enterBackoff("tls setup failed");
      } else {                           // socket open + MQTT connect
        AtRes r = openConnectTick();
        if (r == AT_OK) { _backoff_step = 0; enterState(ST_READY); _last_poll = millis(); }
        else if (r == AT_FAIL) enterBackoff("mqtt connect failed");
      }
      break;
    }

    case ST_READY: {
      // Watch for an async disconnect URC (+QMTSTAT: <idx>,<err>).
      int n = readLine(5);
      if (n > 0 && strstr(_line, "+QMTSTAT:")) enterBackoff("broker disconnect");
      break;
    }

    case ST_BACKOFF:
      if ((long)(millis() - _backoff_until) >= 0) enterState(ST_INIT);  // modem stays powered
      break;
  }
}

// ---- publish / provisioning ----------------------------------------------

// Parse the "+QMTPUB: <idx>,<msgid>,<result>" line in _line -> result code (or -9).
static int parseQmtpubResult(const char* line) {
  const char* p = strstr(line, "+QMTPUB:");
  const char* c1 = p ? strchr(p, ',') : nullptr;       // ,<msgid>
  const char* c2 = c1 ? strchr(c1 + 1, ',') : nullptr; // ,<result>
  return c2 ? atoi(c2 + 1) : -9;
}

bool BG77Modem::publish(const char* topic, const char* payload, size_t len, uint8_t qos, bool retain) {
  if (_state != ST_READY || !topic || !payload) { snprintf(_last_pub, sizeof(_last_pub), "notready"); return false; }

  char cmd[192];
  uint16_t mid = (qos == 0) ? 0 : _msg_id++;

  // Attempt 1: AT+QMTPUBEX=<idx>,<msgid>,<qos>,<retain>,"<topic>",<length> (length-delimited,
  // up to 4096 B). Some BG77 firmware revisions reject QMTPUBEX -> we fall back to QMTPUB below.
  snprintf(cmd, sizeof(cmd), "AT+QMTPUBEX=%u,%u,%u,%u,\"%s\",%u",
           MQTT_CLIENT_IDX, mid, qos, retain ? 1 : 0, topic, (unsigned)len);
  flushInput();
  sendAT(cmd);
  if (waitForPrompt(T_AT)) {                       // got the '>' data prompt
    _ser.write((const uint8_t*)payload, len);
    if (!waitFor("+QMTPUB:", T_PUB)) { snprintf(_last_pub, sizeof(_last_pub), "ex:noack"); return false; }
    int rc = parseQmtpubResult(_line);
    if (rc == 0) { snprintf(_last_pub, sizeof(_last_pub), "ex:ok"); return true; }
    snprintf(_last_pub, sizeof(_last_pub), "ex:rc%d", rc);
    return false;
  }

  // Attempt 2 (fallback): AT+QMTPUB (CTRL-Z-delimited, <=560 B). If QMTPUBEX gave no prompt,
  // the older QMTPUB may still be accepted.
  if (len <= 560) {
    mid = (qos == 0) ? 0 : _msg_id++;
    snprintf(cmd, sizeof(cmd), "AT+QMTPUB=%u,%u,%u,%u,\"%s\"",
             MQTT_CLIENT_IDX, mid, qos, retain ? 1 : 0, topic);
    flushInput();
    sendAT(cmd);
    if (!waitForPrompt(T_AT)) { snprintf(_last_pub, sizeof(_last_pub), "noprompt"); return false; }
    _ser.write((const uint8_t*)payload, len);
    _ser.write((uint8_t)0x1A);                     // CTRL-Z terminates QMTPUB data
    if (!waitFor("+QMTPUB:", T_PUB)) { snprintf(_last_pub, sizeof(_last_pub), "pub:noack"); return false; }
    int rc = parseQmtpubResult(_line);
    if (rc == 0) { snprintf(_last_pub, sizeof(_last_pub), "pub:ok"); return true; }
    snprintf(_last_pub, sizeof(_last_pub), "pub:rc%d", rc);
    return false;
  }

  snprintf(_last_pub, sizeof(_last_pub), "noprompt-big");
  return false;
}

// ---- GNSS ------------------------------------------------------------------

bool BG77Modem::gnssEnable() {
  // AT+QGPS=1 = standalone GNSS. If it's already on the modem returns an error; treat
  // that as success. Default WWAN priority is kept so GNSS acquires in LTE idle time
  // without disrupting the MQTT uplink (fine for a mostly-idle stationary observer).
  sendExpect("AT+QGPS=1", "OK", T_CFG);
  return true;
}

bool BG77Modem::gnssDisable() {
  return sendExpect("AT+QGPSEND", "OK", T_CFG);
}

bool BG77Modem::gnssGetFix(float& lat, float& lon) {
  // AT+QGPSLOC=2 -> +QGPSLOC: <utc>,<lat>,<lon>,<hdop>,<alt>,<fix>,...  (lat/lon are
  // signed decimal degrees in mode 2). While unfixed the modem returns +CME ERROR: 516.
  flushInput();
  sendAT("AT+QGPSLOC=2");
  bool got = false;
  unsigned long start = millis();
  while ((millis() - start) < T_CFG) {
    int n = readLine(T_CFG - (millis() - start));
    if (n < 0) break;
    char* p = strstr(_line, "+QGPSLOC:");
    if (p) {
      char* c = strchr(p, ',');          // skip the UTC field
      float la = 0, lo = 0;
      if (c && sscanf(c + 1, "%f,%f", &la, &lo) == 2) { lat = la; lon = lo; got = true; }
    }
    if (strstr(_line, "OK") || strstr(_line, "ERROR")) break;
  }
  return got;
}

bool BG77Modem::sendRawAT(const char* cmd, char* out, size_t out_size, uint32_t timeout_ms) {
  if (out && out_size) out[0] = 0;
  if (!cmd) return false;
  flushInput();
  sendAT(cmd);
  size_t used = 0;
  bool done = false, ok = false;
  unsigned long start = millis();
  while ((millis() - start) < timeout_ms && !done) {
    int n = readLine(timeout_ms - (millis() - start));
    if (n < 0) break;
    if (out && used + (size_t)n + 2 < out_size) {   // append "line\n"
      memcpy(out + used, _line, n);
      used += n;
      out[used++] = '\n';
      out[used] = 0;
    }
    if (strstr(_line, "OK")) { ok = true; done = true; }
    else if (strstr(_line, "ERROR")) { ok = false; done = true; }
  }
  return ok;
}

bool BG77Modem::uploadCACert(const char* name, const char* pem, size_t pem_len) {
  if (!name || !pem || pem_len == 0) return false;
  char cmd[64];
  // Remove any stale copy, then upload. AT+QFUPL=<name>,<len>[,<timeout>] -> "CONNECT" prompt.
  snprintf(cmd, sizeof(cmd), "AT+QFDEL=\"%s\"", name);
  sendExpect(cmd, "OK", T_CFG);   // ok if it didn't exist
  snprintf(cmd, sizeof(cmd), "AT+QFUPL=\"%s\",%u,60", name, (unsigned)pem_len);
  flushInput();
  sendAT(cmd);
  if (!waitFor("CONNECT", T_CFG)) return false;
  _ser.write((const uint8_t*)pem, pem_len);
  // Success line: +QFUPL: <uploaded_len>,<checksum>
  return waitFor("+QFUPL:", 20000);
}
