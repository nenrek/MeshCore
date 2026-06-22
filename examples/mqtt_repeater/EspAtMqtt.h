#pragma once
// ---------------------------------------------------------------------------
// EspAtMqtt — drive a RAK2305 (ESP32-WROVER running ESP-AT) over UART to
// publish MeshCore packet events to an MQTT broker.
//
// WHY this exists (see memory project_mqtt_repeater): the RAK3401's internal
// SX1262 owns both chip-select lines the RAK13800 could use, so on-board
// Ethernet (SPI) is impossible on that Core. The RAK2305 talks over UART1
// instead — no SPI, no CS collision — and its ESP-AT firmware can do
// MQTT-over-TLS *and* MQTT-over-WebSocket-Secure (scheme 7-10), so the ESP32
// carries the whole TLS/WSS stack the nRF52 can't fit. The nRF52 only ever
// emits AT command strings on Serial1 (P0.15 RX / P0.16 TX on the RAK3401).
//
// DESIGN — non-blocking. A repeater must never stall LoRa RX, but an AT round
// trip over WiFi/WSS can take hundreds of ms. So the packet hook only ENQUEUES
// a pre-formatted JSON record (microseconds); loop() drains the queue through a
// non-blocking AT state machine, one publish at a time, with a '>' prompt /
// +MQTTPUB:OK handshake (AT+MQTTPUBRAW — raw bytes, no JSON escaping needed).
//
// Topic / payload layout matches Cisien/meshcoretomqtt (LetsMesh observers):
//   <prefix>/<IATA>/<PUBKEY>/status    retained "online"/"offline" (+ LWT)
//   <prefix>/<IATA>/<PUBKEY>/packets   one JSON event per unique RX packet
//   <prefix>/<IATA>/<PUBKEY>/debug     periodic heartbeat / "mqtt test"
//
// Persistence: /mqtt.cfg   CLI: handleCommand() at the bottom.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <RTClib.h>
#include <Mesh.h>
#include <target.h>     // global radio_driver (getLastRSSI), board, rtc_clock

#ifndef MQTT_AT_BAUD
  #define MQTT_AT_BAUD            115200      // RAK2305 ESP-AT default
#endif
#ifndef MQTT_DEFAULT_PORT
  #define MQTT_DEFAULT_PORT       443         // LetsMesh WSS endpoint
#endif
#ifndef MQTT_DEFAULT_SCHEME
  // 1=TCP 2-5=TLS 6=WS 7=WSS(no verify) 8=WSS(verify CA) ...
  #define MQTT_DEFAULT_SCHEME     7
#endif
#ifndef MQTT_KEEPALIVE_S
  #define MQTT_KEEPALIVE_S        120
#endif
#ifndef MQTT_HEARTBEAT_MS
  #define MQTT_HEARTBEAT_MS       60000UL
#endif
#ifndef MQTT_BACKOFF_MIN_MS
  #define MQTT_BACKOFF_MIN_MS     5000UL
#endif
#ifndef MQTT_BACKOFF_MAX_MS
  #define MQTT_BACKOFF_MAX_MS     60000UL
#endif
#ifndef MQTT_MAX_PAYLOAD
  // Must hold a full packet's `raw` hex (up to 2*MAX_TRANS_UNIT = 510 chars)
  // plus the surrounding meshcoretomqtt JSON envelope (~280 chars).
  #define MQTT_MAX_PAYLOAD        1024
#endif
#ifndef MQTT_QUEUE_LEN
  #define MQTT_QUEUE_LEN          8
#endif
#define MQTT_RX_BUF               320

// Leaf topics
#define LEAF_STATUS   0
#define LEAF_PACKETS  1
#define LEAF_DEBUG    2

class EspAtMqtt {
  Stream*      _at;          // UART to the RAK2305 (Serial1)
  FILESYSTEM*  _fs;
  mesh::RTCClock* _rtc;

  // ---- persisted config (/mqtt.cfg) ----
  bool     _enabled;
  char     _ssid[33];
  char     _wifi_pass[64];
  char     _host[80];
  uint16_t _port;
  uint8_t  _scheme;
  char     _user[64];
  char     _pass[64];
  char     _ws_path[40];     // websocket resource path (scheme >= 6)
  char     _prefix[24];      // default "meshcore"
  char     _iata[8];         // 3-letter location code

  // ---- runtime identity ----
  const char* _node_name;
  char     _pubkey_hex[2 * PUB_KEY_SIZE + 1];
  char     _client_id[40];

  // ---- connection state machine ----
  enum CState { CS_OFF, CS_BRINGUP, CS_ONLINE, CS_BACKOFF };
  CState   _cstate;
  uint8_t  _step;
  bool     _step_sent;
  unsigned long _step_deadline;
  unsigned long _backoff_until;
  uint32_t _backoff_ms;

  // ---- publish sub-state ----
  enum PState { PUB_IDLE, PUB_WAITPROMPT, PUB_WAITRESULT };
  PState   _pstate;
  unsigned long _pub_deadline;
  unsigned long _next_heartbeat;

  // ---- AT rx accumulator ----
  char     _rx[MQTT_RX_BUF];
  int      _rxlen;
  bool     _verbose;
  bool     _wifi_got_ip;
  bool     _hw_seen;          // any byte ever received from the ESP-AT module
  bool     _no_hw;            // gave up — no RAK2305 detected, repeater-only
  uint8_t  _bringup_fails;    // consecutive failed bring-up cycles

  // ---- publish queue (ring) ----
  struct PubItem { uint8_t leaf; bool retain; uint16_t len; char payload[MQTT_MAX_PAYLOAD]; };
  PubItem  _q[MQTT_QUEUE_LEN];
  uint8_t  _qhead, _qtail, _qcount;

  // ---- counters ----
  uint32_t _pub_ok, _pub_fail, _pkts_seen, _q_drops, _conn_attempts;

  // bring-up steps
  enum { ST_ATE0, ST_CWMODE, ST_CWJAP, ST_USERCFG, ST_CONNCFG, ST_CONN, ST_PUBSTATUS, ST_DONE };

  /* ===================== small helpers ===================== */
  static char hexNib(uint8_t n) { return n < 10 ? ('0' + n) : ('A' + n - 10); }
  static void toHex(char* dst, const uint8_t* src, int len) {
    for (int i = 0; i < len; i++) { dst[2*i] = hexNib(src[i] >> 4); dst[2*i+1] = hexNib(src[i] & 0xF); }
    dst[2*len] = 0;
  }
  void clearRx() { _rxlen = 0; _rx[0] = 0; }
  bool seen(const char* tok) { return strstr(_rx, tok) != nullptr; }
  bool sawOK()    { return seen("OK\r\n"); }
  bool sawError() { return seen("ERROR"); }

  void buildClientId() {
    // <prefix>_<first 3 bytes of pubkey>
    snprintf(_client_id, sizeof(_client_id), "%s_%c%c%c%c%c%c",
             _prefix[0] ? _prefix : "meshcore",
             _pubkey_hex[0], _pubkey_hex[1], _pubkey_hex[2],
             _pubkey_hex[3], _pubkey_hex[4], _pubkey_hex[5]);
  }

  // The RAK2305 ESP-AT build (ESP-IDF v4.0) caps MQTT topic strings at 64 chars
  // (measured: 64 OK, 66 ERROR). The full 64-hex pubkey overflows that, so the
  // key segment is truncated to fit. Budget is computed against the LONGEST leaf
  // ("packets"=7) so the key is identical across status/packets/debug — the map
  // correlates an observer's topics by that key.
  #define MQTT_TOPIC_MAX 64
  void topicFor(uint8_t leaf, char* buf, int n) {
    const char* l = (leaf == LEAF_STATUS) ? "status" : (leaf == LEAF_PACKETS) ? "packets" : "debug";
    const char* prefix = _prefix[0] ? _prefix : "meshcore";
    const char* iata   = _iata[0]   ? _iata   : "XXX";
    const char* key    = _pubkey_hex[0] ? _pubkey_hex : "00";
    const int LONGEST_LEAF = 7;  // "packets"
    int fixed  = (int)strlen(prefix) + 1 + (int)strlen(iata) + 1 + 1 + LONGEST_LEAF;
    int keymax = MQTT_TOPIC_MAX - fixed; if (keymax < 0) keymax = 0;
    int keylen = (int)strlen(key);   if (keylen > keymax) keylen = keymax;
    snprintf(buf, n, "%s/%s/%.*s/%s", prefix, iata, keylen, key, l);
  }

  int isoTime(char* buf, int n) {
    uint32_t now = _rtc ? _rtc->getCurrentTime() : 0;
    DateTime dt(now);
    return snprintf(buf, n, "%04u-%02u-%02uT%02u:%02u:%02uZ",
                    dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
  }

  // send "<cmd>\r\n" to the ESP-AT and reset the rx accumulator
  void sendLine(const char* cmd) {
    if (_verbose) { Serial.print("[AT>] "); Serial.println(cmd); }
    _at->print(cmd);
    _at->print("\r\n");
    clearRx();
  }

  /* ===================== queue ===================== */
  bool enqueue(uint8_t leaf, const char* payload, bool retain) {
    if (_qcount >= MQTT_QUEUE_LEN) { _q_drops++; return false; }
    PubItem& it = _q[_qhead];
    it.leaf = leaf; it.retain = retain;
    strncpy(it.payload, payload, MQTT_MAX_PAYLOAD - 1);
    it.payload[MQTT_MAX_PAYLOAD - 1] = 0;
    it.len = (uint16_t)strlen(it.payload);
    _qhead = (_qhead + 1) % MQTT_QUEUE_LEN;
    _qcount++;
    return true;
  }
  PubItem* qfront() { return _qcount ? &_q[_qtail] : nullptr; }
  void qpop() { if (_qcount) { _qtail = (_qtail + 1) % MQTT_QUEUE_LEN; _qcount--; } }

  /* ===================== AT rx pump ===================== */
  void pump() {
    while (_at->available()) {
      char c = (char)_at->read();
      _hw_seen = true;   // something is answering on the UART → a module is present
      if (_verbose) Serial.write(c);
      if (_rxlen >= MQTT_RX_BUF - 1) {
        // keep the tail half so long responses / prompts aren't lost
        memmove(_rx, _rx + MQTT_RX_BUF / 2, MQTT_RX_BUF / 2);
        _rxlen = MQTT_RX_BUF / 2;
      }
      _rx[_rxlen++] = c;
      _rx[_rxlen] = 0;
    }
  }

  /* ===================== bring-up ===================== */
  void issueStep(unsigned long now) {
    char cmd[200];
    char topic[140];
    unsigned long to = 5000;  // default per-step timeout
    switch (_step) {
      case ST_ATE0:    strcpy(cmd, "ATE0"); to = 2000; break;
      case ST_CWMODE:  strcpy(cmd, "AT+CWMODE=1"); break;
      case ST_CWJAP:
        snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", _ssid, _wifi_pass);
        to = 20000;  // association + DHCP
        _wifi_got_ip = false;
        break;
      case ST_USERCFG:
        buildClientId();
        snprintf(cmd, sizeof(cmd),
                 "AT+MQTTUSERCFG=0,%u,\"%s\",\"%s\",\"%s\",0,0,\"%s\"",
                 (unsigned)_scheme, _client_id, _user, _pass, _ws_path);
        break;
      case ST_CONNCFG:
        topicFor(LEAF_STATUS, topic, sizeof(topic));
        // LWT: broker publishes "offline" retained if we drop unexpectedly
        snprintf(cmd, sizeof(cmd),
                 "AT+MQTTCONNCFG=0,%u,0,\"%s\",\"offline\",0,1",
                 (unsigned)MQTT_KEEPALIVE_S, topic);
        break;
      case ST_CONN:
        snprintf(cmd, sizeof(cmd),
                 "AT+MQTTCONN=0,\"%s\",%u,1", _host, (unsigned)_port);
        to = 20000;  // TLS/WSS handshake
        _conn_attempts++;
        break;
      default: return;
    }
    sendLine(cmd);
    _step_sent = true;
    _step_deadline = now + to;
  }

  void advanceStep() { _step++; _step_sent = false; }

  void failBringup(const char* why) {
    Serial.printf("[MQTT] bring-up failed at step %u (%s)\n", _step, why);
    // If the module never answered a single byte across a couple of cycles,
    // there's no RAK2305 attached — stop hammering the UART and let the board
    // run as a plain repeater. 'mqtt retry' (or 'mqtt on') re-probes.
    if (!_hw_seen && ++_bringup_fails >= 2) {
      Serial.println("[MQTT] no RAK2305 detected on UART1 — running repeater-only. Use 'mqtt retry' to re-probe.");
      _no_hw = true;
      _cstate = CS_BACKOFF; _backoff_until = millis() + 0xFFFFFFF;  // dormant until retry
      _step = ST_ATE0; _step_sent = false;
      return;
    }
    if (_backoff_ms < MQTT_BACKOFF_MAX_MS) _backoff_ms *= 2;
    _backoff_until = millis() + _backoff_ms;
    _cstate = CS_BACKOFF;
    _step = ST_ATE0; _step_sent = false;
  }

  void serviceBringup(unsigned long now) {
    // ST_PUBSTATUS is a virtual finalize step that sends no AT command, so it
    // must run BEFORE the issueStep guard below: issueStep() has no case for it
    // and its default-return never sets _step_sent, which would otherwise loop
    // here forever — connected to the broker but never transitioning to online.
    // (Latent until the first successful MQTTCONN.)
    if (_step == ST_PUBSTATUS) {
      _backoff_ms = MQTT_BACKOFF_MIN_MS;
      enqueue(LEAF_STATUS, "online", true);   // retained "online"
      _next_heartbeat = now + 2000;
      _pstate = PUB_IDLE;
      _cstate = CS_ONLINE;
      Serial.printf("[MQTT] online: %s:%u scheme=%u as %s\n",
                    _host, (unsigned)_port, (unsigned)_scheme, _client_id);
      return;
    }
    if (!_step_sent) { issueStep(now); return; }
    bool timeout = (long)(now - _step_deadline) >= 0;

    switch (_step) {
      case ST_ATE0:   // tolerate failure (echo may already be off / ESP booting)
        if (sawOK() || sawError() || timeout) advanceStep();
        break;
      case ST_CWMODE:
        if (sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("CWMODE");
        break;
      case ST_CWJAP:
        if (seen("WIFI GOT IP")) _wifi_got_ip = true;
        if (_wifi_got_ip && sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("CWJAP (wifi?)");
        break;
      case ST_USERCFG:
        if (sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("MQTTUSERCFG");
        break;
      case ST_CONNCFG:
        if (sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("MQTTCONNCFG");
        break;
      case ST_CONN:
        // ESP-AT can emit "+MQTTCONNECTED" then "ERROR" (e.g. CONNECT URC plus a
        // benign command-level error); treat the URC as authoritative success.
        if (seen("+MQTTCONNECTED") || sawOK()) advanceStep();
        else if (sawError() || timeout) failBringup("MQTTCONN (broker?)");
        break;
      // ST_PUBSTATUS handled at top of serviceBringup (no AT command).
    }
  }

  /* ===================== steady-state publish ===================== */
  void serviceOnline(unsigned long now) {
    if (seen("+MQTTDISCONNECTED")) { failBringup("disconnected"); return; }

    switch (_pstate) {
      case PUB_IDLE: {
        PubItem* it = qfront();
        if (!it) {  // nothing queued — heartbeat if due
          if ((long)(now - _next_heartbeat) >= 0) {
            queueHeartbeat();
            _next_heartbeat = now + MQTT_HEARTBEAT_MS;
          }
          return;
        }
        char topic[140];
        topicFor(it->leaf, topic, sizeof(topic));
        char cmd[200];
        snprintf(cmd, sizeof(cmd), "AT+MQTTPUBRAW=0,\"%s\",%u,0,%u",
                 topic, (unsigned)it->len, it->retain ? 1u : 0u);
        sendLine(cmd);
        _pstate = PUB_WAITPROMPT;
        _pub_deadline = now + 8000;
        break;
      }
      case PUB_WAITPROMPT: {
        if (seen(">")) {
          PubItem* it = qfront();
          if (it) { _at->write((const uint8_t*)it->payload, it->len);  // raw, no CRLF
                    if (_verbose) { Serial.print("[AT>] <"); Serial.print(it->len); Serial.println(" raw bytes>"); } }
          clearRx();
          _pstate = PUB_WAITRESULT;
          _pub_deadline = now + 10000;
        } else if (sawError() || (long)(now - _pub_deadline) >= 0) {
          qpop(); _pub_fail++; _pstate = PUB_IDLE;
        }
        break;
      }
      case PUB_WAITRESULT: {
        if (seen("+MQTTPUB:OK"))        { qpop(); _pub_ok++;   _pstate = PUB_IDLE; }
        else if (seen("+MQTTPUB:FAIL")) { qpop(); _pub_fail++; _pstate = PUB_IDLE; }
        else if ((long)(now - _pub_deadline) >= 0) { qpop(); _pub_fail++; _pstate = PUB_IDLE; }
        break;
      }
    }
  }

  void queueHeartbeat() {
    char ts[40]; isoTime(ts, sizeof(ts));
    static char p[MQTT_MAX_PAYLOAD];  // 1KB — keep off the stack
    snprintf(p, sizeof(p),
      "{\"origin\":\"%s\",\"origin_id\":\"%s\",\"timestamp\":\"%s\","
      "\"type\":\"HEARTBEAT\",\"uptime_s\":%lu,\"pkts\":%lu,"
      "\"pub_ok\":%lu,\"pub_fail\":%lu,\"q_drops\":%lu}",
      _node_name ? _node_name : "node", _pubkey_hex, ts,
      (unsigned long)(millis() / 1000), (unsigned long)_pkts_seen,
      (unsigned long)_pub_ok, (unsigned long)_pub_fail, (unsigned long)_q_drops);
    enqueue(LEAF_DEBUG, p, false);
  }

public:
  EspAtMqtt()
    : _at(nullptr), _fs(nullptr), _rtc(nullptr),
      _enabled(false), _port(MQTT_DEFAULT_PORT), _scheme(MQTT_DEFAULT_SCHEME),
      _node_name(nullptr), _cstate(CS_OFF), _step(ST_ATE0), _step_sent(false),
      _step_deadline(0), _backoff_until(0), _backoff_ms(MQTT_BACKOFF_MIN_MS),
      _pstate(PUB_IDLE), _pub_deadline(0), _next_heartbeat(0),
      _rxlen(0), _verbose(false), _wifi_got_ip(false),
      _hw_seen(false), _no_hw(false), _bringup_fails(0),
      _qhead(0), _qtail(0), _qcount(0),
      _pub_ok(0), _pub_fail(0), _pkts_seen(0), _q_drops(0), _conn_attempts(0)
  {
    _ssid[0] = _wifi_pass[0] = _host[0] = _user[0] = _pass[0] = 0;
    _ws_path[0] = _iata[0] = _pubkey_hex[0] = _client_id[0] = 0;
    strncpy(_prefix, "meshcore", sizeof(_prefix));
  }

  void setStream(Stream* s)        { _at = s; }
  void setRTC(mesh::RTCClock* r)   { _rtc = r; }
  void setNodeName(const char* n)  { _node_name = n; }
  void setPubKey(const uint8_t* k, int len) {
    int n = len > PUB_KEY_SIZE ? PUB_KEY_SIZE : len;
    toHex(_pubkey_hex, k, n);
  }
  bool isEnabled()  const { return _enabled; }
  bool isOnline()   const { return _cstate == CS_ONLINE; }

  /* ===================== persistence ===================== */
  void load(FILESYSTEM* fs) {
    _fs = fs;
    if (!_fs || !_fs->exists("/mqtt.cfg")) return;
    File f = _fs->open("/mqtt.cfg");
    if (!f) return;
    char buf[512];
    int len = f.read((uint8_t*)buf, sizeof(buf) - 1);
    f.close();
    if (len <= 0) return;
    buf[len] = 0;
    auto val = [](const char* src, const char* key, char* dst, int dn) {
      const char* p = strstr(src, key); if (!p) return;
      p += strlen(key); int i = 0;
      while (*p && *p != '\n' && *p != '\r' && i < dn - 1) dst[i++] = *p++;
      dst[i] = 0;
    };
    char tmp[16];
    tmp[0] = 0;  // val() leaves dst untouched when the key is missing
    val(buf, "enabled=", tmp, sizeof(tmp)); if (tmp[0]) _enabled = atoi(tmp);
    val(buf, "ssid=",    _ssid, sizeof(_ssid));
    val(buf, "wpass=",   _wifi_pass, sizeof(_wifi_pass));
    val(buf, "host=",    _host, sizeof(_host));
    tmp[0] = 0;
    val(buf, "port=",    tmp, sizeof(tmp)); if (tmp[0]) _port = (uint16_t)atoi(tmp);
    tmp[0] = 0;
    val(buf, "scheme=",  tmp, sizeof(tmp)); if (tmp[0]) _scheme = (uint8_t)atoi(tmp);
    val(buf, "user=",    _user, sizeof(_user));
    val(buf, "mpass=",   _pass, sizeof(_pass));
    val(buf, "wspath=",  _ws_path, sizeof(_ws_path));
    val(buf, "prefix=",  _prefix, sizeof(_prefix));
    val(buf, "iata=",    _iata, sizeof(_iata));
    Serial.printf("[MQTT] cfg: en=%d ssid=%s host=%s:%u scheme=%u iata=%s\n",
                  (int)_enabled, _ssid, _host, (unsigned)_port, (unsigned)_scheme,
                  _iata[0] ? _iata : "(unset)");
  }

  void save() {
    if (!_fs) { Serial.println("[MQTT] save FAILED: no filesystem"); return; }
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    _fs->remove("/mqtt.cfg");
    File f = _fs->open("/mqtt.cfg", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = _fs->open("/mqtt.cfg", "w");
#else
    File f = _fs->open("/mqtt.cfg", "w", true);
#endif
    if (!f) { Serial.println("[MQTT] save FAILED: open /mqtt.cfg"); return; }
    char buf[512];
    int n = snprintf(buf, sizeof(buf),
      "enabled=%d\nssid=%s\nwpass=%s\nhost=%s\nport=%u\nscheme=%u\n"
      "user=%s\nmpass=%s\nwspath=%s\nprefix=%s\niata=%s\n",
      (int)_enabled, _ssid, _wifi_pass, _host, (unsigned)_port, (unsigned)_scheme,
      _user, _pass, _ws_path, _prefix, _iata);
    int w = f.write((const uint8_t*)buf, n);
    f.close();
    if (w != n) Serial.printf("[MQTT] save FAILED: short write %d/%d\n", w, n);
  }

  /* ===================== packet hook (fast, non-blocking) ===================== */
  // Called once per unique received packet from MqttMeshTables::hasSeen().
  // Only formats + enqueues — never touches the UART.
  void onPacketReceived(const mesh::Packet* pkt) {
    if (!_enabled || !pkt) return;
    _pkts_seen++;
    if (_cstate != CS_ONLINE && _qcount >= MQTT_QUEUE_LEN) return;  // avoid churn while offline

    // The full over-the-air frame, hex-encoded, is the `raw` field the map
    // decoders (meshcore-decoder / meshcore-go) need to recover node identity,
    // position, adverts and routes. Without it the packet cannot be decoded.
    // Buffers are static (single-threaded packet hook) to keep this off the stack.
    static uint8_t raw[MAX_TRANS_UNIT];
    static char    raw_hex[2 * MAX_TRANS_UNIT + 1];
    static char    p[MQTT_MAX_PAYLOAD];
    uint8_t raw_len = pkt->writeTo(raw);
    toHex(raw_hex, raw, raw_len);

    uint8_t hash[MAX_HASH_SIZE];
    pkt->calculatePacketHash(hash);
    char hash_hex[2 * MAX_HASH_SIZE + 1];
    toHex(hash_hex, hash, MAX_HASH_SIZE);

    char ts[40]; isoTime(ts, sizeof(ts));
    int rssi = (int)radio_driver.getLastRSSI();
    float snr = pkt->getSNR();

    // meshcoretomqtt envelope — matches meshmapper / letsmesh / beacon ingest.
    snprintf(p, sizeof(p),
      "{\"origin\":\"%s\",\"origin_id\":\"%s\",\"timestamp\":\"%s\","
      "\"type\":\"PACKET\",\"direction\":\"rx\",\"raw\":\"%s\","
      "\"len\":%u,\"payload_len\":%u,\"packet_type\":%u,\"route\":\"%s\","
      "\"hash\":\"%s\",\"SNR\":%.2f,\"RSSI\":%d}",
      _node_name ? _node_name : "node", _pubkey_hex, ts, raw_hex,
      (unsigned)raw_len, (unsigned)pkt->payload_len, (unsigned)pkt->getPayloadType(),
      pkt->isRouteFlood() ? "F" : "D", hash_hex, (double)snr, rssi);
    enqueue(LEAF_PACKETS, p, false);
  }

  /* ===================== main loop ===================== */
  void loop() {
    if (!_enabled || !_at || _no_hw) return;
    pump();
    unsigned long now = millis();
    switch (_cstate) {
      case CS_OFF:     _step = ST_ATE0; _step_sent = false; _wifi_got_ip = false;
                       _cstate = CS_BRINGUP; break;
      case CS_BRINGUP: serviceBringup(now); break;
      case CS_ONLINE:  serviceOnline(now); break;
      case CS_BACKOFF: if ((long)(now - _backoff_until) >= 0) _cstate = CS_OFF; break;
    }
  }

  /* ===================== CLI ===================== */
  // Returns true if the command was one of ours.
  bool handleCommand(const char* cmd, char* reply) {
    if (strcmp(cmd, "mqtt status") == 0) {
      const char* st = _no_hw ? "no-2305" :
                       _cstate == CS_ONLINE ? "online" :
                       _cstate == CS_BRINGUP ? "connecting" :
                       _cstate == CS_BACKOFF ? "backoff" : "off";
      snprintf(reply, 160,
        "%s [%s] ssid=%s %s:%u scheme=%u iata=%s q=%u/%u pkts=%lu ok=%lu fail=%lu drop=%lu",
        _enabled ? "ON" : "OFF", st, _ssid[0] ? _ssid : "?", _host[0] ? _host : "?",
        (unsigned)_port, (unsigned)_scheme, _iata[0] ? _iata : "?",
        (unsigned)_qcount, (unsigned)MQTT_QUEUE_LEN,
        (unsigned long)_pkts_seen, (unsigned long)_pub_ok, (unsigned long)_pub_fail,
        (unsigned long)_q_drops);
      return true;
    }
    if (strcmp(cmd, "mqtt cfg") == 0) {  // debug: ground truth on persistence
      if (!_fs) { strcpy(reply, "no filesystem"); return true; }
      if (!_fs->exists("/mqtt.cfg")) { strcpy(reply, "/mqtt.cfg MISSING"); return true; }
      File f = _fs->open("/mqtt.cfg");
      if (!f) { strcpy(reply, "/mqtt.cfg exists but open failed"); return true; }
      char buf[512];
      int len = f.read((uint8_t*)buf, sizeof(buf) - 1);
      f.close();
      if (len < 0) len = 0;
      buf[len] = 0;
      Serial.printf("[MQTT] /mqtt.cfg (%d bytes):\n%s", len, buf);
      snprintf(reply, 160, "%d bytes", len);
      return true;
    }
    if (strncmp(cmd, "mqtt at ", 8) == 0) {  // debug: raw AT passthrough, blocks ~3s
      if (!_at) { strcpy(reply, "no UART"); return true; }
      while (_at->available()) _at->read();   // drop stale rx
      _at->print(cmd + 8); _at->print("\r\n");
      unsigned long until = millis() + 3000;
      int n = 0;
      while ((long)(millis() - until) < 0) {
        while (_at->available()) {
          char c = (char)_at->read();
          Serial.write(c);
          n++; _hw_seen = true;
        }
      }
      snprintf(reply, 160, "%s%d bytes in 3s", n ? "\n  -> " : "", n);
      return true;
    }
    if (strcmp(cmd, "mqtt on") == 0)  { _enabled = true;  save();
                                        _no_hw = false; _hw_seen = false; _bringup_fails = 0;
                                        _backoff_ms = MQTT_BACKOFF_MIN_MS; _cstate = CS_OFF;
                                        strcpy(reply, "MQTT enabled"); return true; }
    if (strcmp(cmd, "mqtt retry") == 0){ _no_hw = false; _hw_seen = false; _bringup_fails = 0;
                                        _backoff_ms = MQTT_BACKOFF_MIN_MS; _cstate = CS_OFF;
                                        strcpy(reply, _enabled ? "Re-probing RAK2305..." : "MQTT is off (use 'mqtt on')"); return true; }
    if (strcmp(cmd, "mqtt off") == 0) { _enabled = false; save();
                                        if (_at) sendLine("AT+MQTTCLEAN=0");
                                        _cstate = CS_OFF; strcpy(reply, "MQTT disabled"); return true; }
    if (strncmp(cmd, "wifi ssid ", 10) == 0) { strncpy(_ssid, cmd + 10, sizeof(_ssid)-1); _ssid[sizeof(_ssid)-1]=0; save(); snprintf(reply,160,"SSID: %s",_ssid); return true; }
    if (strncmp(cmd, "wifi pass ", 10) == 0) { strncpy(_wifi_pass, cmd + 10, sizeof(_wifi_pass)-1); _wifi_pass[sizeof(_wifi_pass)-1]=0; save(); strcpy(reply,"WiFi password set"); return true; }
    if (strncmp(cmd, "mqtt host ", 10) == 0) { strncpy(_host, cmd + 10, sizeof(_host)-1); _host[sizeof(_host)-1]=0; save(); snprintf(reply,160,"Host: %s",_host); return true; }
    if (strncmp(cmd, "mqtt port ", 10) == 0) { _port = (uint16_t)atoi(cmd + 10); save(); snprintf(reply,160,"Port: %u",(unsigned)_port); return true; }
    if (strncmp(cmd, "mqtt scheme ", 12) == 0){ _scheme = (uint8_t)atoi(cmd + 12); save(); snprintf(reply,160,"Scheme: %u (1=TCP 7/8=WSS)",(unsigned)_scheme); return true; }
    if (strncmp(cmd, "mqtt user ", 10) == 0) { strncpy(_user, cmd + 10, sizeof(_user)-1); _user[sizeof(_user)-1]=0; save(); snprintf(reply,160,"User: %s",_user); return true; }
    if (strncmp(cmd, "mqtt mpass ", 11) == 0) { strncpy(_pass, cmd + 11, sizeof(_pass)-1); _pass[sizeof(_pass)-1]=0; save(); strcpy(reply,"MQTT password set"); return true; }
    if (strncmp(cmd, "mqtt path ", 10) == 0) { strncpy(_ws_path, cmd + 10, sizeof(_ws_path)-1); _ws_path[sizeof(_ws_path)-1]=0; save(); snprintf(reply,160,"WS path: %s",_ws_path); return true; }
    if (strncmp(cmd, "mqtt prefix ", 12) == 0){ strncpy(_prefix, cmd + 12, sizeof(_prefix)-1); _prefix[sizeof(_prefix)-1]=0; save(); snprintf(reply,160,"Prefix: %s",_prefix); return true; }
    if (strncmp(cmd, "mqtt iata ", 10) == 0) { strncpy(_iata, cmd + 10, sizeof(_iata)-1); _iata[sizeof(_iata)-1]=0; for(char*p=_iata;*p;p++)*p=toupper((int)*p); save(); snprintf(reply,160,"IATA: %s",_iata); return true; }
    if (strcmp(cmd, "mqtt pubkey") == 0)    { snprintf(reply,160,"PubKey: %s",_pubkey_hex); return true; }
    if (strcmp(cmd, "mqtt verbose on") == 0){ _verbose = true;  strcpy(reply,"AT trace ON"); return true; }
    if (strcmp(cmd, "mqtt verbose off")== 0){ _verbose = false; strcpy(reply,"AT trace OFF"); return true; }
    if (strcmp(cmd, "mqtt test") == 0) {
      char ts[40]; isoTime(ts, sizeof(ts));
      static char p[MQTT_MAX_PAYLOAD];
      snprintf(p, sizeof(p), "{\"origin\":\"%s\",\"origin_id\":\"%s\",\"timestamp\":\"%s\",\"type\":\"TEST\"}",
               _node_name ? _node_name : "node", _pubkey_hex, ts);
      bool ok = enqueue(LEAF_DEBUG, p, false);
      strcpy(reply, ok ? "Test queued" : "Queue full");
      return true;
    }
    if (strcmp(cmd, "mqtt help") == 0) {
      Serial.println("--- MQTT/WiFi (RAK2305 ESP-AT) ---");
      Serial.println("wifi ssid <s> / wifi pass <p>");
      Serial.println("mqtt host <h> / port <n> / scheme <1|7|8>");
      Serial.println("mqtt user <u> / mpass <p> / path <wspath>");
      Serial.println("mqtt prefix <p> / iata <CODE> / pubkey");
      Serial.println("mqtt on / off / retry / status / test / verbose on|off");
      Serial.println("debug: mqtt cfg / mqtt at <raw AT cmd> / mqtt baud <n>  (use with mqtt off)");
      strcpy(reply, "See Serial output");
      return true;
    }
    return false;
  }
};
