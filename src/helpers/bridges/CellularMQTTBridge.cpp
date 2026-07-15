#include "CellularMQTTBridge.h"

#ifdef WITH_CELLULAR_MQTT_BRIDGE

#include <math.h>
#include <string.h>

// Variant-provided wiring (see variants/rak3401_cellular). Defaults keep this compilable
// even if a variant forgets a define; the real pins come from build flags.
#ifndef CELLULAR_UART
  #define CELLULAR_UART Serial1
#endif
#ifndef CELLULAR_UART_BAUD
  #define CELLULAR_UART_BAUD 115200
#endif
#ifndef CELLULAR_PWRKEY_PIN
  #define CELLULAR_PWRKEY_PIN (-1)
#endif
#ifndef CELLULAR_POWER_EN_PIN
  #define CELLULAR_POWER_EN_PIN (-1)
#endif
#ifndef CELLULAR_STATUS_PIN
  #define CELLULAR_STATUS_PIN (-1)
#endif

CellularMQTTBridge::CellularMQTTBridge(NodePrefs* prefs, mesh::PacketManager* mgr,
                                       mesh::RTCClock* rtc, mesh::LocalIdentity* identity)
  : BridgeBase(prefs, mgr, rtc),
    _identity(identity),
    _modem(CELLULAR_UART, CELLULAR_PWRKEY_PIN, CELLULAR_POWER_EN_PIN, CELLULAR_STATUS_PIN) {}

// ---- helpers --------------------------------------------------------------

void CellularMQTTBridge::resolveOrigin() {
  const char* o = _prefs->cellular_origin;
  if (o && o[0]) {
    // strip surrounding quotes if present
    if (o[0] == '"') o++;
    strncpy(_origin, o, sizeof(_origin) - 1);
    _origin[sizeof(_origin) - 1] = 0;
    size_t n = strlen(_origin);
    if (n && _origin[n - 1] == '"') _origin[n - 1] = 0;
  } else {
    strncpy(_origin, _prefs->node_name, sizeof(_origin) - 1);
    _origin[sizeof(_origin) - 1] = 0;
  }
}

bool CellularMQTTBridge::buildTopic(MsgType type, char* buf, size_t buf_size) {
  const char* iata = _prefs->cellular_iata;
  if (!iata || iata[0] == '\0' || _device_id[0] == '\0') return false;
  const char* t = (type == MSG_STATUS) ? "status" : (type == MSG_PACKETS) ? "packets" : "raw";
  snprintf(buf, buf_size, "meshcore/%s/%s/%s", iata, _device_id, t);
  return true;
}

bool CellularMQTTBridge::enqueue(const char* topic, const char* payload, size_t len,
                                 uint8_t qos, bool retain) {
  if (len == 0 || len >= PACKET_BUF) { _dropped++; return false; }
  if (_q_count >= QUEUE_DEPTH) { _dropped++; return false; }   // full: drop newest
  Pending& p = _queue[_q_tail];
  strncpy(p.topic, topic, sizeof(p.topic) - 1);
  p.topic[sizeof(p.topic) - 1] = 0;
  memcpy(p.payload, payload, len);
  p.len = (uint16_t)len;
  p.qos = qos;
  p.retain = retain;
  _q_tail = (_q_tail + 1) % QUEUE_DEPTH;
  _q_count++;
  return true;
}

void CellularMQTTBridge::drainOne() {
  if (_q_count == 0 || !_modem.isReady()) return;
  Pending& p = _queue[_q_head];
  bool ok = _modem.publish(p.topic, p.payload, p.len, p.qos, p.retain);
  // Pop regardless (one attempt) to avoid head-of-line stalls; count the outcome.
  _q_head = (_q_head + 1) % QUEUE_DEPTH;
  _q_count--;
  if (ok) _published++; else _dropped++;
}

// ---- lifecycle ------------------------------------------------------------

void CellularMQTTBridge::begin() {
  if (_initialized) return;

  // Bring up the modem UART (Serial1 on the WisBlock).
#if defined(NRF52_PLATFORM) && defined(CELLULAR_UART_RX) && defined(CELLULAR_UART_TX)
  ((Uart*)&CELLULAR_UART)->setPins(CELLULAR_UART_RX, CELLULAR_UART_TX);
#endif
  CELLULAR_UART.begin(CELLULAR_UART_BAUD);

  // Broker: bare host + port (the BG77 does raw MQTT-over-TLS/mqtts, not WSS). Strip a URI
  // scheme defensively in case the operator pasted one out of habit.
  char host[64] = {0};
  const char* h = _prefs->cellular_host;
  const char* scheme = strstr(h, "://");
  if (scheme) h = scheme + 3;
  size_t i = 0;
  while (h[i] && h[i] != '/' && h[i] != ':' && i < sizeof(host) - 1) { host[i] = h[i]; i++; }
  host[i] = 0;
  uint16_t port = _prefs->cellular_port ? _prefs->cellular_port : 8883;

  resolveOrigin();
  _modem.setBroker(host, port);
  _modem.setCredentials(_prefs->cellular_user, _prefs->cellular_pass);
  _modem.setClientId(_device_id[0] ? _device_id : _origin);
  _modem.setAPN(_prefs->cellular_apn[0] ? _prefs->cellular_apn : "hologram");
  if (_prefs->cellular_band[0]) _modem.setBand(_prefs->cellular_band);
  _modem.setTLS(_prefs->cellular_tls != 0, _prefs->cellular_tls_verify != 0);
  _modem.setTimeSyncCallback(&CellularMQTTBridge::onModemTime, this);
#ifdef CELLULAR_AT_DEBUG
  _modem.setDebugStream(&Serial);   // echo AT traffic to USB during bring-up
#endif

  _modem.begin();
  _last_status = 0;   // publish status as soon as the broker is up
  _initialized = true;
}

void CellularMQTTBridge::end() {
  if (!_initialized) return;
  _modem.end();
  _q_head = _q_tail = _q_count = 0;
  _initialized = false;
}

void CellularMQTTBridge::loop() {
  if (!_initialized) return;
  _modem.loop();

  if (_modem.isReady()) {
    // Periodic /status. cellular_status_interval is stored in ms (0 -> default 5 min).
    if (_prefs->cellular_status_enabled) {
      uint32_t interval = _prefs->cellular_status_interval ? _prefs->cellular_status_interval : 300000UL;
      unsigned long now = millis();
      if ((now - _last_status) >= interval) {
        _last_status = now;
        buildAndQueueStatus();
      }
    }
    drainOne();   // one modem exchange per loop iteration
  }

  // GNSS self-location runs once the modem is up (independent of broker connection), but
  // never while a non-blocking bring-up command is in flight — the blocking QGPS/QGPSLOC
  // exchange would flush and clobber that pending response (e.g. the 45s QMTOPEN), breaking
  // the LTE connect. isBusy() gate confines GNSS to the idle gaps between atTick commands.
  if (_prefs->cellular_gps_enabled && _modem.isUp() && !_modem.isBusy()) handleGnss();
}

void CellularMQTTBridge::handleGnss() {
  unsigned long now = millis();
  if (!_gps_on) {
    if ((long)(now - _gps_next_attempt) >= 0) {   // due for an acquisition attempt
      _modem.gnssEnable();
      _gps_on = true;
      _gps_started = now;
      _gps_last_poll = 0;
    }
    return;
  }
  // acquiring: poll for a fix every ~5s; give up after 2 min and retry later
  if ((now - _gps_last_poll) < 5000) return;
  _gps_last_poll = now;
  float lat = 0, lon = 0;
  if (_modem.gnssGetFix(lat, lon)) {
    _prefs->node_lat = lat;             // picked up by the next self-advert -> map pin
    _prefs->node_lon = lon;
    _modem.gnssDisable();
    _gps_on = false;
    _gps_next_attempt = now + 6UL * 3600 * 1000;   // refresh in 6 h (stationary node)
  } else if ((now - _gps_started) >= 120000) {
    _modem.gnssDisable();
    _gps_on = false;
    _gps_next_attempt = now + 10UL * 60 * 1000;     // no fix yet — retry in 10 min
  }
}

// ---- packet ingest --------------------------------------------------------

void CellularMQTTBridge::storeRawRadioData(const uint8_t* raw_data, int len, float snr, float rssi) {
  if (len > 0 && len <= (int)sizeof(_staged_raw)) {
    memcpy(_staged_raw, raw_data, len);
    _staged_len = len;
    _staged_snr = snr;
    _staged_rssi = rssi;
    _staged_ts = millis();
    _staged_valid = true;
  }
}

void CellularMQTTBridge::onPacketReceived(mesh::Packet* packet) {
  if (!_initialized || !_prefs->cellular_pkts_enabled || !_prefs->cellular_rx_enabled) return;
  buildAndQueuePacket(packet, false, _staged_snr, _staged_rssi);
}

void CellularMQTTBridge::sendPacket(mesh::Packet* packet) {
  uint8_t tx_mode = _prefs->cellular_tx_enabled;
  if (!_initialized || !_prefs->cellular_pkts_enabled || tx_mode == 0) return;
  if (tx_mode == 2) {   // advert-only: just our own self-advert
    if (packet->getPayloadType() != PAYLOAD_TYPE_ADVERT) return;
    if (packet->payload_len < PUB_KEY_SIZE) return;
    if (!_identity || memcmp(_identity->pub_key, packet->payload, PUB_KEY_SIZE) != 0) return;
  }
  buildAndQueuePacket(packet, true, 0, 0);
}

void CellularMQTTBridge::buildAndQueuePacket(mesh::Packet* packet, bool is_tx, float snr, float rssi) {
  if (!packet) return;
  char origin_id[65];
  strncpy(origin_id, _device_id, sizeof(origin_id) - 1);
  origin_id[sizeof(origin_id) - 1] = 0;

  int len;
  bool staged_fresh = (!is_tx && _staged_valid && (millis() - _staged_ts) < 1000 && _staged_len > 0);
  if (staged_fresh) {
    float score = _radio ? _radio->packetScore(_staged_snr, _staged_len) : NAN;
    len = MQTTMessageBuilder::buildPacketJSONFromRaw(
      _json_doc, _staged_raw, _staged_len, packet, is_tx, _origin, origin_id,
      _staged_snr, _staged_rssi, score, _timezone, _scratch, PACKET_BUF);
    _staged_valid = false;
  } else {
    // Reconstruct wire bytes from the packet when we have no fresh raw capture.
    uint8_t recon[512];
    uint8_t rlen = packet->writeTo(recon);
    if (rlen > 0) {
      float score = (_radio && !is_tx) ? _radio->packetScore(snr, rlen) : NAN;
      len = MQTTMessageBuilder::buildPacketJSONFromRaw(
        _json_doc, recon, rlen, packet, is_tx, _origin, origin_id,
        snr, rssi, score, _timezone, _scratch, PACKET_BUF);
    } else {
      len = MQTTMessageBuilder::buildPacketJSON(
        _json_doc, packet, is_tx, _origin, origin_id, _timezone, _scratch, PACKET_BUF);
    }
  }

  if (len <= 0) return;
  char topic[128];
  if (!buildTopic(MSG_PACKETS, topic, sizeof(topic))) return;
  enqueue(topic, _scratch, (size_t)len, 0, false);
}

// ---- status ---------------------------------------------------------------

void CellularMQTTBridge::buildAndQueueStatus() {
  resolveOrigin();

  char origin_id[65];
  strncpy(origin_id, _device_id, sizeof(origin_id) - 1);
  origin_id[sizeof(origin_id) - 1] = 0;

  char timestamp[40];
  uint32_t epoch = _rtc ? _rtc->getCurrentTime() : 0;
  MQTTMessageBuilder::formatIsoTimestampForMqtt((time_t)epoch, 0, _timezone, timestamp, sizeof(timestamp));

  char radio_info[64];
  snprintf(radio_info, sizeof(radio_info), "%.6f,%.1f,%d,%d",
           _prefs->freq, _prefs->bw, _prefs->sf, _prefs->cr);

  char client_version[64];
  snprintf(client_version, sizeof(client_version), "meshcore/%s", _firmware_version);

  int battery_mv = -1, uptime_secs = -1, errors = -1, noise_floor = -999;
  int tx_air_secs = -1, rx_air_secs = -1, recv_errors = -1, packets_sent = -1, packets_received = -1;
  if (_board) battery_mv = _board->getBattMilliVolts();
  if (_ms) uptime_secs = _ms->getMillis() / 1000;
  if (_dispatcher) {
    errors = _dispatcher->getErrFlags();
    tx_air_secs = _dispatcher->getTotalAirTime() / 1000;
    rx_air_secs = _dispatcher->getReceiveAirTime() / 1000;
    packets_sent = (int)(_dispatcher->getNumSentFlood() + _dispatcher->getNumSentDirect());
    packets_received = (int)(_dispatcher->getNumRecvFlood() + _dispatcher->getNumRecvDirect());
  }
  if (_radio) {
    noise_floor = (int16_t)_radio->getNoiseFloor();
    recv_errors = (int)_radio->getPacketsRecvErrors();
  }

  int len = MQTTMessageBuilder::buildStatusMessage(
    _json_doc, _origin, origin_id, _board_model, _firmware_version, radio_info,
    client_version, "online", timestamp, _scratch, STATUS_BUF,
    battery_mv, uptime_secs, errors, /*queue_len*/ _q_count, noise_floor,
    tx_air_secs, rx_air_secs, recv_errors, /*internal_heap*/ -1,
    packets_sent, packets_received, _prefs->disable_fwd ? "off" : "on");

  if (len <= 0) return;
  char topic[128];
  if (!buildTopic(MSG_STATUS, topic, sizeof(topic))) return;
  enqueue(topic, _scratch, (size_t)len, 1, true);   // QoS 1, retained (parity with MQTTBridge)
}

// ---- metadata / stats setters --------------------------------------------

void CellularMQTTBridge::setDeviceID(const char* v)        { if (v) { strncpy(_device_id, v, sizeof(_device_id)-1); _device_id[sizeof(_device_id)-1]=0; } }
void CellularMQTTBridge::setFirmwareVersion(const char* v) { if (v) { strncpy(_firmware_version, v, sizeof(_firmware_version)-1); _firmware_version[sizeof(_firmware_version)-1]=0; } }
void CellularMQTTBridge::setBoardModel(const char* v)      { if (v) { strncpy(_board_model, v, sizeof(_board_model)-1); _board_model[sizeof(_board_model)-1]=0; } }
void CellularMQTTBridge::setBuildDate(const char* /*v*/)   { /* carried in firmware string; not separately published */ }
void CellularMQTTBridge::setOrigin(const char* v)          { if (v) { strncpy(_origin, v, sizeof(_origin)-1); _origin[sizeof(_origin)-1]=0; } }
void CellularMQTTBridge::setIATA(const char* v)            { if (v) { strncpy(_iata, v, sizeof(_iata)-1); _iata[sizeof(_iata)-1]=0; } }

void CellularMQTTBridge::setStatsSources(mesh::Dispatcher* dispatcher, mesh::Radio* radio,
                                         mesh::MainBoard* board, mesh::MillisecondClock* ms) {
  _dispatcher = dispatcher;
  _radio = radio;
  _board = board;
  _ms = ms;
}

void CellularMQTTBridge::formatStatus(char* buf, size_t buf_size) {
  int csq = _modem.lastRssiDbm();   // 0 = unknown (CSQ 99)
  char gps[32];
  if (_prefs->node_lat != 0.0 || _prefs->node_lon != 0.0)
    snprintf(gps, sizeof(gps), "%.5f,%.5f", _prefs->node_lat, _prefs->node_lon);
  else
    snprintf(gps, sizeof(gps), "%s", _prefs->cellular_gps_enabled ? "acq" : "off");
  snprintf(buf, buf_size, "modem=%s reg=%d csq=%d mqtt=%s q=%d pub=%lu drop=%lu err=%s pube=%s gps=%s",
           _modem.stateName(), _modem.ceregStat(), csq,
           _modem.isReady() ? "up" : "down", _q_count,
           (unsigned long)_published, (unsigned long)_dropped, _modem.lastError(),
           _modem.lastPubResult(), gps);
}

void CellularMQTTBridge::onModemTime(void* ctx, uint32_t epoch) {
  CellularMQTTBridge* self = (CellularMQTTBridge*)ctx;
  if (self && self->_rtc && epoch > 1700000000UL) self->_rtc->setCurrentTime(epoch);
}

#endif // WITH_CELLULAR_MQTT_BRIDGE
