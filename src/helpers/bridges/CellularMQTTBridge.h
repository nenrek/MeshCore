#pragma once

#include "MeshCore.h"
#include "helpers/bridges/BridgeBase.h"
#include "helpers/MQTTMessageBuilder.h"
#include "helpers/cellular/BG77Modem.h"
#include <ArduinoJson.h>

#ifdef WITH_CELLULAR_MQTT_BRIDGE

/**
 * @brief MQTT bridge that uplinks mesh packets over an LTE-M modem (RAK5860 / BG77).
 *
 * Single-chip counterpart to MQTTBridge: instead of an ESP32 + WiFi + esp-mqtt, this
 * runs on the nRF52 (which already has the LoRa packets) and drives the BG77's onboard
 * TLS + MQTT client over UART (see BG77Modem). It REUSES MQTTMessageBuilder verbatim, so
 * the JSON on the wire — topics meshcore/{IATA}/{DEVICE_ID}/{status,packets,raw}, QoS 1
 * status (retained), packet payloads — is byte-identical to the WiFi observers'.
 *
 * Differences vs MQTTBridge (deliberately simpler): one broker (your Mosquitto), no WSS
 * (BG77 can't), no JWT, no subscribe/downlink, no FreeRTOS task (advanced from loop()).
 * Broker + credentials are read from custom slot 0 prefs (mqtt1.server/port/username/
 * password); cellular-specific settings come from the cell.* prefs.
 */
class CellularMQTTBridge : public BridgeBase {
public:
  CellularMQTTBridge(NodePrefs* prefs, mesh::PacketManager* mgr, mesh::RTCClock* rtc,
                     mesh::LocalIdentity* identity);

  // AbstractBridge
  void begin() override;
  void end() override;
  void loop() override;
  bool isRunning() const override { return _initialized; }
  void onPacketReceived(mesh::Packet* packet) override;   // RX packets
  void sendPacket(mesh::Packet* packet) override;         // TX packets

  // Called by MyMesh::logRxRaw before onPacketReceived — stages the raw bytes + SNR/RSSI
  // so the packet JSON carries the true wire bytes and link metadata.
  void storeRawRadioData(const uint8_t* raw_data, int len, float snr, float rssi);

  // Device metadata (parity with MQTTBridge; set by MyMesh::setBridgeState).
  void setDeviceID(const char* device_id);
  void setFirmwareVersion(const char* v);
  void setBoardModel(const char* m);
  void setBuildDate(const char* d);
  void setOrigin(const char* o);
  void setIATA(const char* iata);

  // Live stat sources for /status (dispatcher/radio/board/clock). Same signature as MQTTBridge.
  void setStatsSources(mesh::Dispatcher* dispatcher, mesh::Radio* radio,
                       mesh::MainBoard* board, mesh::MillisecondClock* ms);

  // No-op: kept because MyMesh::restartBridgeSlot() calls it unconditionally. The cellular
  // bridge has a single implicit slot, so per-slot preset changes don't apply.
  void setSlotPreset(int /*slot*/, const char* /*preset*/) {}

  // Bridge API surface MyMesh calls under WITH_BRIDGE (parity with MQTTBridge).
  int  getQueueSize() const { return _q_count; }
  // Time comes from the LTE network (AT+QLTS), not NTP — these satisfy the MyMesh
  // overrides. requestForcedNtpSync reports whether the modem (hence clock) is up.
  bool requestForcedNtpSync(uint32_t /*timeout_ms*/ = 30000) { return _modem.isReady(); }
  bool ntpDiag(char* reply, size_t reply_size, bool /*verbose*/) {
    snprintf(reply, reply_size, "> cellular: clock via LTE network (AT+QLTS), modem %s",
             _modem.isReady() ? "ready" : "not ready");
    return true;
  }

  BG77Modem& modem() { return _modem; }
  // Compact one-line status for the mesh-pollable `cell.status` CLI command.
  void formatStatus(char* buf, size_t buf_size);

private:
  enum MsgType { MSG_STATUS, MSG_PACKETS, MSG_RAW };
  bool buildTopic(MsgType type, char* buf, size_t buf_size);
  void resolveOrigin();                 // node_name / mqtt_origin -> _origin
  void buildAndQueueStatus();
  void buildAndQueuePacket(mesh::Packet* packet, bool is_tx, float snr, float rssi);
  // Enqueue a ready-to-send payload; drains in loop() so the modem never blocks the RX path.
  bool enqueue(const char* topic, const char* payload, size_t len, uint8_t qos, bool retain);
  void drainOne();
  // Non-blocking GNSS self-location: acquire a fix, set the node's advert lat/lon, refresh
  // periodically. Runs only when cell.gps is on and the modem is attached.
  void handleGnss();
  // Static trampoline so BG77Modem can push network time into the mesh RTC clock.
  static void onModemTime(void* ctx, uint32_t epoch);

  static const size_t PACKET_BUF = 2048;   // matches MQTTBridge PUBLISH_JSON_BUFFER_SIZE
  static const size_t STATUS_BUF = 768;    // matches MQTTBridge STATUS_JSON_BUFFER_SIZE
  static const int    QUEUE_DEPTH = 4;     // pending publishes buffered while the modem works

  mesh::LocalIdentity* _identity;
  BG77Modem _modem;

  mesh::Dispatcher* _dispatcher = nullptr;
  mesh::Radio* _radio = nullptr;
  mesh::MainBoard* _board = nullptr;
  mesh::MillisecondClock* _ms = nullptr;
  Timezone* _timezone = nullptr;   // MQTTMessageBuilder ignores it (UTC); nullptr is fine

  char _device_id[65] = {0};
  char _origin[32] = {0};
  char _iata[8] = {0};
  char _board_model[40] = {0};
  char _firmware_version[24] = {0};

  // Staged raw-radio data for the next onPacketReceived (parity with MQTTBridge staging).
  uint8_t _staged_raw[256];
  int   _staged_len = 0;
  float _staged_snr = 0, _staged_rssi = 0;
  unsigned long _staged_ts = 0;
  bool  _staged_valid = false;

  unsigned long _last_status = 0;
  uint32_t _published = 0, _dropped = 0;

  // GNSS acquisition state. One-shot per boot: on a stationary node a single fix is all we
  // need, and each attempt briefly tears down the uplink (BG77 GNSS preempts LTE), so we
  // don't refresh on success — reboot to re-acquire.
  bool _gps_on = false;
  bool _gps_done = false;
  unsigned long _gps_started = 0, _gps_last_poll = 0, _gps_next_attempt = 0;
  static const unsigned long GNSS_ACQ_WINDOW_MS = 120000;   // give a fix 2 min before giving up
  static const unsigned long GNSS_RETRY_MS      = 1800000;  // 30 min before another disruptive try

  // JSON is built in the producer (RX/TX callback, while the packet is valid) into _scratch,
  // then copied into the ring; loop() pops and hands one entry at a time to the modem.
  StaticJsonDocument<PACKET_BUF> _json_doc;
  char _scratch[PACKET_BUF];

  struct Pending {
    char topic[128];
    uint16_t len;
    uint8_t qos;
    bool retain;
    char payload[PACKET_BUF];
  };
  Pending _queue[QUEUE_DEPTH];
  int _q_head = 0, _q_tail = 0, _q_count = 0;
};

#endif // WITH_CELLULAR_MQTT_BRIDGE
