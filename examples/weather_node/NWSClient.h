#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <RAK13800_W5100S.h>
#include <ArduinoJson.h>

// Manually instantiate SPI1 for SPIM2 peripheral
#if defined(NRF52_PLATFORM)
  SPIClass SPI1(NRF_SPIM2, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI);
#endif

#ifndef NWS_PROXY_IP
  #define NWS_PROXY_IP "192.168.8.68"
#endif

#ifndef NWS_PROXY_PORT
  #define NWS_PROXY_PORT 8085
#endif

#ifndef NWS_ZONE
  #define NWS_ZONE "WIZ038,WIZ039,WIZ040"
#endif

#ifndef NWS_MAX_ALERTS
  #define NWS_MAX_ALERTS 8
#endif

#ifndef ETH_CS_PIN
  #define ETH_CS_PIN    26
#endif

#ifndef ETH_RST_PIN
  #define ETH_RST_PIN   21
#endif

struct NWSAlert {
  char event[48];
  char severity[16];
  char headline[140];
  char id_hash[16];
};

// NWS severity levels (lowest to highest)
enum NWSSeverityLevel : uint8_t {
  NWS_SEV_ALL      = 0,  // Unknown and above
  NWS_SEV_MINOR    = 1,
  NWS_SEV_MODERATE = 2,
  NWS_SEV_SEVERE   = 3,
  NWS_SEV_EXTREME  = 4
};

class NWSClient {
  EthernetClient _client;
  bool _eth_ready;
  byte _mac[6];
  NWSAlert _alerts[NWS_MAX_ALERTS];
  int _num_alerts;
  char _sent_hashes[NWS_MAX_ALERTS][16];
  int _num_sent;
  char _zone[64];
  uint8_t _min_severity;  // NWSSeverityLevel — alerts below this are not counted as new

  static uint8_t severityToLevel(const char* sev) {
    if (strcasecmp(sev, "Extreme") == 0)  return NWS_SEV_EXTREME;
    if (strcasecmp(sev, "Severe") == 0)   return NWS_SEV_SEVERE;
    if (strcasecmp(sev, "Moderate") == 0) return NWS_SEV_MODERATE;
    if (strcasecmp(sev, "Minor") == 0)    return NWS_SEV_MINOR;
    return NWS_SEV_ALL; // Unknown
  }

  void generateMAC() {
    uint32_t id0 = NRF_FICR->DEVICEID[0];
    _mac[0] = 0x02; _mac[1] = 0x42;
    _mac[2] = (id0 >> 24) & 0xFF; _mac[3] = (id0 >> 16) & 0xFF;
    _mac[4] = (id0 >> 8) & 0xFF; _mac[5] = id0 & 0xFF;
  }

  bool alreadySent(const char* hash) const {
    for (int i = 0; i < _num_sent; i++) {
      if (strcmp(_sent_hashes[i], hash) == 0) return true;
    }
    return false;
  }

  void markSent(const char* hash) {
    if (_num_sent >= NWS_MAX_ALERTS) {
      for (int i = 0; i < NWS_MAX_ALERTS - 1; i++) {
        memcpy(_sent_hashes[i], _sent_hashes[i + 1], 16);
      }
      _num_sent = NWS_MAX_ALERTS - 1;
    }
    strncpy(_sent_hashes[_num_sent], hash, 15);
    _sent_hashes[_num_sent][15] = 0;
    _num_sent++;
  }

public:
  NWSClient() : _eth_ready(false), _num_alerts(0), _num_sent(0), _min_severity(NWS_SEV_SEVERE) {
    generateMAC();
    memset(_alerts, 0, sizeof(_alerts));
    memset(_sent_hashes, 0, sizeof(_sent_hashes));
    strncpy(_zone, NWS_ZONE, sizeof(_zone) - 1);
    _zone[sizeof(_zone) - 1] = 0;
  }

  void setMinSeverity(uint8_t level) { _min_severity = level; }
  uint8_t getMinSeverity() const { return _min_severity; }

  void setZone(const char* zone) {
    strncpy(_zone, zone, sizeof(_zone) - 1);
    _zone[sizeof(_zone) - 1] = 0;
  }
  const char* getZone() const { return _zone; }

  bool begin() {
    Serial.println("[ETH] Enabling 3.3V rail...");
    pinMode(34, OUTPUT); digitalWrite(34, HIGH); delay(500);
    
    Serial.println("[ETH] Resetting W5100S hardware...");
    pinMode(ETH_RST_PIN, OUTPUT); digitalWrite(ETH_RST_PIN, LOW); delay(100); digitalWrite(ETH_RST_PIN, HIGH); delay(500);

    SPI1.begin();
    Ethernet.init(SPI1, ETH_CS_PIN);

    if (Ethernet.begin(_mac) == 0) {
      Serial.println("[ETH] DHCP FAILED.");
      return false;
    }
    Serial.print("[ETH] IP:      "); Serial.println(Ethernet.localIP());
    Serial.print("[ETH] Subnet:  "); Serial.println(Ethernet.subnetMask());
    Serial.print("[ETH] Gateway: "); Serial.println(Ethernet.gatewayIP());
    Serial.print("[ETH] DNS:     "); Serial.println(Ethernet.dnsServerIP());

    // If DHCP gave a wrong subnet mask (e.g. /16 instead of /24), the W5100S
    // will try to ARP directly for cross-subnet hosts instead of routing through
    // the gateway. Force /24 to ensure traffic to 192.168.8.x goes via the gateway.
    IPAddress gw = Ethernet.gatewayIP();
    IPAddress forced_subnet(255, 255, 255, 0);
    if (Ethernet.subnetMask() != forced_subnet) {
      Serial.println("[ETH] WARNING: Subnet mask not /24, forcing correction...");
      Ethernet.begin(_mac, Ethernet.localIP(), Ethernet.dnsServerIP(), gw, forced_subnet);
      Serial.print("[ETH] Subnet corrected: "); Serial.println(Ethernet.subnetMask());
    }

    _eth_ready = true;
    return true;
  }

  bool testGateway() {
    EthernetClient test;
    test.setConnectionTimeout(2000);
    IPAddress gw = Ethernet.gatewayIP();
    Serial.print("[DIAG] Testing Gateway ("); Serial.print(gw); Serial.print(")... ");
    bool ok = (test.connect(gw, 80) || test.connect(gw, 53));
    Serial.println(ok ? "REACHABLE" : "UNREACHABLE");
    test.stop();
    return ok;
  }

  void runPathTrace() {
    IPAddress proxy; proxy.fromString(NWS_PROXY_IP);
    Serial.print("[DIAG] Path Trace to "); Serial.print(proxy); Serial.println(":");
    EthernetClient trace;
    trace.setConnectionTimeout(1500);
    Serial.print("  - SSH (22): "); Serial.println(trace.connect(proxy, 22) ? "OPEN" : "TIMEOUT"); trace.stop();
    Serial.print("  - HTTP (80): "); Serial.println(trace.connect(proxy, 80) ? "OPEN" : "TIMEOUT"); trace.stop();
    Serial.print("  - PROXY (8085): "); Serial.println(trace.connect(proxy, NWS_PROXY_PORT) ? "OPEN" : "TIMEOUT"); trace.stop();

    if (!testGateway()) {
      Serial.println("[ETH] Gateway lost after path trace — W5100S socket exhaustion, recovering...");
      recover();
    }
  }

  bool recover() {
    _eth_ready = false;
    Serial.println("[ETH] Hard resetting W5100S...");
    pinMode(ETH_RST_PIN, OUTPUT);
    digitalWrite(ETH_RST_PIN, LOW); delay(100);
    digitalWrite(ETH_RST_PIN, HIGH); delay(500);

    SPI1.begin();
    Ethernet.init(SPI1, ETH_CS_PIN);

    if (Ethernet.begin(_mac) == 0) {
      Serial.println("[ETH] Recovery DHCP failed.");
      return false;
    }
    Serial.print("[ETH] Recovered. IP: "); Serial.println(Ethernet.localIP());
    _eth_ready = true;
    return true;
  }

  int pollAlerts() {
    if (!_eth_ready) return 0;
    _num_alerts = 0;
    IPAddress proxy; proxy.fromString(NWS_PROXY_IP);

    Serial.print("[NWS] Connecting to proxy...");
    _client.setConnectionTimeout(5000);
    int result = _client.connect(proxy, NWS_PROXY_PORT);

    if (result != 1) {
      Serial.print(" FAIL (Code:"); Serial.print(result);
      Serial.print(", Status:"); Serial.print(_client.status()); Serial.println(")");
      _client.stop();

      // W5100S commonly loses state on first use after boot. Recover and retry once
      // before falling back to the full path trace diagnostic.
      Serial.println("[ETH] First connect failed, attempting recovery before diagnostics...");
      if (recover()) {
        Serial.print("[NWS] Retrying proxy connection...");
        result = _client.connect(proxy, NWS_PROXY_PORT);
        if (result != 1) {
          Serial.print(" FAIL (Code:"); Serial.print(result);
          Serial.print(", Status:"); Serial.print(_client.status()); Serial.println(")");
          _client.stop();
          runPathTrace();
          return 0;
        }
        Serial.println(" SUCCESS.");
      } else {
        runPathTrace();
        return 0;
      }
    }

    Serial.println(" SUCCESS. Requesting alerts...");
    _client.print("GET /alerts/active?zone="); _client.print(_zone); _client.println(" HTTP/1.1");
    _client.print("Host: "); _client.println(NWS_PROXY_IP);
    _client.println("Connection: close");
    _client.println();

    unsigned long timeout = millis();
    while (!_client.available()) {
      if (millis() - timeout > 10000) { _client.stop(); return 0; }
      delay(10);
    }

    String body = "";
    bool headers_done = false;
    while (_client.available()) {
      if (!headers_done) {
        String line = _client.readStringUntil('\n');
        if (line == "\r") headers_done = true;
      } else {
        char c = _client.read();
        if (body.length() < 12288) body += c;
      }
    }
    _client.stop();

    JsonDocument doc;
    if (deserializeJson(doc, body)) return 0;

    JsonArray features = doc["features"].as<JsonArray>();
    int new_severe = 0;
    for (JsonObject feature : features) {
      if (_num_alerts >= NWS_MAX_ALERTS) break;
      JsonObject props = feature["properties"];
      strncpy(_alerts[_num_alerts].event, props["event"] | "Unknown", 47);
      strncpy(_alerts[_num_alerts].severity, props["severity"] | "Unknown", 15);
      strncpy(_alerts[_num_alerts].headline, props["headline"] | "", 139);
      
      unsigned long hash = 5381;
      const char* id = feature["id"] | "";
      for (const char* p = id; *p; p++) hash = ((hash << 5) + hash) + *p;
      snprintf(_alerts[_num_alerts].id_hash, 16, "%08lX", hash);

      if (severityToLevel(_alerts[_num_alerts].severity) >= _min_severity) {
        if (!alreadySent(_alerts[_num_alerts].id_hash)) new_severe++;
      }
      _num_alerts++;
    }
    return new_severe;
  }

  // Restored original support methods for MyMesh.h
  int getNumAlerts() const { return _num_alerts; }
  bool isAlertNew(int idx) const { return idx >= 0 && idx < _num_alerts && !alreadySent(_alerts[idx].id_hash); }
  bool isAlertAboveThreshold(int idx) const {
    return idx >= 0 && idx < _num_alerts &&
           severityToLevel(_alerts[idx].severity) >= _min_severity;
  }
  const char* getAlertSeverity(int idx) const {
    return (idx >= 0 && idx < _num_alerts) ? _alerts[idx].severity : "";
  }
  void markAlertSent(int idx) { if (idx >= 0 && idx < _num_alerts) markSent(_alerts[idx].id_hash); }
  void clearSentHistory() { _num_sent = 0; memset(_sent_hashes, 0, sizeof(_sent_hashes)); }
  bool isReady() const { return _eth_ready; }
  void maintain() { if (_eth_ready) Ethernet.maintain(); }

  // Compact header: "[Severe] Winter Storm Warning"
  int formatHeader(int idx, char* buf, int bufSize) const {
    if (idx < 0 || idx >= _num_alerts) return 0;
    return snprintf(buf, bufSize, "[%s] %s", _alerts[idx].severity, _alerts[idx].event);
  }

  // Full headline detail text
  int formatDetail(int idx, char* buf, int bufSize) const {
    if (idx < 0 || idx >= _num_alerts) return 0;
    return snprintf(buf, bufSize, "%s", _alerts[idx].headline);
  }

  // Legacy single-line format, used by CLI "nws alerts"
  int formatForMesh(int idx, char* buf, int bufSize) const {
    if (idx < 0 || idx >= _num_alerts) return 0;
    return snprintf(buf, bufSize, "NWS %s: %s", _alerts[idx].severity, _alerts[idx].headline);
  }
};