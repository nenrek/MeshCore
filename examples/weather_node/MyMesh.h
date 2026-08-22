#pragma once

#include "SensorMesh.h"
#include "NWSClient.h"
#include "NWSDisplayData.h"
#include <Dns.h>

#ifdef DISPLAY_CLASS
  #include "UITask.h"
#endif

#ifndef NWS_POLL_INTERVAL
  #define NWS_POLL_INTERVAL 120000   // 2 minutes in ms
#endif

// Mesh broadcast spacing. LoRa is half-duplex + multi-hop: parts/alerts sent too
// close together flood-collide and get missed. PART_GAP spaces the 3 parts of one
// alert; ALERT_GAP spaces consecutive alerts (must exceed 2*PART_GAP so a whole
// alert clears the mesh before the next starts).
#ifndef NWS_ALERT_PART_GAP_MS
  #define NWS_ALERT_PART_GAP_MS 8000    // gap between the 3 parts of ONE alert
#endif
#ifndef NWS_ALERT_GAP_MS
  #define NWS_ALERT_GAP_MS 45000        // gap between CONSECUTIVE alerts
#endif

#ifndef SENSOR_READ_INTERVAL_SECS
  #define SENSOR_READ_INTERVAL_SECS 60
#endif

#ifndef FIRMWARE_BUILD_DATE
  #define FIRMWARE_BUILD_DATE   "04 Apr 2026"
#endif

#ifndef FIRMWARE_VERSION
  #define FIRMWARE_VERSION   "v1.0.0-nws"
#endif

#undef FIRMWARE_ROLE
#define FIRMWARE_ROLE "weather"

// Hashtag channel the severe weather alerts broadcast on. Public by design:
// the key is derived from the name (first 16 bytes of sha256("#name"), same
// derivation the companion apps use), so anyone can join by adding the
// hashtag channel in their app. Change at runtime with `nws hashtag <name>`.
#ifndef NWS_HASHTAG
  #define NWS_HASHTAG "#atw-wx"
#endif

// Public channel key
static const uint8_t NWS_PUB_CHANNEL_KEY[CIPHER_KEY_SIZE] = {
  0x8b, 0x33, 0x87, 0xe9, 0xc5, 0xcd, 0xea, 0x6a,
  0xc9, 0xe5, 0xed, 0xba, 0xa1, 0x15, 0xcd, 0x72
};

class MyMesh : public SensorMesh {
  NWSClient* _nws;
  FILESYSTEM* _nws_fs;
  uint8_t  _severe_key[CIPHER_KEY_SIZE];
  char     _severe_hashtag[24];
  unsigned long _next_nws_poll;
  unsigned long _next_eth_retry;
  bool _poll_requested = false;   // NWS_MANUAL_POLL: only poll on explicit `nws poll`
  int8_t _wx_state = -1;          // last wxdiag TLS state: -1 unknown, 0 fail, 1 ok (edge-report)
  unsigned long _next_mesh_broadcast;
  unsigned long _next_weekly_announce;
  int  _pending_alert_idx;
  bool _has_pending_alerts;
  bool _boot_announced;
  unsigned long _boot_announce_at;
  uint32_t _alerts_sent_total;
  uint32_t _polls_total;
  unsigned long _last_sent_history_clear;
  int8_t   _utc_offset;  // hours offset from UTC, e.g. -5 for CDT
  // Uptime Kuma push monitor
  bool     _uk_enabled;
  char     _uk_host[40];
  uint16_t _uk_port;
  char     _uk_token[64];
  char     _uk_path[48];   // push path prefix, default "/api/push/"
  uint32_t _uk_interval_ms;
  unsigned long _next_uk_push;
  bool     _uk_last_ok;
  uint8_t  _min_severity;   // NWSSeverityLevel — minimum level to broadcast
  NWSDisplayData _display_data;

public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio,
         mesh::MillisecondClock& ms, mesh::RNG& rng,
         mesh::RTCClock& rtc, mesh::MeshTables& tables)
    : SensorMesh(board, radio, ms, rng, rtc, tables),
      _nws(nullptr), _nws_fs(nullptr),
      _next_nws_poll(0), _next_eth_retry(0), _next_mesh_broadcast(0), _next_weekly_announce(0),
      _pending_alert_idx(0), _has_pending_alerts(false),
      _alerts_sent_total(0), _polls_total(0), _last_sent_history_clear(0), _utc_offset(0), _boot_announced(false), _boot_announce_at(0),
      _uk_enabled(false), _uk_port(3001), _uk_interval_ms(300000), _next_uk_push(0), _uk_last_ok(false),
      _min_severity(NWS_SEV_MODERATE)
  {
    memset(_severe_key, 0, sizeof(_severe_key));
    _severe_hashtag[0] = 0;
    _uk_host[0] = 0;
    _uk_token[0] = 0;
    strncpy(_uk_path, "/api/push/", sizeof(_uk_path));
    memset(&_display_data, 0, sizeof(_display_data));
    _display_data.display_timeout_secs = 60;  // default 60s
  }

  NWSDisplayData* getDisplayData() { return &_display_data; }

  void setNWSClient(NWSClient* client) { _nws = client; }

  const char* getRole() override { return FIRMWARE_ROLE; }

  // Called from main.cpp after begin(fs) — derives severe channel key and loads persisted zone
  void loadNWSPrefs(FILESYSTEM* fs) {
    _nws_fs = fs;

    // Severe alerts broadcast on a public hashtag channel: the key is derived
    // from the channel name, so subscribers just add the hashtag in their app.
    applySevereHashtag(NWS_HASHTAG);

    loadUptimePrefs(fs);

    bool has_cfg = fs && _nws && fs->exists("/nws.cfg");
    bool cfg_has_hashtag = false;
    if (has_cfg) {
      File f = fs->open("/nws.cfg");
      if (f) {
        char buf[320];
        int len = f.read((uint8_t*)buf, sizeof(buf) - 1);
        f.close();
        if (len > 0) {
          buf[len] = 0;

          char* o = strstr(buf, "utc_offset=");
          if (o) _utc_offset = (int8_t)atoi(o + 11);

          char* px = strstr(buf, "proxy=");
          if (px) {
            px += 6;
            char host[64]; int i = 0;
            while (*px && *px != '\n' && *px != '\r' && i < (int)sizeof(host) - 1) host[i++] = *px++;
            host[i] = 0;
            uint16_t port = _nws->getProxyPort();
            char* pp = strstr(buf, "proxyport=");
            if (pp) port = (uint16_t)atoi(pp + 10);
            if (host[0]) { _nws->setProxy(host, port); Serial.print("[NWS] Loaded proxy: "); Serial.print(host); Serial.print(":"); Serial.println(port); }
          }
          char* dt = strstr(buf, "display_timeout=");
          if (dt) _display_data.display_timeout_secs = (uint32_t)atol(dt + 16);
          char* sv = strstr(buf, "severity=");
          if (sv) _min_severity = (uint8_t)atoi(sv + 9);

          char* ht = strstr(buf, "hashtag=");
          if (ht) {
            ht += 8;
            char name[24]; int i = 0;
            while (*ht && *ht != '\n' && *ht != '\r' && i < (int)sizeof(name) - 1) name[i++] = *ht++;
            name[i] = 0;
            if (name[0]) { applySevereHashtag(name); cfg_has_hashtag = true; }
          }

          // Keep this parse last: it truncates buf at the zones value's end
          char* p = strstr(buf, "zones=");
          if (p) {
            p += 6;
            char* end = p;
            while (*end && *end != '\n' && *end != '\r') end++;
            *end = 0;
            if (strlen(p) > 0) {
              _nws->setZone(p);
              Serial.print("[NWS] Loaded zones: "); Serial.println(p);
            }
          }
        }
      }
    }

    // Config predates the hashtag channel + ATW scoping: apply the new zone
    // and severity defaults once and persist (proxy/offset/timeout are kept).
    if (has_cfg && !cfg_has_hashtag) {
      _nws->setZone(NWS_ZONE);
      _min_severity = NWS_SEV_MODERATE;
      saveNWSPrefs();
      Serial.println("[NWS] Migrated config: hashtag channel + ATW zones");
    }

    Serial.print("[NWS] Severe channel: "); Serial.println(_severe_hashtag);

    // Sync settings to NWSClient + initial display state
    if (_nws) {
      _nws->setMinSeverity(_min_severity);
      strncpy(_display_data.zone, _nws->getZone(), sizeof(_display_data.zone) - 1);
    }
    _display_data.uk_enabled = _uk_enabled;
    strncpy(_display_data.uk_host, _uk_host, sizeof(_display_data.uk_host) - 1);
    _display_data.uk_port = _uk_port;
  }

protected:
  Trigger severe_weather, extreme_weather;

  /* ========================== Helpers ========================== */

  static const char* severityLevelName(uint8_t level) {
    switch (level) {
      case NWS_SEV_ALL:      return "all";
      case NWS_SEV_MINOR:    return "minor";
      case NWS_SEV_MODERATE: return "moderate";
      case NWS_SEV_SEVERE:   return "severe";
      case NWS_SEV_EXTREME:  return "extreme";
      default:               return "severe";
    }
  }

  static uint8_t parseSeverityLevel(const char* name) {
    if (strcmp(name, "0") == 0 || strcasecmp(name, "all")      == 0) return NWS_SEV_ALL;
    if (strcmp(name, "1") == 0 || strcasecmp(name, "minor")    == 0) return NWS_SEV_MINOR;
    if (strcmp(name, "2") == 0 || strcasecmp(name, "moderate") == 0) return NWS_SEV_MODERATE;
    if (strcmp(name, "3") == 0 || strcasecmp(name, "severe")   == 0) return NWS_SEV_SEVERE;
    if (strcmp(name, "4") == 0 || strcasecmp(name, "extreme")  == 0) return NWS_SEV_EXTREME;
    return NWS_SEV_SEVERE; // default
  }

  // Returns node name if set, otherwise "NWS Alerts"
  const char* getDisplayName() {
    const char* name = getNodeName();
    return (name && name[0]) ? name : "NWS Alerts";
  }

  // Set the severe alerts hashtag channel. The canonical form keeps the
  // leading '#' — it is part of the sha256 input, matching the companion-app
  // hashtag-channel derivation: key = first 16 bytes of sha256("#name").
  void applySevereHashtag(const char* name) {
    if (name[0] == '#') {
      strncpy(_severe_hashtag, name, sizeof(_severe_hashtag) - 1);
      _severe_hashtag[sizeof(_severe_hashtag) - 1] = 0;
    } else {
      _severe_hashtag[0] = '#';
      strncpy(&_severe_hashtag[1], name, sizeof(_severe_hashtag) - 2);
      _severe_hashtag[sizeof(_severe_hashtag) - 1] = 0;
    }
    mesh::Utils::sha256(_severe_key, CIPHER_KEY_SIZE,
                        (const uint8_t*)_severe_hashtag, strlen(_severe_hashtag));
  }

  void saveNWSPrefs() {
    if (!_nws_fs || !_nws) return;
    char buf[320];
    snprintf(buf, sizeof(buf),
      "utc_offset=%d\ndisplay_timeout=%lu\nseverity=%d\nhashtag=%s\nzones=%s\nproxy=%s\nproxyport=%u\n",
      (int)_utc_offset, (unsigned long)_display_data.display_timeout_secs, (int)_min_severity,
      _severe_hashtag, _nws->getZone(), _nws->getProxyHost(), (unsigned)_nws->getProxyPort());
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    _nws_fs->remove("/nws.cfg");
    File f = _nws_fs->open("/nws.cfg", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = _nws_fs->open("/nws.cfg", "w");
#else
    File f = _nws_fs->open("/nws.cfg", "w", true);
#endif
    if (!f) return;
    f.write((const uint8_t*)buf, strlen(buf));
    f.close();
  }

  /* ========================== Uptime Kuma ========================== */

  void loadUptimePrefs(FILESYSTEM* fs) {
    if (!fs || !fs->exists("/uptime.cfg")) return;
    File f = fs->open("/uptime.cfg");
    if (!f) return;
    char buf[200];
    int len = f.read((uint8_t*)buf, sizeof(buf) - 1);
    f.close();
    if (len <= 0) return;
    buf[len] = 0;

    auto parseVal = [](const char* src, const char* key, char* dest, int dest_size) {
      const char* p = strstr(src, key);
      if (!p) return;
      p += strlen(key);
      int i = 0;
      while (*p && *p != '\n' && *p != '\r' && i < dest_size - 1) dest[i++] = *p++;
      dest[i] = 0;
    };

    char tmp[64];
    parseVal(buf, "enabled=", tmp, sizeof(tmp)); if (tmp[0]) _uk_enabled = atoi(tmp);
    parseVal(buf, "host=",    _uk_host,  sizeof(_uk_host));
    parseVal(buf, "token=",   _uk_token, sizeof(_uk_token));
    parseVal(buf, "path=",    _uk_path,  sizeof(_uk_path)); if (!_uk_path[0]) strncpy(_uk_path, "/api/push/", sizeof(_uk_path));
    parseVal(buf, "port=",    tmp, sizeof(tmp)); if (tmp[0]) _uk_port = (uint16_t)atoi(tmp);
    parseVal(buf, "interval=",tmp, sizeof(tmp)); if (tmp[0]) _uk_interval_ms = (uint32_t)atoi(tmp) * 1000;

    if (_uk_enabled && _uk_host[0] && _uk_token[0])
      Serial.printf("[UK] Loaded: %s:%u interval:%lus\n", _uk_host, _uk_port, (unsigned long)_uk_interval_ms/1000);
  }

  void saveUptimePrefs() {
    if (!_nws_fs) return;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
    _nws_fs->remove("/uptime.cfg");
    File f = _nws_fs->open("/uptime.cfg", FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
    File f = _nws_fs->open("/uptime.cfg", "w");
#else
    File f = _nws_fs->open("/uptime.cfg", "w", true);
#endif
    if (!f) return;
    char buf[200];
    snprintf(buf, sizeof(buf), "enabled=%d\nhost=%s\nport=%u\ntoken=%s\npath=%s\ninterval=%lu\n",
      (int)_uk_enabled, _uk_host, (unsigned)_uk_port, _uk_token, _uk_path, (unsigned long)(_uk_interval_ms / 1000));
    f.write((const uint8_t*)buf, strlen(buf));
    f.close();
  }

  void pushToUptimeKuma() {
    if (!_uk_enabled || !_uk_host[0] || !_uk_token[0]) return;
    if (!_nws || !_nws->isReady()) return;

    IPAddress host_ip;
    if (!host_ip.fromString(_uk_host)) {
      DNSClient dns;
      dns.begin(Ethernet.dnsServerIP());
      if (dns.getHostByName(_uk_host, host_ip) != 1) {
        Serial.print("[UK] DNS lookup failed for: "); Serial.println(_uk_host);
        _uk_last_ok = false;
        return;
      }
    }

    char msg[64];
    snprintf(msg, sizeof(msg), "Polls:%lu+Alerts:%lu+ETH:OK",
      (unsigned long)_polls_total, (unsigned long)_alerts_sent_total);

    for (int attempt = 1; attempt <= 3; attempt++) {
      if (attempt > 1) delay(3000);  // allow W5100S to release stale socket
      EthernetClient client;
      client.setConnectionTimeout(5000);
      if (!client.connect(host_ip, _uk_port)) {
        Serial.printf("[UK] Connect failed (attempt %d/3)\n", attempt);
        client.stop();
        continue;
      }

      client.print("GET "); client.print(_uk_path); client.print(_uk_token);
      client.print("?status=up&msg="); client.print(msg);
      client.println("&ping= HTTP/1.1");
      client.print("Host: "); client.println(_uk_host);
      client.println("Connection: close");
      client.println();

      unsigned long timeout = millis();
      while (!client.available() && millis() - timeout < 5000) delay(10);

      _uk_last_ok = false;
      if (client.available()) {
        String line = client.readStringUntil('\n');
        Serial.printf("[UK] Response (attempt %d/3): ", attempt); Serial.println(line);
        _uk_last_ok = line.indexOf("200") >= 0;
      } else {
        Serial.printf("[UK] No response (attempt %d/3, timeout)\n", attempt);
      }
      client.stop();

      if (_uk_last_ok) break;
      // delay handled at top of loop
    }

    _display_data.uk_last_ok = _uk_last_ok;
    Serial.print("[UK] Push: "); Serial.println(_uk_last_ok ? "OK" : "FAIL");
  }

  // Build a GroupChannel from a raw 16-byte key
  void formatTimestamp(char* buf, size_t len, uint32_t unix_ts) {
    // Apply UTC offset
    int32_t adjusted = (int32_t)unix_ts + (int32_t)_utc_offset * 3600;
    if (adjusted < 0) adjusted = 0;
    unix_ts = (uint32_t)adjusted;
    const char* tz_label = _utc_offset == 0 ? "UTC" : (_utc_offset < 0 ? "UTC-" : "UTC+");
    int tz_abs = _utc_offset < 0 ? -_utc_offset : _utc_offset;
    // Simple UTC breakdown — no mktime/gmtime on NRF52
    uint32_t s = unix_ts % 60; uint32_t m = (unix_ts / 60) % 60;
    uint32_t h = (unix_ts / 3600) % 24;
    uint32_t days = unix_ts / 86400;          // days since 1970-01-01
    uint32_t y = 1970;
    while (true) {
      bool leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
      uint32_t dy = leap ? 366 : 365;
      if (days < dy) break;
      days -= dy; y++;
    }
    static const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    bool leap = (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0));
    uint32_t mo = 1;
    for (; mo <= 12; mo++) {
      uint32_t d = dim[mo-1] + (mo == 2 && leap ? 1 : 0);
      if (days < d) break;
      days -= d;
    }
    if (_utc_offset == 0) {
      snprintf(buf, len, "%04lu-%02lu-%02lu %02lu:%02lu:%02lu UTC",
        (unsigned long)y, (unsigned long)mo, (unsigned long)(days+1),
        (unsigned long)h, (unsigned long)m, (unsigned long)s);
    } else {
      snprintf(buf, len, "%04lu-%02lu-%02lu %02lu:%02lu:%02lu %s%d",
        (unsigned long)y, (unsigned long)mo, (unsigned long)(days+1),
        (unsigned long)h, (unsigned long)m, (unsigned long)s,
        tz_label, tz_abs);
    }
  }

  void buildChannel(const uint8_t* key, mesh::GroupChannel& ch) {
    mesh::Utils::sha256(ch.hash, PATH_HASH_SIZE, key, CIPHER_KEY_SIZE);
    memset(ch.secret, 0, sizeof(ch.secret));
    memcpy(ch.secret, key, CIPHER_KEY_SIZE);
  }

  // Send a single formatted message to a channel
  void sendGroupMsg(const uint8_t* channel_key, const char* text, uint32_t delay_ms) {
    mesh::GroupChannel ch;
    buildChannel(channel_key, ch);

    uint8_t data[5 + 160];
    uint32_t ts = getRTCClock()->getCurrentTime();
    memcpy(data, &ts, 4);
    data[4] = 0x00;
    int body_len = snprintf((char*)&data[5], 160, "%s", text);
    if (body_len <= 0) return;

    mesh::Packet* pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_TXT, ch, data, 5 + body_len);
    if (pkt) sendFlood(pkt, delay_ms);
  }

  /* ========================== NWS Logic ========================== */

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);
    alertIf(batt_voltage < 3.4f, extreme_weather, HIGH_PRI_ALERT, "WX Node: Battery critical!");
    alertIf(batt_voltage < 3.6f, severe_weather,  LOW_PRI_ALERT,  "WX Node: Battery low");
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago,
                      MinMaxAvg dest[], int max_num) override { return 0; }

  bool handleCustomCommand(uint32_t sender_timestamp,
                           char* command, char* reply) override {
    if (strcmp(command, "help") == 0) {
      Serial.println("--- NWS Node Commands ---");
      Serial.println("nws status               - ETH/poll/alert stats");
      Serial.println("nws poll                 - trigger immediate poll");
      Serial.println("nws test                 - broadcast test alert to severe channel");
      Serial.println("nws zone                 - show current zones");
      Serial.println("nws zone <z1,z2,...>     - set zones (e.g. WIZ038,WIZ039)");
      Serial.println("nws proxy                - show NWS proxy host:port");
      Serial.println("nws proxy <host> [port]  - set proxy (hostname or IP), persisted");
      Serial.println("nws utcoffset            - show UTC offset");
      Serial.println("nws utcoffset <hours>    - set UTC offset (e.g. -5 for CDT, -6 for CST)");
      Serial.println("nws severity             - show min broadcast severity");
      Serial.println("nws severity <level>     - set level: all/minor/moderate/severe/extreme (default: moderate)");
      Serial.println("display timeout          - show display auto-off timeout");
      Serial.println("display timeout <secs>   - set auto-off (0 or 'off' = always on)");
      Serial.println("nws hashtag              - show alerts hashtag channel");
      Serial.println("nws hashtag <name>       - set alerts hashtag channel (key derived from name)");
      Serial.println("nws channel              - show hashtag channel + derived key");
      Serial.println("nws announce             - send weekly announcement now");
      Serial.println("--- Uptime Kuma Commands ---");
      Serial.println("uptime status            - show UK config");
      Serial.println("uptime on / off          - enable/disable monitoring");
      Serial.println("uptime host <hostname>   - set host (DNS supported)");
      Serial.println("uptime port <port>       - set port (default 3001)");
      Serial.println("uptime path <path>       - set push path (default /api/push/)");
      Serial.println("uptime token <token>     - set push token");
      Serial.println("uptime interval <secs>   - set push interval (default 300)");
      Serial.println("uptime test              - send immediate push");
      strcpy(reply, "See Serial output for help");
      return true;
    }

    if (strcmp(command, "nws poll") == 0) {
      _next_nws_poll = 0;
      _poll_requested = true;   // NWS_MANUAL_POLL gate
      strcpy(reply, "NWS poll triggered");
      return true;
    }


    if (strcmp(command, "nws status") == 0) {
      snprintf(reply, 160, "ETH:%s Polls:%lu Sent:%lu Zone:%s",
        (_nws && _nws->isReady()) ? "OK" : "DOWN",
        (unsigned long)_polls_total,
        (unsigned long)_alerts_sent_total,
        _nws ? _nws->getZone() : NWS_ZONE);
      return true;
    }

    if (strcmp(command, "nws alerts") == 0) {
      if (!_nws || _nws->getNumAlerts() == 0) {
        strcpy(reply, "No active alerts");
      } else {
        char msg[160];
        _nws->formatForMesh(0, msg, sizeof(msg));
        snprintf(reply, 160, "%d active. Latest: %s", _nws->getNumAlerts(), msg);
      }
      return true;
    }

    if (strcmp(command, "nws clear") == 0) {
      if (_nws) _nws->clearSentHistory();
      strcpy(reply, "Sent history cleared");
      return true;
    }

    if (strcmp(command, "nws test") == 0) {
      // Demonstrate the live summary pipeline: a sample NWS-format description run
      // through the same extractSummary() a real alert uses, so you can eyeball the
      // notification format without waiting on real weather.
      static const char* sample_desc =
        "* WHAT...Strong thunderstorms producing wind gusts up to 50 mph and "
        "pea size hail.\n* WHERE...Outagamie County.\n* WHEN...Until 515 PM CDT.";
      char summary[160];
      NWSClient::extractSummary(sample_desc, summary, sizeof(summary));
      char issued[20];
      NWSClient::formatIssued("2026-06-24T15:00:00-05:00", issued, sizeof(issued));
      char meta[80];
      snprintf(meta, sizeof(meta), "Issued %s by NWS Green Bay WI", issued);
      Serial.print("[NWS-TEST] 2/3: "); Serial.println(summary);
      Serial.print("[NWS-TEST] 3/3: "); Serial.println(meta);
      broadcastWeatherAlert("Special Weather Statement", summary, meta, /*is_test=*/true);
      strcpy(reply, "Test alert sent (3 parts, each tagged [TEST])");
      return true;
    }

    // nws zone          → show current zones
    // nws zone <zones>  → set zones and persist
    if (strcmp(command, "nws zone") == 0) {
      snprintf(reply, 160, "Zone: %s", _nws ? _nws->getZone() : NWS_ZONE);
      return true;
    }
    if (strncmp(command, "nws zone ", 9) == 0) {
      const char* new_zone = command + 9;
      if (_nws) { _nws->setZone(new_zone); saveNWSPrefs(); }
      snprintf(reply, 160, "Zone set: %s", new_zone);
      return true;
    }

    // nws proxy              → show host:port
    // nws proxy <host> [port] → set proxy host (hostname or IP) and optional port, persist
    if (strcmp(command, "nws proxy") == 0) {
      snprintf(reply, 160, "Proxy: %s:%u", _nws ? _nws->getProxyHost() : NWS_PROXY_HOST,
               _nws ? _nws->getProxyPort() : NWS_PROXY_PORT);
      return true;
    }
    if (strncmp(command, "nws proxy ", 10) == 0 && _nws) {
      char host[64]; const char* a = command + 10;
      int i = 0; while (*a && *a != ' ' && i < (int)sizeof(host) - 1) host[i++] = *a++;
      host[i] = 0;
      uint16_t port = _nws->getProxyPort();          // keep current port unless one is given
      while (*a == ' ') a++;
      if (*a) port = (uint16_t)atoi(a);
      _nws->setProxy(host, port);
      saveNWSPrefs();
      snprintf(reply, 160, "Proxy set: %s:%u (applies next poll)", _nws->getProxyHost(), _nws->getProxyPort());
      return true;
    }

    if (strcmp(command, "nws utcoffset") == 0) {
      snprintf(reply, 160, "UTC offset: %+d", (int)_utc_offset);
      return true;
    }
    if (strncmp(command, "nws utcoffset ", 14) == 0) {
      _utc_offset = (int8_t)atoi(command + 14);
      saveNWSPrefs();
      snprintf(reply, 160, "UTC offset set: %+d", (int)_utc_offset);
      return true;
    }

    if (strcmp(command, "display timeout") == 0) {
      if (_display_data.display_timeout_secs == 0)
        strcpy(reply, "Display timeout: always on");
      else
        snprintf(reply, 160, "Display timeout: %lus", (unsigned long)_display_data.display_timeout_secs);
      return true;
    }
    if (strncmp(command, "display timeout ", 16) == 0) {
      const char* val = command + 16;
      if (strcmp(val, "off") == 0 || strcmp(val, "0") == 0) {
        _display_data.display_timeout_secs = 0;
        strcpy(reply, "Display always on");
      } else {
        _display_data.display_timeout_secs = (uint32_t)atol(val);
        snprintf(reply, 160, "Display timeout: %lus", (unsigned long)_display_data.display_timeout_secs);
      }
      saveNWSPrefs();
      return true;
    }

    // nws severity  → show/set minimum broadcast severity level
    if (strcmp(command, "nws severity") == 0) {
      snprintf(reply, 160, "Min severity: %s (%d)", severityLevelName(_min_severity), (int)_min_severity);
      return true;
    }
    if (strncmp(command, "nws severity ", 13) == 0) {
      uint8_t level = parseSeverityLevel(command + 13);
      _min_severity = level;
      if (_nws) _nws->setMinSeverity(_min_severity);
      saveNWSPrefs();
      snprintf(reply, 160, "Min severity: %s (%d)", severityLevelName(_min_severity), (int)_min_severity);
      return true;
    }

    // nws hashtag          → show alerts hashtag channel
    // nws hashtag <name>   → set alerts hashtag channel (key re-derived), persist
    if (strcmp(command, "nws hashtag") == 0) {
      snprintf(reply, 160, "Hashtag channel: %s", _severe_hashtag);
      return true;
    }
    if (strncmp(command, "nws hashtag ", 12) == 0) {
      const char* name = command + 12;
      while (*name == ' ') name++;
      if (!*name) { strcpy(reply, "Error: no name given"); return true; }
      applySevereHashtag(name);
      saveNWSPrefs();
      snprintf(reply, 160, "Hashtag channel set: %s", _severe_hashtag);
      return true;
    }

    // nws channel  → show the hashtag channel and its derived key
    if (strcmp(command, "nws channel") == 0) {
      char hex[33];
      mesh::Utils::toHex(hex, _severe_key, CIPHER_KEY_SIZE);
      for (int i = 0; hex[i]; i++) hex[i] = tolower(hex[i]);
      snprintf(reply, 160, "%s (add as hashtag channel) key=%s", _severe_hashtag, hex);
      return true;
    }

    // nws announce  → immediately send weekly public channel announcement
    if (strcmp(command, "nws announce") == 0) {
      announceChannelToPublic();
      _next_weekly_announce = futureMillis(7UL * 24 * 60 * 60 * 1000);
      strcpy(reply, "Channel announcement sent");
      return true;
    }

    if (strcmp(command, "uptime status") == 0) {
      snprintf(reply, 160, "%s host:%s port:%u path:%s token:%s interval:%lus last:%s",
        _uk_enabled ? "ON" : "OFF", _uk_host[0] ? _uk_host : "?",
        (unsigned)_uk_port, _uk_path,
        _uk_token[0] ? "(set)" : "(not set)",
        (unsigned long)(_uk_interval_ms / 1000),
        _uk_last_ok ? "OK" : "FAIL");
      return true;
    }

    if (strcmp(command, "uptime on") == 0) {
      _uk_enabled = true; saveUptimePrefs();
      _display_data.uk_enabled = _uk_enabled;
      strcpy(reply, "Uptime Kuma monitoring enabled");
      return true;
    }
    if (strcmp(command, "uptime off") == 0) {
      _uk_enabled = false; saveUptimePrefs();
      _display_data.uk_enabled = _uk_enabled;
      strcpy(reply, "Uptime Kuma monitoring disabled");
      return true;
    }

    if (strncmp(command, "uptime host ", 12) == 0) {
      strncpy(_uk_host, command + 12, sizeof(_uk_host) - 1);
      _uk_host[sizeof(_uk_host) - 1] = 0;
      saveUptimePrefs();
      strncpy(_display_data.uk_host, _uk_host, sizeof(_display_data.uk_host) - 1);
      snprintf(reply, 160, "Host set: %s", _uk_host);
      return true;
    }
    if (strncmp(command, "uptime port ", 12) == 0) {
      _uk_port = (uint16_t)atoi(command + 12);
      saveUptimePrefs();
      _display_data.uk_port = _uk_port;
      snprintf(reply, 160, "Port set: %u", (unsigned)_uk_port);
      return true;
    }
    if (strncmp(command, "uptime token ", 13) == 0) {
      strncpy(_uk_token, command + 13, sizeof(_uk_token) - 1);
      _uk_token[sizeof(_uk_token) - 1] = 0;
      saveUptimePrefs();
      strcpy(reply, "Token set");
      return true;
    }
    if (strncmp(command, "uptime path ", 12) == 0) {
      strncpy(_uk_path, command + 12, sizeof(_uk_path) - 1);
      _uk_path[sizeof(_uk_path) - 1] = 0;
      saveUptimePrefs();
      snprintf(reply, 160, "Path set: %s", _uk_path);
      return true;
    }
    if (strncmp(command, "uptime interval ", 16) == 0) {
      _uk_interval_ms = (uint32_t)atoi(command + 16) * 1000;
      saveUptimePrefs();
      snprintf(reply, 160, "Interval set: %lus", (unsigned long)(_uk_interval_ms / 1000));
      return true;
    }

    if (strcmp(command, "uptime test") == 0) {
      pushToUptimeKuma();
      strcpy(reply, _uk_last_ok ? "Push OK" : "Push FAILED");
      return true;
    }

    if (strcmp(command, "nws ping") == 0) {
      if (!_nws || !_nws->isReady()) { strcpy(reply, "ETH not ready"); return true; }
      EthernetClient testClient;
      testClient.setConnectionTimeout(10000);
      IPAddress proxy;
      if (!_nws->resolveProxy(proxy)) { strcpy(reply, "Proxy DNS resolve failed"); return true; }
      int result = testClient.connect(proxy, _nws->getProxyPort());
      if (result) {
        testClient.println("GET /health HTTP/1.1");
        testClient.print("Host: "); testClient.println(_nws->getProxyHost());
        testClient.println("Connection: close");
        testClient.println();
        unsigned long timeout = millis();
        while (!testClient.available() && millis() - timeout < 5000) delay(10);
        String response = "";
        while (testClient.available()) {
          char c = testClient.read();
          if (response.length() < 100) response += c;
        }
        testClient.stop();
        snprintf(reply, 160, "OK: %s", response.c_str());
      } else {
        testClient.stop();
        strcpy(reply, "Connection failed");
      }
      return true;
    }

    return false;
  }

  /* ========================== Mesh Broadcast ========================== */

  // Once a week: tell the public channel where the alerts live. The alerts
  // channel is a hashtag channel, so the name IS the key — one message, no
  // secret to distribute.
  void announceChannelToPublic() {
    char msg[160];
    snprintf(msg, sizeof(msg),
      "%s: NWS weather alerts for the Appleton (ATW) area on %s - add it as a hashtag channel to join.",
      getDisplayName(), _severe_hashtag);
    sendGroupMsg(NWS_PUB_CHANNEL_KEY, msg, getRNG()->nextInt(500, 2000));

    Serial.println("[NWS] Weekly channel announcement sent to public channel");
  }

  // Send one part of a multi-part alert: "<name>: (i/total) <text>", trimmed at a
  // word boundary to the 163-char channel body limit.
  void sendAlertPart(const char* text, int idx, int total, uint32_t delay) {
    const int MAX_BODY = 163;
    char body[176];
    int n = snprintf(body, sizeof(body), "%s: (%d/%d) %s",
                     getDisplayName(), idx, total, text);
    if (n > MAX_BODY) {                 // too long -> trim back to a word boundary
      body[MAX_BODY] = 0;
      char* last_space = strrchr(body, ' ');
      if (last_space && last_space > body) *last_space = 0;
    }
    sendGroupMsg(_severe_key, body, delay);
  }

  // Broadcast a 3-part alert to the severe weather channel, staggered ~4s apart and
  // tagged (1/3)(2/3)(3/3) so the receiver can see they belong to one alert:
  //   1/3 header   "[Severity] Event"
  //   2/3 detail   distilled hazard summary
  //   3/3 meta     "Issued <date/time> by <office>"
  void broadcastWeatherAlert(const char* header, const char* detail, const char* meta,
                             bool is_test = false) {
    uint32_t base_delay = getRNG()->nextInt(500, 2000);
    if (is_test) {
      // A test must be unmistakable: tag EVERY part with [TEST], so no single part
      // can be mistaken for a real alert if the others are missed on the mesh.
      char h[96], d[176], m[96];
      snprintf(h, sizeof(h), "[TEST] %s", header);
      snprintf(d, sizeof(d), "[TEST] %s", detail);
      snprintf(m, sizeof(m), "[TEST] %s", meta);
      sendAlertPart(h, 1, 3, base_delay);
      sendAlertPart(d, 2, 3, base_delay + NWS_ALERT_PART_GAP_MS);
      sendAlertPart(m, 3, 3, base_delay + 2 * NWS_ALERT_PART_GAP_MS);
    } else {
      sendAlertPart(header, 1, 3, base_delay);
      sendAlertPart(detail, 2, 3, base_delay + NWS_ALERT_PART_GAP_MS);
      sendAlertPart(meta,   3, 3, base_delay + 2 * NWS_ALERT_PART_GAP_MS);
    }

    _alerts_sent_total++;
    _display_data.alerts_sent_total = _alerts_sent_total;
    strncpy(_display_data.last_header, header, sizeof(_display_data.last_header) - 1);
    Serial.print("[NWS-TX] "); Serial.println(header);
  }

public:
  /* ========================== Main Loop ========================== */

  void loop() {
    SensorMesh::loop();

    // Ethernet link/DHCP (incl. late cable plug-in and recovery) is managed by the RAK13800
    // interface's loop() in main.cpp — no eth retry needed here. Just wait until it's up.
    if (!_nws || !_nws->isReady()) return;

    // Clear alert history every 6 hours to allow re-alerting
    if (_last_sent_history_clear == 0 || millisHasNowPassed(_last_sent_history_clear)) {
      _nws->clearSentHistory();
      _last_sent_history_clear = futureMillis(6UL * 60 * 60 * 1000);
    }

    // Boot announcement — 30s after first successful NWS poll
    if (!_boot_announced && _boot_announce_at > 0 && millisHasNowPassed(_boot_announce_at)) {
      _boot_announced = true;
      char ts[32], msg[160];
      formatTimestamp(ts, sizeof(ts), getRTCClock()->getCurrentTime());
      snprintf(msg, sizeof(msg), "%s: Node online. Connected to NWS. %s",
        getDisplayName(), ts);
      sendGroupMsg(_severe_key, msg, getRNG()->nextInt(500, 2000));
      Serial.println("[NWS] Boot announcement sent to severe channel");
    }

    // Weekly public channel announcement — first one 10 minutes after boot
    if (_next_weekly_announce == 0) {
      _next_weekly_announce = futureMillis(10UL * 60 * 1000);
    } else if (millisHasNowPassed(_next_weekly_announce)) {
      announceChannelToPublic();
      _next_weekly_announce = futureMillis(7UL * 24 * 60 * 60 * 1000);
    }

    // Uptime Kuma heartbeat push
    if (_uk_enabled && _next_uk_push > 0 && millisHasNowPassed(_next_uk_push)) {
      pushToUptimeKuma();
      _next_uk_push = futureMillis(_uk_interval_ms);
    }

    // NWS polls
#ifdef NWS_MANUAL_POLL
    // Debug/iteration build: never auto-poll (a 30s+ blocking TLS handshake stalls USB and
    // makes serial flashing impossible). Poll only on an explicit `nws poll`, so the node
    // stays idle and serial-flashable between tests.
    if (_poll_requested) {
      _poll_requested = false;
#else
    if (_next_nws_poll == 0 || millisHasNowPassed(_next_nws_poll)) {
#endif
      _polls_total++;
      _nws->setNow(getRTCClock()->getCurrentTime());   // for TLS cert-date validation
      int new_alerts = _nws->pollAlerts();
      // Report the direct-HTTPS TLS health over the mesh, but ONLY on a state change (first OK
      // after boot, a new failure, or recovery) — polls run every 2 min, so broadcasting every
      // poll would spam the shared public channel forever. Read it from a companion on the NWS
      // public channel: meshcore-cli -t <ip> ... . Severe alerts go out on the separate path.
      {
        int8_t st = (new_alerts < 0) ? 0 : 1;
        if (st != _wx_state) {
          char m[100]; uint32_t t = getRTCClock()->getCurrentTime();
          if (st == 0) snprintf(m, sizeof(m), "wxdiag: TLS FAIL t=%lu", (unsigned long)t);
          else snprintf(m, sizeof(m), "wxdiag: TLS OK alerts=%d t=%lu", new_alerts, (unsigned long)t);
          sendGroupMsg(NWS_PUB_CHANNEL_KEY, m, 1200);
          _wx_state = st;
          Serial.print("[NWS] mesh-report(change): "); Serial.println(m);
        } else {
          Serial.print("[NWS] poll TLS "); Serial.print(st ? "OK" : "FAIL");
          Serial.println(" (no wxdiag change)");
        }
      }
      if (new_alerts < 0) new_alerts = 0;   // -1 = TLS failure; clamp for the logic below
      _display_data.polls_total = _polls_total;
      _display_data.eth_ready = _nws->isReady();
      _display_data.active_alerts = _nws->getNumAlerts();
      if (!_boot_announced && _nws->isReady() && _boot_announce_at == 0) {
        _boot_announce_at = futureMillis(30000);  // 30s after first successful poll
        Serial.println("[NWS] Boot announcement scheduled in 30s");
        if (_uk_enabled && _next_uk_push == 0)
          _next_uk_push = futureMillis(35000);  // first UK push 5s after boot announcement
      }
      if (new_alerts > 0) {
        _pending_alert_idx = 0;
        _has_pending_alerts = true;
        _next_mesh_broadcast = futureMillis(2000);
      }
      _next_nws_poll = futureMillis(NWS_POLL_INTERVAL);
    }

    // Sequential alert broadcast — 15s gap (accounts for 4s detail delay + buffer)
    if (_has_pending_alerts && millisHasNowPassed(_next_mesh_broadcast)) {
      bool found = false;
      while (_pending_alert_idx < _nws->getNumAlerts()) {
        if (_nws->isAlertNew(_pending_alert_idx) && _nws->isAlertAboveThreshold(_pending_alert_idx)) {
          char header[80];
          char detail[160];
          char meta[80];
          _nws->formatHeader(_pending_alert_idx, header, sizeof(header));
          _nws->formatDetail(_pending_alert_idx, detail, sizeof(detail));
          _nws->formatMeta(_pending_alert_idx, meta, sizeof(meta));
          broadcastWeatherAlert(header, detail, meta);
          _nws->markAlertSent(_pending_alert_idx);
          _pending_alert_idx++;
          found = true;
          _next_mesh_broadcast = futureMillis(NWS_ALERT_GAP_MS);  // let this alert's parts clear the mesh first
          break;
        }
        _pending_alert_idx++;
      }
      if (!found) _has_pending_alerts = false;
    }
  }
};
