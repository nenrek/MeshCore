#pragma once
#if defined(ESP32)
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "Nrf52SerialDfu.h"

// ---------------------------------------------------------------------------
// nRF52 OTA orchestrator (Phase 7, WiFi path) — runs on the RAK2305 ESP32.
//   1. download the new nRF52 <base>.bin + <base>.dat (Nordic init packet)
//   2. send "MCDFU" over the inter-chip UART -> nRF52 app enterUartDfu() (reset
//      into the bootloader's UART serial-DFU on Serial1)
//   3. drive the DFU via Nrf52SerialDfu over that same UART
//   4. report a one-line result
//
// The DFU is a blocking call, so the normal UartRadio loop is not serviced while
// it runs (the nRF52 is in the bootloader anyway); it resumes once the nRF52
// reboots onto the new app.
// ---------------------------------------------------------------------------

// CA bundle for HTTPS (same one the wifiobs OTA uses; embedded by PsychicMqttClient).
extern const uint8_t rootca_crt_bundle_start[] asm("_binary_src_certs_x509_crt_bundle_bin_start") __attribute__((weak));
extern const uint8_t rootca_crt_bundle_end[]   asm("_binary_src_certs_x509_crt_bundle_bin_end")   __attribute__((weak));

namespace nrf52dfu {

// GET `url` into a freshly-allocated buffer (PSRAM). Caller frees. Returns len or -1.
inline int downloadToBuffer(const String& url, uint8_t** out) {
  *out = nullptr;
  WiFiClientSecure sclient;
  WiFiClient pclient;
  const bool https = url.startsWith("https://");
  if (https) sclient.setCACertBundle(rootca_crt_bundle_start);
  (void)rootca_crt_bundle_end;

  HTTPClient http;
  bool begun = https ? http.begin(sclient, url) : http.begin(pclient, url);
  if (!begun) return -1;
  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return -1; }

  int total = http.getSize();                 // -1 if chunked/unknown
  size_t cap = (total > 0) ? (size_t)total : (512 * 1024);
  uint8_t* buf = (uint8_t*)ps_malloc(cap);
  if (!buf) buf = (uint8_t*)malloc(cap);
  if (!buf) { http.end(); return -1; }

  WiFiClient* stream = http.getStreamPtr();
  size_t got = 0;
  uint32_t lastData = millis();
  while (http.connected() && (total < 0 || got < (size_t)total)) {
    size_t avail = stream->available();
    if (avail) {
      if (got + avail > cap) break;           // overflow guard
      int n = stream->readBytes(buf + got, avail);
      if (n > 0) { got += n; lastData = millis(); }
    } else {
      if (millis() - lastData > 8000) break;  // stall timeout
      delay(2);
    }
  }
  http.end();
  if (total > 0 && got != (size_t)total) { free(buf); return -1; }
  *out = buf;
  return (int)got;
}

// Core flow: download baseUrl + ".bin"/".dat" into PSRAM, tell the nRF52 to enter
// UART DFU via `triggerMsg`, wait, and drive the DFU over the same UART. If the
// download fails and `failMsg` is non-null, send it (so a caller that's blocked
// waiting can un-block). The result is observed via the nRF52's version once it
// reboots, plus a best-effort breadcrumb GET on the payload host (debug aid).
inline void pullAndFlash(Stream& uart, const char* baseUrl,
                         const char* triggerMsg, const char* failMsg) {
  String base(baseUrl); base.trim();
  uint8_t* bin = nullptr; uint8_t* dat = nullptr;
  int binLen = downloadToBuffer(base + ".bin", &bin);
  int datLen = (binLen > 0) ? downloadToBuffer(base + ".dat", &dat) : -1;
  if (binLen <= 0 || datLen <= 0) {
    if (failMsg) { uart.print(failMsg); uart.print("\r\n"); uart.flush(); }  // abort: nRF52 keeps running its app
    if (bin) free(bin);
    if (dat) free(dat);
    return;
  }

  // Files staged. Tell the nRF52 to enter UART DFU (it resets), then flash it.
  uart.print(triggerMsg); uart.print("\r\n");
  uart.flush();
  delay(1800);                                        // nRF52 resets into the bootloader UART DFU

  Nrf52SerialDfu dfu(uart);
  bool ok = dfu.doDfu(dat, (size_t)datLen, bin, (size_t)binLen);
  free(bin); free(dat);

  // The ESP32 is USB-less, so report the DFU outcome by GETting a breadcrumb URL on
  // the SAME host the payload came from (it lands in that server's access log). Best-
  // effort; the nRF52 is meanwhile validating+rebooting (on success) or stuck in the
  // bootloader (on failure -> recover via UF2). stage: 0=ok 1=START 2=INIT 3=DATA.
  {
    // host prefix = everything up to the 3rd '/', i.e. "http://host:port"
    int slashes = 0; int cut = base.length();
    for (int i = 0; i < (int)base.length(); i++) {
      if (base[i] == '/') { if (++slashes == 3) { cut = i; break; } }
    }
    String rep = base.substring(0, cut);
    rep += "/DFURESULT_ok"; rep += ok ? "1" : "0";
    rep += "_stage"; rep += String(dfu.lastStage);
    rep += "_off";   rep += String(dfu.failOff);
    rep += "_acks";  rep += String(dfu.dataAcks);
    rep += "_rtx";   rep += String(dfu.retransmits);
    rep += "_sack";  rep += (dfu.dbgStartHex[0] ? dfu.dbgStartHex : "none");   // START ack bytes
    rep += "_irx";   rep += String(dfu.initRxBytes);                          // bytes seen during INIT
    rep += "_iack";  rep += (dfu.dbgInitHex[0] ? dfu.dbgInitHex : "none");    // INIT-phase bytes
    uint8_t* junk = nullptr;
    int rc = downloadToBuffer(rep, &junk);
    if (junk) free(junk);
    (void)rc;
  }
}

// Local path (nRF52-initiated): the nRF52 sent "MCPULL <baseUrl>" and is waiting in
// its trigger loop for "MCPULLED" (or "MCPULLFAIL" on a download failure).
inline void runDfuPull(Stream& uart, const char* baseUrl) {
  pullAndFlash(uart, baseUrl, "MCPULLED", "MCPULLFAIL");
}

// Remote path (MQTT-initiated): a fleet-control downlink asked us to flash the nRF52.
// The nRF52 is in its NORMAL loop, which handles the unsolicited "MCDFUNOW" line ->
// enterUartDfu(). No fail message (nothing is blocked waiting) — a download failure
// just no-ops and the nRF52 keeps running its current app.
inline void runDfuRemote(Stream& uart, const char* baseUrl) {
  pullAndFlash(uart, baseUrl, "MCDFUNOW", nullptr);
}

} // namespace nrf52dfu
#endif // ESP32
