#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------------
// nRF52 serial-DFU host (Phase 7 nRF52-OTA, WiFi path).
//
// Drives the (dual-backend OTAFIX) bootloader's UART serial-DFU — the SDK11
// dfu_transport_serial protocol (SLIP + HCI framing + START/INIT/DATA/STOP DFU
// packets). This is a C++ port of adafruit-nrfutil's `dfu serial` host, so the
// RAK2305 ESP32 can flash the nRF52's app over the inter-chip Serial1 (the same
// line that carried `MCPKT`/`esp` in normal operation).
//
// Protocol (verified against nordicsemi/dfu/dfu_transport_serial.py):
//   packet = 0xC0 | SLIP-esc( hdr[4] + dfuData + crc16LE[2] ) | 0xC0
//   hdr[0] = seq(3b) | ack=(seq+1)%8 (3b) | DIP<<6 | RP<<7   (DIP=RP=1)
//   hdr[1] = HCI_TYPE(14) | (len & 0x0F)<<4 ;  hdr[2] = (len & 0x0FF0)>>4
//   hdr[3] = (~(hdr0+hdr1+hdr2) + 1) & 0xFF   (header checksum)
//   crc16  = Nordic variant, init 0xFFFF, over hdr+data, appended little-endian
//   SLIP   = 0xC0 -> 0xDB 0xDC ; 0xDB -> 0xDB 0xDD
//   dfuData: START=int32(3)+int32(mode)+int32(sd)+int32(bl)+int32(app);
//            INIT=int32(1)+initPacket; DATA=int32(4)+<=512B; STOP=int32(5)
//   Reliable packets: after each, the target returns a SLIP ACK frame; the ref
//   host just drains it to stay in sync (does not strictly verify the seq).
//
// Precondition: the nRF52 must already be in UART DFU mode (app issued
// `reboot uartdfu` / GPREGRET 0x4f) before calling doDfu().
// ---------------------------------------------------------------------------

#ifndef NRF52DFU_LOG
  #define NRF52DFU_LOG(...) do{}while(0)
#endif

class Nrf52SerialDfu {
public:
  explicit Nrf52SerialDfu(Stream& uart) : _uart(uart), _seq(0) {}

  // Diagnostics from the last doDfu() run (for the WiFi-path result report, since
  // the ESP32 is USB-less). stage: 0=ok, 1=START no-ack, 2=INIT no-ack,
  // 3=DATA no-ack. failOff = byte offset of the failing DATA packet. dataAcks =
  // count of DATA packets ACKed. retransmits = total reliable-packet resends.
  int      lastStage   = -1;
  uint32_t failOff     = 0;
  uint32_t dataAcks    = 0;
  uint32_t retransmits = 0;
  // Raw-wire diagnostics: hex of bytes the bootloader sent during the START and
  // INIT ack-waits, plus the uncapped byte-count seen during the INIT phase.
  char     dbgStartHex[48] = {0};
  char     dbgInitHex[48]  = {0};
  uint32_t initRxBytes     = 0;

  // initPacket = the Nordic .dat init packet; firmware = the raw app .bin.
  // Returns true if START->INIT->DATA->STOP all completed (target then reboots).
  bool doDfu(const uint8_t* initPacket, size_t initLen,
             const uint8_t* firmware, size_t fwLen) {
    _seq = 0;
    lastStage = -1; failOff = 0; dataAcks = 0; retransmits = 0;
    dbgStartHex[0] = 0; dbgInitHex[0] = 0; initRxBytes = 0;
    while (_uart.available()) _uart.read();          // drain stale rx

    // 1) START (app-only: mode=4, sd=bl=0, app=fwLen)
    uint8_t start[20];
    putLE32(start + 0, DFU_START_PACKET);
    putLE32(start + 4, DFU_UPDATE_MODE_APP);
    putLE32(start + 8, 0);                            // softdevice size
    putLE32(start + 12, 0);                           // bootloader size
    putLE32(start + 16, (uint32_t)fwLen);             // app size
    NRF52DFU_LOG("[dfu] START app=%u\n", (unsigned)fwLen);
    _capOn = true; _capn = 0; _capTotal = 0;
    bool startOk = sendReliable(start, sizeof(start), START_ACK_TIMEOUT_MS);
    toHex(_cap, _capn, dbgStartHex, sizeof(dbgStartHex));
    if (!startOk) { lastStage = 1; _capOn = false; return false; }
    // START kicks a full app-bank erase; the nRF52 CPU is BLOCKED in NVMC for the
    // duration (its UART ISR is starved -> any byte sent now is dropped). Wait it
    // out before INIT, exactly like the ref host: ~89.7ms per 4K page erased.
    delay(eraseWaitMs((uint32_t)fwLen));

    // 2) INIT packet (.dat) + 2 bytes of 0x0000 padding. The ref host appends this
    //    padding (int16 0x0000) after the init packet; without it the SDK11
    //    init-packet handler rejects it and never ACKs.
    if (4 + initLen + 2 > sizeof(_scratch)) { lastStage = 2; return false; }
    putLE32(_scratch, DFU_INIT_PACKET);
    memcpy(_scratch + 4, initPacket, initLen);
    _scratch[4 + initLen]     = 0x00;
    _scratch[4 + initLen + 1] = 0x00;
    _capn = 0; _capTotal = 0;
    bool initOk = sendReliable(_scratch, 4 + initLen + 2, ACK_TIMEOUT_MS);
    toHex(_cap, _capn, dbgInitHex, sizeof(dbgInitHex));
    initRxBytes = _capTotal;
    _capOn = false;
    if (!initOk) { lastStage = 2; return false; }

    // 3) DATA in <=512B chunks
    int chunkIdx = 0;
    for (size_t off = 0; off < fwLen; off += DFU_DATA_CHUNK) {
      size_t n = fwLen - off; if (n > DFU_DATA_CHUNK) n = DFU_DATA_CHUNK;
      putLE32(_scratch, DFU_DATA_PACKET);
      memcpy(_scratch + 4, firmware + off, n);
      if (!sendReliable(_scratch, 4 + n, ACK_TIMEOUT_MS)) { lastStage = 3; failOff = (uint32_t)off; return false; }
      dataAcks++;
      // Every 8th 512B frame completes a 4K flash page -> the nRF52 blocks to write
      // it. Give it a breather (the ref host does the same) to avoid ISR overrun.
      if ((++chunkIdx % 8) == 0) delay(PAGE_WRITE_MS);
    }

    // 4) STOP_DATA -> validate + activate + reboot (no ACK; the target reboots)
    uint8_t stop[4];
    putLE32(stop, DFU_STOP_DATA_PACKET);
    sendHciPacket(stop, sizeof(stop));
    NRF52DFU_LOG("[dfu] STOP sent; target should validate+reboot\n");
    lastStage = 0;
    return true;
  }

private:
  static const uint32_t DFU_INIT_PACKET      = 1;
  static const uint32_t DFU_START_PACKET     = 3;
  static const uint32_t DFU_DATA_PACKET      = 4;
  static const uint32_t DFU_STOP_DATA_PACKET = 5;
  static const uint32_t DFU_UPDATE_MODE_APP  = 4;
  static const uint8_t  HCI_PACKET_TYPE      = 14;
  static const size_t   DFU_DATA_CHUNK       = 512;
  static const uint32_t ACK_TIMEOUT_MS       = 3000;   // generous: covers per-page lazy erase
  static const uint32_t START_ACK_TIMEOUT_MS = 6000;   // START may kick a full app-bank erase
  static const uint32_t PAGE_ERASE_MS        = 90;     // ~89.7ms per 4K page (nRF52840 max)
  static const uint32_t PAGE_WRITE_MS        = 110;    // breather after each 4K page of DATA
  static const int      MAX_TX_ATTEMPTS      = 8;      // reliable-packet resends before giving up

  // Post-START erase wait, mirroring the ref host: one 4K page per (fwLen/4096)+1,
  // ~90ms each; floor 500ms. The nRF52 blocks its UART during this, so INIT must wait.
  static uint32_t eraseWaitMs(uint32_t fwLen) {
    uint32_t pages = (fwLen / 4096) + 1;
    uint32_t ms = pages * PAGE_ERASE_MS;
    return ms < 500 ? 500 : ms;
  }

  Stream&  _uart;
  uint8_t  _seq;
  uint8_t  _scratch[4 + DFU_DATA_CHUNK];               // DFU packet build buffer
  // Raw-wire capture (diagnostic): filled by waitAck when _capOn.
  uint8_t  _cap[20];
  int      _capn    = 0;
  uint32_t _capTotal = 0;
  bool     _capOn   = false;

  static inline void putLE32(uint8_t* p, uint32_t v) {
    p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = (v >> 24) & 0xFF;
  }

  // Nordic CRC-16 (matches nordicsemi/dfu/crc16.py). Streamable via crcInit.
  static uint16_t crc16(const uint8_t* data, size_t len, uint16_t crcInit) {
    uint32_t crc = crcInit;
    for (size_t i = 0; i < len; i++) {
      crc = ((crc >> 8) & 0x00FF) | ((crc << 8) & 0xFF00);
      crc ^= data[i];
      crc ^= (crc & 0x00FF) >> 4;
      crc ^= (crc << 8) << 4;
      crc ^= ((crc & 0x00FF) << 4) << 1;
    }
    return (uint16_t)(crc & 0xFFFF);
  }

  void slipWrite(const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; i++) {
      uint8_t b = d[i];
      if (b == 0xC0)      { _uart.write((uint8_t)0xDB); _uart.write((uint8_t)0xDC); }
      else if (b == 0xDB) { _uart.write((uint8_t)0xDB); _uart.write((uint8_t)0xDD); }
      else                { _uart.write(b); }
    }
  }

  // Frame + send one HCI packet carrying `data`, using the CURRENT _seq (does not
  // advance it — so a retransmit resends an identical packet, matching the ref host).
  void sendFramed(const uint8_t* data, size_t len) {
    uint8_t hdr[4];
    hdr[0] = (uint8_t)((_seq & 0x07) | ((((_seq + 1) % 8) & 0x07) << 3) | (1 << 6) | (1 << 7));
    hdr[1] = (uint8_t)(HCI_PACKET_TYPE | ((len & 0x000F) << 4));
    hdr[2] = (uint8_t)((len & 0x0FF0) >> 4);
    hdr[3] = (uint8_t)(~((uint32_t)hdr[0] + hdr[1] + hdr[2]) + 1);
    uint16_t crc = crc16(hdr, 4, 0xFFFF);
    crc = crc16(data, len, crc);
    uint8_t crcb[2] = { (uint8_t)(crc & 0xFF), (uint8_t)((crc >> 8) & 0xFF) };

    _uart.write((uint8_t)0xC0);
    slipWrite(hdr, 4);
    slipWrite(data, len);
    slipWrite(crcb, 2);
    _uart.write((uint8_t)0xC0);
    _uart.flush();
  }

  // Advance seq and send one packet (fire-and-forget; used for STOP which has no ACK).
  void sendHciPacket(const uint8_t* data, size_t len) {
    _seq = (_seq + 1) % 8;
    sendFramed(data, len);
  }

  // Reliable send: advance seq once, then (re)transmit the SAME packet until the
  // target ACKs or MAX_TX_ATTEMPTS is exhausted. Resends carry the same seq so a
  // dup (lost-ACK) is recognized by the bootloader's transport layer. This is what
  // lets the transfer survive a byte dropped while the nRF52's NVMC blocks its UART
  // ISR during a flash-page erase — the single biggest gap vs. the ref host.
  bool sendReliable(const uint8_t* data, size_t len, uint32_t timeoutMs) {
    _seq = (_seq + 1) % 8;
    for (int attempt = 0; attempt < MAX_TX_ATTEMPTS; attempt++) {
      sendFramed(data, len);
      if (waitAck(timeoutMs)) return true;
      retransmits++;
    }
    return false;
  }

  // Drain the target's ACK: wait for a full SLIP frame (two 0xC0). Returns false
  // on timeout (target not responding). Does not verify the ack seq (matches ref).
  // When _capOn, records every received byte (capped hex + uncapped count) for the
  // WiFi-path diagnostic breadcrumb — so we can see what the bootloader actually
  // sends for START vs INIT.
  bool waitAck(uint32_t timeoutMs) {
    uint32_t start = millis();
    int c0 = 0;
    while ((millis() - start) < timeoutMs) {
      while (_uart.available()) {
        uint8_t b = (uint8_t)_uart.read();
        if (_capOn) { _capTotal++; if (_capn < (int)sizeof(_cap)) _cap[_capn++] = b; }
        if (b == 0xC0) { if (++c0 >= 2) return true; }
      }
      delay(1);
    }
    return false;
  }

  static void toHex(const uint8_t* b, int n, char* out, int outCap) {
    static const char* H = "0123456789abcdef";
    int j = 0;
    for (int i = 0; i < n && j + 2 < outCap; i++) { out[j++] = H[b[i] >> 4]; out[j++] = H[b[i] & 0xF]; }
    out[j] = 0;
  }
};
