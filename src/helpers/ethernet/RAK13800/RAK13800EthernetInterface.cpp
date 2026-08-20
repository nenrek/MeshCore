#include "RAK13800EthernetInterface.h"
#include "../../nrf52/EthernetMac.h"
#include <SPI.h>
#include <EthernetUdp.h>

#define PIN_SPI1_MISO (29) // (0 + 29)
#define PIN_SPI1_MOSI (30) // (0 + 30)
#define PIN_SPI1_SCK (3)   // (0 + 3)

SPIClass ETHERNET_SPI_PORT(NRF_SPIM1, PIN_SPI1_MISO, PIN_SPI1_SCK, PIN_SPI1_MOSI);

#define PIN_ETHERNET_POWER_EN WB_IO2    // output, high to enable
#define PIN_ETHERNET_RESET 21
#define PIN_ETHERNET_SS 26

bool RAK13800EthernetInterface::begin() {

  // WB_IO2 (power enable) is already driven HIGH by early constructor
  // in RAK4631Board.cpp to support POE boot.
  // Skip hardware reset — the W5100S comes out of power-on reset cleanly,
  // and toggling reset kills the PHY link which breaks POE power.
#ifdef PIN_ETHERNET_RESET
        pinMode(PIN_ETHERNET_RESET, OUTPUT);
        digitalWrite(PIN_ETHERNET_RESET, HIGH);
#endif

  // generate mac address
  uint8_t mac[6];
  generateEthernetMac(mac);
  ETHERNET_DEBUG_PRINTLN(
      "Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X",
      mac[0],
      mac[1],
      mac[2],
      mac[3],
      mac[4],
      mac[5]);
  ETHERNET_SPI_PORT.begin();
  Ethernet.init(ETHERNET_SPI_PORT, PIN_ETHERNET_SS);

  memcpy(_mac, mac, sizeof(_mac));   // keep for deferred DHCP if the link comes up later

  // Use static IP if build flags are defined, otherwise DHCP
  #if defined(ETHERNET_STATIC_IP) && defined(ETHERNET_STATIC_GATEWAY) && defined(ETHERNET_STATIC_SUBNET) && defined(ETHERNET_STATIC_DNS)
  IPAddress ip(ETHERNET_STATIC_IP);
  IPAddress gateway(ETHERNET_STATIC_GATEWAY);
  IPAddress subnet(ETHERNET_STATIC_SUBNET);
  IPAddress dns(ETHERNET_STATIC_DNS);
  Ethernet.begin(mac, ip, dns, gateway, subnet);
  server.begin();
  _dhcp_up = true;   // static config: no DHCP wait
  ETHERNET_DEBUG_PRINTLN("Ethernet (static) up; listening on TCP port: %d", ETHERNET_TCP_PORT);
  #else
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    ETHERNET_DEBUG_PRINTLN("Ethernet hardware not found.");
    return false;    // no W5100S chip: ethernet unavailable, but the node still boots
  }
  // NON-BLOCKING DHCP: only request an address when the PHY link is actually up, so a
  // missing/unpatched cable can never stall boot — this node also serves BLE/USB, and
  // the upstream `Ethernet.begin(mac)` blocks (and with no link can hang far past its
  // DHCP timeout). loop() retries once a cable is plugged in.
  tryDhcp();
  #endif

  return true;   // always succeed so the node boots and the interface is registered
}

// Request DHCP only if the link is up; bounded so a slow server can't hang the node.
// Idempotent + safe to call repeatedly from loop() until we're up.
void RAK13800EthernetInterface::tryDhcp() {
  if (_dhcp_up) return;
  if (Ethernet.linkStatus() != LinkON) return;   // no cable/link yet -> defer, don't block
  ETHERNET_DEBUG_PRINTLN("Ethernet link up; requesting DHCP...");
  if (Ethernet.begin(_mac, 8000, 4000) != 0) {
    _dhcp_up = true;
    server.begin();
    ETHERNET_DEBUG_PRINT_IP("IP Address", Ethernet.localIP());
    ETHERNET_DEBUG_PRINTLN("listening on TCP port: %d", ETHERNET_TCP_PORT);
  } else {
    ETHERNET_DEBUG_PRINTLN("DHCP request failed; will retry");
  }
}

int RAK13800EthernetInterface::available() {
  return client.available();
}

int RAK13800EthernetInterface::read() {
  return client.read();
}

size_t RAK13800EthernetInterface::write(const uint8_t *buf, size_t size) {
  return client.write(buf, size);
}

bool RAK13800EthernetInterface::isConnected() const {
  return _isConnected;
}

void RAK13800EthernetInterface::loop() {

  // Not up yet (no link at boot / cable unplugged): poll for a link and DHCP when it
  // appears, throttled. Nothing to service until we have an IP + a listening server.
  if (!_dhcp_up) {
    if (millis() >= _next_dhcp_try) {
      _next_dhcp_try = millis() + 3000;
      tryDhcp();
    }
    return;
  }

  Ethernet.maintain();

  auto newClient = server.accept();
  if (newClient) {
    IPAddress remoteIp = newClient.remoteIP();
    uint16_t remotePort = newClient.remotePort();
    ETHERNET_DEBUG_PRINTLN("New client accepted %u.%u.%u.%u:%u", remoteIp[0], remoteIp[1], remoteIp[2], remoteIp[3], remotePort);
    if (client) {
      ETHERNET_DEBUG_PRINTLN("Closing previous client");
      client.stop();
    }
    client = newClient;
    onClientConnected();
  }

  _isConnected = client.connected();

}
