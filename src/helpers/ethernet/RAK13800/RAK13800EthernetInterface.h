#pragma once

#include "../SerialEthernetInterface.h"
#include <SPI.h>
#include <RAK13800_W5100S.h>

class RAK13800EthernetInterface : public SerialEthernetInterface {

  bool _isConnected;
  bool _dhcp_up;                 // true once DHCP succeeded and the server is listening
  uint8_t _mac[6];              // kept so loop() can (re)request DHCP when a link appears
  unsigned long _next_dhcp_try;  // throttle for the deferred DHCP retry
  EthernetServer server;
  EthernetClient client;

  void tryDhcp();                // non-blocking: request DHCP only when the link is up

  public:
    RAK13800EthernetInterface() : server(EthernetServer(ETHERNET_TCP_PORT)) {
      _isConnected = false;
      _dhcp_up = false;
      _next_dhcp_try = 0;
    }

    bool begin();
    void loop() override;

    // BaseSerialInterface methods
    bool isConnected() const override;

    int available() override;
    int read() override;
    size_t write(const uint8_t *buf, size_t size) override;
};
