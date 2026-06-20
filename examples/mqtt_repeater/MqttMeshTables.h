#pragma once
// MeshTables subclass that fans each unique received packet out to MQTT
// exactly once. It piggybacks on the dedup table the dispatcher already calls:
// when hasSeen() returns false the packet is new, which is the moment we hand
// it to the bridge (RSSI/SNR are still the freshest values from the radio).
//
// The hook is non-blocking — onPacketReceived() only formats + enqueues; the
// actual UART/AT publishing happens later in EspAtMqtt::loop().

#include <helpers/SimpleMeshTables.h>
#include "EspAtMqtt.h"

class MqttMeshTables : public SimpleMeshTables {
  EspAtMqtt* _bridge;
public:
  MqttMeshTables() : _bridge(nullptr) {}
  void setBridge(EspAtMqtt* b) { _bridge = b; }

  bool hasSeen(const mesh::Packet* packet) override {
    bool already = SimpleMeshTables::hasSeen(packet);
    if (!already && _bridge) _bridge->onPacketReceived(packet);
    return already;
  }
};
