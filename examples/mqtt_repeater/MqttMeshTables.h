#pragma once
// MeshTables subclass that fans each unique received packet out to MQTT
// exactly once. It piggybacks on the dedup table the dispatcher already calls:
// 1.17 split the old check-and-mark hasSeen() into wasSeen() (check) + markSeen()
// (mark). The dispatcher always does `if (!wasSeen(pkt)) { markSeen(pkt); ... }`,
// so markSeen() fires exactly once per unique packet — the moment we hand it to
// the bridge (RSSI/SNR are still the freshest values from the radio).
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

  void markSeen(const mesh::Packet* packet) override {
    SimpleMeshTables::markSeen(packet);
    if (_bridge) _bridge->onPacketReceived(packet);  // fired once, when the packet is first seen (new)
  }
};
