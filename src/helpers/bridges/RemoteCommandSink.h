#pragma once
#include <stddef.h>

// Implemented by MyMesh, held by each MQTT bridge (cellular + WiFi). Executes a remote-command
// envelope received over the MQTT downlink and produces the ack payload. All security policy —
// replay guard, command allowlist, optional Ed25519 signature — lives in the implementation so
// both bridges share one code path, and the command runs through the same CLI a local console
// would use.
class RemoteCommandSink {
public:
  virtual ~RemoteCommandSink() {}
  // `envelope` = the raw downlink payload (JSON: {"id","seq","cmd","sig"}). Writes the ack JSON
  // (NUL-terminated, at most ack_size) into `ack`. Returns true if an ack should be published.
  // The implementation MUST publish the ack for disruptive commands before applying them, since
  // a command that restarts the bridge would otherwise drop the connection the ack travels on.
  virtual bool runRemoteCommand(const char* envelope, char* ack, size_t ack_size) = 0;
};
