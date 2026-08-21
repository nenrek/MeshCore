Import("env")
import os, re

# SSLClient's record buffer defaults to 2 KB (unsigned char m_iobuf[2048]) — too small to hold
# Let's Encrypt's ~4.5 KB cert chain, so the api.weather.gov handshake fails. Bump it to 16 KB.
# The lib has no build-flag for this, so we patch the source in libdeps before compiling.
# Runs at script load; the lib is already installed on any incremental build. On a *fresh* clean
# build the lib may not be present yet — just run `pio run` twice, or the committed firmware.hex
# already has the 16 KB buffer baked in.
def _patch():
    path = os.path.join(env.subst("$PROJECT_LIBDEPS_DIR"), env.subst("$PIOENV"),
                        "SSLClient", "src", "SSLClient.h")
    if not os.path.exists(path):
        print("[patch_sslclient] SSLClient not installed yet; skipping (run again after LDF)")
        return
    src = open(path).read()
    new = re.sub(r"unsigned char m_iobuf\[\d+\]", "unsigned char m_iobuf[16384]", src)
    if new != src:
        open(path, "w").write(new)
        print("[patch_sslclient] m_iobuf -> 16384")

_patch()
