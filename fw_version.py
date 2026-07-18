# fw_version.py — embed a custom fleet FIRMWARE_VERSION on a plain `pio run`.
#
# PlatformIO pre-script. It reads the env's `custom_fw_version` option (e.g.
# "v1.16.0-mw.cell.1") and appends the short git commit hash, producing
#   v1.16.0-mw.cell.1+g1a2b3c   (or +g1a2b3c.dirty with uncommitted changes)
# then defines FIRMWARE_VERSION to it, overriding the #ifndef fallback in the
# example headers. `ver`, the app's version, and the MQTT status feed all read
# that macro, so this is the single lever for fleet version reporting.
#
# No-op when the env doesn't set custom_fw_version, so stock upstream envs are
# untouched. Relates each build to upstream (the vX.Y.Z base) while uniquely
# identifying our firmware (fleet.variant.rev) and pinning the exact commit.
Import("env")  # noqa: F821
import subprocess

base = (env.GetProjectOption("custom_fw_version", "") or "").strip()
if not base:
    print("fw_version: custom_fw_version not set — keeping default FIRMWARE_VERSION")
else:
    def _git(*args):
        try:
            return subprocess.check_output(["git"] + list(args),
                                           stderr=subprocess.DEVNULL).decode().strip()
        except Exception:
            return ""
    short = _git("rev-parse", "--short=7", "HEAD") or "nogit"
    dirty = ".dirty" if _git("status", "--porcelain") else ""
    version = "%s+g%s%s" % (base, short, dirty)

    # Ensure exactly one FIRMWARE_VERSION define (drop any prior, e.g. from build.sh).
    defines = env.get("CPPDEFINES", [])
    defines = [d for d in defines
               if not (isinstance(d, (list, tuple)) and d and d[0] == "FIRMWARE_VERSION")]
    env.Replace(CPPDEFINES=defines)
    env.Append(CPPDEFINES=[("FIRMWARE_VERSION", env.StringifyMacro(version))])
    print("fw_version: FIRMWARE_VERSION = %s" % version)
