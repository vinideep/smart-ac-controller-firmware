Import("env")
import subprocess

try:
    count = subprocess.check_output(["git", "rev-list", "--count", "HEAD"]).decode().strip()
    version = f"v1.3.{count}"
except Exception:
    version = "v1.3.1"

env.Append(CPPDEFINES=[("FIRMWARE_VERSION", f'\\"{version}\\"')])
print(f"*** Auto-Incremented Firmware Version: {version} ***")
