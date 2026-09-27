import urllib.request
import json
import time
import sys

URL = "http://192.168.1.10/api/ac/state"

data = json.dumps({
    "power": True,
    "temperature": 25,
    "mode": "cool",
    "fan_speed": "med"
}).encode("utf-8")

print("==================================================")
print("     BLASTING AC POWER ON VIA WI-FI (NO USB NEEDED)")
print("==================================================")
print(f"Target: {URL}")
print("Aiming IR transmitter at AC from across the room...")
print("Blasting every 2 seconds for 10 bursts...\n")

for i in range(1, 11):
    try:
        req = urllib.request.Request(URL, data=data, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=3) as resp:
            resp_body = resp.read().decode()
            print(f"[{i}/10] Blasted Power ON (25°C Cool Med) -> AC State: {resp_body}")
    except Exception as e:
        print(f"[{i}/10] Request error: {e}")
    time.sleep(2)

print("\nDone! If the transistor is connected properly, the AC should have beeped.")
