import urllib.request
import json
import time

URL = "http://100.116.177.5:4000/api/devices/esp32-e5ca5c/ac/power"

print("==================================================")
print("     2N2222A BOOSTED IR TRANSMITTER TEST         ")
print("==================================================")
print("Aim the IR LED at your AC unit from across the room (3-5m).")
print("Sending Power Toggle commands every 3 seconds...\n")

for i in range(1, 6):
    power_state = (i % 2 != 0) # Alternate ON / OFF
    state_str = "POWER ON (25°C Cool)" if power_state else "POWER OFF"
    print(f"[{i}/5] Blasting {state_str}...")
    
    try:
        data = json.dumps({"power": power_state}).encode("utf-8")
        req = urllib.request.Request(URL, data=data, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=3) as resp:
            print(f"      Sent to controller -> AC should beep now!")
    except Exception as e:
        print(f"      Request error: {e}")
        
    time.sleep(3)

print("\n==================================================")
print("Test complete. If the AC unit beeped from across")
print("the room, the 2N2222A transistor driver is working!")
print("==================================================")
