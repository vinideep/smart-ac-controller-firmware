import serial
import time
import sys
import json

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"

s = serial.Serial()
s.port = PORT
s.baudrate = 115200
s.timeout = 0.2
s.dtr = False
s.rts = False
s.open()
time.sleep(1)
s.reset_input_buffer()

print("==================================================")
print("  REPLAYING YOUR EXACT PHONE REMOTE SIGNAL (25°C COOL) ")
print("==================================================")
print("Hold the IR transmitter LED 10-30 cm from the AC receiver window.")
print("Transmitting every 3 seconds for 30 seconds...\n")

# Use atomic SET_AC_STATE 1 25 cool med (exact match to your phone)
cmd = b"SET_AC_STATE 1 25 cool med\n"

for i in range(1, 11):
    print(f"[{i}/10] Blasting Power ON (25°C Cool Med Fan)...")
    s.write(cmd)
    s.flush()
    
    t_end = time.time() + 3
    while time.time() < t_end:
        line = s.readline().decode("utf-8", errors="replace").strip()
        if "[CMD_OK]" in line:
            print("  ", line)
        elif "ir_transmit" in line:
            print("   Transmitted successfully!")

s.close()
print("\nDone. If the AC beeps when held close (10-30 cm), the code is 100% verified.")
print("To reach across the room (2-4 meters), a 2N2222 / BC547 transistor driver is needed.")
