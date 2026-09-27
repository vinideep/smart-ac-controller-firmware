import serial
import time
import sys

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
print("     CAMERA VISIBILITY TEST (10 CONTINUOUS FLASHES) ")
print("==================================================")
print("1. Point your smartphone's FRONT (selfie) camera directly at the clear IR LED.")
print("   (Rear cameras have strong IR-blocking glass; front selfie cameras do not).")
print("2. Look at your phone screen — you will see the LED flash purple/pink.\n")

for i in range(1, 11):
    print(f"[{i}/10] FLASHING NOW...")
    s.write(b"SET_AC_STATE 1 25 cool med\n")
    s.flush()
    time.sleep(2)

s.close()
print("\nFlash test complete.")
