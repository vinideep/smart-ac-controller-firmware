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

print("Blasting AC Power ON/OFF every 3 seconds for 30 seconds.")
print("Hold the transmitter 10-30 cm from the AC receiver...")

for i in range(1, 11):
    cmd = b"POWER_ON\n" if i % 2 == 1 else b"POWER_OFF\n"
    print(f"[{i}/10] Sending {cmd.decode().strip()}...")
    s.write(cmd)
    s.flush()
    time.sleep(3)

s.close()
print("Done.")
