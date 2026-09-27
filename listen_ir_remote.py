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
print("     LISTENING FOR YOUR PHONE REMOTE (GPIO 14)    ")
print("==================================================")
print("AIM YOUR PHONE AT THE BLACK IR RECEIVER SENSOR")
print("AND PRESS THE 'POWER' BUTTON ON YOUR PHONE APP...")
print("==================================================\n")

capturing = False
capture_lines = []

try:
    while True:
        line = s.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        
        if "IR SIGNAL CAPTURED" in line:
            capturing = True
            capture_lines = [line]
            print("\n>>> SIGNAL RECEIVED FROM PHONE! <<<")
            continue
            
        if capturing:
            capture_lines.append(line)
            print("  ", line)
            if "Raw Timing (" in line or "rawLength" in line or len(capture_lines) > 25:
                capturing = False
                print("\nCaptured remote profile successfully!")
                
        elif "ir_capture" in line or "IR_SNOOPER" in line or "IR_MIRROR" in line:
            print("[INFO]", line)

except KeyboardInterrupt:
    print("\nExiting listener.")
finally:
    s.close()
