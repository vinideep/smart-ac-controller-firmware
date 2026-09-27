import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbserial-0001"
BAUD = 115200

print(f"Connecting to ESP32 on {PORT} at {BAUD} baud...")
try:
    s = serial.Serial()
    s.port = PORT
    s.baudrate = BAUD
    s.timeout = 0.2
    s.dtr = False
    s.rts = False
    s.open()
except Exception as e:
    print(f"Error opening serial port: {e}")
    sys.exit(1)

print("Waiting for ESP32 ready state...")
t_wait = time.time() + 4
while time.time() < t_wait:
    line = s.readline().decode("utf-8", errors="replace").strip()
    if "Ready. Commands:" in line or "Beginning telemetry stream" in line:
        break
time.sleep(0.5)
s.reset_input_buffer()

print("==================================================")
print("       IR TRANSMITTER <-> RECEIVER LOOPBACK TEST  ")
print("==================================================")
print("Position the IR LED 1-2 cm facing the IR receiver.")
print("Firing 5 test bursts...\n")

success = False
for attempt in range(1, 6):
    print(f"--- [Burst {attempt}/5] Transmitting TEST_TX ---")
    s.write(b"TEST_TX\n")
    s.flush()

    t_end = time.time() + 2.5
    while time.time() < t_end:
        line = s.readline().decode("utf-8", errors="replace").strip()
        if not line:
            continue
        if "[CMD_OK]" in line or "ir_transmit" in line:
            print(f"  [TX] {line}")
        elif "IR SIGNAL CAPTURED" in line or "IR_SNOOPER" in line or "IR_MIRROR" in line:
            print(f"  [RX] >>> {line} <<<")
            success = True
        elif "Ready" in line or "Connected" in line:
            print(f"  [ESP32] {line}")

    if success:
        break
    time.sleep(0.5)

s.close()

print("\n==================================================")
if success:
    print("RESULT: SUCCESS! IR Receiver detected the transmitted signal.")
else:
    print("RESULT: NO SIGNAL RECEIVED.")
    print("Troubleshooting checklist:")
    print("1. Point a smartphone camera at the IR LED during burst to see if it blinks purple.")
    print("2. Check transistor wiring (Base -> GPIO 25, Collector -> LED cathode, Emitter -> GND).")
    print("3. Ensure IR LED anode (+) connects to 3.3V/5V via resistor (100R to 220R).")
    print("4. Verify IR receiver signal pin is connected to GPIO 14.")
print("==================================================")
