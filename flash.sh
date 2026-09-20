#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$DIR/../.." && pwd)"
export PLATFORMIO_ENABLE_TELEMETRY=0

# Detect serial port (try python serial tools, then ls, fallback to usbserial-0001)
PORT=""
if [ -x "$WORKSPACE_ROOT/.venv/bin/python" ]; then
  PORT=$("$WORKSPACE_ROOT/.venv/bin/python" -c "import serial.tools.list_ports; ports=[p.device for p in serial.tools.list_ports.comports() if 'usb' in p.device.lower() or 'uart' in p.device.lower()]; print(ports[0] if ports else '')" 2>/dev/null || true)
fi
if [ -z "$PORT" ]; then
  PORT=$(ls /dev/cu.* 2>/dev/null | grep -iE 'usbserial|slab|ch34|uart' | head -n 1 || true)
fi
if [ -z "$PORT" ] && [ -e "/dev/cu.usbserial-0001" ]; then
  PORT="/dev/cu.usbserial-0001"
fi

if [ -z "$PORT" ]; then
  echo "No ESP32 serial port detected."
  exit 1
fi

echo "=================================================="
echo "ESP32 Detected on Port: $PORT"
echo "=================================================="

# Check for PlatformIO in workspace venv or system
if [ -x "$WORKSPACE_ROOT/.venv/bin/pio" ]; then
  PIO_CMD="$WORKSPACE_ROOT/.venv/bin/pio"
elif command -v pio >/dev/null 2>&1; then
  PIO_CMD="pio"
else
  PIO_CMD=""
fi

if [ -n "$PIO_CMD" ]; then
  echo "Building and uploading firmware with PlatformIO..."
  cd "$DIR"
  $PIO_CMD run --target upload --upload-port "$PORT"
  echo "Starting Serial Monitor (115200 baud)... Press Ctrl+C to exit."
  $PIO_CMD device monitor -b 115200 -p "$PORT"
elif command -v arduino-cli >/dev/null 2>&1; then
  echo "Building and uploading with arduino-cli..."
  SKETCH="$DIR/arduino/azure_essence_smart_ac/azure_essence_smart_ac.ino"
  arduino-cli compile --fqbn esp32:esp32:esp32 "$SKETCH"
  arduino-cli upload -p "$PORT" --fqbn esp32:esp32:esp32 "$SKETCH"
  arduino-cli monitor -p "$PORT" -c baudrate=115200
else
  echo "Please open the Arduino IDE and flash:"
  echo "  $DIR/arduino/azure_essence_smart_ac/azure_essence_smart_ac.ino"
fi
