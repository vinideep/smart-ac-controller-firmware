#!/usr/bin/env python3
"""
IR Capture Monitor & Interactive Calibration Utility
Project: AI Smart Controller Retrofit for Azure Essence AE-18 AC (Milestone 3)

Features:
- Automatic serial port detection across macOS/Linux USB-UART drivers.
- Interactive step-by-step calibration sequence guiding the user through remote button presses.
- Real-time frame decoding (protocol, bits, state bytes, raw timings).
- State diff analysis between successive AC commands.
- Passive logging mode for general observation.
- Persistent JSON serialization to captured_ir_codes.json.
"""
import sys
import os
import json
import time
import argparse
from typing import Optional, List, Dict, Any

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial is required. Install with: pip install pyserial")
    sys.exit(1)

DEFAULT_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 115200
OUTPUT_FILE = os.path.join(os.path.dirname(__file__), "captured_ir_codes.json")

CALIBRATION_STEPS = [
    {
        "key": "power_on_cool_25_auto",
        "name": "Power ON (Cool 25°C, Auto Fan)",
        "prompt": "Set AC to Cool 25°C, Auto Fan. Aim phone IR blaster at GPIO 14 and click: POWER ON",
    },
    {
        "key": "temp_up_26",
        "name": "Temp UP (Cool 26°C)",
        "prompt": "Aim phone IR blaster at GPIO 14 and click: TEMP UP (set to 26°C)",
    },
    {
        "key": "temp_down_24",
        "name": "Temp DOWN (Cool 24°C)",
        "prompt": "Aim phone IR blaster at GPIO 14 and click: TEMP DOWN (set to 24°C)",
    },
    {
        "key": "fan_speed",
        "name": "Fan Speed (Med / High)",
        "prompt": "Aim phone IR blaster at GPIO 14 and click: FAN SPEED (cycle Fan Med or High)",
    },
    {
        "key": "power_off",
        "name": "Power OFF",
        "prompt": "Aim phone IR blaster at GPIO 14 and click: POWER OFF",
    },
]


def detect_serial_port(specified_port: Optional[str] = None) -> str:
    """Detect available ESP32 serial port or use specified port."""
    if specified_port and os.path.exists(specified_port):
        return specified_port

    if specified_port and specified_port != DEFAULT_PORT:
        return specified_port

    # Search list_ports for USB serial devices
    ports = serial.tools.list_ports.comports()
    for p in ports:
        dev = p.device.lower()
        desc = (p.description or "").lower()
        if any(k in dev or k in desc for k in ["usbserial", "ch34", "slab", "uart", "cp210", "wch"]):
            return p.device

    # Check fallback path
    if os.path.exists(DEFAULT_PORT):
        return DEFAULT_PORT

    # Fallback to first available port if any
    if ports:
        return ports[0].device

    return specified_port or DEFAULT_PORT


def print_state_diff(prev_state: List[int], curr_state: List[int]) -> None:
    """Display byte-level difference between previous and current state packets."""
    if not prev_state or not curr_state:
        return

    diffs = []
    max_len = max(len(prev_state), len(curr_state))
    for i in range(max_len):
        b_prev = prev_state[i] if i < len(prev_state) else None
        b_curr = curr_state[i] if i < len(curr_state) else None
        if b_prev != b_curr:
            prev_str = f"0x{b_prev:02X}" if b_prev is not None else "--"
            curr_str = f"0x{b_curr:02X}" if b_curr is not None else "--"
            diffs.append(f"Byte [{i}]: {prev_str} -> {curr_str}")

    if diffs:
        print("  ┌─ State Diff Analysis vs Previous:")
        for d in diffs:
            print(f"  │  • {d}")
        print("  └───────────────────────────────────")
    else:
        print("  Note: State bytes identical to previous command.")


def save_captured_data(data: Dict[str, Any], filepath: str) -> None:
    """Save captured dataset cleanly to JSON."""
    with open(filepath, "w") as f:
        json.dump(data, f, indent=2)


def run_interactive_calibration(ser: serial.Serial, filepath: str) -> None:
    """Step-by-step guided calibration sequence."""
    print()
    print("==================================================================")
    print("  ESP32 IR Learning: Interactive Remote Calibration Sequence     ")
    print("==================================================================")
    print("This procedure will guide you button-by-button to record the Azure")
    print("Essence AE-18 remote operational frames into captured_ir_codes.json")
    print("Aim your phone IR blaster 5–15 cm from the receiver on GPIO 27.")
    print("Press Ctrl+C at any time to save progress and exit.")
    print("==================================================================")

    # Load existing database
    db: Dict[str, Any] = {
        "metadata": {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "target": "Azure Essence AE-18 Smart AC",
            "milestone": 3,
        },
        "buttons": {},
        "raw_events": [],
    }
    if os.path.exists(filepath):
        try:
            with open(filepath, "r") as f:
                loaded = json.load(f)
                if isinstance(loaded, dict) and "buttons" in loaded:
                    db = loaded
                elif isinstance(loaded, list):
                    db["raw_events"] = loaded
        except Exception:
            pass

    step_idx = 0
    total_steps = len(CALIBRATION_STEPS)
    prev_state: Optional[List[int]] = None

    while step_idx < total_steps:
        step = CALIBRATION_STEPS[step_idx]
        print(f"\n👉 [STEP {step_idx + 1}/{total_steps}] {step['name']}")
        print(f"   Instruction: {step['prompt']}")
        print("   [Listening on serial port... waiting for IR transmission]")

        captured_payload: Optional[Dict[str, Any]] = None

        while captured_payload is None:
            line = ser.readline()
            if not line:
                continue
            text = line.decode("utf-8", errors="replace").rstrip()
            if not text:
                continue

            # Print non-IR lines to provide live diagnostic feedback
            if not text.startswith('{"type":"ir_capture"'):
                if not text.startswith('{"temperature":'):
                    print(f"  {text}")
                continue

            # Parse IR capture line
            try:
                payload = json.loads(text)
                captured_payload = payload
            except json.JSONDecodeError:
                continue

        # Valid payload captured!
        curr_state = captured_payload.get("state")
        protocol = captured_payload.get("protocol", "UNKNOWN")
        bits = captured_payload.get("bits", 0)
        hex_code = captured_payload.get("hex", "")
        raw_len = captured_payload.get("raw_len", 0)
        ac_desc = captured_payload.get("ac_desc", "")

        print(f"\n✅ [SUCCESS] Captured: {step['name']}")
        print(f"  • Protocol : {protocol} (Bits: {bits})")
        print(f"  • Code/Hex : {hex_code}")
        print(f"  • Raw Len  : {raw_len} transitions")
        if ac_desc:
            print(f"  • AC Decode: {ac_desc}")
        if curr_state:
            hex_bytes = " ".join(f"{b:02X}" for b in curr_state)
            print(f"  • State Hex: {hex_bytes}")

        # Compute diff if applicable
        if prev_state and curr_state:
            print_state_diff(prev_state, curr_state)

        # Store record
        captured_payload["step_key"] = step["key"]
        captured_payload["step_name"] = step["name"]
        captured_payload["recorded_at"] = time.strftime("%Y-%m-%d %H:%M:%S")

        db["buttons"][step["key"]] = captured_payload
        db["raw_events"].append(captured_payload)
        save_captured_data(db, filepath)
        print(f"  [Saved to {filepath}]")

        prev_state = curr_state
        step_idx += 1
        time.sleep(0.5)

    print()
    print("==================================================================")
    print("🎉 CALIBRATION COMPLETE! All 5 baseline remote buttons captured.")
    print(f"Summary saved to: {filepath}")
    print("==================================================================")
    print_summary(db)


def run_passive_monitor(ser: serial.Serial, filepath: str) -> None:
    """Passive listening mode."""
    print("==================================================")
    print("  ESP32 IR Capture Monitor Active (Passive Mode) ")
    print("  Aim your phone IR remote at GPIO 27 and click.  ")
    print("  Press Ctrl+C to finish.                         ")
    print("==================================================")

    db: Dict[str, Any] = {
        "metadata": {
            "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "target": "Azure Essence AE-18 Smart AC",
            "milestone": 3,
        },
        "buttons": {},
        "raw_events": [],
    }
    if os.path.exists(filepath):
        try:
            with open(filepath, "r") as f:
                loaded = json.load(f)
                if isinstance(loaded, dict):
                    db = loaded
                elif isinstance(loaded, list):
                    db["raw_events"] = loaded
        except Exception:
            pass

    try:
        while True:
            line = ser.readline()
            if not line:
                continue
            text = line.decode("utf-8", errors="replace").rstrip()
            if not text:
                continue

            print(text)

            if text.startswith('{"type":"ir_capture"'):
                try:
                    payload = json.loads(text)
                    payload["recorded_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
                    db["raw_events"].append(payload)
                    save_captured_data(db, filepath)
                    print(f"\n[SAVED] Captured frame #{len(db['raw_events'])} saved to {filepath}\n")
                except json.JSONDecodeError:
                    pass
    except KeyboardInterrupt:
        print("\nExiting monitor. All captured signals saved.")


def print_summary(db: Dict[str, Any]) -> None:
    """Print clean summary table of captured remote commands."""
    buttons = db.get("buttons", {})
    events = db.get("raw_events", [])
    print(f"\nDatabase Summary: {len(buttons)} calibrated buttons, {len(events)} total frames recorded.")
    if buttons:
        print(f"{'Button Name':<32} {'Protocol':<12} {'Bits':<6} {'Hex/Code':<20} {'Raw Len':<8}")
        print("-" * 80)
        for key, info in buttons.items():
            name = info.get("step_name", key)
            proto = info.get("protocol", "UNKNOWN")
            bits = str(info.get("bits", 0))
            hex_val = info.get("hex", "")
            raw_len = str(info.get("raw_len", 0))
            print(f"{name:<32} {proto:<12} {bits:<6} {hex_val:<20} {raw_len:<8}")
    print()


def main():
    parser = argparse.ArgumentParser(description="IR Capture Monitor & Calibration Utility")
    parser.add_argument("port", nargs="?", default=None, help="Serial port (e.g. /dev/cu.usbserial-0001)")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help=f"Baud rate (default: {BAUD_RATE})")
    parser.add_argument("--passive", action="store_true", help="Passive logging mode instead of guided calibration")
    parser.add_argument("--summary", action="store_true", help="Show summary of existing captured_ir_codes.json and exit")
    parser.add_argument("--output", default=OUTPUT_FILE, help="Path to output JSON file")
    args = parser.parse_args()

    if args.summary:
        if os.path.exists(args.output):
            with open(args.output, "r") as f:
                data = json.load(f)
                if isinstance(data, dict):
                    print_summary(data)
                else:
                    print(f"File contains {len(data)} raw frames.")
        else:
            print(f"No capture file found at {args.output}")
        return

    port = detect_serial_port(args.port)
    print(f"Connecting to ESP32 on {port} at {args.baud} baud...")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.5)
    except Exception as e:
        print(f"Error opening port {port}: {e}")
        print("\nTroubleshooting tips:")
        print("  1. Verify ESP32 is plugged in with data USB cable.")
        print("  2. Check available ports: ls /dev/cu.*")
        print(f"  3. Try specifying port manually: python monitor_ir.py <port>")
        sys.exit(1)

    try:
        if args.passive or not sys.stdin.isatty():
            run_passive_monitor(ser, args.output)
        else:
            run_interactive_calibration(ser, args.output)
    except KeyboardInterrupt:
        print("\nOperation cancelled by user.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
