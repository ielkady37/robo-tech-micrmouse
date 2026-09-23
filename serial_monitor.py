#!/usr/bin/env python3
"""Print serial output from the ESP32.

Usage: python3 serial_monitor.py [port] [baud]
Defaults: first /dev/ttyUSB* or /dev/ttyACM* found, 115200 baud.
"""
import glob
import sys
import time

import serial


def find_port():
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No serial port found (is the ESP plugged in?)")
    return ports[0]


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

    print(f"Listening on {port} @ {baud} (Ctrl+C to quit)")
    while True:
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                while True:
                    line = ser.readline()
                    if line:
                        print(line.decode("utf-8", errors="replace").rstrip())
        except serial.SerialException as e:
            print(f"[serial error: {e}] reconnecting...")
            time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nBye")
