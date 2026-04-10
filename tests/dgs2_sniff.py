#!/usr/bin/env python3

"""Simple serial sniffer for DGS2 gas sensors.

Usage examples:

  python3 dgs2_sniff.py /dev/ttyAMA2 9600
  python3 dgs2_sniff.py /dev/ttyAMA3 9600

Adjust the baud rate to whatever the DGS2 datasheet specifies.
"""

import sys
import time

import serial


def main() -> None:
    if len(sys.argv) < 3:
        print("Usage: dgs2_sniff.py SERIAL_PORT BAUD")
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2])

    print(f"Opening {port} at {baud} baud...")
    ser = serial.Serial(port=port, baudrate=baud, timeout=1.0)

    try:
        while True:
            data = ser.read(64)
            if data:
                # Print both hex and ASCII-ish view
                hex_str = " ".join(f"{b:02X}" for b in data)
                ascii_str = "".join(chr(b) if 32 <= b <= 126 else "." for b in data)
                print(f"HEX: {hex_str}\nASCII: {ascii_str}")
            else:
                # Avoid hammering CPU if idle
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
