#!/usr/bin/env python3

"""Simple UART3 loopback test on /dev/ttyAMA3.

Usage on the Pi (after copying this file to ~/tests):

  cd ~/tests
  python3 uart3_loopback.py

Make sure GPIO12 (TX3, pin 32) is jumpered directly to GPIO13 (RX3, pin 33).
You should see the transmitted string echoed back if the overlay and pins
are correctly configured and /dev/ttyAMA3 exists.
"""

import time

import serial


def main() -> None:
    port = "/dev/ttyAMA3"
    baud = 9600  # Any baud is fine for loopback as long as TX/RX share it

    print(f"Opening {port} at {baud} baud for loopback test...")
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.5)

    try:
        while True:
            msg = b"HELLO UART3 LOOPBACK\r\n"
            ser.write(msg)
            time.sleep(0.2)
            data = ser.read(64)
            if data:
                print(f"TX: {msg!r}")
                print(f"RX: {data!r}")
            else:
                print("RX: (nothing)")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
