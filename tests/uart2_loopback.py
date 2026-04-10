#!/usr/bin/env python3

"""Simple UART2 loopback test on /dev/ttyAMA2.

Usage on the Pi (after copying this file to ~/tests):

  cd ~/tests
  python3 uart2_loopback.py

Make sure GPIO4 (TX2, pin 7) is jumpered directly to GPIO5 (RX2, pin 29).
You should see the transmitted string echoed back if the overlay and pins
are correctly configured.
"""

import time

import serial


def main() -> None:
    port = "/dev/ttyAMA2"
    baud = 9600  # Any baud is fine for loopback as long as TX/RX share it

    print(f"Opening {port} at {baud} baud for loopback test...")
    ser = serial.Serial(port=port, baudrate=baud, timeout=0.5)

    try:
        while True:
            msg = b"HELLO UART2 LOOPBACK\r\n"
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
