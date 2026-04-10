#!/usr/bin/env python3

"""Active DGS2 command/response test over UART.

Usage on the Pi (after copying this file to ~/tests):

    cd ~/tests
    # Positional args (original style)
    python3 dgs2_command_test.py /dev/ttyAMA2 9600

    # Or with flags
    python3 dgs2_command_test.py --port /dev/ttyAMA4 --baud 9600

This script:
- Opens the given serial port at the given baud.
- Sends 'C' + CR once to toggle continuous-measurement mode (per DGS2 manual).
- Then reads and prints incoming lines.
- On Ctrl+C, it will send 'C' + CR again to turn continuous mode off.

If you prefer single measurements, change the initial command to '\r'.
"""

import argparse
import sys
import time

import serial


def main() -> None:
        parser = argparse.ArgumentParser(description="Active DGS2 UART command/response test")
        parser.add_argument("serial_port", nargs="?", help="Serial device path, e.g. /dev/ttyAMA2")
        parser.add_argument("baud", nargs="?", type=int, help="Baud rate, e.g. 9600")
        parser.add_argument("-p", "--port", dest="port", help="Serial device path, e.g. /dev/ttyAMA2")
        parser.add_argument("-b", "--baud", dest="baud_flag", type=int, help="Baud rate, e.g. 9600")

        args = parser.parse_args()

        port = args.port or args.serial_port
        baud = args.baud_flag if args.baud_flag is not None else args.baud

        if not port or baud is None:
                parser.error("You must specify a serial port and baud (positional or via --port/--baud)")

    print(f"Opening {port} at {baud} baud...")
    ser = serial.Serial(port=port, baudrate=baud, timeout=1.0)

    # Give sensor a moment after power-on if needed
    print("Waiting 2 seconds for sensor to stabilize...")
    time.sleep(2.0)

    # Enable continuous measurement mode: 'C' + CR (per manual)
    cmd = b"C\r"
    print(f"Sending command: {cmd!r} (enable continuous measurement)")
    ser.write(cmd)
    ser.flush()

    try:
        while True:
            line = ser.readline()
            if line:
                print(f"RX: {line!r}")
            else:
                # Nothing received this second; print a heartbeat so you know it's alive
                print("RX: (no data yet)")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nCtrl+C received. Attempting to disable continuous mode...")
        try:
            ser.write(b"C\r")
            ser.flush()
        except Exception:
            pass
    finally:
        ser.close()


if __name__ == "__main__":
    main()
