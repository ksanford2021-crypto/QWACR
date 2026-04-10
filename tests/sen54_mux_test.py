#!/usr/bin/env python3

import time

from smbus2 import SMBus

I2C_BUS = 1
MUX_ADDR = 0x70
SEN54_ADDR = 0x69  # default from datasheet

# On the TCA9548A, bit 0 enables channel 0
MUX_PORT_SEN54 = 0x01


def select_mux_channel(bus: SMBus, channel_byte: int) -> None:
    """Select a single TCA9548A channel by writing the channel bitmask."""
    bus.write_byte(MUX_ADDR, channel_byte)


def read_product_name(bus: SMBus) -> bytes:
    """Read the SEN54 product name field as a basic sanity check.

    This follows the Sensirion SEN5x I2C protocol:
    - Command 0x0026: "Read product name"
    - Returns a fixed-length ASCII string.
    Here we just read a few bytes to prove the device responds.
    """

    # SEN5x commands are 16-bit big-endian words
    # 0x0026 -> bytes 0x00, 0x26
    bus.write_i2c_block_data(SEN54_ADDR, 0x00, [0x26])
    time.sleep(0.05)

    # Read back 16 bytes (enough to see ASCII characters and CRCs)
    data = bus.read_i2c_block_data(SEN54_ADDR, 0x00, 16)
    return bytes(data)


def main() -> None:
    with SMBus(I2C_BUS) as bus:
        # Route I2C1 to mux port 0 (SEN54)
        print("Selecting mux port 0 for SEN54...")
        select_mux_channel(bus, MUX_PORT_SEN54)
        time.sleep(0.01)

        # Quick presence check via i2c read
        try:
            product_raw = read_product_name(bus)
        except OSError as exc:
            print(f"Error talking to SEN54 at 0x{SEN54_ADDR:02X}: {exc}")
            return

        print("Raw product-name bytes from SEN54:")
        print(" ".join(f"{b:02X}" for b in product_raw))
        try:
            # Some bytes will be CRCs, but ASCII letters should be visible
            ascii_guess = product_raw.decode(errors="ignore")
            print(f"ASCII guess: {ascii_guess!r}")
        except Exception:
            pass


if __name__ == "__main__":
    main()
