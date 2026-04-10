#!/usr/bin/env python3

import time

from smbus2 import SMBus

I2C_BUS = 1
MUX_ADDR = 0x70
MLX90640_ADDR = 0x33

# On the TCA9548A, bit 1 enables channel 1
MUX_PORT_MLX = 0x02


def select_mux_channel(bus: SMBus, channel_byte: int) -> None:
    bus.write_byte(MUX_ADDR, channel_byte)


def read_device_id(bus: SMBus) -> int:
    """Read MLX90640 device ID register as a basic check.

    The ID is stored in the EEPROM / configuration space. Here we use
    register 0x2407 per the datasheet (two bytes, big-endian).
    """

    # MLX90640 uses 16-bit register addresses; smbus2 doesn't support
    # 16-bit regs directly, so we send the high/low bytes manually.
    reg_high = 0x24
    reg_low = 0x07

    # Write register address, then read 2 bytes
    bus.write_i2c_block_data(MLX90640_ADDR, reg_high, [reg_low])
    time.sleep(0.01)
    data = bus.read_i2c_block_data(MLX90640_ADDR, reg_high, 2)
    return (data[0] << 8) | data[1]


def main() -> None:
    with SMBus(I2C_BUS) as bus:
        print("Selecting mux port 1 for MLX90640...")
        select_mux_channel(bus, MUX_PORT_MLX)
        time.sleep(0.01)

        try:
            dev_id = read_device_id(bus)
        except OSError as exc:
            print(f"Error talking to MLX90640 at 0x{MLX90640_ADDR:02X}: {exc}")
            return

        print(f"MLX90640 device ID: 0x{dev_id:04X}")


if __name__ == "__main__":
    main()
