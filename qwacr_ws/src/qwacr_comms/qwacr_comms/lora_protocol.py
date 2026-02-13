"""LoRa UART framing and packet encode/decode utilities."""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Optional, Tuple


HEADER_BYTE = 0xA5
MSG_TELEMETRY = 0x01
MSG_TELEOP = 0x02
MSG_ESTOP = 0xFF


@dataclass
class TeleopCommand:
    linear_x: float
    angular_z: float
    flags: int = 0


@dataclass
class Telemetry:
    gps_lat: float
    gps_lon: float
    gps_fix: int
    battery_v: float
    battery_a: float
    system_status: int
    odom_distance: float
    rssi: int = 0
    snr: int = 0


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE with poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def frame_packet(msg_id: int, payload: bytes) -> bytes:
    """Frame a packet: [HEADER][MSG_ID][LEN][PAYLOAD][CRC16]."""
    length = len(payload)
    header = bytes([HEADER_BYTE, msg_id, length])
    crc = crc16_ccitt(header + payload)
    return header + payload + struct.pack("<H", crc)


def parse_frame(buffer: bytearray) -> Tuple[Optional[Tuple[int, bytes]], bytearray]:
    """Parse a frame from buffer. Returns (msg_id, payload) or None.

    Buffer is consumed up to the parsed frame.
    """
    if len(buffer) < 4:
        return None, buffer

    # Sync to header
    try:
        start = buffer.index(HEADER_BYTE)
    except ValueError:
        return None, bytearray()

    if start > 0:
        buffer = buffer[start:]

    if len(buffer) < 4:
        return None, buffer

    msg_id = buffer[1]
    length = buffer[2]
    frame_len = 3 + length + 2

    if len(buffer) < frame_len:
        return None, buffer

    payload = bytes(buffer[3 : 3 + length])
    recv_crc = struct.unpack("<H", buffer[3 + length : frame_len])[0]
    calc_crc = crc16_ccitt(bytes(buffer[:3]) + payload)

    if recv_crc != calc_crc:
        # Drop the header and resync
        return None, buffer[1:]

    remaining = buffer[frame_len:]
    return (msg_id, payload), remaining


def pack_teleop(cmd: TeleopCommand) -> bytes:
    """Pack teleop command payload."""
    linear = int(cmd.linear_x * 1000)
    angular = int(cmd.angular_z * 1000)
    payload = struct.pack("<iih", 0, linear, angular)  # placeholder timestamp
    payload += struct.pack("<B", cmd.flags)
    return frame_packet(MSG_TELEOP, payload)


def unpack_teleop(payload: bytes) -> Optional[TeleopCommand]:
    """Unpack teleop payload into TeleopCommand."""
    if len(payload) < 11:
        return None
    _, linear, angular = struct.unpack("<iih", payload[:10])
    flags = payload[10]
    return TeleopCommand(linear_x=linear / 1000.0, angular_z=angular / 1000.0, flags=flags)


def pack_telemetry(telemetry: Telemetry) -> bytes:
    """Pack telemetry payload."""
    payload = struct.pack(
        "<i ffB HHB f bb",
        0,  # placeholder timestamp
        telemetry.gps_lat,
        telemetry.gps_lon,
        telemetry.gps_fix,
        int(telemetry.battery_v * 100),
        int(telemetry.battery_a * 100),
        telemetry.system_status,
        telemetry.odom_distance,
        telemetry.rssi,
        telemetry.snr,
    )
    return frame_packet(MSG_TELEMETRY, payload)


def unpack_telemetry(payload: bytes) -> Optional[Telemetry]:
    """Unpack telemetry payload into Telemetry."""
    if len(payload) < 25:
        return None
    _, gps_lat, gps_lon, gps_fix, batt_v, batt_a, status, odom, rssi, snr = struct.unpack(
        "<i ffB HHB f bb", payload[:25]
    )
    return Telemetry(
        gps_lat=gps_lat,
        gps_lon=gps_lon,
        gps_fix=gps_fix,
        battery_v=batt_v / 100.0,
        battery_a=batt_a / 100.0,
        system_status=status,
        odom_distance=odom,
        rssi=rssi,
        snr=snr,
    )
