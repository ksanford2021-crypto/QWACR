#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union


@dataclass
class Dgs2Measurement:
    """Parsed measurement from a DGS2 module.

    Fields follow the format described in the DGS2 manual. The module sends
    lines of the form (comma-separated):

        SensorSN, PPB, TEMPERATURE, RELATIVE_HUMIDITY, ADC_G, ADC_T, ADC_H

    where TEMPERATURE and RELATIVE_HUMIDITY are scaled by 100
    (e.g. 2436 -> 24.36 °C, 3278 -> 32.78 %RH).
    """

    sensor_sn: str
    ppb: float
    temperature_c: float
    relative_humidity_percent: float
    adc_g: int
    adc_t: int
    adc_h: int
    raw_line: str


def parse_measurement(
    line: Union[bytes, str],
) -> Optional[Dgs2Measurement]:
    """Parse a single DGS2 measurement line.

    Returns None if the line cannot be parsed.
    """

    if isinstance(line, bytes):
        try:
            text = line.decode("ascii", errors="ignore")
        except Exception:
            return None
    else:
        text = line

    text = text.strip()  # remove CR/LF and surrounding spaces
    if not text:
        return None

    parts = [p.strip() for p in text.split(",")]
    if len(parts) < 7:
        # Unexpected format
        return None

    sensor_sn = parts[0]

    try:
        ppb = float(int(parts[1]))
        temp_counts = int(parts[2])
        rh_counts = int(parts[3])
        adc_g = int(parts[4])
        adc_t = int(parts[5])
        # Last field may potentially contain extra whitespace or tokens
        adc_h_str = parts[6].split()[0]
        adc_h = int(adc_h_str)
    except (ValueError, IndexError):
        return None

    temperature_c = temp_counts / 100.0
    relative_humidity_percent = rh_counts / 100.0

    return Dgs2Measurement(
        sensor_sn=sensor_sn,
        ppb=ppb,
        temperature_c=temperature_c,
        relative_humidity_percent=relative_humidity_percent,
        adc_g=adc_g,
        adc_t=adc_t,
        adc_h=adc_h,
        raw_line=text,
    )
