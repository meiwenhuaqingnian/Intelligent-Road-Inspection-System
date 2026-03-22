#!/usr/bin/env python3
"""
Read NMEA sentences from a GPS module over UART and parse RMC (GPRMC/GNRMC).

Default device/baud match many common GPS modules: /dev/serial0 @ 9600.
"""

from __future__ import annotations

import argparse
import datetime as dt
import sys
from dataclasses import dataclass
from typing import Optional, Tuple

import serial


@dataclass(frozen=True)
class RmcFix:
    talker: str  # e.g. "GP" or "GN"
    utc_time: Optional[dt.time]
    status: str  # 'A' valid, 'V' void
    latitude_deg: Optional[float]  # decimal degrees, +N / -S
    longitude_deg: Optional[float]  # decimal degrees, +E / -W
    raw_lat: str
    raw_ns: str
    raw_lon: str
    raw_ew: str


def _nmea_checksum(payload: str) -> int:
    chk = 0
    for ch in payload:
        chk ^= ord(ch)
    return chk


def _validate_nmea(line: str) -> Tuple[bool, str]:
    """
    Returns (ok, reason). Accepts both \r\n and \n stripped strings.
    """
    s = line.strip()
    if not s.startswith("$"):
        return False, "missing '$'"
    star = s.find("*")
    if star == -1:
        return False, "missing '*' checksum separator"
    payload = s[1:star]
    checksum_str = s[star + 1 : star + 3]
    if len(checksum_str) != 2:
        return False, "checksum length != 2"
    try:
        expected = int(checksum_str, 16)
    except ValueError:
        return False, "checksum not hex"
    actual = _nmea_checksum(payload)
    if actual != expected:
        return False, f"checksum mismatch expected={expected:02X} actual={actual:02X}"
    return True, "ok"


def _parse_utc_time(hhmmss: str) -> Optional[dt.time]:
    if not hhmmss:
        return None
    # hhmmss[.sss]
    try:
        hh = int(hhmmss[0:2])
        mm = int(hhmmss[2:4])
        ss = int(hhmmss[4:6])
        micros = 0
        if len(hhmmss) > 6 and hhmmss[6] == ".":
            frac = hhmmss[7:]
            # pad / truncate to microseconds
            frac = (frac + "000000")[:6]
            micros = int(frac)
        return dt.time(hour=hh, minute=mm, second=ss, microsecond=micros, tzinfo=dt.timezone.utc)
    except Exception:
        return None


def _nmea_degmin_to_decimal(value: str, hemi: str, is_lat: bool) -> Optional[float]:
    """
    Convert ddmm.mmmm (lat) or dddmm.mmmm (lon) to decimal degrees.
    hemi: N/S/E/W
    """
    if not value or not hemi:
        return None
    hemi = hemi.upper()
    try:
        deg_len = 2 if is_lat else 3
        deg = float(value[:deg_len])
        minutes = float(value[deg_len:])
        dec = deg + minutes / 60.0
        if hemi in ("S", "W"):
            dec = -dec
        return dec
    except Exception:
        return None


def parse_rmc_from_line(line: str) -> Optional[RmcFix]:
    """
    Accepts a full NMEA line (e.g. $GNRMC,...*hh). Returns RmcFix or None.
    """
    ok, _ = _validate_nmea(line)
    if not ok:
        return None

    s = line.strip()
    payload = s[1 : s.find("*")]
    fields = payload.split(",")
    if not fields:
        return None

    msg = fields[0]  # e.g. GNRMC
    if len(msg) != 5 or not msg.endswith("RMC"):
        return None
    talker = msg[:2]

    # RMC (minimum) fields we care:
    # 1 UTC time, 2 status, 3 lat, 4 N/S, 5 lon, 6 E/W
    utc = _parse_utc_time(fields[1] if len(fields) > 1 else "")
    status = (fields[2] if len(fields) > 2 else "").strip().upper() or "?"
    raw_lat = fields[3] if len(fields) > 3 else ""
    raw_ns = fields[4] if len(fields) > 4 else ""
    raw_lon = fields[5] if len(fields) > 5 else ""
    raw_ew = fields[6] if len(fields) > 6 else ""

    lat = _nmea_degmin_to_decimal(raw_lat, raw_ns, is_lat=True)
    lon = _nmea_degmin_to_decimal(raw_lon, raw_ew, is_lat=False)

    return RmcFix(
        talker=talker,
        utc_time=utc,
        status=status,
        latitude_deg=lat,
        longitude_deg=lon,
        raw_lat=raw_lat,
        raw_ns=raw_ns,
        raw_lon=raw_lon,
        raw_ew=raw_ew,
    )


def _format_fix(fix: RmcFix) -> str:
    t = fix.utc_time.isoformat() if fix.utc_time else "N/A"
    if fix.status == "A" and fix.latitude_deg is not None and fix.longitude_deg is not None:
        return (
            f"[{fix.talker}RMC] UTC={t} status=A "
            f"lat={fix.latitude_deg:.6f} lon={fix.longitude_deg:.6f} "
            f"(raw {fix.raw_lat}{fix.raw_ns} {fix.raw_lon}{fix.raw_ew})"
        )
    return f"[{fix.talker}RMC] UTC={t} status={fix.status} (raw {fix.raw_lat}{fix.raw_ns} {fix.raw_lon}{fix.raw_ew})"


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description="Raspberry Pi GPS RMC reader (NMEA over UART)")
    ap.add_argument("--port", default="/dev/serial0", help="Serial device, e.g. /dev/serial0 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=9600, help="Baud rate (default 9600)")
    ap.add_argument("--timeout", type=float, default=1.0, help="Read timeout seconds")
    ap.add_argument("--raw", action="store_true", help="Print raw NMEA lines too")
    args = ap.parse_args(argv)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"Failed to open serial port {args.port}: {e}", file=sys.stderr)
        return 2

    print(f"Reading NMEA from {args.port} @ {args.baud} ... (Ctrl+C to stop)")
    try:
        while True:
            line_bytes = ser.readline()
            if not line_bytes:
                continue
            try:
                line = line_bytes.decode("ascii", errors="ignore")
            except Exception:
                continue

            if args.raw:
                print(line.rstrip("\r\n"))

            fix = parse_rmc_from_line(line)
            if fix is not None:
                print(_format_fix(fix))
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

