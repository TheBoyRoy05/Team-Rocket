#!/usr/bin/env python3
"""
Decode flight.bin from main-lora (TRB\\x01 + TR v1 frames, 48 B) or TRB\\x02 (+ lora_tx, lora_ack).

Usage:
  python scripts/flight_bin_to_csv.py path/to/flight.bin > out.csv

Scales match src/flight_packet.hpp (centi-deg, centi-m/s², etc.).
"""

from __future__ import annotations

import struct
import sys

HEADER_PREFIX = b"TRB"
# FlightPacketV1 body (48 B): magic(2)+ver(1)+t_ms(4)+cal(4)+15×int16+bmp_ok+temp+press+alt
REC_BODY_FMT = "<2s B I 4B" + "h" * 15 + "B h H h"


def main() -> None:
    if len(sys.argv) != 2:
        print("usage: flight_bin_to_csv.py flight.bin", file=sys.stderr)
        sys.exit(2)
    path = sys.argv[1]
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        print(f"cannot open {path!r}: {e}", file=sys.stderr)
        print(
            "hint: on Git Bash use forward slashes, e.g. "
            "/c/Users/you/Desktop/flight.bin or quote a Windows path.",
            file=sys.stderr,
        )
        sys.exit(1)
    if len(data) < len(HEADER_PREFIX) + 2:
        print("file too small", file=sys.stderr)
        sys.exit(1)
    if data[:3] != HEADER_PREFIX:
        print(f"bad header prefix: expected TRB, got {data[:3]!r}", file=sys.stderr)
        sys.exit(1)
    fmt_ver = data[3]
    if fmt_ver == 1:
        rec_fmt = REC_BODY_FMT
    elif fmt_ver == 2:
        rec_fmt = REC_BODY_FMT + "B B"
    else:
        print(f"unsupported TRB format version {fmt_ver}", file=sys.stderr)
        sys.exit(1)

    rec_size = struct.calcsize(rec_fmt)
    header_len = 4
    if len(data) < header_len + rec_size:
        print("file too small for one record", file=sys.stderr)
        sys.exit(1)

    body = data[header_len:]
    n = len(body) // rec_size
    if len(body) % rec_size:
        print(
            f"warning: {len(body) % rec_size} trailing bytes (partial frame?)",
            file=sys.stderr,
        )

    csv_cols = (
        "TR,t_ms,cal_sys,cal_gyro,cal_accel,cal_mag,"
        "euler_h,euler_r,euler_p,"
        "acc_gx,acc_gy,acc_gz,lin_ax,lin_ay,lin_az,"
        "gyr_x,gyr_y,gyr_z,mag_x,mag_y,mag_z,"
        "bmp_ok,temp_C,press_hPa,alt_m"
    )
    if fmt_ver == 2:
        csv_cols += ",lora_tx_done,lora_ack_ok"
    print(csv_cols)

    off = 0
    for _ in range(n):
        chunk = body[off : off + rec_size]
        off += rec_size
        tup = struct.unpack(rec_fmt, chunk)
        magic, ver, t_ms, cs, cg, ca, cm = tup[0], tup[1], tup[2], tup[3], tup[4], tup[5], tup[6]
        if magic != b"TR" or ver != 1:
            print(f"skip bad magic/ver at offset {off - rec_size}", file=sys.stderr)
            continue
        euler_h_cd, euler_r_cd, euler_p_cd = tup[7], tup[8], tup[9]
        acc_gx, acc_gy, acc_gz = tup[10], tup[11], tup[12]
        lin_ax, lin_ay, lin_az = tup[13], tup[14], tup[15]
        gyr_x, gyr_y, gyr_z = tup[16], tup[17], tup[18]
        mag_x, mag_y, mag_z = tup[19], tup[20], tup[21]
        bmp_ok, temp_cc, press_dhpa, alt_dm = tup[22], tup[23], tup[24], tup[25]
        lora_tx = tup[26] if fmt_ver == 2 else None
        lora_ack = tup[27] if fmt_ver == 2 else None

        def r2(x):
            return round(x / 100.0, 2)

        def r3(x):
            return round(x / 100.0, 3)

        def r1(x):
            return round(x / 10.0, 1)

        line = (
            f"TR,{t_ms},{cs},{cg},{ca},{cm},"
            f"{r2(euler_h_cd)},{r2(euler_r_cd)},{r2(euler_p_cd)},"
            f"{r3(acc_gx)},{r3(acc_gy)},{r3(acc_gz)},"
            f"{r3(lin_ax)},{r3(lin_ay)},{r3(lin_az)},"
            f"{r3(gyr_x)},{r3(gyr_y)},{r3(gyr_z)},"
            f"{r3(mag_x)},{r3(mag_y)},{r3(mag_z)},"
            f"{bmp_ok},{r2(temp_cc)},{r1(press_dhpa)},{r1(alt_dm)}"
        )
        if fmt_ver == 2:
            line += f",{lora_tx},{lora_ack}"
        print(line)


if __name__ == "__main__":
    main()
