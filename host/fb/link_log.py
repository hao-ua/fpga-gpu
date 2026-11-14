#!/usr/bin/env python3
"""link_log.py — log board A's board-link counters every --ival seconds; prints a line only when something
changed (CRC / parity / discard counters, eye, phase, trained), so error bursts and retrains show up in time.

  python3 link_log.py --secs 600 --ival 0.5
"""
import argparse
import sys
import time

import fbproto as fp


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=fp.BOARD_IP)
    ap.add_argument("--secs", type=float, default=600.0)
    ap.add_argument("--ival", type=float, default=0.5)
    a = ap.parse_args()
    b = fp.Board(a.ip)
    prev = None
    t0 = time.monotonic()
    n_chg = 0
    try:
        while time.monotonic() - t0 < a.secs:
            st = b.status()
            lf = fp.decode_link_flags(st["link_flags"])
            cur = (st["link_crc_par"] >> 16, st["link_crc_par"] & 0xFFFF, st["link_disc_retx"] >> 16,
                   lf["eye"], lf["phase"], lf["trained"], lf["lock"])
            if prev is None or cur != prev:
                d = [(c - p) & 0xFFFF for c, p in zip(cur[:3], prev[:3])] if prev else [0, 0, 0]
                print("%8.2f s  crc %5d (+%d) par %5d (+%d) disc %5d (+%d)  eye %3d phase %3d trained %d lock %d"
                      % (time.monotonic() - t0, cur[0], d[0], cur[1], d[1], cur[2], d[2], cur[3], cur[4], cur[5], cur[6]))
                sys.stdout.flush()
                n_chg += prev is not None
            prev = cur
            time.sleep(a.ival)
    except fp.ProtoError as e:
        print("ERROR:", e)
        return 1
    finally:
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
