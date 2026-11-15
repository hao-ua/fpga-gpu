#!/usr/bin/env python3
"""link_beat.py — do board A's link receive errors follow the beat between A's and B's 1 ms timers?

Board A sends its IDLE keep-alive every 1 ms of A's clock, board B its STATUS every 1 ms of B's clock; the two
oscillators differ by some ppm, so the two packet trains slide past each other once every 1 ms / ppm. This polls
A's STATUS (uptime in A milliseconds, B STATUS packets = B milliseconds, CRC / parity counters), measures the ppm
difference, and prints the beat phase (0..1) at which each error appeared. Errors bunched at one phase mean the
coincidence of the two packet trains matters.

  python3 link_beat.py --secs 600
"""
import argparse
import sys
import time

import fbproto as fp


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=fp.BOARD_IP)
    ap.add_argument("--secs", type=float, default=600.0)
    ap.add_argument("--ival", type=float, default=0.2)
    a = ap.parse_args()
    b = fp.Board(a.ip)
    rows = []
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < a.secs:
            st = b.status()
            rows.append((st["uptime_ms"], st["b_status_pkts"], st["link_crc_par"] >> 16, st["link_crc_par"] & 0xFFFF))
            time.sleep(a.ival)
    except fp.ProtoError as e:
        print("ERROR:", e)
        return 1
    finally:
        b.close()
    (ua, sa, _, _), (ub, sb, _, _) = rows[0], rows[-1]
    ppm = ((sb - sa) - (ub - ua)) / (ub - ua) * 1e6
    beat_ms = 1.0 / abs(ppm) * 1e6 if ppm else float("inf")
    print("A %d ms, B STATUS %d -> B timer %+.1f ppm against A, beat period %.1f s" % (ub - ua, sb - sa, ppm, beat_ms / 1000))
    phases = []
    for (u0, s0, c0, p0), (u1, s1, c1, p1) in zip(rows, rows[1:]):
        dc, dp = (c1 - c0) & 0xFFFF, (p1 - p0) & 0xFFFF
        if dc or dp:
            # beat phase of the error (A's clock), from the start of the log; +-ival resolution
            ph = (((u0 + u1) / 2 - ua) / beat_ms) % 1.0
            phases.append(ph)
            print("  t %8.1f s  crc +%d par +%d  beat phase %.3f" % (((u0 + u1) / 2 - ua) / 1000, dc, dp, ph))
    if phases:
        hist = [0] * 10
        for ph in phases:
            hist[min(9, int(ph * 10))] += 1
        print("phase histogram (10 bins):", hist)
    return 0


if __name__ == "__main__":
    sys.exit(main())
