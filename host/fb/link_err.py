#!/usr/bin/env python3
"""link_err.py — board-link error rate on board A's receiver (B -> A lane) under different loads.

Reads board A's STATUS (layout 2) before and after each phase and prints the CRC / parity / discard
deltas per minute, the trained eye and the number of B STATUS packets. Loads (one per phase):
  idle     nothing
  fence    FENCE commands back to back (each one a CMD packet A -> B and an EVENT packet B -> A)
  amem     MEMTEST on board A's DDR (512 MB blocks at 512 MB, A's DDR busy the whole phase)
  eth      only STATUS polling every 50 ms (light Ethernet traffic; use udp_blast.ps1 for line rate)

  python3 link_err.py --phases idle,amem,idle,fence --secs 120
  python3 link_err.py --linkctl 0 --secs 600           # board A's fill words off (REG_LINKCTL) during the run

With STATUS layout 3 each phase also prints board A's receive diagnostics (fill words, error histogram by
position in the burst, last error).
"""
import argparse
import sys
import time

import fbproto as fp


def snap(b):
    st = b.status()
    lf = fp.decode_link_flags(st["link_flags"])
    return {"t": time.monotonic(), "crc": st["link_crc_par"] >> 16, "par": st["link_crc_par"] & 0xFFFF,
            "disc": st["link_disc_retx"] >> 16, "rew": st["link_disc_retx"] & 0xFFFF, "eye": lf["eye"],
            "trained": lf["trained"], "bst": st["b_status_pkts"], "bev": st["b_events"], "fwd": st["fwd"],
            "phase": lf["phase"]}


def d16(a, b):
    return (b - a) & 0xFFFF


def run_phase(b, name, secs):
    s0 = snap(b)
    t_end = s0["t"] + secs
    n = lost = 0
    while time.monotonic() < t_end:
        if name == "fence":
            try:
                b.fence(timeout=1.0)
            except fp.ProtoError:
                lost += 1           # EVENT datagram lost between board A and this host (UDP, not the link)
            n += 1
        elif name == "amem":
            errors, _ = b.memtest(0x2000_0000, 0x2000_0000, seed=0x1234 + n, timeout=60.0)
            n += 1
            if errors:
                print("  A MEMTEST errors %d" % errors)
        elif name == "eth":
            b.status()
            n += 1
            time.sleep(0.05)
        else:
            time.sleep(1.0)
    s1 = snap(b)
    mins = (s1["t"] - s0["t"]) / 60.0
    crc, par, disc = d16(s0["crc"], s1["crc"]), d16(s0["par"], s1["par"]), d16(s0["disc"], s1["disc"])
    if lost:
        print("  %d EVENT datagrams not received by the host" % lost)
    print("%-6s %5.1f s  ops %6d | crc %3d par %3d disc %3d -> %.1f crc/min %.1f par/min | eye %d->%d phase %d trained %d"
          " | B status +%d events +%d"
          % (name, mins * 60, n, crc, par, disc, crc / mins, par / mins, s0["eye"], s1["eye"], s1["phase"],
             s1["trained"], s1["bst"] - s0["bst"], s1["bev"] - s0["bev"]))
    st = b.status()
    if "ld_hist" in st:
        h, le = st["ld_hist"], st["ld_last_err"]
        print("         fill A %d B %d | A rx: fills %d (parity %d), real-word parity %d, hist pos0/1-3/4+/long-gap %d/%d/%d/%d, "
              "bursts %d (long gap %d), lock losses %d, trainings %d, last error gap %d pos %d par %X"
              % (st["linkctl"] & 1, (st["b_link_info"] >> 16) & 1, st["ld_fills"], st["ld_fill_perr"], st["ld_perrs"],
                 h >> 24, (h >> 16) & 255, (h >> 8) & 255, h & 255, st["ld_bursts"], st["ld_bursts_lg"], st["link_drops"],
                 st["link_frm_trn"] & 255, le >> 16, (le >> 8) & 255, le & 15))
    sys.stdout.flush()
    return crc, par, mins


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=fp.BOARD_IP)
    ap.add_argument("--phases", default="idle")
    ap.add_argument("--secs", type=float, default=120.0)
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--linkctl", type=lambda s: int(s, 0), help="write board A's REG_LINKCTL first (bit 0 = fill words)")
    a = ap.parse_args()
    b = fp.Board(a.ip)
    try:
        b.hello()
        if a.linkctl is not None:
            print("REG_LINKCTL := %d (board A fill words %s)" % (b.reg_write(fp.REG_LINKCTL, a.linkctl), "on" if a.linkctl & 1 else "off"))
        tot = {}
        for _ in range(a.repeat):
            for ph in a.phases.split(","):
                crc, par, mins = run_phase(b, ph, a.secs)
                t = tot.setdefault(ph, [0, 0, 0.0])
                t[0] += crc; t[1] += par; t[2] += mins
        if a.repeat > 1 or len(tot) > 1:
            for ph, (crc, par, mins) in tot.items():
                print("total %-6s %6.1f min: %.2f crc/min %.2f par/min" % (ph, mins, crc / mins, par / mins))
    except fp.ProtoError as e:
        print("ERROR:", e)
        return 1
    finally:
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
