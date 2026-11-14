#!/usr/bin/env python3
"""eth_rx_sweep — find the RGMII receive capture window on board A.
   python3 eth_rx_sweep.py [--ip 192.168.1.10] [--rxdly 0|1] [--entry-tap 16] [--frames 3000] [--size 1400] [--set]
For each IDELAY tap 0..31 the tap is applied in trial mode (the board reverts to the last committed tap
after 200 ms if nothing gets through), a burst of large unsequenced datagrams is sent (the board counts
and discards them), and the good-frame / FCS-error counters are read back.
--rxdly switches the PHY's 2 ns RX clock delay first, through a PHY option trial together with
--entry-tap (the board re-inits the PHY and reverts everything after 8 s unless traffic gets through).
The recommended tap is the centre of the error-free window: it maximises the margin measured on both
sides (a window touching tap 0 or 31 is truncated; the eye may continue, but untested taps prove nothing).
Measured on board A at 1000 Mbit/s: RXDLY 0 -> taps 0..11, RXDLY 1 -> taps 7..31; the best
working point is RXDLY 1, tap 19.
Without --set the original RXDLY / tap are restored at the end."""
import argparse
import struct
import time

import fbproto as fp


def wait_link_status(b, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            st = b.status(timeout=0.6)
            if fp.decode_flags(st["flags"])["link"]:
                return st
        except fp.ProtoError:
            pass
        time.sleep(0.3)
    return None


def committed(b, tries=4):
    """True once the board has left trial mode (a STATUS_REQ got through). A periodic STATUS alone proves
    nothing about the receive path, so the trial flag (REG_PHYOPT / REG_IDELAY bit 8) is checked."""
    for _ in range(tries):
        try:
            b.status(timeout=0.4)
            if not (b.reg_read(fp.REG_PHYOPT) & 0x100) and not (b.reg_read(fp.REG_IDELAY) & 0x100):
                return True
        except fp.ProtoError:
            pass
        time.sleep(0.2)
    return False


def phy_trial(b, rxdly, adv1000, tap):
    """Apply RXDLY / advertisement / tap as a PHY option trial and commit it with traffic once the link is up."""
    val = (rxdly & 1) | ((adv1000 & 1) << 1) | 0x100 | 0x200 | ((tap & 31) << 16)
    try:
        b.reg_write(fp.REG_PHYOPT, val)
    except fp.ProtoError:
        pass                                    # the reply can be lost in the re-init; the status check decides
    time.sleep(1.0)
    st = wait_link_status(b, 6.5)
    return st if st is not None and committed(b) else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--rxdly", type=int, choices=(0, 1))
    ap.add_argument("--entry-tap", type=int, default=16)
    ap.add_argument("--frames", type=int, default=3000)
    ap.add_argument("--size", type=int, default=1400)
    ap.add_argument("--set", action="store_true", help="keep RXDLY and apply the recommended tap")
    a = ap.parse_args()
    b = fp.Board(a.ip)
    junk = struct.pack("<HBBI", fp.MAGIC, fp.VERSION, 0x7F, 0) + bytes(range(256)) * (a.size // 256) + bytes(a.size % 256)
    res = {}
    try:
        b.hello()
        st0 = b.status()
        fl0 = fp.decode_flags(st0["flags"])
        orig_rx, orig_adv, orig_tap = fl0["rxdly"], fl0["adv1000"], fl0["tap"]
        rx = orig_rx if a.rxdly is None else a.rxdly
        print("link %s, RXDLY %d, tap %d" % (fl0["speed"], orig_rx, orig_tap))
        if rx != orig_rx:
            st = phy_trial(b, rx, orig_adv, a.entry_tap)
            if st is None:
                print("RESULT: RXDLY %d with entry tap %d does not receive (board reverted to RXDLY %d tap %d)" % (
                    rx, a.entry_tap, orig_rx, orig_tap))
                return
            print("switched to RXDLY %d, tap %d (%s)" % (rx, a.entry_tap, fp.decode_flags(st["flags"])["speed"]))
        for tap in range(32):
            ok = False
            for _ in range(10):                                  # apply in trial mode (retry while marginal)
                try:
                    b.reg_write(fp.REG_IDELAY, fp.IDELAY_TRIAL | tap)
                    ok = True
                    break
                except fp.ProtoError:
                    time.sleep(0.25)
            if not ok:
                res[tap] = None
                print("tap %2d: could not apply" % tap)
                continue
            try:
                s0 = b.status(timeout=0.4)
            except fp.ProtoError:
                res[tap] = None
                print("tap %2d: dead (board reverted)" % tap)
                time.sleep(0.3)
                continue
            for _ in range(a.frames):
                b.sock.sendto(junk, b.addr)
            time.sleep(0.05)
            try:
                s1 = b.status(timeout=0.6)
            except fp.ProtoError:
                res[tap] = None
                print("tap %2d: lost contact" % tap)
                time.sleep(0.3)
                continue
            good, fcs = s1["rx_ok"] - s0["rx_ok"], s1["rx_fcs"] - s0["rx_fcs"]
            bad_other = (s1["rx_len"] - s0["rx_len"]) + (s1["rx_er"] - s0["rx_er"])
            res[tap] = (good, fcs + bad_other)
            print("tap %2d: good %6d  bad %6d  (sent %d)" % (tap, good, fcs + bad_other, a.frames))
        clean = [t for t in range(32) if res.get(t) and res[t][1] == 0 and res[t][0] >= a.frames]
        best, run = [], []
        for t in range(32):
            if t in clean:
                run.append(t)
                if len(run) > len(best):
                    best = list(run)
            else:
                run = []
        if not best:
            print("RESULT: RXDLY %d: no error-free tap" % rx)
            rec = None
        else:
            # The centre maximises the margin that was actually measured on both sides. At a truncated end
            # the eye may continue beyond the IDELAY range, but nothing beyond the last tested tap is known.
            lo_t, hi_t = best[0] == 0, best[-1] == 31
            rec = best[len(best) // 2]
            m = min(rec - best[0], best[-1] - rec) + 1
            note = "centre: >= %d taps (%.2f ns) measured clean on both sides%s" % (
                m, m * 0.078, "; window truncated at tap %s, the eye continues beyond" % (
                    "0 and 31" if lo_t and hi_t else ("0" if lo_t else "31")) if lo_t or hi_t else "")
            print("RESULT: RXDLY %d: error-free taps %d..%d (%d taps = %.2f ns); recommended tap %d (%s)" % (
                rx, best[0], best[-1], len(best), len(best) * 0.078, rec, note))
        if a.set and rec is not None:
            b.reg_write(fp.REG_PHYOPT, rx | (orig_adv << 1) | 0x200 | (rec << 16))   # plain write: commit RXDLY + tap
            print("set: RXDLY %d, tap %d" % (rx, b.reg_read(fp.REG_IDELAY) & 31))
        elif rx != orig_rx:
            st = phy_trial(b, orig_rx, orig_adv, orig_tap)
            print("restored RXDLY %d tap %d: %s" % (orig_rx, orig_tap, "ok" if st else "no link (board reverts by itself)"))
        else:
            b.reg_write(fp.REG_IDELAY, orig_tap)
            print("restored tap %d" % orig_tap)
    finally:
        b.close()


if __name__ == "__main__":
    main()
