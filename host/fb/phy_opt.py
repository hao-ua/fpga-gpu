#!/usr/bin/env python3
"""phy_opt — change the RTL8211F options on board A safely (PHY option trial).
   python3 phy_opt.py [--ip 192.168.1.10] [--rxdly 0|1] [--adv1000 0|1] [--tap N] [--eee 0|1]
--rxdly 1 enables the PHY's 2 ns RX clock delay; --adv1000 0 stops advertising 1000BASE-T (a 100 Mbit/s
link through auto-negotiation); --tap sets the IDELAY tap together with the change. The board re-inits
the PHY and keeps the new settings only if traffic gets through once the link is back; otherwise it
reverts after 8 s (and re-inits again).
--eee sets the PHY's EEE advertisement and restarts auto-negotiation, on its own: no re-init, no trial,
and it lasts until the PHY's next reset, where board A's PHY set-up (mdio_mgr) turns EEE off again."""
import argparse
import time

import fbproto as fp
from eth_rx_sweep import wait_link_status, committed


def set_eee(b, on):
    b.phy_write(0x10, 0x0006 if on else 0x0000, 0x0a5d)
    t0 = time.monotonic()
    try:
        b.phy_write(0x00, 0x1340)
    except fp.ProtoError:
        pass                                    # the link is going down: the answer may not arrive
    time.sleep(1.0)
    if wait_link_status(b, 12.0) is None:
        print("no link 13 s after restarting auto-negotiation")
        return
    print("link back after %.1f s: PHYSR %04X, EEE advertised %04X, by the link partner %04X" % (
        time.monotonic() - t0, b.phy_read(0x1a, 0x0a43), b.phy_read(0x10, 0x0a5d), b.phy_read(0x11, 0x0a5d)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--rxdly", type=int, choices=(0, 1))
    ap.add_argument("--adv1000", type=int, choices=(0, 1))
    ap.add_argument("--tap", type=int)
    ap.add_argument("--eee", type=int, choices=(0, 1))
    a = ap.parse_args()
    b = fp.Board(a.ip)
    try:
        b.hello()
        if a.eee is not None:
            set_eee(b, a.eee)
            return
        fl = fp.decode_flags(b.status()["flags"])
        rx = fl["rxdly"] if a.rxdly is None else a.rxdly
        adv = fl["adv1000"] if a.adv1000 is None else a.adv1000
        val = rx | (adv << 1) | 0x100
        if a.tap is not None:
            val |= 0x200 | ((a.tap & 31) << 16)
        t0 = time.monotonic()
        try:
            b.reg_write(fp.REG_PHYOPT, val)
        except fp.ProtoError:
            pass
        time.sleep(1.0)
        st = wait_link_status(b, 6.5)
        if st is not None and not committed(b):
            st = None
        if st is None:
            print("no traffic after the change: the board reverts to %s, RXDLY %d, tap %d within 8 s" % (
                fl["speed"], fl["rxdly"], fl["tap"]))
            return
        f2 = fp.decode_flags(st["flags"])
        print("committed after %.1f s: %s, RXDLY %d, adv1000 %d, tap %d, PHYSR %04X" % (
            time.monotonic() - t0, f2["speed"], f2["rxdly"], f2["adv1000"], f2["tap"], st["phy"] & 0xFFFF))
    finally:
        b.close()


if __name__ == "__main__":
    main()
