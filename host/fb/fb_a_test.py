#!/usr/bin/env python3
"""fb_a_test — board A bring-up acceptance test.
   python3 fb_a_test.py [--ip 192.168.1.10] [--mb 64] [--memtest-mb 64] [--quick]
Checks: HELLO/ACK, STATUS, registers, PHY ID over MDIO, DDR MEMTEST, error events, buffer and raw
memory writes/reads at random offsets and lengths (byte exact), write/read throughput, final counters."""
import argparse
import random
import sys
import time

import fbproto as fp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--mb", type=int, default=64, help="MB for the write-throughput test")
    ap.add_argument("--read-mb", type=int, default=16, help="MB for the read-throughput test")
    ap.add_argument("--memtest-mb", type=int, default=64)
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    fails = []

    def check(cond, what):
        print(("  ok   " if cond else "  FAIL ") + what)
        if not cond:
            fails.append(what)

    b = fp.Board(a.ip)
    try:
        exp = b.hello()
        check(exp == 1, "HELLO -> ACK expected seq %s" % exp)
        st = b.status()
        print(fp.format_status(st))
        fl = fp.decode_flags(st["flags"])
        check(fl["calib"] == 1, "DDR calibrated")
        check(fl["link"] == 1, "Ethernet link up (%s)" % fl["speed"])

        b.reg_write(fp.REG_SCRATCH, 0xC0DE1234)
        check(b.reg_read(fp.REG_SCRATCH) == 0xC0DE1234, "scratch register")
        print("  build id %d, IDELAY tap %d" % (b.reg_read(fp.REG_BUILD), b.reg_read(fp.REG_IDELAY) & 31))
        id1, id2 = b.phy_read(2), b.phy_read(3)
        check(id1 == 0x001C and (id2 & 0xFFF0) == 0xC910, "PHY ID %04X %04X (RTL8211F 001C C91x)" % (id1, id2))
        print("  PHY BMCR %04X BMSR %04X ANAR %04X ANLPAR %04X GBCR %04X GBSR %04X PHYSR %04X" % (
            b.phy_read(0), b.phy_read(1), b.phy_read(4), b.phy_read(5), b.phy_read(9), b.phy_read(10),
            b.phy_read(0x1A, page=0xA43)))

        # DDR memtest over part of the heap
        t0 = time.monotonic()
        mt_bytes = (1 if a.quick else a.memtest_mb) << 20
        err, bad = b.memtest(0x2000_0000, mt_bytes, seed=0xA5A5_0001, timeout=60)
        check(err == 0, "MEMTEST %d MB at 512 MB: %d errors%s (%.2f s)" % (
            mt_bytes >> 20, err, "" if err == 0 else " first at %08X" % bad, time.monotonic() - t0))
        err, bad = b.memtest(0x2000_0000, mt_bytes, seed=~0xA5A5_0001 & 0xFFFFFFFF, timeout=60)
        check(err == 0, "MEMTEST inverted seed: %d errors" % err)

        # error events
        for name, fn, code in [
                ("write to a missing handle", lambda: b.send_seq(fp.T_WRITE_BUFFER, b"\x09\x00\x00\x00\x00\x00\x00\x00abcd"), 2),
                ("buffer outside the heap", lambda: b.create_buffer(10, 0x0100_0000, 4096), 4),
                ("image command without board B", lambda: b.send_seq(fp.T_CREATE_IMAGE, b"\x01\xC0" + bytes(10)), 5)]:
            seq = fn()
            try:
                b.wait_event(seq, 2)
                check(False, "%s -> error event" % name)
            except fp.ProtoError as e:
                check(fp.ERR_NAMES[code] in str(e), "%s -> %s" % (name, e))
        b.take_errors()                                  # the three deliberate errors above

        # buffer round trips at random offsets and lengths (DDR holds old data: zero-fill first)
        rnd = random.Random(1)
        H, BASE, SIZE = 1, fp.HEAP_A, 4 << 20
        b.create_buffer(H, BASE, SIZE)
        b.write_buffer(H, 0, bytes(SIZE))
        ref = bytearray(SIZE)
        for i in range(10 if a.quick else 60):
            off = rnd.randrange(0, SIZE - 20000)
            n = rnd.choice([1, 2, 3, 5, 31, 33, 1439, 1440, 1441, 4000, 17000])
            d = rnd.randbytes(n)
            b.write_buffer(H, off, d)
            ref[off:off + n] = d
            if i % 7 == 6:
                o2 = rnd.randrange(0, SIZE - 30000)
                n2 = rnd.randrange(1, 30000)
                got = b.read_buffer(H, o2, n2)
                if got != bytes(ref[o2:o2 + n2]):
                    check(False, "buffer read %d bytes at %d" % (n2, o2))
        b.wait_acked()
        got = b.read_buffer(H, 0, 1 << 20)
        check(got == bytes(ref[:1 << 20]), "buffer: random writes, 1 MB readback byte exact")
        d = rnd.randbytes(5000)
        b.mem_write(0x3000_0003, d)
        check(b.mem_read(0x3000_0001, 5004)[2:5002] == d, "raw memory write/read at odd addresses")
        b.fence()
        check(not b.take_errors(), "no unexpected error events")

        # throughput
        if not a.quick:
            n = a.mb << 20
            data = random.Random(7).randbytes(n)
            b.create_buffer(2, 0x1000_0000, n)
            t0 = time.monotonic()
            b.write_buffer(2, 0, data)
            b.fence(timeout=60)
            dt = time.monotonic() - t0
            print("  write throughput: %d MB in %.2f s = %.1f MB/s (resent datagrams %d)" % (a.mb, dt, a.mb / dt, b.n_resent))
            rn = a.read_mb << 20
            t0 = time.monotonic()
            got = b.read_buffer(2, 0, rn)
            dt = time.monotonic() - t0
            print("  read throughput: %d MB in %.2f s = %.1f MB/s" % (a.read_mb, dt, a.read_mb / dt))
            check(got == data[:rn], "throughput data byte exact")

        st = b.status()
        print(fp.format_status(st))
        check(st["rx_fcs"] == 0, "no FCS errors (%d)" % st["rx_fcs"])
    finally:
        b.close()
    print("RESULT: %s" % ("PASS" if not fails else "FAIL (%d): %s" % (len(fails), "; ".join(fails))))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
