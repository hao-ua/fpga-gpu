#!/usr/bin/env python3
"""link_soak.py — board-link soak under image streaming.

Writes the smooth test picture to board B back to back with WRITE_IMAGE (the payload that broke the forward lane
before the payload scrambler), alternating storage / transfer formats, with a FENCE after every --fence-every frames
(each one an EVENT packet on the backward lane). Every --report seconds it prints what moved in the link counters of
both receivers (board A's STATUS, layout 3 or 4): CRC / parity / discards / real-word parity / frame errors / lock
losses / trainings on A, CRC / parity / real-word parity / trainings on B, B's command errors, EV_ERROR events,
host-level datagram resends; with layout 4 also both boards' XOFF events, ARQ timeouts / rewinds / NAKs, the words
received (for an error rate per word) and B's error histogram. The soak passes when no error counter moved, no
retrain happened and B received exactly the CMD packets A forwarded (layout 4).

  python3 link_soak.py --secs 3600 --report 60
  python3 link_soak.py --secs 300 --formats rgba8:bgrx,rgb565:rgb565 --window 8 --present
"""
import argparse
import sys
import time

import fbproto as fp
import fb_image


def d16(a, b):
    return (b - a) & 0xFFFF


def d32(a, b):
    return (b - a) & 0xFFFFFFFF


def snap(b):
    st = b.status()
    lf = fp.decode_link_flags(st["link_flags"])
    s = {
        "t": time.monotonic(),
        # board A's receiver (backward lane) and transmitter
        "a_crc": st["link_crc_par"] >> 16, "a_par": st["link_crc_par"] & 0xFFFF,
        "a_disc": st["link_disc_retx"] >> 16, "a_rew": st["link_disc_retx"] & 0xFFFF,
        "a_perr": st["ld_perrs"], "a_fillperr": st["ld_fill_perr"], "a_fills": st["ld_fills"],
        "a_drops": st["link_drops"], "a_frm": st["link_frm_trn"] >> 16, "a_trn": st["link_frm_trn"] & 0xFF,
        "eye": lf["eye"], "phase": lf["phase"], "trained": lf["trained"], "lock": lf["lock"], "b_alive": lf["b_alive"],
        "fill_a": st["linkctl"] & 1, "fill_b": (st["b_link_info"] >> 16) & 1,
        # board B's receiver (forward lane) and command processor
        "b_crc": st["b_link_crc_par"] >> 16, "b_par": st["b_link_crc_par"] & 0xFFFF,
        "b_rwpar": st["b_link_info"] & 0xFFFF, "b_trn": st["b_link_info"] >> 24,
        "b_cmds": st["b_cp"] & 0xFFFF, "b_errs": st["b_cp"] >> 16, "b_pixels": st["b_pixels"],
        "fwd": st["fwd"], "b_ev": st["b_events"], "b_st": st["b_status_pkts"],
        # host <-> board A datagrams
        "h_dup": st["dup"], "h_gap": st["gap"],
    }
    if "b_ld_hist" in st:                                   # layout 4
        s.update({
            "b_hist": st["b_ld_hist"], "b_last": st["b_ld_last_err"], "b_parmap": st["b_link_parmap"],
            "b_xoff": st["b_xoff"], "b_to": st["b_retx"] >> 16, "b_rew": st["b_retx"] & 0xFFFF,
            "b_naks": st["b_disc"] >> 16, "b_disc": st["b_disc"] & 0xFFFF, "b_cmd_pkts": st["b_cmd_pkts"],
            "b_words": st["b_rx_words"], "a_xoff": st["a_xoff"], "a_naks": st["a_naks_to"] >> 16,
            "a_to": st["a_naks_to"] & 0xFFFF, "a_words": st["a_rx_words"],
        })
    return s


# counters whose movement is an error (16-bit wrapping ones first), and informational ones
ERR16 = ("a_crc", "a_par", "a_disc", "a_frm", "b_crc", "b_par", "b_rwpar", "b_errs")
ERR32 = ("a_perr",)
ERR8 = ("a_trn", "b_trn")
INFO16 = ("a_rew", "b_cmds", "b_to", "b_rew", "b_naks", "b_disc", "a_naks", "a_to")
INFO32 = ("a_fillperr", "a_drops", "fwd", "b_ev", "b_st", "h_dup", "h_gap", "b_xoff", "a_xoff", "b_cmd_pkts", "b_words",
          "a_words", "b_pixels")


def delta(s0, s1):
    d = {}
    for k in ERR16 + INFO16:
        d[k] = d16(s0[k], s1[k]) if k in s1 else 0
    for k in ERR32 + INFO32:
        d[k] = d32(s0[k], s1[k]) if k in s1 else 0
    for k in ERR8:
        d[k] = (s1[k] - s0[k]) & 0xFF
    return d


def n_err(d, ev_errors):
    return sum(d[k] for k in ERR16 + ERR32 + ERR8) + ev_errors


def fmt_line(tag, s1, d, frames, mbytes, secs, resent, ev_errors):
    err = n_err(d, ev_errors)
    line = ("%s %6.0f s  frames %6d (%.1f /s, %.1f MB/s) resent %d | A rx crc +%d par +%d disc +%d perr +%d frm +%d "
            "drops +%d trn +%d eye %d ph %d | A tx rew +%d fillperr +%d | B rx crc +%d par +%d rwpar +%d trn +%d | "
            "B cmds +%d errs +%d | EV_ERROR %d | host dup +%d gap +%d"
            % (tag, secs, frames, frames / secs if secs else 0.0, mbytes / secs if secs else 0.0, resent,
               d["a_crc"], d["a_par"], d["a_disc"], d["a_perr"], d["a_frm"], d["a_drops"], d["a_trn"], s1["eye"], s1["phase"],
               d["a_rew"], d["a_fillperr"], d["b_crc"], d["b_par"], d["b_rwpar"], d["b_trn"], d["b_cmds"], d["b_errs"],
               ev_errors, d["h_dup"], d["h_gap"]))
    if "b_words" in s1:
        line += (" | B words +%.3fG xoff +%d tx to/rew +%d/+%d rx naks/disc +%d/+%d hist %08X | A words +%.3fG xoff +%d "
                 "naks/to +%d/+%d | fwd +%d = B cmd pkts +%d"
                 % (d["b_words"] / 1e9, d["b_xoff"], d["b_to"], d["b_rew"], d["b_naks"], d["b_disc"], s1["b_hist"],
                    d["a_words"] / 1e9, d["a_xoff"], d["a_naks"], d["a_to"], d["fwd"], d["b_cmd_pkts"]))
    if err:
        line += "   <-- ERRORS"
    return line


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=fp.BOARD_IP)
    ap.add_argument("--secs", type=float, default=3600.0)
    ap.add_argument("--report", type=float, default=60.0, help="seconds between counter reports")
    ap.add_argument("--formats", default="rgba8:bgrx,rgb565:rgb565,rgba8:rgb,rgb565:bgrx",
                    help="comma list of storage:transfer pairs cycled frame by frame")
    ap.add_argument("--fence-every", type=int, default=1, help="FENCE after every N frames (0: never)")
    ap.add_argument("--window", type=int, default=64, help="host send window (datagrams in flight)")
    ap.add_argument("--index", type=int, default=33, help="first image table index (one image per storage format)")
    ap.add_argument("--base", type=lambda s: int(s, 0), default=0x2100_0000, help="DDR address of the first image")
    ap.add_argument("--present", action="store_true", help="present the first image once at the start")
    ap.add_argument("--verify", action="store_true", help="read the last frame back at the end and compare")
    a = ap.parse_args()

    pairs = []
    for item in a.formats.split(","):
        s, t = item.split(":")
        pairs.append((fp.FMT_RGBA8 if s == "rgba8" else fp.FMT_RGB565, fp.TF_NAMES[t], s, t))
    b = fp.Board(a.ip, window=a.window)
    rc = 1
    try:
        b.hello()
        st = b.status()
        if not fp.decode_link_flags(st["link_flags"])["b_alive"]:
            print("board B is not reachable over the board link"); return 1
        mode = fp.decode_b_flags(st["b_flags"])["mode"]
        w, h = fb_image.MODE_SIZE.get(mode, (800, 600))
        print("mode %d: %dx%d test picture, formats %s, fence every %d frame(s), window %d, STATUS layout %d"
              % (mode, w, h, a.formats, a.fence_every, a.window, st["layout"]))
        sys.stdout.flush()
        _, _, rgb = fb_image.test_picture(w, h)
        # one image per storage format, pixel buffers per pair
        images, bufs = {}, []
        base = a.base
        for sfmt, tfmt, sname, tname in pairs:
            if sfmt not in images:
                bpp = fp.STORAGE_BPP[sfmt]
                pitch = (w * bpp + 31) & ~31
                handle = fp.image_handle_b(a.index + len(images))
                b.create_image(handle, base, w, h, sfmt, pitch=pitch,
                               usage=fp.USE_SAMPLED | fp.USE_SCANOUT | fp.USE_TRANSFER)
                images[sfmt] = (handle, base, pitch)
                base += (pitch * h + 0xFFFFF) & ~0xFFFFF
            bufs.append((images[sfmt][0], fb_image.to_transfer(rgb, tfmt, sfmt), tfmt, sfmt, "%s:%s" % (sname, tname)))
        b.fence(timeout=30.0)
        errs = b.take_errors()
        if errs:
            for e in errs:
                print("EV_ERROR seq %d: %s (type %02X, detail %08X)" % (e[0], fp.ERR_NAMES.get(e[1], e[1]), e[2], e[3]))
            return 1
        if a.present:
            b.present(bufs[0][0], frame_id=a.index)
        s_start = s_rep = snap(b)
        print("start: fill A %d B %d, A eye %d phase %d, B alive %d, A rx crc %d par %d disc %d perr %d, B rx crc %d par %d"
              % (s_start["fill_a"], s_start["fill_b"], s_start["eye"], s_start["phase"], s_start["b_alive"],
                 s_start["a_crc"], s_start["a_par"], s_start["a_disc"], s_start["a_perr"], s_start["b_crc"], s_start["b_par"]))
        sys.stdout.flush()
        frames = rep_frames = 0
        mbytes = rep_mbytes = 0.0
        resent0 = rep_resent = b.n_resent
        ev_total = 0
        t0 = time.monotonic()
        last = None
        try:
            while time.monotonic() - t0 < a.secs:
                handle, px, tfmt, sfmt, name = bufs[frames % len(bufs)]
                b.write_image(handle, px, tfmt, 0, 0, w, h, storage_fmt=sfmt)
                frames += 1
                mbytes += len(px) / 1e6
                last = (handle, px, tfmt, sfmt, name)
                if a.fence_every and frames % a.fence_every == 0:
                    b.fence(timeout=30.0)
                now = time.monotonic()
                if now - s_rep["t"] >= a.report:
                    s1 = snap(b)
                    d = delta(s_rep, s1)
                    errs = b.take_errors()
                    for e in errs:
                        print("EV_ERROR seq %d: %s (type %02X, detail %08X)" % (e[0], fp.ERR_NAMES.get(e[1], e[1]), e[2], e[3]))
                    ev_total += len(errs)
                    print(fmt_line("  ", s1, d, frames - rep_frames, mbytes - rep_mbytes, s1["t"] - s_rep["t"],
                                   b.n_resent - rep_resent, len(errs)))
                    if n_err(d, len(errs)) and "b_last" in s1:
                        print("         B last error: gap %d pos %d last %d par %X, B parmap %08X, A last error gap %d pos %d par %X"
                              % (s1["b_last"] >> 16, (s1["b_last"] >> 8) & 255, (s1["b_last"] >> 4) & 1, s1["b_last"] & 15,
                                 s1["b_parmap"], 0, 0, 0))
                    sys.stdout.flush()
                    s_rep, rep_frames, rep_mbytes, rep_resent = s1, frames, mbytes, b.n_resent
        except KeyboardInterrupt:
            print("interrupted")
        if a.fence_every == 0:
            b.fence(timeout=30.0)
        s_end = snap(b)
        d = delta(s_start, s_end)
        errs = b.take_errors()
        for e in errs:
            print("EV_ERROR seq %d: %s (type %02X, detail %08X)" % (e[0], fp.ERR_NAMES.get(e[1], e[1]), e[2], e[3]))
        ev_total += len(errs)
        err_total = n_err(d, ev_total)
        secs = s_end["t"] - s_start["t"]
        print(fmt_line("ALL", s_end, d, frames, mbytes, secs, b.n_resent - resent0, ev_total))
        print("B pixels written +%d (expected %d mod 2^32), commands forwarded +%d, B status packets +%d, B events +%d"
              % (d["b_pixels"], (frames * w * h) & 0xFFFFFFFF, d["fwd"], d["b_st"], d["b_ev"]))
        if d["b_pixels"] != (frames * w * h) & 0xFFFFFFFF:
            print("MISMATCH: B pixel count"); err_total += 1
        if "b_cmd_pkts" in s_end:
            if d["b_cmd_pkts"] != d["fwd"]:
                print("MISMATCH: B received %d CMD packets, A forwarded %d" % (d["b_cmd_pkts"], d["fwd"])); err_total += 1
            b_ev = d["b_crc"] + d["b_par"]
            a_ev = d["a_crc"] + d["a_par"]
            print("rates: forward lane %d events in %.3f G words (%.1e / word); backward lane %d events in %.3f G words (%s), "
                  "fill-word parity %d in ~%.1f G fill words"
                  % (b_ev, d["b_words"] / 1e9, b_ev / d["b_words"] if d["b_words"] else 0.0, a_ev, d["a_words"] / 1e9,
                     ("%.1e / word" % (a_ev / d["a_words"])) if d["a_words"] and a_ev else "0", d["a_fillperr"],
                     secs * 1.0e8 / 1e9))
        if a.verify and last:
            handle, px, tfmt, sfmt, name = last
            _, ibase, pitch = images[sfmt]
            t2 = time.monotonic()
            mem = b.mem_read(ibase, pitch * h, board=1)
            tb = fp.transfer_bpp(tfmt, sfmt)
            bpp = fp.STORAGE_BPP[sfmt]
            cache, bad = {}, 0
            for y in range(h):
                for x in range(w):
                    i = y * w + x
                    src = bytes(px[i * tb:(i + 1) * tb])
                    exp = cache.get(src)
                    if exp is None:
                        exp = cache[src] = fp.to_storage(tfmt, sfmt, src)
                    o = y * pitch + x * bpp
                    if mem[o:o + bpp] != exp:
                        bad += 1
            print("verify (%s): %d bytes read back in %.1f s, %d pixels differ" % (name, len(mem), time.monotonic() - t2, bad))
            err_total += bad
        rc = 0 if err_total == 0 else 1
        print("SOAK %s: %d frames in %.0f s, %d link / command errors" % ("PASS" if rc == 0 else "FAIL", frames, secs, err_total))
    except fp.ProtoError as e:
        print("ERROR:", e)
        return 1
    finally:
        b.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
