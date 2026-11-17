#!/usr/bin/env python3
"""fb_2d.py — hardware test and demo of board B's DirectDraw-class 2D features:
READ_IMAGE, FILL_IMAGE, COPY_IMAGE (stretch, mirrors, source colour key, overlap), 8-bit palette, hardware cursor
(colour / AND-XOR / premultiplied, fast-path moves), credit flow control.

Every check reads the result back over Ethernet (READ_IMAGE RAW) and compares it with a Python model of the
board's rules (exact RGB565 expansion round(c * 255 / 31), rounding back, nearest stretch at pixel centres,
per-channel key ranges). Then a short visual demo runs on the monitor (--demo-secs, 0 = none).

  python3 fb_2d.py                     # all checks, then a 10 s demo
  python3 fb_2d.py --demo-secs 30      # longer demo (palette animation, moving cursor)
  python3 fb_2d.py --no-checks         # demo only
"""
import argparse
import math
import random
import struct
import sys
import time

import fbproto as fp
from fb_image import MODE_SIZE, test_picture, to_transfer

BASES = {"A": 0x2A00_0000, "B": 0x2B00_0000, "C": 0x2B80_0000, "P": 0x2C00_0000, "R": 0x2C80_0000}


def bpp(fmt):
    return fp.STORAGE_BPP[fmt]


def raw_px(data, fmt, w, x, y):
    b = bpp(fmt)
    o = (y * w + x) * b
    return int.from_bytes(data[o:o + b], "little")


def dec(fmt, v):
    if fmt == fp.FMT_RGB565:
        return fp._e(v >> 11, 5), fp._e((v >> 5) & 63, 6), fp._e(v & 31, 5), 255
    return (v >> 24) & 255, (v >> 16) & 255, (v >> 8) & 255, v & 255


def enc(fmt, r, g, b, a):
    if fmt == fp.FMT_RGB565:
        return (fp._r(r, 5) << 11) | (fp._r(g, 6) << 5) | fp._r(b, 5)
    return (r << 24) | (g << 16) | (b << 8) | a


class Img:
    """A board-B image and the host's copy of its stored pixels (level 0, layer 0, tightly modelled)."""

    def __init__(self, board, index, w, h, fmt, base):
        self.b, self.index, self.w, self.h, self.fmt, self.base = board, index, w, h, fmt, base
        self.handle = fp.image_handle_b(index)
        self.pitch = (w * bpp(fmt) + 31) & ~31
        board.create_image(self.handle, base, w, h, fmt, pitch=self.pitch)
        self.px = [0] * (w * h)

    def write_raw(self, values):
        self.px = list(values)
        b = bpp(self.fmt)
        data = b"".join(v.to_bytes(b, "little") for v in values)
        self.b.write_image(self.handle, data, fp.TF_RAW, 0, 0, self.w, self.h, storage_fmt=self.fmt)

    def read_raw(self):
        return self.b.read_image(self.handle, fp.TF_RAW, 0, 0, self.w, self.h, storage_fmt=self.fmt)

    def check(self, what):
        data = self.read_raw()
        bad = [(i % self.w, i // self.w) for i in range(self.w * self.h)
               if raw_px(data, self.fmt, self.w, i % self.w, i // self.w) != self.px[i]]
        if bad:
            x, y = bad[0]
            raise SystemExit("FAIL %s: %d pixels differ, first at (%d, %d): %08X expected %08X"
                             % (what, len(bad), x, y, raw_px(data, self.fmt, self.w, x, y), self.px[y * self.w + x]))
        print("ok: %s (%d pixels read back)" % (what, self.w * self.h))


def model_fill(img, tf, value, x, y, w, h, mask):
    if tf == fp.TF_RAW:
        fv = value
    else:
        r, g, b, a = dec(fp.FMT_RGBA8, int.from_bytes(fp.to_storage(tf, fp.FMT_RGBA8, value.to_bytes(4, "little")), "little"))
        fv = enc(img.fmt, r, g, b, a) if img.fmt != fp.FMT_P8 else value
    for j in range(h):
        for i in range(w):
            k = (y + j) * img.w + x + i
            ov = img.px[k]
            if img.fmt == fp.FMT_RGBA8:
                nv = ov
                for bit, sh in ((1, 24), (2, 16), (4, 8), (8, 0)):
                    if mask & bit:
                        nv = (nv & ~(0xFF << sh)) | (fv & (0xFF << sh))
            elif img.fmt == fp.FMT_RGB565:
                m = (0xF800 if mask & 1 else 0) | (0x07E0 if mask & 2 else 0) | (0x001F if mask & 4 else 0)
                nv = (ov & ~m) | (fv & m)
            else:
                nv = fv & 0xFF if mask else ov
            img.px[k] = nv & 0xFFFFFFFF


def model_copy(src, dst, sx, sy, sw, sh, dx, dy, dw, dh, mx=False, my=False, key=None, alpha_one=False, swizzle=None,
               tile=None):
    """COPY_IMAGE on the bench's pixels: stretch (pixel centres), mirrors, key; alpha_one / swizzle (r, g, b, a source
    channels) on the RGBA8 values; tile = (px, py): the source rectangle repeated from that phase"""
    snap = [src.px[(sy + v) * src.w + sx + u] for v in range(sh) for u in range(sw)]
    for j in range(dh):
        for i in range(dw):
            u = ((2 * i + 1) * sw) // (2 * dw)
            v = ((2 * j + 1) * sh) // (2 * dh)
            if mx:
                u = sw - 1 - u
            if my:
                v = sh - 1 - v
            if tile is not None:
                u, v = (tile[0] + i) % sw, (tile[1] + j) % sh
            s = snap[v * sw + u]
            if key:
                lo, hi = key
                if src.fmt == fp.FMT_RGBA8:
                    chans = [(24, 8), (16, 8), (8, 8)]
                elif src.fmt == fp.FMT_RGB565:
                    chans = [(11, 5), (5, 6), (0, 5)]
                else:
                    chans = [(0, 8)]
                if all(((lo >> o) & ((1 << n) - 1)) <= ((s >> o) & ((1 << n) - 1)) <= ((hi >> o) & ((1 << n) - 1))
                       for o, n in chans):
                    continue
            if src.fmt == fp.FMT_P8:
                d = s
            else:
                c = list(dec(src.fmt, s))
                if swizzle is not None:
                    c = [c[swizzle[k]] for k in range(4)]
                if alpha_one:
                    c[3] = 255
                d = enc(dst.fmt, *c)
            dst.px[(dy + j) * dst.w + dx + i] = d


def errors_check(b, what):
    b.fence(timeout=30.0)
    errs = b.take_errors()
    if errs:
        raise SystemExit("FAIL %s: EV_ERROR %s" % (what, [(e[0], fp.ERR_NAMES.get(e[1], e[1])) for e in errs]))


def arrow_shape():
    """A white arrow with a black outline (colour mode, straight alpha), semi-transparent shadow."""
    words = [0] * 4096
    for y in range(64):
        for x in range(64):
            if x <= y and x < 40 and y < 48 and x <= 48 - y // 2:
                inside = x < y - 1 and x < 39 and y < 47
                words[y * 64 + x] = 0xFFFFFFFF if inside else 0xFF000000
            elif 2 <= x <= y + 2 and x < 42 and y < 50:
                words[y * 64 + x] = 0x60000000
    return words


def ibeam_shape():
    """A Windows-style inverting I-beam (AND / XOR: XOR white over AND 1 = invert)."""
    words = [1 << 24] * 4096                       # AND 1, XOR 0 = transparent
    for y in range(4, 60):
        for x in range(30, 34):
            words[y * 64 + x] = (1 << 24) | 0xFFFFFF  # invert
    for y in (4, 5, 58, 59):
        for x in range(22, 42):
            words[y * 64 + x] = (1 << 24) | 0xFFFFFF
    return words


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=fp.BOARD_IP)
    ap.add_argument("--index", type=int, default=48, help="first image table index used (5 images)")
    ap.add_argument("--demo-secs", type=float, default=10.0)
    ap.add_argument("--no-checks", action="store_true")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()
    rng = random.Random(a.seed)
    b = fp.Board(a.ip)
    try:
        b.hello()
        st0 = b.status()
        if st0["layout"] < 5:
            print("board A reports STATUS layout %d: the boards run bitstreams without the §20 features" % st0["layout"])
            return 1
        if not fp.decode_link_flags(st0["link_flags"])["b_alive"]:
            print("board B is not reachable over the board link")
            return 1
        mode = fp.decode_b_flags(st0["b_flags"])["mode"]
        mw, mh = MODE_SIZE[mode]
        b.take_errors()
        # images: A = RGBA8 screen, B = RGB565 256x160 source, C = RGBA8 200x120, P = P8 screen, R = RGB565 120x80
        A = Img(b, a.index, mw, mh, fp.FMT_RGBA8, BASES["A"])
        B = Img(b, a.index + 1, 256, 160, fp.FMT_RGB565, BASES["B"])
        C = Img(b, a.index + 2, 200, 120, fp.FMT_RGBA8, BASES["C"])
        P = Img(b, a.index + 3, mw, mh, fp.FMT_P8, BASES["P"])
        R = Img(b, a.index + 4, 120, 80, fp.FMT_RGB565, BASES["R"])
        w, h, rgb = test_picture(mw, mh)
        A.write_raw([int.from_bytes(fp.to_storage(fp.TF_RGB888, fp.FMT_RGBA8, rgb[i * 3:i * 3 + 3]), "little")
                     for i in range(mw * mh)])
        B.write_raw([rng.randrange(65536) for _ in range(256 * 160)])
        C.write_raw([rng.randrange(1 << 32) for _ in range(200 * 120)])
        R.write_raw([((x * 31 // 119) << 11) | ((y * 63 // 79) << 5) | ((x + y) & 31) for y in range(80) for x in range(120)])
        P.write_raw([((x // 8) + (y // 8) * 7) & 255 for y in range(mh) for x in range(mw)])
        errors_check(b, "create / write")
        b.present(A.handle)
        if not a.no_checks:
            t0 = time.monotonic()
            for img, what in ((A, "WRITE_IMAGE RGBA8 screen"), (B, "WRITE_IMAGE RGB565"), (P, "WRITE_IMAGE P8")):
                img.check(what)
            # READ_IMAGE in every transfer format, against from_storage of the stored bytes
            raw = B.read_raw()
            for tf in (fp.TF_RGBA8888, fp.TF_BGRA8888, fp.TF_BGRX8888, fp.TF_RGB565, fp.TF_RGB888, fp.TF_BGR888):
                got = b.read_image(B.handle, tf, 10, 7, 100, 50, storage_fmt=B.fmt)
                tb = fp.transfer_bpp(tf, B.fmt)
                exp = b"".join(fp.from_storage(B.fmt, tf, raw[((7 + j) * 256 + 10 + i) * 2:((7 + j) * 256 + 10 + i) * 2 + 2])
                               for j in range(50) for i in range(100))
                if got != exp:
                    k = next(i for i in range(len(exp)) if got[i] != exp[i])
                    raise SystemExit("FAIL READ_IMAGE format %02X: byte %d = %02X expected %02X" % (tf, k, got[k], exp[k]))
                del tb
            sub = b.read_image(B.handle, fp.TF_RGBA8888, 0, 0, 256, 160, storage_fmt=B.fmt, first=1000, count=777)
            full = b.read_image(B.handle, fp.TF_RGBA8888, 0, 0, 256, 160, storage_fmt=B.fmt)
            if sub != full[4000:4000 + 777 * 4]:
                raise SystemExit("FAIL READ_IMAGE first / count sub-range")
            print("ok: READ_IMAGE RGBA8888 / BGRA / BGRX / RGB565 / RGB888 / BGR888 from RGB565 (exact expansion), sub-range")
            # FILL_IMAGE
            fills = [(A, fp.TF_RGBA8888, 0x80FF4020, 10, 10, 100, 60, 0xF), (A, fp.TF_BGRX8888, 0x00112233, 50, 40, 70, 90, 0x5),
                     (B, fp.TF_RGB565, 0xF81F, 3, 3, 40, 30, 0x7), (B, fp.TF_RGBA8888, 0x00C08040, 60, 20, 90, 70, 0x2),
                     (P, fp.TF_RAW, 0xAB, 100, 100, 60, 40, 0x1)]
            for img, tf, v, x, y, fw, fh, m in fills:
                b.fill_image(img.handle, tf, v, x, y, fw, fh, mask=m)
                model_fill(img, tf, v, x, y, fw, fh, m)
            errors_check(b, "FILL_IMAGE")
            A.check("FILL_IMAGE RGBA8 (byte masks)"); B.check("FILL_IMAGE RGB565 (plain + read-modify-write)"); P.check("FILL_IMAGE P8")
            # COPY_IMAGE: conversion, stretch, mirrors, key, overlap
            copies = [(B, C, 0, 0, 256, 160, 0, 0, 200, 120, False, False, None),          # shrink + RGB565 -> RGBA8
                      (B, C, 20, 20, 30, 20, 100, 50, 90, 60, True, False, None),          # enlarge, mirror x
                      (A, C, 0, 0, 120, 80, 5, 5, 60, 40, False, True, (0x00000000, 0x80FFFF00)),   # key R <= 0x80
                      (C, R, 0, 0, 200, 120, 0, 0, 120, 80, True, True, None),             # RGBA8 -> RGB565 both mirrors
                      (R, R, 0, 0, 100, 60, 7, 5, 100, 60, False, False, None),            # overlap down / right
                      (R, R, 10, 10, 100, 60, 2, 3, 100, 60, False, False, None),          # overlap up / left
                      (P, P, 0, 0, 200, 100, 13, 9, 200, 100, False, False, (0x10, 0x40))] # P8 overlap + index key
            for s, d, sx, sy, sw, sh, dx, dy, dw, dh, mx, my, key in copies:
                b.copy_image(s.handle, d.handle, sx, sy, sw, sh, dx, dy, dw, dh, mirror_x=mx, mirror_y=my, key=key)
                model_copy(s, d, sx, sy, sw, sh, dx, dy, dw, dh, mx, my, key)
                errors_check(b, "COPY_IMAGE")
            C.check("COPY_IMAGE stretch / mirror / key / RGB565 -> RGBA8"); R.check("COPY_IMAGE RGBA8 -> RGB565, overlapping")
            P.check("COPY_IMAGE P8 overlap + key")
            # COPY_IMAGE flags 3..6 (board B STATUS flags bit 14): alpha = 255, swizzle, tiles, lists; queued back to back
            if fp.decode_b_flags(st0["b_flags"]).get("copy_ext"):
                ext = [(B, C, 0, 0, 200, 120, 0, 0, 200, 120, dict(alpha_one=True)),         # RGB565 -> RGBA8, alpha = 255
                       (C, C, 5, 5, 150, 100, 20, 10, 150, 100, {}),                          # one surface (bursts)
                       (A, C, 10, 10, 60, 40, 30, 30, 60, 40, dict(swizzle=(3, 2, 1, 0))),    # R G B A <- A B G R
                       (R, C, 4, 4, 9, 7, 0, 60, 200, 60, dict(tile=(2, 3), alpha_one=True))]  # a 9 x 7 tile
                for s, d, sx, sy, sw, sh, dx, dy, dw, dh, kw in ext:
                    b.copy_image(s.handle, d.handle, sx, sy, sw, sh, dx, dy, dw, dh, **kw)
                    model_copy(s, d, sx, sy, sw, sh, dx, dy, dw, dh, **kw)
                rects = [(40, 20, 30, 10), (15, 50, 60, 8), (90, 60, 25, 15)]                 # a list, source offset (-10, -5)
                b.copy_image(B.handle, R.handle, 0, 0, 20, 10, 10, 5, rects=rects)
                model_copy(B, R, 0, 0, 20, 10, 10, 5, 20, 10)
                for x, y, rw, rh in rects:
                    model_copy(B, R, x - 10, y - 5, rw, rh, x, y, rw, rh)
                errors_check(b, "COPY_IMAGE flags 3..6")
                C.check("COPY_IMAGE alpha = 255 / swizzle / tile"); R.check("COPY_IMAGE list")
                print("ok: COPY_IMAGE alpha = 255, swizzle, tile, list")
            # rejects
            b.copy_image(P.handle, A.handle, 0, 0, 4, 4, 0, 0)               # P8 -> RGBA8
            b.fence(timeout=10.0)
            errs = b.take_errors()
            if len(errs) != 1 or errs[0][1] != 3:
                raise SystemExit("FAIL: COPY P8 -> RGBA8 not rejected with ERR_RANGE (%r)" % errs)
            print("ok: COPY P8 -> RGBA8 rejected; checks done in %.1f s, %d READ_DATA chunks re-requested"
                  % (time.monotonic() - t0, b.n_rerequested))
        # ---------------- demo ----------------
        st1 = b.status()
        pal = [(int(127 + 127 * math.sin(i / 16.0)) << 16) | (int(127 + 127 * math.sin(i / 11.0 + 2)) << 8) |
               int(127 + 127 * math.sin(i / 7.0 + 4)) for i in range(256)]
        b.set_palette(0, pal)
        b.present(P.handle)
        b.cursor_shape(arrow_shape())
        b.cursor_show(fp.CUR_COLOR, hot=(0, 0), pos=(mw // 2, mh // 2))
        t0 = time.monotonic()
        frame = 0
        shown_ibeam = False
        while time.monotonic() - t0 < a.demo_secs:
            t = time.monotonic() - t0
            k = int(t * 60)
            if k != frame:                                                   # palette animation: rotate the colours
                frame = k
                b.set_palette(0, pal[k % 256:] + pal[:k % 256])
            b.cursor_pos(int(mw / 2 + mw / 3 * math.cos(t * 1.3)), int(mh / 2 + mh / 3 * math.sin(t * 2.1)))
            if t > a.demo_secs / 2 and not shown_ibeam:
                b.present(A.handle, wait=False)
                b.cursor_shape(ibeam_shape())
                b.cursor_show(fp.CUR_ANDXOR, hot=(32, 32), take=True)
                shown_ibeam = True
            time.sleep(0.008)
        b.fence(timeout=10.0)
        errs = b.take_errors()
        st2 = b.status()
        d = lambda k: (st2[k] - st1[k]) & 0xFFFFFFFF
        print("demo: %.0f s, palette publishes %d, fast-path cursor packets %d sent / %d received, credit waits %d cycles"
              % (a.demo_secs, (st2["b_cur_pal"] - st1["b_cur_pal"]) & 0xFFFF, ((st2["cur_sent_sync"] >> 16) - (st1["cur_sent_sync"] >> 16)) & 0xFFFF,
                 ((st2["b_cur_pal"] >> 16) - (st1["b_cur_pal"] >> 16)) & 0xFFFF, d("cr_wait")))
        print("link during the run: A tx timeouts %d, B xoff %d, credit syncs %d, EV_ERROR %d"
              % ((st2["a_naks_to"] & 0xFFFF) - (st0["a_naks_to"] & 0xFFFF), (st2["b_xoff"] - st0["b_xoff"]) & 0xFFFFFFFF,
                 ((st2["cur_sent_sync"] & 0xFFFF) - (st0["cur_sent_sync"] & 0xFFFF)) & 0xFFFF, len(errs)))
        if errs:
            return 1
    except fp.ProtoError as e:
        print("ERROR:", e)
        return 1
    finally:
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
