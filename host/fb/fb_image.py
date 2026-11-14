#!/usr/bin/env python3
"""fb_image.py — show a picture on board B's HDMI output over Ethernet (CREATE_IMAGE + WRITE_IMAGE + PRESENT).

The host creates an image in board B's DDR (host-managed heap 256 MB .. 1 GB), writes the pixels with WRITE_IMAGE
datagrams (any transfer format; board B converts to the storage format), presents it and, with --verify, reads the
image memory back with MEM_READ (board 1) and compares it byte for byte with fbproto.to_storage, the reference of
board B's conversion.

Sources: a generated test picture (default) or a file: PNG (8-bit grey / RGB / RGBA, not interlaced), PPM (P6) or
BMP (24 / 32 bit, uncompressed). The picture is shown at the top-left; --fit scales it (nearest neighbour) to the
current video mode.

  python3 fb_image.py                              # test picture in the current mode, RGBA8 storage, BGRX transfer
  python3 fb_image.py --file photo.png --fit --verify
  python3 fb_image.py --mode 2 --storage rgb565 --xfer rgb
"""
import argparse
import struct
import sys
import time
import zlib

import fbproto as fp

MODE_SIZE = {0: (640, 480), 1: (800, 600), 2: (1280, 720), 3: (1280, 800), 4: (1024, 768)}


# ---------------------------------------------------------------- picture sources -> (w, h, RGB bytes)
def load_png(data):
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG file")
    pos, idat, w = 8, bytearray(), 0
    while pos < len(data):
        ln, typ = struct.unpack(">I4s", data[pos:pos + 8])
        body = data[pos + 8:pos + 8 + ln]
        pos += 12 + ln
        if typ == b"IHDR":
            w, h, depth, ctype, _, _, inter = struct.unpack(">IIBBBBB", body)
            if depth != 8 or ctype not in (0, 2, 4, 6) or inter:
                raise ValueError("PNG: only 8-bit grey / RGB / RGBA without interlacing is supported")
        elif typ == b"IDAT":
            idat += body
        elif typ == b"IEND":
            break
    ch = {0: 1, 2: 3, 4: 2, 6: 4}[ctype]
    raw = zlib.decompress(bytes(idat))
    stride = w * ch
    out = bytearray(w * h * 3)
    prev = bytearray(stride)
    p = 0
    for y in range(h):
        f = raw[p]; line = bytearray(raw[p + 1:p + 1 + stride]); p += 1 + stride
        if f == 1:
            for i in range(ch, stride): line[i] = (line[i] + line[i - ch]) & 255
        elif f == 2:
            for i in range(stride): line[i] = (line[i] + prev[i]) & 255
        elif f == 3:
            for i in range(stride): line[i] = (line[i] + ((line[i - ch] if i >= ch else 0) + prev[i]) // 2) & 255
        elif f == 4:
            for i in range(stride):
                a = line[i - ch] if i >= ch else 0; b = prev[i]; c = prev[i - ch] if i >= ch else 0
                pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                line[i] = (line[i] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 255
        prev = line
        o = y * w * 3
        if ch >= 3:
            out[o:o + w * 3:3], out[o + 1:o + w * 3:3], out[o + 2:o + w * 3:3] = line[0::ch], line[1::ch], line[2::ch]
        else:
            g = line[0::ch]
            out[o:o + w * 3:3], out[o + 1:o + w * 3:3], out[o + 2:o + w * 3:3] = g, g, g
    return w, h, bytes(out)


def load_ppm(data):
    tok, pos = [], 0
    while len(tok) < 4:
        while data[pos:pos + 1].isspace(): pos += 1
        if data[pos:pos + 1] == b"#":
            while data[pos:pos + 1] not in (b"\n", b""): pos += 1
            continue
        s = pos
        while not data[pos:pos + 1].isspace(): pos += 1
        tok.append(data[s:pos])
    if tok[0] != b"P6" or int(tok[3]) != 255:
        raise ValueError("PPM: only binary P6 with maxval 255 is supported")
    w, h = int(tok[1]), int(tok[2])
    return w, h, data[pos + 1:pos + 1 + w * h * 3]


def load_bmp(data):
    off, = struct.unpack_from("<I", data, 10)
    w, h, _, bpp, comp = struct.unpack_from("<iiHHI", data, 18)
    if bpp not in (24, 32) or comp not in (0, 3):
        raise ValueError("BMP: only uncompressed 24 / 32-bit is supported")
    bottom_up = h > 0; h = abs(h); bp = bpp // 8
    stride = (w * bp + 3) & ~3
    out = bytearray(w * h * 3)
    for y in range(h):
        sy = h - 1 - y if bottom_up else y
        row = data[off + sy * stride: off + sy * stride + w * bp]
        o = y * w * 3
        out[o:o + w * 3:3], out[o + 1:o + w * 3:3], out[o + 2:o + w * 3:3] = row[2::bp], row[1::bp], row[0::bp]
    return w, h, bytes(out)


def test_picture(w, h):
    """Colour wheel-like gradient (x = hue, y = brightness), white border, corner markers red / green / blue /
    yellow (top-left / top-right / bottom-left / bottom-right), a grey ramp at the bottom and a grid."""
    out = bytearray(w * h * 3)
    for y in range(h):
        v = 255 - (y * 200) // max(1, h - 1)
        for x in range(w):
            hseg = (x * 6 * 256) // w
            k, f = hseg >> 8, hseg & 255
            r, g, b = [(255, f, 0), (255 - f, 255, 0), (0, 255, f), (0, 255 - f, 255), (f, 0, 255), (255, 0, 255 - f)][k % 6]
            c = ((r * v) >> 8, (g * v) >> 8, (b * v) >> 8)
            if y >= h - 24:
                c = ((x * 255) // max(1, w - 1),) * 3
            if x % 64 == 0 or y % 64 == 0:
                c = (40, 40, 40)
            if x == 0 or y == 0 or x == w - 1 or y == h - 1:
                c = (255, 255, 255)
            if 4 <= y < 36 and 4 <= x < 36: c = (255, 0, 0)
            if 4 <= y < 36 and w - 36 <= x < w - 4: c = (0, 255, 0)
            if h - 36 <= y < h - 4 and 4 <= x < 36: c = (0, 0, 255)
            if h - 36 <= y < h - 4 and w - 36 <= x < w - 4: c = (255, 255, 0)
            o = (y * w + x) * 3
            out[o:o + 3] = bytes(c)
    return w, h, bytes(out)


def scale(w, h, rgb, nw, nh):
    out = bytearray(nw * nh * 3)
    for y in range(nh):
        sy = (y * h) // nh
        for x in range(nw):
            s = (sy * w + (x * w) // nw) * 3
            o = (y * nw + x) * 3
            out[o:o + 3] = rgb[s:s + 3]
    return out


# ---------------------------------------------------------------- RGB -> transfer format bytes
def to_transfer(rgb, tfmt, storage):
    n = len(rgb) // 3
    r, g, b = rgb[0::3], rgb[1::3], rgb[2::3]
    if tfmt in (fp.TF_BGRX8888, fp.TF_BGRA8888, fp.TF_RGBA8888):
        out = bytearray(n * 4)
        if tfmt == fp.TF_RGBA8888:
            out[0::4], out[1::4], out[2::4] = r, g, b
        else:
            out[0::4], out[1::4], out[2::4] = b, g, r
        out[3::4] = b"\xff" * n
        return out
    if tfmt in (fp.TF_RGB888, fp.TF_BGR888):
        out = bytearray(n * 3)
        out[0::3], out[1::3], out[2::3] = (r, g, b) if tfmt == fp.TF_RGB888 else (b, g, r)
        return out
    if tfmt == fp.TF_RGB565 or (tfmt == fp.TF_RAW and storage == fp.FMT_RGB565):
        out = bytearray(n * 2)
        for i in range(n):
            v = ((r[i] >> 3) << 11) | ((g[i] >> 2) << 5) | (b[i] >> 3)
            out[2 * i], out[2 * i + 1] = v & 255, v >> 8
        return out
    out = bytearray(n * 4)                                  # RAW into RGBA8 storage: bytes A, B, G, R
    out[0::4], out[1::4], out[2::4], out[3::4] = b"\xff" * n, b, g, r
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.1.10")
    ap.add_argument("--file", help="PNG / PPM / BMP file (default: generated test picture)")
    ap.add_argument("--mode", type=int, choices=sorted(MODE_SIZE), help="set this video mode first")
    ap.add_argument("--fit", action="store_true", help="scale the picture to the mode size")
    ap.add_argument("--index", type=int, default=32, help="image table index (handle 0xC000 + index)")
    ap.add_argument("--base", type=lambda s: int(s, 0), default=0x2000_0000, help="image address in B's DDR")
    ap.add_argument("--storage", choices=("rgba8", "rgb565"), default="rgba8")
    ap.add_argument("--xfer", choices=sorted(fp.TF_NAMES), default="bgrx", help="transfer format of the pixel data")
    ap.add_argument("--verify", action="store_true", help="read the image back (MEM_READ) and compare")
    a = ap.parse_args()
    storage = fp.FMT_RGBA8 if a.storage == "rgba8" else fp.FMT_RGB565
    tfmt = fp.TF_NAMES[a.xfer]
    b = fp.Board(a.ip)
    try:
        b.hello()
        st = b.status()
        if not fp.decode_link_flags(st["link_flags"])["b_alive"]:
            print("board B is not reachable over the board link"); return 1
        if a.mode is not None:
            b.set_mode(a.mode)
        mode = a.mode if a.mode is not None else fp.decode_b_flags(st["b_flags"])["mode"]
        mw, mh = MODE_SIZE.get(mode, (800, 600))
        if a.file:
            data = open(a.file, "rb").read()
            loader = load_png if data[:4] == b"\x89PNG" else load_ppm if data[:2] == b"P6" else load_bmp
            w, h, rgb = loader(data)
            if a.fit and (w, h) != (mw, mh):
                rgb, w, h = scale(w, h, rgb, mw, mh), mw, mh
        else:
            print("generating a %dx%d test picture ..." % (mw, mh)); sys.stdout.flush()
            w, h, rgb = test_picture(mw, mh)
        if w > 2048 or h > 2048:
            print("picture too large (%dx%d, max 2048)" % (w, h)); return 1
        px = to_transfer(rgb, tfmt, storage)
        bpp = fp.STORAGE_BPP[storage]
        pitch = (w * bpp + 31) & ~31                         # rows on 32-byte boundaries (any pitch % bpp == 0 works)
        handle = fp.image_handle_b(a.index)
        b.take_errors()
        t0 = time.monotonic()
        b.create_image(handle, a.base, w, h, storage, pitch=pitch, usage=fp.USE_SAMPLED | fp.USE_SCANOUT | fp.USE_TRANSFER)
        n = b.write_image(handle, px, tfmt, 0, 0, w, h, storage_fmt=storage)
        b.fence(timeout=30.0)
        t1 = time.monotonic()
        errs = b.take_errors()
        if errs:
            for e in errs:
                print("EV_ERROR seq %d: %s (type %02X, detail %08X)" % (e[0], fp.ERR_NAMES.get(e[1], e[1]), e[2], e[3]))
            return 1
        print("image %dx%d (%s storage, %s transfer): %d pixels, %d bytes in %.2f s = %.1f MB/s, resent %d datagrams"
              % (w, h, a.storage, a.xfer, n, len(px), t1 - t0, len(px) / (t1 - t0) / 1e6, b.n_resent))
        fid, vs, _ = b.present(handle, frame_id=a.index)
        print("presented as handle %04X (B vsync count %d)" % (handle, vs))
        if a.verify:
            t2 = time.monotonic()
            mem = b.mem_read(a.base, pitch * h, board=1)
            tb = fp.transfer_bpp(tfmt, storage)
            cache, bad, first = {}, 0, None
            for y in range(h):
                for x in range(w):
                    i = y * w + x
                    src = bytes(px[i * tb:(i + 1) * tb])
                    exp = cache.get(src)
                    if exp is None:
                        exp = cache[src] = fp.to_storage(tfmt, storage, src)
                    o = y * pitch + x * bpp
                    if mem[o:o + bpp] != exp:
                        bad += 1
                        if first is None: first = (x, y, mem[o:o + bpp].hex(), exp.hex())
            print("verify: %d bytes read back in %.2f s, %d pixels differ%s"
                  % (len(mem), time.monotonic() - t2, bad, "" if not first else " (first at %s: %s expected %s)" % (
                      first[:2], first[2], first[3])))
            if bad:
                return 1
    except fp.ProtoError as e:
        print("ERROR:", e)
        return 1
    finally:
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
