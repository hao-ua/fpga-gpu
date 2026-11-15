#!/usr/bin/env python3
"""fb_video.py — play video on board B's HDMI output over Ethernet (WRITE_IMAGE + PRESENT per frame).

Sources (exactly one):
  FILE / URL     any video ffmpeg can decode (ffmpeg and ffprobe on PATH, or --ffmpeg); --loop repeats it,
                 --start seeks. Board B has no audio output: the sound is not played.
  --test         a generated animation, no ffmpeg needed: colours scrolling left with 8 vertical white lines (a tear
                 shows as a broken line), a bouncing box and the frame number (skipped frames show as jumps)
  --raw FILE|-   raw frames already in the transfer format (--xfer), FILE or '-' for stdin; needs --size

The picture is scaled to fit the current video mode keeping its aspect (--stretch fills the mode, --size W x H
sets the video area) and centred. Two images in board B's DDR (handles 0xC000 + --index and + 1, full mode size,
black around the video) are used in turn: each frame is written into the image that is not on screen and
presented with the hold flag, so board B runs the next frame's writes only after this frame became current at
vsync. The image being written is never the one being scanned out (no tearing) and the host needs no round trip
per frame.

Cadence: board B's refresh is the clock. Source frame i owns the vsyncs [n(i), n(i+1)) with n(i) = round(i x
refresh / fps), e.g. 2 vsyncs per frame for 30 fps at 60 Hz, 3-2-3-2 for 24 fps. The frame is presented (hold)
once per vsync of its slot, so it stays on screen exactly that long whatever the network jitter, as long as the
next frame's data is in board A's queue in time; the host keeps up to --depth frames in flight for that. A frame
whose slot is already over when the host gets to it is skipped (counted; --no-skip plays every frame, slower
than real time); a frame with an empty slot (source faster than the refresh) is never sent.

Bandwidth: a frame costs width x height x bytes per pixel of the transfer format (rgb565 2, rgb / bgr 3, the rest 4).
The host -> board A path sustains about 50 MB/s, e.g. 800x450 RGB565 at 30 fps = 21.6 MB/s.

  python3 fb_video.py --test                        # test animation at 30 fps in the current mode, until Ctrl+C
  python3 fb_video.py movie.mp4                     # fit to the current mode, RGB565
  python3 fb_video.py movie.mp4 --mode 2 --loop     # 1280x720, endless
  python3 fb_video.py movie.mp4 --size 640x360 --xfer rgb --storage rgba8
  ffmpeg -i x.mkv -vf scale=800:450 -pix_fmt rgb565le -f rawvideo - | python3 fb_video.py --raw - --size 800x450 --fps 25
"""
import argparse
import collections
import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import time

import fbproto as fp
from fb_image import MODE_SIZE, to_transfer

# refresh rate per video mode: pixel clock (MMCM settings) / total pixels per frame (src/rtl/disp/video_modes.svh)
REFRESH = {0: 25.15625e6 / (800 * 525), 1: 40e6 / (1056 * 628), 2: 74.25e6 / (1650 * 750), 3: 71e6 / (1440 * 823)}


def ffmpeg_pix(tfmt, storage):
    """ffmpeg pixel format whose bytes are exactly the transfer format (byte order as fbproto.to_storage)."""
    if tfmt == fp.TF_RAW:
        return "rgb565le" if storage == fp.FMT_RGB565 else "abgr"      # RGBA8 storage bytes: A, B, G, R
    return {fp.TF_RGBA8888: "rgba", fp.TF_BGRA8888: "bgra", fp.TF_BGRX8888: "bgr0", fp.TF_RGB565: "rgb565le",
            fp.TF_RGB888: "rgb24", fp.TF_BGR888: "bgr24"}[tfmt]


def parse_size(s):
    try:
        w, h = s.lower().split("x")
        w, h = int(w), int(h)
    except ValueError:
        raise argparse.ArgumentTypeError("size must look like 800x450")
    if not (1 <= w <= 2048 and 1 <= h <= 2048):
        raise argparse.ArgumentTypeError("size out of range")
    return w, h


def fit(sw, sh, aw, ah):
    """Largest even size with the aspect sw : sh inside aw x ah."""
    if sw * ah >= sh * aw:
        w, h = aw, round(aw * sh / sw)
    else:
        w, h = round(ah * sw / sh), ah
    return max(2, w & ~1), max(2, h & ~1)


# ---------------------------------------------------------------- ffmpeg
def find_ffmpeg(hint):
    ff = hint or shutil.which("ffmpeg")
    if not ff or not (os.path.isfile(ff) or shutil.which(ff)):
        return None, None
    d, name = os.path.split(ff)
    probe = os.path.join(d, "ffprobe" + os.path.splitext(name)[1]) if d else None
    if not probe or not os.path.isfile(probe):
        probe = shutil.which("ffprobe")
    return ff, probe


def _rate(s):
    try:
        n, d = (s or "0/0").split("/")
        return float(n) / float(d) if float(d) else None
    except ValueError:
        return None


def probe_video(ffprobe, src):
    """(display width, height, frames per second) of the first video stream (sample aspect and rotation applied)."""
    r = subprocess.run([ffprobe, "-v", "error", "-select_streams", "v:0", "-show_streams", "-of", "json", src],
                       stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=60)
    streams = json.loads(r.stdout or "{}").get("streams") if r.returncode == 0 else None
    if not streams:
        raise SystemExit("ffprobe found no video stream in %s%s" % (src, (": " + r.stderr.strip()) if r.stderr else ""))
    st = streams[0]
    w, h = int(st["width"]), int(st["height"])
    try:
        n, d = (int(v) for v in st.get("sample_aspect_ratio", "1:1").split(":"))
        sar = n / d if n > 0 and d > 0 else 1.0
    except ValueError:
        sar = 1.0
    rot = 0
    for sd in st.get("side_data_list", []):
        if "rotation" in sd:
            rot = int(float(sd["rotation"]))
    rot = rot or int(float(st.get("tags", {}).get("rotate", 0) or 0))
    dw, dh = w * sar, float(h)
    if rot % 180:                                           # ffmpeg auto-rotates: display size swaps
        dw, dh = dh, dw
    fps = _rate(st.get("avg_frame_rate")) or _rate(st.get("r_frame_rate")) or 30.0
    return dw, dh, fps


# ---------------------------------------------------------------- frame sources: next() -> bytes or None (end)
class StreamSource:
    """Fixed-size frames from a byte stream (ffmpeg's stdout, a file, stdin), read ahead by a thread."""

    def __init__(self, stream, frame_bytes, proc=None, ahead=4):
        self.stream, self.n, self.proc = stream, frame_bytes, proc
        self.q = queue.Queue(maxsize=ahead)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            while True:
                buf = self.stream.read(self.n)              # buffered: n bytes, fewer only at the end
                if not buf or len(buf) < self.n:
                    break
                self.q.put(buf)
        except (OSError, ValueError):
            pass
        finally:
            self.q.put(None)

    def next(self):
        return self.q.get()

    def close(self):
        if self.proc and self.proc.poll() is None:
            self.proc.kill()                                # it may be blocked writing into the full pipe
            self.proc.wait()


def bounce(p, span):
    if span <= 0:
        return 0
    p %= 2 * span
    return p if p <= span else 2 * span - p


class TestSource:
    """Generated animation: hue across x (periodic, scrolls left dx pixels per frame) with 8 white vertical lines,
    brightness falling down the screen, a dark row every 64 lines, a bouncing white box and the frame number."""
    DW, DH, T, GAP, NDIG = 24, 44, 6, 8, 6
    DIGITS = ("abcdef", "bc", "abdeg", "abcdg", "bcfg", "acdfg", "acdefg", "abc", "abcdefg", "abcdfg")

    def __init__(self, w, h, tfmt, storage, dx=4):
        self.w, self.h, self.dx = w, h, dx
        self.tb = tb = fp.transfer_bpp(tfmt, storage)
        self.white = bytes(to_transfer(b"\xff\xff\xff", tfmt, storage))
        self.black = bytes(to_transfer(b"\x00\x00\x00", tfmt, storage))
        hue = bytearray(w * 3)
        for x in range(w):
            hseg = (x * 6 * 256) // w
            k, f = hseg >> 8, hseg & 255
            hue[3 * x:3 * x + 3] = bytes([(255, f, 0), (255 - f, 255, 0), (0, 255, f), (0, 255 - f, 255),
                                          (f, 0, 255), (255, 0, 255 - f)][k % 6])
        spacing = max(2, w // 8)
        tables, rgb = {}, bytearray(w * h * 3)
        for y in range(h):
            v = 255 - (y * 160) // max(1, h - 1)
            if v not in tables:
                tables[v] = bytes((c * v) >> 8 for c in range(256))
            row = bytearray(hue.translate(tables[v])) if y % 64 != 63 else bytearray(b"\x28" * (w * 3))
            for x in range(0, w, spacing):
                n = min(2, w - x)
                row[3 * x:3 * (x + n)] = b"\xff" * (3 * n)
            rgb[y * w * 3:(y + 1) * w * 3] = row
        px = to_transfer(rgb, tfmt, storage)
        rb = w * tb
        self.rows2 = [bytes(px[y * rb:(y + 1) * rb]) * 2 for y in range(h)]
        self.box = max(8, min(w, h) // 8)
        self.i = 0

    def fill(self, buf, x, y, fw, fh, c):
        x0, x1, y0, y1 = max(0, x), min(self.w, x + fw), max(0, y), min(self.h, y + fh)
        if x0 >= x1:
            return
        run = c * (x1 - x0)
        for yy in range(y0, y1):
            o = (yy * self.w + x0) * self.tb
            buf[o:o + len(run)] = run

    def number(self, buf, x, y, n):
        dw, dh, t = self.DW, self.DH, self.T
        seg = {"a": (0, 0, dw, t), "b": (dw - t, 0, t, dh // 2), "c": (dw - t, dh // 2, t, dh - dh // 2),
               "d": (0, dh - t, dw, t), "e": (0, dh // 2, t, dh - dh // 2), "f": (0, 0, t, dh // 2),
               "g": (0, dh // 2 - t // 2, dw, t)}
        self.fill(buf, x - 8, y - 8, self.NDIG * (dw + self.GAP) - self.GAP + 16, dh + 16, self.black)
        for j, ch in enumerate("%0*d" % (self.NDIG, n % 10 ** self.NDIG)):
            for s in self.DIGITS[int(ch)]:
                sx, sy, sw, sh = seg[s]
                self.fill(buf, x + j * (dw + self.GAP) + sx, y + sy, sw, sh, self.white)

    def next(self):
        i, w, h, rb = self.i, self.w, self.h, self.w * self.tb
        self.i += 1
        o = ((i * self.dx) % w) * self.tb
        buf = bytearray(b"".join([r[o:o + rb] for r in self.rows2]))
        self.fill(buf, bounce(i * 5, w - self.box), bounce(i * 3, h - self.box), self.box, self.box, self.white)
        self.number(buf, 16, 16, i)
        return buf

    def close(self):
        pass


# ---------------------------------------------------------------- statistics of the frames that reached the screen
class Shown:
    """Per shown frame: B's vsync count when it became current. Its duration on screen (the next shown frame's
    count minus its own) is compared with the vsyncs it was presented for: longer = the next frame came late."""

    def __init__(self):
        self.n, self.dropped, self.last = 0, 0, None
        self.hist, self.off = collections.Counter(), collections.Counter()

    def take(self, ev, reps):
        _code, _fid, vs, dropped = ev
        self.n += 1
        self.dropped += dropped
        if self.last is not None:
            lvs, lreps = self.last
            dur = (vs - lvs) & 0xFFFFFFFF
            self.hist[dur] += 1
            if dur != lreps:
                self.off[dur - lreps] += 1
        self.last = (vs, reps)

    def summary(self):
        total, bad = sum(self.hist.values()), sum(self.off.values())
        s = "vsyncs on screen per frame %s; %d of %d as planned" % (
            ", ".join("%d: %d" % kv for kv in sorted(self.hist.items())) or "-", total - bad, total)
        return s + (" (others off by %s)" % ", ".join("%+d: %d" % kv for kv in sorted(self.off.items())) if bad else "")


class Counters:
    """Board counters of interest, accumulated over STATUS snapshots taken often enough (about once a second)
    that the 16-bit halves cannot wrap in between."""
    def __init__(self, st):
        self.prev, self.sum = st, collections.OrderedDict(
            (k, 0) for k in ("B underruns", "B command errors", "B commands", "A rx crc", "B rx crc", "A rewinds",
                             "A tx timeouts", "B xoff"))

    def add(self, s1):
        s0, self.prev = self.prev, s1
        def hi(k): return ((s1.get(k, 0) >> 16) - (s0.get(k, 0) >> 16)) & 0xFFFF
        def lo(k): return ((s1.get(k, 0) & 0xFFFF) - (s0.get(k, 0) & 0xFFFF)) & 0xFFFF
        def w(k): return (s1.get(k, 0) - s0.get(k, 0)) & 0xFFFFFFFF
        for k, v in (("B underruns", hi("b_underruns_moderr")), ("B command errors", hi("b_cp")),
                     ("B commands", lo("b_cp")), ("A rx crc", hi("link_crc_par")), ("B rx crc", hi("b_link_crc_par")),
                     ("A rewinds", lo("link_disc_retx")), ("A tx timeouts", lo("a_naks_to")), ("B xoff", w("b_xoff"))):
            self.sum[k] += v

    def sample(self, b):
        try:
            self.add(b.status())
        except fp.ProtoError:                               # a lost STATUS reply: try again next time
            pass


def verify(b, handle_base, pitch, storage, tfmt, frame, ox, oy, vw, vh):
    """Read the video rectangle of the image back (MEM_READ board 1) and compare it with B's conversion."""
    bpp, tb = fp.STORAGE_BPP[storage], fp.transfer_bpp(tfmt, storage)
    t0 = time.monotonic()
    mem = b.mem_read(handle_base + oy * pitch, pitch * vh, board=1)
    cache, bad, first = {}, 0, None
    for y in range(vh):
        for x in range(vw):
            i = y * vw + x
            src = bytes(frame[i * tb:(i + 1) * tb])
            exp = cache.get(src)
            if exp is None:
                exp = cache[src] = fp.to_storage(tfmt, storage, src)
            o = y * pitch + (ox + x) * bpp
            if mem[o:o + bpp] != exp:
                bad += 1
                if first is None:
                    first = (ox + x, oy + y, mem[o:o + bpp].hex(), exp.hex())
    print("verify: last frame read back (%d bytes in %.1f s), %d pixels differ%s"
          % (len(mem), time.monotonic() - t0, bad, "" if not first else " (first at %s: %s expected %s)"
             % (first[:2], first[2], first[3])))
    return bad == 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", nargs="?", help="video file or URL (decoded by ffmpeg)")
    ap.add_argument("--test", action="store_true", help="play the generated test animation")
    ap.add_argument("--raw", metavar="FILE", help="raw frames in the transfer format ('-' = stdin); needs --size")
    ap.add_argument("--ip", default=fp.BOARD_IP)
    ap.add_argument("--mode", type=int, choices=sorted(MODE_SIZE), help="set this video mode first")
    ap.add_argument("--size", type=parse_size, help="video area WxH, centred (default: fit the mode)")
    ap.add_argument("--stretch", action="store_true", help="fill the video area, ignoring the aspect ratio")
    ap.add_argument("--fps", type=float, help="frame rate (files: resampled by ffmpeg; default: the file's, 30 else)")
    ap.add_argument("--loop", action="store_true", help="repeat the file endlessly")
    ap.add_argument("--start", type=float, help="start this many seconds into the file")
    ap.add_argument("--frames", type=int, help="stop after this many source frames")
    ap.add_argument("--secs", type=float, help="stop after this many seconds")
    ap.add_argument("--no-skip", action="store_true", help="never skip late frames (playback slows down instead)")
    ap.add_argument("--depth", type=int, default=3, help="frames in flight: sent, not yet off the screen (default 3)")
    ap.add_argument("--storage", choices=("rgb565", "rgba8"), default="rgb565", help="image format in B's DDR")
    ap.add_argument("--xfer", choices=sorted(fp.TF_NAMES), default="rgb565", help="transfer format of the pixels")
    ap.add_argument("--index", type=int, default=40, help="first image table index of the two buffers (>= 16)")
    ap.add_argument("--base", type=lambda s: int(s, 0), default=0x2800_0000, help="address of the buffers in B's DDR")
    ap.add_argument("--ffmpeg", help="path of the ffmpeg executable (ffprobe is looked for next to it)")
    ap.add_argument("--report", type=float, default=2.0, help="progress line every this many seconds (0: off)")
    ap.add_argument("--verify", action="store_true", help="at the end, read the last frame back and compare")
    a = ap.parse_args()

    if sum((a.source is not None, a.test, a.raw is not None)) != 1:
        ap.error("give exactly one source: a video file, --test or --raw")
    if a.raw is not None and not a.size:
        ap.error("--raw needs --size")
    if a.depth < 1:
        ap.error("--depth must be at least 1")
    storage = fp.FMT_RGBA8 if a.storage == "rgba8" else fp.FMT_RGB565
    tfmt = fp.TF_NAMES[a.xfer]
    tb = fp.transfer_bpp(tfmt, storage)
    ff = probe = None
    if a.source is not None:
        ff, probe = find_ffmpeg(a.ffmpeg)
        if not ff or not probe:
            print("ffmpeg / ffprobe not found. In WSL: sudo apt install ffmpeg (or pass --ffmpeg PATH). "
                  "--test needs neither.")
            return 1

    b = fp.Board(a.ip)
    src, rc = None, 0
    try:
        b.hello()
        st0 = b.status()
        if not fp.decode_link_flags(st0["link_flags"])["b_alive"]:
            print("board B is not reachable over the board link")
            return 1
        if a.mode is not None:
            b.set_mode(a.mode)
        bflags = fp.decode_b_flags(st0["b_flags"])
        mode = a.mode if a.mode is not None else bflags["mode"]
        mw, mh = MODE_SIZE.get(mode, (800, 600))
        area = a.size or (mw, mh)
        if area[0] > mw or area[1] > mh:
            print("video area %dx%d is larger than mode %d (%dx%d)" % (area + (mode, mw, mh)))
            return 1

        # ---- source and video size
        if a.source is not None:
            sw, sh, fps = probe_video(probe, a.source)
            vw, vh = area if a.stretch else fit(sw, sh, *area)
            fps = a.fps or fps
            vf = "scale=%d:%d:flags=bicubic" % (vw, vh) + (",fps=%g" % a.fps if a.fps else "")
            cmd = [ff, "-hide_banner", "-loglevel", "error", "-nostdin"]
            cmd += ["-stream_loop", "-1"] if a.loop else []
            cmd += ["-ss", "%g" % a.start] if a.start else []
            cmd += ["-i", a.source, "-map", "0:v:0", "-an", "-sn", "-vf", vf, "-pix_fmt", ffmpeg_pix(tfmt, storage),
                    "-f", "rawvideo", "-"]
            proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE)
            src = StreamSource(proc.stdout, vw * vh * tb, proc)
            what = "%s (%dx%d, %.3f fps)" % (a.source, round(sw), round(sh), fps)
        elif a.raw is not None:
            vw, vh = a.size
            fps = a.fps or 30.0
            stream = sys.stdin.buffer if a.raw == "-" else open(a.raw, "rb")
            src = StreamSource(stream, vw * vh * tb)
            what = "raw %s (%dx%d %s, %.3f fps)" % (a.raw, vw, vh, ffmpeg_pix(tfmt, storage), fps)
        else:
            vw, vh = area
            fps = a.fps or 30.0
            print("generating the %dx%d test animation ..." % (vw, vh))
            sys.stdout.flush()
            src = TestSource(vw, vh, tfmt, storage)
            what = "test animation (%dx%d, %.3f fps)" % (vw, vh, fps)
        ox, oy = (mw - vw) // 2, (mh - vh) // 2
        fbytes = vw * vh * tb
        print("%s -> %dx%d at (%d, %d) in mode %d (%dx%d); %s transfer -> %s: %.2f MB per frame, %.1f MB/s"
              % (what, vw, vh, ox, oy, mode, mw, mh, a.xfer, a.storage, fbytes / 1e6, fbytes * fps / 1e6))
        if fbytes * fps > 50e6:
            print("  note: above the ~50 MB/s the host -> board A path sustains; some frames may be skipped "
                  "(a smaller --size or a lower --fps help%s)" % (", so does --xfer rgb565" if tb > 2 else ""))

        # ---- the two buffers: full mode size, black around the video
        bpp = fp.STORAGE_BPP[storage]
        pitch = (mw * bpp + 31) & ~31
        stride = (pitch * mh + 0xFFFFF) & ~0xFFFFF
        hb, he = fp.heaps(st0)['b']
        if a.base < hb or a.base + 2 * stride > he:
            print("buffers do not fit in B's heap %08X-%08X (base %08X, 2 x %d bytes)" % (hb, he, a.base, stride))
            return 1
        bufs = [(fp.image_handle_b(a.index + i), a.base + i * stride) for i in range(2)]
        if bflags["cur_img"] == a.index:                    # start with the buffer that is not on screen
            bufs.reverse()
        b.take_errors()
        for handle, base in bufs:
            b.create_image(handle, base, mw, mh, storage, pitch=pitch,
                           usage=fp.USE_SAMPLED | fp.USE_SCANOUT | fp.USE_TRANSFER)
            if (vw, vh) != (mw, mh):
                b.write_image(handle, bytes(mw * mh * tb), tfmt, 0, 0, mw, mh, storage_fmt=storage)
        b.fence(timeout=30.0)
        errs = b.take_errors()
        if errs:
            for e in errs:
                print("EV_ERROR seq %d: %s (type %02X, detail %08X)" % (e[0], fp.ERR_NAMES.get(e[1], e[1]), e[2], e[3]))
            return 1
        cnt = Counters(b.status())
        t_cnt = time.monotonic()

        # ---- play: slot of source frame i = B vsyncs [n(i), n(i+1)) after the first frame became current
        refresh = REFRESH.get(mode, 60.0)
        per = refresh / fps

        def slot(i):
            return int(i * per + 0.5)

        print("playing at %.3f Hz refresh, %.3f vsyncs per frame (Ctrl+C stops)" % (refresh, per))
        sys.stdout.flush()
        shown, pending = Shown(), collections.deque()       # pending: [frame index, present seqs, events]
        i, sent, skipped, empty, nbytes, last = -1, 0, 0, 0, 0, None
        t_first = t_rep = t_zero = None                     # t_zero: host time of vsync n(0)
        rep = (0, 0)                                        # sent, bytes at the last report

        def collect(p):
            for s in p[1]:
                if s not in p[2]:
                    p[2][s] = b.wait_event(s, timeout=5.0)
            shown.take(p[2][p[1][0]], len(p[1]))

        try:
            while True:
                if a.frames is not None and i + 1 >= a.frames:
                    break
                if a.secs is not None and t_first is not None and time.monotonic() - t_first >= a.secs:
                    break
                frame = src.next()
                if frame is None:
                    break
                i += 1
                now = time.monotonic()
                if t_first is None:
                    t_first = t_rep = now
                reps = slot(i + 1) - slot(i)
                if reps == 0:                               # source faster than the refresh
                    empty += 1
                    continue
                if t_zero is not None and not a.no_skip and now > t_zero + slot(i + 1) / refresh:
                    skipped += 1                            # its slot is over already
                    continue
                while len(pending) >= a.depth:
                    collect(pending.popleft())
                handle = bufs[sent & 1][0]
                b.write_image(handle, frame, tfmt, ox, oy, vw, vh, storage_fmt=storage)
                p = [i, [b.present(handle, frame_id=i, hold=True, wait=False) for _ in range(reps)], {}]
                pending.append(p)
                if t_zero is None:                          # the schedule starts when the first frame is shown
                    p[2][p[1][0]] = b.wait_event(p[1][0], timeout=5.0)
                    t_zero = time.monotonic() - slot(i) / refresh
                sent += 1
                nbytes += len(frame)
                last = (bufs[(sent - 1) & 1][1], frame)
                errs = b.take_errors()
                if errs:
                    for e in errs:
                        print("EV_ERROR seq %d: %s (type %02X, detail %08X)"
                              % (e[0], fp.ERR_NAMES.get(e[1], e[1]), e[2], e[3]))
                    rc = 1
                    break
                now = time.monotonic()
                if now - t_cnt >= 1.0:
                    cnt.sample(b)
                    t_cnt = now
                if a.report and now - t_rep >= a.report:
                    dt = now - t_rep
                    print("%7.1f s  frame %6d  sent %6d (%5.1f fps)  shown %6d  skipped %5d  %5.1f MB/s  resent %d"
                          % (now - t_first, i, sent, (sent - rep[0]) / dt, shown.n, skipped,
                             (nbytes - rep[1]) / dt / 1e6, b.n_resent))
                    sys.stdout.flush()
                    t_rep, rep = now, (sent, nbytes)
        except KeyboardInterrupt:
            print("stopped")
        src.close()
        while pending:
            collect(pending.popleft())
        el = time.monotonic() - t_first if t_first else 0.0
        b.fence(timeout=30.0)
        cnt.add(b.status())
        print("played %d of %d source frames in %.1f s (%.1f fps), skipped late %d%s; shown %d (replaced before "
              "shown %d)" % (sent, i + 1, el, sent / el if el else 0, skipped,
                             ", faster than the refresh %d" % empty if empty else "", shown.n, shown.dropped))
        print(shown.summary())
        print("data %.1f MB at %.1f MB/s, resent datagrams %d"
              % (nbytes / 1e6, nbytes / el / 1e6 if el else 0, b.n_resent))
        d = cnt.sum
        print("board counters during playback: " + ", ".join("%s %d" % kv for kv in d.items()))
        if d["B underruns"] or d["B command errors"]:
            rc = 1
        if a.verify and last:
            if not verify(b, last[0], pitch, storage, tfmt, last[1], ox, oy, vw, vh):
                rc = 1
    except fp.ProtoError as e:
        print("ERROR:", e)
        rc = 1
    finally:
        if src:
            src.close()
        b.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
