#!/usr/bin/env python3
"""fb_mode.py — switch board B's video mode and show its test images over Ethernet (step 3, first slice).

The host talks to board A (UDP); board A forwards SET_MODE / PRESENT / FENCE over the board link to board B.
Board B's built-in test images are table entries 0..9 (handles 0xC000..0xC009); image 2 x mode is the full-screen
RGBA8 image of mode `mode`, 2 x mode + 1 its RGB565 copy stored bottom row first, 8 a small image, 9 a large one.

  python3 fb_mode.py --mode 2                 # 1280x720, then present image 4
  python3 fb_mode.py --mode 1 --image 3       # 800x600 with image 3
  python3 fb_mode.py --image 8 --now          # only present image 8, flip at once
  python3 fb_mode.py --cycle 3                # every mode in turn, 3 s each, with its image, then back
  python3 fb_mode.py --status                 # board A / link / board B status only
"""
import argparse
import sys
import time

import fbproto as fp


def show(b, mode, image, now):
    t0 = time.monotonic()
    if mode is not None:
        b.set_mode(mode)
        t1 = time.monotonic()
        print("mode %d (%s) set in %.1f ms" % (mode, fp.MODE_NAMES[mode], (t1 - t0) * 1000))
        if image is None:
            image = 2 * mode
    if image is not None:
        t1 = time.monotonic()
        fid, vsyncs, dropped = b.present(fp.image_handle(image), frame_id=image, at_once=now)
        print("image %d on screen after %.1f ms (B vsync count %d%s)"
              % (image, (time.monotonic() - t1) * 1000, vsyncs, ", replaced" if dropped else ""))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=fp.BOARD_IP)
    ap.add_argument("--mode", type=int, choices=sorted(fp.MODE_NAMES))
    ap.add_argument("--image", type=int, choices=range(10))
    ap.add_argument("--now", action="store_true", help="flip at once instead of at vsync")
    ap.add_argument("--cycle", type=float, metavar="SECONDS", help="cycle through every mode")
    ap.add_argument("--status", action="store_true")
    a = ap.parse_args()
    b = fp.Board(a.ip)
    try:
        b.hello()
        st = b.status()
        if not fp.decode_link_flags(st["link_flags"])["b_alive"]:
            print(fp.format_status(st))
            print("board B is not reachable over the board link (no STATUS from B)")
            return 1
        if a.cycle:
            start = fp.decode_b_flags(st["b_flags"])["mode"]
            for m in sorted(fp.MODE_NAMES):
                show(b, m, None, False)
                time.sleep(a.cycle)
            show(b, start, None, False)
        elif a.mode is not None or a.image is not None:
            show(b, a.mode, a.image, a.now)
        b.fence()
        print(fp.format_status(b.status()))
    except fp.ProtoError as e:
        print("ERROR:", e)
        return 1
    finally:
        b.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
