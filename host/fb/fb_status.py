#!/usr/bin/env python3
"""fb_status — print board status (STATUS datagram), once or repeatedly.
   python3 fb_status.py [--ip 192.168.1.10] [--loop SECONDS] [--raw]"""
import argparse
import time

import fbproto


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default=fbproto.BOARD_IP)
    ap.add_argument("--loop", type=float, default=0.0, help="repeat every N seconds")
    ap.add_argument("--raw", action="store_true", help="print every status word")
    a = ap.parse_args()
    b = fbproto.Board(a.ip)
    try:
        while True:
            st = b.status()
            if a.raw:
                print(" ".join("%s=%d" % (k, v) for k, v in st.items()))
            else:
                print(fbproto.format_status(st))
            if a.loop <= 0:
                break
            time.sleep(a.loop)
    finally:
        b.close()


if __name__ == "__main__":
    main()
