#!/usr/bin/env python3
"""fb_hang — what board A can say about a board B that stopped answering, read-only.
Run it before any reset, with B's own view from `tools/b_jtag.tcl <serial> status`: cp_busy, render_busy.

  python3 host/fb/fb_hang.py [--ip 192.168.1.10]

Render beats in flight 0 with render_busy 1: b_render waits on a stream nobody will send (the lost-abort hypothesis). Beats
in flight > 0: B stopped taking the stream, look behind the sequencer. Unconsumed CMD words: B's command processor stopped
taking commands, its FLUSH may never have started. Never while a game or test runs: board A answers whoever spoke last.
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fbproto as fp


def main():
    ip = sys.argv[sys.argv.index('--ip') + 1] if '--ip' in sys.argv else "192.168.1.10"
    bd = fp.Board(ip)
    bd.hello()
    st = bd.status()
    print(f"render beats in flight to B {st['rs_pkts_inflight'] >> 16}, RSTREAM packets {st['rs_pkts_inflight'] & 0xFFFF}")
    print(f"CMD words: A sent {st['cr_sent']}, B reported {st['cr_consumed']}, B taken {st['b_cb_taken']}, unconsumed {(st['cr_sent'] - st['cr_consumed']) & 0xFFFFFFFF}")
    print(f"geometry busy / records {st['geom_busy_tris']:08x}, geom_err {st['geom_err']:08x}, last frame's quads {st['render_quads']}")
    print(f"A errors {st['errs']}; " + fp.b_frame_split(st))
    print("link", fp.decode_link_flags(st['link_flags']))
    return 0


if __name__ == '__main__': sys.exit(main())
