#!/usr/bin/env python3
"""g2_relay.py: carry the boards' UDP protocol between a remote Linux host and the PC that is cabled to board A.

The boards answer only the most recent sender on their own network. A Linux machine elsewhere, reached over a VPN,
talks to them through two relay ends. Each forwards datagrams unchanged, so the remote side speaks the ordinary
protocol (host/fb/fbproto.py, the fpgagpu kernel module):

  remote:  python3 g2_relay.py remote --listen 192.168.53.1:5100 --allow 192.168.73.30 --local 127.0.0.1:5000
           programs on the remote host send to 127.0.0.1:5000 (the kernel module: ip=127.0.0.1)
  pc:      python3 g2_relay.py pc --remote 192.168.53.1:5100 --board 192.168.1.10:5000

The PC end starts the flow: it sends a keepalive to the remote end every 2 s. So no inbound firewall rule is needed on
the PC, and NAT on the way (WSL, a router) keeps the flow open. Keepalives start with KA_MAGIC and are never forwarded.
The remote end accepts that flow only from --allow, and forwards what local programs send only once a PC end is
known. The latest local sender gets the board's replies, as the boards themselves do.

Counters go to stderr every 10 s (or at once with SIGUSR1). Only the Python standard library is used.
"""
import argparse
import select
import signal
import socket
import sys
import time

KA_MAGIC = b"G2RLY-KA"          # never a protocol datagram: those start with the little-endian magic 0x4246 ("FB")
KA_PERIOD = 2.0
STATS_PERIOD = 10.0


def addr(text):
    host, _, port = text.rpartition(":")
    return (host or "0.0.0.0", int(port))


class Stats:
    def __init__(self, name):
        self.name, self.c, self.t = name, {}, time.monotonic()

    def add(self, key, n=1):
        self.c[key] = self.c.get(key, 0) + n

    def show(self, force=False):
        now = time.monotonic()
        if force or now - self.t >= STATS_PERIOD:
            self.t = now
            sys.stderr.write("%s %s %s\n" % (time.strftime("%H:%M:%S"), self.name,
                                             " ".join("%s=%d" % kv for kv in sorted(self.c.items()))))
            sys.stderr.flush()


def run_pc(a):
    """The PC end: one socket towards the remote end, one towards the board."""
    up = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    up.connect(addr(a.remote))
    board = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    board.connect(addr(a.board))
    st = Stats("pc")
    signal.signal(signal.SIGUSR1, lambda *_: st.show(True))
    last_ka = 0.0
    while True:
        now = time.monotonic()
        if now - last_ka >= KA_PERIOD:
            try:
                up.send(KA_MAGIC)
                st.add("ka_sent")
            except OSError:
                st.add("ka_err")
            last_ka = now
        r, _, _ = select.select([up, board], [], [], 0.5)
        if up in r:
            try:
                d = up.recv(2048)
            except OSError:                              # ICMP unreachable while the remote end is down
                st.add("up_err")
                d = b""
            if d and not d.startswith(KA_MAGIC):
                board.send(d)
                st.add("to_board")
                st.add("to_board_bytes", len(d))
        if board in r:
            try:
                d = board.recv(2048)
            except OSError:
                st.add("board_err")
                d = b""
            if d:
                up.send(d)
                st.add("from_board")
                st.add("from_board_bytes", len(d))
        st.show()


def run_remote(a):
    """The remote end: the PC end's flow on --listen, local programs on --local."""
    wan = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    wan.bind(addr(a.listen))
    loc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    loc.bind(addr(a.local))
    allow = a.allow
    pc = None                                            # the PC end's (address, port), from its keepalives
    client = None                                        # the latest local sender
    st = Stats("remote")
    signal.signal(signal.SIGUSR1, lambda *_: st.show(True))
    while True:
        r, _, _ = select.select([wan, loc], [], [], 0.5)
        if wan in r:
            d, src = wan.recvfrom(2048)
            if allow and src[0] != allow:
                st.add("wan_refused")
            elif d.startswith(KA_MAGIC):
                if pc != src:
                    sys.stderr.write("%s remote: PC end at %s:%d\n" % (time.strftime("%H:%M:%S"), src[0], src[1]))
                pc = src
                st.add("ka_recv")
            elif client:
                loc.sendto(d, client)
                st.add("to_local")
                st.add("to_local_bytes", len(d))
            else:
                st.add("no_local_client")
        if loc in r:
            d, src = loc.recvfrom(2048)
            client = src
            if pc:
                wan.sendto(d, pc)
                st.add("to_pc")
                st.add("to_pc_bytes", len(d))
            else:
                st.add("no_pc_end")
        st.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="end", required=True)
    p = sp.add_parser("pc", help="the end on the PC cabled to board A")
    p.add_argument("--remote", required=True, help="the remote end's address:port (its VPN address)")
    p.add_argument("--board", default="192.168.1.10:5000", help="board A's address:port")
    r = sp.add_parser("remote", help="the end on the remote Linux host")
    r.add_argument("--listen", required=True, help="address:port for the PC end's flow (the VPN address only)")
    r.add_argument("--allow", default="", help="accept the PC end's flow only from this IPv4 address")
    r.add_argument("--local", default="127.0.0.1:5000", help="address:port local programs send to")
    a = ap.parse_args()
    try:
        (run_pc if a.end == "pc" else run_remote)(a)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
