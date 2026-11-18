#!/usr/bin/env python3
"""g2_relay.py: carry the boards' UDP protocol between a remote Linux host and the PC that is cabled to board A.

The boards answer only the most recent sender on their own network. A Linux machine elsewhere, reached over a VPN,
talks to them through two relay ends. Each forwards datagrams unchanged, so the remote side speaks the ordinary
protocol (host/fb/fbproto.py, the fpgagpu kernel module):

  remote:  python3 g2_relay.py remote --listen <remote-address>:5100 --allow <pc-address> --local 127.0.0.1:5000
           programs on the remote host send to 127.0.0.1:5000 (the kernel module: ip=127.0.0.1)
  pc:      python3 g2_relay.py pc --remote <remote-address>:5100 --board 192.168.1.10:5000

The PC end starts the flow: it sends a keepalive to the remote end every 2 s. So no inbound firewall rule is needed on
the PC, and NAT on the way (WSL, a router) keeps the flow open. Keepalives start with KA_MAGIC and are never forwarded.
The remote end accepts that flow only from --allow, and forwards what local programs send only once a PC end is
known. The latest local sender gets the board's replies, as the boards themselves do.

Between the two ends every datagram travels in pieces of at most PIECE bytes, each with an 8-byte header {PIECE_MAGIC,
message id, index, count}, and is put together again on the other side. The protocol's datagrams are up to 1472 bytes
(WRITE_* data, READ_DATA chunks), and a VPN path (an MTU of 1420, say) would fragment them; IP fragments are often
dropped on the way, which loses every large datagram. Pieces fit any common path unfragmented. A message whose pieces do
not all arrive within REASSEMBLY_S is dropped; the protocol resends it.

Counters go to stderr every 10 s (or at once with SIGUSR1). Only the Python standard library is used.
"""
import argparse
import select
import signal
import socket
import struct
import sys
import time

KA_MAGIC = b"G2RLY-KA"          # never a protocol datagram: those start with the little-endian magic 0x4246 ("FB")
KA_PERIOD = 2.0
STATS_PERIOD = 10.0
PIECE_MAGIC = b"G2"             # pieces: b"G2" + u16 message id + u8 index + u8 count + u16 bytes + data
PIECE = 1200                    # data bytes per piece: with the headers far below a 1420-byte tunnel MTU
REASSEMBLY_S = 1.0


class Pieces:
    """cut datagrams into pieces and put them together again"""

    def __init__(self, st):
        self.st, self.next_id, self.partial = st, 0, {}

    def cut(self, d):
        self.next_id = (self.next_id + 1) & 0xFFFF
        count = max(1, (len(d) + PIECE - 1) // PIECE)
        return [PIECE_MAGIC + struct.pack("<HBBH", self.next_id, i, count, len(d[i * PIECE:(i + 1) * PIECE]))
                + d[i * PIECE:(i + 1) * PIECE] for i in range(count)]

    def join(self, p, src):
        """the whole datagram once its last piece is in, else None"""
        if len(p) < 8 or not p.startswith(PIECE_MAGIC):
            self.st.add("bad_piece")
            return None
        mid, idx, count, n = struct.unpack_from("<HBBH", p, 2)
        data = p[8:8 + n]
        if count == 1:
            return data
        now = time.monotonic()
        for k in [k for k, v in self.partial.items() if now - v[0] > REASSEMBLY_S]:
            del self.partial[k]
            self.st.add("dropped_partial")
        ent = self.partial.setdefault((src, mid), [now, count, {}])
        ent[2][idx] = data
        if len(ent[2]) < count:
            return None
        del self.partial[(src, mid)]
        return b"".join(ent[2][i] for i in range(count))


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
    pieces = Pieces(st)
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
                d = pieces.join(d, "remote")
                if d is not None:
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
                for p in pieces.cut(d):
                    up.send(p)
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
    pieces = Pieces(st)
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
            else:
                d = pieces.join(d, src)
                if d is None:
                    pass
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
                for p in pieces.cut(d):
                    wan.sendto(p, pc)
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
