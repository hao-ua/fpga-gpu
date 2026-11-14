#!/usr/bin/env python3
"""fbproto_test — board-less checks of fbproto.Board's host rules:
BATCH packing (sub-message headers, padding, the BATCH_MAX split, SubRef keys, the flush before a command B
answers unless b_sub_index, the flush on wait_event / wait_acked / reads), EVENT keying by (seq, index), and the
heartbeat resend rule of _on_ack (resend when an ACK still expects the oldest datagram after 2 ms, once per 4 ms),
and reads whose every answer is lost.
The socket is replaced by a capture; nothing is sent.   python3 host/fb/fbproto_test.py  -> prints OK
"""
import os, struct, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fbproto as fp

sent = []
answer = [None]                               # called with every datagram sent: a board that answers


class FakeSock:
    def sendto(self, data, addr):
        sent.append(bytes(data))
        if answer[0]: answer[0](bytes(data))
    def setsockopt(self, *a): pass
    def bind(self, *a): pass
    def settimeout(self, *a): pass
    def recvfrom(self, n): raise OSError          # ends the receive thread at once
    def close(self): pass


def make_board(**kw):
    fp.socket.socket = lambda *a, **k: FakeSock()
    b = fp.Board("127.0.0.1", **kw)
    b.credit = 1 << 30
    b.running = False                         # no receive / maintenance threads: nothing answers, so the 10 ms stall
    b.rx_thread.join(1); b.mt_thread.join(1)  # timer would resend everything every 10 ms while a test waits
    return b


def hdr(d): return struct.unpack_from("<HBBI", d, 0)


def subs(body):
    """the sub-messages of a BATCH body: [(type, flags, bytes, body)]"""
    out, o = [], 0
    while o < len(body):
        t, f, n = struct.unpack_from("<BBH", body, o)
        out.append((t, f, n, body[o + 4:o + 4 + n])); o += 4 + ((n + 3) & ~3)
    return out


def check(cond, what):
    if not cond: raise SystemExit("FAIL: " + what)


def test_packing_no_fence():
    b = make_board()
    del sent[:]
    with b.batch():
        r0 = b.frame_begin()
        r1 = b.vs_uniforms(0, [(1.0, 2.0, 3.0, 4.0)])
        r2 = b.draw(0, 36, 0, index_handle=5)
    check(len(sent) == 1, "one BATCH datagram, got %d" % len(sent))
    m, v, t, seq = hdr(sent[0])
    check(t == fp.T_BATCH and seq == 1, "BATCH header")
    ss = subs(sent[0][8:])
    check([s[0] for s in ss] == [fp.T_FRAME_BEGIN, fp.T_VS_UNIFORMS, fp.T_DRAW], "sub-message types %r" % [s[0] for s in ss])
    check(ss[1][2] == 20 and len(ss[1][3]) == 20 and ss[2][2] == 20, "sub-message byte counts %r" % [s[2] for s in ss])
    check([r.idx for r in (r0, r1, r2)] == [0, 1, 2] and r0.seq == 1 and r2.dgram is r0.dgram, "SubRefs")
    check(b._event_key(r2) == (1, 2) and b._event_key(7) == (7, 0), "event keys")
    check(b.n_batches == 1 and b.n_batched == 3, "counters")
    # padding: a 3-byte body occupies a word
    del sent[:]
    with b.batch():
        b.send_seq(0x7E, b"abc"); b.send_seq(0x7D, b"")
    ss = subs(sent[0][8:])
    check(len(sent[0]) == 8 + 8 + 4 and ss[0][2] == 3 and ss[0][3] == b"abc" and ss[1] == (0x7D, 0, 0, b""), "padding")
    print("  packing ok")


def test_split_and_flush_rules():
    b = make_board()
    del sent[:]
    with b.batch():
        refs = [b.send_seq(fp.T_WRITE_BUFFER, struct.pack("<HHI", 1, 0, 0) + bytes(600)) for i in range(5)]
    # 5 x (4 + 608) = 3060 bytes -> 1464 holds 2 (1224), so 2 + 2 + 1 datagrams
    check(len(sent) == 3, "BATCH_MAX split: %d datagrams" % len(sent))
    check([len(subs(d[8:])) for d in sent] == [2, 2, 1], "split sizes %r" % [len(subs(d[8:])) for d in sent])
    check([r.seq for r in refs] == [1, 1, 2, 2, 3] and [r.idx for r in refs] == [0, 1, 0, 1, 0], "SubRefs across the split")
    # a command B answers with an EVENT goes alone (b_sub_index False): the collected batch first, then the plain datagram
    del sent[:]
    with b.batch():
        b.frame_begin(); b.draw(0, 3)
        s = b.send_seq(fp.T_PRESENT, struct.pack("<HHI", 0xC001, 0, 7))
        b.flush(0xC001, 0)
    check(len(sent) == 3 and hdr(sent[0])[2] == fp.T_BATCH and hdr(sent[1])[2] == fp.T_PRESENT and hdr(sent[2])[2] == fp.T_BATCH, "B-answered command goes alone")
    check(isinstance(s, int) and s == hdr(sent[1])[3], "plain seq of the PRESENT")
    # with b_sub_index the PRESENT stays inside
    b2 = make_board(b_sub_index=True)
    del sent[:]
    with b2.batch():
        b2.frame_begin()
        s = b2.send_seq(fp.T_PRESENT, struct.pack("<HHI", 0xC001, 0, 7))
        b2.flush(0xC001, 0)
    check(len(sent) == 1 and [x[0] for x in subs(sent[0][8:])] == [fp.T_FRAME_BEGIN, fp.T_PRESENT, fp.T_FLUSH] and s.idx == 1, "b_sub_index keeps B-answered commands inside")
    # too large for a batch
    try:
        with b.batch(): b.send_seq(fp.T_MEM_WRITE, bytes(1470)); raise SystemExit("FAIL: oversize accepted")
    except fp.ProtoError: pass
    # an empty block sends nothing; nesting is refused
    del sent[:]
    with b.batch(): pass
    check(sent == [], "empty batch")
    try:
        with b.batch():
            with b.batch(): pass
        raise SystemExit("FAIL: nested batch accepted")
    except fp.ProtoError: pass
    print("  split / flush rules ok")


def test_events_and_wait():
    b = make_board()
    del sent[:]
    with b.batch():
        b.frame_begin(); r = b.send_seq(0x7C, b"")
    # the EVENTs arrive (simulated) before the wait: keyed (seq, idx); sub-message 0's is an error (bad handle)
    seq = hdr(sent[0])[3]
    with b.cv:
        b.events[(seq, 1)] = (fp.EV_FENCE, 5, 0, 0)
        b.events[(seq, 0)] = (fp.EV_ERROR, 2, 0x56, 0)
    check(b.wait_event(r, timeout=0.1) == (fp.EV_FENCE, 5, 0, 0), "event by SubRef")
    try:
        b.wait_event(seq, timeout=0.1); raise SystemExit("FAIL: error event not raised")
    except fp.ProtoError as e:
        check("bad handle" in str(e), "error text %s" % e)
    # a wait on a sub-message of the OPEN batch sends the batch first
    del sent[:]
    with b.batch():
        r = b.send_seq(0x7C, b"")
        try: b.wait_event(r, timeout=0.05)
        except fp.ProtoError: pass
        check(len(sent) == 1 and r.seq == hdr(sent[0])[3], "wait_event flushes the open batch")
    # the receive path: an EVENT datagram with index 3 in the code word
    seq = 99
    d = struct.pack("<HBBI", fp.MAGIC, fp.VERSION, fp.R_EVENT, seq) + struct.pack("<IIII", (3 << 8) | fp.EV_ERROR, 6, 0x55, 0x1234)
    # feed it through the same decode as _rx_loop (the thread is dead: decode inline)
    magic, ver, typ, s = struct.unpack_from("<HBBI", d, 0)
    codew, a0, a1, a2 = struct.unpack_from("<IIII", d, 8)
    code, idx = codew & 0xFF, (codew >> 8) & 0xFF
    check((code, idx) == (fp.EV_ERROR, 3), "code word decode")
    print("  events ok")


def test_lost_reads():
    """all the answers to all the requests in flight lost: nothing later shows that up, so a read must notice by
    itself, and may only give up on silence. The clock runs 10 ms a look, so the 10 s of the timeout cost 2 s."""
    t = [2000.0]
    def clock():
        t[0] += 0.01
        return t[0]
    fp.time.monotonic = clock
    b = make_board(window=1 << 20)            # nothing ACKs here
    mem = b"".join(struct.pack("<I", i) for i in range(3 << 18))
    n, drop = [0], [None]

    def board(d):
        if hdr(d)[2] != fp.T_MEM_READ: return
        addr, length, tag = struct.unpack_from("<III", d, 12)
        n[0] += 1
        if drop[0](n[0]): return
        for off in range(0, length, fp.RD_CHUNK):
            b.reads[tag][off] = mem[addr + off:addr + min(off + fp.RD_CHUNK, length)]

    def read(how, length):
        n[0], drop[0] = 0, how
        t0 = t[0]
        return b.mem_read(0, length), t[0] - t0

    answer[0] = board
    r, d = read(lambda k: k in (3, 4), len(mem))           # the two a read keeps in flight
    check(r == mem, "read, two requests lost: the data differs")
    check(d < 2.5, "two lost requests took %.2f s" % d)
    r, d = read(lambda k: k == 1, 1000)                    # a read of one request
    check(r == mem[:1000], "read, its only request lost: the data differs")
    check(d < 1.5, "the only request lost took %.2f s" % d)
    r, d = read(lambda k: k % 4, len(mem))                 # slow but alive, past the 10 s of the timeout
    check(r == mem, "slow read: the data differs")
    check(d > 10.5, "the slow read took only %.2f s: it proves nothing about the timeout" % d)
    t0 = t[0]
    try:
        read(lambda k: True, len(mem))                     # nothing answers: it must end at its timeout, and say so
        raise SystemExit("FAIL: nothing answered, and the read returned")
    except fp.ProtoError as e:
        check("no data for 10 s" in str(e), "error text %s" % e)
    check(9.5 < t[0] - t0 < 12.0, "nothing answered: the read gave up after %.2f s" % (t[0] - t0))
    answer[0] = None
    print("  lost reads ok")


def test_heartbeat_rule():
    b = make_board()
    del sent[:]
    t = [1000.0]
    fp.time.monotonic = lambda: t[0]
    b.send_seq(0x7C, b"")                     # seq 1 in flight at t = 1000
    b.send_seq(0x7C, b"")                     # seq 2
    check(b.last_progress == 1000.0, "stall clock starts with the window's first datagram")
    t[0] = 1000.0015
    with b.cv: b._on_ack(1, 0, 1 << 20); b._hb_check(t[0])   # a heartbeat ACK 1.5 ms later still expects seq 1: too early
    check(b.hb_suspect is None and b.resend_req == 0 and b.n_hb_resend == 0, "no suspect before 2 ms")
    t[0] = 1000.0025
    with b.cv: b._on_ack(1, 0, 1 << 20); b._hb_check(t[0])   # 2.5 ms: suspect marked, not resent yet
    check(b.hb_suspect == (1, 1000.0025) and b.resend_req == 0, "suspect after 2 ms without progress")
    t[0] = 1000.004
    with b.cv: b._on_ack(2, 0, 1 << 20); b._hb_check(t[0])   # a late progress ACK 1.5 ms later clears it (the stale-ACK case)
    check(b.hb_suspect is None and b.resend_req == 0 and b.n_hb_resend == 0 and list(b.unacked) == [2], "progress clears the suspect")
    t[0] = 1000.007
    with b.cv: b._on_ack(2, 0, 1 << 20); b._hb_check(t[0])   # 3 ms after the progress: suspect again
    check(b.hb_suspect == (2, 1000.007) and b.resend_req == 0, "second suspect")
    t[0] = 1000.0085
    with b.cv: b._hb_check(t[0])                              # the maintenance loop 1.5 ms later: not yet
    check(b.resend_req == 0, "confirmation waits 2 ms")
    t[0] = 1000.0092
    with b.cv: b._hb_check(t[0])                              # 2.2 ms with no progress since the mark: resend
    check(b.resend_req == 1 and b.n_hb_resend == 1 and b.hb_suspect is None, "resend confirmed after 2 ms")
    b.resend_req = 0; b.last_resend = t[0]
    t[0] = 1000.012
    with b.cv: b._on_ack(2, 0, 1 << 20); b._hb_check(t[0])   # suspect again right away (no progress for 5 ms)
    t[0] = 1000.0125
    with b.cv: b._hb_check(t[0])
    check(b.resend_req == 0 and b.hb_suspect is not None, "one resend per 4 ms")
    t[0] = 1000.0135
    with b.cv: b._hb_check(t[0])                              # 4.3 ms after the last resend, 1.5 ms after the mark: not yet
    check(b.resend_req == 0, "confirmation still waits its 2 ms")
    t[0] = 1000.0145
    with b.cv: b._hb_check(t[0])
    check(b.resend_req == 1 and b.n_hb_resend == 2, "resend again")
    b.resend_req = 0
    t[0] = 1000.015
    with b.cv: b._on_ack(3, 0, 1 << 20)       # progress to seq 3: nothing left, the stall clock restarts
    check(b.resend_req == 0 and b.last_progress == 1000.015 and list(b.unacked) == [], "progress resets")
    b.send_seq(0x7C, b"")                     # a new seq 3 for the stale-ACK and NAK checks
    t[0] = 1000.0151
    with b.cv: b._on_ack(2, 0, 1 << 20)
    check(b.resend_req == 0 and b.hb_suspect is None, "a stale ACK right after a send: no suspect")
    t[0] = 1000.0152
    with b.cv: b._on_ack(3, fp.ACKF_NAK, 1 << 20)   # a NAK (A expects 3, our oldest) still resends at once
    check(b.resend_req == 1, "NAK rule unchanged")
    print("  heartbeat rule ok")


if __name__ == "__main__":
    test_packing_no_fence()
    test_split_and_flush_rules()
    test_events_and_wait()
    test_lost_reads()
    test_heartbeat_rule()
    print("OK fbproto_test")
