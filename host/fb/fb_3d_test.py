#!/usr/bin/env python3
"""fb_3d_test — a 3D frame through both boards on the hardware (the simulation twin is
sim/tb_two_boards3d.sv): the draw of tools/b_ref.py aframe <seed> (B's resources, DRAW_STATE and the FLUSH / FENCE in
tools/gen/ab_<seed>_cmds.hex; A's vertex buffers, program and draw commands in av_<seed>_ddr.hex / as_<seed>_cmds.hex)
is replayed through board A — B's commands are forwarded over the board link, A's geometry runs, the FLUSH makes A
stream the frame's quads to B — and the render target (and the D24S8 depth-stencil image when the draw has one) is
read back from B's DDR through A (MEM_READ board 1) and compared with the model's ab_<seed>_expect.hex bit for bit.

  python3 host/fb/fb_3d_test.py [seed] [--ip 192.168.1.10] [--abort]
    The seed's tools/gen files are the benches' (WSL: bash tools/gen_a_goldens.sh; python3 tools/b_ref.py aframe <seed>);
    seeds 11, 12 (triangles), 13 (points), 15 (a line strip of width 3) are the ones tb_two_boards3d runs.
    --abort: first a FLUSH with a bad image handle: A streams its frame regardless, B consumes and drops it (every FLUSH
    owns one frame of the render stream) and answers ERR_HANDLE; nothing may stay in flight and the real frame must still
    come out bit-exact (tb_two_boards3d +ABORT on the hardware). A's own RENDER_TIMEOUT (3 s,
    ERR_RENDER) only fires when B does not take the stream at all (dead or restarting).
Needs board A with the 3D build (a_geom + a_render) and board B with the 3D path (the D1 format).
"""
import os, sys, struct, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fbproto

GEN = os.path.join(HERE, '..', '..', 'tools', 'gen')
Q_MARK = 0x51                       # as_<seed>_cmds.hex: a datagram starts with the burst {Q_MARK, type, body bytes} (fb_pkg)


def read_words(name):
    with open(os.path.join(GEN, name)) as fh:
        return [int(x, 16) for x in fh.read().split()]


def b_commands(seed):
    """ab_<seed>_cmds.hex: lines "n w0 w1 ... " with w0 = {type[23:16], body bytes}, w1 = the file's own seq; the
    terminating "0" line. Returns [(type, body bytes)] in order."""
    out = []
    with open(os.path.join(GEN, f"ab_{seed}_cmds.hex")) as fh:
        for ln in fh:
            w = [int(x, 16) for x in ln.split()]
            if not w or w[0] == 0: break
            n, words = w[0], w[1:]
            assert len(words) == n, f"ab_{seed}_cmds.hex: a line of {len(words)} words claims {n}"
            out.append(((words[0] >> 16) & 0xFF, b''.join(struct.pack("<I", x) for x in words[2:])))
    return out


def a_commands(seed):
    """as_<seed>_cmds.hex: one 256-bit ingress-queue burst per line; a datagram = a header burst whose low word is
    {Q_MARK, type, body bytes} (then its seq, unused here) and the body in 32-byte bursts, little-endian — the reading
    of tb_two_boards3d / tb_board_a. Returns [(type, body bytes)]."""
    out = []
    ws = read_words(f"as_{seed}_cmds.hex")
    i = 0
    while i < len(ws):
        w = ws[i]; i += 1
        if (w >> 24) & 0xFF != Q_MARK: continue
        ty, blen = (w >> 16) & 0xFF, w & 0xFFFF
        body = b''
        for _ in range((blen + 31) // 32):
            body += ws[i].to_bytes(32, 'little'); i += 1
        out.append((ty, body[:blen]))
    return out


def a_memory(seed):
    """av_<seed>_ddr.hex: "burst_index 256-bit data" lines -> {word address: 32-bit word} (A's DDR image)"""
    img = {}
    with open(os.path.join(GEN, f"av_{seed}_ddr.hex")) as fh:
        for ln in fh:
            p = ln.split()
            if len(p) != 2: continue
            bi, bd = int(p[0], 16), int(p[1], 16)
            for k in range(8): img[bi * 8 + k] = (bd >> (32 * k)) & 0xFFFFFFFF
    return img


def main():
    a = sys.argv[1:]
    ip = a[a.index('--ip') + 1] if '--ip' in a else fbproto.BOARD_IP
    seed = int(a[0]) if a and not a[0].startswith('--') else 11
    bcmds = b_commands(seed)
    acmds = a_commands(seed)
    img = a_memory(seed)
    pv = read_words(f"ab_{seed}_params.hex")
    base, bpl, W, H = pv[0], pv[1], pv[2], pv[3]
    fmt, d16, ds, ds_base, ds_bpl = pv[16], pv[17], pv[18], pv[19], pv[20]
    bpp = 4 if fmt == 0 else 2
    print(f"seed {seed}: {len(bcmds)} B commands, {len(acmds)} A commands, target {W} x {H} "
          f"{['RGBA8', 'RGBA4444', 'RGBA5551', 'RGB565'][fmt]}, depth {16 if d16 else 24} bits"
          f"{', depth-stencil image' if ds else ''}")

    bd = fbproto.Board(ip)
    bd.hello()
    st = bd.status()
    print(f"board A build {st['build']}, link flags {st['link_flags']:#x}, B flags {st['b_flags']:#x}")
    # ---- B's resources and draw state (the FLUSH and the FENCE go after A's draw) ----
    flush = fence = None
    t0 = time.time()
    for ty, body in bcmds:
        if ty == fbproto.T_FLUSH: flush = body
        elif ty == fbproto.T_FENCE: fence = body
        else: bd.send_seq(ty, body)
    bd.wait_acked(30.0)
    t1 = time.time()
    # ---- A's draw: the buffers from the DDR image at each CREATE_BUFFER, then the command itself ----
    fence_seqs = []
    reloc = int(a[a.index('--reloc') + 1], 0) if '--reloc' in a else 0    # every A buffer moved by this many bytes (diagnosis)
    for ty, body in acmds:
        if ty == fbproto.T_CREATE_BUFFER and reloc:
            h, u, b0, s0 = struct.unpack("<HHII", body[:12]); body = struct.pack("<HHII", h, u, b0 + reloc, s0) + body[12:]
        seq = bd.send_seq(ty, body)
        if ty == fbproto.T_FENCE: fence_seqs.append(seq)
        if ty == fbproto.T_CREATE_BUFFER:
            handle, _, base_b, size_b = struct.unpack("<HHII", body[:12]); base_b -= reloc
            base_w, size_w = base_b // 4, size_b // 4
            used = max((k - base_w + 1 for k in img if base_w <= k < base_w + size_w), default=0)
            size_w = (used + 7) // 8 * 8
            data = b''.join(struct.pack("<I", img.get(base_w + k, 0)) for k in range(size_w))
            bd.write_buffer(handle, 0, data)
            bd.wait_acked(30.0)
            if '--slow' in a: time.sleep(0.05)               # the buffer's DDR writes long done before the next command
    for seq in fence_seqs:
        code, a0, _, _ = bd.wait_event(seq, 30.0)
        if code != fbproto.EV_FENCE: raise fbproto.ProtoError(f"A's fence: event {code:#x}")
    t2 = time.time()
    # ---- --abort: a FLUSH B rejects, A's frame given up after its timeout, the credit re-synced ----
    if '--abort' in a:
        bad = bytearray(flush); bad[0:2] = b'\x00\xcc'            # body word 0 = {flags, handle}: handle 0xCC00 (no such image)
        sbad = bd.send_seq(fbproto.T_FLUSH, bytes(bad))
        ta = time.time()
        try:
            bd.wait_event(sbad, 5.0); print("  abort: no error event for the bad FLUSH"); sys.exit(1)
        except fbproto.ProtoError as e:
            print(f"  abort: first event {e}")
        time.sleep(0.5)
        errs0 = bd.take_errors()
        codes = sorted(set(e[1] for e in errs0 if e[0] == sbad))    # (seq, code, type, detail, sub-message index)
        st = bd.status()
        print(f"  abort: error codes for the bad FLUSH after {time.time() - ta:.2f} s: {codes} (2 = bad handle from B; 9 would be A's timeout); "
              f"A render beats in flight {st['rs_pkts_inflight'] >> 16}, B cp busy {(st['b_flags'] >> 12) & 1}")
        if codes != [2] or (st['rs_pkts_inflight'] >> 16) != 0:
            print("  abort: the rejected FLUSH did not clean up"); sys.exit(1)
    # ---- FLUSH (A forwards it and streams the frame to B), FENCE back from B ----
    for rep in range(2 if '--twice' in a else 1):           # --twice: the same frame flushed again (A's bank rendered twice)
        if rep: st = bd.status(); print(f"  after the first FLUSH: records {st['geom_busy_tris'] & 0xFFFFF} quads {st['render_quads']}")
        bd.send_seq(fbproto.T_FLUSH, flush)
        seq = bd.send_seq(fbproto.T_FENCE, fence)
        code, a0, _, _ = bd.wait_event(seq, 30.0)
        if code != fbproto.EV_FENCE or a0 != 0x5EED: raise fbproto.ProtoError(f"frame fence: event {code:#x} {a0:#x}")
    t3 = time.time()
    errs = bd.take_errors()
    for e in errs: print("  error event:", e)
    # ---- the target (and the depth-stencil image) from B's DDR, through A ----
    got = bd.mem_read(base * 4, H * bpl * 32, board=1)
    exp = read_words(f"ab_{seed}_expect.hex")
    bad = 0
    for y in range(H):
        for x in range(W):
            off = y * bpl * 32 + x * bpp
            g = int.from_bytes(got[off:off + bpp], 'little')
            if g != exp[y * W + x]:
                bad += 1
                if bad < 10: print(f"  pixel ({x}, {y}): {g:0{2 * bpp}x}, expected {exp[y * W + x]:0{2 * bpp}x}")
    print(f"target: {W * H} pixels, {bad} differ")
    bad_ds = 0
    if ds:
        gds = bd.mem_read(ds_base * 4, H * ds_bpl * 32, board=1)
        eds = read_words(f"ab_{seed}_expect_ds.hex")
        for y in range(H):
            for x in range(W):
                off = y * ds_bpl * 32 + x * 4
                g = int.from_bytes(gds[off:off + 4], 'little')
                if g != eds[y * W + x]:
                    bad_ds += 1
                    if bad_ds < 10: print(f"  depth-stencil ({x}, {y}): {g:08x}, expected {eds[y * W + x]:08x}")
        print(f"depth-stencil image: {bad_ds} words differ")
    if '--dump' in a:                                        # A's triangle records of both frame banks against the model's
        exp_rec = []
        with open(os.path.join(GEN, f"as_{seed}_rec.hex")) as fh:
            toks = fh.read().split(); i = 0
            while i + 4 <= len(toks):
                rid, lo, hi, n = (int(t, 16) for t in toks[i:i + 4]); i += 4
                exp_rec.append((rid, lo, hi, [int(t, 16) for t in toks[i:i + n]])); i += n
        for bank in (0, 1):
            rec_base = 0x4000000 + bank * 0x8000000           # bytes: REC_BASE word 0x1000000, REC_BANK_W 0x2000000 words
            nrec = max(r[0] for r in exp_rec) + 1
            got = bd.mem_read(rec_base, nrec * 512, board=0)
            rbad = 0; first = []                              # (the other bank holds another frame's records: expected to differ)
            for rid, lo, hi, words in exp_rec:
                for w, e in enumerate(words):
                    g = int.from_bytes(got[rid * 512 + 4 * w: rid * 512 + 4 * w + 4], 'little')
                    if g != e:
                        rbad += 1
                        if len(first) < 6: first.append(f"rec {rid} w{w}: {g:08x} exp {e:08x}")
            print(f"  records bank {bank}: {len(exp_rec)} records, {rbad} words differ" + (": " + "; ".join(first) if first else ""))
            if rbad and '--dumpfull' in a:
                for rid, lo, hi, words in exp_rec:
                    gw = [int.from_bytes(got[rid * 512 + 4 * w: rid * 512 + 4 * w + 4], 'little') for w in range(len(words))]
                    if gw != words:
                        print(f"    rec {rid} got: " + " ".join(f"{g:08x}" for g in gw))
                        print(f"    rec {rid} exp: " + " ".join(f"{e:08x}" for e in words))
        for bank in (0, 1):                                   # the first bin-list chunks of each pool: {next, tid} entries
            pool = 0x14000000 + bank * 0x2000000              # bytes: BINLIST_BASE word 0x5000000, POOL_W 0x800000 words
            got = bd.mem_read(pool, 32 * 8, board=0)
            ents = [int.from_bytes(got[8 * k: 8 * k + 8], 'little') for k in range(32)]
            print(f"  pool bank {bank} entries 0..31 (next:tid): " + " ".join(f"{(e >> 32) & 0x7FFFFF}:{e & 0xFFFFF}" for e in ents))
    st = bd.status()
    rs = st['rs_pkts_inflight']
    print(f"timing: B commands {1000 * (t1 - t0):.1f} ms, A draw + geometry {1000 * (t2 - t1):.1f} ms, "
          f"frame (FLUSH to FENCE) {1000 * (t3 - t2):.1f} ms; A: cmds {st['cmds']} errs {st['errs']} fwd {st['fwd']} "
          f"records {st['geom_busy_tris'] & 0xFFFFF} quads {st['render_quads']} RSTREAM packets {rs & 0xFFFF} "
          f"beats in flight {rs >> 16}")
    if rs >> 16: print("  render beats still in flight"); errs.append(('render', 'in flight', rs >> 16, 0))
    ok = bad == 0 and bad_ds == 0 and not errs
    print("fb_3d_test:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == '__main__': main()
