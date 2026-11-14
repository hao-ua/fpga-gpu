#!/usr/bin/env python3
"""fb_geom_test — board A's geometry stage on the hardware: the draw of tools/a_setup_ref.py
<seed> is uploaded (CREATE / WRITE_BUFFER: the two vertex buffers, the index buffer, the vertex program), the commands
FRAME_BEGIN, VS_PROGRAM, VS_UNIFORMS, 16 x VERTEX_ATTR, VS_STATE, DRAW are sent, a FENCE waits for the geometry, and the
triangle records are read back from A's GPU-internal memory (MEM_READ at 64 MB + id * 512) and compared word for word
with the model's as_<seed>_rec.hex. Needs board A with the 3D geometry build (board_a_core with a_geom).

  python3 host/fb/fb_geom_test.py [seed] [--ip 192.168.1.10] [a_setup_ref.py options ...]
"""
import os, sys, struct, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto
import a_setup_ref, a_vtx_ref

REC_BASE_BYTES = 64 << 20
REC_BANK_BYTES = 128 << 20          # bank 1's records follow bank 0's (a_geom REC_BANK_W = 2^25 words)
SCRUB = 0xFFFFFFFF                  # never a record header: marks a slot nobody wrote in this run


def main():
    a = sys.argv[1:]
    ip = a[a.index('--ip') + 1] if '--ip' in a else "192.168.1.10"
    seed = int(a[0]) if a and not a[0].startswith('--') else 1
    # the model: builds the draw (writes the av_ / as_ files) — the same code path as the benches
    sys.argv = ['a_setup_ref.py'] + a
    a_setup_ref.main()
    # parse_args returns (seed, mode, nv, nidx, indexed, u8, first, ioff, types, far): far must not land in build_draw's `write`
    s_, mode, nv, nidx, indexed, u8, first, ioff, types, far = a_vtx_ref.parse_args(a)
    d = a_vtx_ref.build_draw(s_, mode, nv, nidx, indexed, u8, first, ioff, types, write=False, proj='--noproj' not in a, far=far)
    mem = d['mem']; b = d['bases']
    gen = os.path.join(HERE, '..', '..', 'tools', 'gen')
    recs = [ln.split() for ln in open(os.path.join(gen, f"as_{seed}_rec.hex"))]
    st = [int(x, 16) for x in open(os.path.join(gen, f"as_{seed}_state.mem")).read().split()]
    stv = st[0]
    vp = [(stv >> (24 * k)) & 0xFFFFFF for k in range(4)]
    cull, cw, did = (stv >> 96) & 3, (stv >> 98) & 1, (stv >> 99) & 0x1FFF
    wid, hgt, fsmask, lw = (stv >> 112) & 0xFFF, (stv >> 124) & 0xFFF, (stv >> 136) & 0xFF, (stv >> 144) & 0xFF
    zs, zo = (stv >> 152) & 0xFFFFFF, (stv >> 176) & 0xFFFFFF                 # the depth range constants (FP24 bits)
    gb = [(stv >> (200 + 24 * k)) & 0xFFFFFF for k in range(4)]                 # the guard band's NDC bounds (the model's exact values)

    def region(base, nwords):
        return b''.join(struct.pack("<I", mem.get(base + w, 0)) for w in range(nwords))
    bd = fbproto.Board(ip)
    bd.hello()
    # FRAME BANKING (a_geom): each FRAME_BEGIN takes the other of two record banks, so the frame's records
    # may land at 64 MB or at 64 MB + 128 MB. Scrub the slots this draw can write in BOTH banks first; after the fence the
    # bank whose first record is no longer the scrub marker is the one the frame used (the other holds a previous run's).
    nslots = max(int(r[0], 16) for r in recs) + 1
    for bank in (0, 1):
        bd.mem_write(REC_BASE_BYTES + bank * REC_BANK_BYTES, struct.pack("<I", SCRUB) * (nslots * 128))
    bd.frame_begin()
    for h, base, size in ((1, b['a'], 0x2000), (2, b['b'], 0x1000), (3, b['idx'], 0x1000), (4, b['prog'], 0x1000)):
        bd.create_buffer(h, 4 * base, 4 * size)
        bd.write_buffer(h, 0, region(base, size))
    bd.vs_program(4, 0, len(d['words']))
    bd.vs_uniforms(0, [tuple(d['params'][i]) for i in range(4)])
    for i, (en, base, size, ty, nm, stride, off) in enumerate(d['desc']):       # bytes; client formats (--types)
        bd.vertex_attr(i, en, size, 0 if not en else (1 if base == 4 * b['a'] else 2), stride, off, tuple(d['consts'][i]), type=ty, normalized=nm)
    bd.vs_state(vp, cull, cw, fsmask, wid, hgt, did, lw, (zs, zo), guard=gb)
    t0 = time.time()
    bd.draw(d['mode'], d['nidx'], d['first'], 3 if d['indexed'] else None, d['ioff_bytes'], d['u8'])
    bd.fence()
    dt = time.time() - t0
    tid0 = int(recs[0][0], 16)
    written = [struct.unpack("<I", bd.mem_read(REC_BASE_BYTES + bank * REC_BANK_BYTES + tid0 * 512, 4))[0] != SCRUB for bank in (0, 1)]
    if written != [True, False] and written != [False, True]:
        print(f"seed {seed}: record {tid0} written in banks {[b for b in (0, 1) if written[b]]} (expected exactly one)")
        sys.exit(1)
    base = REC_BASE_BYTES + written.index(True) * REC_BANK_BYTES
    bad = 0
    for r in recs:
        tid, n = int(r[0], 16), int(r[3], 16)
        exp = [int(x, 16) for x in r[4:4 + n]]
        got = bd.mem_read(base + tid * 512, 4 * n)
        gw = list(struct.unpack("<%dI" % n, got))
        if gw != exp:
            bad += 1
            if bad <= 4:
                k = next(i for i in range(n) if gw[i] != exp[i])
                print(f"  record {tid} word {k}: got {gw[k]:08x} expected {exp[k]:08x}")
    print(f"seed {seed}: {len(recs)} records in bank {written.index(True)}, {bad} differ; draw + fence {dt * 1000:.1f} ms")
    sys.exit(1 if bad else 0)


if __name__ == '__main__': main()
