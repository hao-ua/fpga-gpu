#!/usr/bin/env python3
"""fb_skip_test.py — band skipping on the boards: STATUS (A layout 13, B flags bit 13), then the same draws without a clear
into two images holding the same random pixels, one FLUSH with flags bit 10 and one without, both read back: they must be
equal, and every row outside the drawn strips must hold the original bytes. Several strips (1 band, 3 bands at unaligned
rows, two strips far apart, nothing at all), targets 800 x 600 and 1280 x 720 (4-line bands above 1024 wide).
Run from host/fb:  python3 fb_skip_test.py [--ip 192.168.1.10]"""
import os, random, struct, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto as fp
import fb_bench as fbb
from fb_shader import words64_bytes, f24, H_VBO, H_IBO, H_VSPROG, A_VBO, A_IBO, A_VSPROG, H_FSPROG, B_FSPROG, DRAW_ID, err_text

H_X, H_Y = 0xC0E0, 0xC0E1
B_X, B_Y = 0x2800_0000, 0x2A00_0000


def strips_mesh(H, strips):
    """full-width quads over rows [y0, y1) (pixel rows, top = row 0 of the image as the walk counts them)"""
    v, idx = [], []
    for y0, y1 in strips:
        a, b = 1.0 - 2.0 * y0 / H, 1.0 - 2.0 * y1 / H
        k = len(v)
        v += [(-0.9, b, 0.0), (0.9, b, 0.0), (0.9, a, 0.0), (-0.9, a, 0.0)]
        idx += [k, k + 1, k + 2, k, k + 2, k + 3]
    return v, idx


def main():
    a = sys.argv[1:]
    ip = a[a.index('--ip') + 1] if '--ip' in a else fp.BOARD_IP
    bd = fp.Board(ip); bd.hello()
    st = bd.status()
    bfl = fp.decode_b_flags(st['b_flags'])
    print(f"board A build {st['build']}, STATUS layout {st['layout']}, B flags band_next {bfl.get('band_next')}, "
          f"bands left out so far {st.get('a_bands_skipped')}")
    if st['layout'] < 13 or not bfl.get('band_next'):
        print("FAIL: the boards do not report band skipping"); return 1
    noops = fp.stage_noops(st)
    fbb.NOOPS = noops
    rv, rf, wv, wf = fbb.compile_pair(fbb.VS_FILL, fbb.FS_VARY)
    bad = 0
    rng = random.Random(7)
    for mode, (W, H) in ((1, (800, 600)), (2, (1280, 720))):
        bd.set_mode(mode)
        nl = 8 if W <= 1024 else 4
        for name, strips in (("one band", [(8 * 10, 8 * 11)]), ("unaligned 3 bands", [(nl * 20 + 3, nl * 22 + 5)]),
                             ("two strips", [(5, 9), (H - 30, H - 11)]), ("nothing drawn", [])):
            orig = bytes(rng.getrandbits(8) for _ in range(W * H * 4))
            for h_, base in ((H_X, B_X), (H_Y, B_Y)):
                bd.create_image(h_, base, W, H, fp.FMT_RGBA8, origin=1, usage=fp.USE_RT | fp.USE_TRANSFER)
                bd.write_image(h_, orig, fp.TF_RAW, 0, 0, W, H)
            v, idx = strips_mesh(H, strips) if strips else strips_mesh(H, [(0, 1)])
            bd.create_buffer(H_VBO, A_VBO, 1 << 20); bd.write_buffer(H_VBO, 0, b''.join(struct.pack("<I", f24(c)) for p in v for c in p))
            bd.create_buffer(H_IBO, A_IBO, 1 << 16); bd.write_buffer(H_IBO, 0, struct.pack("<%dH" % len(idx), *idx))
            bd.create_buffer(H_VSPROG, A_VSPROG, 4096); bd.write_buffer(H_VSPROG, 0, words64_bytes(wv))
            bd.vs_program(H_VSPROG, 0, len(wv))
            for i in range(16):
                if i == 0: bd.vertex_attr(0, True, 3, H_VBO, 12, 0, type=fp.AT_FP24)
                else: bd.vertex_attr(i, False, 4, 0, 0, 0, (0.0, 0.0, 0.0, 1.0))
            mask = 0
            for _, (slot, _ty) in rf['varyings'].items(): mask |= 1 << slot
            bd.create_buffer(H_FSPROG, B_FSPROG, 65536); bd.write_buffer(H_FSPROG, 0, words64_bytes(wf))
            ufs = fbb.uniforms(rf, {})
            h = [0] * 23; h[0] = 0xF << 16; h[11] = len(wf) << 16 | H_FSPROG; h[13] = len(ufs)
            bd.vs_state((W * 8.0, W * 8.0, H * 8.0, H * 8.0), cull=0, front_cw=0, fsmask=mask or 1, width=W, height=H,
                        draw_id=DRAW_ID, zc=fp.ZC_LAZY)
            bd.draw_state(DRAW_ID, h, ufs)
            uvs = fbb.uniforms(rv, {'uS': [(0.7, 0.3, 0.0, 0.0)]})
            bd.fence(timeout=20.0)
            st0 = bd.status()
            t = {}
            for h_, fl in ((H_X, fp.FL_SKIP_BANDS), (H_Y, 0)):
                t0 = time.monotonic()
                with bd.batch():
                    bd.frame_begin(); bd.vs_uniforms(0, uvs)
                    if strips: bd.draw(0, len(idx), 0, index_handle=H_IBO)
                    bd.flush(h_, fl, (0, 0, 0, 0), clear_depth=0xFFFFFF, scissor=(0, 0, W, H))
                bd.fence(timeout=20.0); t[h_] = (time.monotonic() - t0) * 1e3
            st1 = bd.status()
            for e in bd.take_errors(): print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4]))); bad += 1
            gx = bd.mem_read(B_X, W * H * 4, board=1); gy = bd.mem_read(B_Y, W * H * 4, board=1)
            skipped = (st1['a_bands_skipped'] - st0['a_bands_skipped']) & 0xFFFFFFFF
            nb = (H + nl - 1) // nl
            drawn = set()
            for y0, y1 in strips:                      # (the image's rows run either way up: GL origin)
                for r in range(y0, y1): drawn.add(r); drawn.add(H - 1 - r)
            rows_bad = [r for r in range(H) if r not in drawn and gx[r * W * 4:(r + 1) * W * 4] != orig[r * W * 4:(r + 1) * W * 4]]
            same = gx == gy
            ok = same and not rows_bad
            bad += not ok
            print(f"{'ok' if ok else 'FAIL'}: {W} x {H} {name}: skip == no skip {same}, rows outside the strips changed "
                  f"{len(rows_bad)}, bands left out {skipped} of {nb}, FLUSH->FENCE {t[H_X]:.2f} ms with skip, {t[H_Y]:.2f} ms without")
    print("BS_EXACT", "PASS" if bad == 0 else f"FAIL ({bad})")
    return 0 if bad == 0 else 1


if __name__ == '__main__': sys.exit(main())
