#!/usr/bin/env python3
"""fb_texperf — texture throughput of the two boards (board B's tmu24 + tex_cache behind the fragment core): a
full-screen quad whose fragment program samples 1, 2 or 4 textures (or a cube map) per pixel, with the texture size,
format, filter, mip mode and minification chosen per run; prints fps, B's per-frame split, cycles per quad and the
derived sample / texel rates (fb_shader's measurement loop).

  python3 host/fb/fb_texperf.py [--ip 192.168.1.10] [--mode M] [--secs S] --case NAME [--check] [--list]
  python3 host/fb/fb_texperf.py --all [--mode M]           # every case, one after the other (RESULT lines)
  --forward: compile with forward=True, as host/win/shc always does (the tunnel cases always are)

Cases (--list prints them): samples per pixel, texture size, storage format, filter {nearest, bilinear, bilinear+mip
nearest, trilinear}, repeats over the screen height (minification = repeats x size / H texels per pixel), cube map.
--check: frame 0 (time 0) is read back from B's DDR and ~40 pixels are compared with the host's sampling of the same
procedural texture (exact where every bilinear tap lies inside one 1/32-block of the texture; the level tint of mip
cases is loose), so a wrong sampler setup cannot be measured as "fast".
Textures: procedural, 32 x 32 colour blocks over the normalised (u, v) at every mip level (R = u block, G = v block,
B = a checker + 8 x the level), uploaded level by level with WRITE_IMAGE (GEN_MIPMAP is not implemented on B).
Cases tunnel and tunnel_naive: tests/fs_tunnel.frag, a Shadertoy tunnel whose uv.y = atan(y, x) / pi jumps from 1 to -1
along the left half of the screen's middle row. tunnel passes textureGrad's derivatives of a uv without the jump (an
explicit LOD, the TMU has no gradient inputs), tunnel_naive leaves the LOD to the quad's own derivatives. Their texture
stores the level in blue, so --check reads back the LOD every pixel got: tunnel must match the host's formula on and
off the seam, tunnel_naive must show the seam (the check's proof that it can see one).
Needs STATUS layout 7. Standard library + the repository's tools.
"""
import collections, math, os, struct, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto as fp
import glslc, isa64
from fb_image import MODE_SIZE
from fb_shader import (words64_bytes, f24, H_VBO, H_IBO, H_VSPROG, A_VBO, A_IBO, A_VSPROG, H_FSPROG, B_FSPROG,
                       H_IMG, B_IMG, DRAW_ID, CLEAR, RENDER_TIMEOUT_CYCLES, err_text)

TESTS = os.path.join(HERE, '..', '..', 'tests')
B_TEX = 0x3100_0000                                             # textures above the two render targets (4 MB each at 0x3000_0000)
H_TEX = (0xC040, 0xC041, 0xC042, 0xC043)                        # board-B image handles of the 4 sampler slots
# filter codes of the DRAW_STATE sampler word bits [19:16] = {mag linear, mip[1:0] (0 none, 1 nearest, 2 linear), min linear}
FILT = {'nearest': 0x0, 'bilinear': 0x9, 'bilinear_mipnearest': 0xB, 'trilinear': 0xD}
TAPS = {'nearest': 1, 'bilinear': 4, 'bilinear_mipnearest': 4, 'trilinear': 8}
WRAP_REPEAT, WRAP_CLAMP, WRAP_MIRROR = 0, 1, 2

# ---- the cases: (shader, [(size, fmt, filter, levels), ...], repeats of textures 0/2, repeats of textures 1/3)
#      repeats r over the screen height -> r * size / H texels per pixel; mips: levels = full chain
RGBA8, RGB565 = fp.FMT_RGBA8, fp.FMT_RGB565
CASES = collections.OrderedDict([
    ('t1_256_bilinear',       ('tex1', [(256, RGBA8, 'bilinear', 1)], 1.0, 0.0)),
    ('t1_256_nearest',        ('tex1', [(256, RGBA8, 'nearest', 1)], 1.0, 0.0)),
    ('t1_256_565_bilinear',   ('tex1', [(256, RGB565, 'bilinear', 1)], 1.0, 0.0)),
    ('t1_2048_mag1',          ('tex1', [(2048, RGBA8, 'bilinear', 1)], 'pix', 0.0)),     # texel ~ pixel, no mips
    ('t1_2048_min8',          ('tex1', [(2048, RGBA8, 'bilinear', 1)], 'min8', 0.0)),    # 8 texels per pixel, no mips: cache-hostile
    ('t1_2048_tri_mag1',      ('tex1', [(2048, RGBA8, 'trilinear', 12)], 'pix', 0.0)),
    ('t1_2048_tri_min8',      ('tex1', [(2048, RGBA8, 'trilinear', 12)], 'min8', 0.0)),  # lambda = 3: level 3 (256 x 256)
    # min6: 6 texels per pixel -> lambda = log2 6 = 2.58, BETWEEN levels 2 and 3: every pixel samples two levels (the
    # 'min8' cases sit exactly on level 3 and are single-level); TMU C1 (one 8-tap job per pixel) is measured on these
    ('t1_2048_tri_min6',      ('tex1', [(2048, RGBA8, 'trilinear', 12)], 'min6', 0.0)),
    ('t2_2048tri_256_min6',   ('tex2', [(2048, RGBA8, 'trilinear', 12), (256, RGBA8, 'bilinear', 1)], 'min6', 1.0)),
    ('t4_2x2048tri_2x256_min6', ('tex4', [(2048, RGBA8, 'trilinear', 12), (256, RGBA8, 'bilinear', 1),
                                          (2048, RGBA8, 'trilinear', 12), (256, RGBA8, 'bilinear', 1)], 'min6', 1.0)),
    ('t1_2048_mipn_min8',     ('tex1', [(2048, RGBA8, 'bilinear_mipnearest', 12)], 'min8', 0.0)),
    ('t1_2048_565_tri_min8',  ('tex1', [(2048, RGB565, 'trilinear', 12)], 'min8', 0.0)),
    ('t2_2048tri_256',        ('tex2', [(2048, RGBA8, 'trilinear', 12), (256, RGBA8, 'bilinear', 1)], 'min8', 1.0)),
    ('t4_2x2048tri_2x256',    ('tex4', [(2048, RGBA8, 'trilinear', 12), (256, RGBA8, 'bilinear', 1),
                                        (2048, RGBA8, 'trilinear', 12), (256, RGBA8, 'bilinear', 1)], 'min8', 1.0)),
    ('t4_4x256_bilinear',     ('tex4', [(256, RGBA8, 'bilinear', 1)] * 4, 1.0, 1.0)),
    ('cube_256_bilinear',     ('texcube', [(256, RGBA8, 'bilinear', 1)], 1.0, 0.0)),
    ('cube_256_trilinear',    ('texcube', [(256, RGBA8, 'trilinear', 9)], 1.0, 0.0)),
    # face-stride probe: with 7 levels a face's chain (349 504 bytes) is a multiple of 32, with 8 (349 520) it is not
    ('cube_256_tri_lv7',      ('texcube', [(256, RGBA8, 'trilinear', 7)], 1.0, 0.0)),
    ('cube_256_tri_lv8',      ('texcube', [(256, RGBA8, 'trilinear', 8)], 1.0, 0.0)),
    ('tunnel',                ('tunnel', [(256, RGBA8, 'trilinear', 9)], 1.0, 0.0)),
    ('tunnel_naive',          ('tunnel_naive', [(256, RGBA8, 'trilinear', 9)], 1.0, 0.0)),
])
SHADERS = {'tex1': 'fs_tex1.frag', 'tex2': 'fs_tex2.frag', 'tex4': 'fs_tex4.frag', 'texcube': 'fs_texcube.frag',
           'tunnel': 'fs_tunnel.frag', 'tunnel_naive': 'fs_tunnel.frag', 'tex3d': 'fs_tex3d.frag'}
TUNNEL_BLUE = 24                                                # blue = this x the mip level


# ------------------------------------------------------------------------------------------------ the procedural texture
def block_colour(ub, vb, level):
    """32 x 32 blocks over (u, v): R = u block, G = v block (x 8), B = checker + 8 x the level tint (capped at 120)"""
    return ((ub * 8) & 0xFF, (vb * 8) & 0xFF, ((ub ^ vb) & 1) * 128 + min(level * 8, 120))


def level_rgba(size, level):
    """one mip level (size x size texels) as RGBA8888 bytes: the block pattern of the normalised coordinates"""
    rows = {}
    out = bytearray()
    for y in range(size):
        vb = (y * 32) // size
        if vb not in rows:
            row = bytearray(size * 4)
            for x in range(size):
                r, g, b = block_colour((x * 32) // size, vb, level)
                row[4 * x:4 * x + 4] = bytes((r, g, b, 255))
            rows[vb] = bytes(row)
        out += rows[vb]
    return bytes(out)


def level_565(size, level):
    out = bytearray(); rows = {}
    for y in range(size):
        vb = (y * 32) // size
        if vb not in rows:
            row = bytearray()
            for x in range(size):
                r, g, b = block_colour((x * 32) // size, vb, level)
                v = ((r * 31 * 2 + 255) // 510 << 11) | ((g * 63 * 2 + 255) // 510 << 5) | ((b * 31 * 2 + 255) // 510)
                row += struct.pack("<H", v)
            rows[vb] = bytes(row)
        out += rows[vb]
    return bytes(out)


def tunnel_level(size, level):
    """a tunnel texture level: R, G = 4 periods of a cosine over u, v, the same at every level; B = the level"""
    ramp = [128 + int(round(127 * math.cos(2.0 * math.pi * 4.0 * (i + 0.5) / size))) for i in range(size)]
    b = level * TUNNEL_BLUE
    return b''.join(bytes(c for r in ramp for c in (r, g, b, 255)) for g in ramp)


def upload_texture(bd, handle, base, size, fmt, levels, layers=1, pattern=None):
    """CREATE_IMAGE + WRITE_IMAGE of every level of every layer; returns the bytes the image occupies"""
    bpp = fp.STORAGE_BPP[fmt]
    bd.create_image(handle, base, size, size, fmt, levels=levels, layers=layers, usage=fp.USE_SAMPLED | fp.USE_TRANSFER)
    chain = 0
    for lv in range(levels):
        s = max(1, size >> lv)
        chain += ((s * bpp * s) + 3) & ~3
    stride = (chain + 31) & ~31
    cache = {}
    for layer in range(layers):
        for lv in range(levels):
            s = max(1, size >> lv)
            tint = lv + layer * 2                                     # cube faces: a different tint per face
            key = (s, tint)
            if key not in cache:
                cache[key] = pattern(s, tint) if pattern else level_rgba(s, tint) if fmt == RGBA8 else level_565(s, tint)
            bd.write_image(handle, cache[key], fp.TF_RGBA8888 if fmt == RGBA8 else fp.TF_RGB565, 0, 0, s, s, level=lv, layer=layer,
                           storage_fmt=fmt)
    return stride * layers


# ------------------------------------------------------------------------------------------------ programs
def compile_programs(shader, forward=False):
    forward = forward or shader.startswith('tunnel')
    src = open(os.path.join(TESTS, SHADERS[shader]), encoding='utf-8').read()
    if shader == 'tunnel_naive': src = src.replace('#define IMPLEMENTATION 1', '#define IMPLEMENTATION 0')
    rv = glslc.compile_stage(open(os.path.join(TESTS, 'fs_quad.vert'), encoding='utf-8').read(), 'vs', isa64=True, forward=forward)
    rf = glslc.compile_stage(src, 'fs', isa64=True, forward=forward)
    wv, _ = isa64.assemble(rv['sasm'].splitlines(), vs=True)
    wf, _ = isa64.assemble(rf['sasm'].splitlines())
    assert rv['attribs'] == {'aPos': (0, 'vec2')} and rv['uniforms'] == {'uParam': (0, 'vec4')} and rv['const_base'] == 1
    assert rv['varyings'] == {'vUV': (0, 'vec4')} and rf['varyings'] == {'vUV': (0, 'vec4')}, (rv['varyings'], rf['varyings'])
    vconst = [tuple(float(c) for c in k) for k in rv['const_pool']]
    fconst = [tuple(float(c) for c in k) for k in rf['const_pool']]
    return wv, wf, vconst, fconst, rf['uniforms'], rf['const_base'], rf['samplers']


def fs_uniforms(funi, fconst, fs_cb, scale):
    """scale: uScale's vector, or {name: vector} for a shader with other uniforms"""
    u = [(0.0, 0.0, 0.0, 0.0)] * fs_cb
    for name, v in (scale if isinstance(scale, dict) else {'uScale': scale}).items(): u[funi[name][0]] = v
    return u + fconst


# ------------------------------------------------------------------------------------------------ the board
def setup(bd, W, H, wv, wf, fconst, funi, fs_cb, samplers, texs, scale, pattern=None):
    quad = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    idx = [0, 1, 2, 0, 2, 3]
    bd.create_buffer(H_VBO, A_VBO, 4096); bd.write_buffer(H_VBO, 0, b''.join(struct.pack("<I", f24(c)) for p in quad for c in p))
    bd.create_buffer(H_IBO, A_IBO, 4096); bd.write_buffer(H_IBO, 0, struct.pack("<%dH" % len(idx), *idx))
    bd.create_buffer(H_VSPROG, A_VSPROG, 4096); bd.write_buffer(H_VSPROG, 0, words64_bytes(wv))
    bd.vs_program(H_VSPROG, 0, len(wv))
    for i in range(16):
        if i == 0: bd.vertex_attr(0, True, 2, H_VBO, 8, 0, type=fp.AT_FP24)
        else: bd.vertex_attr(i, False, 4, 0, 0, 0, (0.0, 0.0, 0.0, 1.0))
    bd.vs_state((W * 8.0, W * 8.0, H * 8.0, H * 8.0), cull=0, front_cw=0, fsmask=0x01, width=W, height=H, draw_id=DRAW_ID)
    bd.create_buffer(H_FSPROG, B_FSPROG, 65536); bd.write_buffer(H_FSPROG, 0, words64_bytes(wf))
    for h, base in zip(H_IMG, B_IMG):
        bd.create_image(h, base, W, H, fp.FMT_RGBA8, origin=1, usage=fp.USE_RT | fp.USE_SCANOUT | fp.USE_TRANSFER)
    # the textures: one image per sampler slot of the program
    t0 = time.monotonic(); base = B_TEX; nbytes = 0
    for unit, (size, fmt, filt, levels, cube) in enumerate(texs):
        n = upload_texture(bd, H_TEX[unit], base, size, fmt, levels, layers=6 if cube else 1, pattern=pattern)
        base += (n + 0xFFFFF) & ~0xFFFFF; nbytes += n
    bd.fence(timeout=60.0)
    print(f"  textures: {nbytes / 1e6:.1f} MB uploaded in {time.monotonic() - t0:.1f} s")
    uni = fs_uniforms(funi, fconst, fs_cb, scale)
    h = [0] * 23
    h[0] = 0xF << 16                                           # no depth test, colour mask RGBA, no blend
    h[11] = len(wf) << 16 | H_FSPROG; h[12] = 0; h[13] = len(uni)
    for name, unit in samplers.items():
        size, fmt, filt, levels, cube = texs[unit]
        h[15 + unit] = H_TEX[unit] | (FILT[filt] << 16) | (WRAP_REPEAT << 20) | (WRAP_REPEAT << 22)
    bd.draw_state(DRAW_ID, h, uni)
    bd.fence(timeout=10.0)
    return len(idx)


def render_frame(bd, i, W, H, aspect, tm, vconst, nidx):
    bd.frame_begin()
    bd.vs_uniforms(0, [(aspect, tm, 0.0, 0.0)] + vconst)
    bd.draw(0, nidx, 0, index_handle=H_IBO)
    cc = tuple(fp.c16(c) for c in CLEAR)
    bd.flush(H_IMG[i & 1], fp.FL_CLEAR_COLOR | fp.FL_CMASK(0xF), cc, scissor=(0, 0, W, H))
    return bd.present(H_IMG[i & 1], frame_id=i, hold=False, wait=False)


# ------------------------------------------------------------------------------------------------ the host's sampling (check)
def host_uv(px, py, W, H, aspect, rep, scroll=0.0):
    """the fragment program's uv of pixel (px, py) (window coordinates, y up) for texture 0's repeats at time 0"""
    nx = (px + 0.5) / W * 2.0 - 1.0; ny = (py + 0.5) / H * 2.0 - 1.0
    return (nx * aspect / aspect) * 0.5 * rep + 0.5, ny * 0.5 * rep + 0.5


def expect_pixel(uv, size, level_tint_lo, level_tint_hi, margin=2.5):
    """the block colour at uv if every tap of a bilinear look-up lies inside one block (else None); B's tint = the
    level (loose: between the two levels given); margin = the taps' reach in level-0 texels (2^level x 1 texel + slack)"""
    u, v = uv[0] % 1.0, uv[1] % 1.0
    ub, vb = int(u * 32), int(v * 32)
    tpb = size / 32.0
    du = (u * 32 - ub) * tpb; dv = (v * 32 - vb) * tpb
    if min(du, tpb - du, dv, tpb - dv) < margin: return None
    r, g, b = block_colour(ub, vb, 0)
    return r, g, (b & 128) + min(level_tint_lo * 8, 120), (b & 128) + min(level_tint_hi * 8, 120)


def check_frame(bd, W, H, aspect, case, texs, rep0, min_lambda, dump=False):
    """frame 0 from B's DDR: ~40 pixels against the host's block colours (nearest / bilinear exact inside a block;
    the mip level tint between floor and ceil of lambda; RGB565: +-8). dump: the whole frame's level-tint histogram
    and where the pixels off the expected level are (mip cases), the frame as tools/gen/texperf_<case>.ppm"""
    data = bd.mem_read(B_IMG[0], W * H * 4, board=1)
    shader = CASES[case][0]
    size, fmt, filt, levels, cube = texs[0]
    if shader in ('tex2', 'tex4'):
        print(f"  check {case}: products of several textures, not compared"); return True
    bad = 0; n = 0; flipped = 0
    lam = min_lambda if filt in ('trilinear', 'bilinear_mipnearest') else 0.0
    lo, hi = (int(math.floor(lam)), int(math.ceil(lam))) if levels > 1 else (0, 0)
    tol = 8 if fmt == RGB565 else 1
    margin = 1.25 * (2 ** (hi + (1 if filt == 'trilinear' else 0))) + 0.5   # the taps reach 2^level level-0 texels
    if dump:
        with open(os.path.join(HERE, '..', '..', 'tools', 'gen', f'texperf_{case}.ppm'), 'wb') as f:
            f.write(b"P6 %d %d 255\n" % (W, H))
            for row in range(H - 1, -1, -1):
                for px in range(W):
                    o = (row * W + px) * 4; f.write(bytes((data[o + 3], data[o + 2], data[o + 1])))
        hist = collections.Counter(); offx = collections.Counter(); offy = collections.Counter(); noff = 0
        for py in range(H):
            for px in range(W):
                o = (py * W + px) * 4; t = (data[o + 1] & 0x7F) / 8.0
                hist[round(t * 2) / 2] += 1
                if levels > 1 and abs(t - lam) > 0.6:
                    noff += 1; offx[px // 32] += 1; offy[py // 32] += 1
        print("  dump: level tint histogram " + ", ".join(f"{k}: {v}" for k, v in sorted(hist.items())))
        if levels > 1:
            print(f"  dump: {noff} pixels off the expected level {lam:.2f}; by 32-pixel column: " +
                  " ".join(f"{k}:{v}" for k, v in sorted(offx.items())) + "; by 32-row band: " + " ".join(f"{k}:{v}" for k, v in sorted(offy.items())))
    for k in range(400):
        px = (k * 97 + 13) % W; py = (k * 61 + 7) % H
        if shader == 'texcube':
            nx = (px + 0.5) / W * 2.0 - 1.0; ny = (py + 0.5) / H * 2.0 - 1.0
            x, y = nx * aspect * rep0, ny * rep0
            if max(abs(x), abs(y)) >= 0.95: continue                   # +Z face only (layer 4), away from the edges
            u = (x + 1.0) * 0.5; v = (1.0 - y) * 0.5                   # ES 2.0 table 3.11 for +Z: s = x, t = -y
            exp = expect_pixel((u, v), size, 4 * 2 + lo, 4 * 2 + hi, margin)
        else:
            exp = expect_pixel(host_uv(px, py, W, H, aspect, rep0), size, lo, hi, margin)
        if exp is None: continue
        n += 1
        got = None
        for row in (py, H - 1 - py):                                   # the target's row order (origin 1 = GL)
            o = (row * W + px) * 4
            a, b, g, r = data[o], data[o + 1], data[o + 2], data[o + 3]
            ok = abs(r - exp[0]) <= tol and abs(g - exp[1]) <= tol and exp[2] - tol <= b <= exp[3] + tol
            if ok: got = (r, g, b); flipped += (row != py); break
        if got is None:
            bad += 1
            if bad <= 5:
                o = (py * W + px) * 4
                print(f"  check: pixel ({px}, {py}) = R{data[o + 3]} G{data[o + 2]} B{data[o + 1]}, expected R{exp[0]} G{exp[1]} B{exp[2]}..{exp[3]}")
        if n >= 40: break
    print(f"  check {case}: {n} pixels compared, {bad} wrong" + (f" (rows read {'bottom-up' if flipped > n / 2 else 'top-down'})" if n else ""))
    return n > 0 and bad == 0


def tunnel_res(W, H):
    """the tunnel's iResolution: its centre row boundary must fall INSIDE a 2 x 2 quad for the seam to exist at all (the
    quad's derivatives span it). H / 2 is even in every mode, so the centre moves up one row: H + 2"""
    return (float(W), float(H + 2), 1.0, 0.0)


def check_tunnel(bd, W, H, shader, size):
    """time 0 from B's DDR. Blue / (24 r) = the LOD each pixel got. tunnel: every sampled pixel and the seam against the
    host's lambda (ES 3.00 3.8.10 on the core's DDX / DDY of uv2) and, where the level still holds the cosine, R and G;
    tunnel_naive: the seam must stand out by at least 2 levels from the rows beside it"""
    data = bd.mem_read(B_IMG[0], W * H * 4, board=1)
    RW, RH = tunnel_res(W, H)[:2]
    def p(px, py): return (2.0 * (px + 0.5) - RW) / RH, (2.0 * (py + 0.5) - RH) / RH
    def r_of(x, y): return ((x * x) ** 4 + (y * y) ** 4) ** 0.125
    def uv2(px, py):
        x, y = p(px, py)
        return 0.3 / r_of(x, y), math.atan2(y, abs(x)) / 3.1415927
    def lam(px, py):
        xl, yl = px & ~1, py & ~1
        dx = [(a - b) * size for a, b in zip(uv2(xl + 1, py), uv2(xl, py))]
        dy = [(a - b) * size for a, b in zip(uv2(px, yl + 1), uv2(px, yl))]
        return max(0.5 * math.log2(max(dx[0] ** 2 + dx[1] ** 2, dy[0] ** 2 + dy[1] ** 2, 1e-30)), 0.0)
    def seen(px, py):
        o = (py * W + px) * 4; x, y = p(px, py); r = r_of(x, y)
        return data[o + 3], data[o + 2], data[o + 1] / (TUNNEL_BLUE * r), r
    seam = [(px, py) for px in range(24, W // 2 - 24, 17) for py in (H // 2, H // 2 + 1)]
    beside = [(px, H // 2 + 8) for px in range(24, W // 2 - 24, 17)]
    lv_seam = sum(seen(*q)[2] for q in seam) / len(seam); lv_side = sum(seen(*q)[2] for q in beside) / len(beside)
    print(f"  check {shader}: mean level {lv_seam:.2f} on the seam, {lv_side:.2f} beside it")
    if shader == 'tunnel_naive':
        return lv_seam > lv_side + 2.0
    pts = seam + [((k * 97 + 13) % W, (k * 61 + 7) % H) for k in range(200)]
    bad = n = 0; worst = 0.0
    for px, py in pts:
        rr, gg, lv, r = seen(px, py)
        if r < 0.3 or r > 0.95: continue
        x, y = p(px, py); n += 1
        e = lam(px, py); d = abs(lv - e); worst = max(worst, d); wrong = d > 0.4
        if e < 2.5:                                     # R and G: the cosine, darkened by r
            u = 0.3 / r; v = math.atan2(y, x) / 3.1415927
            ex = [(128 + 127 * math.cos(2.0 * math.pi * 4.0 * t)) * r for t in (u, v)]
            wrong = wrong or abs(rr - ex[0]) > 24 or abs(gg - ex[1]) > 24
        if wrong:
            bad += 1
            if bad <= 5: print(f"  check: pixel ({px}, {py}) R{rr} G{gg} level {lv:.2f}, expected level {e:.2f}")
    print(f"  check {shader}: {n} pixels compared, {bad} wrong, worst level error {worst:.2f}")
    return n > 100 and bad == 0


# ------------------------------------------------------------------------------------------------ one case
def sim_uops(wf, funi, fconst, fs_cb, scale, aspect):
    """the core simulator's micro-op count for one warp of 16 pixels (the bench texture model answers the TEXes):
    the core's issue cycles for 4 quads -> the ideal cycles per quad = uops / 4"""
    params = [[f24(c) for c in v] for v in fs_uniforms(funi, fconst, fs_cb, scale)]
    params += [[0] * 4] * (64 - len(params))
    varin = [[[f24(0.1 * (l & 3) * aspect), f24(0.05 * (l >> 2)), f24(0.0), f24(1.0)]] + [[0] * 4] * 15 for l in range(16)]
    res = isa64.simulate_fs2(wf, isa64.FSInputs(params, varin, active=0xFFFF, interp=False, tex_fn=isa64.tex_model))
    return res.uops, res.err


def run_case(bd, case, mode, secs, check, dump=False, forward=False):
    shader, texl, rep0, rep1 = CASES[case]
    texs = [(size, fmt, filt, levels, shader == 'texcube') for size, fmt, filt, levels in texl]
    wv, wf, vconst, fconst, funi, fs_cb, samplers = compile_programs(shader, forward)
    bd.set_mode(mode)
    W, H = MODE_SIZE[mode]; aspect = W / H; nquads = (W // 2) * (H // 2)
    size0 = texs[0][0]
    rep0 = {'pix': H / size0, 'min6': 6.0 * H / size0, 'min8': 8.0 * H / size0}.get(rep0, rep0)   # repeats over H: texels per pixel = rep * size / H
    if shader == 'texcube': scale = (rep0, 0.3, 1.0 / aspect, 0.0)
    elif shader.startswith('tunnel'): scale = {'iResolution': tunnel_res(W, H), 'iChannelSize': (float(size0), float(size0), 0.0, 0.0)}
    else: scale = (rep0, 0.05, 1.0 / aspect, rep1)
    tpp = rep0 * size0 / H                                              # texels per pixel of texture 0 (vertical)
    min_lambda = math.log2(max(tpp, 1.0))
    uops, serr = sim_uops(wf, funi, fconst, fs_cb, scale, aspect)
    print(f"=== {case}: {shader}, {len(wf)} instructions ({uops} micro-ops per warp = {uops / 4:.0f} ideal cycles per quad"
          f"{', SIM ERROR' if serr else ''}), mode {mode} ({W} x {H}); textures " +
          ", ".join(f"{s}x{s}{'x6' if c else ''} {'RGBA8' if f == RGBA8 else 'RGB565'} {fl} lv{l}" for s, f, fl, l, c in texs) +
          f"; texture 0: {tpp:.2f} texels per pixel (lambda {min_lambda:.2f})")
    nidx = setup(bd, W, H, wv, wf, fconst, funi, fs_cb, samplers, texs, scale, tunnel_level if shader.startswith('tunnel') else None)
    errs = bd.take_errors()
    for e in errs: print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
    if errs: return None
    st0 = bd.status(); t0 = time.monotonic(); shown = 0; retrains = 0; ok = True; chk = True
    pending = collections.deque(); i = -1; checked = not check
    while True:
        if time.monotonic() - t0 >= secs: break
        i += 1
        while len(pending) >= 2:
            fi, seq = pending.popleft()
            try: bd.wait_event(seq, timeout=20.0); shown += 1
            except fp.ProtoError as e: print("  frame %d: %s" % (fi, e))
        seq = render_frame(bd, i, W, H, aspect, 0.0 if not checked else 0.03 * i, vconst, nidx)
        pending.append((i, seq))
        if not checked:
            bd.wait_event(seq, timeout=20.0); shown += 1; pending.clear(); checked = True
            chk = check_tunnel(bd, W, H, shader, size0) if shader.startswith('tunnel') else check_frame(bd, W, H, aspect, case, texs, rep0, min_lambda, dump)
            t0 = time.monotonic(); i = -1; st0 = bd.status()               # the measurement starts after the check
        errs = bd.take_errors()
        for e in errs:
            print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
            if e[1] == 9 and e[3] < RENDER_TIMEOUT_CYCLES: retrains += 1
            else: ok = False
        if not ok: break
    while pending:
        fi, seq = pending.popleft()
        try: bd.wait_event(seq, timeout=20.0); shown += 1
        except fp.ProtoError as e: print("  frame %d: %s" % (fi, e))
    el = time.monotonic() - t0
    bd.fence(timeout=30.0)
    st = bd.status(); n = i + 1; q = max(1, st['render_quads']); fps = n / el if el else 0.0
    cyc = st['b_cyc_frame'] / q
    nsamp = len(samplers); taps = sum(TAPS[t[2]] for t in texs[:nsamp]) if shader != 'tex4' else sum(TAPS[t[2]] for t in texs)
    msamp = fps * W * H * nsamp / 1e6; mtex = fps * W * H * taps / 1e6
    print(f"  {n} frames in {el:.1f} s = {fps:.2f} fps, {fps * q / 1e6:.3f} M quads/s; {msamp:.1f} M samples/s, {mtex:.1f} M texel taps/s; "
          f"link retrains {retrains}, A errs {st['errs'] - st0['errs']}; " + fp.b_frame_split(st))
    res = (f"RESULT {case} mode={mode} {W}x{H} fps={fps:.2f} cyc_per_quad={cyc:.1f} fs_busy={100.0 * st['b_cyc_fs'] / max(1, st['b_cyc_frame']):.1f}% "
           f"ropstall={100.0 * st['b_cyc_ropstall'] / max(1, st['b_cyc_frame']):.1f}% starved={100.0 * st['b_cyc_starved'] / max(1, st['b_cyc_frame']):.1f}% "
           f"samples_per_px={nsamp} taps_per_px={taps} Msamples={msamp:.1f} Mtaps={mtex:.1f} instr={len(wf)} ideal_cyc={uops / 4:.1f} "
           f"check={'ok' if chk else 'FAIL'}{'' if ok else ' ERRORS'}")
    print(res); sys.stdout.flush()
    return res


def main():
    a = sys.argv[1:]
    def opt(name, default=None, conv=str):
        return conv(a[a.index(name) + 1]) if name in a else default
    if '--list' in a:
        for k, v in CASES.items(): print(k, v)
        return
    ip = opt('--ip', "192.168.1.10"); mode = opt('--mode', 1, int); secs = opt('--secs', 6.0, float)
    cases = list(CASES) if '--all' in a else [opt('--case', 't1_256_bilinear')]
    for c in cases:
        if c not in CASES: raise SystemExit("unknown case %s (--list)" % c)
    bd = fp.Board(ip)
    bd.hello()
    st = bd.status()
    lfl = fp.decode_link_flags(st['link_flags'])
    if not (lfl['trained'] and lfl['b_alive']): raise fp.ProtoError("the board link is not up / B is not alive")
    print(f"board A build {st['build']}")
    results = []
    for c in cases:
        r = run_case(bd, c, mode, secs, '--check' in a or '--all' in a, '--dump' in a, '--forward' in a)
        results.append(r or f"RESULT {c} FAILED")
    print("\n".join(results))


if __name__ == '__main__': main()
