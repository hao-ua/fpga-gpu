#!/usr/bin/env python3
"""fb_floor — the OpenArena "lava through the floor" check: a lava plane (red) under a large floor plane (grey) that runs
from behind the camera to far ahead, the eye 50 units above the floor (Quake 3's view height above the feet), near 4, the
floor in two big triangles as a Q3 brush face; the lava is drawn FIRST, then the floor, depth LEQUAL with writes. Every
ray below the horizon meets the floor before the lava, so no lava pixel may survive: any red pixel is a hole in the floor.
Many random views (yaw, pitch, position); the views with holes are listed and the worst one is written as a PNG.

  python3 host/fb/fb_floor.py [--ip 192.168.1.10] [--mode 0] [--views 60] [--seed 1] [--png out.png] [--order lava|floor]
  [--split N] (the floor as N x N cells instead of two triangles)
"""
import collections, math, os, random, struct, sys, time, zlib
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, '..', '..', 'tools'))
import fbproto as fp
from fb_image import MODE_SIZE
from fb_shader import words64_bytes, f24, H_VBO, H_IBO, H_VSPROG, A_VBO, A_IBO, A_VSPROG, H_FSPROG, b_fsprog, H_IMG, B_IMG, DRAW_ID, err_text
from fb_bench import compile_pair, uniforms

VS = """attribute vec3 aPos;
attribute vec4 aCol;
uniform mat4 uM;
varying vec4 vCol;
void main() { gl_Position = uM * vec4(aPos, 1.0); vCol = aCol; }
"""
FS = "precision mediump float;\nvarying vec4 vCol;\nvoid main() { gl_FragColor = vCol; }\n"
# --depth: the window depth in R (high 8 bits) and G (the next 8)
FS_DEPTH = ("precision highp float;\nvoid main() { float z = gl_FragCoord.z * 255.0; float h = floor(z);\n"
            "  gl_FragColor = vec4(h / 255.0, floor(fract(z) * 255.0) / 255.0, 0.0, 1.0); }\n")


def ref_depth(eye, yaw, pitch, W, H, fovy=73.74, near=4.0, far=4096.0):
    """the exact window depth of the floor per pixel (memory row order: row 0 at the bottom), None where no floor"""
    f = (-math.sin(yaw) * math.cos(pitch), -math.sin(pitch), -math.cos(yaw) * math.cos(pitch))
    r = (-f[2], 0.0, f[0]); n = math.hypot(r[0], r[2]); r = (r[0] / n, 0.0, r[2] / n)
    u = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
    ty = math.tan(math.radians(fovy) / 2); tx = ty * W / H
    out = []
    for my in range(H):
        ny = -1.0 + 2.0 * (my + 0.5) / H
        row = []
        for px in range(W):
            nx = 2.0 * (px + 0.5) / W - 1.0
            d = tuple(f[i] + nx * tx * r[i] + ny * ty * u[i] for i in range(3))
            z = None
            if d[1] < 0:
                t = -eye[1] / d[1]
                hx = eye[0] + t * d[0]; hz = eye[2] + t * d[2]
                if t >= near and -1024 <= hx <= 1024 and -2048 <= hz <= 512:
                    zc = (far + near) / (near - far) * (-t) + 2 * far * near / (near - far)
                    z = 0.5 * zc / t + 0.5
            row.append(z)
        out.append(row)
    return out
RED, GREY, SKY = (1.0, 0.0, 0.0, 1.0), (0.5, 0.5, 0.5, 1.0), (0.0, 0.0, 1.0, 1.0)


def mat_mul(a, b): return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def mvp(eye, yaw, pitch, W, H, fovy=73.74, near=4.0, far=4096.0):
    f = (-math.sin(yaw) * math.cos(pitch), -math.sin(pitch), -math.cos(yaw) * math.cos(pitch))
    r = (-f[2], 0.0, f[0]); n = math.hypot(r[0], r[2]); r = (r[0] / n, 0.0, r[2] / n)
    u = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    V = [[r[0], r[1], r[2], -dot(eye, r)], [u[0], u[1], u[2], -dot(eye, u)], [-f[0], -f[1], -f[2], dot(eye, f)], [0, 0, 0, 1]]
    c = 1.0 / math.tan(math.radians(fovy) / 2); asp = W / H
    P = [[c / asp, 0, 0, 0], [0, c, 0, 0], [0, 0, (far + near) / (near - far), 2 * far * near / (near - far)], [0, 0, -1, 0]]
    M = mat_mul(P, V)
    return [tuple(M[i][k] for i in range(4)) for k in range(4)]          # columns


def reference(eye, yaw, pitch, W, H, fovy=73.74, near=4.0, lava=True):
    """the expected image, top row first: a ray per pixel centre against the two planes (the nearest hit inside its extent,
    beyond the near plane); 'L' lava, 'F' floor, 'S' sky"""
    f = (-math.sin(yaw) * math.cos(pitch), -math.sin(pitch), -math.cos(yaw) * math.cos(pitch))
    r = (-f[2], 0.0, f[0]); n = math.hypot(r[0], r[2]); r = (r[0] / n, 0.0, r[2] / n)
    u = (r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0])
    ty = math.tan(math.radians(fovy) / 2); tx = ty * W / H
    out = []
    for py in range(H):
        ny = 1.0 - 2.0 * (py + 0.5) / H
        row = []
        for px in range(W):
            nx = 2.0 * (px + 0.5) / W - 1.0
            d = tuple(f[i] + nx * tx * r[i] + ny * ty * u[i] for i in range(3))   # depth along f is 1 per unit t
            best = 'S'; bt = None
            for (y, x0, x1, z0, z1, tag) in ((0.0, -1024, 1024, -2048, 512, 'F'), (-64.0, -512, 512, -1024, 256, 'L'))[:2 if lava else 1]:
                if d[1] >= 0: continue
                t = (y - eye[1]) / d[1]
                if t < near: continue
                hx = eye[0] + t * d[0]; hz = eye[2] + t * d[2]
                if x0 <= hx <= x1 and z0 <= hz <= z1 and (bt is None or t < bt): best, bt = tag, t
            row.append(best)
        out.append(row)
    return out


def plane(x0, x1, z0, z1, y, col, n):
    v = []; idx = []
    for j in range(n + 1):
        for i in range(n + 1):
            v.append((x0 + (x1 - x0) * i / n, y, z0 + (z1 - z0) * j / n) + col)
    for j in range(n):
        for i in range(n):
            a = j * (n + 1) + i; b = a + 1; d = a + n + 1; e = d + 1
            idx += [a, b, e, a, e, d]
    return v, idx


def png(path, W, H, rgba_rows):
    raw = b''.join(b'\x00' + row for row in rgba_rows)
    def chunk(t, d): return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    open(path, 'wb').write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack(">IIBBBBB", W, H, 8, 6, 0, 0, 0)) +
                           chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def main():
    a = sys.argv[1:]
    def opt(name, default, conv=str): return conv(a[a.index(name) + 1]) if name in a else default
    ip = opt('--ip', fp.BOARD_IP); mode = opt('--mode', 0, int); views = opt('--views', 60, int)
    rnd = random.Random(opt('--seed', 1, int)); out = opt('--png', None); order = opt('--order', 'lava')
    split = opt('--split', 1, int)
    W, H = MODE_SIZE[mode]
    bd = fp.Board(ip); bd.hello(); bd.set_mode(mode)
    st = bd.status(); print(f"board A build {st['build']}, {W} x {H}, order {order}, floor {split} x {split} cells")
    depthviz = '--depth' in a
    rv, rf, wv, wf = compile_pair(VS, FS_DEPTH if depthviz else FS)
    lv, li = plane(-512, 512, -1024, 256, -64.0, RED, 1)
    fv, fi = plane(-1024, 1024, -2048, 512.0, 0.0, GREY, split)
    if order == 'lava': v = lv + fv; idx = li + [k + len(lv) for k in fi]
    elif order == 'floor': v = fv + lv; idx = fi + [k + len(fv) for k in li]
    else: v = fv; idx = fi                                                  # --order none: the floor alone
    bd.create_buffer(H_VBO, A_VBO, 4 << 20); bd.write_buffer(H_VBO, 0, b''.join(struct.pack("<I", f24(c)) for p in v for c in p))
    bd.create_buffer(H_IBO, A_IBO, 1 << 20); bd.write_buffer(H_IBO, 0, struct.pack("<%dH" % len(idx), *idx))
    bd.create_buffer(H_VSPROG, A_VSPROG, 4096); bd.write_buffer(H_VSPROG, 0, words64_bytes(wv)); bd.vs_program(H_VSPROG, 0, len(wv))
    loc = {n: (s[0] if isinstance(s, (tuple, list)) else s) for n, s in rv['attribs'].items()}
    print(f"attribute slots {loc}")
    for i in range(16):
        if i == loc['aPos']: bd.vertex_attr(i, True, 3, H_VBO, 28, 0, type=fp.AT_FP24)
        elif i == loc['aCol']: bd.vertex_attr(i, True, 4, H_VBO, 28, 12, type=fp.AT_FP24)
        else: bd.vertex_attr(i, False, 4, 0, 0, 0, (0.0, 0.0, 0.0, 1.0))
    mask = 0
    for name, (slot, _ty) in rf['varyings'].items(): mask |= 1 << slot
    bd.vs_state((W * 8.0, W * 8.0, H * 8.0, H * 8.0), cull=0, front_cw=0, fsmask=mask or 1, width=W, height=H, draw_id=DRAW_ID)
    bd.create_buffer(H_FSPROG, b_fsprog(bd), 65536); bd.write_buffer(H_FSPROG, 0, words64_bytes(wf))
    bd.create_image(H_IMG[0], B_IMG[0], W, H, fp.FMT_RGBA8, origin=1, usage=fp.USE_RT | fp.USE_SCANOUT | fp.USE_TRANSFER)
    ufs = uniforms(rf, {})
    h = [0] * 23
    h[0] = 0xF << 16 | 1 | 1 << 1 | fp.FN_LEQUAL << 2
    h[11] = len(wf) << 16 | H_FSPROG; h[13] = len(ufs)
    bd.draw_state(DRAW_ID, h, ufs)
    bd.fence(timeout=10.0)
    for e in bd.take_errors(): print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
    cc = tuple(fp.c16(c) for c in SKY)
    worst = None; bad = 0
    for k in range(views):
        eye = (rnd.uniform(-300, 300), 50.0, rnd.uniform(-600, 100))
        yaw = rnd.uniform(0, 2 * math.pi); pitch = math.radians(rnd.uniform(0, 70))
        if k == 0: eye, yaw, pitch = (0.0, 50.0, 0.0), 0.0, math.radians(30)
        uvs = uniforms(rv, {'uM': mvp(eye, yaw, pitch, W, H)})
        with bd.batch():
            bd.frame_begin()
            bd.vs_uniforms(0, uvs)
            bd.draw(0, len(idx), 0, index_handle=H_IBO)
            bd.flush(H_IMG[0], fp.FL_CLEAR_COLOR | fp.FL_CMASK(0xF) | fp.FL_CLEAR_DEPTH | fp.FL_CLEAR_STENCIL, cc,
                     clear_depth=0xFFFFFF, scissor=(0, 0, W, H))
        bd.fence(timeout=20.0)
        for e in bd.take_errors(): print("  EV_ERROR seq %d: %s" % (e[0], err_text(e[:4])))
        img = bd.mem_read(B_IMG[0], W * H * 4, board=1)
        red = grey = sky = other = 0; rows_red = set()
        for p in range(W * H):
            r, g, b = img[4 * p + 3], img[4 * p + 2], img[4 * p + 1]          # the word 0xRRGGBBAA, little-endian
            if r > 200 and g < 50 and b < 50: red += 1; rows_red.add(p // W)
            elif abs(r - 128) < 20 and abs(g - 128) < 20 and abs(b - 128) < 20: grey += 1
            elif b > 200 and r < 50: sky += 1
            else: other += 1
        if '--colours' in a:
            cnt = collections.Counter(bytes(img[4 * p:4 * p + 4]) for p in range(0, W * H, 7))
            print("  colours (every 7th pixel):", ", ".join(f"{c.hex()} x{n}" for c, n in cnt.most_common(5)))
        tag = f"view {k:3d}: eye ({eye[0]:6.1f}, {eye[1]:4.1f}, {eye[2]:6.1f}) yaw {math.degrees(yaw):5.1f} pitch {math.degrees(pitch):4.1f}: " \
              f"floor {grey} sky {sky} LAVA {red} other {other}"
        if depthviz:
            rd = ref_depth(eye, yaw, pitch, W, H)
            errs = []; worst_px = None
            for my in range(H):
                for x in range(W):
                    z = rd[my][x]
                    if z is None: continue
                    p = my * W + x
                    got = (img[4 * p + 3] * 256 + img[4 * p + 2]) / 65280.0          # R high, G low (x / 255 each)
                    e = got - z; errs.append(e)
                    if worst_px is None or abs(e) > abs(worst_px[0]): worst_px = (e, x, my, got, z)
            if errs:
                big = sum(1 for e in errs if abs(e) > 2e-3)
                tag += f"\n  depth vs exact over {len(errs)} floor pixels: mean {sum(errs) / len(errs):+.5f}, |err| > 0.002 at {big}, " \
                       f"worst {worst_px[0]:+.4f} at ({worst_px[1]}, {worst_px[2]}) got {worst_px[3]:.4f} want {worst_px[4]:.4f}"
                # a few sample rows
                for my in (0, H // 8, H // 4, H // 2, 3 * H // 4, H - 1):
                    x = W // 2; z = rd[my][x]; p = my * W + x
                    if z is not None: tag += f"\n    row {my:3d} col {x}: got {(img[4 * p + 3] * 256 + img[4 * p + 2]) / 65280.0:.4f} want {z:.4f}"
        if '--ref' in a:
            ref = reference(eye, yaw, pitch, W, H, lava=(order != 'none'))
            def cls(p):
                r_, g_, b_ = img[4 * p + 3], img[4 * p + 2], img[4 * p + 1]
                return 'L' if r_ > 200 and g_ < 50 else 'F' if abs(r_ - 128) < 20 and abs(b_ - 128) < 20 else 'S' if b_ > 200 else '?'
            for name, rowmap in (("memory row 0 = top", lambda y: y), ("memory row 0 = bottom", lambda y: H - 1 - y)):
                diff = collections.Counter()
                for y in range(H):
                    my = rowmap(y)
                    for x in range(W):
                        got = cls(my * W + x)
                        if got != ref[y][x]: diff[ref[y][x] + '->' + got] += 1
                tag += f"\n  vs reference ({name}): {sum(diff.values())} pixels differ " + " ".join(f"{k_} {n_}" for k_, n_ in diff.most_common(6))
        if red: bad += 1; tag += f" rows {min(rows_red)}..{max(rows_red)}"
        print(tag); sys.stdout.flush()
        if red and (worst is None or red > worst[0]) or (worst is None and k == views - 1): worst = (red, img, k)
    print(f"fb_floor: {bad} of {views} views show lava through the floor")
    if out and worst:
        img = worst[1]
        rgba = bytearray(len(img)); rgba[0::4] = img[3::4]; rgba[1::4] = img[2::4]; rgba[2::4] = img[1::4]; rgba[3::4] = img[0::4]
        png(out, W, H, [bytes(rgba[y * W * 4:(y + 1) * W * 4]) for y in range(H)][::-1])   # memory row 0 = the bottom (GL)
        print(f"wrote view {worst[2]} ({worst[0]} lava pixels) to {out}")
    return 1 if bad else 0


if __name__ == '__main__': sys.exit(main())
