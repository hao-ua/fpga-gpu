#!/usr/bin/env python3
"""record_py — the reference byte stream of the parity test: fb_cube's setup + frames,
fb_texperf's upload and a 2D sequence through the real fbproto packers, recorded instead of sent. g2_parity.c produces the
same scenes through gpu2core; parity.ps1 compares the files byte for byte.

  python3 host/win/tests/record_py.py OUT.dgram

File: [u32 length][datagram] ...
"""
import os, struct, sys
HERE = os.path.dirname(os.path.abspath(__file__))
FB = os.path.join(HERE, '..', '..', 'fb')
sys.path.insert(0, FB); sys.path.insert(0, os.path.join(HERE, '..', '..', '..', 'tools'))
import fbproto as fp
import fb_cube, fb_texperf


class FakeSock:
    def sendto(self, data, addr): pass
    def setsockopt(self, *a): pass
    def bind(self, *a): pass
    def settimeout(self, *a): pass
    def recvfrom(self, n): raise OSError
    def close(self): pass


class RecordingBoard(fp.Board):
    """fbproto.Board with the datagram writer in place of the socket: no ACKs, no waits. Reads record the requests of a read
    without losses; events are not awaited. Everything above _send_seq is the unchanged production code."""

    def __init__(self, path):
        fp.socket.socket = lambda *a, **k: FakeSock()
        super().__init__("127.0.0.1", b_sub_index=True)
        self.running = False
        self.rx_thread.join(1); self.mt_thread.join(1)
        self.f = open(path, 'wb')

    def _rec(self, d):
        self.f.write(struct.pack("<I", len(d)) + d)

    def _send_seq(self, typ, body=b"", timeout=10.0):
        seq = self.next_seq
        self.next_seq = (self.next_seq + 1) & 0xFFFFFFFF or 1
        self._rec(self._msg(typ, seq, body))
        return seq

    def send_unseq(self, typ, body=b""):
        self._rec(self._msg(typ, 0, body))

    def wait_event(self, ref, timeout=10.0):
        self._event_key(ref)
        return (0, 0, 0, 0)

    def fence(self, fid=None, timeout=10.0):
        if fid is None:
            fp.Board._fence_id += 1
            fid = fp.Board._fence_id
        self.send_seq(fp.T_FENCE, struct.pack("<I", fid))
        return fid

    def set_mode(self, mode, timeout=5.0):
        self.send_seq(fp.T_SET_MODE, struct.pack("<I", mode))
        return mode

    def _read(self, make, total, rel=True, base=0, max_req=256 * 1024, **kw):
        CH = fp.RD_CHUNK
        max_req = max(CH, max_req // CH * CH)
        for s in range(0, total, max_req):
            fp.Board._tag = (fp.Board._tag + 1) & 0xFFFFFFFF or 1
            typ, body = make(s, min(max_req, total - s), fp.Board._tag)
            self.send_seq(typ, body)
            self._flush_batch()
        return bytes(total)


def cube_scene(bd, W, H, ncubes, instanced, frames):
    wv, wf, vconst, fconst, fs_cb = fb_cube.compile_programs(instanced)
    scene = fb_cube.Scene(ncubes, W / H, 1.0)
    nidx = fb_cube.setup(bd, W, H, scene, instanced, wv, wf, vconst, fconst, fs_cb)
    for i in range(frames):
        fb_cube.render_frame(bd, i, W, H, scene, instanced, vconst, 1.0, nidx, True, True)


def textures(bd):
    fb_texperf.upload_texture(bd, 0xC040, 0x31000000, 2048, fp.FMT_RGBA8, 12)
    fb_texperf.upload_texture(bd, 0xC041, 0x32000000, 256, fp.FMT_RGB565, 1)
    fb_texperf.upload_texture(bd, 0xC042, 0x32400000, 64, fp.FMT_RGBA8, 7, layers=6)


def two_d(bd):
    pix = bytes(i & 255 for i in range(256 * 4 * 16))
    bd.create_image(0xC050, 0x2A000000, 256, 16, fp.FMT_RGBA8, pitch=1024)
    bd.write_image(0xC050, pix, fp.TF_RGBA8888, 0, 0, 256, 16)
    bd.write_image(0xC050, pix[:300], fp.TF_RGB888, 3, 4, 10, 10, first_pixel=7)
    bd.write_image(0xC050, pix[:64], fp.TF_RAW, 0, 0, 8, 2, storage_fmt=fp.FMT_RGB565)
    bd.fill_image(0xC050, fp.TF_RGBA8888, 0x80FF4020, 10, 10, 100, 60, mask=0xF)
    bd.fill_image(0xC050, fp.TF_RAW, 0xAB, 100, 100, 60, 40, mask=0x1, level=1, layer=2)
    bd.copy_image(0xC050, 0xC051, 0, 0, 120, 80, 5, 5, 60, 40, mirror_x=False, mirror_y=True, key=(0x00000000, 0x80FFFF00))
    bd.copy_image(0xC050, 0xC051, 20, 20, 30, 20, 100, 50, mirror_x=True)
    bd.read_image(0xC050, fp.TF_RGBA8888, 0, 0, 800, 600)
    bd.read_image(0xC050, fp.TF_BGR888, 10, 7, 100, 50, first=1000, count=777)
    bd.mem_read(0x30000000, 1 << 20, board=1)
    bd.read_buffer(0x0021, 16, 300000)
    bd.mem_write(0x30000000, pix[:3000], board=1)
    bd.memtest(0x20000000, 1 << 20, board=1, timeout=0)
    bd.set_mode(2)
    bd.set_palette(4, [0x112233, 0x445566, 0xFFFFFF])
    bd.cursor_shape([i * 0x01020304 & 0xFFFFFFFF for i in range(4096)])
    bd.cursor_show(fp.CUR_ANDXOR, hot=(32, 16), pos=(100, 200))
    bd.cursor_show(fp.CUR_COLOR, take=False)
    bd.cursor_move(-5, 7)
    bd.cursor_pos(300, -2)
    bd.cursor_hide()
    bd.create_image(0xC052, 0x2B000000, 64, 64, fp.FMT_D24S8 if hasattr(fp, 'FMT_D24S8') else 0x10, pitch=256, usage=fp.USE_RT)
    fp.STORAGE_BPP.setdefault(0x10, 4)
    bd.destroy(0xC052)
    bd.present(0xC050, frame_id=77, at_once=True, wait=False)
    # a batch with commands B answers (b_sub_index) and a split at BATCH_MAX
    with bd.batch():
        bd.frame_begin()
        for k in range(300):
            bd.vs_uniforms(k & 7, [(float(k), 0.5, -2.0, 1e-3)])
            bd.draw(k % 7, 3 + k, k, index_handle=None if k & 1 else 0x22, index_offset=k * 2, index_u8=bool(k & 2))
        bd.vertex_attr(5, True, 4, 0x21, 16, 12, type=fp.AT_UBYTE, normalized=True)
        bd.vs_state((400.5 * 8, 8.0, -300.25 * 8, 2400.0), cull=3, front_cw=1, fsmask=0xFF, width=1280, height=800, draw_id=8191, line_width=63.6,
                    depth_range=(0.0, 0.3))
        bd.draw_state(8191, [i * 0x01010101 for i in range(23)], [(1.0, 2.0, 3.0, 4.0)] * 3)
        bd.flush(0xC050, fp.FL_CLEAR_COLOR | fp.FL_CLEAR_SCISSOR | fp.FL_DITHER | fp.FL_CLEAR_DEPTH | fp.FL_CLEAR_STENCIL | fp.FL_DEPTH16 | fp.FL_CMASK(0x9),
                 (fp.c16(0.1), fp.c16(0.2, 5), fp.c16(0.3, 6), fp.c16(1.0)), clear_depth=0x123456, clear_stencil=0x7F, scissor=(1, 2, 3, 4),
                 ds_handle=0xC052, stencil_mask=0xA5)
        bd.present(0xC050, frame_id=78, hold=True, wait=False)
        bd.fence()
    # fp24 edge cases through a uniform
    bd.vs_uniforms(0, [(0.0, -0.0, 1.0, -1.0), (1e30, -1e30, 1e-30, -1e-30), (1.0000152587890625, 1.0000228881835938, 0.1, 3.141592653589793),
                       (float('inf'), float('-inf'), float('nan'), 5.421010862427522e-20)])


def main():
    bd = RecordingBoard(sys.argv[1])
    cube_scene(bd, 800, 600, 32, True, 3)
    cube_scene(bd, 640, 480, 16, False, 3)
    textures(bd)
    two_d(bd)
    bd.f.close()
    print("recorded %s: next seq %d" % (sys.argv[1], bd.next_seq))


if __name__ == '__main__':
    main()
