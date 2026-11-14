/* g2_parity — the C half of the parity test: the scenes of record_py.py through gpu2core
 * into a .dgram file; parity.ps1 compares it with Python's byte for byte.   g2_parity OUT.dgram */
#include <stdio.h>
#include "g2_cube_scene.h"

/* fb_texperf.block_colour / level_rgba / level_565 */
static void block_colour(uint32_t ub, uint32_t vb, uint32_t level, uint32_t c[3])
{
    uint32_t tint = level * 8 < 120 ? level * 8 : 120;
    c[0] = (ub * 8) & 0xFF;
    c[1] = (vb * 8) & 0xFF;
    c[2] = ((ub ^ vb) & 1) * 128 + tint;
}

static void *level_pixels(uint32_t size, uint32_t level, int rgb565)
{
    uint8_t *out = (uint8_t *)malloc((size_t)size * size * (rgb565 ? 2 : 4)), *p = out;
    uint32_t x, y, c[3];
    for (y = 0; y < size; y++) {
        uint32_t vb = (y * 32) / size;
        for (x = 0; x < size; x++) {
            block_colour((x * 32) / size, vb, level, c);
            if (rgb565) {
                uint32_t v = ((c[0] * 31 * 2 + 255) / 510) << 11 | ((c[1] * 63 * 2 + 255) / 510) << 5 | ((c[2] * 31 * 2 + 255) / 510);
                *p++ = (uint8_t)v; *p++ = (uint8_t)(v >> 8);
            } else {
                *p++ = (uint8_t)c[0]; *p++ = (uint8_t)c[1]; *p++ = (uint8_t)c[2]; *p++ = 255;
            }
        }
    }
    return out;
}

static void upload_texture(g2_board *bd, uint16_t handle, uint32_t base, uint32_t size, uint32_t fmt, uint32_t levels, uint32_t layers)
{
    uint32_t bpp = g2_storage_bpp(fmt), layer, lv;
    g2_create_image(bd, handle, base, size, size, fmt, 0, 0, levels, layers, 0, G2_USE_SAMPLED | G2_USE_TRANSFER);
    for (layer = 0; layer < layers; layer++) {
        for (lv = 0; lv < levels; lv++) {
            uint32_t s = size >> lv ? size >> lv : 1;
            void *px = level_pixels(s, lv + layer * 2, fmt == G2_FMT_RGB565);
            g2_write_image(bd, handle, px, (size_t)s * s * bpp, fmt == G2_FMT_RGBA8 ? G2_TF_RGBA8888 : G2_TF_RGB565, 0, 0, s, s, lv, layer, 0, fmt);
            free(px);
        }
    }
}

static void two_d(g2_board *bd)
{
    static uint8_t pix[256 * 4 * 16], out[1 << 21];
    static uint32_t cur[4096];
    uint32_t pal[3] = { 0x112233, 0x445566, 0xFFFFFF }, i, cc[4], sc[4] = { 1, 2, 3, 4 }, h[23], uni[12], u4[16], c1[4], dummy;
    int k;
    double vp[4] = { 400.5 * 8, 8.0, -300.25 * 8, 2400.0 };
    for (i = 0; i < sizeof pix; i++) pix[i] = (uint8_t)i;
    g2_create_image(bd, 0xC050, 0x2A000000u, 256, 16, G2_FMT_RGBA8, 1024, 0, 1, 1, 0, G2_USE_SAMPLED | G2_USE_SCANOUT | G2_USE_TRANSFER);
    g2_write_image(bd, 0xC050, pix, sizeof pix, G2_TF_RGBA8888, 0, 0, 256, 16, 0, 0, 0, G2_FMT_RGBA8);
    g2_write_image(bd, 0xC050, pix, 300, G2_TF_RGB888, 3, 4, 10, 10, 0, 0, 7, G2_FMT_RGBA8);
    g2_write_image(bd, 0xC050, pix, 64, G2_TF_RAW, 0, 0, 8, 2, 0, 0, 0, G2_FMT_RGB565);
    g2_fill_image(bd, 0xC050, G2_TF_RGBA8888, 0x80FF4020u, 10, 10, 100, 60, 0xF, 0, 0);
    g2_fill_image(bd, 0xC050, G2_TF_RAW, 0xAB, 100, 100, 60, 40, 0x1, 1, 2);
    g2_copy_image(bd, 0xC050, 0xC051, 0, 0, 120, 80, 5, 5, 60, 40, 0, 1, 1, 0x00000000u, 0x80FFFF00u, 0, 0, 0, 0);
    g2_copy_image(bd, 0xC050, 0xC051, 20, 20, 30, 20, 100, 50, 30, 20, 1, 0, 0, 0, 0, 0, 0, 0, 0);
    g2_read_image(bd, 0xC050, G2_TF_RGBA8888, 0, 0, 800, 600, 0, 0, G2_FMT_RGBA8, 0, 0, out);
    g2_read_image(bd, 0xC050, G2_TF_BGR888, 10, 7, 100, 50, 0, 0, G2_FMT_RGBA8, 1000, 777, out);
    g2_mem_read(bd, 0x30000000u, out, 1u << 20, 1);
    g2_read_buffer(bd, 0x0021, 16, out, 300000);
    g2_mem_write(bd, 0x30000000u, pix, 3000, 1);
    g2_memtest(bd, 0x20000000u, 1u << 20, 0x5EED1234u, 3, 1, 0.0, &dummy, &dummy);
    g2_set_mode(bd, 2, 5.0);
    g2_set_palette(bd, 4, pal, 3);
    for (i = 0; i < 4096; i++) cur[i] = i * 0x01020304u;
    g2_cursor_shape(bd, cur, 4096);
    g2_cursor_show(bd, G2_CUR_ANDXOR, 32, 16, 1, 100, 200, 1);
    g2_cursor_show(bd, G2_CUR_COLOR, 0, 0, 0, 0, 0, 0);
    g2_cursor_move(bd, -5, 7);
    g2_cursor_pos(bd, 300, -2);
    g2_cursor_hide(bd);
    g2_create_image(bd, 0xC052, 0x2B000000u, 64, 64, G2_FMT_D24S8, 256, 0, 1, 1, 0, G2_USE_RT);
    g2_destroy(bd, 0xC052);
    g2_present(bd, 0xC050, 77, 1, 0);
    g2_batch_begin(bd);
    g2_frame_begin(bd);
    for (k = 0; k < 300; k++) {
        put_vec4(u4, (double)k, 0.5, -2.0, 1e-3);
        g2_vs_uniforms(bd, (uint32_t)(k & 7), u4, 1);
        g2_draw(bd, k % 7, (uint32_t)(3 + k), (uint32_t)k, !(k & 1), 0x22, (uint32_t)(k * 2), (k & 2) != 0);
    }
    c1[0] = 0; c1[1] = 0; c1[2] = 0; c1[3] = g2_fp24(1.0);
    g2_vertex_attr(bd, 5, 1, 4, 0x21, 16, 12, c1, G2_AT_UBYTE, 1);
    g2_vs_state(bd, vp, 3, 1, 0xFF, 1280, 800, 8191, 63.6, 0.0, 0.3, NULL);
    for (i = 0; i < 23; i++) h[i] = i * 0x01010101u;
    for (i = 0; i < 3; i++) put_vec4(uni + 4 * i, 1.0, 2.0, 3.0, 4.0);
    {
        uint32_t uu[12];
        memcpy(uu, uni, sizeof uu);
        g2_draw_state(bd, 8191, h, uu, 3);
    }
    cc[0] = g2_c16(0.1, 8); cc[1] = g2_c16(0.2, 5); cc[2] = g2_c16(0.3, 6); cc[3] = g2_c16(1.0, 8);
    g2_flush(bd, 0xC050, G2_FL_CLEAR_COLOR | G2_FL_CLEAR_SCISSOR | G2_FL_DITHER | G2_FL_CLEAR_DEPTH | G2_FL_CLEAR_STENCIL | G2_FL_DEPTH16 | G2_FL_CMASK(0x9),
             cc, 0x123456, 0x7F, sc, 0xC052, 0xA5);
    g2_present(bd, 0xC050, 78, 0, 1);
    g2_fence(bd, 1.0);
    g2_batch_end(bd);
    {
        uint32_t e[16];
        put_vec4(e, 0.0, -0.0, 1.0, -1.0);
        put_vec4(e + 4, 1e30, -1e30, 1e-30, -1e-30);
        put_vec4(e + 8, 1.0000152587890625, 1.0000228881835938, 0.1, 3.141592653589793);
        put_vec4(e + 12, INFINITY, -INFINITY, NAN, 5.421010862427522e-20);
        g2_vs_uniforms(bd, 0, e, 4);
    }
}

static void cube_scene(g2_board *bd, uint32_t W, uint32_t H, int ncubes, int instanced, int frames)
{
    scene_t s;
    uint32_t nidx;
    int i;
    scene_init(&s, ncubes, (double)W / (double)H, 1.0);
    nidx = scene_setup(bd, W, H, &s, instanced);
    for (i = 0; i < frames; i++) scene_frame(bd, i, W, H, &s, instanced, 1.0, nidx, 1);
    scene_free(&s);
}

int main(int argc, char **argv)
{
    g2_options o;
    g2_board *bd;
    g2_counters c;
    memset(&o, 0, sizeof o);
    o.b_sub_index = 1;
    o.record_path = argc > 1 ? argv[1] : "out.dgram";
    bd = g2_open(NULL, 0, &o);
    if (!bd) return 1;
    cube_scene(bd, 800, 600, 32, 1, 3);
    cube_scene(bd, 640, 480, 16, 0, 3);
    upload_texture(bd, 0xC040, 0x31000000u, 2048, G2_FMT_RGBA8, 12, 1);
    upload_texture(bd, 0xC041, 0x32000000u, 256, G2_FMT_RGB565, 1, 1);
    upload_texture(bd, 0xC042, 0x32400000u, 64, G2_FMT_RGBA8, 7, 6);
    two_d(bd);
    g2_get_counters(bd, &c);
    printf("recorded %s: next seq %u\n", o.record_path, c.next_seq);
    g2_close(bd);
    return 0;
}
