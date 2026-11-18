/* libGLESv2.dll — OpenGL ES 2.0 and EGL on the two-board GPU. The hardware's native API: every ES 2.0 feature maps onto g2_render, the work here is the object model.
 *
 * Shaders compile at glCompileShader (for the log) and again at link, the fragment stage with the vertex stage's varying order: GLSL matches varyings by name, the compiler numbers them by declaration, the hardware by number.
 * Storage is the real one for the ES formats (RGBA8, RGB565, RGBA4444, RGBA5551); RGB / L / A / LA expand to RGBA8 at upload, as the TMU samples no 8-bit format. DXT and ETC1 decode on the host to RGBA8:
 * ETC1's texels have 8 bits a channel, which RGB565 would not keep.
 * A 3D texture is a volume of the TMU on a board B that has them, in the same storage. GL_OES_texture_3D is not advertised on another B.
 * The TMU's descriptor has no pitch, so a texture has tight rows; B renders into a pitch that is a multiple of 32 and into one layer. A texture that cannot be both renders into a second image, copied across before it is sampled.
 * B's WRITE_IMAGE / READ_IMAGE move 4444 and 5551 RAW only, so every conversion to or from those is done here. Level 0 creates the image on B; a level that does not fit its chain waits in host memory (es_pend).
 * B renders at most 1 280 wide and scans out at most 1 280 x 800. A larger window (a fullscreen game on a big desktop) is rendered smaller: the application goes on believing in its window, and viewport, scissor and reads scale.
 * Not scaled: gl_FragCoord and gl_PointSize, which a shader computes in its own window coordinates.
 * libEGL.dll only forwards to the egl* exports here; exports keep ANGLE's ordinals, because games linked against ANGLE import by ordinal (gen_exports.py).
 */
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#define WIN32_LEAN_AND_MEAN
#define _GDI32_
#define GL_GLEXT_PROTOTYPES
#include <windows.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "g2_dxt.h"
#include "g2_env.h"
#include "g2_render.h"
#include "g2_shc.h"

#ifndef GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS
#define GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS 0x8CD9
#endif

#define MAX_ATTR 16
#define MAX_UNITS 8
#define GL_WRITE_ONLY_OES 0x88B9
#define GL_BUFFER_ACCESS_OES 0x88BB
#define GL_BUFFER_MAPPED_OES 0x88BC
#define GL_BUFFER_MAP_POINTER_OES 0x88BD
#define GL_FRAGMENT_SHADER_DERIVATIVE_HINT_OES 0x8B8B
#define GL_VERTEX_ARRAY_BINDING_OES 0x85B5
#define GL_ETC1_RGB8_OES 0x8D64
#define GL_DEPTH_STENCIL_OES 0x84F9
#define GL_UNSIGNED_INT_24_8_OES 0x84FA
#define GL_DEPTH_COMPONENT32_OES 0x81A7
#define GL_TEXTURE_3D_OES 0x806F
#define GL_TEXTURE_BINDING_3D_OES 0x806A
#define GL_TEXTURE_WRAP_R_OES 0x8072
#define GL_MAX_3D_TEXTURE_SIZE_OES 0x8073
#define GL_SAMPLER_3D_OES 0x8B5F
#define GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_3D_ZOFFSET_OES 0x8CD4
#define GL_COLOR_EXT 0x1800
#define GL_DEPTH_EXT 0x1801
#define GL_STENCIL_EXT 0x1802
#define MAX_LEVELS 12
#define MAX_TEX 2048
#define MAX_3D 128                              /* GL_MAX_3D_TEXTURE_SIZE_OES: what all three dimensions reach, the TMU's depth stops at 255. A volume may be up to G2R_VOL_WH x G2R_VOL_WH x G2R_VOL_D */
#define MAX_RT_W 1280
#define MAX_SCAN_H 800
#define MAX_VIEWPORT 4096
#define MAX_UNI 512
#define HASH 1024
#define NOFMT 0xFFFFFFFFu
#define MASK_A 8                                /* FILL_IMAGE's channel mask */

#define ES_APIENTRY APIENTRY
/* the extensions ANGLE exports, past what glext.h knows */
#define GL_READ_FRAMEBUFFER_ANGLE 0x8CA8
#define GL_DRAW_FRAMEBUFFER_ANGLE 0x8CA9
#define GL_READ_FRAMEBUFFER_BINDING_ANGLE 0x8CAA
#define GL_VERTEX_ATTRIB_ARRAY_DIVISOR_ANGLE 0x88FE
#define GL_TRANSLATED_SHADER_SOURCE_LENGTH_ANGLE 0x93A0
#define GL_TEXTURE_IMMUTABLE_FORMAT_EXT 0x912F
#define GL_BGRA8_EXT 0x93A1
#ifndef GL_UNSIGNED_SHORT_4_4_4_4_REV_EXT
#define GL_UNSIGNED_SHORT_4_4_4_4_REV_EXT 0x8365
#endif
#ifndef GL_UNSIGNED_SHORT_1_5_5_5_REV_EXT
#define GL_UNSIGNED_SHORT_1_5_5_5_REV_EXT 0x8366
#endif
#define GL_LUMINANCE8_EXT 0x8040
#define GL_ALPHA8_EXT 0x803C
#define GL_LUMINANCE8_ALPHA8_EXT 0x8045
#define GL_ALL_COMPLETED_NV 0x84F2
#define GL_FENCE_STATUS_NV 0x84F3
#define GL_FENCE_CONDITION_NV 0x84F4
#define GL_PROGRAM_BINARY_LENGTH_OES 0x8741
#define GL_NUM_PROGRAM_BINARY_FORMATS_OES 0x87FE
#define GL_PROGRAM_BINARY_FORMATS_OES 0x87FF
#define G2_PROGRAM_BINARY 0x47325042u                   /* "G2PB": everything here compiles at link, so a program's binary is its two sources and its attribute bindings */
#define SET_ERR(e) do { if (S.error == GL_NO_ERROR) S.error = (e); } while (0)

typedef struct es_buf { struct es_buf *next; GLuint name; uint8_t *data; GLsizeiptr size; GLenum usage; g2r_kept keep; int mapped; } es_buf;

/* a level that is not (yet) part of the chain on B: uploaded before level 0, or of another size or format than the chain */
typedef struct es_pend { uint8_t *px; uint32_t fmt, etf; int ebpp; } es_pend;

typedef struct es_tex {
    struct es_tex *next;
    GLuint name;
    GLenum target;
    int w, h, d, levels, layers;                /* the image on B; d: a volume's depth */
    uint16_t handle;
    uint32_t base, g2fmt;
    GLenum format;                              /* level 0's client format */
    int compressed;
    uint32_t lkey[6][MAX_LEVELS];               /* format | type << 16, 0: not defined */
    int lw[6][MAX_LEVELS], lh[6][MAX_LEVELS], ld[MAX_LEVELS];
    es_pend *pend[6][MAX_LEVELS];
    GLenum min_f, mag_f, wrap_s, wrap_t, wrap_r;
    uint32_t used_pass;
    int immutable;                              /* EXT_texture_storage: its levels; 0: an ordinary texture */
    uint16_t rt;                                /* the render image, when this one cannot be rendered to */
    uint32_t rt_base;
    int rt_face, rt_level, rt_w, rt_h, rt_dirty, rt_stale;  /* dirty: rendered, the texture lags; stale: the texture changed, rt lags */
    uint16_t rt_ds, ds_alias;                   /* a depth texture as a depth-stencil target: rt's memory, or its own */
    int att_refs, orphan;                       /* framebuffer attachments that hold it; orphan: its name was deleted, it lives until the last one lets go */
} es_tex;

typedef struct es_rb { struct es_rb *next; GLuint name; GLenum ifmt; int w, h; uint16_t handle; uint32_t base, g2fmt; int att_refs, orphan; } es_rb;
/* an attachment holds the object, not its name: a texture or renderbuffer deleted while attached to a framebuffer that is not bound stays that framebuffer's image, and a new object under the old
   name is another one. face: the cube map face (or GL_TEXTURE_2D); lvl: the level; z: the slice of a 3D texture */
typedef struct es_att { GLenum type; GLuint name; struct es_tex *tex; es_rb *rb; GLenum face; int lvl, z; } es_att;
enum { ATT_COLOR, ATT_DEPTH, ATT_STENCIL };
typedef struct es_fb { struct es_fb *next; GLuint name; es_att a[3]; uint16_t blind; uint32_t blind_base; int blind_w, blind_h; } es_fb;     /* blind: a framebuffer without colour, B has no colourless pass */
static int att_index(GLenum a) { return a == GL_DEPTH_ATTACHMENT ? ATT_DEPTH : a == GL_STENCIL_ATTACHMENT ? ATT_STENCIL : ATT_COLOR; }

typedef struct es_shader {
    struct es_shader *next;
    GLuint name;
    GLenum type;
    char *src, *compiled_src;                   /* a link uses what was compiled, not the latest glShaderSource */
    int compiled, del, refs;
    char log[600];
} es_shader;

typedef struct es_uni {
    char name[96];
    GLenum type;
    int size;                                   /* array elements */
    int isarr;                                  /* declared as an array, also of one element: its name reads "[0]" */
    int stride, comps;                          /* slots per element, floats per slot */
    int voff;                                   /* where its values start in the program's uval */
    int vs, fs;                                 /* base slot, -1: not in that stage */
    int sampler;                                /* first sampler slot, -1: not a sampler */
    int vc, fc;                                 /* the lane it starts at in each stage, when a link packed it */
} es_uni;

/* an active attribute: loc is its location, the generic attribute it reads, and the vertex core's input the program reads it from; n: locations, a matrix one a column */
typedef struct es_attr { char name[64]; int loc, n; GLenum type; } es_attr;
typedef struct es_bind { char name[64]; GLuint loc; } es_bind;

typedef struct es_prog {
    struct es_prog *next;
    GLuint name;
    GLuint vs, fs;
    int linked, validated, del;
    char *bin_vs, *bin_fs;                      /* what the last link compiled: OES_get_program_binary, as a shader may be gone by then */
    char log[800];
    g2r_prog *p;
    es_uni *uni; int nuni;
    uint32_t *uval;                             /* every uniform's value as the application gave it, 32 bits a component: float bits, or an int (int, bool, sampler); glGetUniform reads these */
    es_attr *attr;                              /* the active attributes; two may share a location (§2.10.4 aliasing), so there can be more than locations */
    int nattr;
    struct { uint32_t mask; g2r_prog *p; } dv[8];  /* the program with these samplers reading depth textures */
    int ndv, dv_next;
    uint32_t dnow;                              /* the samplers that do at this draw */
    es_bind bind[32];                           /* glBindAttribLocation's */
    int nbind;
    es_bind lbind[32];                          /* the bindings the last link used: a link again for depth sampling takes these, later ones wait for glLinkProgram */
    int nlbind;
    int samp_unit[MAX_UNITS], nsamp;
    int dr_vs, dr_fs;            /* gl_DepthRange's first slot in each stage, -1 if the stage does not read it */
    int ts[MAX_UNITS];           /* the slot of the texture size a unit's texture*GradEXT reads, -1 for none */
    uint32_t vu[4 * 256], fu[4 * 64];
} es_prog;

typedef struct es_va { int enabled, size, normalized; GLenum type; GLsizei stride; const void *ptr; GLuint buffer; float cur[4]; GLuint divisor; } es_va;
typedef struct es_vao { struct es_vao *next; GLuint name; es_va va[MAX_ATTR]; GLuint elem_buf; } es_vao;

/* what a draw renders into */
typedef struct es_tg {
    g2r_target t;
    int has_d, has_s, no_alpha, face, level;
    uint32_t fmt;
    es_tex *tex, *dtex;                         /* the colour attachment's texture, the depth attachment's */
    es_fb *fb;
    int vw, vh;                                 /* its size as the application sees it */
    double kx, ky;                              /* the target's pixels per application pixel: 1 except for a window larger than the hardware */
} es_tg;

static struct {
    g2_board *bd;
    g2r *r;
    /* window system */
    HWND hwnd;
    int vw, vh, fb_w, fb_h, mode, mode_w, mode_h, back, front, scan_idx, hold, stretch, swap_interval;      /* vw, vh: the window; fb_w, fb_h: what is rendered */
    uint32_t frame;
    uint16_t color[3], ds, scan[3];
    uint32_t color_base[3], ds_base, scan_base[3];
    int cfg, depth16, stencil;              /* the surface's EGL config: its depth has 16 bits, it has stencil */
    uint32_t color_fmt;                     /* and its colour: RGBA8 or RGB565 */
    /* objects: shared by all contexts, names never repeat */
    es_buf *bufs[HASH];
    es_tex *texs[HASH], *tex_default[3];
    es_rb *rbs[HASH];
    es_fb *fbs[HASH];
    es_shader *shaders[HASH];
    es_prog *progs[HASH];
    GLuint next_name;
    char ctx_begin;                             /* from here to ctx_end: one context's state, swapped by eglMakeCurrent */
    GLenum error;
    int vp_set, tex_storing;
    GLuint array_buf, elem_buf, fb, read_fb, rb;
    GLuint vao; es_vao vao0; es_vao *vaos;     /* GL_OES_vertex_array_object: the bound name, the default object while another is bound, the rest */
    GLenum hint_deriv;
    int active, draw_none;
    GLuint tex2d[MAX_UNITS], cube[MAX_UNITS], tex3d[MAX_UNITS];
    es_prog *cur;
    es_va va[MAX_ATTR];
    int depth_test, stencil_test, blend, cull, scissor_test, dither, po_fill, sample_cov, sample_a2c, sample_inv;
    GLenum depth_func, cull_mode, front_face;
    GLenum b_src, b_dst, b_src_a, b_dst_a, b_eq, b_eq_a;
    float b_color[4], clear_color[4], clear_depth, depth_n, depth_f, po_factor, po_units, line_width, sample_val;
    int depth_mask, color_mask, clear_stencil;
    GLenum st_func[2], st_fail[2], st_zfail[2], st_zpass[2];
    int st_ref[2];
    GLuint st_vmask[2], st_wmask[2];
    int vp[4], sc[4];
    int un_align, pk_align;
    GLenum hint_mip;
    char ctx_end;
    int live;
} S;

/* ---------------------------------------------------------------- object tables */
#define FIND(tab, nm, ty) ty *o; for (o = S.tab[(nm) % HASH]; o; o = o->next) if (o->name == (nm)) break
static GLuint gen_name(void) { return ++S.next_name; }
static es_buf *buf_of(GLuint n) { FIND(bufs, n, es_buf); return o; }
static es_tex *tex_of(GLuint n) { FIND(texs, n, es_tex); return o; }
static es_rb *rb_of(GLuint n) { FIND(rbs, n, es_rb); return o; }
static es_fb *fb_of(GLuint n) { FIND(fbs, n, es_fb); return o; }
static es_shader *sh_of(GLuint n) { FIND(shaders, n, es_shader); return o; }
static es_prog *pr_of(GLuint n) { FIND(progs, n, es_prog); return o; }
#define NEW(tab, nm, ty, out) do { ty *x_ = (ty *)calloc(1, sizeof(ty)); if (x_) { x_->name = (nm); x_->next = S.tab[(nm) % HASH]; S.tab[(nm) % HASH] = x_; } (out) = x_; } while (0)
#define DEL(tab, nm, ty, on) do { ty **pp_ = &S.tab[(nm) % HASH]; while (*pp_) { ty *x_ = *pp_; if (x_->name == (nm)) { *pp_ = x_->next; on; free(x_); break; } pp_ = &x_->next; } } while (0)
/* the object out of its name's table, not freed; NULL: no object has the name */
#define UNLINK(tab, nm, ty, out) do { ty **pp_ = &S.tab[(nm) % HASH]; (out) = NULL; while (*pp_) { if ((*pp_)->name == (nm)) { (out) = *pp_; *pp_ = (out)->next; break; } pp_ = &(*pp_)->next; } } while (0)

/* §2.10.1: shaders and programs share one name space. A name of the other kind is INVALID_OPERATION, a name of neither INVALID_VALUE */
static es_shader *shader_arg(GLuint n) { es_shader *s = sh_of(n); if (!s) SET_ERR(pr_of(n) ? GL_INVALID_OPERATION : GL_INVALID_VALUE); return s; }
static es_prog *prog_arg(GLuint n) { es_prog *p = pr_of(n); if (!p) SET_ERR(sh_of(n) ? GL_INVALID_OPERATION : GL_INVALID_VALUE); return p; }

/* ---------------------------------------------------------------- the default framebuffer */
static int dev_up(void)
{
    uint32_t st[G2_ST_WORDS];
    int n = 0;
    if (S.r) return 0;
    g2e_crash_handler();
    S.bd = g2e_connect("gles");
    if (!S.bd) return -1;
    S.r = g2r_create(S.bd);
    if (!S.r) return -1;
    S.hold = g2e_get_int("GPU2_HOLD", 1);
    S.stretch = !strcmp(g2e_get("GPU2_SCALE", "fit"), "stretch");
    if (g2_status(S.bd, st, &n) >= 0) {
        S.mode = g2_mode_of_flags(st[G2_ST_B_FLAGS]);
        g2sc_set_target(G2_B_FLAG_XOPS(st[G2_ST_B_FLAGS]));    /* the fragment-core operations this board B has: the compiler emits them */
        if (g2sc_target()) g2e_log("gles: board B runs SQRT / RCPM / RSQM %s, CLAMP %s", g2sc_target() & 1 ? "yes" : "no", g2sc_target() & 2 ? "yes" : "no");
        g2sc_set_absent(G2_A_FLAG_ABSENT(st[G2_ST_FLAGS]), G2_B_FLAG_ABSENT(st[G2_ST_B_FLAGS]));     /* the units the cores lack: the compiler replaces their ops */
        if (g2sc_absent(G2SC_VS) | g2sc_absent(G2SC_FS)) g2e_log("gles: units absent (1 integer divider, 2 SIN / COS): vertex core %u, fragment core %u", g2sc_absent(G2SC_VS), g2sc_absent(G2SC_FS));
    }
    S.mode_w = G2_MODE_W[S.mode]; S.mode_h = G2_MODE_H[S.mode];
    return 0;
}

/* EGL configs (eglChooseConfig): colour bits, depth bits, stencil bits */
static const struct { int color, depth, stencil; } CFG[] = { { 32, 24, 8 }, { 32, 0, 0 }, { 32, 16, 0 }, { 16, 24, 8 }, { 16, 16, 0 }, { 16, 0, 0 } };
#define NCFG ((int)(sizeof CFG / sizeof CFG[0]))

static void default_fb(int vw, int vh, int cfg, int window)
{
    int i, w, h, maxh = window ? MAX_SCAN_H : MAX_TEX, maxw = g2e_get_int("GPU2_MAX_W", MAX_RT_W);    /* a lower GPU2_MAX_W trades resolution for speed: fill rate is what limits most games */
    int depth = CFG[cfg - 1].depth != 0;
    if (vw < 1) vw = 640;
    if (vh < 1) vh = 480;
    if (vw == S.vw && vh == S.vh && S.color[0] && S.cfg == cfg) return;
    S.cfg = cfg; S.depth16 = CFG[cfg - 1].depth == 16; S.stencil = CFG[cfg - 1].stencil != 0;
    S.color_fmt = CFG[cfg - 1].color == 16 ? G2_FMT_RGB565 : G2_FMT_RGBA8;
    if (maxw > MAX_RT_W || maxw < 1) maxw = MAX_RT_W;
    w = vw; h = vh;
    if (w > maxw || h > maxh) {
        double s = (double)maxw / w;
        if ((double)maxh / h < s) s = (double)maxh / h;
        w = (int)(w * s + 0.5); h = (int)(h * s + 0.5);
        if (w > maxw) w = maxw;
        if (h > maxh) h = maxh;
    }
    g2r_end_pass(S.r);
    for (i = 0; i < 3; i++) {
        if (S.color[i]) g2r_image_destroy(S.r, S.color[i], S.color_base[i]);
        if (S.scan[i]) g2r_image_destroy(S.r, S.scan[i], S.scan_base[i]);
        S.color[i] = S.scan[i] = 0;
    }
    if (S.ds) g2r_image_destroy(S.r, S.ds, S.ds_base);
    S.ds = 0;
    if (window) {                               /* the smallest mode that holds the window */
        if (g2_mode_select(S.bd, w, h, &S.mode) < 0) g2e_log("gles: SET_MODE for %d x %d: %s", w, h, g2_error_text(S.bd));
        S.mode_w = G2_MODE_W[S.mode]; S.mode_h = G2_MODE_H[S.mode];
    }
    for (i = 0; i < 3; i++) {
        if (g2r_image_create(S.r, &S.color[i], &S.color_base[i], (uint32_t)w, (uint32_t)h, S.color_fmt, 1, 1, 1, G2_USE_RT | G2_USE_SCANOUT | G2_USE_TRANSFER | G2_USE_SAMPLED) < 0)
            g2e_log("gles: no room for the framebuffer");
    }
    if (depth) {
        g2r_image_create(S.r, &S.ds, &S.ds_base, (uint32_t)w, (uint32_t)h, G2_FMT_D24S8, 1, 1, 1, G2_USE_RT | G2_USE_TRANSFER);
        g2r_stencil_undefined(S.r, S.ds);
    }
    if (w != S.mode_w || h != S.mode_h) {
        for (i = 0; i < 3; i++) {
            g2r_image_create(S.r, &S.scan[i], &S.scan_base[i], (uint32_t)S.mode_w, (uint32_t)S.mode_h, G2_FMT_RGBA8, 1, 1, 1, G2_USE_SCANOUT | G2_USE_TRANSFER);
            g2_fill_image(S.bd, S.scan[i], G2_TF_RAW, 0x000000FFu, 0, 0, (uint32_t)S.mode_w, (uint32_t)S.mode_h, 0xF, 0, 0);
        }
    }
    S.vw = vw; S.vh = vh; S.fb_w = w; S.fb_h = h; S.back = S.front = 0;
    g2e_log("gles: window %d x %d, framebuffer %d x %d, colour %d depth %d stencil %d, HDMI mode %d", vw, vh, w, h, CFG[cfg - 1].color, CFG[cfg - 1].depth, CFG[cfg - 1].stencil, S.mode);
}

static void win_size(HWND w, int *x, int *y)
{
    RECT r;
    *x = 640; *y = 480;
    if (w && GetClientRect(w, &r) && r.right > 0 && r.bottom > 0) { *x = r.right; *y = r.bottom; }
}

static void dump_frame(uint16_t image)
{
    static int every = -1;
    char path[MAX_PATH];
    uint8_t *px;
    FILE *f;
    if (!g2e_get("GPU2_DUMP", NULL)) return;                    /* frames to files, to check them without the monitor */
    if (every < 0) every = g2e_get_int("GPU2_DUMP_EVERY", 120);
    if (S.frame % (uint32_t)every != 1) return;
    snprintf(path, sizeof path, "%s\\frame_%05u.bmp", g2e_get("GPU2_DUMP", ""), S.frame);
    px = (uint8_t *)malloc((size_t)S.fb_w * S.fb_h * 4);
    f = fopen(path, "wb");
    if (px && f) {
        BITMAPFILEHEADER fh;
        BITMAPINFOHEADER ih;
        g2_fence(S.bd, 5.0);
        g2_read_image(S.bd, image, G2_TF_BGRA8888, 0, 0, (uint32_t)S.fb_w, (uint32_t)S.fb_h, 0, 0, S.color_fmt, 0, 0, px);
        memset(&fh, 0, sizeof fh); memset(&ih, 0, sizeof ih);
        fh.bfType = 0x4D42; fh.bfOffBits = sizeof fh + sizeof ih; fh.bfSize = fh.bfOffBits + (DWORD)S.fb_w * S.fb_h * 4;
        ih.biSize = sizeof ih; ih.biWidth = S.fb_w; ih.biHeight = S.fb_h; ih.biPlanes = 1; ih.biBitCount = 32; ih.biCompression = BI_RGB;     /* bottom-up, as GL */
        fwrite(&fh, sizeof fh, 1, f); fwrite(&ih, sizeof ih, 1, f); fwrite(px, 4, (size_t)S.fb_w * S.fb_h, f);
    }
    if (f) fclose(f);
    free(px);
}

static void swap_buffers(void)
{
    uint16_t image = S.color[S.back];
    int hold = S.hold && S.swap_interval, w, h;
    g2r_end_pass(S.r);
    if (S.fb_w == S.mode_w && S.fb_h == S.mode_h) {
        g2r_present(S.r, image, hold);
    } else {
        int dw = S.mode_w, dh = S.mode_h, dx = 0, dy = 0;
        if (!S.stretch) {
            int sx = S.mode_w / S.fb_w, sy = S.mode_h / S.fb_h, s = sx < sy ? sx : sy;
            if (s < 1) s = 1;
            dw = S.fb_w * s; dh = S.fb_h * s;
            if (dw > S.mode_w) dw = S.mode_w;
            if (dh > S.mode_h) dh = S.mode_h;
            dx = (S.mode_w - dw) / 2; dy = (S.mode_h - dh) / 2;
        }
        S.scan_idx = (S.scan_idx + 1) % 3;
        g2_copy_image(S.bd, image, S.scan[S.scan_idx], 0, 0, (uint32_t)S.fb_w, (uint32_t)S.fb_h, (uint32_t)dx, (uint32_t)dy, (uint32_t)dw, (uint32_t)dh, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        g2r_present(S.r, S.scan[S.scan_idx], hold);
    }
    S.frame++;
    g2r_stats_tick(S.r);
    dump_frame(image);
    S.front = S.back;
    S.back = (S.back + 1) % 3;
    /* EGL_BUFFER_DESTROYED: the colour buffer is undefined after a swap, which spares the next frame a band load */
    g2r_discard_color(S.r, S.color[S.back]);
    if (S.hwnd) {
        win_size(S.hwnd, &w, &h);
        if (w != S.vw || h != S.vh) default_fb(w, h, S.cfg, 1);
    }
}

static int mip_dim(int d, int l);
static int is_face(GLenum t);

static int rb_bits(uint32_t fmt, int c)
{
    static const uint8_t b[4][4] = { { 8, 8, 8, 8 }, { 5, 6, 5, 0 }, { 4, 4, 4, 4 }, { 5, 5, 5, 1 } };
    return b[fmt == G2_FMT_RGB565 ? 1 : fmt == G2_FMT_RGBA4444 ? 2 : fmt == G2_FMT_RGBA5551 ? 3 : 0][c];
}

static int depth_rb(GLenum f) { return f == GL_DEPTH_COMPONENT16 || f == GL_DEPTH_COMPONENT24 || f == GL_DEPTH24_STENCIL8; }
static int stencil_rb(GLenum f) { return f == GL_STENCIL_INDEX8 || f == GL_DEPTH24_STENCIL8; }

/* §4.4.5. B renders level 0 of a single layer, <= 1 280 wide, with a pitch that is a multiple of 32; depth and stencil share one word, so they cannot be two images (FRAMEBUFFER_UNSUPPORTED is the spec's answer for such a combination) */
static GLenum fb_target(es_tg *g)
{
    es_fb *f = S.fb ? fb_of(S.fb) : NULL;
    const es_att *c, *da, *sa;
    GLenum unsupported = 0;
    memset(g, 0, sizeof *g);
    g->fmt = G2_FMT_RGBA8;
    if (!f) {
        g->fmt = S.color_fmt;
        g->t.color = S.color[S.back]; g->t.ds = S.ds;
        g->t.width = (uint32_t)S.fb_w; g->t.height = (uint32_t)S.fb_h;
        g->t.bits_r = rb_bits(g->fmt, 0); g->t.bits_g = rb_bits(g->fmt, 1); g->t.bits_b = rb_bits(g->fmt, 2); g->t.bits_a = rb_bits(g->fmt, 3);
        g->t.depth16 = S.depth16;
        g->has_d = S.ds != 0; g->has_s = S.ds != 0 && S.stencil;
        g->vw = S.vw; g->vh = S.vh;
        g->kx = (double)S.fb_w / S.vw; g->ky = (double)S.fb_h / S.vh;
        return GL_FRAMEBUFFER_COMPLETE;
    }
    c = &f->a[ATT_COLOR]; da = &f->a[ATT_DEPTH]; sa = &f->a[ATT_STENCIL];
    if (!c->type && !da->type && !sa->type) return GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT;
    if (c->type == GL_TEXTURE) {
        es_tex *x = c->tex;
        int face = is_face(c->face) ? (int)(c->face - GL_TEXTURE_CUBE_MAP_POSITIVE_X) : 0;
        if (x && x->target == GL_TEXTURE_3D_OES && c->z >= x->d) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT;
        if (!x || !x->handle || !x->lkey[x->target == GL_TEXTURE_3D_OES ? 0 : face][0] || x->compressed || (x->format != GL_RGBA && x->format != GL_RGB && x->format != GL_BGRA_EXT)) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT;
        g->fmt = x->g2fmt;
        g->t.color = x->handle; g->t.width = (uint32_t)x->w; g->t.height = (uint32_t)x->h;
        g->no_alpha = x->format == GL_RGB && x->g2fmt == G2_FMT_RGBA8;
        g->tex = x;
        g->face = x->target == GL_TEXTURE_3D_OES ? c->z : face;
        g->level = c->lvl;
        g->t.width = (uint32_t)mip_dim(x->w, c->lvl); g->t.height = (uint32_t)mip_dim(x->h, c->lvl);
        if (x->lkey[x->target == GL_TEXTURE_3D_OES ? 0 : face][c->lvl] == 0) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT;
        if (g->t.width > MAX_RT_W) unsupported = GL_FRAMEBUFFER_UNSUPPORTED;
    } else if (c->type == GL_RENDERBUFFER) {
        es_rb *b = c->rb;
        if (!b || !b->handle || depth_rb(b->ifmt) || stencil_rb(b->ifmt)) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT;
        g->fmt = b->g2fmt;
        g->t.color = b->handle; g->t.width = (uint32_t)b->w; g->t.height = (uint32_t)b->h;
        g->no_alpha = b->ifmt == GL_RGB8;
    }
    g->fb = f;
    g->t.bits_r = rb_bits(g->fmt, 0); g->t.bits_g = rb_bits(g->fmt, 1); g->t.bits_b = rb_bits(g->fmt, 2); g->t.bits_a = rb_bits(g->fmt, 3);
    if (da->type == GL_TEXTURE || sa->type == GL_TEXTURE) {     /* OES_depth_texture: one word holds depth and stencil, so one texture */
        es_tex *x = da->type == GL_TEXTURE ? da->tex : sa->tex;
        if (!x || !x->handle || !x->lkey[0][0] || (x->format != GL_DEPTH_COMPONENT && x->format != GL_DEPTH_STENCIL_OES) || (sa->type == GL_TEXTURE && x->format != GL_DEPTH_STENCIL_OES)) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT;
        if ((da->type && da->type != GL_TEXTURE) || (sa->type && sa->type != GL_TEXTURE) || (da->type && sa->type && da->tex != sa->tex)) unsupported = GL_FRAMEBUFFER_UNSUPPORTED;
        if (g->t.width && ((uint32_t)x->w != g->t.width || (uint32_t)x->h != g->t.height)) return GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS;
        if (x->w > MAX_RT_W) unsupported = GL_FRAMEBUFFER_UNSUPPORTED;
        g->t.width = (uint32_t)x->w; g->t.height = (uint32_t)x->h;
        g->dtex = x;
        g->has_d = da->type != 0; g->has_s = sa->type != 0;
    } else if (da->type || sa->type) {
        es_rb *d = da->type ? da->rb : NULL, *s = sa->type ? sa->rb : NULL, *b = d ? d : s;
        if ((da->type && (!d || !d->handle || !depth_rb(d->ifmt))) || (sa->type && (!s || !s->handle || !stencil_rb(s->ifmt)))) return GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT;
        if (g->t.color && ((uint32_t)b->w != g->t.width || (uint32_t)b->h != g->t.height)) return GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS;
        if (d && s && (d != s)) unsupported = GL_FRAMEBUFFER_UNSUPPORTED;
        g->t.ds = b->handle;
        g->t.width = (uint32_t)b->w; g->t.height = (uint32_t)b->h;
        g->has_d = d != NULL; g->has_s = s != NULL;
        g->t.depth16 = d && d->ifmt == GL_DEPTH_COMPONENT16;
    }
    g->vw = (int)g->t.width; g->vh = (int)g->t.height;
    g->kx = g->ky = 1.0;
    return unsupported ? unsupported : GL_FRAMEBUFFER_COMPLETE;
}

/* the scissor box in the target's own pixels, clipped to it: x0, y0, x1, y1 */
static void scissor_box(const es_tg *g, int *b)
{
    b[0] = S.sc[0] < 0 ? 0 : (int)floor(S.sc[0] * g->kx);
    b[1] = S.sc[1] < 0 ? 0 : (int)floor(S.sc[1] * g->ky);
    b[2] = (int)ceil((S.sc[0] + S.sc[2]) * g->kx);
    b[3] = (int)ceil((S.sc[1] + S.sc[3]) * g->ky);
    if (b[2] > (int)g->t.width) b[2] = (int)g->t.width;
    if (b[3] > (int)g->t.height) b[3] = (int)g->t.height;
    if (b[2] < b[0]) b[2] = b[0];
    if (b[3] < b[1]) b[3] = b[1];
}

static int tex_needs_rt(const es_tex *t) { return t->layers > 1 || t->target == GL_TEXTURE_3D_OES || (t->w * (int)g2_storage_bpp(t->g2fmt)) % 32; }

static void tex_copy_rt(es_tex *t, int to_tex)
{
    uint32_t w = (uint32_t)t->rt_w, h = (uint32_t)t->rt_h, f = (uint32_t)t->rt_face, l = (uint32_t)t->rt_level;
    g2r_end_pass(S.r);
    if (to_tex) g2_copy_image(S.bd, t->rt, t->handle, 0, 0, w, h, 0, 0, w, h, 0, 0, 0, 0, 0, 0, 0, l, f);
    else g2_copy_image(S.bd, t->handle, t->rt, 0, 0, w, h, 0, 0, w, h, 0, 0, 0, 0, 0, l, f, 0, 0);
}

/* before anything uses the texture itself */
static void tex_resolve(es_tex *t)
{
    if (t->rt_dirty) tex_copy_rt(t, 1);
    t->rt_dirty = 0;
}

static void tex_drop_rt(es_tex *t)
{
    if (t->ds_alias) { g2r_end_pass(S.r); g2r_alias_destroy(S.r, t->ds_alias); t->ds_alias = 0; }
    if (t->rt_ds) { g2r_end_pass(S.r); g2r_alias_destroy(S.r, t->rt_ds); t->rt_ds = 0; }
    if (t->rt) { g2r_end_pass(S.r); g2r_image_destroy(S.r, t->rt, t->rt_base); }
    t->rt = 0; t->rt_dirty = t->rt_stale = 0;
}

/* what a framebuffer renders in when it cannot be the texture: another level, another layer, a pitch that is no multiple of 32; for a depth
   texture (ds) rows as long as board B loads a depth row, which its D24S8 alias renders through */
static int tex_rt(es_tex *x, int face, int level, int w, int h, int ds)
{
    if (x->rt && (x->rt_w != w || x->rt_h != h)) { tex_resolve(x); if (x->rt_ds) { g2r_end_pass(S.r); g2r_alias_destroy(S.r, x->rt_ds); x->rt_ds = 0; } g2r_end_pass(S.r); g2r_image_destroy(S.r, x->rt, x->rt_base); x->rt = 0; x->rt_stale = 0; }
    if (!x->rt) {
        if ((ds ? g2r_image_create_ds_rows(S.r, &x->rt, &x->rt_base, (uint32_t)w, (uint32_t)h, x->g2fmt, 1, G2_USE_RT | G2_USE_TRANSFER)
                : g2r_image_create(S.r, &x->rt, &x->rt_base, (uint32_t)w, (uint32_t)h, x->g2fmt, 1, 1, 1, G2_USE_RT | G2_USE_TRANSFER)) < 0) { x->rt = 0; SET_ERR(GL_OUT_OF_MEMORY); return -1; }
        x->rt_stale = 1; x->rt_face = face; x->rt_level = level; x->rt_w = w; x->rt_h = h;
    }
    if (x->rt_face != face || x->rt_level != level) { tex_resolve(x); x->rt_face = face; x->rt_level = level; x->rt_stale = 1; }
    if (x->rt_stale) { tex_copy_rt(x, 0); x->rt_stale = 0; }
    x->rt_dirty = 1;
    return 0;
}

static int bind_target(es_tg *g)
{
    if (fb_target(g) != GL_FRAMEBUFFER_COMPLETE) { SET_ERR(GL_INVALID_FRAMEBUFFER_OPERATION); return -1; }
    if (g->tex && (tex_needs_rt(g->tex) || g->level)) {
        if (tex_rt(g->tex, g->face, g->level, (int)g->t.width, (int)g->t.height, 0) < 0) return -1;
        g->t.color = g->tex->rt;
    }
    if (g->dtex) {
        es_tex *x = g->dtex;
        if (tex_needs_rt(x) || (uint32_t)x->w * 4 != g2r_ds_pitch((uint32_t)x->w)) {     /* its tight rows are shorter than a depth row of board B */
            if (tex_rt(x, 0, 0, x->w, x->h, 1) < 0) return -1;
            if (!x->rt_ds && g2r_image_alias(S.r, &x->rt_ds, x->rt_base, (uint32_t)x->w, (uint32_t)x->h, G2_FMT_D24S8, g2r_ds_pitch((uint32_t)x->w), 1, 1, G2_USE_RT) < 0) { x->rt_ds = 0; SET_ERR(GL_OUT_OF_MEMORY); return -1; }
            g->t.ds = x->rt_ds;
        } else {
            if (!x->ds_alias && g2r_image_alias(S.r, &x->ds_alias, x->base, (uint32_t)x->w, (uint32_t)x->h, G2_FMT_D24S8, (uint32_t)x->w * 4, 1, 1, G2_USE_RT) < 0) { x->ds_alias = 0; SET_ERR(GL_OUT_OF_MEMORY); return -1; }
            g->t.ds = x->ds_alias;
        }
    }
    if (!g->t.color && g->fb) {                                 /* depth alone: nothing reads this one */
        es_fb *f = g->fb;
        int w = (int)g->t.width, h = (int)g->t.height;
        if (f->blind && (f->blind_w != w || f->blind_h != h)) { g2r_end_pass(S.r); g2r_image_destroy(S.r, f->blind, f->blind_base); f->blind = 0; }
        if (!f->blind) {
            if (g2r_image_create(S.r, &f->blind, &f->blind_base, (uint32_t)w, (uint32_t)h, G2_FMT_RGBA8, 1, 1, 1, G2_USE_RT) < 0) { f->blind = 0; SET_ERR(GL_OUT_OF_MEMORY); return -1; }
            f->blind_w = w; f->blind_h = h;
        }
        g->t.color = f->blind;
    }
    g2r_set_target(S.r, &g->t);
    if (g->tex) g->tex->used_pass = g2r_pass_id(S.r);
    if (g->dtex) g->dtex->used_pass = g2r_pass_id(S.r);
    return 0;
}

/* ---------------------------------------------------------------- buffers */
void ES_APIENTRY glGenBuffers(GLsizei n, GLuint *b) { GLsizei i; if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; } for (i = 0; i < n; i++) b[i] = gen_name(); }
void ES_APIENTRY glDeleteBuffers(GLsizei n, const GLuint *b)
{
    GLsizei i;
    int a;
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        if (S.array_buf == b[i]) S.array_buf = 0;
        if (S.elem_buf == b[i]) S.elem_buf = 0;
        for (a = 0; a < MAX_ATTR; a++) if (S.va[a].buffer == b[i]) S.va[a].buffer = 0;
        DEL(bufs, b[i], es_buf, (g2r_keep_changed(S.r, &x_->keep), free(x_->data)));
    }
}
GLboolean ES_APIENTRY glIsBuffer(GLuint b) { return b && buf_of(b); }
void ES_APIENTRY glBindBuffer(GLenum t, GLuint b)
{
    if (t != GL_ARRAY_BUFFER && t != GL_ELEMENT_ARRAY_BUFFER) { SET_ERR(GL_INVALID_ENUM); return; }
    if (b && !buf_of(b)) { es_buf *o; NEW(bufs, b, es_buf, o); (void)o; }
    if (t == GL_ARRAY_BUFFER) S.array_buf = b; else S.elem_buf = b;
}
static es_buf *bound_buf(GLenum t)
{
    GLuint n;
    if (t != GL_ARRAY_BUFFER && t != GL_ELEMENT_ARRAY_BUFFER) { SET_ERR(GL_INVALID_ENUM); return NULL; }
    n = t == GL_ELEMENT_ARRAY_BUFFER ? S.elem_buf : S.array_buf;
    if (!n) { SET_ERR(GL_INVALID_OPERATION); return NULL; }
    return buf_of(n);
}
void ES_APIENTRY glBufferData(GLenum t, GLsizeiptr size, const void *d, GLenum usage)
{
    es_buf *o;
    if (usage != GL_STREAM_DRAW && usage != GL_STATIC_DRAW && usage != GL_DYNAMIC_DRAW) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!(o = bound_buf(t))) return;
    if (size < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    g2r_keep_changed(S.r, &o->keep);
    free(o->data);
    o->mapped = 0;
    o->data = (uint8_t *)malloc(size ? (size_t)size : 1);
    o->size = 0;
    if (!o->data) { SET_ERR(GL_OUT_OF_MEMORY); return; }
    if (d) memcpy(o->data, d, (size_t)size);
    o->size = size; o->usage = usage;
}
void ES_APIENTRY glBufferSubData(GLenum t, GLintptr off, GLsizeiptr size, const void *d)
{
    es_buf *o = bound_buf(t);
    if (!o) return;
    if (off < 0 || size < 0 || off + size > o->size) { SET_ERR(GL_INVALID_VALUE); return; }
    if (o->mapped) { SET_ERR(GL_INVALID_OPERATION); return; }
    g2r_keep_changed(S.r, &o->keep);
    if (d) memcpy(o->data + off, d, (size_t)size);
}
void ES_APIENTRY glGetBufferParameteriv(GLenum t, GLenum p, GLint *v)
{
    es_buf *o = bound_buf(t);
    if (!o) return;
    if (p == GL_BUFFER_SIZE) *v = (GLint)o->size;
    else if (p == GL_BUFFER_USAGE) *v = (GLint)(o->usage ? o->usage : GL_STATIC_DRAW);
    else if (p == GL_BUFFER_ACCESS_OES) *v = GL_WRITE_ONLY_OES;
    else if (p == GL_BUFFER_MAPPED_OES) *v = o->mapped;
    else SET_ERR(GL_INVALID_ENUM);
}
/* GL_OES_mapbuffer: a buffer is a PC copy, so a map is that copy; what was kept on board A goes at the map */
void *ES_APIENTRY glMapBufferOES(GLenum t, GLenum access)
{
    es_buf *o = bound_buf(t);
    if (!o) return NULL;
    if (access != GL_WRITE_ONLY_OES) { SET_ERR(GL_INVALID_ENUM); return NULL; }
    if (o->mapped || !o->data) { SET_ERR(GL_INVALID_OPERATION); return NULL; }
    g2r_keep_changed(S.r, &o->keep);
    o->mapped = 1;
    return o->data;
}
GLboolean ES_APIENTRY glUnmapBufferOES(GLenum t)
{
    es_buf *o = bound_buf(t);
    if (!o) return GL_FALSE;
    if (!o->mapped) { SET_ERR(GL_INVALID_OPERATION); return GL_FALSE; }
    o->mapped = 0;
    return GL_TRUE;
}
void ES_APIENTRY glGetBufferPointervOES(GLenum t, GLenum p, void **v)
{
    es_buf *o = bound_buf(t);
    if (!o) return;
    if (p != GL_BUFFER_MAP_POINTER_OES) { SET_ERR(GL_INVALID_ENUM); return; }
    *v = o->mapped ? o->data : NULL;
}

/* ---------------------------------------------------------------- pixel formats */
static const struct { uint8_t sh[4], bits[4]; } F16[3] = { { { 11, 5, 0, 0 }, { 5, 6, 5, 0 } }, { { 12, 8, 4, 0 }, { 4, 4, 4, 4 } }, { { 11, 6, 1, 0 }, { 5, 5, 5, 1 } } };
static int f16_of(uint32_t fmt) { return fmt == G2_FMT_RGB565 ? 0 : fmt == G2_FMT_RGBA4444 ? 1 : 2; }

static void unpack16(uint16_t v, int f, uint8_t *d)
{
    int c;
    for (c = 0; c < 4; c++) {
        uint32_t m = (1u << F16[f].bits[c]) - 1;
        d[c] = m ? (uint8_t)(((v >> F16[f].sh[c] & m) * 510 + m) / (2 * m)) : 255;
    }
}

static uint16_t pack16(const uint8_t *s, int f)
{
    uint32_t v = 0;
    int c;
    for (c = 0; c < 4; c++) {
        uint32_t m = (1u << F16[f].bits[c]) - 1;
        v |= (((uint32_t)s[c] * 2 * m + 255) / 510) << F16[f].sh[c];
    }
    return (uint16_t)v;
}

static uint32_t natural_fmt(GLenum format, GLenum type)
{
    switch (type) {
    case GL_UNSIGNED_BYTE:
        return format == GL_RGBA || format == GL_RGB || format == GL_BGRA_EXT || format == GL_LUMINANCE || format == GL_ALPHA || format == GL_LUMINANCE_ALPHA ? G2_FMT_RGBA8 : NOFMT;
    case GL_UNSIGNED_SHORT_5_6_5: return format == GL_RGB ? G2_FMT_RGB565 : NOFMT;
    case GL_UNSIGNED_SHORT_4_4_4_4: return format == GL_RGBA ? G2_FMT_RGBA4444 : NOFMT;
    case GL_UNSIGNED_SHORT_5_5_5_1: return format == GL_RGBA ? G2_FMT_RGBA5551 : NOFMT;
    case GL_UNSIGNED_SHORT: case GL_UNSIGNED_INT: return format == GL_DEPTH_COMPONENT ? G2_FMT_RGBA8 : NOFMT;      /* OES_depth_texture: the word of a D24S8 target in the bytes of RGBA8 */
    case GL_UNSIGNED_INT_24_8_OES: return format == GL_DEPTH_STENCIL_OES ? G2_FMT_RGBA8 : NOFMT;
    default: return NOFMT;
    }
}
/* §3.7.1, table 3.4 and the extensions' rows: 0 = format and type are accepted values and go together; INVALID_ENUM: one of them is not an accepted value; INVALID_OPERATION: accepted values that do not
   go together */
static GLenum format_type_error(GLenum format, GLenum type)
{
    int fok = format == GL_RGBA || format == GL_RGB || format == GL_LUMINANCE_ALPHA || format == GL_LUMINANCE || format == GL_ALPHA || format == GL_BGRA_EXT || format == GL_DEPTH_COMPONENT || format == GL_DEPTH_STENCIL_OES;
    int tok = type == GL_UNSIGNED_BYTE || type == GL_UNSIGNED_SHORT_5_6_5 || type == GL_UNSIGNED_SHORT_4_4_4_4 || type == GL_UNSIGNED_SHORT_5_5_5_1 || type == GL_UNSIGNED_SHORT || type == GL_UNSIGNED_INT || type == GL_UNSIGNED_INT_24_8_OES;
    if (!fok || !tok) return GL_INVALID_ENUM;
    return natural_fmt(format, type) == NOFMT ? GL_INVALID_OPERATION : 0;
}
static int client_bpp(GLenum format, GLenum type) { return type == GL_UNSIGNED_INT || type == GL_UNSIGNED_INT_24_8_OES ? 4 : type != GL_UNSIGNED_BYTE ? 2 : format == GL_LUMINANCE || format == GL_ALPHA ? 1 : format == GL_LUMINANCE_ALPHA ? 2 : format == GL_RGB ? 3 : 4; }

static void px_rgba8(const uint8_t *s, int c, GLenum format, GLenum type, uint8_t *d)
{
    if (type != GL_UNSIGNED_BYTE) {
        uint16_t v;
        memcpy(&v, s + 2 * c, 2);
        unpack16(v, type == GL_UNSIGNED_SHORT_5_6_5 ? 0 : type == GL_UNSIGNED_SHORT_4_4_4_4 ? 1 : 2, d);
        return;
    }
    switch (format) {
    case GL_RGBA: memcpy(d, s + 4 * c, 4); break;
    case GL_BGRA_EXT: d[0] = s[4 * c + 2]; d[1] = s[4 * c + 1]; d[2] = s[4 * c]; d[3] = s[4 * c + 3]; break;
    case GL_RGB: memcpy(d, s + 3 * c, 3); d[3] = 255; break;
    case GL_LUMINANCE: d[0] = d[1] = d[2] = s[c]; d[3] = 255; break;
    case GL_ALPHA: d[0] = d[1] = d[2] = 0; d[3] = s[c]; break;
    default: d[0] = d[1] = d[2] = s[2 * c]; d[3] = s[2 * c + 1]; break;
    }
}

/* client rows to tight rows of the storage format `fmt`; NULL: no memory */
static uint8_t *to_storage(const void *pixels, int w, int h, GLenum format, GLenum type, uint32_t fmt, uint32_t *etf, int *ebpp)
{
    int cb = client_bpp(format, type), a = S.un_align, row = (w * cb + a - 1) / a * a, nat = natural_fmt(format, type) == fmt, r, c;
    uint8_t *out;
    *ebpp = fmt == G2_FMT_RGBA8 ? 4 : 2;
    *etf = fmt == G2_FMT_RGBA8 ? G2_TF_RGBA8888 : G2_TF_RAW;
    out = (uint8_t *)malloc((size_t)w * h * *ebpp);
    if (!out) { SET_ERR(GL_OUT_OF_MEMORY); return NULL; }
    if (format == GL_DEPTH_COMPONENT || format == GL_DEPTH_STENCIL_OES) {       /* {stencil, depth}, R the top byte */
        for (r = 0; r < h; r++) {
            const uint8_t *s = (const uint8_t *)pixels + (ptrdiff_t)r * row;
            for (c = 0; c < w; c++) {
                uint32_t d, st = 0;
                uint8_t *o = out + ((size_t)r * w + c) * 4;
                if (type == GL_UNSIGNED_SHORT) { uint16_t v; memcpy(&v, s + 2 * c, 2); d = (uint32_t)v << 8 | v >> 8; }
                else { uint32_t v; memcpy(&v, s + 4 * c, 4); d = v >> 8; st = type == GL_UNSIGNED_INT_24_8_OES ? v & 255 : 0; }
                o[0] = (uint8_t)st; o[1] = (uint8_t)(d >> 16); o[2] = (uint8_t)(d >> 8); o[3] = (uint8_t)d;
            }
        }
        return out;
    }
    for (r = 0; r < h; r++) {
        const uint8_t *s = (const uint8_t *)pixels + (ptrdiff_t)r * row;
        uint8_t *d = out + (size_t)r * w * *ebpp;
        if (nat && *ebpp == cb && format != GL_BGRA_EXT) { memcpy(d, s, (size_t)w * cb); continue; }
        for (c = 0; c < w; c++) {
            uint8_t px[4];
            px_rgba8(s, c, format, type, px);
            if (*ebpp == 4) { memcpy(d + 4 * c, px, 4); }
            else { uint16_t v = pack16(px, f16_of(fmt)); memcpy(d + 2 * c, &v, 2); }
        }
    }
    return out;
}

/* ---------------------------------------------------------------- textures */
static void tex_init(es_tex *t, GLenum target) { t->min_f = GL_NEAREST_MIPMAP_LINEAR; t->mag_f = GL_LINEAR; t->wrap_s = t->wrap_t = t->wrap_r = GL_REPEAT; t->target = target; t->used_pass = 0xFFFFFFFFu; }
/* kind: 0 2D, 1 cube map, 2 3D */
static es_tex *unit_tex(int u, int kind)
{
    GLuint n = kind == 2 ? S.tex3d[u] : kind ? S.cube[u] : S.tex2d[u];
    if (n) return tex_of(n);
    if (!S.tex_default[kind]) {
        S.tex_default[kind] = (es_tex *)calloc(1, sizeof(es_tex));
        tex_init(S.tex_default[kind], kind == 2 ? GL_TEXTURE_3D_OES : kind ? GL_TEXTURE_CUBE_MAP : GL_TEXTURE_2D);
    }
    return S.tex_default[kind];
}
static int has_3d(void) { return S.r && g2r_has_3d(S.r); }
static int tex_kind(GLenum target) { return target == GL_TEXTURE_CUBE_MAP ? 1 : target == GL_TEXTURE_3D_OES ? 2 : 0; }
static int tex_target(GLenum target) { return target == GL_TEXTURE_2D || target == GL_TEXTURE_CUBE_MAP || (target == GL_TEXTURE_3D_OES && has_3d()); }
void ES_APIENTRY glGenTextures(GLsizei n, GLuint *t) { GLsizei i; if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; } for (i = 0; i < n; i++) t[i] = gen_name(); }
GLboolean ES_APIENTRY glIsTexture(GLuint t) { return t && tex_of(t); }
void ES_APIENTRY glActiveTexture(GLenum t)
{
    if (t < GL_TEXTURE0 || t >= GL_TEXTURE0 + MAX_UNITS) { SET_ERR(GL_INVALID_ENUM); return; }
    S.active = (int)(t - GL_TEXTURE0);
}
void ES_APIENTRY glBindTexture(GLenum target, GLuint n)
{
    es_tex *t;
    if (!tex_target(target)) { SET_ERR(GL_INVALID_ENUM); return; }
    if (n) {
        t = tex_of(n);
        if (!t) {
            NEW(texs, n, es_tex, t);
            if (!t) return;
            tex_init(t, target);
        } else if (t->target != target) { SET_ERR(GL_INVALID_OPERATION); return; }
    }
    if (target == GL_TEXTURE_2D) S.tex2d[S.active] = n; else if (target == GL_TEXTURE_3D_OES) S.tex3d[S.active] = n; else S.cube[S.active] = n;
}
static void free_pend(es_tex *t, int f, int l)
{
    if (t->pend[f][l]) { free(t->pend[f][l]->px); free(t->pend[f][l]); t->pend[f][l] = NULL; }
}
static void tex_free(es_tex *t)
{
    int f, l;
    for (f = 0; f < 6; f++) for (l = 0; l < MAX_LEVELS; l++) free_pend(t, f, l);
    if (S.r) tex_drop_rt(t);
    /* the open pass ends only if it used the texture: its draws wait for the FLUSH and still need the texels */
    if (t->handle && S.r) { if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r); g2r_image_destroy(S.r, t->handle, t->base); }
}
static void rb_free(es_rb *b) { if (b->handle && S.r) { g2r_end_pass(S.r); g2r_image_destroy(S.r, b->handle, b->base); } }

/* an attachment point let go of its image: an object whose name is gone dies with the last attachment that held it */
static void att_release(es_att *a)
{
    if (a->tex && !--a->tex->att_refs && a->tex->orphan) { tex_free(a->tex); free(a->tex); }
    if (a->rb && !--a->rb->att_refs && a->rb->orphan) { rb_free(a->rb); free(a->rb); }
    memset(a, 0, sizeof *a);
}
static void att_set(es_att *a, GLenum type, GLuint name, es_tex *t, es_rb *b, GLenum face, int lvl, int z)
{
    if (t) t->att_refs++;
    if (b) b->att_refs++;
    att_release(a);
    if (!t && !b) return;
    a->type = type; a->name = name; a->tex = t; a->rb = b; a->face = face; a->lvl = lvl; a->z = z;
}
/* §4.4.2: an image deleted while attached to the bound framebuffer (draw or read) is detached from it, as if attached again with name 0. Other framebuffers keep it */
static void detach_bound(es_tex *t, es_rb *b)
{
    GLuint bound[2];
    int i, k;
    bound[0] = S.fb; bound[1] = S.read_fb;
    for (i = 0; i < 2; i++) {
        es_fb *f = bound[i] ? fb_of(bound[i]) : NULL;
        if (!f) continue;
        for (k = 0; k < 3; k++) if ((t && f->a[k].tex == t) || (b && f->a[k].rb == b)) att_release(&f->a[k]);
    }
}
void ES_APIENTRY glDeleteTextures(GLsizei n, const GLuint *t)
{
    GLsizei i;
    int u;
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        es_tex *x;
        if (!t[i] || !(x = tex_of(t[i]))) continue;
        for (u = 0; u < MAX_UNITS; u++) { if (S.tex2d[u] == t[i]) S.tex2d[u] = 0; if (S.cube[u] == t[i]) S.cube[u] = 0; if (S.tex3d[u] == t[i]) S.tex3d[u] = 0; }
        detach_bound(x, NULL);
        UNLINK(texs, t[i], es_tex, x);
        if (x->att_refs) x->orphan = 1;
        else { tex_free(x); free(x); }
    }
}
static void tex_param(GLenum target, GLenum p, GLint v)
{
    es_tex *t;
    if (!tex_target(target)) { SET_ERR(GL_INVALID_ENUM); return; }
    t = unit_tex(S.active, tex_kind(target));
    switch (p) {
    case GL_TEXTURE_MIN_FILTER:
        if (v != GL_NEAREST && v != GL_LINEAR && v != GL_NEAREST_MIPMAP_NEAREST && v != GL_LINEAR_MIPMAP_NEAREST && v != GL_NEAREST_MIPMAP_LINEAR && v != GL_LINEAR_MIPMAP_LINEAR) { SET_ERR(GL_INVALID_ENUM); return; }
        t->min_f = (GLenum)v;
        break;
    case GL_TEXTURE_MAG_FILTER:
        if (v != GL_NEAREST && v != GL_LINEAR) { SET_ERR(GL_INVALID_ENUM); return; }
        t->mag_f = (GLenum)v;
        break;
    case GL_TEXTURE_WRAP_S: case GL_TEXTURE_WRAP_T: case GL_TEXTURE_WRAP_R_OES:
        if (p == GL_TEXTURE_WRAP_R_OES && target != GL_TEXTURE_3D_OES) { SET_ERR(GL_INVALID_ENUM); return; }
        if (v != GL_REPEAT && v != GL_CLAMP_TO_EDGE && v != GL_MIRRORED_REPEAT) { SET_ERR(GL_INVALID_ENUM); return; }
        if (p == GL_TEXTURE_WRAP_S) t->wrap_s = (GLenum)v; else if (p == GL_TEXTURE_WRAP_T) t->wrap_t = (GLenum)v; else t->wrap_r = (GLenum)v;
        break;
    default: SET_ERR(GL_INVALID_ENUM); break;
    }
}
void ES_APIENTRY glTexParameteri(GLenum t, GLenum p, GLint v) { tex_param(t, p, v); }
void ES_APIENTRY glTexParameterf(GLenum t, GLenum p, GLfloat v) { tex_param(t, p, (GLint)v); }
void ES_APIENTRY glTexParameteriv(GLenum t, GLenum p, const GLint *v) { tex_param(t, p, v[0]); }
void ES_APIENTRY glTexParameterfv(GLenum t, GLenum p, const GLfloat *v) { tex_param(t, p, (GLint)v[0]); }
void ES_APIENTRY glGetTexParameteriv(GLenum target, GLenum p, GLint *v)
{
    es_tex *t;
    if (!tex_target(target)) { SET_ERR(GL_INVALID_ENUM); return; }
    t = unit_tex(S.active, tex_kind(target));
    switch (p) {
    case GL_TEXTURE_MIN_FILTER: *v = (GLint)t->min_f; break;
    case GL_TEXTURE_MAG_FILTER: *v = (GLint)t->mag_f; break;
    case GL_TEXTURE_WRAP_S: *v = (GLint)t->wrap_s; break;
    case GL_TEXTURE_WRAP_T: *v = (GLint)t->wrap_t; break;
    case GL_TEXTURE_WRAP_R_OES: if (target == GL_TEXTURE_3D_OES) *v = (GLint)t->wrap_r; else SET_ERR(GL_INVALID_ENUM); break;
    case GL_TEXTURE_IMMUTABLE_FORMAT_EXT: *v = t->immutable != 0; break;
    default: SET_ERR(GL_INVALID_ENUM); break;
    }
}
void ES_APIENTRY glGetTexParameterfv(GLenum target, GLenum p, GLfloat *v) { GLint i = 0; glGetTexParameteriv(target, p, &i); *v = (GLfloat)i; }

static int is_face(GLenum t) { return t >= GL_TEXTURE_CUBE_MAP_POSITIVE_X && t <= GL_TEXTURE_CUBE_MAP_NEGATIVE_Z; }
static int image_target(GLenum t) { if (t == GL_TEXTURE_2D || is_face(t)) return 1; SET_ERR(GL_INVALID_ENUM); return 0; }
static int face_of(GLenum t) { return is_face(t) ? (int)(t - GL_TEXTURE_CUBE_MAP_POSITIVE_X) : 0; }
static int ilog2(int v) { int n = 0; while (v > 1) { v >>= 1; n++; } return n; }
static int mip_dim(int d, int l) { d >>= l; return d ? d : 1; }
static int is_pot(int v) { return !(v & (v - 1)); }

static int level_fits(const es_tex *t, int level, int w, int h, uint32_t fmt)
{
    return t->handle && fmt == t->g2fmt && level < t->levels && w == mip_dim(t->w, level) && h == mip_dim(t->h, level);
}

/* pixels of a level already defined: on B, or in the waiting copy */
static void tex_write(es_tex *t, int level, int face, int x, int y, int w, int h, uint32_t etf, int ebpp, const uint8_t *px)
{
    es_pend *pd = t->pend[face][level];
    if (pd) {
        int r, lw = t->lw[face][level];
        for (r = 0; r < h; r++) memcpy(pd->px + ((size_t)(y + r) * lw + x) * ebpp, px + (size_t)r * w * ebpp, (size_t)w * ebpp);
        return;
    }
    tex_resolve(t);
    if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r);    /* B writes at once, draws wait for the FLUSH: earlier draws must keep the old texels */
    g2_write_image(S.bd, t->handle, px, (size_t)w * h * ebpp, etf, (uint32_t)x, (uint32_t)y, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)face, 0, t->g2fmt);
    if (t->rt) t->rt_stale = 1;
}

static int tex_storage(es_tex *t, int w, int h, uint32_t fmt, int layers)
{
    int f, l, levels = S.tex_storing ? S.tex_storing : 1 + ilog2(w > h ? w : h);    /* GL_OES_texture_npot: the TMU sizes a level floor(d / 2^l) and wraps any size. EXT_texture_storage: the TMU's chain ends where the levels asked for end */
    tex_drop_rt(t);
    if (t->handle) {
        if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r);
        g2r_image_destroy(S.r, t->handle, t->base);
        t->handle = 0;
    }
    for (f = 0; f < 6; f++) for (l = 0; l < MAX_LEVELS; l++) if (!t->pend[f][l]) t->lkey[f][l] = 0;
    if (g2r_image_create_tight(S.r, &t->handle, &t->base, (uint32_t)w, (uint32_t)h, fmt, (uint32_t)levels, (uint32_t)layers, 1, G2_USE_SAMPLED | G2_USE_RT | G2_USE_TRANSFER) < 0) { SET_ERR(GL_OUT_OF_MEMORY); return -1; }
    t->w = w; t->h = h; t->levels = levels; t->layers = layers; t->g2fmt = fmt;
    return 0;
}

static void flush_pending(es_tex *t)
{
    int f, l;
    for (f = 0; f < 6; f++) {
        for (l = 1; l < MAX_LEVELS; l++) {
            es_pend *pd = t->pend[f][l];
            if (pd && level_fits(t, l, t->lw[f][l], t->lh[f][l], pd->fmt)) {
                t->pend[f][l] = NULL;
                tex_write(t, l, f, 0, 0, t->lw[f][l], t->lh[f][l], pd->etf, pd->ebpp, pd->px);
                free(pd->px); free(pd);
            }
        }
    }
}

/* glTexImage2D after validation: px = tight rows in storage `fmt`, owned; NULL: no data */
static void tex_image(GLenum target, int level, int w, int h, uint32_t key, GLenum format, uint32_t fmt, int compressed, uint8_t *px, uint32_t etf, int ebpp)
{
    int cube = is_face(target), face = face_of(target);
    es_tex *t = unit_tex(S.active, cube);
    free_pend(t, face, level);
    if (level == 0) {
        if (w && h && (!t->handle || t->g2fmt != fmt || t->w != w || t->h != h) && tex_storage(t, w, h, fmt, cube ? 6 : 1) < 0) { free(px); return; }
        t->format = format; t->compressed = compressed;
    }
    t->lkey[face][level] = w && h ? key : 0;
    t->lw[face][level] = w; t->lh[face][level] = h;
    if (level == 0) flush_pending(t);
    if (!level_fits(t, level, w, h, fmt)) {
        if (px) {
            es_pend *pd = (es_pend *)malloc(sizeof *pd);
            if (!pd) { SET_ERR(GL_OUT_OF_MEMORY); free(px); return; }
            pd->px = px; pd->fmt = fmt; pd->etf = etf; pd->ebpp = ebpp;
            t->pend[face][level] = pd;
        }
        return;
    }
    if (px) {
        tex_write(t, level, face, 0, 0, w, h, etf, ebpp, px);
        free(px);
    } else if (format == GL_RGB && fmt == G2_FMT_RGBA8 && w && h) {     /* no data yet, but its alpha reads 1, also after rendering with alpha masked off */
        tex_resolve(t);
        if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r);
        g2_fill_image(S.bd, t->handle, G2_TF_RAW, 0x000000FFu, 0, 0, (uint32_t)w, (uint32_t)h, MASK_A, (uint32_t)level, (uint32_t)face);
        if (t->rt) t->rt_stale = 1;
    }
}

void ES_APIENTRY glTexImage2D(GLenum target, GLint level, GLint ifmt, GLsizei w, GLsizei h, GLint border, GLenum format, GLenum type, const void *pixels)
{
    uint32_t fmt, etf = 0;
    int ebpp = 0;
    uint8_t *px = NULL;
    GLenum e;
    if (!image_target(target)) return;
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || w > MAX_TEX >> level || h > MAX_TEX >> level || border) { SET_ERR(GL_INVALID_VALUE); return; }
    if (is_face(target) && w != h) { SET_ERR(GL_INVALID_VALUE); return; }
    switch ((GLenum)ifmt) {                                     /* internalformat is no enum here: one it does not take is INVALID_VALUE */
    case GL_ALPHA: case GL_LUMINANCE: case GL_LUMINANCE_ALPHA: case GL_RGB: case GL_RGBA: case GL_BGRA_EXT: case GL_BGRA8_EXT: case GL_DEPTH_COMPONENT: case GL_DEPTH_STENCIL_OES: break;
    default: SET_ERR(GL_INVALID_VALUE); return;
    }
    if ((e = format_type_error(format, type))) { SET_ERR(e); return; }
    fmt = natural_fmt(format, type);
    if ((GLenum)ifmt == GL_BGRA8_EXT ? format != GL_BGRA_EXT : (GLenum)ifmt != format) { SET_ERR(GL_INVALID_OPERATION); return; }     /* EXT_texture_format_BGRA8888: BGRA8_EXT is BGRA_EXT's sized form */
    if ((format == GL_DEPTH_COMPONENT || format == GL_DEPTH_STENCIL_OES) && (target != GL_TEXTURE_2D || level)) { SET_ERR(GL_INVALID_OPERATION); return; }    /* OES_depth_texture: one level of a 2D texture */
    if (!S.live) return;
    if (unit_tex(S.active, is_face(target))->immutable && !S.tex_storing) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (pixels && w && h && !(px = to_storage(pixels, w, h, format, type, fmt, &etf, &ebpp))) return;
    tex_image(target, level, w, h, (uint32_t)format | (uint32_t)type << 16, format, fmt, 0, px, etf, ebpp);
}

void ES_APIENTRY glTexSubImage2D(GLenum target, GLint level, GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, const void *pixels)
{
    uint32_t etf;
    int ebpp, face = face_of(target);
    uint8_t *px;
    es_tex *t;
    GLenum e;
    if (!image_target(target)) return;
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || x < 0 || y < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if ((e = format_type_error(format, type))) { SET_ERR(e); return; }
    t = unit_tex(S.active, is_face(target));
    if (!S.live) return;
    if (!t->lkey[face][level] || (t->lkey[face][level] & 0xFFFF) != format) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (x + w > t->lw[face][level] || y + h > t->lh[face][level]) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!w || !h || !pixels) return;
    px = to_storage(pixels, w, h, format, type, t->pend[face][level] ? t->pend[face][level]->fmt : t->g2fmt, &etf, &ebpp);
    if (!px) return;
    tex_write(t, level, face, x, y, w, h, etf, ebpp, px);
    free(px);
}

static int dxt_kind(GLenum f)
{
    switch (f) {
    case GL_COMPRESSED_RGB_S3TC_DXT1_EXT: return G2_DXT1;
    case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT: return G2_DXT1A;
    case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT: return G2_DXT3;
    case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT: return G2_DXT5;
    default: return 0;
    }
}

void ES_APIENTRY glCompressedTexImage2D(GLenum target, GLint level, GLenum f, GLsizei w, GLsizei h, GLint border, GLsizei size, const void *data)
{
    int k = dxt_kind(f), etc = f == GL_ETC1_RGB8_OES;
    uint8_t *px = NULL;
    if (!image_target(target)) return;
    if (!k && !etc) { SET_ERR(GL_INVALID_ENUM); return; }
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || w > MAX_TEX >> level || h > MAX_TEX >> level || border || (is_face(target) && w != h)) { SET_ERR(GL_INVALID_VALUE); return; }
    if ((uint32_t)size != (etc ? g2_etc1_bytes(w, h) : g2_dxt_bytes(k, w, h))) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!S.live) return;
    if (data && w && h) {
        px = (uint8_t *)malloc((size_t)w * h * 4);
        if (!px) { SET_ERR(GL_OUT_OF_MEMORY); return; }
        if (etc) g2_etc1_decode((const uint8_t *)data, w, h, px);
        else g2_dxt_decode(k, (const uint8_t *)data, w, h, px);
    }
    tex_image(target, level, w, h, (uint32_t)f, f, G2_FMT_RGBA8, 1, px, G2_TF_RGBA8888, 4);
}

void ES_APIENTRY glCompressedTexSubImage2D(GLenum target, GLint level, GLint x, GLint y, GLsizei w, GLsizei h, GLenum f, GLsizei size, const void *data)
{
    int k = dxt_kind(f), face = face_of(target);
    uint8_t *px;
    es_tex *t;
    if (!image_target(target)) return;
    if (f == GL_ETC1_RGB8_OES) { SET_ERR(GL_INVALID_OPERATION); return; }     /* OES_compressed_ETC1_RGB8_texture has no sub-images */
    if (!k) { SET_ERR(GL_INVALID_ENUM); return; }
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || x < 0 || y < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    t = unit_tex(S.active, is_face(target));
    if (!S.live) return;
    if (t->lkey[face][level] != (uint32_t)f) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (x + w > t->lw[face][level] || y + h > t->lh[face][level]) { SET_ERR(GL_INVALID_VALUE); return; }
    if ((x % 4) || (y % 4) || ((w % 4) && x + w != t->lw[face][level]) || ((h % 4) && y + h != t->lh[face][level])) { SET_ERR(GL_INVALID_OPERATION); return; }
    if ((uint32_t)size != g2_dxt_bytes(k, w, h)) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!w || !h || !data) return;
    px = (uint8_t *)malloc((size_t)w * h * 4);
    if (!px) { SET_ERR(GL_OUT_OF_MEMORY); return; }
    g2_dxt_decode(k, (const uint8_t *)data, w, h, px);
    tex_write(t, level, face, x, y, w, h, G2_TF_RGBA8888, 4, px);
    free(px);
}

/* ---------------------------------------------------------------- GL_OES_texture_3D: a volume on B. A waiting level holds its slices one after the other, as B does */
static int level3_fits(const es_tex *t, int level, int w, int h, int d, uint32_t fmt)
{
    return t->handle && fmt == t->g2fmt && level < t->levels && w == mip_dim(t->w, level) && h == mip_dim(t->h, level) && d == mip_dim(t->d, level);
}

static void tex3_write(es_tex *t, int level, int x, int y, int z, int w, int h, int d, uint32_t etf, int ebpp, const uint8_t *px)
{
    es_pend *pd = t->pend[0][level];
    int s, r;
    if (pd) {
        int lw = t->lw[0][level], lh = t->lh[0][level];
        for (s = 0; s < d; s++) for (r = 0; r < h; r++) memcpy(pd->px + (((size_t)(z + s) * lh + y + r) * lw + x) * ebpp, px + ((size_t)s * h + r) * w * ebpp, (size_t)w * ebpp);
        return;
    }
    if (!level3_fits(t, level, t->lw[0][level], t->lh[0][level], t->ld[level], t->g2fmt)) return;
    tex_resolve(t);
    if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r);    /* B writes at once, draws wait for the FLUSH */
    for (s = 0; s < d; s++) g2_write_image(S.bd, t->handle, px + (size_t)s * w * h * ebpp, (size_t)w * h * ebpp, etf, (uint32_t)x, (uint32_t)y, (uint32_t)w, (uint32_t)h, (uint32_t)level, (uint32_t)(z + s), 0, t->g2fmt);
    if (t->rt) t->rt_stale = 1;
}

static int tex3_storage(es_tex *t, int w, int h, int d, uint32_t fmt)
{
    int l, m = w > h ? w : h, levels = S.tex_storing ? S.tex_storing : 1 + ilog2(m > d ? m : d);
    tex_drop_rt(t);
    if (t->handle) {
        if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r);
        g2r_image_destroy(S.r, t->handle, t->base);
        t->handle = 0;
    }
    for (l = 0; l < MAX_LEVELS; l++) if (!t->pend[0][l]) t->lkey[0][l] = 0;
    if (g2r_volume_create(S.r, &t->handle, &t->base, (uint32_t)w, (uint32_t)h, (uint32_t)d, fmt, (uint32_t)levels, G2_USE_SAMPLED | G2_USE_TRANSFER) < 0) { SET_ERR(GL_OUT_OF_MEMORY); return -1; }
    t->w = w; t->h = h; t->d = d; t->levels = levels; t->layers = 1; t->g2fmt = fmt;
    return 0;
}

/* px: tight rows of the storage, slice after slice, owned; NULL: no data */
static void tex3_image(int level, int w, int h, int d, uint32_t key, GLenum format, uint32_t fmt, uint8_t *px, uint32_t etf, int ebpp)
{
    es_tex *t = unit_tex(S.active, 2);
    int l, s, some = w && h && d;
    free_pend(t, 0, level);
    if (level == 0) {
        if (some && (!t->handle || t->g2fmt != fmt || t->w != w || t->h != h || t->d != d) && tex3_storage(t, w, h, d, fmt) < 0) { free(px); return; }
        t->format = format; t->compressed = 0;
    }
    t->lkey[0][level] = some ? key : 0;
    t->lw[0][level] = w; t->lh[0][level] = h; t->ld[level] = d;
    if (level == 0) {
        for (l = 1; l < MAX_LEVELS; l++) {
            es_pend *pd = t->pend[0][l];
            if (pd && level3_fits(t, l, t->lw[0][l], t->lh[0][l], t->ld[l], pd->fmt)) {
                t->pend[0][l] = NULL;
                tex3_write(t, l, 0, 0, 0, t->lw[0][l], t->lh[0][l], t->ld[l], pd->etf, pd->ebpp, pd->px);
                free(pd->px); free(pd);
            }
        }
    }
    if (!level3_fits(t, level, w, h, d, fmt)) {
        if (px) {
            es_pend *pd = (es_pend *)malloc(sizeof *pd);
            if (!pd) { SET_ERR(GL_OUT_OF_MEMORY); free(px); return; }
            pd->px = px; pd->fmt = fmt; pd->etf = etf; pd->ebpp = ebpp;
            t->pend[0][level] = pd;
        }
        return;
    }
    if (px) {
        tex3_write(t, level, 0, 0, 0, w, h, d, etf, ebpp, px);
        free(px);
    } else if (format == GL_RGB && fmt == G2_FMT_RGBA8 && some) {
        if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r);
        for (s = 0; s < d; s++) g2_fill_image(S.bd, t->handle, G2_TF_RAW, 0x000000FFu, 0, 0, (uint32_t)w, (uint32_t)h, MASK_A, (uint32_t)level, (uint32_t)s);
    }
}

void ES_APIENTRY glTexImage3DOES(GLenum target, GLint level, GLenum ifmt, GLsizei w, GLsizei h, GLsizei d, GLint border, GLenum format, GLenum type, const void *pixels)
{
    uint32_t fmt, etf = 0;
    int ebpp = 0;
    uint8_t *px = NULL;
    GLenum e;
    if (target != GL_TEXTURE_3D_OES || !has_3d()) { SET_ERR(GL_INVALID_ENUM); return; }
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || d < 0 || w > G2R_VOL_WH >> level || h > G2R_VOL_WH >> level || d > G2R_VOL_D >> level || border) { SET_ERR(GL_INVALID_VALUE); return; }
    if ((e = format_type_error(format, type))) { SET_ERR(e); return; }
    fmt = natural_fmt(format, type);
    if (ifmt != format) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (!S.live) return;
    if (unit_tex(S.active, 2)->immutable && !S.tex_storing) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (pixels && w && h && d && !(px = to_storage(pixels, w, h * d, format, type, fmt, &etf, &ebpp))) return;   /* the rows of a slice run on into the next one's */
    tex3_image(level, w, h, d, (uint32_t)format | (uint32_t)type << 16, format, fmt, px, etf, ebpp);
}

void ES_APIENTRY glTexSubImage3DOES(GLenum target, GLint level, GLint x, GLint y, GLint z, GLsizei w, GLsizei h, GLsizei d, GLenum format, GLenum type, const void *pixels)
{
    uint32_t etf;
    int ebpp;
    uint8_t *px;
    es_tex *t;
    GLenum e;
    if (target != GL_TEXTURE_3D_OES || !has_3d()) { SET_ERR(GL_INVALID_ENUM); return; }
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || d < 0 || x < 0 || y < 0 || z < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if ((e = format_type_error(format, type))) { SET_ERR(e); return; }
    t = unit_tex(S.active, 2);
    if (!S.live) return;
    if (!t->lkey[0][level] || (t->lkey[0][level] & 0xFFFF) != format) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (x + w > t->lw[0][level] || y + h > t->lh[0][level] || z + d > t->ld[level]) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!w || !h || !d || !pixels) return;
    px = to_storage(pixels, w, h * d, format, type, t->pend[0][level] ? t->pend[0][level]->fmt : t->g2fmt, &etf, &ebpp);
    if (!px) return;
    tex3_write(t, level, x, y, z, w, h, d, etf, ebpp, px);
    free(px);
}

/* the extension has entry points for compressed volumes and this library no format they take */
void ES_APIENTRY glCompressedTexImage3DOES(GLenum target, GLint level, GLenum f, GLsizei w, GLsizei h, GLsizei d, GLint border, GLsizei size, const void *data) { (void)target; (void)level; (void)f; (void)w; (void)h; (void)d; (void)border; (void)size; (void)data; SET_ERR(GL_INVALID_ENUM); }
void ES_APIENTRY glCompressedTexSubImage3DOES(GLenum target, GLint level, GLint x, GLint y, GLint z, GLsizei w, GLsizei h, GLsizei d, GLenum f, GLsizei size, const void *data) { (void)target; (void)level; (void)x; (void)y; (void)z; (void)w; (void)h; (void)d; (void)f; (void)size; (void)data; SET_ERR(GL_INVALID_ENUM); }

/* the level below by a 2 x 2 x 2 box */
static void generate_3d(void)
{
    es_tex *t = unit_tex(S.active, 2);
    int l, bpp, n16, x, y, z, c, w, h, d;
    uint32_t tf;
    uint8_t *cur;
    if (!S.live) return;
    if (!t->handle || !t->lkey[0][0]) { SET_ERR(GL_INVALID_OPERATION); return; }
    w = t->w; h = t->h; d = t->d;
    bpp = t->g2fmt == G2_FMT_RGBA8 ? 4 : 2;
    n16 = bpp == 2 ? f16_of(t->g2fmt) : 0;
    tf = bpp == 4 ? G2_TF_RGBA8888 : G2_TF_RAW;
    tex_resolve(t);
    if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r);
    g2r_finish(S.r);
    cur = (uint8_t *)malloc((size_t)w * h * d * bpp);
    if (!cur) { SET_ERR(GL_OUT_OF_MEMORY); return; }
    for (z = 0; z < d; z++) g2_read_image(S.bd, t->handle, tf, 0, 0, (uint32_t)w, (uint32_t)h, 0, (uint32_t)z, t->g2fmt, 0, 0, cur + (size_t)z * w * h * bpp);
    for (l = 1; l < t->levels; l++) {
        int nw = w > 1 ? w / 2 : 1, nh = h > 1 ? h / 2 : 1, nd = d > 1 ? d / 2 : 1;
        uint8_t *nx = (uint8_t *)malloc((size_t)nw * nh * nd * bpp);
        if (!nx) { SET_ERR(GL_OUT_OF_MEMORY); break; }
        for (z = 0; z < nd; z++) for (y = 0; y < nh; y++) for (x = 0; x < nw; x++) {
            int i, j, k;
            size_t at[8];
            for (k = 0; k < 2; k++) for (j = 0; j < 2; j++) for (i = 0; i < 2; i++) at[k * 4 + j * 2 + i] = ((size_t)(d > 1 ? 2 * z + k : 0) * h + (h > 1 ? 2 * y + j : 0)) * w + (w > 1 ? 2 * x + i : 0);
            if (bpp == 4) {
                for (c = 0; c < 4; c++) {
                    int sum = 0;
                    for (i = 0; i < 8; i++) sum += cur[at[i] * 4 + c];
                    nx[(((size_t)z * nh + y) * nw + x) * 4 + c] = (uint8_t)((sum + 4) >> 3);
                }
            } else {
                const uint16_t *s = (const uint16_t *)cur;
                uint32_t v = 0;
                for (c = 0; c < 4; c++) {
                    uint32_t m = (1u << F16[n16].bits[c]) - 1, sum = 0;
                    int sh = F16[n16].sh[c];
                    for (i = 0; i < 8; i++) sum += s[at[i]] >> sh & m;
                    v |= ((sum + 4) >> 3) << sh;
                }
                ((uint16_t *)nx)[((size_t)z * nh + y) * nw + x] = (uint16_t)v;
            }
        }
        free_pend(t, 0, l);
        t->lkey[0][l] = t->lkey[0][0]; t->lw[0][l] = nw; t->lh[0][l] = nh; t->ld[l] = nd;
        tex3_write(t, l, 0, 0, 0, nw, nh, nd, tf, bpp, nx);
        free(cur);
        cur = nx; w = nw; h = nh; d = nd;
    }
    free(cur);
}

/* §3.7.11: the level below by a 2 x 2 box. B has no GEN_MIPMAP, so the base level comes back and the chain goes up */
void ES_APIENTRY glGenerateMipmap(GLenum target)
{
    int cube = target == GL_TEXTURE_CUBE_MAP, faces = cube ? 6 : 1, f, l, n16, bpp;
    es_tex *t;
    uint32_t tf;
    if (target == GL_TEXTURE_3D_OES && has_3d()) { generate_3d(); return; }
    if (target != GL_TEXTURE_2D && !cube) { SET_ERR(GL_INVALID_ENUM); return; }
    t = unit_tex(S.active, cube);
    if (!S.live) return;
    if (!t->handle || !t->lkey[0][0] || t->compressed || t->format == GL_DEPTH_COMPONENT || t->format == GL_DEPTH_STENCIL_OES) { SET_ERR(GL_INVALID_OPERATION); return; }
    for (f = 1; f < faces; f++) if (t->lkey[f][0] != t->lkey[0][0] || t->lw[f][0] != t->w) { SET_ERR(GL_INVALID_OPERATION); return; }
    bpp = t->g2fmt == G2_FMT_RGBA8 ? 4 : 2;
    n16 = bpp == 2 ? f16_of(t->g2fmt) : 0;
    tf = bpp == 4 ? G2_TF_RGBA8888 : G2_TF_RAW;
    tex_resolve(t);
    if (t->used_pass == g2r_pass_id(S.r)) g2r_end_pass(S.r);
    g2r_finish(S.r);
    for (f = 0; f < faces; f++) {
        int w = t->w, h = t->h;
        uint8_t *cur = (uint8_t *)malloc((size_t)w * h * bpp);
        if (!cur) { SET_ERR(GL_OUT_OF_MEMORY); return; }
        g2_read_image(S.bd, t->handle, tf, 0, 0, (uint32_t)w, (uint32_t)h, 0, (uint32_t)f, t->g2fmt, 0, 0, cur);
        for (l = 1; l < t->levels; l++) {
            int nw = w > 1 ? w / 2 : 1, nh = h > 1 ? h / 2 : 1, x, y;
            uint8_t *nx = (uint8_t *)malloc((size_t)nw * nh * bpp);
            if (!nx) { SET_ERR(GL_OUT_OF_MEMORY); break; }
            for (y = 0; y < nh; y++) {
                int y0 = h > 1 ? 2 * y : 0, y1 = h > 1 ? 2 * y + 1 : 0;
                for (x = 0; x < nw; x++) {
                    int x0 = w > 1 ? 2 * x : 0, x1 = w > 1 ? 2 * x + 1 : 0, c;
                    if (bpp == 4) {
                        for (c = 0; c < 4; c++) {
                            nx[(y * nw + x) * 4 + c] = (uint8_t)((cur[(y0 * w + x0) * 4 + c] + cur[(y0 * w + x1) * 4 + c] + cur[(y1 * w + x0) * 4 + c] + cur[(y1 * w + x1) * 4 + c] + 2) >> 2);
                        }
                    } else {
                        const uint16_t *s = (const uint16_t *)cur;
                        uint32_t v = 0;
                        for (c = 0; c < 4; c++) {
                            uint32_t m = (1u << F16[n16].bits[c]) - 1;
                            int sh = F16[n16].sh[c];
                            v |= (((s[y0 * w + x0] >> sh & m) + (s[y0 * w + x1] >> sh & m) + (s[y1 * w + x0] >> sh & m) + (s[y1 * w + x1] >> sh & m) + 2) >> 2) << sh;
                        }
                        ((uint16_t *)nx)[y * nw + x] = (uint16_t)v;
                    }
                }
            }
            free_pend(t, f, l);
            g2_write_image(S.bd, t->handle, nx, (size_t)nw * nh * bpp, tf, 0, 0, (uint32_t)nw, (uint32_t)nh, (uint32_t)l, (uint32_t)f, 0, t->g2fmt);
            t->lkey[f][l] = t->lkey[f][0]; t->lw[f][l] = nw; t->lh[f][l] = nh;
            free(cur);
            cur = nx; w = nw; h = nh;
        }
        free(cur);
    }
}

/* the colour target's rectangle as RGBA8; alpha 255 where the target has none */
static int read_rgba8(const es_tg *g, int x, int y, int w, int h, uint8_t *out)
{
    int i, n = w * h, rc, f;
    uint16_t *tmp;
    if (g->kx != 1.0 || g->ky != 1.0) {                         /* a window rendered smaller: the nearest pixel of what exists */
        int rx0 = (int)floor(x * g->kx), ry0 = (int)floor(y * g->ky), rx1 = (int)ceil((x + w) * g->kx), ry1 = (int)ceil((y + h) * g->ky), r, c, rw, rh;
        uint8_t *big;
        if (rx1 > (int)g->t.width) rx1 = (int)g->t.width;
        if (ry1 > (int)g->t.height) ry1 = (int)g->t.height;
        rw = rx1 - rx0; rh = ry1 - ry0;
        big = (uint8_t *)malloc((size_t)rw * rh * 4);
        if (!big) return -1;
        rc = g2_read_image(S.bd, g->t.color, G2_TF_RGBA8888, (uint32_t)rx0, (uint32_t)ry0, (uint32_t)rw, (uint32_t)rh, 0, 0, G2_FMT_RGBA8, 0, 0, big);
        for (r = 0; r < h; r++) {
            int sy = (int)((y + r + 0.5) * g->ky) - ry0;
            if (sy >= rh) sy = rh - 1;
            for (c = 0; c < w; c++) {
                int sx = (int)((x + c + 0.5) * g->kx) - rx0;
                if (sx >= rw) sx = rw - 1;
                memcpy(out + ((size_t)r * w + c) * 4, big + ((size_t)sy * rw + sx) * 4, 4);
            }
        }
        free(big);
        return rc;
    }
    if (g->fmt == G2_FMT_RGBA8) return g2_read_image(S.bd, g->t.color, G2_TF_RGBA8888, (uint32_t)x, (uint32_t)y, (uint32_t)w, (uint32_t)h, 0, 0, G2_FMT_RGBA8, 0, 0, out);
    tmp = (uint16_t *)malloc((size_t)n * 2);
    if (!tmp) return -1;
    rc = g2_read_image(S.bd, g->t.color, G2_TF_RAW, (uint32_t)x, (uint32_t)y, (uint32_t)w, (uint32_t)h, 0, 0, g->fmt, 0, 0, tmp);
    f = f16_of(g->fmt);
    for (i = 0; i < n; i++) unpack16(tmp[i], f, out + 4 * i);
    free(tmp);
    return rc;
}

/* §4.4.5: a command that reads the framebuffer reads none that is not complete. g: what it would read */
static int read_fb_target(es_tg *g)
{
    GLuint draw_fb = S.fb;
    GLenum st;
    S.fb = S.read_fb;
    st = fb_target(g);
    S.fb = draw_fb;
    if (st == GL_FRAMEBUFFER_COMPLETE) return 1;
    SET_ERR(GL_INVALID_FRAMEBUFFER_OPERATION);
    return 0;
}
static int read_fb_complete(void) { es_tg g; return read_fb_target(&g); }

/* table 3.6: which channels of the framebuffer a copy takes */
static void copy_into(es_tex *t, int level, int face, int xo, int yo, int x, int y, int w, int h, GLenum tformat)
{
    es_tg g;
    int i, bound;
    GLuint draw_fb = S.fb;
    int vol = t->target == GL_TEXTURE_3D_OES;                   /* face: the slice */
    es_pend *pd = t->pend[vol ? 0 : face][level];
    uint32_t dst_fmt = pd ? pd->fmt : t->g2fmt;
    S.fb = S.read_fb;
    bound = bind_target(&g);
    S.fb = draw_fb;
    if (bound < 0) return;
    if ((tformat == GL_ALPHA || tformat == GL_LUMINANCE_ALPHA || tformat == GL_RGBA || tformat == GL_BGRA_EXT) && (g.no_alpha || g.fmt == G2_FMT_RGB565)) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (x < 0) { xo -= x; w += x; x = 0; }
    if (y < 0) { yo -= y; h += y; y = 0; }
    if (x + w > g.vw) w = g.vw - x;
    if (y + h > g.vh) h = g.vh - y;
    if (w <= 0 || h <= 0) return;
    if (g.tex != t) tex_resolve(t);
    g2r_end_pass(S.r);
    t->used_pass = 0xFFFFFFFFu;
    if (t->rt) t->rt_stale = 1;
    if (!vol && g.fmt == dst_fmt && !pd && tformat != GL_LUMINANCE && tformat != GL_LUMINANCE_ALPHA && tformat != GL_ALPHA) {
        int sx0 = (int)floor(x * g.kx), sy0 = (int)floor(y * g.ky), sx1 = (int)ceil((x + w) * g.kx), sy1 = (int)ceil((y + h) * g.ky);
        if (sx1 > (int)g.t.width) sx1 = (int)g.t.width;
        if (sy1 > (int)g.t.height) sy1 = (int)g.t.height;
        g2_copy_image(S.bd, g.t.color, t->handle, (uint32_t)sx0, (uint32_t)sy0, (uint32_t)(sx1 - sx0), (uint32_t)(sy1 - sy0), (uint32_t)xo, (uint32_t)yo, (uint32_t)w, (uint32_t)h, 0, 0, 0, 0, 0, 0, 0, (uint32_t)level, (uint32_t)face);
        if (tformat == GL_RGB && dst_fmt == G2_FMT_RGBA8) g2_fill_image(S.bd, t->handle, G2_TF_RAW, 0x000000FFu, (uint32_t)xo, (uint32_t)yo, (uint32_t)w, (uint32_t)h, MASK_A, (uint32_t)level, (uint32_t)face);
    } else {
        uint8_t *px = (uint8_t *)malloc((size_t)w * h * 4), *st;
        uint32_t etf;
        int ebpp, un = S.un_align;
        if (!px) { SET_ERR(GL_OUT_OF_MEMORY); return; }
        g2r_finish(S.r);
        if (read_rgba8(&g, x, y, w, h, px) < 0) { free(px); return; }
        for (i = 0; i < w * h; i++) {
            uint8_t *p = px + 4 * i;
            switch (tformat) {
            case GL_RGB: p[3] = 255; break;
            case GL_LUMINANCE: p[1] = p[2] = p[0]; p[3] = 255; break;
            case GL_LUMINANCE_ALPHA: p[1] = p[2] = p[0]; break;
            case GL_ALPHA: p[0] = p[1] = p[2] = 0; break;
            default: break;
            }
        }
        S.un_align = 1;
        st = to_storage(px, w, h, GL_RGBA, GL_UNSIGNED_BYTE, dst_fmt, &etf, &ebpp);
        S.un_align = un;
        if (st && vol) tex3_write(t, level, xo, yo, face, w, h, 1, etf, ebpp, st);
        else if (st) tex_write(t, level, face, xo, yo, w, h, etf, ebpp, st);
        free(st);
        free(px);
    }
}

void ES_APIENTRY glCopyTexSubImage3DOES(GLenum target, GLint level, GLint xo, GLint yo, GLint zo, GLint x, GLint y, GLsizei w, GLsizei h)
{
    es_tex *t;
    if (target != GL_TEXTURE_3D_OES || !has_3d()) { SET_ERR(GL_INVALID_ENUM); return; }
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || xo < 0 || yo < 0 || zo < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    t = unit_tex(S.active, 2);
    if (!S.live) return;
    if (!read_fb_complete()) return;
    if (!t->lkey[0][level]) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (xo + w > t->lw[0][level] || yo + h > t->lh[0][level] || zo >= t->ld[level]) { SET_ERR(GL_INVALID_VALUE); return; }
    if (w && h) copy_into(t, level, zo, xo, yo, x, y, w, h, t->lkey[0][level] & 0xFFFF);
}

void ES_APIENTRY glCopyTexSubImage2D(GLenum target, GLint level, GLint xo, GLint yo, GLint x, GLint y, GLsizei w, GLsizei h)
{
    es_tex *t;
    int face = face_of(target);
    if (!image_target(target)) return;
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || xo < 0 || yo < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    t = unit_tex(S.active, is_face(target));
    if (!S.live) return;
    if (!read_fb_complete()) return;
    if (!t->lkey[face][level] || (t->lkey[face][level] >> 16) == 0) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (xo + w > t->lw[face][level] || yo + h > t->lh[face][level]) { SET_ERR(GL_INVALID_VALUE); return; }
    if (w && h) copy_into(t, level, face, xo, yo, x, y, w, h, t->lkey[face][level] & 0xFFFF);
}

void ES_APIENTRY glCopyTexImage2D(GLenum target, GLint level, GLenum ifmt, GLint x, GLint y, GLsizei w, GLsizei h, GLint border)
{
    es_tex *t;
    if (!image_target(target)) return;
    if (ifmt != GL_ALPHA && ifmt != GL_LUMINANCE && ifmt != GL_LUMINANCE_ALPHA && ifmt != GL_RGB && ifmt != GL_RGBA) { SET_ERR(GL_INVALID_ENUM); return; }
    if (level < 0 || level >= MAX_LEVELS || w < 0 || h < 0 || w > MAX_TEX >> level || h > MAX_TEX >> level || border || (is_face(target) && w != h)) { SET_ERR(GL_INVALID_VALUE); return; }
    if (S.live) {                                               /* before the image is defined: a command that fails changes nothing */
        es_tg g;
        if (!read_fb_target(&g)) return;
        if ((ifmt == GL_ALPHA || ifmt == GL_LUMINANCE_ALPHA || ifmt == GL_RGBA) && (g.no_alpha || g.fmt == G2_FMT_RGB565)) { SET_ERR(GL_INVALID_OPERATION); return; }    /* table 3.6 */
    }
    {
        GLenum before = S.error, now;
        S.error = GL_NO_ERROR;
        glTexImage2D(target, level, (GLint)ifmt, w, h, border, ifmt, GL_UNSIGNED_BYTE, NULL);
        now = S.error;
        S.error = before != GL_NO_ERROR ? before : now;
        if (now != GL_NO_ERROR || !S.live || !w || !h) return;
    }
    t = unit_tex(S.active, is_face(target));
    copy_into(t, level, face_of(target), 0, 0, x, y, w, h, ifmt);
}

/* §3.8.2 */
static int tex_complete(const es_tex *t)
{
    int faces = t->target == GL_TEXTURE_CUBE_MAP ? 6 : 1, f, l, n;
    int mip = t->min_f != GL_NEAREST && t->min_f != GL_LINEAR;
    uint32_t key = t->lkey[0][0];
    if (!t->handle || !key) return 0;
    if (faces == 6 && t->w != t->h) return 0;
    if (t->format == GL_DEPTH_COMPONENT || t->format == GL_DEPTH_STENCIL_OES) return t->mag_f == GL_NEAREST && (t->min_f == GL_NEAREST || t->min_f == GL_NEAREST_MIPMAP_NEAREST);    /* OES_depth_texture */
    if (t->target == GL_TEXTURE_3D_OES) {
        if (t->lw[0][0] != t->w || t->lh[0][0] != t->h || t->ld[0] != t->d) return 0;
        n = t->immutable && t->immutable < t->levels ? t->immutable : t->levels;
        if (mip) for (l = 1; l < n; l++) if (t->lkey[0][l] != key || t->lw[0][l] != mip_dim(t->w, l) || t->lh[0][l] != mip_dim(t->h, l) || t->ld[l] != mip_dim(t->d, l) || t->pend[0][l]) return 0;
        return 1;
    }
    n = 1 + ilog2(t->w > t->h ? t->w : t->h);
    if (t->immutable && t->immutable < n) n = t->immutable;     /* EXT_texture_storage: as many levels as it was given */
    for (f = 0; f < faces; f++) {
        if (t->lkey[f][0] != key || t->lw[f][0] != t->w || t->lh[f][0] != t->h) return 0;
        if (mip) for (l = 1; l < n && l < MAX_LEVELS; l++) if (t->lkey[f][l] != key || t->lw[f][l] != mip_dim(t->w, l) || t->lh[f][l] != mip_dim(t->h, l) || t->pend[f][l]) return 0;
    }
    return 1;
}

/* ---------------------------------------------------------------- renderbuffers, framebuffers */
void ES_APIENTRY glGenRenderbuffers(GLsizei n, GLuint *b) { GLsizei i; if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; } for (i = 0; i < n; i++) b[i] = gen_name(); }
GLboolean ES_APIENTRY glIsRenderbuffer(GLuint b) { return b && rb_of(b); }
void ES_APIENTRY glBindRenderbuffer(GLenum t, GLuint b)
{
    if (t != GL_RENDERBUFFER) { SET_ERR(GL_INVALID_ENUM); return; }
    if (b && !rb_of(b)) { es_rb *o; NEW(rbs, b, es_rb, o); (void)o; }
    S.rb = b;
}
void ES_APIENTRY glDeleteRenderbuffers(GLsizei n, const GLuint *b)
{
    GLsizei i;
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        es_rb *x;
        if (!b[i] || !(x = rb_of(b[i]))) continue;
        if (S.rb == b[i]) S.rb = 0;
        detach_bound(NULL, x);
        UNLINK(rbs, b[i], es_rb, x);
        if (x->att_refs) x->orphan = 1;
        else { rb_free(x); free(x); }
    }
}
void ES_APIENTRY glRenderbufferStorage(GLenum t, GLenum ifmt, GLsizei w, GLsizei h)
{
    es_rb *b = S.rb ? rb_of(S.rb) : NULL;
    uint32_t f;
    if (t != GL_RENDERBUFFER) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!b) { SET_ERR(GL_INVALID_OPERATION); return; }
    switch (ifmt) {
    case GL_RGBA4: f = G2_FMT_RGBA4444; break;
    case GL_RGB5_A1: f = G2_FMT_RGBA5551; break;
    case GL_RGB565: f = G2_FMT_RGB565; break;
    case GL_RGBA8: case GL_RGB8: f = G2_FMT_RGBA8; break;
    case GL_BGRA_EXT: case GL_BGRA8_EXT: f = G2_FMT_RGBA8; break;     /* EXT_texture_format_BGRA8888's colour-renderable formats of table 4.5; the order of the bytes in memory is no renderbuffer's business */
    case GL_DEPTH_COMPONENT16: case GL_DEPTH_COMPONENT24: case GL_STENCIL_INDEX8: case GL_DEPTH24_STENCIL8: f = G2_FMT_D24S8; break;
    default: SET_ERR(GL_INVALID_ENUM); return;
    }
    if (w < 0 || h < 0 || w > MAX_RT_W || h > MAX_RT_W) { SET_ERR(GL_INVALID_VALUE); return; }     /* both against MAX_RENDERBUFFER_SIZE */
    if (!S.live) return;
    if (b->handle) { g2r_end_pass(S.r); g2r_image_destroy(S.r, b->handle, b->base); b->handle = 0; }
    b->ifmt = ifmt; b->w = w; b->h = h; b->g2fmt = f;
    if (!w || !h) return;
    if (g2r_image_create(S.r, &b->handle, &b->base, (uint32_t)w, (uint32_t)h, f, 1, 1, 1, G2_USE_RT | G2_USE_TRANSFER) < 0) { SET_ERR(GL_OUT_OF_MEMORY); return; }
    if (f == G2_FMT_D24S8) g2r_stencil_undefined(S.r, b->handle);
    if (ifmt == GL_RGB8) g2_fill_image(S.bd, b->handle, G2_TF_RAW, 0x000000FFu, 0, 0, (uint32_t)w, (uint32_t)h, 0xF, 0, 0);     /* stored alpha: destination alpha must read 1 */
}
void ES_APIENTRY glGetRenderbufferParameteriv(GLenum t, GLenum p, GLint *v)
{
    es_rb *b = S.rb ? rb_of(S.rb) : NULL;
    if (t != GL_RENDERBUFFER) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!b) { SET_ERR(GL_INVALID_OPERATION); return; }
    switch (p) {
    case GL_RENDERBUFFER_WIDTH: *v = b->w; break;
    case GL_RENDERBUFFER_HEIGHT: *v = b->h; break;
    case GL_RENDERBUFFER_INTERNAL_FORMAT: *v = (GLint)(b->ifmt ? b->ifmt : GL_RGBA4); break;
    case GL_RENDERBUFFER_RED_SIZE: *v = b->g2fmt == G2_FMT_D24S8 || !b->ifmt ? 0 : rb_bits(b->g2fmt, 0); break;
    case GL_RENDERBUFFER_GREEN_SIZE: *v = b->g2fmt == G2_FMT_D24S8 || !b->ifmt ? 0 : rb_bits(b->g2fmt, 1); break;
    case GL_RENDERBUFFER_BLUE_SIZE: *v = b->g2fmt == G2_FMT_D24S8 || !b->ifmt ? 0 : rb_bits(b->g2fmt, 2); break;
    case GL_RENDERBUFFER_ALPHA_SIZE: *v = b->g2fmt == G2_FMT_D24S8 || !b->ifmt || b->ifmt == GL_RGB8 ? 0 : rb_bits(b->g2fmt, 3); break;
    case GL_RENDERBUFFER_DEPTH_SIZE: *v = b->ifmt == GL_DEPTH_COMPONENT16 ? 16 : depth_rb(b->ifmt) ? 24 : 0; break;
    case GL_RENDERBUFFER_STENCIL_SIZE: *v = stencil_rb(b->ifmt) ? 8 : 0; break;
    default: SET_ERR(GL_INVALID_ENUM); break;
    }
}
void ES_APIENTRY glGenFramebuffers(GLsizei n, GLuint *b) { GLsizei i; if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; } for (i = 0; i < n; i++) b[i] = gen_name(); }
GLboolean ES_APIENTRY glIsFramebuffer(GLuint b) { return b && fb_of(b); }
void ES_APIENTRY glBindFramebuffer(GLenum t, GLuint b)
{
    if (t != GL_FRAMEBUFFER && t != GL_READ_FRAMEBUFFER_ANGLE && t != GL_DRAW_FRAMEBUFFER_ANGLE) { SET_ERR(GL_INVALID_ENUM); return; }
    if (b && !fb_of(b)) { es_fb *o; NEW(fbs, b, es_fb, o); (void)o; }
    if (t != GL_READ_FRAMEBUFFER_ANGLE) S.fb = b;
    if (t != GL_DRAW_FRAMEBUFFER_ANGLE) S.read_fb = b;
}
void ES_APIENTRY glDeleteFramebuffers(GLsizei n, const GLuint *b)
{
    GLsizei i;
    int k;
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        es_fb *f;
        if (!b[i]) continue;
        if (S.fb == b[i]) S.fb = 0;
        if (S.read_fb == b[i]) S.read_fb = 0;
        UNLINK(fbs, b[i], es_fb, f);
        if (!f) continue;
        for (k = 0; k < 3; k++) att_release(&f->a[k]);
        if (f->blind && S.r) { g2r_end_pass(S.r); g2r_image_destroy(S.r, f->blind, f->blind_base); }
        free(f);
    }
}
GLenum ES_APIENTRY glCheckFramebufferStatus(GLenum t)
{
    es_tg g;
    if (t != GL_FRAMEBUFFER) { SET_ERR(GL_INVALID_ENUM); return 0; }
    return fb_target(&g);
}
void ES_APIENTRY glFramebufferTexture2D(GLenum t, GLenum att, GLenum tt, GLuint tex, GLint level)
{
    es_fb *f = S.fb ? fb_of(S.fb) : NULL;
    if (t != GL_FRAMEBUFFER) { SET_ERR(GL_INVALID_ENUM); return; }
    if (att != GL_COLOR_ATTACHMENT0 && att != GL_DEPTH_ATTACHMENT && att != GL_STENCIL_ATTACHMENT) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!f) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (tex) {
        es_tex *x = tex_of(tex);
        if (tt != GL_TEXTURE_2D && !is_face(tt)) { SET_ERR(GL_INVALID_ENUM); return; }
        if (!x || x->target == GL_TEXTURE_3D_OES || (x->target == GL_TEXTURE_2D) != (tt == GL_TEXTURE_2D)) { SET_ERR(GL_INVALID_OPERATION); return; }
        if (level < 0 || level >= MAX_LEVELS || (level && att != GL_COLOR_ATTACHMENT0)) { SET_ERR(GL_INVALID_VALUE); return; }     /* OES_fbo_render_mipmap */
    }
    att_set(&f->a[att_index(att)], GL_TEXTURE, tex, tex ? tex_of(tex) : NULL, NULL, tt, level, 0);
}
void ES_APIENTRY glFramebufferTexture3DOES(GLenum t, GLenum att, GLenum tt, GLuint tex, GLint level, GLint z)
{
    es_fb *f = S.fb ? fb_of(S.fb) : NULL;
    if (t != GL_FRAMEBUFFER || !has_3d()) { SET_ERR(GL_INVALID_ENUM); return; }
    if (att != GL_COLOR_ATTACHMENT0 && att != GL_DEPTH_ATTACHMENT && att != GL_STENCIL_ATTACHMENT) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!f) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (tex) {
        es_tex *x = tex_of(tex);
        if (tt != GL_TEXTURE_3D_OES) { SET_ERR(GL_INVALID_ENUM); return; }
        if (!x || x->target != GL_TEXTURE_3D_OES) { SET_ERR(GL_INVALID_OPERATION); return; }
        if (level < 0 || level >= MAX_LEVELS || z < 0 || z >= G2R_VOL_D) { SET_ERR(GL_INVALID_VALUE); return; }
    }
    att_set(&f->a[att_index(att)], GL_TEXTURE, tex, tex ? tex_of(tex) : NULL, NULL, tt, level, z);
}
void ES_APIENTRY glFramebufferRenderbuffer(GLenum t, GLenum att, GLenum rt, GLuint rb)
{
    es_fb *f = S.fb ? fb_of(S.fb) : NULL;
    if (t != GL_FRAMEBUFFER || (rt != GL_RENDERBUFFER && rb)) { SET_ERR(GL_INVALID_ENUM); return; }
    if (att != GL_COLOR_ATTACHMENT0 && att != GL_DEPTH_ATTACHMENT && att != GL_STENCIL_ATTACHMENT) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!f || (rb && !rb_of(rb))) { SET_ERR(GL_INVALID_OPERATION); return; }
    att_set(&f->a[att_index(att)], GL_RENDERBUFFER, rb, NULL, rb ? rb_of(rb) : NULL, 0, 0, 0);
}
void ES_APIENTRY glGetFramebufferAttachmentParameteriv(GLenum t, GLenum att, GLenum p, GLint *v)
{
    es_fb *f = S.fb ? fb_of(S.fb) : NULL;
    const es_att *a;
    if (t != GL_FRAMEBUFFER) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!f) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (att != GL_COLOR_ATTACHMENT0 && att != GL_DEPTH_ATTACHMENT && att != GL_STENCIL_ATTACHMENT) { SET_ERR(GL_INVALID_ENUM); return; }
    a = &f->a[att_index(att)];
    switch (p) {
    case GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE: *v = a->type ? (GLint)a->type : GL_NONE; break;
    case GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME: if (!a->type) SET_ERR(GL_INVALID_ENUM); else *v = (GLint)a->name; break;
    case GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL: if (a->type != GL_TEXTURE) SET_ERR(GL_INVALID_ENUM); else *v = a->lvl; break;
    case GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_CUBE_MAP_FACE: if (a->type != GL_TEXTURE) SET_ERR(GL_INVALID_ENUM); else *v = is_face(a->face) ? (GLint)a->face : 0; break;
    case GL_FRAMEBUFFER_ATTACHMENT_TEXTURE_3D_ZOFFSET_OES: if (a->type != GL_TEXTURE || !has_3d()) SET_ERR(GL_INVALID_ENUM); else *v = a->z; break;
    default: SET_ERR(GL_INVALID_ENUM); break;
    }
}

/* ---------------------------------------------------------------- shaders */
GLuint ES_APIENTRY glCreateShader(GLenum type)
{
    GLuint n;
    es_shader *s;
    if (type != GL_VERTEX_SHADER && type != GL_FRAGMENT_SHADER) { SET_ERR(GL_INVALID_ENUM); return 0; }
    n = gen_name();
    NEW(shaders, n, es_shader, s);
    if (s) s->type = type;
    return n;
}
static void shader_free(GLuint n) { DEL(shaders, n, es_shader, { free(x_->src); free(x_->compiled_src); }); }
void ES_APIENTRY glDeleteShader(GLuint n)
{
    es_shader *s;
    if (!n) return;
    if (!(s = shader_arg(n))) return;
    s->del = 1;
    if (!s->refs) shader_free(n);
}
GLboolean ES_APIENTRY glIsShader(GLuint n) { return n && sh_of(n); }
void ES_APIENTRY glShaderSource(GLuint n, GLsizei count, const GLchar *const *str, const GLint *len)
{
    es_shader *s = shader_arg(n);
    size_t total = 0, o = 0;
    GLsizei i;
    if (!s) return;
    if (count < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < count; i++) total += len && len[i] >= 0 ? (size_t)len[i] : strlen(str[i]);
    free(s->src);
    s->src = (char *)malloc(total + 1);
    if (!s->src) { SET_ERR(GL_OUT_OF_MEMORY); return; }
    for (i = 0; i < count; i++) {
        size_t l = len && len[i] >= 0 ? (size_t)len[i] : strlen(str[i]);
        memcpy(s->src + o, str[i], l);
        o += l;
    }
    s->src[o] = 0;
}
void ES_APIENTRY glCompileShader(GLuint n)
{
    es_shader *s = shader_arg(n);
    g2sc_result r;
    if (!s) return;
    s->compiled = 0;
    if (!s->src) { snprintf(s->log, sizeof s->log, "no source"); return; }
    memset(&r, 0, sizeof r);
    s->compiled = g2sc_compile_strict(s->src, s->type == GL_VERTEX_SHADER ? G2SC_VS : G2SC_FS, &r) == 0;
    snprintf(s->log, sizeof s->log, "%s", r.error);
    if (s->compiled) {
        free(s->compiled_src);
        s->compiled_src = (char *)malloc(strlen(s->src) + 1);
        if (s->compiled_src) strcpy(s->compiled_src, s->src);
    } else {
        g2e_log("gles: shader does not compile: %s\n%s", r.error, s->src);
    }
    g2sc_free(&r);
}
void ES_APIENTRY glGetShaderiv(GLuint n, GLenum p, GLint *v)
{
    es_shader *s = shader_arg(n);
    if (!s) return;
    switch (p) {
    case GL_SHADER_TYPE: *v = (GLint)s->type; break;
    case GL_DELETE_STATUS: *v = s->del; break;
    case GL_COMPILE_STATUS: *v = s->compiled; break;
    case GL_INFO_LOG_LENGTH: *v = s->log[0] ? (GLint)strlen(s->log) + 1 : 0; break;
    case GL_SHADER_SOURCE_LENGTH: *v = s->src ? (GLint)strlen(s->src) + 1 : 0; break;
    case GL_TRANSLATED_SHADER_SOURCE_LENGTH_ANGLE: *v = s->compiled_src ? (GLint)strlen(s->compiled_src) + 1 : 0; break;
    default: SET_ERR(GL_INVALID_ENUM); break;
    }
}
void ES_APIENTRY glGetShaderInfoLog(GLuint n, GLsizei max, GLsizei *len, GLchar *log)
{
    es_shader *s = shader_arg(n);
    int l;
    if (!s) return;
    if (max < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!max) { if (len) *len = 0; return; }
    l = snprintf(log, (size_t)max, "%s", s->log);
    if (len) *len = l < max ? l : max - 1;
}
void ES_APIENTRY glGetShaderSource(GLuint n, GLsizei max, GLsizei *len, GLchar *src)
{
    es_shader *s = shader_arg(n);
    int l;
    if (!s) return;
    if (max < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!max) { if (len) *len = 0; return; }
    l = snprintf(src, (size_t)max, "%s", s->src ? s->src : "");
    if (len) *len = l < max ? l : max - 1;
}
void ES_APIENTRY glReleaseShaderCompiler(void) {}
void ES_APIENTRY glShaderBinary(GLsizei c, const GLuint *s, GLenum f, const void *b, GLsizei l) { (void)c; (void)s; (void)f; (void)b; (void)l; SET_ERR(GL_INVALID_ENUM); }
void ES_APIENTRY glGetShaderPrecisionFormat(GLenum st, GLenum pt, GLint *range, GLint *precision)
{
    if (st != GL_VERTEX_SHADER && st != GL_FRAGMENT_SHADER) { SET_ERR(GL_INVALID_ENUM); return; }
    switch (pt) {
    case GL_LOW_INT: case GL_MEDIUM_INT: case GL_HIGH_INT: range[0] = range[1] = 16; *precision = 0; break;     /* integers live in FP24: exact to 2^17 */
    case GL_LOW_FLOAT: case GL_MEDIUM_FLOAT: case GL_HIGH_FLOAT: range[0] = range[1] = 62; *precision = 16; break;  /* FP24: e7 m16 */
    default: SET_ERR(GL_INVALID_ENUM); break;
    }
}

/* ---------------------------------------------------------------- programs */
GLuint ES_APIENTRY glCreateProgram(void) { GLuint n = gen_name(); es_prog *p; NEW(progs, n, es_prog, p); (void)p; return n; }
GLboolean ES_APIENTRY glIsProgram(GLuint n) { return n && pr_of(n); }
static void prog_free(es_prog *p)
{
    GLuint n = p->name;
    if (p->vs) { es_shader *s = sh_of(p->vs); if (s && !--s->refs && s->del) shader_free(p->vs); }
    if (p->fs) { es_shader *s = sh_of(p->fs); if (s && !--s->refs && s->del) shader_free(p->fs); }
    DEL(progs, n, es_prog, { free(x_->uni); free(x_->uval); free(x_->attr); free(x_->bin_vs); free(x_->bin_fs); });
}
void ES_APIENTRY glDeleteProgram(GLuint n)
{
    es_prog *p;
    if (!n) return;
    if (!(p = prog_arg(n))) return;
    p->del = 1;
    if (S.cur != p) prog_free(p);                               /* else when it stops being in use */
}
void ES_APIENTRY glAttachShader(GLuint pn, GLuint sn)
{
    es_prog *p = prog_arg(pn);
    es_shader *s;
    if (!p || !(s = shader_arg(sn))) return;
    if ((s->type == GL_VERTEX_SHADER ? p->vs : p->fs)) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (s->type == GL_VERTEX_SHADER) p->vs = sn; else p->fs = sn;
    s->refs++;
}
void ES_APIENTRY glDetachShader(GLuint pn, GLuint sn)
{
    es_prog *p = prog_arg(pn);
    es_shader *s;
    if (!p || !(s = shader_arg(sn))) return;
    if (p->vs == sn) p->vs = 0; else if (p->fs == sn) p->fs = 0; else { SET_ERR(GL_INVALID_OPERATION); return; }
    if (!--s->refs && s->del) shader_free(sn);
}
void ES_APIENTRY glGetAttachedShaders(GLuint pn, GLsizei max, GLsizei *n, GLuint *out)
{
    es_prog *p = prog_arg(pn);
    int c = 0;
    if (!p) return;
    if (max < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (p->vs && c < max) out[c++] = p->vs;
    if (p->fs && c < max) out[c++] = p->fs;
    if (n) *n = c;
}
void ES_APIENTRY glBindAttribLocation(GLuint pn, GLuint loc, const GLchar *name)
{
    es_prog *p = prog_arg(pn);
    int i;
    if (!p) return;
    if (loc >= MAX_ATTR) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!strncmp(name, "gl_", 3)) { SET_ERR(GL_INVALID_OPERATION); return; }
    for (i = 0; i < p->nbind; i++) if (!strcmp(p->bind[i].name, name)) break;
    if (i == p->nbind) {
        if (p->nbind == (int)(sizeof p->bind / sizeof p->bind[0])) return;
        p->nbind++;
    }
    snprintf(p->bind[i].name, sizeof p->bind[i].name, "%s", name);
    p->bind[i].loc = loc;
}

static GLenum type_enum(const char *t)
{
    static const struct { const char *n; GLenum e; } T[] = { { "float", GL_FLOAT }, { "vec2", GL_FLOAT_VEC2 }, { "vec3", GL_FLOAT_VEC3 }, { "vec4", GL_FLOAT_VEC4 }, { "int", GL_INT }, { "ivec2", GL_INT_VEC2 }, { "ivec3", GL_INT_VEC3 },
        { "ivec4", GL_INT_VEC4 }, { "bool", GL_BOOL }, { "bvec2", GL_BOOL_VEC2 }, { "bvec3", GL_BOOL_VEC3 }, { "bvec4", GL_BOOL_VEC4 }, { "mat2", GL_FLOAT_MAT2 }, { "mat3", GL_FLOAT_MAT3 }, { "mat4", GL_FLOAT_MAT4 },
        { "sampler2D", GL_SAMPLER_2D }, { "samplerCube", GL_SAMPLER_CUBE }, { "sampler3D", GL_SAMPLER_3D_OES } };
    size_t i;
    for (i = 0; i < sizeof T / sizeof T[0]; i++) if (!strcmp(T[i].n, t)) return T[i].e;
    return 0;
}

/* slots: registers of a matrix; comps: floats per register */
static void type_shape(GLenum e, int *slots, int *comps)
{
    switch (e) {
    case GL_FLOAT_MAT2: *slots = 2; *comps = 2; break;
    case GL_FLOAT_MAT3: *slots = 3; *comps = 3; break;
    case GL_FLOAT_MAT4: *slots = 4; *comps = 4; break;
    case GL_FLOAT_VEC2: case GL_INT_VEC2: case GL_BOOL_VEC2: *slots = 1; *comps = 2; break;
    case GL_FLOAT_VEC3: case GL_INT_VEC3: case GL_BOOL_VEC3: *slots = 1; *comps = 3; break;
    case GL_FLOAT_VEC4: case GL_INT_VEC4: case GL_BOOL_VEC4: *slots = 1; *comps = 4; break;
    default: *slots = 1; *comps = 1; break;
    }
}

static es_uni *uni_add(es_prog *p, const char *name, const char *type, int size, int vs, int fs, char *err, size_t nerr)
{
    es_uni *u;
    GLenum e = type_enum(type);
    int i;
    for (i = 0; i < p->nuni; i++) {
        if (!strcmp(p->uni[i].name, name)) {                    /* the same uniform in the other stage */
            if (p->uni[i].type != e || p->uni[i].size != (size ? size : 1)) snprintf(err, nerr, "uniform %s differs between the stages", name);
            if (vs >= 0) p->uni[i].vs = vs;
            if (fs >= 0) p->uni[i].fs = fs;
            return &p->uni[i];
        }
    }
    if (p->nuni >= MAX_UNI) { snprintf(err, nerr, "too many uniforms"); return NULL; }
    u = &p->uni[p->nuni++];
    memset(u, 0, sizeof *u);
    snprintf(u->name, sizeof u->name, "%s", name);
    u->type = e;
    u->size = size ? size : 1;
    u->isarr = size > 0;
    u->vs = vs; u->fs = fs; u->sampler = -1; u->vc = u->fc = 0;
    type_shape(u->type, &u->stride, &u->comps);
    return u;
}

/* struct uniforms flatten to "s.member", "a[1].member": the names glGetUniformLocation is asked for */
static void add_member(es_prog *p, const g2sc_result *r, const char *prefix, const char *type, int count, int base, int stage, char *err, size_t nerr)
{
    int k, m, st;
    for (st = 0; st < r->nstructs; st++) if (!strcmp(r->structs[st].name, type)) break;
    if (st == r->nstructs) { uni_add(p, prefix, type, count, stage == 0 ? base : -1, stage == 1 ? base : -1, err, nerr); return; }
    for (k = 0; k < (count ? count : 1); k++) {
        const g2sc_struct *s = &r->structs[st];
        char name[96];
        int b = base + k * s->nslots;
        if (count) snprintf(name, sizeof name, "%s[%d]", prefix, k); else snprintf(name, sizeof name, "%s", prefix);
        for (m = 0; m < s->nmembers; m++) {
            char full[128];
            snprintf(full, sizeof full, "%s.%s", name, s->members[m].name);
            add_member(p, r, full, s->members[m].type, s->members[m].array, b + s->members[m].offset, stage, err, nerr);
        }
    }
}

static const char *sym_name(const g2sc_sym *y) { return y->lname ? y->lname : y->name; }
static const char *sym_type(const g2sc_sym *y) { return y->ltype ? y->ltype : y->type; }

void ES_APIENTRY glLinkProgram(GLuint pn)
{
    es_prog *p = prog_arg(pn);
    es_shader *vs, *fs;
    g2sc_result rv, rf;
    g2sc_linked l;
    uint32_t fsmask = 0, *uval;
    int *umv = NULL, *umf = NULL;
    int i, units = 0, nval, old_nuni;
    g2r_prog *hw;
    es_uni *old;
    es_attr *na = NULL;
    g2sc_bind gb[sizeof p->bind / sizeof p->bind[0]];
    char err[sizeof p->log];
    if (!p) return;
    p->linked = 0; p->validated = 0; err[0] = 0;
    p->ndv = 0;
    vs = p->vs ? sh_of(p->vs) : NULL; fs = p->fs ? sh_of(p->fs) : NULL;
    memset(&rv, 0, sizeof rv); memset(&rf, 0, sizeof rf);
    if (!vs || !fs) { snprintf(p->log, sizeof p->log, "a program needs a vertex and a fragment shader"); return; }
    if (!vs->compiled || !fs->compiled || !vs->compiled_src || !fs->compiled_src) { snprintf(p->log, sizeof p->log, "a shader did not compile"); return; }
    if (!S.r) { snprintf(p->log, sizeof p->log, "no current context"); return; }
    /* §2.10.4: the compiler places the active attributes, a bound one at its location (two may share one), the rest at the lowest free locations; a location is the vertex core's input */
    for (i = 0; i < p->nbind; i++) { gb[i].name = p->bind[i].name; gb[i].loc = (int)p->bind[i].loc; }
    if (g2sc_link_bound(vs->compiled_src, fs->compiled_src, gb, p->nbind, 0, &l) < 0) { snprintf(err, sizeof err, "%s", l.error); goto done; }
    rv = l.vs; rf = l.fs; umv = l.umap_vs; umf = l.umap_fs; fsmask = l.fsmask;
    if (rv.vs_tex) { snprintf(err, sizeof err, "the vertex stage has no texture units (MAX_VERTEX_TEXTURE_IMAGE_UNITS is 0)"); goto done; }    /* declared and unused is legal: Cocos2d declares them in every shader */
    for (i = 0, units = 0; i < rf.nsamplers; i++) units += rf.samplers[i].has_array ? rf.samplers[i].array : 1;
    if (units > MAX_UNITS) { snprintf(err, sizeof err, "%d samplers, the hardware has %d", units, MAX_UNITS); goto done; }
    na = (es_attr *)calloc((size_t)(rv.nattribs ? rv.nattribs : 1), sizeof *na);
    if (!na) { snprintf(err, sizeof err, "out of memory"); goto done; }
    for (i = 0; i < rv.nattribs; i++) {
        int c;
        snprintf(na[i].name, sizeof na[i].name, "%s", sym_name(&rv.attribs[i]));
        na[i].type = type_enum(sym_type(&rv.attribs[i]));
        type_shape(na[i].type, &na[i].n, &c);
        na[i].loc = rv.attribs[i].loc;
    }
    old = p->uni; old_nuni = p->nuni;
    p->uni = (es_uni *)calloc(MAX_UNI, sizeof *p->uni);
    if (!p->uni) { snprintf(err, sizeof err, "out of memory"); p->uni = old; goto done; }
    p->nuni = 0;
    p->dr_vs = p->dr_fs = -1;
    for (i = 0; i < MAX_UNITS; i++) p->ts[i] = -1;
    for (i = 0; i < rv.nuniforms; i++) {
        if (!strcmp(sym_name(&rv.uniforms[i]), "gl_DepthRange")) p->dr_vs = rv.uniforms[i].loc;
        add_member(p, &rv, sym_name(&rv.uniforms[i]), sym_type(&rv.uniforms[i]), rv.uniforms[i].array, rv.uniforms[i].loc, 0, err, sizeof err);
    }
    for (i = 0; i < rf.nuniforms; i++) {
        const char *un = sym_name(&rf.uniforms[i]);
        if (!strncmp(un, "_g2_ts", 6)) { if (atoi(un + 6) < MAX_UNITS) p->ts[atoi(un + 6)] = rf.uniforms[i].loc; continue; }    /* the compiler's own: filled at each draw */
        if (!strcmp(un, "gl_DepthRange")) p->dr_fs = rf.uniforms[i].loc;
        add_member(p, &rf, un, sym_type(&rf.uniforms[i]), rf.uniforms[i].array, rf.uniforms[i].loc, 1, err, sizeof err);
    }
    for (i = 0; i < rf.nsamplers; i++) {
        es_uni *u = uni_add(p, sym_name(&rf.samplers[i]), sym_type(&rf.samplers[i]), rf.samplers[i].has_array ? rf.samplers[i].array : 0, -1, -1, err, sizeof err);
        if (u) { u->sampler = rf.samplers[i].loc; u->stride = 1; u->comps = 1; }
    }
    if (err[0]) { free(p->uni); p->uni = old; p->nuni = old_nuni; goto done; }
    for (i = 0; umv && i < p->nuni; i++) if (p->uni[i].vs >= 0) { int at = umv[p->uni[i].vs]; p->uni[i].vs = at >> 2; p->uni[i].vc = at & 3; }
    for (i = 0; umf && i < p->nuni; i++) if (p->uni[i].fs >= 0) { int at = umf[p->uni[i].fs]; p->uni[i].fs = at >> 2; p->uni[i].fc = at & 3; }
    if (umv && p->dr_vs >= 0) p->dr_vs = umv[p->dr_vs] >> 2;
    if (umf && p->dr_fs >= 0) p->dr_fs = umf[p->dr_fs] >> 2;
    for (i = 0; umf && i < MAX_UNITS; i++) if (p->ts[i] >= 0) p->ts[i] = umf[p->ts[i]] >> 2;
    for (i = 0, nval = 0; i < p->nuni; i++) { p->uni[i].voff = nval; nval += p->uni[i].size * p->uni[i].stride * p->uni[i].comps; }
    uval = (uint32_t *)calloc((size_t)(nval ? nval : 1), 4);    /* §2.10.4: a link sets every uniform to 0 */
    if (!uval) { snprintf(err, sizeof err, "out of memory"); free(p->uni); p->uni = old; p->nuni = old_nuni; goto done; }
    hw = g2r_prog_create(S.r, rv.words, (uint32_t)rv.nwords, (const double (*)[4])rv.const_pool, (uint32_t)rv.nconst, (uint32_t)rv.const_base, rf.words, (uint32_t)rf.nwords,
                         (const double (*)[4])rf.const_pool, (uint32_t)rf.nconst, (uint32_t)rf.const_base, fsmask);
    if (!hw) { snprintf(err, sizeof err, "no room for the programs on the boards"); free(uval); free(p->uni); p->uni = old; p->nuni = old_nuni; goto done; }
    free(old);
    free(p->uval);
    p->uval = uval;
    free(p->attr);
    p->attr = na; na = NULL;
    p->nattr = rv.nattribs;
    memcpy(p->lbind, p->bind, sizeof p->lbind);
    p->nlbind = p->nbind;
    p->p = hw;
    p->nsamp = units;
    memset(p->samp_unit, 0, sizeof p->samp_unit);
    memset(p->vu, 0, sizeof p->vu); memset(p->fu, 0, sizeof p->fu);
    p->linked = 1;
    free(p->bin_vs); free(p->bin_fs);
    p->bin_vs = _strdup(vs->compiled_src); p->bin_fs = _strdup(fs->compiled_src);
done:
    free(na);
    free(umv); free(umf);
    g2sc_free(&rv); g2sc_free(&rf);
    snprintf(p->log, sizeof p->log, "%s", err);
    if (!p->linked) g2e_log("gles: link failed: %s", p->log);
}

void ES_APIENTRY glUseProgram(GLuint n)
{
    es_prog *p = NULL, *old = S.cur;
    if (n && !(p = prog_arg(n))) return;
    if (p && !p->linked) { SET_ERR(GL_INVALID_OPERATION); return; }
    S.cur = p;
    if (old && old != p && old->del) prog_free(old);
}
void ES_APIENTRY glValidateProgram(GLuint n) { es_prog *p = prog_arg(n); if (!p) return; p->validated = p->linked; }
void ES_APIENTRY glGetProgramiv(GLuint n, GLenum q, GLint *v)
{
    es_prog *p = prog_arg(n);
    int i, m = 0;
    if (!p) return;
    switch (q) {
    case GL_DELETE_STATUS: *v = p->del; break;
    case GL_LINK_STATUS: *v = p->linked; break;
    case GL_VALIDATE_STATUS: *v = p->validated; break;
    case GL_INFO_LOG_LENGTH: *v = p->log[0] ? (GLint)strlen(p->log) + 1 : 0; break;
    case GL_ATTACHED_SHADERS: *v = (p->vs != 0) + (p->fs != 0); break;
    case GL_PROGRAM_BINARY_LENGTH_OES: *v = p->linked && p->bin_vs ? (GLint)(16 + strlen(p->bin_vs) + strlen(p->bin_fs) + (size_t)p->nbind * sizeof p->bind[0]) : 0; break;
    case GL_ACTIVE_ATTRIBUTES: *v = p->nattr; break;
    case GL_ACTIVE_ATTRIBUTE_MAX_LENGTH: for (i = 0; i < p->nattr; i++) { int l = (int)strlen(p->attr[i].name) + 1; if (l > m) m = l; } *v = m; break;
    case GL_ACTIVE_UNIFORMS: *v = p->nuni; break;
    case GL_ACTIVE_UNIFORM_MAX_LENGTH: for (i = 0; i < p->nuni; i++) { int l = (int)strlen(p->uni[i].name) + (p->uni[i].isarr ? 3 : 0) + 1; if (l > m) m = l; } *v = m; break;     /* glGetActiveUniform's name and its NUL */
    default: SET_ERR(GL_INVALID_ENUM); break;
    }
}
void ES_APIENTRY glGetProgramInfoLog(GLuint n, GLsizei max, GLsizei *len, GLchar *log)
{
    es_prog *p = prog_arg(n);
    int l;
    if (!p) return;
    if (max < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!max) { if (len) *len = 0; return; }
    l = snprintf(log, (size_t)max, "%s", p->log);
    if (len) *len = l < max ? l : max - 1;
}
void ES_APIENTRY glGetActiveAttrib(GLuint n, GLuint i, GLsizei max, GLsizei *len, GLint *size, GLenum *type, GLchar *name)
{
    es_prog *p = prog_arg(n);
    int l;
    if (!p) return;
    if ((int)i >= p->nattr || max < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    l = max ? snprintf(name, (size_t)max, "%s", p->attr[i].name) : 0;
    if (len) *len = l < max ? l : (max ? max - 1 : 0);
    *size = 1; *type = p->attr[i].type;
}
GLint ES_APIENTRY glGetAttribLocation(GLuint n, const GLchar *name)
{
    es_prog *p = prog_arg(n);
    int i;
    if (!p) return -1;
    if (!p->linked) { SET_ERR(GL_INVALID_OPERATION); return -1; }
    for (i = 0; i < p->nattr; i++) if (!strcmp(p->attr[i].name, name)) return p->attr[i].loc;
    return -1;
}
/* §2.10.4: an array's name with "[0]", also of an array of one */
void ES_APIENTRY glGetActiveUniform(GLuint n, GLuint i, GLsizei max, GLsizei *len, GLint *size, GLenum *type, GLchar *name)
{
    es_prog *p = prog_arg(n);
    int l;
    if (!p) return;
    if ((int)i >= p->nuni || max < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    l = max ? snprintf(name, (size_t)max, "%s%s", p->uni[i].name, p->uni[i].isarr ? "[0]" : "") : 0;
    if (len) *len = l < max ? l : (max ? max - 1 : 0);
    *size = p->uni[i].size; *type = p->uni[i].type;
}

/* a location: uniform index << 8 | array element */
GLint ES_APIENTRY glGetUniformLocation(GLuint n, const GLchar *name)
{
    es_prog *p = prog_arg(n);
    char base[96];
    char *br;
    int el, i;
    if (!p) return -1;
    if (!p->linked) { SET_ERR(GL_INVALID_OPERATION); return -1; }
    if (!strncmp(name, "gl_", 3)) return -1;                    /* §2.10.4: names with the reserved prefix have no location */
    snprintf(base, sizeof base, "%s", name);
    for (i = 0; i < p->nuni; i++) if (!strcmp(p->uni[i].name, base)) return i << 8;
    br = strrchr(base, '[');
    if (br && br[strlen(br) - 1] == ']') {
        el = atoi(br + 1);
        *br = 0;
        for (i = 0; i < p->nuni; i++) if (!strcmp(p->uni[i].name, base) && p->uni[i].isarr && el >= 0 && el < p->uni[i].size) return i << 8 | el;
    }
    return -1;
}

static void uni_store(es_prog *p, const es_uni *u, int slot_off, int comp, uint32_t bits)
{
    if (u->vs >= 0 && u->vs + slot_off < 256) p->vu[4 * (u->vs + slot_off) + u->vc + comp] = bits;
    if (u->fs >= 0 && u->fs + slot_off < 64) p->fu[4 * (u->fs + slot_off) + u->fc + comp] = bits;
}

/* the value's base type: 0 float, 1 int, 2 bool */
static int uni_base(GLenum t)
{
    switch (t) {
    case GL_INT: case GL_INT_VEC2: case GL_INT_VEC3: case GL_INT_VEC4: return 1;
    case GL_BOOL: case GL_BOOL_VEC2: case GL_BOOL_VEC3: case GL_BOOL_VEC4: return 2;
    default: return 0;
    }
}

static int is_sampler_type(GLenum t) { return t == GL_SAMPLER_2D || t == GL_SAMPLER_CUBE || t == GL_SAMPLER_3D_OES; }

/* §2.10.4. kind: 'f', 'i', 'm' (a matrix of comps columns). Int uniforms are stored as floats: the compiler keeps them that way, FTOI on load. uval keeps what was given.
   A sampler is set by Uniform1i{v} alone; one inside a struct has no unit of the compiler's (u->sampler < 0), so its value is kept and read back, and no draw sees it */
static void uni_set(GLint loc, GLsizei n, int kind, int comps, const float *v, const int *iv)
{
    es_prog *p = S.cur;
    const es_uni *u;
    int e, c, idx, el, per, base, ismat;
    if (!p) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (loc == -1) return;
    idx = loc >> 8; el = loc & 255;
    if (idx < 0 || idx >= p->nuni || el >= p->uni[idx].size) { SET_ERR(GL_INVALID_OPERATION); return; }
    u = &p->uni[idx];
    if (n > 1 && !u->isarr) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (is_sampler_type(u->type)) {
        if (kind != 'i' || comps != 1) { SET_ERR(GL_INVALID_OPERATION); return; }
        for (e = 0; e < n; e++) if (iv[e] < 0 || iv[e] >= MAX_UNITS) { SET_ERR(GL_INVALID_VALUE); return; }
        for (e = 0; e < n && el + e < u->size; e++) {
            p->uval[u->voff + el + e] = (uint32_t)iv[e];
            if (u->sampler >= 0) p->samp_unit[u->sampler + el + e] = iv[e];
        }
        return;
    }
    base = uni_base(u->type);
    ismat = u->stride > 1;
    if (kind == 'm' ? !ismat || u->stride != comps : ismat || u->comps != comps || (kind == 'f' && base == 1) || (kind == 'i' && base == 0)) { SET_ERR(GL_INVALID_OPERATION); return; }
    per = kind == 'm' ? comps * comps : comps;                  /* = stride * comps: one element */
    for (e = 0; e < n && el + e < u->size; e++) {
        for (c = 0; c < per; c++) {
            uint32_t raw;
            double d;
            if (base == 2) { int b = kind == 'i' ? iv[e * per + c] != 0 : v[e * per + c] != 0.0f; raw = (uint32_t)b; d = b; }
            else if (base == 1) { raw = (uint32_t)iv[e * per + c]; d = iv[e * per + c]; }
            else { memcpy(&raw, &v[e * per + c], 4); d = v[e * per + c]; }
            p->uval[u->voff + (el + e) * per + c] = raw;
            uni_store(p, u, (el + e) * u->stride + c / u->comps, c % u->comps, g2_fp24(d));
        }
    }
}

#define UF(n, p, ...) void ES_APIENTRY n { float x[] = { __VA_ARGS__ }; uni_set(l, 1, 'f', p, x, NULL); }
#define UI(n, p, ...) void ES_APIENTRY n { int x[] = { __VA_ARGS__ }; uni_set(l, 1, 'i', p, NULL, x); }
UF(glUniform1f(GLint l, GLfloat a), 1, a)
UF(glUniform2f(GLint l, GLfloat a, GLfloat b), 2, a, b)
UF(glUniform3f(GLint l, GLfloat a, GLfloat b, GLfloat c), 3, a, b, c)
UF(glUniform4f(GLint l, GLfloat a, GLfloat b, GLfloat c, GLfloat d), 4, a, b, c, d)
UI(glUniform1i(GLint l, GLint a), 1, a)
UI(glUniform2i(GLint l, GLint a, GLint b), 2, a, b)
UI(glUniform3i(GLint l, GLint a, GLint b, GLint c), 3, a, b, c)
UI(glUniform4i(GLint l, GLint a, GLint b, GLint c, GLint d), 4, a, b, c, d)
void ES_APIENTRY glUniform1fv(GLint l, GLsizei n, const GLfloat *v) { uni_set(l, n, 'f', 1, v, NULL); }
void ES_APIENTRY glUniform2fv(GLint l, GLsizei n, const GLfloat *v) { uni_set(l, n, 'f', 2, v, NULL); }
void ES_APIENTRY glUniform3fv(GLint l, GLsizei n, const GLfloat *v) { uni_set(l, n, 'f', 3, v, NULL); }
void ES_APIENTRY glUniform4fv(GLint l, GLsizei n, const GLfloat *v) { uni_set(l, n, 'f', 4, v, NULL); }
void ES_APIENTRY glUniform1iv(GLint l, GLsizei n, const GLint *v) { uni_set(l, n, 'i', 1, NULL, v); }
void ES_APIENTRY glUniform2iv(GLint l, GLsizei n, const GLint *v) { uni_set(l, n, 'i', 2, NULL, v); }
void ES_APIENTRY glUniform3iv(GLint l, GLsizei n, const GLint *v) { uni_set(l, n, 'i', 3, NULL, v); }
void ES_APIENTRY glUniform4iv(GLint l, GLsizei n, const GLint *v) { uni_set(l, n, 'i', 4, NULL, v); }
void ES_APIENTRY glUniformMatrix2fv(GLint l, GLsizei n, GLboolean t, const GLfloat *v) { if (t) { SET_ERR(GL_INVALID_VALUE); return; } uni_set(l, n, 'm', 2, v, NULL); }
void ES_APIENTRY glUniformMatrix3fv(GLint l, GLsizei n, GLboolean t, const GLfloat *v) { if (t) { SET_ERR(GL_INVALID_VALUE); return; } uni_set(l, n, 'm', 3, v, NULL); }
void ES_APIENTRY glUniformMatrix4fv(GLint l, GLsizei n, GLboolean t, const GLfloat *v) { if (t) { SET_ERR(GL_INVALID_VALUE); return; } uni_set(l, n, 'm', 4, v, NULL); }

/* §6.1.2: a floating-point value asked for as an integer is rounded to the nearest one, and clamped to what GLint holds */
static GLint round_int(double f)
{
    if (!(f == f)) return 0;
    if (f >= 2147483647.0) return 2147483647;
    if (f <= -2147483648.0) return (GLint)(-2147483647 - 1);
    return (GLint)floor(f + 0.5);
}

/* §6.1.8: the value the uniform holds, as Uniform* gave it: fv as floats, else iv as integers. A location is valid for the program it came from, the whole uniform or an element of an array */
static void uni_get(GLuint n, GLint loc, GLfloat *fv, GLint *iv)
{
    es_prog *p = prog_arg(n);
    const es_uni *u;
    int e, c, cnt, base;
    if (!p) return;
    if (!p->linked || loc < 0 || (loc >> 8) >= p->nuni || (loc & 255) >= p->uni[loc >> 8].size) { SET_ERR(GL_INVALID_OPERATION); return; }
    u = &p->uni[loc >> 8]; e = loc & 255;
    cnt = u->stride * u->comps;
    base = is_sampler_type(u->type) ? 1 : uni_base(u->type);
    for (c = 0; c < cnt; c++) {
        uint32_t raw = p->uval[u->voff + e * cnt + c];
        if (base == 0) { float f; memcpy(&f, &raw, 4); if (fv) fv[c] = f; else iv[c] = round_int(f); }
        else if (fv) fv[c] = (GLfloat)(int32_t)raw;
        else iv[c] = (GLint)(int32_t)raw;
    }
}
void ES_APIENTRY glGetUniformfv(GLuint n, GLint loc, GLfloat *v) { uni_get(n, loc, v, NULL); }
void ES_APIENTRY glGetUniformiv(GLuint n, GLint loc, GLint *v) { uni_get(n, loc, NULL, v); }

/* ---------------------------------------------------------------- vertex attributes */
static es_va *va_of(GLuint i) { if (i >= MAX_ATTR) { SET_ERR(GL_INVALID_VALUE); return NULL; } return &S.va[i]; }
void ES_APIENTRY glEnableVertexAttribArray(GLuint i) { es_va *a = va_of(i); if (a) a->enabled = 1; }
void ES_APIENTRY glDisableVertexAttribArray(GLuint i) { es_va *a = va_of(i); if (a) a->enabled = 0; }
void ES_APIENTRY glVertexAttribPointer(GLuint i, GLint size, GLenum type, GLboolean norm, GLsizei stride, const void *p)
{
    es_va *a = va_of(i);
    if (!a) return;
    if (size < 1 || size > 4 || stride < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (type != GL_BYTE && type != GL_UNSIGNED_BYTE && type != GL_SHORT && type != GL_UNSIGNED_SHORT && type != GL_FIXED && type != GL_FLOAT) { SET_ERR(GL_INVALID_ENUM); return; }
    a->size = size; a->type = type; a->normalized = norm != 0; a->stride = stride; a->ptr = p; a->buffer = S.array_buf;
}
void ES_APIENTRY glVertexAttrib4f(GLuint i, GLfloat x, GLfloat y, GLfloat z, GLfloat w) { es_va *a = va_of(i); if (a) { a->cur[0] = x; a->cur[1] = y; a->cur[2] = z; a->cur[3] = w; } }
void ES_APIENTRY glVertexAttrib1f(GLuint i, GLfloat x) { glVertexAttrib4f(i, x, 0, 0, 1); }
void ES_APIENTRY glVertexAttrib2f(GLuint i, GLfloat x, GLfloat y) { glVertexAttrib4f(i, x, y, 0, 1); }
void ES_APIENTRY glVertexAttrib3f(GLuint i, GLfloat x, GLfloat y, GLfloat z) { glVertexAttrib4f(i, x, y, z, 1); }
void ES_APIENTRY glVertexAttrib1fv(GLuint i, const GLfloat *v) { glVertexAttrib4f(i, v[0], 0, 0, 1); }
void ES_APIENTRY glVertexAttrib2fv(GLuint i, const GLfloat *v) { glVertexAttrib4f(i, v[0], v[1], 0, 1); }
void ES_APIENTRY glVertexAttrib3fv(GLuint i, const GLfloat *v) { glVertexAttrib4f(i, v[0], v[1], v[2], 1); }
void ES_APIENTRY glVertexAttrib4fv(GLuint i, const GLfloat *v) { glVertexAttrib4f(i, v[0], v[1], v[2], v[3]); }
void ES_APIENTRY glGetVertexAttribfv(GLuint i, GLenum p, GLfloat *v)
{
    es_va *a = va_of(i);
    if (!a) return;
    switch (p) {
    case GL_CURRENT_VERTEX_ATTRIB: memcpy(v, a->cur, 16); break;
    case GL_VERTEX_ATTRIB_ARRAY_ENABLED: *v = (GLfloat)a->enabled; break;
    case GL_VERTEX_ATTRIB_ARRAY_SIZE: *v = (GLfloat)a->size; break;
    case GL_VERTEX_ATTRIB_ARRAY_STRIDE: *v = (GLfloat)a->stride; break;
    case GL_VERTEX_ATTRIB_ARRAY_TYPE: *v = (GLfloat)a->type; break;
    case GL_VERTEX_ATTRIB_ARRAY_NORMALIZED: *v = (GLfloat)a->normalized; break;
    case GL_VERTEX_ATTRIB_ARRAY_DIVISOR_ANGLE: *v = (GLfloat)a->divisor; break;
    case GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING: *v = (GLfloat)a->buffer; break;
    default: SET_ERR(GL_INVALID_ENUM); break;
    }
}
void ES_APIENTRY glGetVertexAttribiv(GLuint i, GLenum p, GLint *v)
{
    float f[4] = { 0, 0, 0, 0 };
    int k, n = p == GL_CURRENT_VERTEX_ATTRIB ? 4 : 1;
    glGetVertexAttribfv(i, p, f);
    for (k = 0; k < n; k++) v[k] = (GLint)f[k];
}
void ES_APIENTRY glGetVertexAttribPointerv(GLuint i, GLenum p, void **v)
{
    es_va *a = va_of(i);
    if (p != GL_VERTEX_ATTRIB_ARRAY_POINTER) { SET_ERR(GL_INVALID_ENUM); return; }
    if (a) *v = (void *)a->ptr;
}

/* ---------------------------------------------------------------- state */
static int *cap_flag(GLenum c)
{
    switch (c) {
    case GL_DEPTH_TEST: return &S.depth_test;
    case GL_STENCIL_TEST: return &S.stencil_test;
    case GL_BLEND: return &S.blend;
    case GL_CULL_FACE: return &S.cull;
    case GL_SCISSOR_TEST: return &S.scissor_test;
    case GL_DITHER: return &S.dither;
    case GL_POLYGON_OFFSET_FILL: return &S.po_fill;
    case GL_SAMPLE_COVERAGE: return &S.sample_cov;
    case GL_SAMPLE_ALPHA_TO_COVERAGE: return &S.sample_a2c;
    default: SET_ERR(GL_INVALID_ENUM); return NULL;
    }
}
void ES_APIENTRY glEnable(GLenum c) { int *f = cap_flag(c); if (f) *f = 1; }
void ES_APIENTRY glDisable(GLenum c) { int *f = cap_flag(c); if (f) *f = 0; }
GLboolean ES_APIENTRY glIsEnabled(GLenum c) { int *f = cap_flag(c); return f && *f; }
static int blend_src_ok(GLenum f)
{
    switch (f) {
    case GL_ZERO: case GL_ONE: case GL_SRC_COLOR: case GL_ONE_MINUS_SRC_COLOR: case GL_DST_COLOR: case GL_ONE_MINUS_DST_COLOR: case GL_SRC_ALPHA: case GL_ONE_MINUS_SRC_ALPHA: case GL_DST_ALPHA: case GL_ONE_MINUS_DST_ALPHA:
    case GL_CONSTANT_COLOR: case GL_ONE_MINUS_CONSTANT_COLOR: case GL_CONSTANT_ALPHA: case GL_ONE_MINUS_CONSTANT_ALPHA: case GL_SRC_ALPHA_SATURATE: return 1;
    default: return 0;
    }
}
void ES_APIENTRY glBlendFuncSeparate(GLenum s, GLenum d, GLenum sa, GLenum da)
{
    if (!blend_src_ok(s) || !blend_src_ok(d) || !blend_src_ok(sa) || !blend_src_ok(da) || d == GL_SRC_ALPHA_SATURATE || da == GL_SRC_ALPHA_SATURATE) { SET_ERR(GL_INVALID_ENUM); return; }
    S.b_src = s; S.b_dst = d; S.b_src_a = sa; S.b_dst_a = da;
}
void ES_APIENTRY glBlendFunc(GLenum s, GLenum d) { glBlendFuncSeparate(s, d, s, d); }
static int blend_eq_ok(GLenum e) { return e == GL_FUNC_ADD || e == GL_FUNC_SUBTRACT || e == GL_FUNC_REVERSE_SUBTRACT || e == GL_MIN || e == GL_MAX; }
void ES_APIENTRY glBlendEquationSeparate(GLenum e, GLenum a) { if (!blend_eq_ok(e) || !blend_eq_ok(a)) { SET_ERR(GL_INVALID_ENUM); return; } S.b_eq = e; S.b_eq_a = a; }
void ES_APIENTRY glBlendEquation(GLenum e) { glBlendEquationSeparate(e, e); }
static float clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }
void ES_APIENTRY glBlendColor(GLclampf r, GLclampf g, GLclampf b, GLclampf a) { S.b_color[0] = clamp01(r); S.b_color[1] = clamp01(g); S.b_color[2] = clamp01(b); S.b_color[3] = clamp01(a); }
static int func_ok(GLenum f) { return f >= GL_NEVER && f <= GL_ALWAYS; }
void ES_APIENTRY glDepthFunc(GLenum f) { if (!func_ok(f)) { SET_ERR(GL_INVALID_ENUM); return; } S.depth_func = f; }
void ES_APIENTRY glDepthMask(GLboolean m) { S.depth_mask = m != 0; }
void ES_APIENTRY glDepthRangef(GLclampf n, GLclampf f) { S.depth_n = clamp01(n); S.depth_f = clamp01(f); }
void ES_APIENTRY glColorMask(GLboolean r, GLboolean g, GLboolean b, GLboolean a) { S.color_mask = (r ? 8 : 0) | (g ? 4 : 0) | (b ? 2 : 0) | (a ? 1 : 0); }
void ES_APIENTRY glCullFace(GLenum m) { if (m != GL_FRONT && m != GL_BACK && m != GL_FRONT_AND_BACK) { SET_ERR(GL_INVALID_ENUM); return; } S.cull_mode = m; }
void ES_APIENTRY glFrontFace(GLenum m) { if (m != GL_CW && m != GL_CCW) { SET_ERR(GL_INVALID_ENUM); return; } S.front_face = m; }
void ES_APIENTRY glPolygonOffset(GLfloat f, GLfloat u) { S.po_factor = f; S.po_units = u; }
void ES_APIENTRY glLineWidth(GLfloat w) { if (w <= 0) { SET_ERR(GL_INVALID_VALUE); return; } S.line_width = w; }
void ES_APIENTRY glScissor(GLint x, GLint y, GLsizei w, GLsizei h) { if (w < 0 || h < 0) { SET_ERR(GL_INVALID_VALUE); return; } S.sc[0] = x; S.sc[1] = y; S.sc[2] = w; S.sc[3] = h; }
void ES_APIENTRY glViewport(GLint x, GLint y, GLsizei w, GLsizei h) { if (w < 0 || h < 0) { SET_ERR(GL_INVALID_VALUE); return; } S.vp[0] = x; S.vp[1] = y; S.vp[2] = w > MAX_VIEWPORT ? MAX_VIEWPORT : w; S.vp[3] = h > MAX_VIEWPORT ? MAX_VIEWPORT : h; }
void ES_APIENTRY glSampleCoverage(GLclampf v, GLboolean i) { S.sample_val = clamp01(v); S.sample_inv = i != 0; }     /* SAMPLE_BUFFERS is 0: no effect */
void ES_APIENTRY glHint(GLenum t, GLenum m)
{
    if (m != GL_FASTEST && m != GL_NICEST && m != GL_DONT_CARE) { SET_ERR(GL_INVALID_ENUM); return; }
    if (t == GL_GENERATE_MIPMAP_HINT) S.hint_mip = m;
    else if (t == GL_FRAGMENT_SHADER_DERIVATIVE_HINT_OES) S.hint_deriv = m;
    else SET_ERR(GL_INVALID_ENUM);
}
void ES_APIENTRY glPixelStorei(GLenum p, GLint v)
{
    if (p != GL_UNPACK_ALIGNMENT && p != GL_PACK_ALIGNMENT) { SET_ERR(GL_INVALID_ENUM); return; }
    if (v != 1 && v != 2 && v != 4 && v != 8) { SET_ERR(GL_INVALID_VALUE); return; }
    if (p == GL_UNPACK_ALIGNMENT) S.un_align = v; else S.pk_align = v;
}
static int face_ok(GLenum f) { if (f == GL_FRONT || f == GL_BACK || f == GL_FRONT_AND_BACK) return 1; SET_ERR(GL_INVALID_ENUM); return 0; }
static int op_ok(GLenum o) { return o == GL_KEEP || o == GL_ZERO || o == GL_REPLACE || o == GL_INCR || o == GL_DECR || o == GL_INVERT || o == GL_INCR_WRAP || o == GL_DECR_WRAP; }
static void st_func(int f, GLenum fn, GLint r, GLuint m) { S.st_func[f] = fn; S.st_ref[f] = r; S.st_vmask[f] = m; }
static void st_op(int f, GLenum a, GLenum b, GLenum c) { S.st_fail[f] = a; S.st_zfail[f] = b; S.st_zpass[f] = c; }
void ES_APIENTRY glStencilFuncSeparate(GLenum face, GLenum fn, GLint r, GLuint m)
{
    if (!face_ok(face)) return;
    if (!func_ok(fn)) { SET_ERR(GL_INVALID_ENUM); return; }
    if (face != GL_BACK) st_func(0, fn, r, m);
    if (face != GL_FRONT) st_func(1, fn, r, m);
}
void ES_APIENTRY glStencilFunc(GLenum fn, GLint r, GLuint m) { glStencilFuncSeparate(GL_FRONT_AND_BACK, fn, r, m); }
void ES_APIENTRY glStencilOpSeparate(GLenum face, GLenum a, GLenum b, GLenum c)
{
    if (!face_ok(face)) return;
    if (!op_ok(a) || !op_ok(b) || !op_ok(c)) { SET_ERR(GL_INVALID_ENUM); return; }
    if (face != GL_BACK) st_op(0, a, b, c);
    if (face != GL_FRONT) st_op(1, a, b, c);
}
void ES_APIENTRY glStencilOp(GLenum a, GLenum b, GLenum c) { glStencilOpSeparate(GL_FRONT_AND_BACK, a, b, c); }
void ES_APIENTRY glStencilMaskSeparate(GLenum face, GLuint m) { if (!face_ok(face)) return; if (face != GL_BACK) S.st_wmask[0] = m; if (face != GL_FRONT) S.st_wmask[1] = m; }
void ES_APIENTRY glStencilMask(GLuint m) { S.st_wmask[0] = S.st_wmask[1] = m; }
void ES_APIENTRY glClearColor(GLclampf r, GLclampf g, GLclampf b, GLclampf a) { S.clear_color[0] = clamp01(r); S.clear_color[1] = clamp01(g); S.clear_color[2] = clamp01(b); S.clear_color[3] = clamp01(a); }
void ES_APIENTRY glClearDepthf(GLclampf d) { S.clear_depth = clamp01(d); }
void ES_APIENTRY glClearStencil(GLint s) { S.clear_stencil = s; }
/* nothing to do: an EGL surface shows at eglSwapBuffers, and a read, a copy or eglWaitGL ends the pass itself. Ending it here would cost the boards a flush of the whole target and its reload */
void ES_APIENTRY glFlush(void) { }
void ES_APIENTRY glFinish(void) { if (S.live) g2r_finish(S.r); }
GLenum ES_APIENTRY glGetError(void) { GLenum e = S.error; S.error = GL_NO_ERROR; return e; }

void ES_APIENTRY glClear(GLbitfield m)
{
    es_tg g;
    uint32_t fl = 0, sc[4];
    double c[4];
    int i, cmask = S.color_mask;
    if (m & ~(GLbitfield)(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)) { SET_ERR(GL_INVALID_VALUE); return; }
    if (!S.live || bind_target(&g) < 0) return;
    if (g.no_alpha) cmask &= 14;
    if ((m & GL_COLOR_BUFFER_BIT) && cmask) fl |= G2_FL_CLEAR_COLOR | G2_FL_CMASK(cmask);
    if ((m & GL_DEPTH_BUFFER_BIT) && S.depth_mask && g.has_d) fl |= G2_FL_CLEAR_DEPTH;
    if ((m & GL_STENCIL_BUFFER_BIT) && g.has_s && (S.st_wmask[0] & 255)) fl |= G2_FL_CLEAR_STENCIL;
    if (!fl) return;
    if (S.dither) fl |= G2_FL_DITHER;
    sc[0] = 0; sc[1] = 0; sc[2] = g.t.width; sc[3] = g.t.height;
    if (S.scissor_test) {
        int b[4];
        scissor_box(&g, b);
        if (b[2] <= b[0] || b[3] <= b[1]) return;
        sc[0] = (uint32_t)b[0]; sc[1] = (uint32_t)b[1]; sc[2] = (uint32_t)b[2]; sc[3] = (uint32_t)b[3];
        fl |= G2_FL_CLEAR_SCISSOR;
    }
    for (i = 0; i < 4; i++) c[i] = S.clear_color[i];
    g2r_clear(S.r, fl, c, g.t.depth16 ? (uint32_t)(S.clear_depth * 65535.0f + 0.5f) : (uint32_t)((double)S.clear_depth * 16777215.0 + 0.5), (uint32_t)S.clear_stencil & 255, S.st_wmask[0] & 255, sc);
}

#define ES_EXTENSIONS "GL_OES_rgb8_rgba8 GL_OES_depth24 GL_OES_packed_depth_stencil GL_OES_element_index_uint GL_OES_texture_npot GL_OES_standard_derivatives GL_OES_fragment_precision_high GL_EXT_shader_texture_lod GL_EXT_frag_depth GL_OES_vertex_array_object GL_OES_mapbuffer GL_EXT_discard_framebuffer GL_EXT_blend_minmax GL_EXT_texture_format_BGRA8888 GL_EXT_read_format_bgra GL_EXT_texture_compression_dxt1 GL_ANGLE_texture_compression_dxt3 GL_ANGLE_texture_compression_dxt5 GL_ANGLE_instanced_arrays GL_ANGLE_framebuffer_blit GL_ANGLE_translated_shader_source GL_NV_fence GL_EXT_robustness GL_EXT_texture_storage GL_OES_get_program_binary GL_OES_compressed_ETC1_RGB8_texture GL_OES_depth_texture GL_OES_fbo_render_mipmap"

const GLubyte *ES_APIENTRY glGetString(GLenum n)
{
    switch (n) {
    case GL_VENDOR: return (const GLubyte *)"gpu2";
    case GL_RENDERER: return (const GLubyte *)"GPU2 two-board FPGA GPU";
    case GL_VERSION: return (const GLubyte *)"OpenGL ES 2.0 gpu2";
    case GL_SHADING_LANGUAGE_VERSION: return (const GLubyte *)"OpenGL ES GLSL ES 1.00 gpu2";
    case GL_EXTENSIONS: return (const GLubyte *)(has_3d() ? ES_EXTENSIONS " GL_OES_texture_3D" : ES_EXTENSIONS);
    default: SET_ERR(GL_INVALID_ENUM); return NULL;
    }
}

/* the number of values written; -1: a legal query with none */
static int get_int(GLenum p, GLint *v)
{
    es_tg g;
    switch (p) {
    case GL_MAX_VERTEX_ATTRIBS: *v = MAX_ATTR; return 1;
    case GL_MAX_VERTEX_UNIFORM_VECTORS: *v = 224; return 1;     /* 256 registers, room left for a program's constants */
    case GL_MAX_FRAGMENT_UNIFORM_VECTORS: *v = 48; return 1;    /* 64 likewise */
    case GL_MAX_VARYING_VECTORS: *v = 8; return 1;
    case GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS: case GL_MAX_TEXTURE_IMAGE_UNITS: *v = MAX_UNITS; return 1;
    case GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS: *v = 0; return 1;
    case GL_MAX_TEXTURE_SIZE: case GL_MAX_CUBE_MAP_TEXTURE_SIZE: *v = MAX_TEX; return 1;
    case GL_MAX_3D_TEXTURE_SIZE_OES: if (!has_3d()) return 0; *v = MAX_3D; return 1;
    case GL_MAX_RENDERBUFFER_SIZE: *v = MAX_RT_W; return 1;
    case GL_MAX_VIEWPORT_DIMS: v[0] = MAX_VIEWPORT; v[1] = MAX_VIEWPORT; return 2;
    case GL_NUM_SHADER_BINARY_FORMATS: *v = 0; return 1;
    case GL_NUM_PROGRAM_BINARY_FORMATS_OES: *v = 1; return 1;
    case GL_PROGRAM_BINARY_FORMATS_OES: *v = (GLint)G2_PROGRAM_BINARY; return 1;
    case GL_READ_FRAMEBUFFER_BINDING_ANGLE: *v = (GLint)S.read_fb; return 1;
    case GL_SHADER_BINARY_FORMATS: return -1;
    case GL_SHADER_COMPILER: *v = 1; return 1;
    case GL_NUM_COMPRESSED_TEXTURE_FORMATS: *v = 5; return 1;
    case GL_COMPRESSED_TEXTURE_FORMATS: v[0] = GL_COMPRESSED_RGB_S3TC_DXT1_EXT; v[1] = GL_COMPRESSED_RGBA_S3TC_DXT1_EXT; v[2] = GL_COMPRESSED_RGBA_S3TC_DXT3_EXT; v[3] = GL_COMPRESSED_RGBA_S3TC_DXT5_EXT; v[4] = GL_ETC1_RGB8_OES; return 5;
    case GL_VIEWPORT: memcpy(v, S.vp, 16); return 4;
    case GL_SCISSOR_BOX: memcpy(v, S.sc, 16); return 4;
    case GL_ARRAY_BUFFER_BINDING: *v = (GLint)S.array_buf; return 1;
    case GL_ELEMENT_ARRAY_BUFFER_BINDING: *v = (GLint)S.elem_buf; return 1;
    case GL_FRAMEBUFFER_BINDING: *v = (GLint)S.fb; return 1;
    case GL_RENDERBUFFER_BINDING: *v = (GLint)S.rb; return 1;
    case GL_CURRENT_PROGRAM: *v = S.cur ? (GLint)S.cur->name : 0; return 1;
    case GL_ACTIVE_TEXTURE: *v = GL_TEXTURE0 + S.active; return 1;
    case GL_TEXTURE_BINDING_2D: *v = (GLint)S.tex2d[S.active]; return 1;
    case GL_TEXTURE_BINDING_CUBE_MAP: *v = (GLint)S.cube[S.active]; return 1;
    case GL_TEXTURE_BINDING_3D_OES: if (!has_3d()) return 0; *v = (GLint)S.tex3d[S.active]; return 1;
    case GL_UNPACK_ALIGNMENT: *v = S.un_align; return 1;
    case GL_PACK_ALIGNMENT: *v = S.pk_align; return 1;
    case GL_SUBPIXEL_BITS: *v = 4; return 1;
    case GL_SAMPLE_BUFFERS: case GL_SAMPLES: *v = 0; return 1;
    case GL_GENERATE_MIPMAP_HINT: *v = (GLint)S.hint_mip; return 1;
    case GL_FRAGMENT_SHADER_DERIVATIVE_HINT_OES: *v = (GLint)S.hint_deriv; return 1;
    case GL_VERTEX_ARRAY_BINDING_OES: *v = (GLint)S.vao; return 1;
    case GL_DEPTH_FUNC: *v = (GLint)S.depth_func; return 1;
    case GL_DEPTH_WRITEMASK: *v = S.depth_mask; return 1;
    case GL_COLOR_WRITEMASK: v[0] = (S.color_mask >> 3) & 1; v[1] = (S.color_mask >> 2) & 1; v[2] = (S.color_mask >> 1) & 1; v[3] = S.color_mask & 1; return 4;
    case GL_CULL_FACE_MODE: *v = (GLint)S.cull_mode; return 1;
    case GL_FRONT_FACE: *v = (GLint)S.front_face; return 1;
    case GL_BLEND_SRC_RGB: *v = (GLint)S.b_src; return 1;
    case GL_BLEND_DST_RGB: *v = (GLint)S.b_dst; return 1;
    case GL_BLEND_SRC_ALPHA: *v = (GLint)S.b_src_a; return 1;
    case GL_BLEND_DST_ALPHA: *v = (GLint)S.b_dst_a; return 1;
    case GL_BLEND_EQUATION_RGB: *v = (GLint)S.b_eq; return 1;
    case GL_BLEND_EQUATION_ALPHA: *v = (GLint)S.b_eq_a; return 1;
    case GL_STENCIL_FUNC: *v = (GLint)S.st_func[0]; return 1;
    case GL_STENCIL_BACK_FUNC: *v = (GLint)S.st_func[1]; return 1;
    case GL_STENCIL_FAIL: *v = (GLint)S.st_fail[0]; return 1;
    case GL_STENCIL_BACK_FAIL: *v = (GLint)S.st_fail[1]; return 1;
    case GL_STENCIL_PASS_DEPTH_FAIL: *v = (GLint)S.st_zfail[0]; return 1;
    case GL_STENCIL_BACK_PASS_DEPTH_FAIL: *v = (GLint)S.st_zfail[1]; return 1;
    case GL_STENCIL_PASS_DEPTH_PASS: *v = (GLint)S.st_zpass[0]; return 1;
    case GL_STENCIL_BACK_PASS_DEPTH_PASS: *v = (GLint)S.st_zpass[1]; return 1;
    case GL_STENCIL_REF: *v = S.st_ref[0]; return 1;
    case GL_STENCIL_BACK_REF: *v = S.st_ref[1]; return 1;
    case GL_STENCIL_VALUE_MASK: *v = (GLint)S.st_vmask[0]; return 1;
    case GL_STENCIL_BACK_VALUE_MASK: *v = (GLint)S.st_vmask[1]; return 1;
    case GL_STENCIL_WRITEMASK: *v = (GLint)S.st_wmask[0]; return 1;
    case GL_STENCIL_BACK_WRITEMASK: *v = (GLint)S.st_wmask[1]; return 1;
    case GL_STENCIL_CLEAR_VALUE: *v = S.clear_stencil; return 1;
    case GL_SAMPLE_COVERAGE_INVERT: *v = S.sample_inv; return 1;
    case GL_IMPLEMENTATION_COLOR_READ_TYPE: *v = GL_UNSIGNED_BYTE; return 1;
    case GL_IMPLEMENTATION_COLOR_READ_FORMAT: *v = GL_RGBA; return 1;
    case GL_RED_BITS: case GL_GREEN_BITS: case GL_BLUE_BITS: case GL_ALPHA_BITS: case GL_DEPTH_BITS: case GL_STENCIL_BITS:
        if (fb_target(&g) != GL_FRAMEBUFFER_COMPLETE) { *v = 0; return 1; }
        *v = p == GL_RED_BITS ? g.t.bits_r : p == GL_GREEN_BITS ? g.t.bits_g : p == GL_BLUE_BITS ? g.t.bits_b : p == GL_ALPHA_BITS ? (g.no_alpha ? 0 : g.t.bits_a) :
             p == GL_DEPTH_BITS ? (g.has_d ? (g.t.depth16 ? 16 : 24) : 0) : (g.has_s ? 8 : 0);
        return 1;
    default: return 0;
    }
}

static int get_float(GLenum p, GLfloat *v)
{
    switch (p) {
    case GL_COLOR_CLEAR_VALUE: memcpy(v, S.clear_color, 16); return 4;
    case GL_BLEND_COLOR: memcpy(v, S.b_color, 16); return 4;
    case GL_DEPTH_CLEAR_VALUE: *v = S.clear_depth; return 1;
    case GL_DEPTH_RANGE: v[0] = S.depth_n; v[1] = S.depth_f; return 2;
    case GL_LINE_WIDTH: *v = S.line_width; return 1;
    case GL_POLYGON_OFFSET_FACTOR: *v = S.po_factor; return 1;
    case GL_POLYGON_OFFSET_UNITS: *v = S.po_units; return 1;
    case GL_ALIASED_LINE_WIDTH_RANGE: case GL_ALIASED_POINT_SIZE_RANGE: v[0] = 1; v[1] = 64; return 2;
    case GL_SAMPLE_COVERAGE_VALUE: *v = S.sample_val; return 1;
    default: return 0;
    }
}
void ES_APIENTRY glGetIntegerv(GLenum p, GLint *v)
{
    float f[4];
    int n, i;
    if (get_int(p, v)) return;
    if ((n = get_float(p, f))) {
        /* §6.1.2: colours, depth range and the depth clear value map onto the integer range, any other value rounds to the nearest integer */
        int norm = p == GL_COLOR_CLEAR_VALUE || p == GL_BLEND_COLOR || p == GL_DEPTH_CLEAR_VALUE || p == GL_DEPTH_RANGE;
        for (i = 0; i < n; i++) v[i] = norm ? (GLint)((double)f[i] * 2147483647.0) : round_int(f[i]);
        return;
    }
    *v = glIsEnabled(p);
}
void ES_APIENTRY glGetFloatv(GLenum p, GLfloat *v)
{
    GLint iv[8];
    int n, i;
    if (get_float(p, v)) return;
    if ((n = get_int(p, iv))) {
        /* the stencil masks are unsigned: all ones is 2^32 - 1, not -1 */
        int uns = p == GL_STENCIL_VALUE_MASK || p == GL_STENCIL_BACK_VALUE_MASK || p == GL_STENCIL_WRITEMASK || p == GL_STENCIL_BACK_WRITEMASK;
        for (i = 0; i < n; i++) v[i] = uns ? (GLfloat)(GLuint)iv[i] : (GLfloat)iv[i];
        return;
    }
    *v = glIsEnabled(p);
}
void ES_APIENTRY glGetBooleanv(GLenum p, GLboolean *v)
{
    GLint iv[8];
    float f[4];
    int n, i;
    if ((n = get_int(p, iv))) { for (i = 0; i < n; i++) v[i] = iv[i] != 0; return; }
    if ((n = get_float(p, f))) { for (i = 0; i < n; i++) v[i] = f[i] != 0; return; }
    *v = glIsEnabled(p);
}

void ES_APIENTRY glReadPixels(GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, void *pixels)
{
    es_tg g;
    uint8_t *tmp;
    int row, r, c, x0, y0, x1, y1;
    if (w < 0 || h < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    /* §4.3.1: format and type from table 3.4 (EXT_read_format_bgra adds BGRA_EXT and its two packed types), of which RGBA / UNSIGNED_BYTE and the
       IMPLEMENTATION_COLOR_READ pair go, and BGRA_EXT / UNSIGNED_BYTE; any other pair of accepted values is INVALID_OPERATION */
    if ((format != GL_RGBA && format != GL_BGRA_EXT && format != GL_RGB && format != GL_ALPHA && format != GL_LUMINANCE && format != GL_LUMINANCE_ALPHA) ||
        (type != GL_UNSIGNED_BYTE && type != GL_UNSIGNED_SHORT_5_6_5 && type != GL_UNSIGNED_SHORT_4_4_4_4 && type != GL_UNSIGNED_SHORT_5_5_5_1 &&
         type != GL_UNSIGNED_SHORT_4_4_4_4_REV_EXT && type != GL_UNSIGNED_SHORT_1_5_5_5_REV_EXT)) { SET_ERR(GL_INVALID_ENUM); return; }
    if ((format != GL_RGBA && format != GL_BGRA_EXT) || type != GL_UNSIGNED_BYTE) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (!S.live) return;
    {
        GLuint draw_fb = S.fb;
        int bound;
        if (!read_fb_target(&g)) return;                        /* also when it reads no pixels */
        if (!w || !h) return;
        S.fb = S.read_fb;
        bound = bind_target(&g);
        S.fb = draw_fb;
        if (bound < 0) return;
    }
    x0 = x < 0 ? 0 : x; y0 = y < 0 ? 0 : y;
    x1 = x + w > g.vw ? g.vw : x + w; y1 = y + h > g.vh ? g.vh : y + h;
    if (x1 <= x0 || y1 <= y0) return;
    g2r_finish(S.r);
    tmp = (uint8_t *)malloc((size_t)(x1 - x0) * (y1 - y0) * 4);
    if (!tmp) { SET_ERR(GL_OUT_OF_MEMORY); return; }
    read_rgba8(&g, x0, y0, x1 - x0, y1 - y0, tmp);
    row = (w * 4 + S.pk_align - 1) / S.pk_align * S.pk_align;
    for (r = y0; r < y1; r++) {
        uint8_t *d = (uint8_t *)pixels + (ptrdiff_t)(r - y) * row + (ptrdiff_t)(x0 - x) * 4;
        const uint8_t *s = tmp + (size_t)(r - y0) * (x1 - x0) * 4;
        if (format == GL_RGBA) { memcpy(d, s, (size_t)(x1 - x0) * 4); continue; }
        for (c = 0; c < x1 - x0; c++) { d[4 * c] = s[4 * c + 2]; d[4 * c + 1] = s[4 * c + 1]; d[4 * c + 2] = s[4 * c]; d[4 * c + 3] = s[4 * c + 3]; }
    }
    free(tmp);
}

/* ---------------------------------------------------------------- draws */
static uint32_t blend_code(GLenum f)
{
    switch (f) {
    case GL_ZERO: return G2R_BF_ZERO;
    case GL_ONE: return G2R_BF_ONE;
    case GL_SRC_COLOR: return G2R_BF_SRC_COLOR;
    case GL_ONE_MINUS_SRC_COLOR: return G2R_BF_ONE_MINUS_SRC_COLOR;
    case GL_DST_COLOR: return G2R_BF_DST_COLOR;
    case GL_ONE_MINUS_DST_COLOR: return G2R_BF_ONE_MINUS_DST_COLOR;
    case GL_SRC_ALPHA: return G2R_BF_SRC_ALPHA;
    case GL_ONE_MINUS_SRC_ALPHA: return G2R_BF_ONE_MINUS_SRC_ALPHA;
    case GL_DST_ALPHA: return G2R_BF_DST_ALPHA;
    case GL_ONE_MINUS_DST_ALPHA: return G2R_BF_ONE_MINUS_DST_ALPHA;
    case GL_CONSTANT_COLOR: return G2R_BF_CONSTANT_COLOR;
    case GL_ONE_MINUS_CONSTANT_COLOR: return G2R_BF_ONE_MINUS_CONSTANT_COLOR;
    case GL_CONSTANT_ALPHA: return G2R_BF_CONSTANT_ALPHA;
    case GL_ONE_MINUS_CONSTANT_ALPHA: return G2R_BF_ONE_MINUS_CONSTANT_ALPHA;
    default: return G2R_BF_SRC_ALPHA_SATURATE;
    }
}
static uint32_t blend_eq(GLenum e) { return e == GL_FUNC_SUBTRACT ? G2R_BOP_SUB : e == GL_FUNC_REVERSE_SUBTRACT ? G2R_BOP_REV_SUB : e == GL_MIN ? G2R_BOP_MIN : e == GL_MAX ? G2R_BOP_MAX : G2R_BOP_ADD; }
static uint32_t sten_op(GLenum o)
{
    static const GLenum c[8] = { GL_KEEP, GL_ZERO, GL_REPLACE, GL_INCR, GL_DECR, GL_INVERT, GL_INCR_WRAP, GL_DECR_WRAP };
    uint32_t i;
    for (i = 0; i < 8; i++) if (c[i] == o) return i;
    return 0;
}
#define WRAP(w) ((w) == GL_REPEAT ? G2R_WRAP_REPEAT : (w) == GL_MIRRORED_REPEAT ? G2R_WRAP_MIRROR : G2R_WRAP_CLAMP)

static void fill_state(g2r_state *s, es_prog *p, const es_tg *g)
{
    const g2r_target *t = &g->t;
    uint32_t *h = s->h, i, cmask = S.draw_none ? 0 : (uint32_t)(g->no_alpha ? S.color_mask & 14 : S.color_mask);
    int depth = S.depth_test && g->has_d, sten = S.stencil_test && g->has_s;
    h[0] = (depth ? 1u : 0) | (depth && S.depth_mask ? 2u : 0) | (uint32_t)(S.depth_func & 7) << 2 | (sten ? 32u : 0) | (S.blend ? 64u : 0) | (S.dither ? 128u : 0) | (S.scissor_test ? 256u : 0) | (S.po_fill ? 512u : 0) | cmask << 16;
    for (i = 0; i < 2; i++) {
        int ref = S.st_ref[i] < 0 ? 0 : S.st_ref[i] > 255 ? 255 : S.st_ref[i];                  /* §4.1.5: clamped to the stencil bits */
        h[1 + i] = (uint32_t)ref | (S.st_vmask[i] & 255) << 8 | (S.st_wmask[i] & 255) << 16 | (uint32_t)(S.st_func[i] & 7) << 24;
    }
    h[3] = sten_op(S.st_fail[0]) | sten_op(S.st_zfail[0]) << 3 | sten_op(S.st_zpass[0]) << 6 | sten_op(S.st_fail[1]) << 16 | sten_op(S.st_zfail[1]) << 19 | sten_op(S.st_zpass[1]) << 22;
    h[4] = blend_code(S.b_src) | blend_code(S.b_dst) << 4 | blend_code(S.b_src_a) << 8 | blend_code(S.b_dst_a) << 12 | blend_eq(S.b_eq) << 16 | blend_eq(S.b_eq_a) << 20;
    h[5] = g2_c16(S.b_color[0], t->bits_r) | g2_c16(S.b_color[1], t->bits_g) << 13;
    h[6] = g2_c16(S.b_color[2], t->bits_b) | g2_c16(S.b_color[3], t->bits_a ? t->bits_a : 8) << 13;
    h[14] = g2_c16(S.b_color[3], t->bits_r) | g2_c16(S.b_color[3], t->bits_g) << 13;
    {
        int b[4];
        scissor_box(g, b);
        h[7] = (uint32_t)b[0] | (uint32_t)b[1] << 16; h[8] = (uint32_t)b[2] | (uint32_t)b[3] << 16;
    }
    h[9] = g2_fp24(S.po_factor);
    h[10] = (uint32_t)(int32_t)S.po_units;
    p->dnow = 0;
    for (i = 0; i < (uint32_t)p->nsamp && i < MAX_UNITS; i++) {
        int u = p->samp_unit[i], j, kind = 0;
        es_tex *x;
        uint32_t mip, mn;
        for (j = 0; j < p->nuni; j++) if (p->uni[j].sampler >= 0 && (int)i >= p->uni[j].sampler && (int)i < p->uni[j].sampler + p->uni[j].size) kind = p->uni[j].type == GL_SAMPLER_CUBE ? 1 : p->uni[j].type == GL_SAMPLER_3D_OES ? 2 : 0;
        x = unit_tex(u, kind);
        tex_resolve(x);
        mn = x->min_f == GL_LINEAR || x->min_f == GL_LINEAR_MIPMAP_NEAREST || x->min_f == GL_LINEAR_MIPMAP_LINEAR;
        mip = x->min_f == GL_NEAREST_MIPMAP_NEAREST || x->min_f == GL_LINEAR_MIPMAP_NEAREST ? 1 : x->min_f == GL_NEAREST_MIPMAP_LINEAR || x->min_f == GL_LINEAR_MIPMAP_LINEAR ? 2 : 0;
        h[15 + i] = G2R_SAMPLER3D(x->handle, G2R_FILTER(x->mag_f == GL_LINEAR, mip, mn), WRAP(x->wrap_s), WRAP(x->wrap_t), kind == 2 ? WRAP(x->wrap_r) : 0, !tex_complete(x));
        if ((x->format == GL_DEPTH_COMPONENT || x->format == GL_DEPTH_STENCIL_OES) && tex_complete(x)) p->dnow |= 1u << i;
        x->used_pass = g2r_pass_id(S.r);
        if (p->ts[i] >= 0) { p->fu[4 * p->ts[i]] = g2_fp24((double)x->w); p->fu[4 * p->ts[i] + 1] = g2_fp24((double)x->h); }
    }
    if (p->dr_vs >= 0) { p->vu[4 * p->dr_vs] = g2_fp24(S.depth_n); p->vu[4 * (p->dr_vs + 1)] = g2_fp24(S.depth_f); p->vu[4 * (p->dr_vs + 2)] = g2_fp24((double)S.depth_f - S.depth_n); }
    if (p->dr_fs >= 0) { p->fu[4 * p->dr_fs] = g2_fp24(S.depth_n); p->fu[4 * (p->dr_fs + 1)] = g2_fp24(S.depth_f); p->fu[4 * (p->dr_fs + 2)] = g2_fp24((double)S.depth_f - S.depth_n); }
    s->vp_x = S.vp[0] * g->kx; s->vp_y = S.vp[1] * g->ky; s->vp_w = S.vp[2] * g->kx; s->vp_h = S.vp[3] * g->ky;
    s->depth_near = S.depth_n; s->depth_far = S.depth_f;
    s->cull = S.cull ? (S.cull_mode == GL_BACK ? 1 : S.cull_mode == GL_FRONT ? 2 : 3) : 0;
    s->front_cw = S.front_face == GL_CW;
    s->line_width = S.line_width;
}

static int at_type(GLenum t, int *bytes)
{
    switch (t) {
    case GL_BYTE: *bytes = 1; return G2_AT_BYTE;
    case GL_UNSIGNED_BYTE: *bytes = 1; return G2_AT_UBYTE;
    case GL_SHORT: *bytes = 2; return G2_AT_SHORT;
    case GL_UNSIGNED_SHORT: *bytes = 2; return G2_AT_USHORT;
    case GL_FIXED: *bytes = 4; return G2_AT_FIXED;
    default: *bytes = 4; return G2_AT_FLOAT;
    }
}

/* a sampler that reads a depth texture is a different fragment program: what the TMU gives is the word of a D24S8 target. Tables stay those of the link */
static g2r_prog *depth_variant(es_prog *p)
{
    g2sc_linked l;
    g2sc_bind gb[sizeof p->lbind / sizeof p->lbind[0]];
    g2r_prog *hw;
    int i;
    if (!p->dnow) return p->p;
    for (i = 0; i < p->ndv; i++) if (p->dv[i].mask == p->dnow) return p->dv[i].p;
    for (i = 0; i < p->nlbind; i++) { gb[i].name = p->lbind[i].name; gb[i].loc = (int)p->lbind[i].loc; }     /* the link's attribute locations, not today's bindings */
    if (g2sc_link_bound(p->bin_vs, p->bin_fs, gb, p->nlbind, p->dnow, &l) < 0) { g2e_log("gles: depth sampling: %s", l.error); return p->p; }
    hw = g2r_prog_create(S.r, l.vs.words, (uint32_t)l.vs.nwords, (const double (*)[4])l.vs.const_pool, (uint32_t)l.vs.nconst, (uint32_t)l.vs.const_base, l.fs.words, (uint32_t)l.fs.nwords,
                         (const double (*)[4])l.fs.const_pool, (uint32_t)l.fs.nconst, (uint32_t)l.fs.const_base, l.fsmask);
    g2sc_link_free(&l);
    if (!hw) return p->p;
    i = p->ndv < 8 ? p->ndv++ : p->dv_next++ % 8;
    p->dv[i].mask = p->dnow; p->dv[i].p = hw;
    return hw;
}

static void do_draw(GLenum mode, const uint32_t *idx, uint32_t n, uint32_t first)
{
    static const uint8_t HW[7] = { 3, 4, 6, 5, 0, 1, 2 };       /* GL_POINTS .. GL_TRIANGLE_FAN to the hardware's modes */
    es_prog *p = S.cur;
    es_tg g;
    g2r_state s;
    g2r_vsrc v[MAX_ATTR];
    void *tmp[MAX_ATTR];
    es_buf *from[MAX_ATTR];
    uint32_t lo = first, hi = first + n - 1, i, ioff = 0;
    int a, c;
    if (mode > GL_TRIANGLE_FAN) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!S.live) return;
    if (fb_target(&g) != GL_FRAMEBUFFER_COMPLETE) { SET_ERR(GL_INVALID_FRAMEBUFFER_OPERATION); return; }     /* §4.4.5: also a draw of nothing, or with no program */
    if (!p || !n) return;                                       /* §2.11: no current program, the draw has no effect */
    if (bind_target(&g) < 0) return;
    if (idx) {
        lo = 0xFFFFFFFFu; hi = 0;
        for (i = 0; i < n; i++) { if (idx[i] < lo) lo = idx[i]; if (idx[i] > hi) hi = idx[i]; }
    }
    memset(&s, 0, sizeof s);
    fill_state(&s, p, &g);
    /* board A drops what is outside x and y of the view volume only at its guard band: the scissor keeps every primitive to the viewport. So a point goes by near and far, and
       of a wide point the part inside the viewport is drawn, also when its centre is just outside */
    g2r_scissor_viewport(&s, g.t.width, g.t.height);
    s.prog = depth_variant(p);
    for (a = 0; a < 16; a++) { s.attr[a].size = 4; s.attr[a].type = G2_AT_FP24; s.attr[a].constant[3] = g2_fp24(1.0); }
    memset(v, 0, sizeof v);
    memset(tmp, 0, sizeof tmp);
    memset(from, 0, sizeof from);
    for (a = 0; a < p->nattr; a++) {
        for (c = 0; c < p->attr[a].n; c++) {                    /* a matrix: one generic attribute per column; a location is also the vertex core's input */
            int k = p->attr[a].loc + c, b, cc;
            es_va *va = &S.va[k];
            g2r_attr *d = &s.attr[k];
            if (!va->enabled) {
                for (cc = 0; cc < 4; cc++) d->constant[cc] = g2_fp24(va->cur[cc]);
                continue;
            }
            {
                const uint8_t *base = (const uint8_t *)va->ptr;
                int ty = at_type(va->type, &b), stride = va->stride ? va->stride : b * va->size;
                if (va->buffer) {
                    es_buf *o = buf_of(va->buffer);
                    if (!o || !o->data || (uintptr_t)va->ptr + (uint64_t)hi * stride + (uint64_t)b * va->size > (uint64_t)o->size) { SET_ERR(GL_INVALID_OPERATION); return; }
                    base = o->data + (uintptr_t)va->ptr;
                    from[k] = o;
                }
                if (!base) continue;
                v[k].base = base;
                v[k].bytes = b * va->size;
                v[k].stride = stride;
                d->enabled = 1; d->size = va->size; d->type = ty; d->normalized = va->normalized;
                d->handle = g2r_stream_handle(S.r);
                d->stride = (uint32_t)stride;
            }
        }
    }
    if (idx && hi - lo >= 65536) {                              /* the hardware's indices are 16 bits: separate vertices */
        for (a = 0; a < MAX_ATTR; a++) {
            uint8_t *t;
            if (!v[a].base) continue;
            t = (uint8_t *)malloc((size_t)n * v[a].bytes);
            if (!t) { SET_ERR(GL_OUT_OF_MEMORY); goto out; }
            for (i = 0; i < n; i++) memcpy(t + (size_t)i * v[a].bytes, v[a].base + (size_t)idx[i] * v[a].stride, (size_t)v[a].bytes);
            tmp[a] = t;
            v[a].base = t; v[a].stride = v[a].bytes;
            s.attr[a].stride = (uint32_t)v[a].bytes;
        }
        idx = NULL; lo = 0; hi = n - 1;
    }
    for (a = 0; a < MAX_ATTR; a++) {                            /* what board A already holds is not sent; vertex lo is what the indices below count from */
        es_buf *o = from[a];
        if (o && !tmp[a] && g2r_keep(S.r, &o->keep, o->data, (uint32_t)o->size)) {
            s.attr[a].handle = g2r_kept_handle(S.r);
            s.attr[a].offset = o->keep.off + (uint32_t)(v[a].base - o->data) + lo * (uint32_t)v[a].stride;
            v[a].base = NULL;
        }
    }
    g2r_gather(S.r, v, MAX_ATTR, lo, hi - lo + 1);
    for (a = 0; a < MAX_ATTR; a++) if (v[a].base) s.attr[a].offset = v[a].off;
    if (idx) {
        uint16_t *ib = (uint16_t *)malloc((size_t)n * 2);
        if (!ib) { SET_ERR(GL_OUT_OF_MEMORY); goto out; }
        for (i = 0; i < n; i++) ib[i] = (uint16_t)(idx[i] - lo);
        ioff = g2r_stream(S.r, ib, n * 2, 2);
        free(ib);
    }
    s.vs_uniforms = p->vu;
    s.fs_uniforms = p->fu;
    g2r_draw(S.r, &s, HW[mode], n, 0, idx != NULL, g2r_stream_handle(S.r), ioff, 0);
out:
    for (a = 0; a < MAX_ATTR; a++) free(tmp[a]);
}

/* an element of an array as floats, by §2.1.2's conversions: an attribute with a divisor is one value per instance */
static void va_value(const es_va *va, uint32_t e, float *o)
{
    const uint8_t *p = (const uint8_t *)va->ptr;
    int b, ty = at_type(va->type, &b), stride = va->stride ? va->stride : b * va->size, k;
    if (va->buffer) {
        es_buf *x = buf_of(va->buffer);
        if (!x || !x->data) return;
        p = x->data + (uintptr_t)va->ptr;
    }
    if (!p) return;
    p += (size_t)e * stride;
    o[0] = o[1] = o[2] = 0.0f; o[3] = 1.0f;
    for (k = 0; k < va->size; k++) {
        switch (ty) {
        case G2_AT_BYTE: { int8_t v = ((const int8_t *)p)[k]; o[k] = va->normalized ? (float)(2 * (int)v + 1) / 255.0f : (float)v; break; }
        case G2_AT_UBYTE: o[k] = va->normalized ? (float)p[k] / 255.0f : (float)p[k]; break;
        case G2_AT_SHORT: { int16_t v; memcpy(&v, p + 2 * k, 2); o[k] = va->normalized ? (float)(2 * (int)v + 1) / 65535.0f : (float)v; break; }
        case G2_AT_USHORT: { uint16_t v; memcpy(&v, p + 2 * k, 2); o[k] = va->normalized ? (float)v / 65535.0f : (float)v; break; }
        case G2_AT_FIXED: { int32_t v; memcpy(&v, p + 4 * k, 4); o[k] = (float)v / 65536.0f; break; }
        default: memcpy(&o[k], p + 4 * k, 4); break;
        }
    }
}

static int any_divisor(void)
{
    int a;
    for (a = 0; a < MAX_ATTR; a++) if (S.va[a].enabled && S.va[a].divisor) return 1;
    return 0;
}

/* ANGLE_instanced_arrays: the hardware has no instancing, so each instance is its own draw, with the arrays that have a divisor fed as the constant value of that instance */
static void draw_instanced(GLenum mode, const uint32_t *idx, uint32_t n, uint32_t first, GLsizei count)
{
    es_va keep[MAX_ATTR];
    GLsizei i;
    int a, zero = 0;
    for (a = 0; a < MAX_ATTR; a++) if (S.va[a].enabled && !S.va[a].divisor) zero = 1;
    if (!zero) { SET_ERR(GL_INVALID_OPERATION); return; }       /* the extension's one limit: at least one array has divisor 0 */
    memcpy(keep, S.va, sizeof keep);
    for (i = 0; i < count; i++) {
        for (a = 0; a < MAX_ATTR; a++) {
            if (!keep[a].enabled || !keep[a].divisor) continue;
            S.va[a].enabled = 0;
            va_value(&keep[a], (uint32_t)i / keep[a].divisor, S.va[a].cur);
        }
        do_draw(mode, idx, n, first);
    }
    memcpy(S.va, keep, sizeof keep);
}

void ES_APIENTRY glDrawArrays(GLenum m, GLint first, GLsizei n)
{
    if (first < 0 || n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (any_divisor()) { draw_instanced(m, NULL, (uint32_t)n, (uint32_t)first, 1); return; }
    do_draw(m, NULL, (uint32_t)n, (uint32_t)first);
}

static void draw_elements(GLenum m, GLsizei n, GLenum type, const void *indices, GLsizei instances)
{
    const uint8_t *ip = (const uint8_t *)indices;
    uint32_t *idx, i;
    int bytes = type == GL_UNSIGNED_BYTE ? 1 : type == GL_UNSIGNED_SHORT ? 2 : 4;
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (type != GL_UNSIGNED_BYTE && type != GL_UNSIGNED_SHORT && type != GL_UNSIGNED_INT) { SET_ERR(GL_INVALID_ENUM); return; }
    if (S.elem_buf) {
        es_buf *o = buf_of(S.elem_buf);
        if (!o || !o->data || (uintptr_t)indices + (uint64_t)n * bytes > (uint64_t)o->size) { SET_ERR(GL_INVALID_OPERATION); return; }
        ip = o->data + (uintptr_t)indices;
    }
    if (!n) { do_draw(m, NULL, 0, 0); return; }                 /* no vertices: the mode and the framebuffer are still checked */
    idx = (uint32_t *)malloc((size_t)n * 4);
    if (!idx) { SET_ERR(GL_OUT_OF_MEMORY); return; }
    for (i = 0; i < (uint32_t)n; i++) {
        if (bytes == 1) idx[i] = ip[i];
        else if (bytes == 2) { uint16_t x; memcpy(&x, ip + 2 * i, 2); idx[i] = x; }
        else memcpy(&idx[i], ip + 4 * i, 4);
    }
    if (any_divisor() || instances != 1) draw_instanced(m, idx, (uint32_t)n, 0, instances);
    else do_draw(m, idx, (uint32_t)n, 0);
    free(idx);
}

void ES_APIENTRY glDrawElements(GLenum m, GLsizei n, GLenum type, const void *indices) { draw_elements(m, n, type, indices, 1); }

/* ---------------------------------------------------------------- what an ANGLE build exports beyond ES 2.0 */
void ES_APIENTRY glVertexAttribDivisorANGLE(GLuint i, GLuint d) { if (i >= MAX_ATTR) { SET_ERR(GL_INVALID_VALUE); return; } S.va[i].divisor = d; }

/* GL_OES_vertex_array_object: S.va holds the bound object's arrays; the current generic values stay with the context */
static void vao_copy(es_va *d, const es_va *s)
{
    int a;
    for (a = 0; a < MAX_ATTR; a++) {
        float c[4];
        memcpy(c, d[a].cur, sizeof c);
        d[a] = s[a];
        memcpy(d[a].cur, c, sizeof c);
    }
}
static es_vao *vao_of(GLuint n) { es_vao *o = S.vaos; while (o && o->name != n) o = o->next; return o; }
void ES_APIENTRY glGenVertexArraysOES(GLsizei n, GLuint *a)
{
    GLsizei i;
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        es_vao *o = (es_vao *)calloc(1, sizeof *o);
        int k;
        if (!o) { SET_ERR(GL_OUT_OF_MEMORY); return; }
        for (k = 0; k < MAX_ATTR; k++) { o->va[k].size = 4; o->va[k].type = GL_FLOAT; }
        o->name = a[i] = gen_name();
        o->next = S.vaos; S.vaos = o;
    }
}
void ES_APIENTRY glBindVertexArrayOES(GLuint n)
{
    es_vao *o = n ? vao_of(n) : &S.vao0, *c = S.vao ? vao_of(S.vao) : &S.vao0;
    if (!o) { SET_ERR(GL_INVALID_OPERATION); return; }
    vao_copy(c->va, S.va); c->elem_buf = S.elem_buf;
    vao_copy(S.va, o->va); S.elem_buf = o->elem_buf;
    S.vao = n;
}
void ES_APIENTRY glDeleteVertexArraysOES(GLsizei n, const GLuint *a)
{
    GLsizei i;
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        es_vao **pp = &S.vaos;
        if (!a[i]) continue;
        if (S.vao == a[i]) glBindVertexArrayOES(0);
        while (*pp && (*pp)->name != a[i]) pp = &(*pp)->next;
        if (*pp) { es_vao *o = *pp; *pp = o->next; free(o); }
    }
}
GLboolean ES_APIENTRY glIsVertexArrayOES(GLuint n) { return n && vao_of(n); }

/* GL_EXT_discard_framebuffer: a hint, and contents that are undefined may stay as they are */
void ES_APIENTRY glDiscardFramebufferEXT(GLenum t, GLsizei n, const GLenum *a)
{
    GLsizei i;
    if (t != GL_FRAMEBUFFER) { SET_ERR(GL_INVALID_ENUM); return; }
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        GLenum e = a[i];
        if (S.fb ? e != GL_COLOR_ATTACHMENT0 && e != GL_DEPTH_ATTACHMENT && e != GL_STENCIL_ATTACHMENT : e != GL_COLOR_EXT && e != GL_DEPTH_EXT && e != GL_STENCIL_EXT) { SET_ERR(GL_INVALID_ENUM); return; }
    }
}
void ES_APIENTRY glDrawArraysInstancedANGLE(GLenum m, GLint first, GLsizei n, GLsizei count)
{
    if (first < 0 || n < 0 || count < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (count) draw_instanced(m, NULL, (uint32_t)n, (uint32_t)first, count);
}
void ES_APIENTRY glDrawElementsInstancedANGLE(GLenum m, GLsizei n, GLenum type, const void *indices, GLsizei count)
{
    if (count < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    if (count) draw_elements(m, n, type, indices, count);
}

/* NV_fence. A test or a finish waits for the board: the extension lets TestFence say so, it only must not say done too early */
typedef struct es_fence { struct es_fence *next; GLuint name; int set, done; } es_fence;
static es_fence *g_fences;
static es_fence *fence_of(GLuint n) { es_fence *f; for (f = g_fences; f; f = f->next) if (f->name == n) return f; return NULL; }
void ES_APIENTRY glGenFencesNV(GLsizei n, GLuint *o) { GLsizei i; if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; } for (i = 0; i < n; i++) o[i] = gen_name(); }
void ES_APIENTRY glDeleteFencesNV(GLsizei n, const GLuint *o)
{
    GLsizei i;
    if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    for (i = 0; i < n; i++) {
        es_fence **p = &g_fences;
        while (*p) { es_fence *f = *p; if (f->name == o[i]) { *p = f->next; free(f); break; } p = &f->next; }
    }
}
GLboolean ES_APIENTRY glIsFenceNV(GLuint n) { return fence_of(n) != NULL; }             /* a name is a fence once it has been set */
void ES_APIENTRY glSetFenceNV(GLuint n, GLenum cond)
{
    es_fence *f = fence_of(n);
    if (cond != GL_ALL_COMPLETED_NV) { SET_ERR(GL_INVALID_ENUM); return; }
    if (!f) {
        f = (es_fence *)calloc(1, sizeof *f);
        if (!f) { SET_ERR(GL_OUT_OF_MEMORY); return; }
        f->name = n; f->next = g_fences; g_fences = f;
    }
    f->set = 1; f->done = 0;
    if (S.live) g2r_end_pass(S.r);
}
GLboolean ES_APIENTRY glTestFenceNV(GLuint n)
{
    es_fence *f = fence_of(n);
    if (!f) { SET_ERR(GL_INVALID_OPERATION); return GL_TRUE; }
    if (!f->done && S.live) g2r_finish(S.r);
    f->done = 1;
    return GL_TRUE;
}
void ES_APIENTRY glFinishFenceNV(GLuint n) { es_fence *f = fence_of(n); if (!f) { SET_ERR(GL_INVALID_OPERATION); return; } glTestFenceNV(n); }
void ES_APIENTRY glGetFenceivNV(GLuint n, GLenum p, GLint *v)
{
    es_fence *f = fence_of(n);
    if (!f) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (p == GL_FENCE_STATUS_NV) *v = glTestFenceNV(n);
    else if (p == GL_FENCE_CONDITION_NV) *v = GL_ALL_COMPLETED_NV;
    else SET_ERR(GL_INVALID_ENUM);
}

/* EXT_occlusion_query_boolean: board B counts quads per frame, as a snapshot of one moment, and nothing per query. Not advertised: the names work, and a query cannot begin */
void ES_APIENTRY glGenQueriesEXT(GLsizei n, GLuint *o) { GLsizei i; if (n < 0) { SET_ERR(GL_INVALID_VALUE); return; } for (i = 0; i < n; i++) o[i] = gen_name(); }
void ES_APIENTRY glDeleteQueriesEXT(GLsizei n, const GLuint *o) { (void)o; if (n < 0) SET_ERR(GL_INVALID_VALUE); }
GLboolean ES_APIENTRY glIsQueryEXT(GLuint n) { (void)n; return GL_FALSE; }
void ES_APIENTRY glBeginQueryEXT(GLenum t, GLuint n) { (void)t; (void)n; SET_ERR(GL_INVALID_ENUM); }
void ES_APIENTRY glEndQueryEXT(GLenum t) { (void)t; SET_ERR(GL_INVALID_ENUM); }
void ES_APIENTRY glGetQueryivEXT(GLenum t, GLenum p, GLint *v) { (void)t; (void)p; (void)v; SET_ERR(GL_INVALID_ENUM); }
void ES_APIENTRY glGetQueryObjectuivEXT(GLuint n, GLenum p, GLuint *v) { (void)n; (void)p; (void)v; SET_ERR(GL_INVALID_OPERATION); }

/* EXT_draw_buffers: one colour output, so a single buffer or none */
void ES_APIENTRY glDrawBuffersEXT(GLsizei n, const GLenum *b)
{
    if (n < 0 || n > 1) { SET_ERR(GL_INVALID_VALUE); return; }
    if (n == 0 || b[0] == GL_NONE) { S.draw_none = 1; return; }
    if (b[0] != (S.fb ? GL_COLOR_ATTACHMENT0 : GL_BACK)) { SET_ERR(GL_INVALID_OPERATION); return; }
    S.draw_none = 0;
}

/* ANGLE_framebuffer_blit: no scaling and no mirror, which is that extension's own limit; depth and stencil of a whole surface */
void ES_APIENTRY glBlitFramebufferANGLE(GLint sx0, GLint sy0, GLint sx1, GLint sy1, GLint dx0, GLint dy0, GLint dx1, GLint dy1, GLbitfield mask, GLenum filter)
{
    es_tg src, dst;
    GLuint draw_fb = S.fb;
    int w = sx1 - sx0, h = sy1 - sy0, bound;
    if (mask & ~(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)) { SET_ERR(GL_INVALID_VALUE); return; }
    if (filter != GL_NEAREST) { SET_ERR(GL_INVALID_ENUM); return; }
    if (w != dx1 - dx0 || h != dy1 - dy0 || w < 0 || h < 0) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (!S.live || !w || !h) return;
    S.fb = S.read_fb;
    bound = bind_target(&src);
    S.fb = draw_fb;
    if (bound < 0 || bind_target(&dst) < 0) return;
    if (src.t.color == dst.t.color && S.read_fb == S.fb) { SET_ERR(GL_INVALID_OPERATION); return; }
    if (mask & GL_COLOR_BUFFER_BIT) {
        if (src.fmt != dst.fmt) { SET_ERR(GL_INVALID_OPERATION); return; }
        if (sx0 < 0 || sy0 < 0 || dx0 < 0 || dy0 < 0 || sx1 > src.vw || sy1 > src.vh || dx1 > dst.vw || dy1 > dst.vh) { SET_ERR(GL_INVALID_OPERATION); return; }
    }
    if (mask & (GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)) {
        if (!src.t.ds || !dst.t.ds || src.t.width != dst.t.width || src.t.height != dst.t.height || sx0 || sy0 || dx0 || dy0 || w != src.vw || h != src.vh) { SET_ERR(GL_INVALID_OPERATION); return; }
    }
    g2r_finish(S.r);
    if (mask & GL_COLOR_BUFFER_BIT) g2_copy_image(S.bd, src.t.color, dst.t.color, (uint32_t)sx0, (uint32_t)sy0, (uint32_t)w, (uint32_t)h, (uint32_t)dx0, (uint32_t)dy0, (uint32_t)w, (uint32_t)h, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (mask & (GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT)) {
        size_t n = (size_t)w * h;
        uint32_t *a = (uint32_t *)malloc(n * 4), *b = (uint32_t *)malloc(n * 4), keep = (mask & GL_DEPTH_BUFFER_BIT ? 0xFFFFFFu : 0) | (mask & GL_STENCIL_BUFFER_BIT ? 0xFF000000u : 0);
        size_t i;
        if (!a || !b) { free(a); free(b); SET_ERR(GL_OUT_OF_MEMORY); return; }
        g2_read_image(S.bd, src.t.ds, G2_TF_RAW, 0, 0, (uint32_t)w, (uint32_t)h, 0, 0, G2_FMT_D24S8, 0, 0, a);
        g2_read_image(S.bd, dst.t.ds, G2_TF_RAW, 0, 0, (uint32_t)w, (uint32_t)h, 0, 0, G2_FMT_D24S8, 0, 0, b);
        for (i = 0; i < n; i++) b[i] = (b[i] & ~keep) | (a[i] & keep);
        g2_write_image(S.bd, dst.t.ds, b, n * 4, G2_TF_RAW, 0, 0, (uint32_t)w, (uint32_t)h, 0, 0, 0, G2_FMT_D24S8);
        free(a); free(b);
    }
}

void ES_APIENTRY glRenderbufferStorageMultisampleANGLE(GLenum t, GLsizei samples, GLenum f, GLsizei w, GLsizei h)
{
    if (samples) { SET_ERR(GL_INVALID_VALUE); return; }         /* GL_MAX_SAMPLES is 0: the hardware has one sample a pixel */
    glRenderbufferStorage(t, f, w, h);
}

void ES_APIENTRY glGetTranslatedShaderSourceANGLE(GLuint n, GLsizei max, GLsizei *len, GLchar *o)
{
    es_shader *s = sh_of(n);
    GLsizei l;
    if (!s) { SET_ERR(GL_INVALID_VALUE); return; }
    if (max < 0) { SET_ERR(GL_INVALID_VALUE); return; }
    l = s->compiled_src ? (GLsizei)strlen(s->compiled_src) : 0;
    if (l >= max) l = max > 0 ? max - 1 : 0;
    if (max > 0) { if (l) memcpy(o, s->compiled_src, (size_t)l); o[l] = 0; }
    if (len) *len = l;
}

/* EXT_robustness: nothing here ever loses its context on the application, and the sized calls check the room */
GLenum ES_APIENTRY glGetGraphicsResetStatusEXT(void) { return GL_NO_ERROR; }
void ES_APIENTRY glReadnPixelsEXT(GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, GLsizei size, void *o)
{
    int row = (w * 4 + S.pk_align - 1) / S.pk_align * S.pk_align;
    if (w > 0 && h > 0 && (long long)row * (h - 1) + (long long)w * 4 > size) { SET_ERR(GL_INVALID_OPERATION); return; }
    glReadPixels(x, y, w, h, format, type, o);
}
static int uni_bytes(GLuint n, GLint loc)
{
    es_prog *p = pr_of(n);
    if (!p || !p->linked || loc < 0 || (loc >> 8) >= p->nuni) return 0;
    return p->uni[loc >> 8].comps * p->uni[loc >> 8].stride * 4;
}
void ES_APIENTRY glGetnUniformfvEXT(GLuint n, GLint loc, GLsizei size, GLfloat *v)
{
    if (size < uni_bytes(n, loc)) { SET_ERR(GL_INVALID_OPERATION); return; }
    glGetUniformfv(n, loc, v);
}
void ES_APIENTRY glGetnUniformivEXT(GLuint n, GLint loc, GLsizei size, GLint *v)
{
    if (size < uni_bytes(n, loc)) { SET_ERR(GL_INVALID_OPERATION); return; }
    glGetUniformiv(n, loc, v);
}

/* EXT_texture_storage */
void ES_APIENTRY glTexStorage2DEXT(GLenum target, GLsizei levels, GLenum ifmt, GLsizei w, GLsizei h)
{
    static const struct { GLenum i, f, t; } M[] = {
        { GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE }, { GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE }, { GL_RGB565, GL_RGB, GL_UNSIGNED_SHORT_5_6_5 }, { GL_RGBA4, GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4 },
        { GL_RGB5_A1, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1 }, { GL_LUMINANCE8_EXT, GL_LUMINANCE, GL_UNSIGNED_BYTE }, { GL_ALPHA8_EXT, GL_ALPHA, GL_UNSIGNED_BYTE },
        { GL_LUMINANCE8_ALPHA8_EXT, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE }, { GL_BGRA8_EXT, GL_BGRA_EXT, GL_UNSIGNED_BYTE },
        { GL_DEPTH_COMPONENT16, GL_DEPTH_COMPONENT, GL_UNSIGNED_SHORT }, { GL_DEPTH_COMPONENT32_OES, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT }, { GL_DEPTH24_STENCIL8, GL_DEPTH_STENCIL_OES, GL_UNSIGNED_INT_24_8_OES }
    };
    int cube = target == GL_TEXTURE_CUBE_MAP, faces = cube ? 6 : 1, m = -1, l, f;
    es_tex *t;
    size_t i;
    if (target != GL_TEXTURE_2D && !cube) { SET_ERR(GL_INVALID_ENUM); return; }
    for (i = 0; i < sizeof M / sizeof M[0]; i++) if (M[i].i == ifmt) m = (int)i;
    if (m < 0) { SET_ERR(GL_INVALID_ENUM); return; }
    if (levels < 1 || w < 1 || h < 1) { SET_ERR(GL_INVALID_VALUE); return; }
    if (levels > 1 + ilog2(w > h ? w : h) || (cube && w != h)) { SET_ERR(GL_INVALID_OPERATION); return; }
    if ((M[m].f == GL_DEPTH_COMPONENT || M[m].f == GL_DEPTH_STENCIL_OES) && (cube || levels > 1)) { SET_ERR(GL_INVALID_OPERATION); return; }
    t = unit_tex(S.active, cube);
    if (!t || !t->name || t->immutable) { SET_ERR(GL_INVALID_OPERATION); return; }
    S.tex_storing = levels;
    for (l = 0; l < levels; l++) {
        for (f = 0; f < faces; f++) {
            GLenum e = cube ? (GLenum)(GL_TEXTURE_CUBE_MAP_POSITIVE_X + f) : GL_TEXTURE_2D;
            glTexImage2D(e, l, (GLint)M[m].f, mip_dim(w, l), mip_dim(h, l), 0, M[m].f, M[m].t, NULL);
        }
    }
    S.tex_storing = 0;
    t->immutable = levels;
}

void ES_APIENTRY glTexStorage3DEXT(GLenum target, GLsizei levels, GLenum ifmt, GLsizei w, GLsizei h, GLsizei d)
{
    static const struct { GLenum i, f, t; } M[] = {
        { GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE }, { GL_RGB8, GL_RGB, GL_UNSIGNED_BYTE }, { GL_RGB565, GL_RGB, GL_UNSIGNED_SHORT_5_6_5 }, { GL_RGBA4, GL_RGBA, GL_UNSIGNED_SHORT_4_4_4_4 }, { GL_RGB5_A1, GL_RGBA, GL_UNSIGNED_SHORT_5_5_5_1 },
        { GL_ALPHA8_EXT, GL_ALPHA, GL_UNSIGNED_BYTE }, { GL_LUMINANCE8_EXT, GL_LUMINANCE, GL_UNSIGNED_BYTE }, { GL_LUMINANCE8_ALPHA8_EXT, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE }, { GL_BGRA8_EXT, GL_BGRA_EXT, GL_UNSIGNED_BYTE }
    };
    int m = -1, l, top = w > h ? w : h;
    es_tex *t;
    size_t i;
    if (target != GL_TEXTURE_3D_OES || !has_3d()) { SET_ERR(GL_INVALID_ENUM); return; }
    for (i = 0; i < sizeof M / sizeof M[0]; i++) if (M[i].i == ifmt) m = (int)i;
    if (m < 0) { SET_ERR(GL_INVALID_ENUM); return; }
    if (levels < 1 || w < 1 || h < 1 || d < 1) { SET_ERR(GL_INVALID_VALUE); return; }
    if (levels > 1 + ilog2(top > d ? top : d)) { SET_ERR(GL_INVALID_OPERATION); return; }
    t = unit_tex(S.active, 2);
    if (!t || !t->name || t->immutable) { SET_ERR(GL_INVALID_OPERATION); return; }
    S.tex_storing = levels;
    for (l = 0; l < levels; l++) glTexImage3DOES(target, l, M[m].f, mip_dim(w, l), mip_dim(h, l), mip_dim(d, l), 0, M[m].f, M[m].t, NULL);
    S.tex_storing = 0;
    t->immutable = levels;
}

/* OES_get_program_binary */
void ES_APIENTRY glGetProgramBinaryOES(GLuint n, GLsizei size, GLsizei *len, GLenum *format, void *out)
{
    es_prog *p = pr_of(n);
    uint32_t h[4];
    size_t need;
    uint8_t *o = (uint8_t *)out;
    if (!p || !p->linked || !p->bin_vs) { SET_ERR(GL_INVALID_OPERATION); return; }
    h[0] = G2_PROGRAM_BINARY; h[1] = (uint32_t)strlen(p->bin_vs); h[2] = (uint32_t)strlen(p->bin_fs); h[3] = (uint32_t)p->nbind;
    need = sizeof h + h[1] + h[2] + (size_t)p->nbind * sizeof p->bind[0];
    if ((size_t)size < need) { SET_ERR(GL_INVALID_OPERATION); return; }
    memcpy(o, h, sizeof h); o += sizeof h;
    memcpy(o, p->bin_vs, h[1]); o += h[1];
    memcpy(o, p->bin_fs, h[2]); o += h[2];
    memcpy(o, p->bind, (size_t)p->nbind * sizeof p->bind[0]);
    if (len) *len = (GLsizei)need;
    if (format) *format = G2_PROGRAM_BINARY;
}

void ES_APIENTRY glProgramBinaryOES(GLuint n, GLenum format, const void *bin, GLsizei len)
{
    es_prog *p = pr_of(n);
    const uint8_t *b = (const uint8_t *)bin;
    uint32_t h[4];
    GLuint vs, fs;
    GLint vl, fl;
    char *v, *f;
    if (!p) { SET_ERR(GL_INVALID_VALUE); return; }
    if (format != G2_PROGRAM_BINARY) { SET_ERR(GL_INVALID_ENUM); return; }
    p->linked = 0;
    snprintf(p->log, sizeof p->log, "the program binary is not valid");
    if (len < (GLsizei)sizeof h) return;
    memcpy(h, b, sizeof h);
    if (h[0] != G2_PROGRAM_BINARY || h[3] > sizeof p->bind / sizeof p->bind[0] || (size_t)len != sizeof h + (size_t)h[1] + h[2] + (size_t)h[3] * sizeof p->bind[0]) return;
    v = (char *)malloc((size_t)h[1] + 1); f = (char *)malloc((size_t)h[2] + 1);
    if (!v || !f) { free(v); free(f); SET_ERR(GL_OUT_OF_MEMORY); return; }
    memcpy(v, b + sizeof h, h[1]); v[h[1]] = 0;
    memcpy(f, b + sizeof h + h[1], h[2]); f[h[2]] = 0;
    memcpy(p->bind, b + sizeof h + h[1] + h[2], (size_t)h[3] * sizeof p->bind[0]);
    p->nbind = (int)h[3];
    vs = glCreateShader(GL_VERTEX_SHADER); fs = glCreateShader(GL_FRAGMENT_SHADER);
    vl = (GLint)h[1]; fl = (GLint)h[2];
    { const GLchar *s = v; glShaderSource(vs, 1, &s, &vl); s = f; glShaderSource(fs, 1, &s, &fl); }
    glCompileShader(vs); glCompileShader(fs);
    glAttachShader(n, vs); glAttachShader(n, fs);
    glLinkProgram(n);
    glDetachShader(n, vs); glDetachShader(n, fs);
    glDeleteShader(vs); glDeleteShader(fs);
    free(v); free(f);
}

static void init_state(void)
{
    int f, a;
    memset(&S.ctx_begin, 0, (size_t)(&S.ctx_end - &S.ctx_begin));
    S.depth_func = GL_LESS; S.cull_mode = GL_BACK; S.front_face = GL_CCW; S.dither = 1;
    S.b_src = S.b_src_a = GL_ONE; S.b_dst = S.b_dst_a = GL_ZERO; S.b_eq = S.b_eq_a = GL_FUNC_ADD;
    S.depth_mask = 1; S.color_mask = 15; S.clear_depth = 1.0f; S.depth_f = 1.0f; S.line_width = 1.0f; S.sample_val = 1.0f;
    S.hint_deriv = GL_DONT_CARE;
    for (a = 0; a < MAX_ATTR; a++) { S.vao0.va[a].size = 4; S.vao0.va[a].type = GL_FLOAT; }
    for (f = 0; f < 2; f++) { S.st_func[f] = GL_ALWAYS; S.st_fail[f] = S.st_zfail[f] = S.st_zpass[f] = GL_KEEP; S.st_vmask[f] = S.st_wmask[f] = 0xFFFFFFFFu; }
    S.un_align = S.pk_align = 4;
    S.hint_mip = GL_DONT_CARE;
    for (a = 0; a < MAX_ATTR; a++) { S.va[a].cur[3] = 1.0f; S.va[a].size = 4; S.va[a].type = GL_FLOAT; }
}

/* ---------------------------------------------------------------- EGL (libEGL.dll forwards here) */
typedef void *EGLDisplay, *EGLConfig, *EGLSurface, *EGLContext;
typedef int EGLint, EGLBoolean;
typedef unsigned int EGLenum;
typedef HWND EGLNativeWindowType;
#define EGLAPI WINAPI
enum { EGL_SUCCESS = 0x3000, EGL_NOT_INITIALIZED = 0x3001, EGL_BAD_ATTRIBUTE = 0x3004, EGL_BAD_CONFIG = 0x3005, EGL_BAD_CONTEXT = 0x3006, EGL_BAD_DISPLAY = 0x3008, EGL_BAD_MATCH = 0x3009, EGL_BAD_PARAMETER = 0x300C, EGL_BAD_SURFACE = 0x300D,
       EGL_NONE = 0x3038, EGL_BUFFER_SIZE = 0x3020, EGL_ALPHA_SIZE = 0x3021, EGL_BLUE_SIZE = 0x3022, EGL_GREEN_SIZE = 0x3023, EGL_RED_SIZE = 0x3024,
       EGL_DEPTH_SIZE = 0x3025, EGL_STENCIL_SIZE = 0x3026, EGL_CONFIG_CAVEAT = 0x3027, EGL_CONFIG_ID = 0x3028, EGL_LEVEL = 0x3029, EGL_MAX_PBUFFER_HEIGHT = 0x302A, EGL_MAX_PBUFFER_PIXELS = 0x302B, EGL_MAX_PBUFFER_WIDTH = 0x302C,
       EGL_NATIVE_RENDERABLE = 0x302D, EGL_NATIVE_VISUAL_ID = 0x302E, EGL_NATIVE_VISUAL_TYPE = 0x302F, EGL_SAMPLES = 0x3031, EGL_SAMPLE_BUFFERS = 0x3032, EGL_SURFACE_TYPE = 0x3033, EGL_TRANSPARENT_TYPE = 0x3034,
       EGL_TRANSPARENT_BLUE_VALUE = 0x3035, EGL_TRANSPARENT_GREEN_VALUE = 0x3036, EGL_TRANSPARENT_RED_VALUE = 0x3037, EGL_BIND_TO_TEXTURE_RGB = 0x3039, EGL_BIND_TO_TEXTURE_RGBA = 0x303A, EGL_MIN_SWAP_INTERVAL = 0x303B,
       EGL_MAX_SWAP_INTERVAL = 0x303C, EGL_LUMINANCE_SIZE = 0x303D, EGL_ALPHA_MASK_SIZE = 0x303E, EGL_COLOR_BUFFER_TYPE = 0x303F, EGL_RENDERABLE_TYPE = 0x3040, EGL_MATCH_NATIVE_PIXMAP = 0x3041, EGL_CONFORMANT = 0x3042, EGL_RGB_BUFFER = 0x308E,
       EGL_OPENGL_ES2_BIT = 4, EGL_WINDOW_BIT = 4, EGL_PBUFFER_BIT = 1, EGL_DONT_CARE = -1,
       EGL_VENDOR = 0x3053, EGL_VERSION = 0x3054, EGL_EXTENSIONS = 0x3055, EGL_CLIENT_APIS = 0x308D, EGL_WIDTH = 0x3057, EGL_HEIGHT = 0x3056, EGL_OPENGL_ES_API = 0x30A0, EGL_DRAW = 0x3059, EGL_READ = 0x305A,
       EGL_RENDER_BUFFER = 0x3086, EGL_BACK_BUFFER = 0x3084, EGL_SWAP_BEHAVIOR = 0x3093, EGL_BUFFER_DESTROYED = 0x3095, EGL_CONTEXT_CLIENT_TYPE = 0x3097, EGL_CONTEXT_CLIENT_VERSION = 0x3098 };

typedef struct egl_surf { HWND hwnd; int w, h, cfg; } egl_surf;
typedef struct egl_ctx { char *saved; int cfg; } egl_ctx;
static EGLint g_egl_err = EGL_SUCCESS;
static egl_surf *g_cur_surf;
static egl_ctx *g_cur_ctx;
static int g_egl_init;
#define E_OK() (g_egl_err = EGL_SUCCESS)
#define DISPLAY ((EGLDisplay)1)
#define CHECK_DPY(d, ret) do { if ((d) != DISPLAY) { g_egl_err = EGL_BAD_DISPLAY; return ret; } if (!g_egl_init) { g_egl_err = EGL_NOT_INITIALIZED; return ret; } } while (0)

EGLint EGLAPI eglGetError(void) { EGLint e = g_egl_err; g_egl_err = EGL_SUCCESS; return e; }
EGLDisplay EGLAPI eglGetDisplay(void *id) { (void)id; E_OK(); return DISPLAY; }
EGLBoolean EGLAPI eglInitialize(EGLDisplay d, EGLint *maj, EGLint *min)
{
    if (d != DISPLAY) { g_egl_err = EGL_BAD_DISPLAY; return 0; }
    g_egl_init = 1;
    if (maj) *maj = 1;
    if (min) *min = 4;
    E_OK();
    return 1;
}
EGLBoolean EGLAPI eglTerminate(EGLDisplay d) { if (d != DISPLAY) { g_egl_err = EGL_BAD_DISPLAY; return 0; } E_OK(); return 1; }
const char *EGLAPI eglQueryString(EGLDisplay d, EGLint n)
{
    CHECK_DPY(d, NULL);
    E_OK();
    switch (n) {
    case EGL_VENDOR: return "gpu2";
    case EGL_VERSION: return "1.4 gpu2";
    case EGL_EXTENSIONS: return "";
    case EGL_CLIENT_APIS: return "OpenGL_ES";
    default: g_egl_err = EGL_BAD_PARAMETER; return NULL;
    }
}

/* configs: what B renders and scans out, colour RGBA8 or RGB565, depth of 24 bits with stencil, of 16 (B's DEPTH_COMPONENT16 mode) or none. Returns 0 for attributes that are 0 here, -1 for
 * names that do not exist */
static int cfg_ok(EGLConfig c) { return (intptr_t)c >= 1 && (intptr_t)c <= NCFG; }

static EGLint cfg_attr(int c, EGLint a)
{
    int r565 = CFG[c - 1].color == 16;
    switch (a) {
    case EGL_BUFFER_SIZE: return CFG[c - 1].color;
    case EGL_RED_SIZE: case EGL_BLUE_SIZE: return r565 ? 5 : 8;
    case EGL_GREEN_SIZE: return r565 ? 6 : 8;
    case EGL_ALPHA_SIZE: return r565 ? 0 : 8;
    case EGL_DEPTH_SIZE: return CFG[c - 1].depth;
    case EGL_STENCIL_SIZE: return CFG[c - 1].stencil;
    case EGL_CONFIG_ID: return c;
    case EGL_SURFACE_TYPE: return EGL_WINDOW_BIT | EGL_PBUFFER_BIT;
    case EGL_RENDERABLE_TYPE: case EGL_CONFORMANT: return EGL_OPENGL_ES2_BIT;
    case EGL_COLOR_BUFFER_TYPE: return EGL_RGB_BUFFER;
    case EGL_CONFIG_CAVEAT: case EGL_TRANSPARENT_TYPE: return EGL_NONE;
    case EGL_MIN_SWAP_INTERVAL: return 0;
    case EGL_MAX_SWAP_INTERVAL: return 1;
    case EGL_MAX_PBUFFER_WIDTH: return MAX_RT_W;
    case EGL_MAX_PBUFFER_HEIGHT: return MAX_TEX;
    case EGL_MAX_PBUFFER_PIXELS: return MAX_RT_W * MAX_TEX;
    case EGL_SAMPLES: case EGL_SAMPLE_BUFFERS: case EGL_LEVEL: case EGL_NATIVE_RENDERABLE: case EGL_NATIVE_VISUAL_ID: case EGL_NATIVE_VISUAL_TYPE: case EGL_BIND_TO_TEXTURE_RGB: case EGL_BIND_TO_TEXTURE_RGBA:
    case EGL_LUMINANCE_SIZE: case EGL_ALPHA_MASK_SIZE: case EGL_TRANSPARENT_RED_VALUE: case EGL_TRANSPARENT_GREEN_VALUE: case EGL_TRANSPARENT_BLUE_VALUE: return 0;
    default: return -1;
    }
}
/* §3.4.1's selection: at least, exact or a mask, per attribute */
static int cfg_match(int c, const EGLint *a)
{
    for (; a && a[0] != EGL_NONE; a += 2) {
        EGLint have = cfg_attr(c, a[0]), v = a[1];
        if (v == EGL_DONT_CARE) continue;
        switch (a[0]) {
        case EGL_BUFFER_SIZE: case EGL_RED_SIZE: case EGL_GREEN_SIZE: case EGL_BLUE_SIZE: case EGL_ALPHA_SIZE: case EGL_DEPTH_SIZE: case EGL_STENCIL_SIZE: case EGL_SAMPLES: case EGL_SAMPLE_BUFFERS: case EGL_LUMINANCE_SIZE: case EGL_ALPHA_MASK_SIZE:
            if (have < v) return 0;
            break;
        case EGL_SURFACE_TYPE: case EGL_RENDERABLE_TYPE: case EGL_CONFORMANT:
            if ((have & v) != v) return 0;
            break;
        case EGL_CONFIG_ID: case EGL_COLOR_BUFFER_TYPE: case EGL_LEVEL: case EGL_CONFIG_CAVEAT: case EGL_TRANSPARENT_TYPE:
            if (have != v) return 0;
            break;
        case EGL_BIND_TO_TEXTURE_RGB: case EGL_BIND_TO_TEXTURE_RGBA:
            if (v == 1) return 0;
            break;
        default: break;
        }
    }
    return 1;
}
EGLBoolean EGLAPI eglGetConfigs(EGLDisplay d, EGLConfig *c, EGLint n, EGLint *out)
{
    int i;
    CHECK_DPY(d, 0);
    if (!out) { g_egl_err = EGL_BAD_PARAMETER; return 0; }
    if (c) for (i = 0; i < n && i < NCFG; i++) c[i] = (EGLConfig)(intptr_t)(i + 1);
    *out = c ? (n < NCFG ? n : NCFG) : NCFG;
    E_OK();
    return 1;
}
/* §3.4.1.2's order: more of the colour bits asked for first, then the smaller buffer, depth, stencil and config id */
static int cfg_before(int a, int b, const EGLint *attr)
{
    static const EGLint COL[4] = { EGL_RED_SIZE, EGL_GREEN_SIZE, EGL_BLUE_SIZE, EGL_ALPHA_SIZE };
    EGLint ca = 0, cb = 0;
    const EGLint *p;
    int i;
    for (p = attr; p && p[0] != EGL_NONE; p += 2) for (i = 0; i < 4; i++) if (p[0] == COL[i] && p[1] != 0 && p[1] != EGL_DONT_CARE) { ca += cfg_attr(a, COL[i]); cb += cfg_attr(b, COL[i]); }
    if (ca != cb) return ca > cb;
    if (CFG[a - 1].color != CFG[b - 1].color) return CFG[a - 1].color < CFG[b - 1].color;
    if (CFG[a - 1].depth != CFG[b - 1].depth) return CFG[a - 1].depth < CFG[b - 1].depth;
    if (CFG[a - 1].stencil != CFG[b - 1].stencil) return CFG[a - 1].stencil < CFG[b - 1].stencil;
    return a < b;
}

EGLBoolean EGLAPI eglChooseConfig(EGLDisplay d, const EGLint *attr, EGLConfig *c, EGLint n, EGLint *out)
{
    int k = 0, i, j, m[NCFG];
    CHECK_DPY(d, 0);
    if (!out) { g_egl_err = EGL_BAD_PARAMETER; return 0; }
    for (i = 1; i <= NCFG; i++) {
        if (!cfg_match(i, attr)) continue;
        for (j = k; j > 0 && cfg_before(i, m[j - 1], attr); j--) m[j] = m[j - 1];
        m[j] = i;
        k++;
    }
    for (i = 0; c && i < k && i < n; i++) c[i] = (EGLConfig)(intptr_t)m[i];
    *out = c ? (k < n ? k : n) : k;
    E_OK();
    return 1;
}
EGLBoolean EGLAPI eglGetConfigAttrib(EGLDisplay d, EGLConfig c, EGLint a, EGLint *v)
{
    int i = (int)(intptr_t)c;
    CHECK_DPY(d, 0);
    if (!cfg_ok(c)) { g_egl_err = EGL_BAD_CONFIG; return 0; }
    *v = cfg_attr(i, a);
    if (*v == -1) { g_egl_err = EGL_BAD_ATTRIBUTE; return 0; }
    E_OK();
    return 1;
}
EGLSurface EGLAPI eglCreateWindowSurface(EGLDisplay d, EGLConfig c, EGLNativeWindowType w, const EGLint *a)
{
    egl_surf *s;
    (void)a;
    CHECK_DPY(d, NULL);
    if (!cfg_ok(c)) { g_egl_err = EGL_BAD_CONFIG; return NULL; }
    s = (egl_surf *)calloc(1, sizeof *s);
    if (!s) return NULL;
    s->hwnd = w; s->cfg = (int)(intptr_t)c;
    win_size(w, &s->w, &s->h);
    E_OK();
    return s;
}
EGLSurface EGLAPI eglCreatePbufferSurface(EGLDisplay d, EGLConfig c, const EGLint *a)
{
    egl_surf *s;
    CHECK_DPY(d, NULL);
    if (!cfg_ok(c)) { g_egl_err = EGL_BAD_CONFIG; return NULL; }
    s = (egl_surf *)calloc(1, sizeof *s);
    if (!s) return NULL;
    s->cfg = (int)(intptr_t)c; s->w = s->h = 1;
    for (; a && a[0] != EGL_NONE; a += 2) { if (a[0] == EGL_WIDTH) s->w = a[1]; if (a[0] == EGL_HEIGHT) s->h = a[1]; }
    E_OK();
    return s;
}
EGLBoolean EGLAPI eglDestroySurface(EGLDisplay d, EGLSurface s)
{
    CHECK_DPY(d, 0);
    if (!s) { g_egl_err = EGL_BAD_SURFACE; return 0; }
    if (s == g_cur_surf) g_cur_surf = NULL;
    free(s);
    E_OK();
    return 1;
}
EGLBoolean EGLAPI eglQuerySurface(EGLDisplay d, EGLSurface s, EGLint a, EGLint *v)
{
    egl_surf *x = (egl_surf *)s;
    CHECK_DPY(d, 0);
    if (!x) { g_egl_err = EGL_BAD_SURFACE; return 0; }
    if (x->hwnd) win_size(x->hwnd, &x->w, &x->h);
    switch (a) {
    case EGL_WIDTH: *v = x->w; break;
    case EGL_HEIGHT: *v = x->h; break;
    case EGL_CONFIG_ID: *v = x->cfg; break;
    case EGL_RENDER_BUFFER: *v = EGL_BACK_BUFFER; break;
    case EGL_SWAP_BEHAVIOR: *v = EGL_BUFFER_DESTROYED; break;
    default: *v = 0; break;
    }
    E_OK();
    return 1;
}
EGLBoolean EGLAPI eglBindAPI(EGLenum a) { if (a != EGL_OPENGL_ES_API) { g_egl_err = EGL_BAD_PARAMETER; return 0; } E_OK(); return 1; }
EGLenum EGLAPI eglQueryAPI(void) { return EGL_OPENGL_ES_API; }
EGLContext EGLAPI eglCreateContext(EGLDisplay d, EGLConfig c, EGLContext sh, const EGLint *a)
{
    size_t n = (size_t)(&S.ctx_end - &S.ctx_begin);
    int ver = 1;
    egl_ctx *x;
    char *keep;
    (void)sh;                                                   /* objects are shared by all contexts here */
    CHECK_DPY(d, NULL);
    if (!cfg_ok(c)) { g_egl_err = EGL_BAD_CONFIG; return NULL; }
    for (; a && a[0] != EGL_NONE; a += 2) {
        if (a[0] == EGL_CONTEXT_CLIENT_VERSION) ver = a[1];
        else { g_egl_err = EGL_BAD_ATTRIBUTE; return NULL; }
    }
    if (ver != 2) { g_egl_err = EGL_BAD_MATCH; return NULL; }   /* OpenGL ES 2.0 only */
    x = (egl_ctx *)calloc(1, sizeof *x);
    keep = (char *)malloc(n);
    if (x) x->saved = (char *)malloc(n);
    if (!x || !keep || !x->saved) { free(keep); if (x) free(x->saved); free(x); return NULL; }
    x->cfg = (int)(intptr_t)c;
    memcpy(keep, &S.ctx_begin, n);                              /* the context's initial state, built in place */
    init_state();
    memcpy(x->saved, &S.ctx_begin, n);
    memcpy(&S.ctx_begin, keep, n);
    free(keep);
    E_OK();
    return x;
}
EGLBoolean EGLAPI eglDestroyContext(EGLDisplay d, EGLContext c)
{
    egl_ctx *x = (egl_ctx *)c;
    CHECK_DPY(d, 0);
    if (!x) { g_egl_err = EGL_BAD_CONTEXT; return 0; }
    if (x == g_cur_ctx) g_cur_ctx = NULL;
    free(x->saved);
    free(x);
    E_OK();
    return 1;
}
EGLBoolean EGLAPI eglMakeCurrent(EGLDisplay d, EGLSurface ds, EGLSurface rs, EGLContext c)
{
    size_t n = (size_t)(&S.ctx_end - &S.ctx_begin);
    egl_surf *x = (egl_surf *)ds;
    egl_ctx *cx = (egl_ctx *)c;
    CHECK_DPY(d, 0);
    if (ds != rs) { g_egl_err = EGL_BAD_MATCH; return 0; }      /* one framebuffer for drawing and reading */
    if (cx != g_cur_ctx) {
        if (S.r) g2r_end_pass(S.r);
        if (g_cur_ctx) memcpy(g_cur_ctx->saved, &S.ctx_begin, n);
        if (cx) memcpy(&S.ctx_begin, cx->saved, n);
        g_cur_ctx = cx;
    }
    g_cur_surf = x;
    if (!x || !cx) { S.live = 0; E_OK(); return 1; }
    if (dev_up() < 0) { g_egl_err = EGL_NOT_INITIALIZED; return 0; }
    if (x->hwnd) win_size(x->hwnd, &x->w, &x->h);
    default_fb(x->w, x->h, x->cfg, x->hwnd != NULL);
    if (!S.vp_set) { S.vp[2] = S.vw; S.vp[3] = S.vh; S.sc[2] = S.vw; S.sc[3] = S.vh; S.vp_set = 1; }     /* §2.12: the first time, the surface's size */
    S.live = 1;
    S.hwnd = x->hwnd;
    E_OK();
    return 1;
}
EGLContext EGLAPI eglGetCurrentContext(void) { return g_cur_ctx; }
EGLSurface EGLAPI eglGetCurrentSurface(EGLint r) { (void)r; return g_cur_surf; }
EGLDisplay EGLAPI eglGetCurrentDisplay(void) { return g_cur_ctx ? DISPLAY : NULL; }
EGLBoolean EGLAPI eglSwapBuffers(EGLDisplay d, EGLSurface s)
{
    CHECK_DPY(d, 0);
    if (s != g_cur_surf || !s) { g_egl_err = EGL_BAD_SURFACE; return 0; }
    if (S.live && ((egl_surf *)s)->hwnd) swap_buffers();        /* EGL 1.4 §3.9.3: on a pbuffer it has no effect, its colour buffer stays as it is */
    E_OK();
    return 1;
}
EGLBoolean EGLAPI eglSwapInterval(EGLDisplay d, EGLint i) { CHECK_DPY(d, 0); S.swap_interval = i > 0; E_OK(); return 1; }
EGLBoolean EGLAPI eglCopyBuffers(EGLDisplay d, EGLSurface s, void *t) { (void)d; (void)s; (void)t; g_egl_err = EGL_BAD_MATCH; return 0; }
EGLBoolean EGLAPI eglWaitGL(void) { if (S.live) g2r_finish(S.r); return 1; }
EGLBoolean EGLAPI eglWaitNative(EGLint e) { (void)e; return 1; }
EGLBoolean EGLAPI eglWaitClient(void) { return eglWaitGL(); }
EGLBoolean EGLAPI eglReleaseThread(void) { E_OK(); return 1; }
EGLBoolean EGLAPI eglSurfaceAttrib(EGLDisplay d, EGLSurface s, EGLint a, EGLint v) { (void)d; (void)s; (void)a; (void)v; E_OK(); return 1; }
EGLBoolean EGLAPI eglBindTexImage(EGLDisplay d, EGLSurface s, EGLint b) { (void)d; (void)s; (void)b; g_egl_err = EGL_BAD_MATCH; return 0; }   /* no config has BIND_TO_TEXTURE */
EGLBoolean EGLAPI eglReleaseTexImage(EGLDisplay d, EGLSurface s, EGLint b) { (void)d; (void)s; (void)b; g_egl_err = EGL_BAD_MATCH; return 0; }
EGLSurface EGLAPI eglCreatePixmapSurface(EGLDisplay d, EGLConfig c, void *p, const EGLint *a) { (void)d; (void)c; (void)p; (void)a; g_egl_err = EGL_BAD_MATCH; return NULL; }
EGLSurface EGLAPI eglCreatePbufferFromClientBuffer(EGLDisplay d, EGLenum t, void *b, EGLConfig c, const EGLint *a) { (void)d; (void)t; (void)b; (void)c; (void)a; g_egl_err = EGL_BAD_PARAMETER; return NULL; }
EGLBoolean EGLAPI eglQueryContext(EGLDisplay d, EGLContext c, EGLint a, EGLint *v)
{
    egl_ctx *x = (egl_ctx *)c;
    CHECK_DPY(d, 0);
    if (!x) { g_egl_err = EGL_BAD_CONTEXT; return 0; }
    switch (a) {
    case EGL_CONFIG_ID: *v = x->cfg; break;
    case EGL_CONTEXT_CLIENT_TYPE: *v = EGL_OPENGL_ES_API; break;
    case EGL_CONTEXT_CLIENT_VERSION: *v = 2; break;
    case EGL_RENDER_BUFFER: *v = EGL_BACK_BUFFER; break;
    default: g_egl_err = EGL_BAD_ATTRIBUTE; return 0;
    }
    E_OK();
    return 1;
}

static HMODULE g_module;
void *EGLAPI eglGetProcAddress(const char *name) { return (void *)GetProcAddress(g_module, name); }

BOOL WINAPI DllMain(HINSTANCE h, DWORD why, LPVOID r)
{
    (void)r;
    if (why == DLL_PROCESS_ATTACH) {
        g_module = h;
        DisableThreadLibraryCalls(h);
        init_state();
        S.swap_interval = 1;
        SetProcessDPIAware();                                   /* before the game's first window (gl_dev.c says why) */
    }
    return TRUE;
}
