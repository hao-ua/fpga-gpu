/* d3d9.dll — Direct3D 9 on the two-board GPU.
 *
 * Coordinates: rendering always maps window row 0 to memory row 0, and `origin` only says how an image is scanned out. So every image here has origin 0 and row 0 at the top, as Direct3D has it: viewport and scissor
 * are memory rows, texture v = 0 is the first row, and a render target samples upright. The vertex stage ends with the same transform in every program (d3d_vs_tail): half a pixel (Direct3D 9 samples at integer
 * window coordinates), y flipped, z from 0 .. w to -w .. w.
 * Winding: the hardware calls area > 0 front in (x, row) coordinates, which is what the eye calls clockwise on a y-down screen, Direct3D's default front.
 *
 * Programs are GLSL ES text compiled by host/win/shc: fixed function from a state key (d3d9_ff.c), shaders from their bytecode (d3d9_sm.c).
 * Vertex and index buffers live on the PC and go through g2_render's ring per draw; a texture is shadow pixels in the application's format, uploaded when first used after a change.
 */
#ifndef D3D9_INTERNAL_H
#define D3D9_INTERNAL_H
#define _WIN32_WINNT 0x0601
#define WIN32_LEAN_AND_MEAN
#define CINTERFACE
#include <windows.h>
#include <objbase.h>
#include <d3d9.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "g2_dxt.h"
#include "g2_env.h"
#include "g2_render.h"
#include "g2_shc.h"

#define MAX_RT_W 1280
#define MAX_SCAN_H 800
#define MAX_TEX_SIZE 2048
#define MAX_STAGES 8
#define MAX_SAMPLERS 16
#define MAX_STREAMS 16
#define MAX_LIGHTS 8
#define MAX_CLIP 4
#define VS_CONSTS 256
#define PS_CONSTS 224
#define MAX_RS 256
#define MAX_TSS 33
#define MAX_SS 14
#define MAX_WM 28               /* world matrices of indexed vertex blending: each costs 7 of the 240 uniforms, a mat4 and a mat3 */
#define MAX_POINT 64.0f

typedef struct d3dev d3dev;
typedef struct d3tex d3tex;

/* how every object the device can bind starts (a surface, texture, buffer, declaration or shader): refs counts all its references, binds
 * the library's own among them (a binding, a state block's, the recorder's, the device's back buffer and depth surface). Release counts the
 * application's alone, refs - binds, and one with none of those left changes nothing and says 0, as Direct3D's runtime does: a game's
 * renderer may take the back buffer and the depth surface and release them at once, then before Reset release them and its bound vertex
 * buffer again until Release says 0 */
typedef struct d3ref { const void *vtbl; LONG refs, binds; } d3ref;

/* a surface: a level of a texture, a render target, a depth-stencil surface, or plain memory */
typedef struct d3surf {
    const IDirect3DSurface9Vtbl *vtbl;
    LONG refs, binds;           /* d3ref; a texture's level has its texture's */
    d3dev *dev;
    d3tex *tex;                 /* a level of this texture: references go to it */
    int level, face;
    UINT w, h;
    UINT vw, vh;                /* an additional swap chain's back buffer beyond the hardware: what the application asked for, it renders w x h. 0: the same */
    D3DFORMAT fmt;
    DWORD usage;
    D3DPOOL pool;
    uint8_t *mem;               /* the shadow, in fmt */
    int pitch;
    int dirty;                  /* mem is newer than the board */
    uint16_t handle;            /* a render target's or depth-stencil's own image */
    uint32_t base, g2fmt;
    uint16_t part;              /* a depth-stencil larger than its target: B needs them equal, so draws use this image of the target's size and the rectangle moves between the two */
    uint32_t part_base;
    UINT part_w, part_h;
    int part_live;              /* part holds what is newer than the top-left of handle */
    int back;                   /* the swap chain's back buffer: handle follows the frame */
    int locked;
    RECT lock;
    DWORD lock_flags;
} d3surf;

/* a level of a volume texture: IDirect3DVolume9. mem holds its slices, the references are the texture's */
typedef struct d3vol {
    const void *vtbl;
    struct d3tex *tex;
    int level;
    UINT w, h, d;
    int pitch, slice;
    uint8_t *mem;
    int dirty, locked;
    DWORD lock_flags;
} d3vol;

/* a texture: 2D, cube or volume (a volume needs a board B with 3D textures). handle has tight rows (the TMU's descriptor has no pitch); a target that cannot render there renders into rt */
struct d3tex {
    const void *vtbl;
    LONG refs, binds;           /* d3ref */
    d3dev *dev;
    int cube, vol, layers, levels, hwl;     /* hwl: the levels on the board: all of them under D3DUSAGE_AUTOGENMIPMAP, where the application sees one */
    int autogen, gen_dirty;     /* the top level changed, the chain lags */
    int depth;                  /* a depth texture: 1 reads as its value (INTZ, DF24, DF16), 2 as the comparison with the coordinate's z (D24S8, D24X8, D16) */
    UINT w, h, d;               /* the image on the board; d: a volume's depth */
    UINT vw, vh;                /* what the application asked for: a render target beyond the hardware renders smaller, sampling is normalised anyway */
    D3DFORMAT fmt;
    DWORD usage;
    D3DPOOL pool;
    d3surf **surf;              /* [layers * levels], a volume has none */
    d3vol **lv;                 /* a volume's levels */
    uint16_t handle;
    uint32_t base, g2fmt;
    uint16_t rt;
    uint32_t rt_base;
    int rt_face, rt_level, rt_w, rt_h, rt_dirty, rt_stale;
    uint16_t rt_ds, ds_alias;   /* a depth texture as a depth-stencil target: rt's memory, or its own */
    uint32_t used_pass;
    DWORD lod, autogen_filter;
    uint16_t view;              /* the image the TMU is given when the largest level in use is not level 0 */
    uint32_t view_base;
    int view_b, view_copy;
    uint32_t rev, view_rev;     /* writes to the image, what a cube map's copy was made from */
};

typedef struct d3buf {
    const void *vtbl;           /* IDirect3DVertexBuffer9 or IDirect3DIndexBuffer9 */
    LONG refs, binds;           /* d3ref */
    d3dev *dev;
    uint8_t *mem;
    UINT size;
    DWORD usage;
    DWORD fvf;
    D3DFORMAT fmt;
    D3DPOOL pool;
    g2r_kept keep;
} d3buf;

typedef struct d3decl {
    const IDirect3DVertexDeclaration9Vtbl *vtbl;
    LONG refs, binds;           /* d3ref */
    d3dev *dev;
    D3DVERTEXELEMENT9 el[65];
    int n;                      /* without the end marker */
    DWORD fvf;                  /* the FVF this was built for, 0: none */
    DWORD shader8;              /* d3d8.dll: the vertex shader handle this declaration came with */
} d3decl;

/* a shader's parsed interface, filled by d3d9_sm.c */
typedef struct d3sh_io { int usage, index; char name[24]; } d3sh_io;
/* c[a0 + base] with a0 loaded from c[reg].comp: one value per draw, which d3_prog_uniforms works out and hands over as a uniform */
#define MAX_REL 16
typedef struct d3rel { short reg, comp, base, round; } d3rel;
/* a0 from a vertex: the constants as one array, read with LDPX (board A's indexed uniform load). Not the whole 256: the uniform space also holds the shader's literals and the fixed uniforms */
#define VCA_N 240
typedef struct d3shader {
    const void *vtbl;           /* IDirect3DVertexShader9 or IDirect3DPixelShader9 */
    LONG refs, binds;           /* d3ref */
    d3dev *dev;
    uint32_t id;                /* never reused: the program cache's key, an address could come back */
    DWORD *tok;
    UINT ntok;                  /* tokens */
    int ps, major, minor;
    char *body;                 /* GLSL for main()'s body + the declarations */
    char *decl;
    char error[200];
    uint32_t used_c[8], used_i, used_b;         /* bit per constant register */
    d3sh_io in[32], out[32];
    int nin, nout;
    d3rel rel[MAX_REL];
    int nrel;
    float *defc;                /* [VS_CONSTS][4]: what def put in the registers, a relative read can land there */
    uint32_t def_mask[8];
    uint32_t loop_i;            /* the integer registers rep and loop read: their values are part of the program */
    uint8_t samp_kind[8];       /* 0 unused, 1 what the stage holds (ps 1.x declares no samplers), 2 2D, 3 cube, 4 volume */
    uint8_t tcomps[8];          /* the components of a texture coordinate that matter, 0: not known */
    int pos_zk, pos_wk;         /* a vertex shader whose position has z = dot(X, c[pos_zk]), w = dot(X, c[pos_wk]) of one X: its body calls G2_ZR(X). -1: no such form (d3d9_sm.c pos_scan) */
} d3shader;

typedef struct d3query { const IDirect3DQuery9Vtbl *vtbl; LONG refs; d3dev *dev; D3DQUERYTYPE type; int issued; LARGE_INTEGER stamp; } d3query;

/* everything a state block can hold */
typedef struct d3state {
    DWORD rs[MAX_RS];
    DWORD tss[MAX_STAGES][MAX_TSS];
    DWORD ss[MAX_SAMPLERS + 5][MAX_SS];
    d3tex *tex[MAX_SAMPLERS + 5];
    d3buf *vb[MAX_STREAMS];
    UINT vb_off[MAX_STREAMS], vb_stride[MAX_STREAMS], vb_freq[MAX_STREAMS];
    d3buf *ib;
    int ib_base;                /* d3d8.dll: Direct3D 8 gives the base vertex with the index buffer */
    d3decl *decl;
    DWORD fvf;
    d3shader *vs, *ps;
    float vc[VS_CONSTS][4], pc[PS_CONSTS][4];
    int vi[16][4], pi[16][4];
    BOOL vb_c[16], pb_c[16];
    D3DMATRIX world, view, proj, texm[MAX_STAGES], wm[MAX_WM];  /* wm[1..]: D3DTS_WORLD1 .. of vertex blending, world is the first */
    D3DMATERIAL9 mat;
    D3DLIGHT9 light[MAX_LIGHTS];
    BOOL light_on[MAX_LIGHTS];
    float clip[6][4];
    D3DVIEWPORT9 vp;
    RECT sc;
    int vp_set;
} d3state;

/* which parts of a d3state a state block carries */
typedef struct d3mask {
    uint32_t rs[MAX_RS / 32], tss[MAX_STAGES], ss[MAX_SAMPLERS + 5];
    uint32_t tex, vb;
    int ib, decl, fvf, vs, ps, xf, mat, light, clip, vp, sc, vc, pc;
} d3mask;

typedef struct d3block { const IDirect3DStateBlock9Vtbl *vtbl; LONG refs; d3dev *dev; d3state s; d3mask m; } d3block;

/* a linked program: fixed function or shaders, the slots of what the draw must fill in */
enum { U_WVP, U_WV, U_PROJ, U_NORM, U_VP, U_RT, U_GLOBAL_AMB, U_MAT_D, U_MAT_A, U_MAT_S, U_MAT_E, U_MAT_P, U_FOG, U_PSIZE,
       U_TF, U_ACOLOR, U_FOGCOL, U_AREF, U_PSCALE, U_TWEEN, U_L0 = 20, U_L_END = U_L0 + 8 * MAX_LIGHTS, U_TM0 = U_L_END, U_CLIP0 = U_TM0 + MAX_STAGES, U_WM = U_CLIP0 + MAX_CLIP, U_NM, U_TK0, U_COUNT = U_TK0 + MAX_STAGES };
#define L_POS 0
#define L_DIR 1
#define L_DIFF 2
#define L_SPEC 3
#define L_AMB 4
#define L_ATT 5
#define L_SPOT 6

typedef struct d3attr { int usage, index, idx, swap_rb, first; } d3attr;    /* first: of the primitive's first vertex, D3DSHADE_FLAT */
typedef struct d3prog {
    struct d3prog *next;
    uint64_t hash;
    void *key;
    size_t key_size;
    g2r_prog *p;
    d3attr attr[16];
    int nattr;
    short vu[U_COUNT], fu[U_COUNT];             /* slots, -1: this stage does not use it */
    short vc_slot[VS_CONSTS], pc_slot[PS_CONSTS];   /* shaders: a constant register's slot, -1: unused */
    short vi_slot[16], pi_slot[16], vb_slot[16], pb_slot[16], vr_slot[MAX_REL];
    short pm_slot[MAX_STAGES], pl_slot[MAX_STAGES], pz_slot[MAX_STAGES];
    short lb_slot[8], bc_slot[8], ts_slot[8];   /* a sampler's LOD bias, border colour, size */
    short vca_slot;                                             /* the constants as an array, -1: this program reads them one by one */
    short zr_slot;                                              /* u_zr: c[pos_wk] - c[pos_zk] of a vertex shader under reversed depth, -1: none */
    int zrev;                                                   /* built for a target whose depth is kept reversed (d3_zrev) */
    int vbase, fbase;
    int flat;                                                   /* its attributes f_u* come from each primitive's first vertex */
    int wm_n;                                                   /* world matrices it reads */
    int nsamp;
    int stage_of[8];                            /* hardware sampler -> D3D sampler */
    int cube_of[8];
} d3prog;

struct d3dev {
    const IDirect3DDevice9Vtbl *vtbl;
    LONG refs;
    IDirect3D9 *d3d;
    D3DPRESENT_PARAMETERS pp;
    HWND hwnd;
    DWORD flags;
    g2_board *bd;
    g2r *r;
    /* the swap chain */
    int mode, mode_w, mode_h, hold, stretch, back_i, scan_i;
    uint32_t frame;
    uint16_t color[3], scan[3];
    uint32_t color_base[3], scan_base[3];
    d3surf *backbuf, *auto_ds;
    void *swapchain;
    int vw, vh;                                 /* what the application calls the back buffer's size */
    /* targets */
    d3surf *rt, *ds;
    d3state st;
    d3block *rec;                               /* BeginStateBlock .. EndStateBlock */
    uint32_t vu[4 * 256], fu[4 * 64];
    d3prog *progs[1024];
    d3decl *fvf_decl[64];
    void *up_mem;
    size_t up_size;
    int in_scene;
    int zrev_ok;                                /* board A clips at z >= 0 on request (G2_A_ZCLIP0) and GPU2_REVZ is not 0: depth can be kept reversed */
    int cursor_on, cursor_have, cursor_new, cursor_app;         /* shown, a shape was set, a shape B has not taken yet, the application places it itself */
    int cursor_hx, cursor_hy, cursor_x, cursor_y;
    int mirror_on, mirror_mode, mirror_hx, mirror_hy;           /* the Windows pointer on B's cursor */
    HCURSOR mirror_cur;
    HANDLE cursor_thread;
    volatile LONG cursor_track, cursor_stop;                    /* the thread moves it */
};

/* GPU2_TRACE_D3D=1: every creation call, and why a draw did not happen, in the log */
int d3_trace(void);
#define D3T(...) do { if (d3_trace()) g2e_log(__VA_ARGS__); } while (0)

/* d3d9_res.c */
extern const IDirect3DSurface9Vtbl d3_surface_vtbl;
extern const IDirect3DTexture9Vtbl d3_texture_vtbl;
extern const IDirect3DCubeTexture9Vtbl d3_cube_vtbl;
extern const IDirect3DVolume9Vtbl d3_volume_vtbl;
extern const IDirect3DVolumeTexture9Vtbl d3_voltex_vtbl;
extern const IDirect3DVertexBuffer9Vtbl d3_vb_vtbl;
extern const IDirect3DIndexBuffer9Vtbl d3_ib_vtbl;
extern const IDirect3DVertexDeclaration9Vtbl d3_decl_vtbl;
extern const IDirect3DVertexShader9Vtbl d3_vs_vtbl;
extern const IDirect3DPixelShader9Vtbl d3_ps_vtbl;
extern const IDirect3DQuery9Vtbl d3_query_vtbl;
extern const IDirect3DStateBlock9Vtbl d3_block_vtbl;
extern const IDirect3DDevice9Vtbl d3_dev_vtbl;
extern const IDirect3D9Vtbl d3_d3d_vtbl;
extern const IDirect3DSwapChain9Vtbl d3_chain_vtbl;
extern const void *d3_vt_surface, *d3_vt_texture, *d3_vt_cube, *d3_vt_volume, *d3_vt_voltex, *d3_vt_vb, *d3_vt_ib, *d3_vt_device, *d3_vt_chain;      /* what new objects get: d3d8.dll is these objects behind Direct3D 8's tables */
int d3_fmt_bytes(D3DFORMAT f);                  /* per pixel; 0: compressed */
int d3_fmt_g2(D3DFORMAT f, uint32_t *g2fmt);    /* 0: not a format the library takes */
int d3_fmt_rt(D3DFORMAT f);
int d3_fmt_ds(D3DFORMAT f);
int d3_fmt_signed(D3DFORMAT f);                 /* 1: red and green are signed, 2: all four are. The TMU has no signed format: they are stored biased, a program undoes it */
int d3_fmt_srgb(D3DFORMAT f);                   /* an 8-bit colour format a sampler can read as sRGB, and a target can be written as sRGB */
int d3_tex_base(const d3tex *x, const DWORD *ss);       /* the largest level in use: D3DSAMP_MAXMIPLEVEL, SetLOD */
uint16_t d3_tex_image(d3tex *x, const DWORD *ss);       /* what the sampler is given */
d3surf *d3_surface_new(d3dev *d, UINT w, UINT h, D3DFORMAT f, DWORD usage, D3DPOOL pool, int shadow);
HRESULT d3_texture_new(d3dev *d, UINT w, UINT h, UINT levels, DWORD usage, D3DFORMAT f, D3DPOOL pool, int cube, d3tex **out);
void d3_tex_prepare(d3tex *t);                  /* uploads what changed */
void d3_tex_ready(d3tex *t);                    /* + what was rendered, + the chain D3DUSAGE_AUTOGENMIPMAP keeps: it can be sampled */
uint16_t d3_tex_ds(d3tex *t);                   /* a depth texture as a D24S8 target */
HRESULT d3_front_read(d3dev *d, d3surf *o);     /* what the screen shows */
int d3_fmt_dtex(D3DFORMAT f);
void d3_tex_resolve(d3tex *t);                  /* what was rendered reaches the sampled image */
uint16_t d3_tex_target(d3tex *t, int face, int level, UINT w, UINT h);  /* the image to render in */
void d3_surf_read(d3surf *s);                   /* the board's pixels into the shadow */
void d3_surf_write(d3surf *s);                  /* a target's shadow onto the board */
HRESULT d3_block_new(d3dev *d, int all, d3block **out);
void d3_block_capture(d3block *b);
void d3_block_apply(d3block *b);
int d3_shadow_bytes(const d3surf *s);
HRESULT d3_volume_new(d3dev *d, UINT w, UINT h, UINT de, UINT levels, DWORD usage, D3DFORMAT f, D3DPOOL pool, d3tex **out);
void d3_ref_set(void *slot, void *v);            /* a bound object: the device holds a reference of its own (d3_bind), as Direct3D's does */
void d3_bind(void *o);                          /* a d3ref's reference of the library's own: the application's Release does not count it */
void d3_unbind(void *o);                        /* gives it back: the object goes when nothing else holds it */
LONG d3_unref(void *o);                         /* a Release's step: the references left, -1 when none was the application's (nothing changes) */

/* d3d9_cursor.c */
int d3_cursor_words(HCURSOR c, uint32_t *w, int *mode, int *hx, int *hy);

/* d3d9_rec.c: a flight recorder, see there */
int d3_rec_on(void);
void d3_rec_draw(d3dev *d, int mode, const uint32_t *idx, uint32_t n, uint32_t first, const uint8_t *up, UINT up_stride);
void d3_rec_clear(d3dev *d, DWORD n, const D3DRECT *rects, DWORD flags, D3DCOLOR c, float z, DWORD st);
void d3_rec_present(d3dev *d);

/* d3d9_dev.c */
/* Depth kept reversed in the depth surface that is bound: far at 0, near at 1. A far vertex's depth is just under 1 the way Direct3D counts, and every stage that carries it on the
   boards is FP24, 17 bits: it resolves 2^-17 there, so a wall 10 m behind another at 100 m would show through. Counted from the far end the same float is as fine as the depth is small. The application
   never sees it: the comparison, the clear value, the bias and the viewport's range are mirrored, and the vertex stage hands board A z' = w - z. A depth texture and a lockable surface keep
   Direct3D's own count, since their values are read */
int d3_zrev(const d3dev *d);
HRESULT d3_device_new(IDirect3D9 *d3d, HWND focus, DWORD flags, D3DPRESENT_PARAMETERS *pp, IDirect3DDevice9 **out);
int d3_has_3d(void);                                            /* board B has 3D textures: volume textures exist (the caps, CheckDeviceFormat, CreateVolumeTexture) */
void d3_caps(D3DCAPS9 *c);

/* d3d9_ff.c */
d3prog *d3_ff_program(d3dev *d, const d3decl *decl, d3shader *vs, d3shader *ps, int mode);
void d3_prog_uniforms(d3dev *d, d3prog *p, int w, int h, double ky);
int d3_invert3(const D3DMATRIX *a, float o[3][3]);              /* < 0: singular. What takes normals into eye space: v * m's inverse transpose */

/* d3d9_sm.c */
int d3_sm_parse(d3shader *s);                   /* < 0: s->error says why */
/* the same bytecode run on the PC, in by input register. < 0: an instruction a vertex shader cannot have */
typedef struct d3vsout { float pos[4], col[2][4], tex[12][4], fog[4], psize[4]; int wrote_fog, wrote_psize; } d3vsout;
int d3_vs_cpu(const d3state *st, const d3shader *s, const float (*in)[4], d3vsout *o);

/* d3d9_cpu.c: vertex processing on the PC */
void d3_elem_read(const uint8_t *p, BYTE type, float *o);       /* an element as four floats, bytes in memory order: a D3DCOLOR is b, g, r, a */
const uint8_t *d3_stream_at(const d3dev *d, const D3DVERTEXELEMENT9 *e, const uint8_t *up, UINT up_stride, uint32_t v, int inst);    /* the element of vertex v, NULL: no such stream */
/* where vertex v lands in clip space, y up: fixed function or the vertex shader, run here. A pretransformed vertex comes back as (x, -y, 0, 1) */
int d3_cpu_position(d3dev *d, const uint8_t *up, UINT up_stride, uint32_t v, int inst, float *o);
HRESULT d3_process_vertices(d3dev *d, UINT src, UINT dst, UINT n, d3buf *out, const d3decl *de, DWORD flags);

/* a small growing string */
typedef struct d3sb { char *p; size_t n, cap; } d3sb;
void d3sb_printf(d3sb *b, const char *fmt, ...);
#endif
