/* opengl32.dll — desktop OpenGL on the two-board GPU: the context, shared by the files of this library.
 *
 * The GPU has no fixed function: every state combination that matters becomes a key, a key a generated GLSL ES program pair (gl_ff.c), compiled at
 * run time by the C port of glslc (host/win/shc) and cached. Geometry reaches board A through the streaming ring of g2_render; immediate mode is
 * collected into one batch until a state change (FLUSH_VERTICES at the top of every state-changing call).
 */
#ifndef GL_INTERNAL_H
#define GL_INTERNAL_H
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#define WIN32_LEAN_AND_MEAN
#define _GDI32_                 /* wingdi.h / gl.h declare the entry points without dllimport: this library defines them */
#define GL_GLEXT_PROTOTYPES     /* the extension entry points need their real prototypes: on x86 the argument size is part of the stdcall name */
#include <windows.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "g2_env.h"
#include "g2_render.h"
#include "g2_shc.h"

#define MAX_UNITS 8
#define MAX_LIGHTS 8
#define MAX_CLIP 6
#define MS_DEPTH 32
#define MAX_LEVELS 12
#define MAX_TEX_SIZE 2048
#define MAX_3D_SIZE 128
#define MAX_ATTRIBS 16
#define IM_VERTS_MAX 65000

typedef float vec4f[4];

typedef struct gl_light {
    int enabled;
    float ambient[4], diffuse[4], specular[4], position[4], spot_dir[3];
    float spot_exp, spot_cut, atten[3];
} gl_light;

typedef struct gl_material { float ambient[4], diffuse[4], specular[4], emission[4], shininess; } gl_material;

/* what a base internal format does in the texture environment (table 3.11): LUMINANCE behaves as RGB and LUMINANCE_ALPHA as RGBA once expanded to RGBA8 */
enum { BF_RGB, BF_RGBA, BF_ALPHA, BF_INTENSITY, BF_DEPTH };

typedef struct gl_level { int defined, w, h, d, border; } gl_level;   /* w, h without the border; d: a 3D texture's slices */

typedef struct gl_tex {
    GLuint name;
    GLenum target;              /* 0 until first bound */
    int w, h, levels, layers;   /* the image's chain, 0 = no image yet */
    uint16_t handle;
    uint32_t base;
    int base_class;
    GLenum internal;
    uint32_t hwfmt;             /* the storage format of every image of the texture: G2_FMT_RGBA8 (0), RGB565, RGBA4444, RGBA5551 (gl_hw_format) */
    gl_level level[6][MAX_LEVELS];
    GLenum min_filter, mag_filter, wrap_s, wrap_t, wrap_r;
    GLint max_level;
    int gen_mipmap;
    uint8_t *level0;            /* RGBA8 of level 0, kept only with GL_GENERATE_MIPMAP */
    float priority;
    float border_color[4];
    GLint base_level;
    float min_lod, max_lod, lod_bias;
    void *comp[6][MAX_LEVELS];  /* glCompressedTexImage's bytes, which the board does not hold: glGetCompressedTexImage */
    int comp_size[6][MAX_LEVELS];
    uint16_t view;              /* the image the TMU is given when BASE_LEVEL or MAX_LEVEL cut the chain */
    uint32_t view_base;
    int view_b, view_n, view_copy;
    uint32_t rev, view_rev;     /* writes to the image, what the copy of a cube chain was made from */
    GLenum compare_mode, compare_func, depth_mode;      /* depth textures: GL_ARB_shadow, GL_ARB_depth_texture */
    uint16_t ds_alias;          /* the same memory as a depth-stencil target */
    uint16_t rt, rt_ds;         /* what a framebuffer renders into when it cannot be the texture: B renders level 0 of one layer, at a pitch that is a multiple of 32. rt_ds: its memory as a depth-stencil target */
    uint32_t rt_base, rt_rev;   /* rt_rev: the rev that rt holds */
    int rt_w, rt_h, rt_level, rt_face, rt_dirty;    /* rt_face: a cube face or a slice. dirty: rendered, the texture lags */
    int w3, h3, d3, b3, aw3, ah3;   /* a 3D texture: its size, its border, the size of the image its slices lie in (gl_tex3d.c) */
    int vol3;                   /* the image is a volume the TMU reads, else the slices side by side */
    int lv3[MAX_LEVELS][3];     /* a level's first texel in the side by side image, its slices a row */
    uint8_t *px3[MAX_LEVELS];   /* the PC's copy, RGBA8, slice after slice */
    uint16_t bimg;              /* level 0 with its border, one level: the mip chain of the TMU halves the image, which a border does not do */
    uint32_t bimg_base;
    int bimg_w, bimg_h;
    float ring[4];              /* the mean of the border texels of level 0: what a mipmapped texture has of its border */
} gl_tex;

typedef struct gl_renderbuffer {
    GLuint name;
    GLenum internal;
    uint32_t hwfmt;             /* a colour buffer's storage format */
    int depth16;                /* DEPTH_COMPONENT16: B's depth of 16 bits */
    int w, h;
    uint16_t handle;
    uint32_t base;
} gl_renderbuffer;

typedef struct gl_fbo {
    GLuint name;
    GLenum color_type, depth_type, stencil_type;        /* 0, GL_TEXTURE, GL_RENDERBUFFER_EXT */
    GLuint color, depth, stencil;
    int color_level, color_face, color_z, depth_level;
    uint16_t blind;             /* the colour target of a framebuffer that has depth alone */
    uint32_t blind_base;
    int blind_w, blind_h;
    uint16_t ds_handle;         /* our D24S8 when the depth attachment is a texture or none matched */
    uint32_t ds_base;
    int ds_w, ds_h;
} gl_fbo;

typedef struct gl_buffer {
    GLuint name;
    uint8_t *data;
    GLsizeiptr size;
    GLenum usage;
    int mapped;
    g2r_kept keep;
} gl_buffer;

typedef struct gl_unit {
    int en_1d, en_2d, en_cube, en_3d;
    GLuint bound_1d, bound_2d, bound_cube, bound_3d;
    GLenum env_mode;
    float env_color[4], lod_bias;
    int coord_replace;
    GLenum comb_rgb, comb_a;
    GLenum src_rgb[3], src_a[3], op_rgb[3], op_a[3];
    float scale_rgb, scale_a;
    int gen_en[4];              /* S T R Q */
    GLenum gen_mode[4];
    float gen_obj[4][4], gen_eye[4][4];
    float tc[4];                /* current texture coordinate */
    float mat[MS_DEPTH][16];
    int mat_sp;
} gl_unit;

typedef struct gl_array {
    int enabled, size;
    GLenum type;
    GLsizei stride;
    const void *ptr;
    GLuint buffer;
    int normalized;             /* a generic attribute's */
} gl_array;

/* attribute slots of the generated programs; aStip: how far along a stippled line, in pixels; aLine: an antialiased line's corner, in gl_draw.c. A_GEN: a user program's generic attributes 1..15: generic 0 is A_POS */
enum { A_POS, A_COLOR, A_NORMAL, A_TC0, A_STIP = A_TC0 + MAX_UNITS, A_LINE, A_FOGC, A_SEC, A_GEN, A_FLAT = A_GEN + MAX_ATTRIBS - 1, A_FLAT_END = A_FLAT + 4, A_COUNT = A_FLAT_END };

/* an attribute of a draw */
typedef struct src {
    const uint8_t *base;        /* the element of vertex 0; NULL: a constant */
    int stride, bytes, size, type, normalized;
    uint32_t constant[4];
    uint32_t off;               /* in the ring, or the kept buffer: the element of the first gathered vertex */
    uint16_t handle;            /* which of the two (gather) */
    gl_buffer *from;            /* the buffer it reads as it is; NULL: client memory or a conversion */
    int conv;                   /* base is a copy made for this draw, not the application's array */
} src;

typedef struct gl_map { int order[2], comps; float d[4]; float *pts; } gl_map;     /* an evaluator: domain u1 u2 v1 v2, control points tightly packed */
typedef struct gl_pixmap { int n; float *v; } gl_pixmap;
#define MAX_EVAL_ORDER 30
#define MAX_PIXEL_MAP 256
#define MAX_NAME_DEPTH 64

/* a generated uniform: what it holds, and where the compiler put it */
typedef struct ff_uni { uint8_t kind, idx; uint16_t slot; } ff_uni;
#define FF_MAX_UNI 160
typedef struct gl_ureq { char name[24]; uint8_t kind, idx; } gl_ureq;

/* uniform kinds */
enum { U_MVP, U_MV, U_NM, U_TM, U_COLOR_SCENE, U_L_POS, U_L_AMB, U_L_DIF, U_L_SPC, U_L_DIR, U_L_ATT, U_L_SPOT, U_M_EMI, U_M_AMB, U_M_DIF, U_M_SPC, U_M_SHI,
       U_BM_EMI, U_BM_AMB, U_BM_DIF, U_BM_SPC, U_BM_SHI, U_GEN_S, U_GEN_T, U_GEN_R, U_GEN_Q, U_FOG, U_CLIP, U_POINT, U_Z, U_F_FOGC, U_F_ALPHA, U_F_ENV, U_F_STIP, U_F_LINE, U_F_POINT, U_F_TSZ, U_F_BRD, U_F_LOD, U_POINTA, U_F_SWL, U_F_SWS, U_F_SWA, U_F_SWD, U_F_RTC };

typedef struct ff_key {
    uint32_t light : 1, color_mat : 1, two_side : 1, local_viewer : 1, sep_spec : 1, normalize : 1, rescale : 1, fog : 2, fog_coord_eye : 1,
             alpha_fn : 3, clip_mask : 6, tex_units : 4, cm_mode : 3, points : 1, poly_stip : 1, line_stip : 1, logic : 2, pad : 1;
    uint8_t light_on, light_pos, light_spot, pad2;
    uint8_t smooth_pt, smooth_ln;   /* cover into alpha, §3.3.2, §3.4.4 */
    uint8_t back_only;              /* the back colour whatever the hardware says: a back polygon drawn as lines has no back */
    uint8_t zoff;                   /* polygon offset of a polygon drawn as lines or points, which the hardware does only for triangles */
    uint8_t mask0;                  /* unit 0 only says which fragments exist: glBitmap, depth pixels */
    uint8_t fogc, sec, pt_att;      /* fog from the fog coordinate; a secondary colour added without lighting; a point's size from its distance */
    uint8_t crep, sprite_low;       /* point sprites: the units whose coordinate is the place in the point, t up the sprite */
    uint8_t need_spec, fogfrag;     /* a user's fragment shader after a generated vertex program reads the secondary colour; gl_FogFragCoord: 1 the eye distance, 2 the fog coordinate */
    struct {
        uint8_t target;         /* 0 off, 1 2D, 2 cube, 3 1D, 4 3D */
        uint8_t base;           /* BF_* */
        uint8_t env;            /* 0 modulate, 1 decal, 2 replace, 3 blend, 4 add, 5 combine */
        uint8_t texmat, proj;
        uint8_t gen[4];         /* 0 none, 1 object, 2 eye, 3 sphere, 4 normal, 5 reflection */
        uint8_t comb_rgb, comb_a, scale_rgb, scale_a;
        uint8_t src_rgb[3], src_a[3], op_rgb[3], op_a[3];
        uint8_t bs, bt;         /* the hardware has no border colour: 0 none, 1 GL_CLAMP linear, 2 GL_CLAMP_TO_BORDER linear, 3 GL_CLAMP_TO_BORDER nearest */
        uint8_t blod;           /* the same with mipmaps: how wide a texel is depends on the level */
        uint8_t bimg, ws, wt;   /* sample gl_tex.bimg, the wrap modes done in the program: 0 repeat, 1 mirror, 2 clamp, 3 edge, 4 border */
        uint8_t lod;            /* 1 a LOD bias, 2 the level worked out here: GL_TEXTURE_MIN_LOD, GL_TEXTURE_MAX_LOD */
        uint8_t sw;             /* the filter is in the program (gen_sw): 1 a colour texture a user's program reads past what the TMU has, 2 a depth texture, 3 a 3D one side by side, 4 a volume */
        uint8_t swmag, swmin, swn, swb, sww[3];     /* linear; 0 nearest 1 linear, 2..5 the mipmap filters; levels in use; the image has its border; wrap of each axis, as ws, wt */
        uint8_t dmode, dcmp;    /* depth: 1 luminance, 2 intensity, 3 alpha; 1 + the compare function in GL_NEVER's order, 0 none */
    } u[MAX_UNITS];
} ff_key;

typedef struct ff_prog {
    ff_key key;
    g2r_prog *p;
    int attr_loc[A_COUNT];      /* -1: not used by this program */
    ff_uni vu[FF_MAX_UNI], fu[FF_MAX_UNI / 2];
    int nvu, nfu;
    int sampler[MAX_UNITS];     /* unit -> sampler slot */
    int stip_sampler;
    struct ff_prog *next;
    struct gs_variant *user;    /* a user's program; user_fs: its fragment shader, so sampler i is its i-th, wherever that points */
    int user_fs;
} ff_prog;

/* display list operations */
enum { LOP_BEGIN, LOP_END, LOP_VERTEX, LOP_COLOR, LOP_NORMAL, LOP_TEXCOORD, LOP_ENABLE, LOP_DISABLE, LOP_BINDTEX, LOP_MATRIXMODE, LOP_PUSHM, LOP_POPM, LOP_LOADI, LOP_LOADM, LOP_MULTM, LOP_TRANSLATE,
       LOP_SCALE, LOP_ROTATE, LOP_BLENDFUNC, LOP_DEPTHFUNC, LOP_DEPTHMASK, LOP_ALPHAFUNC, LOP_COLORMASK, LOP_CULLFACE, LOP_FRONTFACE, LOP_SHADEMODEL, LOP_LINEWIDTH, LOP_POINTSIZE, LOP_POLYGONMODE,
       LOP_POLYOFFSET, LOP_MATERIAL, LOP_LIGHT, LOP_TEXENVF, LOP_CALL, LOP_BITMAP, LOP_EDGEFLAG, LOP_FOGC, LOP_SEC, LOP_ATTRIB, LOP_USEPROG, LOP_UNIFORM };
typedef struct gl_listop { int kind; uint32_t a, b; float f[4]; void *big; int w, h; } gl_listop;
typedef struct gl_list { GLuint name; gl_listop *ops; int n, cap; } gl_list;

typedef struct gl_ctx {
    int live;
    GLenum error;

    /* matrices (column-major) */
    GLenum matrix_mode;
    float mv[MS_DEPTH][16], pr[MS_DEPTH][16];
    int mv_sp, pr_sp;

    /* current values */
    float color[4], normal[3], sec_color[4], fog_coord;
    float raster_pos[4];
    int raster_valid;
    float raster_color[4], raster_tc[4], raster_dist;
    int edge_flag;

    /* enables */
    int depth_test, cull_face, blend, scissor_test, alpha_test, stencil_test, fog, lighting, color_material, normalize, rescale_normal, dither, poly_offset_fill,
        poly_offset_line, poly_offset_point, clip_en[MAX_CLIP], color_logic_op, line_smooth, point_smooth, polygon_smooth, line_stipple, polygon_stipple, auto_normal;

    /* state */
    GLenum cull_mode, front_face, shade_model, depth_func, alpha_func, poly_mode_front, poly_mode_back;
    GLenum blend_src, blend_dst, blend_src_a, blend_dst_a, blend_eq, blend_eq_a;
    float blend_color[4], alpha_ref, clear_color[4], clear_depth, poly_factor, poly_units, line_width, point_size, depth_near, depth_far;
    int depth_mask, color_mask, clear_stencil;
    GLenum st_func[2], st_fail[2], st_zfail[2], st_zpass[2];
    int st_ref[2];
    GLuint st_vmask[2], st_wmask[2];
    int vp[4], sc[4];
    float clip_plane[MAX_CLIP][4];
    GLenum fog_mode;
    float fog_density, fog_start, fog_end, fog_color[4];
    gl_light light[MAX_LIGHTS];
    gl_material mat[2];
    float lm_ambient[4];
    int lm_two_side, lm_local_viewer;
    GLenum lm_color_control, cm_face, cm_mode;

    /* textures */
    gl_unit unit[MAX_UNITS];
    int active_unit, client_unit;

    /* multisample state, of no effect: SAMPLE_BUFFERS is 0 */
    int multisample, sample_a2c, sample_a2o, sample_cov, sample_inv;
    float sample_val;

    /* fog coordinates, secondary colours, point parameters */
    GLenum fog_src;
    int color_sum;
    float pt_min, pt_max, pt_fade, pt_att[3];

    /* client arrays */
    gl_array a_vertex, a_color, a_normal, a_tc[MAX_UNITS], a_edge, a_fog, a_sec, a_attr[MAX_ATTRIBS];
    float attr[MAX_ATTRIBS][4];
    struct gs_prog *cur;        /* the user's program in use, or NULL */
    GLuint program;
    int vp_point_size, vp_two_side, point_sprite;
    GLenum sprite_origin;
    int internal_draw;          /* glDrawPixels, glBitmap: pictures: no vertex shader, and a fragment shader sees their colour */
    GLuint pic_tex;             /* the picture, while a fragment shader keeps unit 0's texture for itself */
    int no_flat;                /* asking a shader what its provoking vertex says: the variant that says it */
    int cva_first, cva_count, cva_locked;
    uint32_t cva_mask;          /* the arrays enabled at the lock: only those are promised not to change */
    struct { const uint8_t *base; int stride, bytes; uint32_t off, gen; } cva[A_COUNT];
    GLuint bound_array, bound_element, bound_pack, bound_unpack;

    /* pixel store */
    int un_align, un_row, un_skip_px, un_skip_row, pk_align, pk_row, pk_skip_px, pk_skip_row, un_image_h, un_skip_images, pk_image_h, pk_skip_images;
    uint32_t draws;             /* every draw, for a query's result */
    uint32_t vs_serial;         /* the uniforms a vertex program runs with on the PC last for one draw */
    GLuint query;               /* the occlusion query in progress */
    uint32_t query_from;
    GLenum draw_buffer, read_buffer;
    float pixel_zoom[2];

    /* framebuffers */
    GLuint fbo, read_fbo;       /* GL_EXT_framebuffer_blit: what draws go to, what reads come from */
    GLuint renderbuffer;

    /* immediate mode: one growing batch */
    int in_begin;
    GLenum prim;
    uint32_t im_first;          /* first vertex of the open glBegin */
    float *im_v;                /* position 4, colour 4, then normal 3 and 4 per enabled unit: the layout of the batch */
    uint32_t im_n, im_cap;      /* vertices */
    uint32_t *im_idx;
    uint32_t im_ni, im_icap;
    uint8_t *im_ef, *im_tri;    /* per vertex its edge flag; per triangle which edges are its polygon's, gl_draw.c */
    int im_class;               /* the batch's primitives: 0 triangles, 1 lines, 2 points */
    int im_vsize, im_off_normal, im_off_tc[MAX_UNITS], im_off_fog, im_off_sec, im_off_attr[MAX_ATTRIBS];
    int drawing_points, drawing_cls, drawing_back, drawing_mask;
    float z_off;                /* gl_ff.c: polygon offset of the next draw, in normalized z */

    /* stipple, logic op, the current colour index (no colour-index formats: only state) */
    int line_factor;
    GLushort line_pattern;
    uint8_t poly_pattern[128];
    GLenum logic_op;
    float cur_index;

    /* the accumulation buffer lives on the PC: 16 bits of range are what floats give it */
    float accum_clear[4], *accum;
    int accum_w, accum_h;

    /* pixel transfer */
    float px_scale[4], px_bias[4];
    int px_shift, px_offset, px_map_color, px_active, px_map_stencil;
    float px_depth_scale, px_depth_bias;
    gl_pixmap pmap[10];

    /* evaluators */
    gl_map map[2][9];
    int map_en[2][9], grid1_n, grid2_n[2];
    float grid1[2], grid2[4];

    /* selection and feedback */
    GLenum render_mode;
    GLuint *sel_buf, sel_n, names[MAX_NAME_DEPTH];
    GLsizei sel_size, fb_size;
    int sel_hit, sel_hits, sel_over, name_sp, fb_over;
    float sel_min, sel_max;
    GLfloat *fb_buf;
    GLenum fb_type;
    GLuint fb_n;

    /* display lists */
    GLuint list_base;
    gl_list *compiling;
    GLenum compile_mode;
    int in_list_exec;
    uint32_t cur_dirty;         /* attributes set since the last recorded vertex: bit 0 colour, 1 normal, 2.. texture units */
    uint32_t tc4;               /* units whose current coordinate has had a q other than 1 */

    /* query strings */
    char ext_string[1024];
} gl_ctx;

/* device: what all contexts share */
typedef struct gl_dev {
    CRITICAL_SECTION cs;
    g2_board *bd;
    g2r *r;
    HDC hdc;
    HWND hwnd;
    int pixel_format;
    int mode, mode_w, mode_h;
    int fb_w, fb_h;             /* the default framebuffer: what is rendered */
    int vw, vh;                 /* the window, the size the application sees: larger than fb_w, fb_h when the hardware is smaller than the window */
    /* three colour images: without PRESENT hold B does not wait for the flip, and with two the next frame would start in the image still on the monitor, which then shows a world without its HUD for up to one refresh */
    uint16_t color[3], ds, scan[3];
    uint32_t color_base[3], ds_base, scan_base[3];
    int back, front, scan_idx, scan_ok;
    int has_depth, has_stencil, doublebuffer;
    uint32_t color_fmt;         /* the pixel format's colour: RGBA8 or RGB565 */
    int depth16;                /* its depth buffer has 16 bits */
    int hold, stretch, swap_interval;
    uint32_t frame;
} gl_dev;

extern gl_ctx *C;
extern gl_dev D;

void gl_im_flush(void);
#define FLUSH_VERTICES() do { if (C->im_n && !C->in_begin) gl_im_flush(); } while (0)
#define SET_ERROR(e) do { if (C->error == GL_NO_ERROR) C->error = (e); } while (0)

/* gl_dev.c */
int gl_dev_up(void);
void gl_dev_framebuffer(int w, int h);
void gl_ctx_init(gl_ctx *c);
gl_ctx *gl_null_ctx(void);
void gl_swap(void);
void gl_bind_target(void);
void gl_cur_target(g2r_target *t);
void gl_read_target(g2r_target *t);
void gl_scale(double *kx, double *ky);                  /* the target's pixels per window pixel: 1 but for a window larger than the hardware */
int gl_scissor_px(const g2r_target *t, int *b);         /* the scissor box in the target's pixels, x0 y0 x1 y1, the whole target without the test; 0: empty */

/* gl_ff.c */
ff_prog *gl_ff_program(void);
void gl_ff_key(ff_key *k);
void gl_ff_sw_key(ff_key *k, int u, const gl_tex *x);
void gl_ff_text(const ff_key *k, char **vs, char **fs, gl_ureq *vu, int *nvu, gl_ureq *fu, int *nfu);
int gl_ff_text_tight(const ff_key *k, char **vs, char **fs);   /* the program's texts, 1 = tightened for g2sc_link (tests/ff_dump.c) */
void gl_ff_sw_text(const ff_key *k, int u, char **text, gl_ureq *fu, int *nfu);
int gl_ff_find(const g2sc_sym *s, int n, const char *name);
int gl_ff_attr_of(const char *name);                    /* a generated program's attribute name -> A_*, or -1 */

/* gl_glsl.c */
ff_prog *gl_glsl_program(void);
void gl_glsl_uniforms(const ff_prog *p, uint32_t *vs, uint32_t *fs);
void gl_glsl_samplers(g2r_state *s, const ff_prog *p);
const gl_tex *gl_glsl_tex(const ff_prog *p, int sampler);
uint32_t gl_glsl_attribs(void);                         /* the generic attributes 1..15 the program in use reads, as a mask */
int gl_glsl_uses_generic0(void);
void gl_glsl_uniform_replay(const gl_listop *o);
int gl_glsl_user_vs(void);
int gl_glsl_user_fs(void);
int gl_vs_clip(const src *in, uint32_t v, float *c);    /* where a user's vertex program puts vertex v; 0: no program */
int gl_vs_outputs(const src *in, uint32_t v, float *clip, float *color, float *tc0, float *cd);   /* cd: each user clip plane's distance, MAX_CLIP */
void gl_vs_flat(const src *in, uint32_t v, float *o);   /* the four colours of a provoking vertex: front, back, front secondary, back secondary */

/* gl_vsrun.c */
void *gl_vs_parse(const char *sasm);
void gl_vs_free(void *p);
void gl_vs_exec(const void *p, const double (*attr)[4], const double (*par)[4], double *pos, double (*vary)[4]);
void gl_ff_uniforms(const ff_prog *p, uint32_t *vs, uint32_t *fs);
const float *gl_cur_matrix(void);
float *gl_top_matrix(GLenum mode);
void gl_mul(float *r, const float *a, const float *b);
void gl_ident(float *m);
int gl_invert(float *out, const float *m);

/* gl_draw.c */
void gl_set_array(gl_array *a, GLint size, GLenum type, GLsizei stride, const void *p);
void gl_draw_arrays(GLenum mode, GLint first, GLsizei count);
void gl_draw_elements(GLenum mode, GLsizei count, GLenum type, const void *indices, GLuint start, GLuint end, int have_range);
void gl_fill_hw_state(g2r_state *s);
void gl_src_get(const src *s, uint32_t v, float *o);
void gl_src_current(src *in);                           /* every attribute as its current value */

/* gl_select.c */
void gl_select_prims(const src *in, int cls, const uint32_t *list, uint32_t n);
void gl_select_raster(GLenum token);
void gl_raster_set(const float *obj);
int gl_ds_type(GLenum format, GLenum type);

/* gl_pixel.c */
void gl_extra_init(gl_ctx *c);
int gl_extra_ints(GLenum p, GLint *v);
void gl_px_one(uint8_t *o);
void gl_px_index(uint32_t i, uint8_t *o);
void gl_px_rows(uint8_t *rgba, int w, int h);
void gl_accum_clear(int x0, int y0, int x1, int y1);
uint16_t gl_stipple_image(int line);
uint32_t gl_px_stencil_value(uint32_t v);
float gl_px_depth_value(float z);
uint8_t *gl_px_fragments(const float *z, const uint32_t *st, int w, int h, float zx, float zy, int *rect);

/* gl_eval.c */
void gl_eval_clear(gl_ctx *c);

/* gl_tex.c */
gl_tex *gl_tex_lookup(GLuint name, GLenum target, int create);
gl_tex *gl_unit_tex(int unit, int kind);              /* 0 2D, 1 cube, 2 1D, 3 3D */
void gl_tex_touch(gl_tex *t);
void gl_tex_resolve(gl_tex *t);                         /* what was rendered goes into the texture */
void gl_tex_rt_drop(gl_tex *t);
const void *gl_unpack_ptr(const void *pixels);          /* GL_PIXEL_UNPACK_BUFFER: an offset into it */
#define GL_FROM_CLIENT(call) do { GLuint keep_ = C->bound_unpack; C->bound_unpack = 0; call; C->bound_unpack = keep_; } while (0)     /* the library's own upload: its pointer is an address whatever the application has bound */
void *gl_pack_ptr(void *pixels);
int gl_pbo_fits(int pack, const void *pixels, size_t bytes);    /* INVALID_OPERATION when not: mapped, or outside the buffer */
int gl_base_class(GLenum internal);
void gl_tex3d_free(gl_tex *t);
void gl_tex3d_rt_load(gl_tex *t);                       /* the rendered slice of a 3D texture, from and to the PC's copy */
void gl_tex3d_rt_store(gl_tex *t);
void gl_tex3d_generate(gl_tex *t);
int gl_tex3d_fits(const gl_tex *t, int level);
int gl_tex3d_tmu(const gl_tex *t);                     /* its wrap modes are the TMU's */
int gl_tex3d_fit(int level, int w, int h, int d, int b);
uint16_t gl_tex3d_view(gl_tex *t, int b, int n);        /* the n levels of a volume from b on */
void gl_tex_view_drop(gl_tex *t);
void gl_tex_proxy(GLenum target, int level, int w, int h, int d, GLenum internal);
int gl_tex_complete(const gl_tex *t);
void gl_tex_prepare(gl_tex *t);
void gl_unpack(uint8_t *rgba, GLsizei w, GLsizei h, GLenum format, GLenum type, const void *pixels, GLenum internal);
void gl_unpack_depth(float *o, GLsizei w, GLsizei h, GLenum type, const void *pixels);
void gl_unpack_ds(float *z, uint32_t *s, GLsizei w, GLsizei h, const void *pixels);    /* GL_DEPTH_STENCIL_EXT, GL_UNSIGNED_INT_24_8_EXT */
void gl_unpack_index(uint32_t *o, GLsizei w, GLsizei h, GLenum type, const void *pixels);
int gl_read_rect(const g2r_target *t, uint16_t image, uint32_t tf, uint32_t fmt, int x, int y, int w, int h, void *out);
/* storage formats (gl_tex.c): what an internal format is stored as, bytes and bits of a format, a colour image's texels to and from R G B A bytes */
uint32_t gl_hw_format(GLenum internal, GLenum type);
void gl_fbo_ds_bits(GLuint fbo, int *depth, int *stencil);
int gl_fmt_bpp(uint32_t fmt);
int gl_fmt_bits(uint32_t fmt, int c);
uint32_t gl_tg_fmt(const g2r_target *t);                 /* a target's colour format, from its bits */
void gl_tg_bits(g2r_target *t, uint32_t fmt);
int gl_img_write(uint16_t img, uint32_t fmt, const uint8_t *rgba, int pitch, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t level, uint32_t layer);   /* pitch in pixels */
int gl_img_read(uint16_t img, uint32_t fmt, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t level, uint32_t layer, uint8_t *rgba);
void gl_quantize(uint32_t fmt, uint8_t *rgba, size_t n); /* what the format keeps of R G B A bytes, as a read gives them back */
gl_tex *gl_bound_tex(GLenum target);
const void *gl_unpack_source(const void *data);          /* what an upload's pointer means: client memory */
void gl_tex_store_rgba(GLenum target, GLint level, GLenum internal, int w, int h, const uint8_t *rgba);        /* tight RGBA8, past pixel store and pixel transfer */
void gl_tex_sub_rgba(GLenum target, GLint level, int x, int y, int w, int h, const uint8_t *rgba);
uint16_t gl_tex_sample_image(gl_tex *t);                 /* the image the draw hands the TMU: BASE_LEVEL and MAX_LEVEL say which part of the chain */
void gl_tex_border_color(const gl_tex *t, float *o);
gl_buffer *gl_buffer_lookup(GLuint name);
gl_fbo *gl_fbo_lookup(GLuint name);
int gl_fbo_target(gl_fbo *f, g2r_target *t);

/* gl_list.c */
void gl_list_i2(int kind, uint32_t a, uint32_t b);
void gl_list_f4(int kind, uint32_t a, float x, float y, float z, float w);
void gl_list_big(int kind, uint32_t a, const void *data, size_t bytes);
void gl_list_bitmap(int w, int h, const float *f4, const void *bits, size_t bytes);
void gl_list_uniform(GLint loc, int kind, int comps, int n, const void *data, size_t bytes);
void gl_list_sync_attribs(void);
/* at the top of a call a display list can hold: record it; GL_COMPILE stops there */
#define LREC_I(kind, a, b) do { if (C->compiling) { gl_list_i2(kind, (uint32_t)(a), (uint32_t)(b)); if (C->compile_mode == GL_COMPILE) return; } } while (0)
#define LREC_F(kind, a, x, y, z, w) do { if (C->compiling) { gl_list_f4(kind, (uint32_t)(a), x, y, z, w); if (C->compile_mode == GL_COMPILE) return; } } while (0)
#endif
