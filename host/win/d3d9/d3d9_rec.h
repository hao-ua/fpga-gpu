/* d3d9_rec.h — the file d3d9_rec.c writes and tests/d3_replay.c reads: the last frames of a game, every operation with the whole state it ran under.
 * rec_head, the tables (shaders, declarations, textures, surfaces), then the operations in order. Ids are the objects' addresses in the recorded process. */
#ifndef D3D9_REC_H
#define D3D9_REC_H
#include <stdint.h>

#define REC_MAGIC 0x43523247u                   /* "G2RC" */
#define REC_VERSION 1

enum { REC_DRAW, REC_CLEAR, REC_PRESENT };

typedef struct rec_head {
    uint32_t magic, version, state_size;        /* sizeof(d3state): the two programs must agree on it */
    uint32_t nframes, nops, nshaders, ndecls, ntex, nsurf;
    uint32_t vw, vh, back_fmt, ds_fmt, windowed;
} rec_head;

typedef struct rec_shader { uint32_t id, ps, ntok; } rec_shader;                    /* + ntok tokens, Direct3D 9's form */
typedef struct rec_decl { uint32_t id, n; } rec_decl;                               /* + n + 1 elements */
typedef struct rec_tex { uint32_t id, w, h, levels, layers, fmt, usage, pool; } rec_tex;   /* + per surface a rec_level and its bytes; no pixels for a target */
typedef struct rec_level { uint32_t pitch, bytes; } rec_level;
typedef struct rec_surf { uint32_t id, tex, level, face, w, h, fmt, usage, back, auto_ds; } rec_surf;

typedef struct rec_ophead {
    uint32_t kind, frame;
    uint32_t mode, n, first, nidx, lo, hi;      /* a draw: the hardware's mode, absolute indices */
    uint32_t rt, ds;
    uint32_t cn, cflags, ccolor, cstencil;      /* a clear */
    float cz;
    uint32_t streams;                           /* the bit of each stream that follows */
} rec_ophead;
/* + d3state, cn rectangles, nidx indices, and per stream a rec_stream and its bytes: the vertices from lo */
typedef struct rec_stream { uint32_t index, stride, bytes; } rec_stream;
#endif
