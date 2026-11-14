/* d3d9_rec.c — a flight recorder for faults that last a frame. GPU2_RECORD=<folder> keeps the last REC_FRAMES frames in memory, every draw and clear with a copy of the whole d3state, the vertices it read and
 * its indices; shaders, textures and targets are held by reference until their operation leaves the ring, and written once. F12, with the game in front, writes the ring to <folder>\rec_NNN.g2rc two frames later
 * (the person saw the fault a moment after it was drawn). GPU2_RECORD_AFTER=<frame> does the same with no key, for tests. tests/d3_replay.c plays the file on the boards with no game. Off: one test per draw. */
#include "d3d9_internal.h"
#include "d3d9_rec.h"

#define REC_FRAMES 48
#define WRITE_DELAY 2

typedef struct rec_op {
    struct rec_op *next;
    rec_ophead h;
    d3state st;
    d3surf *rt, *ds;
    uint32_t *idx;
    D3DRECT *rects;
    struct { uint8_t *data; uint32_t bytes, stride; } s[MAX_STREAMS];
} rec_op;

static rec_op *g_first[REC_FRAMES], *g_last[REC_FRAMES];
static int g_at, g_left, g_key, g_on = -1, g_file;
static long long g_after = -1;
static uint32_t g_frame;

static void hold(void *o) { if (o) d3_bind(o); }               /* the recorder's own: a game that releases until 0 does not take it */
static void drop(void *o) { if (o) d3_unbind(o); }

static void op_refs(rec_op *o, void (*f)(void *))
{
    int i;
    for (i = 0; i < MAX_SAMPLERS + 5; i++) f(o->st.tex[i]);
    f(o->st.decl); f(o->st.vs); f(o->st.ps); f(o->rt); f(o->ds);
}

static void op_free(rec_op *o)
{
    int i;
    op_refs(o, drop);
    for (i = 0; i < MAX_STREAMS; i++) free(o->s[i].data);
    free(o->idx); free(o->rects); free(o);
}

int d3_rec_on(void)
{
    if (g_on < 0) {
        g_on = g2e_get("GPU2_RECORD", NULL) != NULL;
        g_after = g2e_get_int("GPU2_RECORD_AFTER", -1);
        if (g_on) g2e_log("record: the last %d frames are kept, F12 saves them", REC_FRAMES);
    }
    return g_on;
}

static rec_op *op_new(d3dev *d, int kind)
{
    rec_op *o = (rec_op *)calloc(1, sizeof *o);
    if (!o) return NULL;
    o->h.kind = (uint32_t)kind; o->h.frame = g_frame;
    memcpy(&o->st, &d->st, sizeof o->st);
    o->rt = d->rt; o->ds = d->ds;
    o->h.rt = (uint32_t)(uintptr_t)d->rt; o->h.ds = (uint32_t)(uintptr_t)d->ds;
    op_refs(o, hold);
    if (g_last[g_at]) g_last[g_at]->next = o; else g_first[g_at] = o;
    g_last[g_at] = o;
    return o;
}

void d3_rec_draw(d3dev *d, int mode, const uint32_t *idx, uint32_t n, uint32_t first, const uint8_t *up, UINT up_stride)
{
    const d3state *st = &d->st;
    rec_op *o = op_new(d, REC_DRAW);
    uint32_t lo = first, hi = first + n - 1, i, used = 0;
    int k;
    if (!o) return;
    if (idx) {
        lo = 0xFFFFFFFFu; hi = 0;
        for (i = 0; i < n; i++) { if (idx[i] < lo) lo = idx[i]; if (idx[i] > hi) hi = idx[i]; }
        o->idx = (uint32_t *)malloc((size_t)n * 4);
        if (o->idx) memcpy(o->idx, idx, (size_t)n * 4);
        o->h.nidx = o->idx ? n : 0;
    }
    o->h.mode = (uint32_t)mode; o->h.n = n; o->h.first = first; o->h.lo = lo; o->h.hi = hi;
    for (k = 0; k < st->decl->n; k++) used |= 1u << st->decl->el[k].Stream;
    for (k = 0; k < MAX_STREAMS; k++) {
        const uint8_t *base;
        uint32_t stride, bytes, room = 0xFFFFFFFFu;
        if (!(used >> k & 1)) continue;
        if (up) { if (k) continue; base = up; stride = up_stride; }
        else {
            const d3buf *b = st->vb[k];
            if (!b) continue;
            base = b->mem + st->vb_off[k]; stride = st->vb_stride[k];
            room = b->size - st->vb_off[k];
        }
        bytes = stride ? (hi - lo + 1) * stride : 64;
        if ((uint64_t)lo * stride >= room) continue;
        if (bytes > room - lo * stride) bytes = room - lo * stride;
        o->s[k].data = (uint8_t *)malloc(bytes);
        if (!o->s[k].data) continue;
        memcpy(o->s[k].data, base + (size_t)lo * stride, bytes);
        o->s[k].bytes = bytes; o->s[k].stride = stride;
        o->h.streams |= 1u << k;
    }
}

void d3_rec_clear(d3dev *d, DWORD n, const D3DRECT *rects, DWORD flags, D3DCOLOR c, float z, DWORD st)
{
    rec_op *o = op_new(d, REC_CLEAR);
    if (!o) return;
    o->h.cflags = flags; o->h.ccolor = c; o->h.cz = z; o->h.cstencil = st;
    if (n && rects) {
        o->rects = (D3DRECT *)malloc((size_t)n * sizeof *rects);
        if (o->rects) { memcpy(o->rects, rects, (size_t)n * sizeof *rects); o->h.cn = n; }
    }
}

/* ---------------------------------------------------------------- the file */
typedef struct seen { const void **v; int n, cap; } seen;

static void seen_add(seen *s, const void *p)
{
    int i;
    if (!p) return;
    for (i = 0; i < s->n; i++) if (s->v[i] == p) return;
    if (s->n == s->cap) {
        int cap = s->cap ? s->cap * 2 : 256;
        const void **v = (const void **)realloc((void *)s->v, (size_t)cap * sizeof *v);
        if (!v) return;
        s->v = v; s->cap = cap;
    }
    s->v[s->n++] = p;
}

static void write_ring(d3dev *d)
{
    char path[MAX_PATH];
    seen sh = { 0 }, de = { 0 }, tx = { 0 }, sf = { 0 };
    rec_head h;
    FILE *f;
    int i, k, age;
    long long total;
    snprintf(path, sizeof path, "%s\\rec_%03d.g2rc", g2e_get("GPU2_RECORD", "."), g_file++);
    f = fopen(path, "wb");
    if (!f) { g2e_log("record: cannot write %s", path); return; }
    memset(&h, 0, sizeof h);
    h.magic = REC_MAGIC; h.version = REC_VERSION; h.state_size = sizeof(d3state);
    h.vw = (uint32_t)d->vw; h.vh = (uint32_t)d->vh; h.back_fmt = d->pp.BackBufferFormat; h.ds_fmt = d->pp.AutoDepthStencilFormat; h.windowed = (uint32_t)d->pp.Windowed;
    for (age = 1; age <= REC_FRAMES; age++) {                   /* oldest first */
        const rec_op *o = g_first[(g_at + age) % REC_FRAMES];
        if (o) h.nframes++;
        for (; o; o = o->next) {
            h.nops++;
            seen_add(&sh, o->st.vs); seen_add(&sh, o->st.ps); seen_add(&de, o->st.decl); seen_add(&sf, o->rt); seen_add(&sf, o->ds);
            for (i = 0; i < MAX_SAMPLERS + 5; i++) seen_add(&tx, o->st.tex[i]);
            if (o->rt && o->rt->tex) seen_add(&tx, o->rt->tex);
        }
    }
    h.nshaders = (uint32_t)sh.n; h.ndecls = (uint32_t)de.n; h.ntex = (uint32_t)tx.n; h.nsurf = (uint32_t)sf.n;
    fwrite(&h, sizeof h, 1, f);
    for (i = 0; i < sh.n; i++) {
        const d3shader *s = (const d3shader *)sh.v[i];
        rec_shader r;
        r.id = (uint32_t)(uintptr_t)s; r.ps = (uint32_t)s->ps; r.ntok = s->ntok;
        fwrite(&r, sizeof r, 1, f);
        fwrite(s->tok, 4, s->ntok, f);
    }
    for (i = 0; i < de.n; i++) {
        const d3decl *x = (const d3decl *)de.v[i];
        rec_decl r;
        r.id = (uint32_t)(uintptr_t)x; r.n = (uint32_t)x->n;
        fwrite(&r, sizeof r, 1, f);
        fwrite(x->el, sizeof x->el[0], (size_t)x->n + 1, f);
    }
    for (i = 0; i < tx.n; i++) {
        const d3tex *t = (const d3tex *)tx.v[i];
        rec_tex r;
        r.id = (uint32_t)(uintptr_t)t; r.w = t->vw; r.h = t->vh; r.levels = (uint32_t)t->levels; r.layers = (uint32_t)t->layers; r.fmt = t->fmt; r.usage = t->usage; r.pool = t->pool;
        fwrite(&r, sizeof r, 1, f);
        for (k = 0; k < t->layers * t->levels; k++) {
            const d3surf *s = t->surf ? t->surf[k] : NULL;              /* a volume has no surfaces: the recording keeps its size alone */
            rec_level l;
            l.pitch = s ? (uint32_t)s->pitch : 0; l.bytes = s && s->mem && !(t->usage & D3DUSAGE_RENDERTARGET) ? (uint32_t)d3_shadow_bytes(s) : 0;
            fwrite(&l, sizeof l, 1, f);
            if (l.bytes) fwrite(s->mem, 1, l.bytes, f);
        }
    }
    for (i = 0; i < sf.n; i++) {
        const d3surf *s = (const d3surf *)sf.v[i];
        rec_surf r;
        r.id = (uint32_t)(uintptr_t)s; r.tex = (uint32_t)(uintptr_t)s->tex; r.level = (uint32_t)s->level; r.face = (uint32_t)s->face; r.w = s->w; r.h = s->h; r.fmt = s->fmt; r.usage = s->usage; r.back = (uint32_t)s->back; r.auto_ds = s == d->auto_ds;
        fwrite(&r, sizeof r, 1, f);
    }
    for (age = 1; age <= REC_FRAMES; age++) {
        const rec_op *o;
        for (o = g_first[(g_at + age) % REC_FRAMES]; o; o = o->next) {
            fwrite(&o->h, sizeof o->h, 1, f);
            fwrite(&o->st, sizeof o->st, 1, f);
            if (o->h.cn) fwrite(o->rects, sizeof o->rects[0], o->h.cn, f);
            if (o->h.nidx) fwrite(o->idx, 4, o->h.nidx, f);
            for (k = 0; k < MAX_STREAMS; k++) {
                if (o->h.streams >> k & 1) {
                    rec_stream r;
                    r.index = (uint32_t)k; r.stride = o->s[k].stride; r.bytes = o->s[k].bytes;
                    fwrite(&r, sizeof r, 1, f);
                    fwrite(o->s[k].data, 1, o->s[k].bytes, f);
                }
            }
        }
    }
    total = _ftelli64(f);
    fclose(f);
    g2e_log("record: %s: %u frames, %u operations, %lld MB", path, h.nframes, h.nops, total >> 20);
    free((void *)sh.v); free((void *)de.v); free((void *)tx.v); free((void *)sf.v);
}

/* the end of a frame, before the swap */
void d3_rec_present(d3dev *d)
{
    rec_op *o = op_new(d, REC_PRESENT);
    int down = (GetAsyncKeyState(VK_F12) & 0x8000) != 0 && GetForegroundWindow() == GetAncestor(d->hwnd, GA_ROOT);
    (void)o;
    if ((down && !g_key && !g_left) || (g_after >= 0 && (long long)g_frame == g_after)) {
        g_left = WRITE_DELAY + 1;
        g2e_log("record: frame %u marked", g_frame);
    }
    g_key = down;
    if (g_left && !--g_left) write_ring(d);
    g_frame++;
    g_at = (g_at + 1) % REC_FRAMES;
    while (g_first[g_at]) { rec_op *x = g_first[g_at]; g_first[g_at] = x->next; op_free(x); }
    g_last[g_at] = NULL;
}
