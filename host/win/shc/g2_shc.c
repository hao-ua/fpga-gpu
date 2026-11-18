/* g2_shc.c — the public API: compile_stage's register-spill fallback, isa64.assemble, the result tables. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "shc_internal.h"

static char *dup_str(const char *s)
{
    size_t n = strlen(s) + 1;
    char *p = (char *)malloc(n);
    if (p) memcpy(p, s, n);
    return p;
}

static void copy_trunc(char *dst, size_t cap, const char *s)
{
    size_t n = strlen(s);
    if (n >= cap) n = cap - 1;
    memcpy(dst, s, n);
    dst[n] = 0;
}

static int syms_out(const ShSym *in, int n, g2sc_sym **out, int *nout)
{
    g2sc_sym *t;
    int i;
    *nout = 0; *out = NULL;
    if (!n) return 0;
    t = (g2sc_sym *)calloc((size_t)n, sizeof(g2sc_sym));
    if (!t) return -1;
    *out = t; *nout = n;
    for (i = 0; i < n; i++) {
        copy_trunc(t[i].name, sizeof t[i].name, in[i].name);
        copy_trunc(t[i].type, sizeof t[i].type, in[i].ty);
        t[i].loc = (int)in[i].loc;
        t[i].has_array = in[i].has_array;
        t[i].array = in[i].has_array ? (int)in[i].array : 0;
        t[i].lname = dup_str(in[i].name);
        t[i].ltype = dup_str(in[i].ty);
        if (!t[i].lname || !t[i].ltype) return -1;
    }
    return 0;
}

static void syms_free(g2sc_sym *t, int n)
{
    int i;
    if (!t) return;
    for (i = 0; i < n; i++) { free((void *)t[i].lname); free((void *)t[i].ltype); }
    free(t);
}

static int result_out(const ShOutput *o, const uint64_t *words, int nwords, g2sc_result *r)
{
    int i, m;
    if (o->const_base > 0x7FFFFFFFLL || o->const_base < -0x7FFFFFFFLL) return -2;
    r->const_base = (int)o->const_base;
    r->nwords = nwords;
    r->words = (uint64_t *)malloc(((size_t)nwords + 1) * sizeof(uint64_t));
    if (!r->words) return -1;
    if (nwords) memcpy(r->words, words, (size_t)nwords * sizeof(uint64_t));
    if (syms_out(o->attribs, o->nattribs, &r->attribs, &r->nattribs)) return -1;
    if (syms_out(o->uniforms, o->nuniforms, &r->uniforms, &r->nuniforms)) return -1;
    if (syms_out(o->samplers, o->nsamplers, &r->samplers, &r->nsamplers)) return -1;
    if (syms_out(o->varyings, o->nvaryings, &r->varyings, &r->nvaryings)) return -1;
    r->nconst = o->npool;
    if (o->npool) {
        r->const_pool = (double (*)[4])malloc((size_t)o->npool * sizeof(double[4]));
        if (!r->const_pool) return -1;
        memcpy(r->const_pool, o->pool, (size_t)o->npool * sizeof(double[4]));
    }
    r->sasm = dup_str(o->sasm);
    if (!r->sasm) return -1;
    r->vs_tex = o->vs_tex;
    if (o->nstructs) {
        r->structs = (g2sc_struct *)calloc((size_t)o->nstructs, sizeof(g2sc_struct));
        if (!r->structs) return -1;
        r->nstructs = o->nstructs;
        for (i = 0; i < o->nstructs; i++) {
            const ShStruct *s = &o->structs[i];
            g2sc_struct *d = &r->structs[i];
            d->name = dup_str(s->name);
            if (!d->name) return -1;
            d->nslots = (int)s->nslots;
            if (s->nmembers) {
                d->members = (g2sc_member *)calloc((size_t)s->nmembers, sizeof(g2sc_member));
                if (!d->members) return -1;
                d->nmembers = s->nmembers;
                for (m = 0; m < s->nmembers; m++) {
                    const ShMember *sm = &s->members[m];
                    g2sc_member *dm = &d->members[m];
                    dm->name = dup_str(sm->name);
                    dm->type = dup_str(sm->ty);
                    if (!dm->name || !dm->type) return -1;
                    dm->offset = (int)sm->offset; dm->nslots = (int)sm->nslots;
                    dm->array = (int)sm->array; dm->has_array = sm->has_array; dm->prec = sm->prec;
                }
            }
        }
    }
    return 0;
}

void g2sc_free(g2sc_result *r)
{
    int i, m;
    if (!r) return;
    free(r->words);
    syms_free(r->attribs, r->nattribs);
    syms_free(r->uniforms, r->nuniforms);
    syms_free(r->samplers, r->nsamplers);
    syms_free(r->varyings, r->nvaryings);
    free(r->const_pool);
    free(r->sasm);
    if (r->structs) {
        for (i = 0; i < r->nstructs; i++) {
            g2sc_struct *d = &r->structs[i];
            for (m = 0; m < d->nmembers; m++) { free((void *)d->members[m].name); free((void *)d->members[m].type); }
            free(d->members);
            free((void *)d->name);
        }
        free(r->structs);
    }
    memset(r, 0, sizeof *r);
}

static void set_error(g2sc_result *r, const char *msg)
{
    copy_trunc(r->error, sizeof r->error, msg);
}

static int compile_opts(const char *src, int stage, const char *const *varying_order, int n_order, uint32_t unread, int strict, int may_pack, const g2sc_pack *pack, const g2sc_bind *binds, int nbinds, g2sc_result *out);
typedef char bind_layout[sizeof(g2sc_bind) == sizeof(ShBind) && offsetof(g2sc_bind, loc) == offsetof(ShBind, loc) ? 1 : -1];     /* the compiler reads a g2sc_bind table as ShBind */

/* the one setting outside a compile (g2_shc.h): what board B's fragment core can run, glslc.compile_stage's xops */
static volatile long g_xops;
void g2sc_set_target(uint32_t xops) { g_xops = (long)(xops & 3u); }
uint32_t g2sc_target(void) { return (uint32_t)g_xops; }
/* and the units each stage's core lacks, glslc.compile_stage's noops */
static volatile long g_noops_vs, g_noops_fs;
void g2sc_set_absent(uint32_t vs, uint32_t fs) { g_noops_vs = (long)(vs & 3u); g_noops_fs = (long)(fs & 3u); }
uint32_t g2sc_absent(int stage) { return (uint32_t)(stage == G2SC_FS ? g_noops_fs : g_noops_vs); }

int g2sc_compile(const char *src, int stage, g2sc_result *out) { return compile_opts(src, stage, NULL, 0, 0, 0, 0, NULL, NULL, 0, out); }
int g2sc_compile_strict(const char *src, int stage, g2sc_result *out) { return compile_opts(src, stage, NULL, 0, 0, 1, 1, NULL, NULL, 0, out); }
int g2sc_compile_ex(const char *src, int stage, const char *const *varying_order, int n_order, g2sc_result *out) { return compile_opts(src, stage, varying_order, n_order, 0, 0, 1, NULL, NULL, 0, out); }
int g2sc_compile_vs(const char *src, uint32_t unread_varyings, g2sc_result *out) { return compile_opts(src, G2SC_VS, NULL, 0, unread_varyings, 0, 0, NULL, NULL, 0, out); }
int g2sc_compile_packed(const char *src, int stage, const char *const *varying_order, int n_order, const g2sc_pack *pack, g2sc_result *out) { return compile_opts(src, stage, varying_order, n_order, pack ? pack->unread : 0, 0, 1, pack, NULL, 0, out); }
int g2sc_compile_bound(const char *src, const g2sc_bind *binds, int nbinds, g2sc_result *out) { return compile_opts(src, G2SC_VS, NULL, 0, 0, 0, 1, NULL, binds, nbinds, out); }

/* the locations the active attributes take, to the last one */
static long long attr_end(const ShOutput *o)
{
    long long end = 0;
    int i;
    for (i = 0; i < o->nattribs; i++) {
        const ShSym *a = &o->attribs[i];
        long long e = a->loc + (!strncmp(a->ty, "mat", 3) ? a->ty[3] - '0' : 1);
        if (e > end) end = e;
    }
    return end;
}

/* the varying vectors as declared, up to the last one */
static int vary_rows(const ShOutput *o)
{
    long long rows = 0;
    int i;
    for (i = 0; i < o->nvaryings; i++) {
        const ShSym *v = &o->varyings[i];
        long long e = v->loc + (!strncmp(v->ty, "mat", 3) ? v->ty[3] - '0' : 1) * (v->has_array ? v->array : 1);
        if (e > rows) rows = e;
    }
    return rows > 0x7FFFFFFFLL ? 0x7FFFFFFF : (int)rows;
}

/* the shortest free run of a column that holds n rows */
static int free_run(const unsigned char *used, int rows, int col, int n, int *top)
{
    int r, start = 0, best = rows + 1;
    for (r = 0; r <= rows; r++) {
        if (r < rows && !(used[r] >> col & 1)) continue;
        if (r - start >= n && r - start < best) { best = r - start; *top = start; }
        start = r + 1;
    }
    return best;
}

int g2sc_pack_vars(const int *rank, const int *rows, int n, int max_rows, int *out)
{
    int *ord = (int *)malloc(((size_t)n + 1) * sizeof(int));
    unsigned char *used = (unsigned char *)calloc((size_t)max_rows + 1, 1);
    int i, k, top = 0, a01, a23, next01, bottom = max_rows, last = 0, rc = -1;
    if (!ord || !used) goto done;
    for (i = 0; i < n; i++) {                                   /* by type, the larger arrays first */
        for (k = i; k > 0 && (rank[ord[k - 1]] > rank[i] || (rank[ord[k - 1]] == rank[i] && rows[ord[k - 1]] < rows[i])); k--) ord[k] = ord[k - 1];
        ord[k] = i;
    }
    for (i = 0; i < n && rank[ord[i]] <= 4; i++) {              /* whole rows, then three columns of a row */
        int v = ord[i], w = rank[v] <= 2 ? 15 : 7;
        if (top + rows[v] > max_rows) goto done;
        out[v] = top * 4;
        for (k = 0; k < rows[v]; k++) used[top + k] = (unsigned char)w;
        top += rows[v];
    }
    a01 = a23 = max_rows - top; next01 = top;
    for (; i < n && rank[ord[i]] == 5; i++) {                   /* two columns: the left pair down, the right pair up */
        int v = ord[i];
        if (rows[v] <= a01) {
            out[v] = next01 * 4;
            for (k = 0; k < rows[v]; k++) used[next01 + k] |= 3;
            next01 += rows[v]; a01 -= rows[v];
        } else if (rows[v] <= a23) {
            bottom -= rows[v];
            out[v] = bottom * 4 + 2;
            for (k = 0; k < rows[v]; k++) used[bottom + k] |= 12;
            a23 -= rows[v];
        } else goto done;
    }
    for (; i < n; i++) {                                        /* one column: the tightest gap */
        int v = ord[i], c, best = max_rows + 1, col = -1, at = 0;
        for (c = 0; c < 4; c++) {
            int t = 0, sz = free_run(used, max_rows, c, rows[v], &t);
            if (sz < best) { best = sz; col = c; at = t; }
        }
        if (col < 0) goto done;
        out[v] = at * 4 + col;
        for (k = 0; k < rows[v]; k++) used[at + k] |= (unsigned char)(1 << col);
    }
    for (i = 0; i < max_rows; i++) if (used[i]) last = i + 1;
    rc = last;
done:
    free(ord); free(used);
    return rc;
}

static int compile_opts(const char *src, int stage, const char *const *varying_order, int n_order, uint32_t unread, int strict, int may_pack, const g2sc_pack *pack, const g2sc_bind *binds, int nbinds, g2sc_result *out)
{
    Shc *h;
    ShOutput o;
    uint64_t *words = NULL;
    int nwords = 0, rc, over, i;
    volatile int attempt = 0;
    if (!out) return -1;
    memset(out, 0, sizeof *out);
    if (!src) { set_error(out, "no source"); return -1; }
    if (stage != G2SC_VS && stage != G2SC_FS) { set_error(out, "bad stage"); return -1; }
    if (!binds || stage != G2SC_VS) nbinds = 0;
    for (i = 0; i < nbinds; i++) if (!binds[i].name || binds[i].loc < 0 || binds[i].loc >= G2SC_MAX_ATTRIBS) { set_error(out, "attribute location out of range"); return -1; }
    h = (Shc *)malloc(sizeof(Shc));
    if (!h) { set_error(out, "out of memory"); return -1; }
    h->binds = (const ShBind *)binds; h->nbinds = nbinds;
    h->vary_order = varying_order; h->nvary_order = varying_order ? n_order : 0;
    h->unread = unread;
    h->strict = strict;
    h->umap = pack ? pack->umap : NULL; h->urows = pack ? pack->urows : 0;
    h->vmap = pack ? pack->vmap : NULL; h->nvmap = pack ? pack->nvmap : 0;
    h->depth_units = pack ? pack->depth_units : 0;
    h->xops = stage == G2SC_FS ? (uint32_t)g_xops & 3u : 0;     /* the vertex core has none of them */
    h->noops = g2sc_absent(stage);
    sh_init(h);
retry:
    if (setjmp(h->jb)) {
        /* compile_stage: a CompileError mentioning 'register spill' retries without the loop-invariant hoist (attempt 1), then with remat_consts=True and no hoist (2) */
        if (attempt < 2 && h->compile_error && strstr(h->err, "register spill")) {
            attempt++;
            sh_done(h);
            sh_init(h);
            goto retry;
        }
        set_error(out, h->err);
        if (!out->error[0]) set_error(out, "compile failed");
        sh_done(h);
        free(h);
        return -1;
    }
    memset(&o, 0, sizeof o);
    h->nohoist = attempt >= 1;
    sh_compile(h, src, stage == G2SC_FS, attempt == 2, &o);
    over = may_pack && (o.const_base + o.npool > (stage == G2SC_VS ? 256 : 64) || (!h->vmap && vary_rows(&o) > 8) || attr_end(&o) > G2SC_MAX_ATTRIBS);
    if (!over) words = sh_assemble(h, o.sasm, stage == G2SC_VS, IF_DEPTH_DEFAULT, LOOP_DEPTH_DEFAULT, &nwords);
    rc = result_out(&o, words, nwords, out);
    if (rc) {
        char msg[sizeof out->error];
        copy_trunc(msg, sizeof msg, rc == -2 ? "uniform slots out of range" : "out of memory");
        g2sc_free(out);
        set_error(out, msg);
        sh_done(h);
        free(h);
        return -1;
    }
    out->remat = attempt == 2;
    out->oversize = over;
    sh_done(h);
    free(h);
    return 0;
}

/* ---------------------------------------------------------------- a link */
#define MAX_VARY_SLOTS 64                       /* as declared: Appendix A.7 packs what fits into 8 rows */

static const char *sym_name(const g2sc_sym *y) { return y->lname ? y->lname : y->name; }
static const char *sym_type(const g2sc_sym *y) { return y->ltype ? y->ltype : y->type; }
static int mat_rows(const char *t) { return !strncmp(t, "mat", 3) ? t[3] - '0' : 1; }
static int vary_slots(const g2sc_sym *y) { return mat_rows(sym_type(y)) * (y->has_array ? y->array : 1); }

static int rank_of(const char *t)
{
    if (!strcmp(t, "mat4")) return 0;
    if (!strcmp(t, "mat2")) return 1;
    if (strstr(t, "vec4")) return 2;
    if (!strcmp(t, "mat3")) return 3;
    if (strstr(t, "vec3")) return 4;
    if (strstr(t, "vec2")) return 5;
    return 6;
}

/* does a fragment program load this varying? The compiled text says: what the shader never uses is gone from it */
static int fs_reads(const char *sasm, int idx)
{
    const char *p = sasm;
    while ((p = strstr(p, "LDV")) != NULL) {                    /* LDV r1 3, or LDV.xy r1 3 */
        const char *q = strchr(p + 3, ' ');
        if (q) q = strchr(q + 1, ' ');
        if (q && atoi(q + 1) == idx) return 1;
        p += 3;
    }
    return 0;
}

typedef struct pk_var { int slot, rows, rank; } pk_var;

static const g2sc_struct *struct_of(const g2sc_result *r, const char *ty)
{
    int i;
    for (i = 0; i < r->nstructs; i++) if (!strcmp(r->structs[i].name, ty)) return &r->structs[i];
    return NULL;
}

/* the uniform slots a stage's code reads, from the text the compile left: bit 0 an LDP of the slot, bit 1 the first slot of an LDPX (a run-time index added) */
static void pk_loads(const char *sasm, unsigned char *ld, int n)
{
    const char *p = sasm;
    while (p && *p) {
        const char *e = strchr(p, '\n'), *end = e ? e : p + strlen(p), *q;
        int x = end - p > 4 && p[3] == 'X';
        if (end - p > 4 && !strncmp(p, "LDP", 3) && (p[3 + x] == ' ' || p[3 + x] == '.') && (q = memchr(p, ' ', (size_t)(end - p))) != NULL
            && (q = memchr(q + 1, ' ', (size_t)(end - q - 1))) != NULL && q + 1 < end && q[1] >= '0' && q[1] <= '9') {
            long v = strtol(q + 1, NULL, 10);
            if (v < n) ld[v] |= (unsigned char)(x ? 2 : 1);
        }
        p = e ? e + 1 : NULL;
    }
}

/* a struct goes member by member, an array of them element by element; one the code indexes at run time (an LDPX in it) stays a block, the index steps over whole
 * structs. What the code never reads takes no row (§2.10.4: not an active uniform), as a sampler in a struct: its slot is a texture unit */
static int pk_leaf(const g2sc_result *r, const char *ty, int slot, int count, pk_var *o, int n, const unsigned char *ld, int nld)
{
    const g2sc_struct *s = struct_of(r, ty);
    int i, k, rows;
    if (s && count) {
        for (k = slot; k < slot + s->nslots * count && k < nld && !(ld[k] & 2); k++) ;
        if (k < slot + s->nslots * count && k < nld) { o[n].slot = slot; o[n].rows = s->nslots * count; o[n].rank = 2; return n + 1; }
        for (k = 0; k < count; k++) n = pk_leaf(r, ty, slot + k * s->nslots, 0, o, n, ld, nld);
        return n;
    }
    if (s) {
        for (i = 0; i < s->nmembers; i++) n = pk_leaf(r, s->members[i].type, slot + s->members[i].offset, s->members[i].has_array ? s->members[i].array : 0, o, n, ld, nld);
        return n;
    }
    rows = mat_rows(ty) * (count ? count : 1);
    for (k = slot; k < slot + rows && k < nld && !ld[k]; k++) ;
    if (k == slot + rows || k >= nld) return n;
    o[n].slot = slot; o[n].rows = rows; o[n].rank = rank_of(ty);
    return n + 1;
}

/* place v: map[slot] = row * 4 + lane of every slot of every variable. Returns the rows in use, < 0 no fit */
static int pk_place(const pk_var *v, int n, int max_rows, int *map)
{
    int *rk = (int *)malloc(((size_t)n + 1) * sizeof(int) * 3), *rw = rk + n + 1, *at = rw + n + 1, i, k, rows = -1;
    if (!rk) return -1;
    for (i = 0; i < n; i++) { rk[i] = v[i].rank; rw[i] = v[i].rows; }
    rows = g2sc_pack_vars(rk, rw, n, max_rows, at);
    if (rows >= 0) for (i = 0; i < n; i++) for (k = 0; k < v[i].rows; k++) map[v[i].slot + k] = at[i] + 4 * k;
    free(rk);
    return rows;
}

/* the uniform slots of a stage, into room rows; the compiler's own and the built-ins stay whole rows, the draw fills them so. A slot the code never reads is -1 */
static int *pk_uniforms(const g2sc_result *r, int room, int *urows)
{
    pk_var *v = (pk_var *)malloc(((size_t)r->const_base + 1) * sizeof *v);
    int *map = (int *)malloc(((size_t)r->const_base + 1) * sizeof(int));
    unsigned char *ld = (unsigned char *)calloc((size_t)r->const_base + 1, 1);
    int i, n = 0;
    if (!v || !map || !ld) { free(v); free(map); free(ld); return NULL; }
    for (i = 0; i <= r->const_base; i++) map[i] = -1;
    pk_loads(r->sasm, ld, r->const_base);
    for (i = 0; i < r->nuniforms; i++) {
        const char *nm = sym_name(&r->uniforms[i]), *ty = sym_type(&r->uniforms[i]);
        int count = r->uniforms[i].has_array ? r->uniforms[i].array : 0;
        if (!strncmp(nm, "gl_", 3) || !strncmp(nm, "_g2_", 4) || !strncmp(nm, "g2", 2)) {
            const g2sc_struct *s = struct_of(r, ty);
            v[n].slot = r->uniforms[i].loc; v[n].rows = s ? s->nslots : mat_rows(ty) * (count ? count : 1); v[n].rank = 2; n++;
        } else n = pk_leaf(r, ty, r->uniforms[i].loc, count, v, n, ld, r->const_base);
    }
    *urows = room > 0 ? pk_place(v, n, room, map) : -1;
    free(v); free(ld);
    if (*urows < 0) { free(map); return NULL; }
    return map;
}

/* the map a packed compile takes: a slot the code never reads loads row 0, a load the optimizer removes (the first compile's code had none of it) */
static int *pk_for_compile(const int *map, int n)
{
    int *c, i;
    if (!map) return NULL;
    c = (int *)malloc(((size_t)n + 1) * sizeof(int));
    if (c) for (i = 0; i <= n; i++) c[i] = map[i] < 0 ? 0 : map[i];
    return c;
}

void g2sc_link_free(g2sc_linked *l)
{
    g2sc_free(&l->vs); g2sc_free(&l->fs);
    free(l->umap_vs); free(l->umap_fs);
    l->umap_vs = l->umap_fs = NULL;
}

static const g2sc_struct *find_struct(const g2sc_result *r, const char *name)
{
    int i;
    for (i = 0; i < r->nstructs; i++) if (!strcmp(r->structs[i].name, name)) return &r->structs[i];
    return NULL;
}

/* the name a struct was declared with: the compiler names one a scope hides "S@3" */
static size_t base_len(const char *n) { const char *at = strchr(n, '@'); return at ? (size_t)(at - n) : strlen(n); }

/* the same struct type in both stages: its name, then its members in order, each of one name, type, array size and precision */
static int same_struct(const g2sc_result *a, const char *ta, const g2sc_result *b, const char *tb, int depth)
{
    const g2sc_struct *sa = find_struct(a, ta), *sb = find_struct(b, tb);
    int m;
    if (!sa || !sb) return !sa && !sb && !strcmp(ta, tb);
    if (depth > 32 || base_len(ta) != base_len(tb) || strncmp(ta, tb, base_len(ta)) || sa->nmembers != sb->nmembers) return 0;
    for (m = 0; m < sa->nmembers; m++) {
        const g2sc_member *x = &sa->members[m], *y = &sb->members[m];
        if (strcmp(x->name, y->name) || x->has_array != y->has_array || x->array != y->array) return 0;
        if (!x->prec != !y->prec || (x->prec && strcmp(x->prec, y->prec))) return 0;
        if (!same_struct(a, x->type, b, y->type, depth + 1)) return 0;
    }
    return 1;
}

#define LINK_ERR(...) do { snprintf(o->error, sizeof o->error, __VA_ARGS__); goto fail; } while (0)
int g2sc_link(const char *vs, const char *fs, g2sc_linked *o) { return g2sc_link_bound(vs, fs, NULL, 0, 0, o); }
int g2sc_link_depth(const char *vs, const char *fs, uint32_t depth_units, g2sc_linked *o) { return g2sc_link_bound(vs, fs, NULL, 0, depth_units, o); }

int g2sc_link_bound(const char *vs, const char *fs, const g2sc_bind *binds, int nbinds, uint32_t depth_units, g2sc_linked *o)
{
    g2sc_pack first;
    const char *order[MAX_VARY_SLOTS];
    pk_var pv[MAX_VARY_SLOTS];
    int vmap[MAX_VARY_SLOTS], npv = 0, pack_v, pack_uv, pack_uf, urv = 0, urf = 0, nslots = 0, rows, nread = 0, i, k;
    uint32_t unread = 0, declared = 0;
    g2sc_result *rv = &o->vs, *rf = &o->fs;
    memset(o, 0, sizeof *o);
    if (!binds) nbinds = 0;
    if (compile_opts(vs, G2SC_VS, NULL, 0, 0, 0, 1, NULL, binds, nbinds, rv) < 0) LINK_ERR("vertex shader: %s", rv->error);
    for (i = 0; i < rv->nattribs; i++) {                    /* §2.10.4: the active attributes need a location each, a matrix one a column */
        const g2sc_sym *a = &rv->attribs[i];
        if (a->loc + mat_rows(sym_type(a)) <= G2SC_MAX_ATTRIBS) continue;
        for (k = nbinds - 1; k >= 0 && strcmp(binds[k].name, sym_name(a)); k--) ;
        if (k >= 0) LINK_ERR("attribute %s does not fit at location %d", sym_name(a), a->loc);
        LINK_ERR("too many attributes: the vertex stage has %d locations", G2SC_MAX_ATTRIBS);
    }
    memset(order, 0, sizeof order);
    for (i = 0; i < rv->nvaryings; i++) {
        int last = rv->varyings[i].loc + vary_slots(&rv->varyings[i]);
        if (last > MAX_VARY_SLOTS) LINK_ERR("too many varyings");
        order[rv->varyings[i].loc] = sym_name(&rv->varyings[i]);
        if (last > nslots) nslots = last;
    }
    /* the fragment stage takes the vertex stage's numbering: the hardware passes varyings by number */
    memset(&first, 0, sizeof first);
    first.depth_units = depth_units;
    if (g2sc_compile_packed(fs, G2SC_FS, order, nslots, &first, rf) < 0) LINK_ERR("fragment shader: %s", rf->error);
    for (i = 0; i < rv->nuniforms; i++) {                   /* a uniform of struct type in both stages: one struct (§4.2.6, §4.5.3) */
        const g2sc_sym *u = &rv->uniforms[i];
        for (k = 0; k < rf->nuniforms; k++) if (!strcmp(sym_name(u), sym_name(&rf->uniforms[k]))) break;
        if (k == rf->nuniforms || (!find_struct(rv, sym_type(u)) && !find_struct(rf, sym_type(&rf->uniforms[k])))) continue;
        if (u->array != rf->uniforms[k].array || !same_struct(rv, sym_type(u), rf, sym_type(&rf->uniforms[k]), 0))
            LINK_ERR("uniform %s has another struct in the fragment shader", sym_name(u));
    }
    pack_uv = rv->const_base + rv->nconst > 256; pack_uf = rf->const_base + rf->nconst > 64;
    pack_v = nslots > 8 || (rf->oversize && !pack_uf) || (rv->oversize && !pack_uv);
    for (i = 0; i < rf->nvaryings; i++) {
        const char *n = sym_name(&rf->varyings[i]);
        int s, n_s = vary_slots(&rf->varyings[i]), read = 0;
        for (s = 0; s < n_s; s++) read |= fs_reads(rf->sasm, rf->varyings[i].loc + s);
        if (!read) continue;                                    /* §2.10.3: only a varying the fragment stage uses needs a partner; declared and unused is legal (Cocos2d) */
        for (k = 0; k < rv->nvaryings; k++) if (!strcmp(n, sym_name(&rv->varyings[k]))) break;
        if (k == rv->nvaryings) LINK_ERR("varying %s is not written by the vertex shader", n);
        if (strcmp(sym_type(&rf->varyings[i]), sym_type(&rv->varyings[k])) || rf->varyings[i].array != rv->varyings[k].array) LINK_ERR("varying %s has another type in the vertex shader", n);
        pv[npv].slot = rf->varyings[i].loc; pv[npv].rows = n_s; pv[npv].rank = rank_of(sym_type(&rf->varyings[i])); npv++;
        for (s = 0; s < n_s; s++) if (fs_reads(rf->sasm, rf->varyings[i].loc + s)) { declared |= 1u << ((rf->varyings[i].loc + s) & 31); nread++; }
    }
    /* Appendix A.7 in the fewest rows, also when one a variable fits but takes more: two vec2 coordinates in one row are one LDV on board B and one row less for board A to interpolate */
    for (rows = 1; rows <= 8 && pk_place(pv, npv, rows, vmap) < 0; rows++) ;
    if (rows < nread) pack_v = 1;
    if (!pack_v) { o->fsmask = declared; unread = ((1u << nslots) - 1) & ~o->fsmask; }   /* what the fragment stage never reads is not worth computing */
    if (pack_v || pack_uv || pack_uf) {
        g2sc_pack pk;
        g2sc_result r2;
        int *cu_v, *cu_f, rc;
        memset(&pk, 0, sizeof pk);
        if (pack_v) {
            for (i = 0; i < MAX_VARY_SLOTS; i++) vmap[i] = -1;
            if (rows > 8 || pk_place(pv, npv, rows, vmap) < 0) LINK_ERR("the varyings do not pack into 8 vectors");
            for (i = 0; i < MAX_VARY_SLOTS; i++) if (vmap[i] >= 0) o->fsmask |= 1u << (vmap[i] >> 2);
            pk.vmap = vmap; pk.nvmap = MAX_VARY_SLOTS;
            o->vpacked = 1;
            memcpy(o->vmap, vmap, sizeof o->vmap);
        }
        if (pack_uv && !(o->umap_vs = pk_uniforms(rv, 256 - rv->nconst, &urv))) LINK_ERR("vertex stage: the uniforms do not pack into %d vectors", 256 - rv->nconst);
        if (pack_uf && !(o->umap_fs = pk_uniforms(rf, 64 - rf->nconst, &urf))) LINK_ERR("fragment stage: the uniforms do not pack into %d vectors", 64 - rf->nconst);
        cu_v = pk_for_compile(o->umap_vs, rv->const_base); cu_f = pk_for_compile(o->umap_fs, rf->const_base);
        if ((o->umap_vs && !cu_v) || (o->umap_fs && !cu_f)) { free(cu_v); free(cu_f); LINK_ERR("out of memory"); }
        pk.umap = cu_v; pk.urows = urv; pk.unread = unread;
        memset(&r2, 0, sizeof r2);
        rc = compile_opts(vs, G2SC_VS, NULL, 0, pk.unread, 0, 1, &pk, binds, nbinds, &r2);
        if (rc < 0) { snprintf(o->error, sizeof o->error, "vertex shader: %s", r2.error); g2sc_free(&r2); free(cu_v); free(cu_f); goto fail; }
        g2sc_free(rv); *rv = r2;
        pk.umap = cu_f; pk.urows = urf; pk.unread = 0; pk.depth_units = depth_units;
        memset(&r2, 0, sizeof r2);
        rc = g2sc_compile_packed(fs, G2SC_FS, order, nslots, &pk, &r2);
        free(cu_v); free(cu_f);
        if (rc < 0) { snprintf(o->error, sizeof o->error, "fragment shader: %s", r2.error); g2sc_free(&r2); goto fail; }
        g2sc_free(rf); *rf = r2;
    } else if (unread) {
        g2sc_result r2;
        memset(&r2, 0, sizeof r2);
        if (compile_opts(vs, G2SC_VS, NULL, 0, unread, 0, 0, NULL, binds, nbinds, &r2) == 0) { g2sc_free(rv); *rv = r2; }
        else g2sc_free(&r2);
    }
    if (rv->const_base + rv->nconst > 256) LINK_ERR("vertex stage: %d uniform and constant vectors, the hardware has 256", rv->const_base + rv->nconst);
    if (rf->const_base + rf->nconst > 64) LINK_ERR("fragment stage: %d uniform and constant vectors, the hardware has 64", rf->const_base + rf->nconst);
    return 0;
fail:
    g2sc_link_free(o);
    return -1;
}

int g2sc_assemble(const char *sasm, int vs, int max_if, int max_loop, uint64_t **words, int *nwords, char *error, int errsize)
{
    Shc *h;
    uint64_t *w, *copy;
    int n = 0;
    if (error && errsize > 0) error[0] = 0;
    *words = NULL; *nwords = 0;
    h = (Shc *)malloc(sizeof(Shc));
    if (!h) return -1;
    sh_init(h);
    if (setjmp(h->jb)) {
        if (error && errsize > 0) copy_trunc(error, (size_t)errsize, h->err[0] ? h->err : "assembly failed");
        sh_done(h);
        free(h);
        return -1;
    }
    w = sh_assemble(h, sasm, vs, max_if, max_loop, &n);
    copy = (uint64_t *)malloc(((size_t)n + 1) * sizeof(uint64_t));
    if (!copy) { sh_done(h); free(h); return -1; }
    if (n) memcpy(copy, w, (size_t)n * sizeof(uint64_t));
    *words = copy; *nwords = n;
    sh_done(h);
    free(h);
    return 0;
}
