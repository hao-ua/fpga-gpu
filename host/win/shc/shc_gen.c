/* shc_gen.c — glslc's class Codegen + class Compiler + _compile_stage's driver, method for method, for the production
 * configuration only: isa64=True, hw_sincos=True (the 32-bit branches and _poly_sin are not here).
 *
 * Python's dynamic typing is part of the behaviour: a value is a Reg, MatVal, Sampler, StructVal, ArrayVal or None, and
 * an operation on the wrong one raises (AttributeError, KeyError, IndexError, TypeError) deep inside. N(), TY() and the
 * checked accessors below raise at the same points. Evaluation order matters: alloc / kf / emit have effects, and C
 * leaves the order of function arguments open, so every nested Python call is a separate statement here. */
#include <string.h>
#include "shc_internal.h"

#define PI_180   0x1.1df46a2529d39p-6     /* math.pi / 180.0 */
#define D180_PI  0x1.ca5dc1a63c1f8p+5     /* 180.0 / math.pi */
#define INV_LN2  0x1.71547652b82fep+0     /* 1.0 / math.log(2) */
#define LN2      0x1.62e42fefa39efp-1     /* math.log(2) */
#define PI_2     0x1.921fb54442d18p+0     /* math.pi / 2 */
#define PI       0x1.921fb54442d18p+1     /* math.pi */

typedef enum { V_REG = 1, V_MAT, V_SAMPLER, V_STRUCT, V_ARRAY } VK;

typedef struct Val {
    int kind;
    const char *ty;
    union {
        struct { int n; int has_const; double cval; const double *cvec; int ncvec; } r;      /* cvec: a float vector's known components (glslc Reg.cvec), NULL = unknown */
        struct { struct Val **cols; int ncols; } m;
        struct { int unit; long long size; } s;      /* size < 0: not an array */
        struct { const char *sname; Dict *members; } st;
        struct { const char *ety; long long size; int local; long long base; struct Val **elems; struct Val *dyn; } a;     /* local: 0 uniform, 1 local, 2 varying; dyn: an int-repr register added to the slot at run time */
    } u;
} Val;

typedef struct Sym { long long slot; const char *ty; int has3; long long n; } Sym;
typedef struct Local { Val *v; const char *ty; } Local;
typedef struct LoopCtl { int hw; Val *brk, *cont; } LoopCtl;
typedef struct ScanScope { Dict names; struct ScanScope *up; } ScanScope;     /* the declarations in scope: name -> S_LDECL */
typedef struct CurRet { Val *retreg; const char *lret, *rt; Val *holder; int flow, depth, any; Val *left; } CurRet;      /* flow: flow_open's LOOP */
typedef struct Func { const char *rt; Param *params; int nparams; Node *body; } Func;
typedef struct StructDef { Member *m; int n; } StructDef;
typedef struct GConst { const char *ty; Node *init; } GConst;

/* consts + creg: tuple(4 floats) -> pool slot, pinned Reg */
typedef struct CEnt { double k[4]; int slot; Val *reg; int used; } CEnt;

typedef struct Cg {
    Shc *h;
    int fs, remat;
    Lines code;
    int vcount;
    unsigned char *sv; int svcap;
    CEnt *ct; int ctcap, ctn;
    double *pool; int npool, poolcap;
    Dict uniforms, samplers, attribs, varyings, const_env, funcs, inlining, structs, global_consts;     /* const_env: global scalar consts; a local's is in its scope, entry with p NULL */
    Dict shadow, vary_used;      /* the varyings and special outputs kept in a local until the end of main */
    Dict **locals; int nlocals, localcap;
    ScanScope *scan;
    LoopCtl *loopctl; int nloop, loopcap;
    CurRet *cur_ret;
    long long next_uniform;
    int next_sampler, next_attr, next_vary;
    Dict sampler_slot;           /* the uniform slot of a sampler in a uniform struct (as text) -> its unit in .i (struct_samplers) */
    Val *zero;
    int nozero;                  /* a declaration without a value is not zeroed (zero_val): the outputs' shadows */
    int label_n;
    int vs_tex;                  /* a vertex program samples a texture */
} Cg;

#define NONE_SZ (-0x7FFFFFFFFFFFFFFFLL)   /* Python's None for an array size */

static Val *eval(Cg *c, Node *e);
static void run_stmt(Cg *c, Node *s);
static Val *binop(Cg *c, const char *op, Node *ea, Node *eb);
static Val *bi_dot(Cg *c, Val *x, Val *y);
static void written(Cg *c, Node *e, Dict *out);
static Val *bi_floor(Cg *c, Val *x);
static Val *bi_abs(Cg *c, Val *x);
static Val *bi_step(Cg *c, Val *edge, Val *x);
static Val *bi_sign(Cg *c, Val *x);
static Val *bi_min(Cg *c, Val *x, Val *y);
static Val *bi_sin(Cg *c, Val *x);
static Val *bi_cos(Cg *c, Val *x);
static Val *bi_texture2D(Cg *c, Val *samp, Val *uv, Val *bias);
static void store(Cg *c, Node *lhs, Val *rv);
static Val *eval_lvalue(Cg *c, Node *e);
static int  fold_const(Cg *c, Node *e, double *out);
static Val *load_uniform_struct(Cg *c, long long base, const char *sname);
static Val *bi_all(Cg *c, Val *x);
static Val *setc(Cg *c, const char *cc, Val *x, Val *y);

/* ---------------- types ---------------- */
static int is_scalar(const char *t) { return !strcmp(t, "float") || !strcmp(t, "int") || !strcmp(t, "bool"); }

static int vec_n(const char *t)
{
    const char *p = t;
    if (*p == 'b' || *p == 'i') p++;
    if (p[0] == 'v' && p[1] == 'e' && p[2] == 'c' && p[3] >= '2' && p[3] <= '4' && !p[4]) return p[3] - '0';
    return 0;
}

static int mat_n(const char *t)
{
    if (t[0] == 'm' && t[1] == 'a' && t[2] == 't' && t[3] >= '2' && t[3] <= '4' && !t[4]) return t[3] - '0';
    return 0;
}

static int ncomp(const char *t) { int n; if (is_scalar(t)) return 1; n = vec_n(t); return n ? n : 4; }
static int ncomp2(Cg *c, const char *t) { int n = mat_n(t); if (!n) sh_py_error(c->h, "KeyError: not a matrix type"); return n; }

static const char *vecn(Cg *c, int n)
{
    switch (n) {
    case 2: return "vec2";
    case 3: return "vec3";
    case 4: return "vec4";
    default: return sh_fmt(c->h, "vec%d", n);
    }
}

static char xyzw(Cg *c, long long i)                  /* "xyzw"[i] */
{
    if (i < 0) i += 4;
    if (i < 0 || i >= 4) sh_py_error(c->h, "IndexError: string index out of range");
    return "xyzw"[i];
}

/* ---------------- values ---------------- */
static Val *new_val(Cg *c, int kind, const char *ty)
{
    Val *v = (Val *)sh_alloc(c->h, sizeof(Val));
    memset(v, 0, sizeof *v);
    v->kind = kind; v->ty = ty;
    return v;
}

static int is_reg(const Val *v) { return v && v->kind == V_REG; }
static int is_mat(const Val *v) { return v && v->kind == V_MAT; }
static int is_struct(const Val *v) { return v && v->kind == V_STRUCT; }
static int is_array(const Val *v) { return v && v->kind == V_ARRAY; }

static int N(Cg *c, const Val *v)                     /* x.n */
{
    if (!is_reg(v)) sh_py_error(c->h, "AttributeError: a value here has no register (.n)");
    return v->u.r.n;
}

static const char *TY(Cg *c, const Val *v)            /* x.ty */
{
    if (!v) sh_py_error(c->h, "AttributeError: 'NoneType' object has no attribute 'ty'");
    return v->ty;
}

static Val *col(Cg *c, const Val *m, long long j)     /* x.cols[j] */
{
    if (!is_mat(m)) sh_py_error(c->h, "AttributeError: a value here has no columns (.cols)");
    if (j < 0) j += m->u.m.ncols;
    if (j < 0 || j >= m->u.m.ncols) sh_py_error(c->h, "IndexError: matrix column out of range");
    return m->u.m.cols[j];
}

static Val *mk_mat(Cg *c, Val **cols, int n, const char *ty)
{
    Val *v = new_val(c, V_MAT, ty);
    v->u.m.cols = cols; v->u.m.ncols = n;
    return v;
}

static Val **new_vals(Cg *c, long long n) { return (Val **)sh_alloc(c->h, (size_t)(n > 0 ? n : 1) * sizeof(Val *)); }

static Val *mk_array(Cg *c, const char *ety, long long size, int local, long long base, Val **elems)
{
    Val *v = new_val(c, V_ARRAY, sh_fmt(c->h, "%s[%L]", ety, size));
    v->u.a.ety = ety; v->u.a.size = size; v->u.a.local = local; v->u.a.base = base; v->u.a.elems = elems; v->u.a.dyn = NULL;
    return v;
}

static Val *mk_sampler(Cg *c, int unit, const char *ty, long long size) { Val *v = new_val(c, V_SAMPLER, ty); v->u.s.unit = unit; v->u.s.size = size; return v; }

static Val *mk_struct(Cg *c, const char *sname, Dict *members)
{
    Val *v = new_val(c, V_STRUCT, sname);
    v->u.st.sname = sname; v->u.st.members = members;
    return v;
}

static Val *member_get(Cg *c, const Val *s, const char *name)   /* x.members[name] */
{
    DictEnt *e;
    if (!is_struct(s)) sh_py_error(c->h, "AttributeError: a value here has no members");
    e = dict_find(s->u.st.members, name);
    if (!e) sh_py_error(c->h, "KeyError: struct member");
    return (Val *)e->p;
}

/* ---------------- emit ---------------- */
static void emit(Cg *c, const char *fmt, ...)
{
    SB b; va_list ap;
    sb_init(&b, c->h, 40);
    va_start(ap, fmt); sb_vfmt(&b, fmt, ap); va_end(ap);
    lines_push(c->h, &c->code, line_parse(c->h, sb_str(&b)));
}

static const char *newlabel(Cg *c, const char *p) { c->label_n++; return sh_fmt(c->h, "%s%d", p, c->label_n); }

static Val *alloc(Cg *c, const char *ty);

/* Appendix A.7: a slot that shares its row: the row, then its own lanes down to .x. ix: LDPX's index register, or -1 */
static void load_at(Cg *c, const char *op, int d, long long at, int ix)
{
    long long row = at >> 2;
    int comp = (int)(at & 3), t = d, sc = d < c->svcap && c->sv[d];     /* a scalar's load is one lane: CSE must not take it for the row */
    if (comp || sc) t = alloc(c, "vec4")->u.r.n;
    if (ix >= 0) emit(c, "%s r%d %L r%d", op, t, row, ix);
    else emit(c, "%s r%d %L", op, t, row);
    if (sc) emit(c, "MOV r%d r%d.%c", d, t, "xyzw"[comp]);
    else if (comp) emit(c, "MOV r%d r%d.%s", d, t, comp == 1 ? "yzww" : comp == 2 ? "zwww" : "wwww");
}

static void emit_ldp(Cg *c, int d, long long slot)
{
    if (c->h->umap) load_at(c, "LDP", d, c->h->umap[slot], -1);
    else emit(c, "LDP r%d %L", d, slot);
}

static void emit_ldpx(Cg *c, int d, long long slot, int ix)
{
    if (c->h->umap) load_at(c, "LDPX", d, c->h->umap[slot], ix);
    else emit(c, "LDPX r%d %L r%d", d, slot, ix);
}

static void emit_ldv(Cg *c, int d, long long slot)
{
    if (c->fs && c->h->vmap) { long long at = slot < c->h->nvmap ? c->h->vmap[slot] : 0; load_at(c, "LDV", d, at < 0 ? 0 : at, -1); }
    else emit(c, "LDV r%d %L", d, slot);
}

static void sv_room(Cg *c, int n)
{
    if (n >= c->svcap) {
        int cap = c->svcap ? c->svcap * 2 : 1024;
        unsigned char *nn;
        while (n >= cap) cap *= 2;
        nn = (unsigned char *)sh_zalloc(c->h, (size_t)cap);
        if (c->svcap) memcpy(nn, c->sv, (size_t)c->svcap);
        c->sv = nn; c->svcap = cap;
    }
}

static void sv_add(Cg *c, int n) { sv_room(c, n); c->sv[n] = 1; }

static Val *alloc(Cg *c, const char *ty)
{
    Val *r = new_val(c, V_REG, ty);
    c->vcount++;
    if (is_scalar(ty)) sv_add(c, c->vcount);
    r->u.r.n = c->vcount;
    return r;
}

/* ---------------- constant pool ---------------- */
static unsigned key_hash(const double k[4])
{
    uint64_t h = 0x9E3779B97F4A7C15ULL;
    int i;
    for (i = 0; i < 4; i++) {
        uint64_t b = 0;
        if (k[i] != 0.0) memcpy(&b, &k[i], 8);    /* 0.0 == -0.0: one key */
        h = (h ^ b) * 0xFF51AFD7ED558CCDULL;
        h ^= h >> 32;
    }
    return (unsigned)h;
}

static int key_same(const double a[4], const double b[4])
{
    return sh_key_eq(a[0], b[0]) && sh_key_eq(a[1], b[1]) && sh_key_eq(a[2], b[2]) && sh_key_eq(a[3], b[3]);
}

static CEnt *ct_get(Cg *c, const double k[4])
{
    unsigned i;
    if (c->ctn * 2 >= c->ctcap) {
        int cap = c->ctcap ? c->ctcap * 2 : 256, j;
        CEnt *nt = (CEnt *)sh_zalloc(c->h, (size_t)cap * sizeof(CEnt));
        for (j = 0; j < c->ctcap; j++) {
            if (c->ct[j].used) {
                for (i = key_hash(c->ct[j].k) & (unsigned)(cap - 1); nt[i].used; i = (i + 1) & (unsigned)(cap - 1)) { }
                nt[i] = c->ct[j];
            }
        }
        c->ct = nt; c->ctcap = cap;
    }
    for (i = key_hash(k) & (unsigned)(c->ctcap - 1); c->ct[i].used; i = (i + 1) & (unsigned)(c->ctcap - 1))
        if (key_same(c->ct[i].k, k)) return &c->ct[i];
    memcpy(c->ct[i].k, k, sizeof(double) * 4);
    c->ct[i].used = 1; c->ct[i].slot = -1; c->ct[i].reg = NULL;
    c->ctn++;
    return &c->ct[i];
}

static int const_slot(Cg *c, const double k[4])
{
    CEnt *e = ct_get(c, k);
    if (e->slot < 0) {
        if (c->npool == c->poolcap) {
            int cap = c->poolcap ? c->poolcap * 2 : 64;
            double *np = (double *)sh_alloc(c->h, (size_t)cap * 4 * sizeof(double));
            if (c->npool) memcpy(np, c->pool, (size_t)c->npool * 4 * sizeof(double));
            c->pool = np; c->poolcap = cap;
        }
        memcpy(&c->pool[c->npool * 4], k, sizeof(double) * 4);
        e->slot = c->npool++;
    }
    return e->slot;
}

static Val *kload(Cg *c, double x, double y, double z, double w)
{
    double k[4];
    int slot;
    Val *r;
    k[0] = x; k[1] = y; k[2] = z; k[3] = w;
    if (!c->remat) {
        CEnt *e = ct_get(c, k);
        if (e->reg) return e->reg;
    }
    slot = const_slot(c, k);
    r = alloc(c, "vec4");
    emit(c, "LDP r%d @%d", r->u.r.n, slot);
    if (!c->remat) ct_get(c, k)->reg = r;
    return r;
}

static Val *kf(Cg *c, double f)
{
    Val *r = kload(c, f, f, f, f);
    r->ty = "float"; r->u.r.has_const = 1; r->u.r.cval = f;
    sv_add(c, r->u.r.n);
    return r;
}

static Val *zeroreg(Cg *c)
{
    if (!c->zero) c->zero = kf(c, 0.0);
    return c->zero;
}

/* A variable declared without a value starts at 0 (glslc _zero). GLSL ES leaves the read of one undefined; WebGL zeroes them and sources written there rely on it — here they
 * read whatever the register last held. One write per register; forward_opt drops it where the variable is written before it is read. A member that is itself an aggregate has no
 * registers until it is written (NULL). */
static void zero_val(Cg *c, Val *v)
{
    long long i;
    if (!v) return;
    switch (v->kind) {
    case V_MAT: for (i = 0; i < v->u.m.ncols; i++) zero_val(c, v->u.m.cols[i]); break;
    case V_STRUCT: for (i = 0; i < v->u.st.members->n; i++) if (v->u.st.members->ent[i].live) zero_val(c, (Val *)v->u.st.members->ent[i].p); break;
    case V_ARRAY: if (v->u.a.local == 1) for (i = 0; i < v->u.a.size; i++) zero_val(c, v->u.a.elems[i]); break;
    case V_REG: {
        Val *z = zeroreg(c);
        emit(c, "%s r%d r%d", !strcmp(v->ty, "int") ? "FTOI" : "MOV", v->u.r.n, z->u.r.n);       /* int: its own representation */
        break; }
    default: break;
    }
}

/* ---------------- int / float representation ---------------- */
static Val *asf(Cg *c, Val *x)
{
    if (is_reg(x) && !strcmp(x->ty, "int")) {
        Val *r = alloc(c, "float");
        emit(c, "ITOF r%d r%d", r->u.r.n, x->u.r.n);
        return r;
    }
    return x;
}

static Val *asi(Cg *c, Val *x)
{
    if (is_reg(x) && strcmp(x->ty, "int")) {
        Val *r = alloc(c, "int");
        emit(c, "FTOI r%d r%d", r->u.r.n, x->u.r.n);
        return r;
    }
    return x;
}

static Val *asi_lit(Cg *c, double val)
{
    Val *f = kf(c, val), *r = alloc(c, "int");
    emit(c, "FTOI r%d r%d", r->u.r.n, f->u.r.n);
    return r;
}

/* ---------------- swizzles ---------------- */
static int swz_index(Cg *c, char ch)
{
    const char *p;
    if ((p = strchr("xyzw", ch)) != NULL) return (int)(p - "xyzw");
    if ((p = strchr("rgba", ch)) != NULL) return (int)(p - "rgba");
    if ((p = strchr("stpq", ch)) != NULL) return (int)(p - "stpq");
    sh_py_error(c->h, "ValueError: substring not found (bad swizzle)");
}

static Val *extract(Cg *c, Val *v, long long comp)
{
    Val *r;
    v = asf(c, v);
    r = alloc(c, "float");
    emit(c, "MOV r%d r%d.%c", r->u.r.n, N(c, v), xyzw(c, comp));
    return r;
}

static const char *swz_src(Cg *c, Val *v, long long comp)
{
    Val *a = asf(c, v);
    return sh_fmt(c->h, "r%d.%c", N(c, a), xyzw(c, comp));
}

static Val *swizzle_read(Cg *c, Val *v, const char *swz)
{
    size_t n = strlen(swz), i;
    int *comps = (int *)sh_alloc(c->h, (n + 1) * sizeof(int)), natural = 1;
    const char *ty;
    Val *r;
    char *s;
    if (is_mat(v)) sh_compile_error(c->h, "cannot swizzle a matrix");
    v = asf(c, v);
    for (i = 0; i < n; i++) {
        comps[i] = swz_index(c, swz[i]);
        if (comps[i] != (int)i) natural = 0;
    }
    ty = n == 1 ? "float" : vecn(c, (int)n);
    if (n == 1) return extract(c, v, comps[0]);
    if (natural && ncomp(TY(c, v)) >= (int)n) {
        r = alloc(c, ty);
        emit(c, "MOV r%d r%d", r->u.r.n, N(c, v));
        return r;
    }
    r = alloc(c, ty);
    s = (char *)sh_alloc(c->h, n + 1);
    for (i = 0; i < n; i++) s[i] = "xyzw"[comps[i]];
    s[n] = 0;
    emit(c, "MOV r%d r%d.%s", r->u.r.n, N(c, v), s);
    return r;
}

/* ---------------- int_binop, masked_write ---------------- */
static Val *int_binop(Cg *c, const char *op, Val *a, Val *b)
{
    Val *r;
    if (!strcmp(op, "+") || !strcmp(op, "-")) {
        r = alloc(c, "int");
        emit(c, "%s r%d r%d r%d", op[0] == '+' ? "IADD" : "ISUB", r->u.r.n, N(c, a), N(c, b));
    } else {
        const char *mn;
        Val *x, *y;
        r = alloc(c, "bool");
        if (!strcmp(op, "<")) { mn = "ISLT"; x = a; y = b; }
        else if (!strcmp(op, ">")) { mn = "ISLT"; x = b; y = a; }
        else if (!strcmp(op, "<=")) { mn = "ISGE"; x = b; y = a; }
        else if (!strcmp(op, ">=")) { mn = "ISGE"; x = a; y = b; }
        else if (!strcmp(op, "==")) { mn = "ISEQ"; x = a; y = b; }
        else { mn = "ISNE"; x = a; y = b; }
        emit(c, "%s r%d r%d r%d", mn, r->u.r.n, N(c, x), N(c, y));
    }
    return r;
}

static void masked_write(Cg *c, Val *lhs, const char *mask, Val *rhs)
{
    size_t n = strlen(mask), i;
    int *comps = (int *)sh_alloc(c->h, (n + 1) * sizeof(int)), natural = 1, l;
    char *m = (char *)sh_alloc(c->h, n + 1), swz[5];
    const char *p;
    for (i = 0; i < n; i++) {
        if ((p = strchr("xyzw", mask[i])) != NULL) comps[i] = (int)(p - "xyzw");
        else if ((p = strchr("rgba", mask[i])) != NULL) comps[i] = (int)(p - "rgba");
        else if ((p = strchr("stpq", mask[i])) != NULL) comps[i] = (int)(p - "stpq");
        else sh_py_error(c->h, "ValueError: substring not found (bad write mask)");
        if (comps[i] != (int)i) natural = 0;
    }
    if (!n) { emit(c, "MOV r%d r%d", N(c, lhs), N(c, rhs)); return; }
    if (natural) {
        for (i = 0; i < n; i++) m[i] = "xyzw"[comps[i]];
        m[n] = 0;
        emit(c, "MOV.%s r%d r%d", m, N(c, lhs), N(c, rhs));
        return;
    }
    if (N(c, rhs) == N(c, lhs)) {
        Val *t = alloc(c, TY(c, rhs));
        emit(c, "MOV r%d r%d", t->u.r.n, N(c, rhs));
        rhs = t;
    }
    {                                         /* sorted(comps) keeps the duplicates */
        size_t k = 0;
        int v;
        for (v = 0; v < 4; v++)
            for (i = 0; i < n; i++) if (comps[i] == v) m[k++] = "xyzw"[v];
        m[k] = 0;
    }
    for (l = 0; l < 4; l++) {
        swz[l] = 'x';
        for (i = 0; i < n; i++) if (comps[i] == l) { swz[l] = "xyzw"[i < 4 ? i : 3]; if (i >= 4) xyzw(c, (long long)i); break; }
    }
    swz[4] = 0;
    emit(c, "MOV.%s r%d r%d.%s", m, N(c, lhs), N(c, rhs), swz);
}

/* ---------------- constructors ---------------- */
static Val *construct_mat(Cg *c, const char *ty, Val **args0, int na)
{
    Val **args = new_vals(c, na);
    int n = 0, i, j;
    for (i = 0; i < na; i++) args[i] = asf(c, args0[i]);
    n = ncomp2(c, ty);
    if (na == 1 && is_mat(args[0])) {                   /* from a matrix: its components where they exist, the identity elsewhere */
        Val *m = args[0], **cols = new_vals(c, n);
        int k = ncomp2(c, m->ty);
        for (j = 0; j < n; j++) {
            Val *cl = alloc(c, vecn(c, n));
            for (i = 0; i < n; i++) {
                const char *src;
                if (j < k && i < k) src = swz_src(c, m->u.m.cols[j], i);
                else { Val *q = kf(c, i == j ? 1.0 : 0.0); src = sh_fmt(c->h, "r%d", q->u.r.n); }
                emit(c, "MOV.%c r%d %s", "xyzw"[i], cl->u.r.n, src);
            }
            cols[j] = cl;
        }
        return mk_mat(c, cols, n, ty);
    }
    if (na == 1 && is_scalar(TY(c, args[0]))) {
        Val *f = args[0], *z = zeroreg(c), **cols = new_vals(c, n);
        for (j = 0; j < n; j++) {
            Val *cl = alloc(c, vecn(c, n));
            emit(c, "MOV r%d r%d", cl->u.r.n, N(c, z));
            emit(c, "MOV.%c r%d r%d", "xyzw"[j], cl->u.r.n, N(c, f));
            cols[j] = cl;
        }
        return mk_mat(c, cols, n, ty);
    }
    if (na == n) {
        int all = 1;
        for (i = 0; i < na; i++) if (!(is_reg(args[i]) && vec_n(args[i]->ty) && ncomp(args[i]->ty) == n)) { all = 0; break; }
        if (all) return mk_mat(c, args, n, ty);
    }
    {                                                   /* anything else: the components in order, column-major */
        const char **srcs = (const char **)sh_alloc(c->h, 64 * sizeof(char *) + (size_t)na * 16 * sizeof(char *));
        int ns = 0, q;
        for (i = 0; i < na; i++) {
            Val *a = args[i];
            if (is_mat(a)) {
                for (j = 0; j < a->u.m.ncols; j++)
                    for (q = 0; q < ncomp(a->u.m.cols[j]->ty); q++) srcs[ns++] = swz_src(c, a->u.m.cols[j], q);
            }
            else if (!is_reg(a)) sh_compile_error(c->h, "unsupported %s constructor (%d args)", ty, na);
            else if (is_scalar(a->ty)) srcs[ns++] = sh_fmt(c->h, "r%d", a->u.r.n);
            else for (q = 0; q < ncomp(a->ty); q++) srcs[ns++] = swz_src(c, a, q);
        }
        if (ns < n * n) sh_compile_error(c->h, "unsupported %s constructor (%d args)", ty, na);
        {
            Val **cols = new_vals(c, n);
            for (j = 0; j < n; j++) {
                Val *cl = alloc(c, vecn(c, n));
                for (i = 0; i < n; i++) emit(c, "MOV.%c r%d %s", "xyzw"[i], cl->u.r.n, srcs[j * n + i]);
                cols[j] = cl;
            }
            return mk_mat(c, cols, n, ty);
        }
    }
}

/* the basic type of a value's type name: float, int, bool */
static char basic(const char *t)
{
    if (!strcmp(t, "int") || (t[0] == 'i' && vec_n(t))) return 'i';
    if (!strcmp(t, "bool") || (t[0] == 'b' && vec_n(t))) return 'b';
    return (!strcmp(t, "float") || (t[0] == 'v' && vec_n(t)) || mat_n(t)) ? 'f' : 0;
}

/* ivecN / bvecN from values of another basic type: an int vector's lanes are integral floats, a bool's are 1.0 / 0.0 */
static Val *convert(Cg *c, Val *r, const char *ty, Val **args, int na)
{
    int i, anyf = 0, nonb = 0;
    for (i = 0; i < na; i++) {
        char b = basic(TY(c, args[i]));
        if (b == 'f') anyf = 1;
        if (b != 'b') nonb = 1;
    }
    if (ty[0] == 'i' && anyf) {
        Val *t = alloc(c, ty);
        emit(c, "TRUNC r%d r%d", t->u.r.n, r->u.r.n);
        return t;
    }
    if (ty[0] == 'b' && nonb) {
        Val *t = alloc(c, ty), *z = zeroreg(c);
        emit(c, "SET SNE r%d r%d r%d", t->u.r.n, r->u.r.n, z->u.r.n);
        return t;
    }
    return r;
}

/* glslc._cvec: the components of a float vector built from known values (constants, or vectors of them) */
static void set_cvec(Cg *c, Val *r, const char *ty, Val **args, int na)
{
    double *vals;
    int n = ncomp(ty), k = 0, i, j;
    if (strcmp(ty, "vec2") && strcmp(ty, "vec3") && strcmp(ty, "vec4")) return;
    vals = (double *)sh_alloc(c->h, (size_t)(na * 4 + n + 1) * sizeof(double));
    for (i = 0; i < na; i++) {
        Val *a = args[i];
        if (!is_reg(a)) return;
        if (is_scalar(a->ty)) {
            if (!a->u.r.has_const) return;
            vals[k++] = a->u.r.cval;
        }
        else if (a->u.r.cvec) { int an = ncomp(a->ty); for (j = 0; j < an && j < a->u.r.ncvec; j++) vals[k++] = a->u.r.cvec[j]; }
        else return;
    }
    if (na == 1 && is_scalar(args[0]->ty)) for (k = 1; k < n; k++) vals[k] = vals[0];
    if (k < n) return;
    r->u.r.cvec = vals; r->u.r.ncvec = n;
}

static Val *construct(Cg *c, const char *ty, Val **args0, int na0)
{
    Val **args, **orig, *r;
    int n, lane = 0, i, cc, na = 0;
    for (i = 0; i < na0; i++) na += is_mat(args0[i]) ? args0[i]->u.m.ncols : 1;
    args = new_vals(c, na); orig = new_vals(c, na);
    na = 0;
    for (i = 0; i < na0; i++) {                         /* a matrix goes in column by column */
        if (is_mat(args0[i])) { for (cc = 0; cc < args0[i]->u.m.ncols; cc++) orig[na++] = args0[i]->u.m.cols[cc]; }
        else orig[na++] = args0[i];
    }
    for (i = 0; i < na; i++) args[i] = asf(c, orig[i]);
    n = ncomp(ty);
    r = alloc(c, ty);
    if (na == 1 && is_scalar(TY(c, args[0]))) {
        emit(c, "MOV r%d r%d", r->u.r.n, N(c, args[0]));
        set_cvec(c, r, ty, args, na);
        return convert(c, r, ty, orig, na);
    }
    for (i = 0; i < na; i++) {
        Val *a = args[i];
        int an = ncomp(TY(c, a));
        if (an > 1 && lane == 0 && lane + an <= n) {
            emit(c, "MOV.%s r%d r%d", an == 2 ? "xy" : an == 3 ? "xyz" : "xyzw", r->u.r.n, N(c, a));
            lane += an;
            continue;
        }
        for (cc = 0; cc < an; cc++) {
            if (lane >= n) break;
            if (an > 1) {
                emit(c, "MOV.%c r%d r%d.%c", "xyzw"[lane], r->u.r.n, N(c, a), "xyzw"[cc]);
                lane++;
                continue;
            }
            emit(c, "MOV.%c r%d r%d", "xyzw"[lane], r->u.r.n, N(c, a));
            lane++;
        }
    }
    set_cvec(c, r, ty, args, na);
    return convert(c, r, ty, orig, na);
}

/* ---------------- lookup / load_var ---------------- */
static void scope_push(Cg *c, Dict *d)
{
    if (c->nlocals == c->localcap) {
        int cap = c->localcap ? c->localcap * 2 : 32;
        Dict **nl = (Dict **)sh_alloc(c->h, (size_t)cap * sizeof(Dict *));
        if (c->nlocals) memcpy(nl, c->locals, (size_t)c->nlocals * sizeof(Dict *));
        c->locals = nl; c->localcap = cap;
    }
    c->locals[c->nlocals++] = d;
}

static Dict *new_dict(Cg *c) { return (Dict *)sh_zalloc(c->h, sizeof(Dict)); }

static Local *local_find(Cg *c, const char *name)
{
    int i;
    for (i = c->nlocals - 1; i >= 0; i--) {
        DictEnt *e = dict_find(c->locals[i], name);
        if (e && e->p) return (Local *)e->p;
    }
    return NULL;
}

/* the entry tracking name as a compile-time scalar, unless a real local or a parameter hides it */
static DictEnt *const_home(Cg *c, const char *name)
{
    int i;
    for (i = c->nlocals - 1; i >= 0; i--) {
        DictEnt *e = dict_find(c->locals[i], name);
        if (e) return e->p ? NULL : e;
    }
    return dict_find(&c->const_env, name);
}

static void local_set(Cg *c, Dict *scope, const char *name, Val *v, const char *ty)
{
    Local *l = (Local *)sh_alloc(c->h, sizeof(Local));
    l->v = v; l->ty = ty;
    dict_put(c->h, scope, name)->p = l;
}

static Val *load_cols(Cg *c, const char *ty, long long slot)
{
    int n = ncomp2(c, ty), i;
    Val **cols = new_vals(c, n);
    for (i = 0; i < n; i++) cols[i] = alloc(c, vecn(c, n));
    for (i = 0; i < n; i++) emit_ldp(c, cols[i]->u.r.n, slot + i);
    return mk_mat(c, cols, n, ty);
}

/* a matrix attribute: one location per column */
static Val *load_attr_cols(Cg *c, const char *ty, long long slot)
{
    int n = ncomp2(c, ty), i;
    Val **cols = new_vals(c, n);
    for (i = 0; i < n; i++) cols[i] = alloc(c, vecn(c, n));
    for (i = 0; i < n; i++) emit_ldv(c, cols[i]->u.r.n, slot + i);
    return mk_mat(c, cols, n, ty);
}

static Val *load_var(Cg *c, const char *name)
{
    DictEnt *e;
    Local *l;
    Val *reg;
    if (!strcmp(name, "gl_FragCoord")) {
        if (!c->fs) sh_compile_error(c->h, "gl_FragCoord is only available in the fragment shader");
        reg = alloc(c, "vec4"); emit(c, "LDV r%d 8", reg->u.r.n);
        return reg;
    }
    if (!strcmp(name, "gl_FrontFacing")) {
        if (!c->fs) sh_compile_error(c->h, "gl_FrontFacing is only available in the fragment shader");
        reg = alloc(c, "vec4"); emit(c, "LDV r%d 9", reg->u.r.n);
        return extract(c, reg, 0);
    }
    if (!strcmp(name, "gl_PointCoord")) {
        if (!c->fs) sh_compile_error(c->h, "gl_PointCoord is only available in the fragment shader");
        reg = alloc(c, "vec2"); emit(c, "LDV r%d 10", reg->u.r.n);
        return reg;
    }
    if ((e = dict_find(&c->global_consts, name)) != NULL) {     /* materialize from its initializer, which sees the globals, not the caller's names */
        int i, hid = 0;
        for (i = 1; i < c->nlocals; i++) { DictEnt *x = dict_find(c->locals[i], name); if (x && x->p) hid = 1; }
        if (!hid) {
            int saved = c->nlocals;
            Val *v;
            c->nlocals = 1;
            v = eval(c, ((GConst *)e->p)->init);
            c->nlocals = saved;
            return v;
        }
    }
    /* lookup(): locals, samplers, uniforms, attribs, varyings */
    if ((l = local_find(c, name)) != NULL) return l->v;
    if ((e = dict_find(&c->samplers, name)) != NULL) return mk_sampler(c, ((Sym *)e->p)->slot, "sampler2D", ((Sym *)e->p)->has3 ? ((Sym *)e->p)->n : -1);
    if ((e = dict_find(&c->uniforms, name)) != NULL) {
        Sym *u = (Sym *)e->p;
        if (u->has3) return mk_array(c, u->ty, u->n, 0, u->slot, NULL);
        if (dict_find(&c->structs, u->ty)) return load_uniform_struct(c, u->slot, u->ty);
        if (mat_n(u->ty)) return load_cols(c, u->ty, u->slot);
        reg = alloc(c, u->ty);
        emit_ldp(c, reg->u.r.n, u->slot);
        if (!strcmp(u->ty, "int")) emit(c, "FTOI r%d r%d", reg->u.r.n, reg->u.r.n);
        return reg;
    }
    if ((e = dict_find(&c->attribs, name)) != NULL && mat_n(((Sym *)e->p)->ty)) return load_attr_cols(c, ((Sym *)e->p)->ty, ((Sym *)e->p)->slot);
    if (!dict_find(&c->attribs, name) && (e = dict_find(&c->varyings, name)) != NULL) {
        Sym *v = (Sym *)e->p;
        if (v->has3) return mk_array(c, v->ty, v->n, 2, v->slot, NULL);
        if (mat_n(v->ty)) return load_attr_cols(c, v->ty, v->slot);
    }
    if ((e = dict_find(&c->attribs, name)) != NULL || (e = dict_find(&c->varyings, name)) != NULL) {
        Sym *a = (Sym *)e->p;
        reg = alloc(c, a->ty);
        emit_ldv(c, reg->u.r.n, a->slot);
        if (!strcmp(a->ty, "int")) emit(c, "FTOI r%d r%d", reg->u.r.n, reg->u.r.n);
        return reg;
    }
    sh_compile_error(c->h, "undefined identifier '%s'", name);
}

/* ---------------- structs ---------------- */
static long long c_range(Cg *c, long long v)          /* uniform slots stay ints: a C limit Python does not have */
{
    if (v > 0x7FFFFFFFLL || v < -0x7FFFFFFFLL) sh_py_error(c->h, "uniform array size out of range");
    return v;
}

static long long py_int(Cg *c, double v)              /* int(float): saturated, every caller range-checks */
{
    double t;
    if (v != v) sh_py_error(c->h, "ValueError: cannot convert float NaN to integer");
    if (v - v != 0.0) sh_py_error(c->h, "OverflowError: cannot convert float infinity to integer");
    if (v >= 4e18) return 4000000000000000000LL;
    if (v <= -4e18) return -4000000000000000000LL;
    t = v;
    return (long long)t;
}

static long long arrsize(Cg *c, Node *ma)
{
    double d;
    if (!ma) return NONE_SZ;
    if (!fold_const(c, ma, &d)) sh_py_error(c->h, "TypeError: int() argument must be a number, not 'NoneType'");
    return c_range(c, py_int(c, d));
}

static long long type_nslots(Cg *c, const char *ty, long long asz)
{
    DictEnt *e = dict_find(&c->structs, ty);
    long long n;
    SH_ENTER(c->h, 2);
    if (e) {
        StructDef *s = (StructDef *)e->p;
        int i;
        n = 0;
        for (i = 0; i < s->n; i++) n += type_nslots(c, s->m[i].ty, arrsize(c, s->m[i].arr));
    }
    else n = mat_n(ty) ? mat_n(ty) : 1;
    SH_LEAVE(c->h, 2);
    return c_range(c, n * ((asz == NONE_SZ || asz == 0) ? 1 : asz));
}

static Val *load_uniform_member(Cg *c, long long slot, const char *ty, long long asz)
{
    Val *reg;
    if (!strcmp(ty, "sampler2D") || !strcmp(ty, "samplerCube") || !strcmp(ty, "sampler3D")) {   /* a sampler in a uniform struct: its unit */
        DictEnt *e = dict_find(&c->sampler_slot, sh_fmt(c->h, "%L", slot));
        if (!e) sh_py_error(c->h, "KeyError: a sampler slot");
        return mk_sampler(c, (int)e->i, "sampler2D", asz == NONE_SZ ? -1 : asz);
    }
    if (asz != NONE_SZ) return mk_array(c, ty, asz, 0, slot, NULL);
    if (dict_find(&c->structs, ty)) return load_uniform_struct(c, slot, ty);
    if (mat_n(ty)) return load_cols(c, ty, slot);
    reg = alloc(c, ty);
    emit_ldp(c, reg->u.r.n, slot);
    if (!strcmp(ty, "int")) emit(c, "FTOI r%d r%d", reg->u.r.n, reg->u.r.n);
    return reg;
}

static Val *load_uniform_struct(Cg *c, long long base, const char *sname)
{
    StructDef *s = (StructDef *)dict_find(&c->structs, sname)->p;
    Dict *members = new_dict(c);
    long long off = 0;
    int i;
    SH_ENTER(c->h, 2);
    for (i = 0; i < s->n; i++) {
        long long asz = arrsize(c, s->m[i].arr);
        Val *m = load_uniform_member(c, base + off, s->m[i].ty, asz);
        dict_put(c->h, members, s->m[i].name)->p = m;
        off += type_nslots(c, s->m[i].ty, asz);
    }
    SH_LEAVE(c->h, 2);
    return mk_struct(c, sname, members);
}

/* ---------------- run-time uniform indexing (ES2 Appendix A.5: a vertex shader indexes uniform arrays with any integer
 * expression; the 64-bit vertex core's LDPX rd, slot, rI loads params[(slot + rI) mod 256] per lane) ---------------- */
static Val *iscale(Cg *c, Val *r, long long n)          /* int-repr r x n, the element's stride in slots */
{
    Val *t, *u, *m;
    if (n == 1) return r;
    if (n == 2 || n == 3 || n == 4) {
        t = alloc(c, "int");
        emit(c, "IADD r%d r%d r%d", t->u.r.n, N(c, r), N(c, r));
        if (n == 2) return t;
        u = alloc(c, "int");
        emit(c, "IADD r%d r%d r%d", u->u.r.n, t->u.r.n, n == 4 ? t->u.r.n : N(c, r));
        return u;
    }
    m = asi_lit(c, (double)n);
    u = alloc(c, "int");
    emit(c, "IMUL r%d r%d r%d", u->u.r.n, N(c, r), m->u.r.n);
    return u;
}

/* as load_uniform_member, every slot offset by the run-time register dyn */
static Val *load_uniform_member_dyn(Cg *c, long long slot, const char *ty, long long asz, Val *dyn)
{
    DictEnt *e;
    Val *reg;
    if (asz != NONE_SZ) {                                       /* an array member: indexed later */
        Val *a = mk_array(c, ty, asz, 0, slot, NULL);
        a->u.a.dyn = dyn;
        return a;
    }
    e = dict_find(&c->structs, ty);
    if (e) {
        StructDef *s = (StructDef *)e->p;
        Dict *members = new_dict(c);
        long long off = 0;
        int i;
        SH_ENTER(c->h, 1);
        for (i = 0; i < s->n; i++) {
            long long a = arrsize(c, s->m[i].arr);
            Val *m = load_uniform_member_dyn(c, slot + off, s->m[i].ty, a, dyn);
            dict_put(c->h, members, s->m[i].name)->p = m;
            off += type_nslots(c, s->m[i].ty, a);
        }
        SH_LEAVE(c->h, 1);
        return mk_struct(c, ty, members);
    }
    if (mat_n(ty)) {
        int n = ncomp2(c, ty), i;
        Val **cols = new_vals(c, n);
        for (i = 0; i < n; i++) cols[i] = alloc(c, vecn(c, n));
        for (i = 0; i < n; i++) emit_ldpx(c, cols[i]->u.r.n, slot + i, N(c, dyn));
        return mk_mat(c, cols, n, ty);
    }
    reg = alloc(c, ty);
    emit_ldpx(c, reg->u.r.n, slot, N(c, dyn));
    if (!strcmp(ty, "int")) emit(c, "FTOI r%d r%d", reg->u.r.n, reg->u.r.n);      /* an int element is stored as a float */
    return reg;
}

/* the slot offset = index x element slots (+ the enclosing struct's run-time offset) in one int-repr register, the constant base in LDPX's immediate */
static Val *uniform_dyn_index(Cg *c, Val *base, Node *idx)
{
    Val *x = eval(c, idx);
    Val *ireg = asi(c, x), *off;                                /* FTOI truncates a float, as int() */
    long long nsl = type_nslots(c, base->u.a.ety, NONE_SZ);
    off = iscale(c, ireg, nsl);
    if (base->u.a.dyn) {
        Val *t = alloc(c, "int");
        emit(c, "IADD r%d r%d r%d", t->u.r.n, N(c, base->u.a.dyn), N(c, off));
        off = t;
    }
    return load_uniform_member_dyn(c, base->u.a.base, base->u.a.ety, NONE_SZ, off);
}

/* fresh registers of v's shape; a uniform array stays, it is read only */
static Val *like(Cg *c, Val *v)
{
    int i;
    if (!v) return NULL;
    if (is_mat(v)) {
        Val **cols = new_vals(c, v->u.m.ncols);
        for (i = 0; i < v->u.m.ncols; i++) cols[i] = like(c, v->u.m.cols[i]);
        return mk_mat(c, cols, v->u.m.ncols, v->ty);
    }
    if (is_struct(v)) {
        Dict *m = new_dict(c), *f = v->u.st.members;
        for (i = 0; i < f->n; i++) {
            Val *x;
            if (!f->ent[i].live) continue;
            x = like(c, (Val *)f->ent[i].p);
            dict_put(c->h, m, f->ent[i].key)->p = x;
        }
        return mk_struct(c, v->u.st.sname, m);
    }
    if (is_array(v) && v->u.a.local == 1) {
        Val **elems = new_vals(c, v->u.a.size);
        long long j;
        for (j = 0; j < v->u.a.size; j++) elems[j] = like(c, v->u.a.elems[j]);
        return mk_array(c, v->u.a.ety, v->u.a.size, 1, 0, elems);
    }
    if (is_reg(v)) return alloc(c, v->ty);
    return v;
}

static Val *decl_local_struct(Cg *c, const char *sname, Val *initval);

/* fresh registers of a type: an element of a local array */
static Val *fresh(Cg *c, const char *ty)
{
    if (mat_n(ty)) {
        int n = ncomp2(c, ty), i;
        Val **cols = new_vals(c, n);
        for (i = 0; i < n; i++) cols[i] = alloc(c, sh_fmt(c->h, "vec%d", n));
        return mk_mat(c, cols, n, ty);
    }
    if (dict_find(&c->structs, ty)) return decl_local_struct(c, ty, NULL);
    return alloc(c, ty);
}

/* element i of an array, at a constant index */
static Val *array_elem(Cg *c, Val *base, long long i)
{
    long long per;
    Val *reg;
    if (base->u.a.local == 1) return base->u.a.elems[i];
    if (base->u.a.local == 2) {
        if (mat_n(base->u.a.ety)) return load_attr_cols(c, base->u.a.ety, base->u.a.base + i * mat_n(base->u.a.ety));  /* one varying vector per column */
        reg = alloc(c, base->u.a.ety);
        emit_ldv(c, reg->u.r.n, base->u.a.base + i);
        return reg;
    }
    if (base->u.a.dyn) {                                        /* a constant index under a run-time struct offset */
        long long nsl = type_nslots(c, base->u.a.ety, NONE_SZ);
        return load_uniform_member_dyn(c, base->u.a.base + i * nsl, base->u.a.ety, NONE_SZ, base->u.a.dyn);
    }
    if (dict_find(&c->structs, base->u.a.ety)) {
        long long nsl = type_nslots(c, base->u.a.ety, NONE_SZ);
        return load_uniform_struct(c, base->u.a.base + i * nsl, base->u.a.ety);
    }
    per = mat_n(base->u.a.ety) ? mat_n(base->u.a.ety) : 1;
    if (mat_n(base->u.a.ety)) return load_cols(c, base->u.a.ety, base->u.a.base + i * per);
    reg = alloc(c, base->u.a.ety);
    emit_ldp(c, reg->u.r.n, base->u.a.base + i * per);
    if (!strcmp(base->u.a.ety, "int")) emit(c, "FTOI r%d r%d", reg->u.r.n, reg->u.r.n);      /* stored as a float */
    return reg;
}

/* a whole value, register by register: the lane mask holds and nothing is shared */
static void copy_val(Cg *c, Val *d, Val *s)
{
    int i;
    if (d == s || !s) return;
    SH_ENTER(c->h, 1);
    if (is_mat(d)) {
        if (!is_mat(s)) sh_py_error(c->h, "AttributeError: a value here has no columns (.cols)");
        for (i = 0; i < d->u.m.ncols && i < s->u.m.ncols; i++) copy_val(c, d->u.m.cols[i], s->u.m.cols[i]);
    } else if (is_struct(d)) {
        Dict *f = d->u.st.members;
        for (i = 0; i < f->n; i++) {
            Val *sm;
            if (!f->ent[i].live) continue;
            sm = member_get(c, s, f->ent[i].key);
            if (!f->ent[i].p) f->ent[i].p = like(c, sm);
            copy_val(c, (Val *)f->ent[i].p, sm);
        }
    } else if (is_array(d)) {
        long long j;
        if (!is_array(s)) sh_py_error(c->h, "AttributeError: a value here has no .kind");
        if (d->u.a.local != 1) sh_compile_error(c->h, "cannot copy a uniform array as a whole");
        for (j = 0; j < d->u.a.size && j < s->u.a.size; j++) copy_val(c, d->u.a.elems[j], array_elem(c, s, j));
    } else {
        s = !strcmp(TY(c, d), "int") ? asi(c, s) : asf(c, s);
        emit(c, "MOV r%d r%d", N(c, d), N(c, s));
    }
    SH_LEAVE(c->h, 1);
}

static Val *clone_val(Cg *c, Val *v)
{
    Val *d = like(c, v);
    copy_val(c, d, v);
    return d;
}

/* cn ? a : b of a matrix or a struct */
static Val *sel_val(Cg *c, Val *cn, Val *a, Val *b)
{
    int i;
    if (is_mat(a)) {
        int n;
        Val **cols;
        if (!is_mat(b)) sh_py_error(c->h, "AttributeError: a value here has no columns (.cols)");
        n = a->u.m.ncols < b->u.m.ncols ? a->u.m.ncols : b->u.m.ncols;
        cols = new_vals(c, n);
        for (i = 0; i < n; i++) cols[i] = sel_val(c, cn, a->u.m.cols[i], b->u.m.cols[i]);
        return mk_mat(c, cols, n, a->ty);
    }
    if (is_struct(a)) {
        Dict *m = new_dict(c), *f = a->u.st.members;
        for (i = 0; i < f->n; i++) {
            Val *x;
            if (!f->ent[i].live) continue;
            x = sel_val(c, cn, (Val *)f->ent[i].p, member_get(c, b, f->ent[i].key));
            dict_put(c->h, m, f->ent[i].key)->p = x;
        }
        return mk_struct(c, a->u.st.sname, m);
    }
    {
        Val *r = alloc(c, TY(c, a));
        emit(c, "SEL r%d r%d r%d r%d", r->u.r.n, N(c, cn), N(c, a), N(c, b));
        return r;
    }
}

static Val *decl_local_struct(Cg *c, const char *sname, Val *initval)
{
    StructDef *s = (StructDef *)dict_find(&c->structs, sname)->p;
    Dict *members = new_dict(c);
    int i;
    for (i = 0; i < s->n; i++) {
        Member *m = &s->m[i];
        Val *iv = initval ? member_get(c, initval, m->name) : NULL;
        if (m->arr || mat_n(m->ty) || dict_find(&c->structs, m->ty)) {
            Val *x = clone_val(c, iv);
            dict_put(c->h, members, m->name)->p = x;
        } else {
            Val *r = alloc(c, m->ty);
            if (iv) {
                iv = !strcmp(m->ty, "int") ? asi(c, iv) : asf(c, iv);
                emit(c, "MOV r%d r%d", r->u.r.n, N(c, iv));
            }
            dict_put(c->h, members, m->name)->p = r;
        }
    }
    return mk_struct(c, sname, members);
}

/* ---------------- fold_const ---------------- */
static int is_cast(const char *s) { return !strcmp(s, "float") || !strcmp(s, "int") || !strcmp(s, "bool"); }

static int get_const(void *ctx, const char *name, double *v, int *isint)
{
    DictEnt *d = const_home((Cg *)ctx, name);
    if (!d) return 0;
    *v = d->d; *isint = (int)d->i;
    return 1;
}

static int fold_const_i(Cg *c, Node *e, double *out, int *isint) { return sh_fold(c->h, e, get_const, c, out, isint); }
static int fold_const(Cg *c, Node *e, double *out) { int i; return fold_const_i(c, e, out, &i); }
/* ---------------- mat_mul, cmp ---------------- */
static Val *mat_mul(Cg *c, Val *a, Val *b)
{
    SH_ENTER(c->h, 1);
    if (is_mat(a) && is_mat(b)) {
        int n = ncomp2(c, a->ty), j;
        Val **cols = new_vals(c, n), *r;
        for (j = 0; j < n; j++) cols[j] = mat_mul(c, a, col(c, b, j));
        r = mk_mat(c, cols, n, a->ty);
        SH_LEAVE(c->h, 1);
        return r;
    }
    if (is_mat(a) && is_reg(b)) {
        int n = ncomp2(c, a->ty), j, first = 1;
        Val *acc = alloc(c, vecn(c, n));
        for (j = 0; j < n; j++) {
            const char *vj = swz_src(c, b, j);
            if (first) { emit(c, "MUL r%d r%d %s", acc->u.r.n, N(c, col(c, a, j)), vj); first = 0; }
            else {
                Val *t = alloc(c, vecn(c, n));
                emit(c, "MUL r%d r%d %s", t->u.r.n, N(c, col(c, a, j)), vj);
                emit(c, "ADD r%d r%d r%d", acc->u.r.n, acc->u.r.n, t->u.r.n);
            }
        }
        SH_LEAVE(c->h, 1);
        return acc;
    }
    if (is_reg(a) && is_mat(b)) {
        int n = ncomp2(c, b->ty), j;
        Val *r = alloc(c, vecn(c, n));
        for (j = 0; j < n; j++) {
            Val *dj = bi_dot(c, a, col(c, b, j));
            emit(c, "MOV.%c r%d r%d", "xyzw"[j], r->u.r.n, N(c, dj));
        }
        SH_LEAVE(c->h, 1);
        return r;
    }
    sh_compile_error(c->h, "only mat*vec / vec*mat supported for matrices");
}

static void cmp_(Cg *c, Val *r, Val *a, Val *b, const char *op)
{
    const char *cc;
    Val *x, *y;
    if (!strcmp(op, "<")) { cc = "SLT"; x = a; y = b; }
    else if (!strcmp(op, ">")) { cc = "SLT"; x = b; y = a; }
    else if (!strcmp(op, "<=")) { cc = "SGE"; x = b; y = a; }
    else if (!strcmp(op, ">=")) { cc = "SGE"; x = a; y = b; }
    else if (!strcmp(op, "==")) { cc = "SEQ"; x = a; y = b; }
    else { cc = "SNE"; x = a; y = b; }
    emit(c, "SET %s r%d r%d r%d", cc, r->u.r.n, N(c, x), N(c, y));
}

/* ---------------- builtins ---------------- */
static Val *un(Cg *c, const char *op, Val *x)
{
    Val *r = alloc(c, TY(c, x));
    emit(c, "%s r%d r%d", op, r->u.r.n, N(c, x));
    return r;
}

static Val *bi_abs(Cg *c, Val *x) { Val *r = alloc(c, TY(c, x)); emit(c, "MOV r%d !r%d", r->u.r.n, N(c, x)); return r; }
static Val *bi_floor(Cg *c, Val *x) { return un(c, "FLR", x); }

static Val *bi_fract(Cg *c, Val *x)
{
    Val *fl = bi_floor(c, x), *r = alloc(c, TY(c, x));
    emit(c, "SUB r%d r%d r%d", r->u.r.n, N(c, x), fl->u.r.n);
    return r;
}

static Val *bi_sign(Cg *c, Val *x)
{
    Val *z = zeroreg(c), *p, *n, *r;
    p = alloc(c, TY(c, x)); emit(c, "SET SLT r%d r%d r%d", p->u.r.n, z->u.r.n, N(c, x));
    n = alloc(c, TY(c, x)); emit(c, "SET SLT r%d r%d r%d", n->u.r.n, N(c, x), z->u.r.n);
    r = alloc(c, TY(c, x)); emit(c, "SUB r%d r%d r%d", r->u.r.n, p->u.r.n, n->u.r.n);
    return r;
}

static Val *bi_min(Cg *c, Val *x, Val *y) { Val *r = alloc(c, TY(c, x)); emit(c, "MIN r%d r%d r%d", r->u.r.n, N(c, x), N(c, y)); return r; }
static Val *bi_max(Cg *c, Val *x, Val *y) { Val *r = alloc(c, TY(c, x)); emit(c, "MAX r%d r%d r%d", r->u.r.n, N(c, x), N(c, y)); return r; }

static Val *setc(Cg *c, const char *cc, Val *x, Val *y)
{
    Val *r = alloc(c, TY(c, x));
    emit(c, "SET %s r%d r%d r%d", cc, r->u.r.n, N(c, x), N(c, y));
    return r;
}

static Val *bi_lessThan(Cg *c, Val *x, Val *y) { return setc(c, "SLT", x, y); }
static Val *bi_lessThanEqual(Cg *c, Val *x, Val *y) { return setc(c, "SGE", y, x); }
static Val *bi_greaterThan(Cg *c, Val *x, Val *y) { return setc(c, "SLT", y, x); }
static Val *bi_greaterThanEqual(Cg *c, Val *x, Val *y) { return setc(c, "SGE", x, y); }
static Val *bi_equal(Cg *c, Val *x, Val *y) { return setc(c, "SEQ", x, y); }
static Val *bi_notEqual(Cg *c, Val *x, Val *y) { return setc(c, "SNE", x, y); }

static Val *reduce_(Cg *c, const char *op, Val *x)
{
    int n = ncomp(TY(c, x)), i;
    Val *acc = extract(c, x, 0);
    for (i = 1; i < n; i++) {
        Val *e = extract(c, x, i), *t = alloc(c, "float");
        emit(c, "%s r%d r%d r%d", op, t->u.r.n, acc->u.r.n, e->u.r.n);
        acc = t;
    }
    return acc;
}

static Val *bi_any(Cg *c, Val *x) { return reduce_(c, "MAX", x); }
static Val *bi_all(Cg *c, Val *x) { return reduce_(c, "MIN", x); }

static Val *bi_not(Cg *c, Val *x)
{
    Val *one = kf(c, 1.0), *r = alloc(c, TY(c, x));
    emit(c, "SUB r%d r%d r%d", r->u.r.n, one->u.r.n, N(c, x));
    return r;
}

static Val *bi_clamp(Cg *c, Val *x, Val *lo, Val *hi)
{
    Val *t;
    if (c->h->xops & 2) {                                       /* board B's CLAMP (glslc.bi_clamp) */
        t = alloc(c, TY(c, x));
        emit(c, "CLAMP r%d r%d r%d r%d", t->u.r.n, N(c, x), N(c, lo), N(c, hi));
        return t;
    }
    t = bi_max(c, x, lo);
    return bi_min(c, t, hi);
}

static Val *builtin_mix(Cg *c, Val *x, Val *y, Val *a)
{
    Val *one = kf(c, 1.0), *ia = alloc(c, TY(c, a)), *t1, *t2, *r;
    emit(c, "SUB r%d r%d r%d", ia->u.r.n, one->u.r.n, N(c, a));
    t1 = alloc(c, TY(c, x)); emit(c, "MUL r%d r%d r%d", t1->u.r.n, N(c, x), ia->u.r.n);
    t2 = alloc(c, TY(c, y)); emit(c, "MUL r%d r%d r%d", t2->u.r.n, N(c, y), N(c, a));
    r = alloc(c, TY(c, x)); emit(c, "ADD r%d r%d r%d", r->u.r.n, t1->u.r.n, t2->u.r.n);
    return r;
}

static Val *bi_mix(Cg *c, Val *x, Val *y, Val *a) { return builtin_mix(c, x, y, a); }

static Val *bi_step(Cg *c, Val *edge, Val *x)
{
    Val *r = alloc(c, TY(c, x));
    emit(c, "SET SGE r%d r%d r%d", r->u.r.n, N(c, x), N(c, edge));
    return r;
}

static Val *reg_const(Cg *c, const Val *v, double *out)   /* x.const: Reg only; NULL = None */
{
    if (!is_reg(v)) sh_py_error(c->h, "AttributeError: a value here has no .const");
    if (!v->u.r.has_const) return NULL;
    *out = v->u.r.cval;
    return (Val *)v;
}

static Val *bi_smoothstep(Cg *c, Val *e0, Val *e1, Val *x)
{
    Val *z = zeroreg(c), *one = kf(c, 1.0), *t, *two, *three, *tt, *p, *r;
    double c0 = 0, c1 = 0;
    int h0 = reg_const(c, e0, &c0) != NULL;
    int h1 = h0 ? reg_const(c, e1, &c1) != NULL : 0;
    if (h0 && h1 && c1 != c0) {
        double k = sh_pf(c->h, 1.0 / (c1 - c0));
        double kb = sh_pf(c->h, -c0 * k);
        Val *kk = kf(c, k), *kbr = kf(c, kb);
        t = alloc(c, TY(c, x));
        emit(c, "MAD r%d r%d r%d r%d", t->u.r.n, N(c, x), kk->u.r.n, kbr->u.r.n);
    } else {
        Val *de = alloc(c, TY(c, e1)), *inv;
        emit(c, "SUB r%d r%d r%d", de->u.r.n, N(c, e1), N(c, e0));
        t = alloc(c, TY(c, x));
        emit(c, "SUB r%d r%d r%d", t->u.r.n, N(c, x), N(c, e0));
        inv = alloc(c, TY(c, e1));
        emit(c, "RCP r%d r%d", inv->u.r.n, de->u.r.n);
        emit(c, "MUL r%d r%d r%d", t->u.r.n, t->u.r.n, inv->u.r.n);
    }
    emit(c, "MAX r%d r%d r%d", t->u.r.n, t->u.r.n, z->u.r.n);
    emit(c, "MIN r%d r%d r%d", t->u.r.n, t->u.r.n, one->u.r.n);
    two = kf(c, 2.0); three = kf(c, 3.0);
    tt = alloc(c, TY(c, x)); emit(c, "MUL r%d r%d r%d", tt->u.r.n, t->u.r.n, t->u.r.n);
    p = alloc(c, TY(c, x)); emit(c, "MAD r%d r%d -r%d r%d", p->u.r.n, t->u.r.n, two->u.r.n, three->u.r.n);
    r = alloc(c, TY(c, x)); emit(c, "MUL r%d r%d r%d", r->u.r.n, tt->u.r.n, p->u.r.n);
    return r;
}

static Val *bi_mod(Cg *c, Val *x, Val *y)
{
    Val *q = alloc(c, TY(c, x)), *inv = alloc(c, TY(c, y)), *fl, *r;
    emit(c, "RCP r%d r%d", inv->u.r.n, N(c, y));
    emit(c, "MUL r%d r%d r%d", q->u.r.n, N(c, x), inv->u.r.n);
    fl = bi_floor(c, q);
    r = alloc(c, TY(c, x));
    emit(c, "MUL r%d r%d r%d", r->u.r.n, fl->u.r.n, N(c, y));
    emit(c, "SUB r%d r%d r%d", r->u.r.n, N(c, x), r->u.r.n);
    return r;
}

/* glslc._basis: k when v's first n components are known to be the unit vector e_k, else -1 */
static int basis_of(const Val *v, int n)
{
    int k, one = -1;
    if (!is_reg(v) || !v->u.r.cvec || v->u.r.ncvec < n) return -1;
    for (k = 0; k < n; k++) {
        if (v->u.r.cvec[k] == 1.0) { if (one >= 0) return -1; one = k; }
        else if (v->u.r.cvec[k] != 0.0) return -1;
    }
    return one;
}

static Val *bi_dot(Cg *c, Val *x, Val *y)
{
    int n = ncomp(TY(c, x)), k;
    Val *r;
    if (n > 1) {                                                /* dot(p, vec3(0.0, 1.0, 0.0)) is p.y (glslc.bi_dot) */
        if ((k = basis_of(y, n)) >= 0) return extract(c, x, k);
        if ((k = basis_of(x, n)) >= 0) return extract(c, y, k);
    }
    r = alloc(c, "float");
    if (n == 1) { emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, x), N(c, y)); return r; }     /* a float's lanes are all the one value: DP4 would count it four times */
    if (n == 3) { emit(c, "DP3 r%d r%d r%d", r->u.r.n, N(c, x), N(c, y)); return r; }
    if (n == 2) {
        Val *t = alloc(c, "float");
        emit(c, "MUL r%d r%d.x r%d.x", t->u.r.n, N(c, x), N(c, y));
        emit(c, "MAD r%d r%d.y r%d.y r%d", r->u.r.n, N(c, x), N(c, y), t->u.r.n);
        return r;
    }
    emit(c, "DP4 r%d r%d r%d", r->u.r.n, N(c, x), N(c, y));
    return r;
}

static Val *bi_length(Cg *c, Val *x)
{
    Val *d = bi_dot(c, x, x), *rs, *p, *r;
    if (c->h->xops & 1) { r = alloc(c, "float"); emit(c, "SQRT r%d r%d", r->u.r.n, d->u.r.n); return r; }      /* board B's SQRT (glslc.bi_length) */
    rs = alloc(c, "float"); emit(c, "RSQRT r%d r%d", rs->u.r.n, d->u.r.n);
    p = alloc(c, "float"); emit(c, "MUL r%d r%d r%d", p->u.r.n, d->u.r.n, rs->u.r.n);
    r = alloc(c, "float"); emit(c, "SEL r%d r%d r%d r%d", r->u.r.n, d->u.r.n, p->u.r.n, d->u.r.n);
    return r;
}

static Val *bi_distance(Cg *c, Val *x, Val *y)
{
    Val *t = alloc(c, TY(c, x));
    emit(c, "SUB r%d r%d r%d", t->u.r.n, N(c, x), N(c, y));
    return bi_length(c, t);
}

static Val *bi_sqrt(Cg *c, Val *x)
{
    Val *rs, *p, *r;
    if (c->h->xops & 1) { r = alloc(c, TY(c, x)); emit(c, "SQRT r%d r%d", r->u.r.n, N(c, x)); return r; }
    rs = alloc(c, TY(c, x));
    emit(c, "RSQRT r%d r%d", rs->u.r.n, N(c, x));
    p = alloc(c, TY(c, x)); emit(c, "MUL r%d r%d r%d", p->u.r.n, N(c, x), rs->u.r.n);
    r = alloc(c, TY(c, x)); emit(c, "SEL r%d r%d r%d r%d", r->u.r.n, N(c, x), p->u.r.n, N(c, x));
    return r;
}

static Val *bi_inversesqrt(Cg *c, Val *x) { return un(c, "RSQRT", x); }

static Val *bi_normalize(Cg *c, Val *x)
{
    Val *d = bi_dot(c, x, x), *rs = alloc(c, "float"), *r;
    emit(c, "RSQRT r%d r%d", rs->u.r.n, d->u.r.n);
    r = alloc(c, TY(c, x));
    emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, x), rs->u.r.n);
    return r;
}

static Val *bi_cross(Cg *c, Val *a, Val *b)
{
    Val *ayzx = swizzle_read(c, a, "yzx"), *bzxy = swizzle_read(c, b, "zxy");
    Val *azxy = swizzle_read(c, a, "zxy"), *byzx = swizzle_read(c, b, "yzx");
    Val *t1 = alloc(c, "vec3"), *t2, *r;
    emit(c, "MUL r%d r%d r%d", t1->u.r.n, ayzx->u.r.n, bzxy->u.r.n);
    t2 = alloc(c, "vec3"); emit(c, "MUL r%d r%d r%d", t2->u.r.n, azxy->u.r.n, byzx->u.r.n);
    r = alloc(c, "vec3"); emit(c, "SUB r%d r%d r%d", r->u.r.n, t1->u.r.n, t2->u.r.n);
    return r;
}

static Val *bi_reflect(Cg *c, Val *I, Val *Nn)
{
    Val *d = bi_dot(c, Nn, I), *two = kf(c, 2.0), *t = alloc(c, TY(c, I)), *r;
    emit(c, "MUL r%d r%d r%d", t->u.r.n, two->u.r.n, d->u.r.n);
    emit(c, "MUL r%d r%d r%d", t->u.r.n, t->u.r.n, N(c, Nn));
    r = alloc(c, TY(c, I));
    emit(c, "SUB r%d r%d r%d", r->u.r.n, N(c, I), t->u.r.n);
    return r;
}

static Val *bi_refract(Cg *c, Val *I, Val *Nn, Val *eta)
{
    Val *d = bi_dot(c, Nn, I), *dd, *one, *omdd, *e2, *t, *k, *kpos, *kc, *neg, *sq, *coef, *ei, *cn, *r, *z;
    dd = alloc(c, "float"); emit(c, "MUL r%d r%d r%d", dd->u.r.n, d->u.r.n, d->u.r.n);
    one = kf(c, 1.0); omdd = alloc(c, "float"); emit(c, "SUB r%d r%d r%d", omdd->u.r.n, one->u.r.n, dd->u.r.n);
    e2 = alloc(c, "float"); emit(c, "MUL r%d r%d r%d", e2->u.r.n, N(c, eta), N(c, eta));
    t = alloc(c, "float"); emit(c, "MUL r%d r%d r%d", t->u.r.n, e2->u.r.n, omdd->u.r.n);
    k = alloc(c, "float"); emit(c, "SUB r%d r%d r%d", k->u.r.n, one->u.r.n, t->u.r.n);
    kpos = alloc(c, "float"); z = zeroreg(c); emit(c, "SET SGE r%d r%d r%d", kpos->u.r.n, k->u.r.n, z->u.r.n);
    kc = alloc(c, "float"); z = zeroreg(c); emit(c, "MAX r%d r%d r%d", kc->u.r.n, k->u.r.n, z->u.r.n);
    neg = alloc(c, "float"); emit(c, "SUB r%d r%d r%d", neg->u.r.n, one->u.r.n, kpos->u.r.n);
    emit(c, "ADD r%d r%d r%d", kc->u.r.n, kc->u.r.n, neg->u.r.n);
    sq = bi_sqrt(c, kc);
    coef = alloc(c, "float"); emit(c, "MUL r%d r%d r%d", coef->u.r.n, N(c, eta), d->u.r.n);
    emit(c, "ADD r%d r%d r%d", coef->u.r.n, coef->u.r.n, sq->u.r.n);
    ei = alloc(c, TY(c, I)); emit(c, "MUL r%d r%d r%d", ei->u.r.n, N(c, eta), N(c, I));
    cn = alloc(c, TY(c, Nn)); emit(c, "MUL r%d r%d r%d", cn->u.r.n, coef->u.r.n, N(c, Nn));
    r = alloc(c, TY(c, I)); emit(c, "SUB r%d r%d r%d", r->u.r.n, ei->u.r.n, cn->u.r.n);
    emit(c, "MUL r%d r%d r%d", r->u.r.n, r->u.r.n, kpos->u.r.n);
    return r;
}

static Val *bi_matrixCompMult(Cg *c, Val *a, Val *b)
{
    int n = ncomp2(c, TY(c, a)), j;
    Val **cols = new_vals(c, n);
    for (j = 0; j < n; j++) {
        Val *cl = alloc(c, vecn(c, n));
        emit(c, "MUL r%d r%d r%d", cl->u.r.n, N(c, col(c, a, j)), N(c, col(c, b, j)));
        cols[j] = cl;
    }
    return mk_mat(c, cols, n, a->ty);
}

static Val *bi_radians(Cg *c, Val *x)
{
    Val *k = kf(c, PI_180), *r = alloc(c, TY(c, x));
    emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, x), k->u.r.n);
    return r;
}

static Val *bi_degrees(Cg *c, Val *x)
{
    Val *k = kf(c, D180_PI), *r = alloc(c, TY(c, x));
    emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, x), k->u.r.n);
    return r;
}

static Val *bi_exp2(Cg *c, Val *x) { return un(c, "EXP2", x); }
static Val *bi_log2(Cg *c, Val *x) { return un(c, "LOG2", x); }

static Val *bi_exp(Cg *c, Val *x)
{
    Val *k = kf(c, INV_LN2), *t = alloc(c, TY(c, x));
    emit(c, "MUL r%d r%d r%d", t->u.r.n, N(c, x), k->u.r.n);
    return un(c, "EXP2", t);
}

static Val *bi_log(Cg *c, Val *x)
{
    Val *l = un(c, "LOG2", x), *k = kf(c, LN2), *r = alloc(c, TY(c, x));
    emit(c, "MUL r%d r%d r%d", r->u.r.n, l->u.r.n, k->u.r.n);
    return r;
}

static Val *bi_powv(Cg *c, Val *x, Val *y)
{
    Val *l = un(c, "LOG2", x), *t = alloc(c, TY(c, x));
    emit(c, "MUL r%d r%d r%d", t->u.r.n, N(c, y), l->u.r.n);
    return un(c, "EXP2", t);
}

/* sin / cos without the SIN / COS unit (h->noops bit 1, glslc._sincos_lowered: its comment has the numbers): whole turns off in three parts (Cody-Waite),
 * the phase in quarter turns clamped to [-3, 3], folded to [-1, 1] by the triangle, then the odd polynomial. 17 instructions. */
static const double SIN_K2PI = 83443.0 / 131072.0;                                  /* 2/pi in FP24 */
static const double SIN_W[3] = { 25.0 / 4.0, 17.0 / 512.0, -76527.0 / 4294967296.0 };          /* C1 + C2 + C3 = 2pi */
static const double SIN_C[4] = { 102943.0 / 65536.0, -84661.0 / 131072.0, 20837.0 / 262144.0, -18297.0 / 4194304.0 };   /* c1, c3, c5, c7 */
static Val *sincos_lowered(Cg *c, Val *x, int cos)
{
    const char *ty;
    Val *u, *k4, *off, *f, *r, *w1, *w2, *w3, *k, *a, *one, *three, *two, *b, *cc, *s, *s2, *c1, *c3, *c5, *c7, *p, *y;
    x = asf(c, x);
    ty = TY(c, x);
    u = alloc(c, ty);
    k4 = kf(c, SIN_K2PI / 4.0);
    off = kf(c, cos ? 0.75 : 0.5);
    emit(c, "MAD r%d r%d r%d r%d", u->u.r.n, N(c, x), k4->u.r.n, off->u.r.n);
    f = un(c, "FLR", u);
    r = alloc(c, ty);
    w1 = kf(c, SIN_W[0]); w2 = kf(c, SIN_W[1]); w3 = kf(c, SIN_W[2]);
    emit(c, "MAD r%d r%d -r%d r%d", r->u.r.n, f->u.r.n, w1->u.r.n, N(c, x));
    emit(c, "MAD r%d r%d -r%d r%d", r->u.r.n, f->u.r.n, w2->u.r.n, r->u.r.n);
    emit(c, "MAD r%d r%d -r%d r%d", r->u.r.n, f->u.r.n, w3->u.r.n, r->u.r.n);
    k = kf(c, SIN_K2PI);
    a = alloc(c, ty);
    if (cos) { one = kf(c, 1.0); emit(c, "MAD r%d r%d r%d r%d", a->u.r.n, r->u.r.n, k->u.r.n, one->u.r.n); }
    else emit(c, "MUL r%d r%d r%d", a->u.r.n, r->u.r.n, k->u.r.n);
    three = kf(c, 3.0); two = kf(c, 2.0);
    emit(c, "MIN r%d r%d r%d", a->u.r.n, a->u.r.n, three->u.r.n);
    emit(c, "MAX r%d r%d -r%d", a->u.r.n, a->u.r.n, three->u.r.n);
    b = alloc(c, ty); emit(c, "SUB r%d r%d r%d", b->u.r.n, two->u.r.n, a->u.r.n);
    cc = alloc(c, ty); emit(c, "SUB r%d -r%d r%d", cc->u.r.n, two->u.r.n, a->u.r.n);
    s = alloc(c, ty);
    emit(c, "MIN r%d r%d r%d", s->u.r.n, a->u.r.n, b->u.r.n);
    emit(c, "MAX r%d r%d r%d", s->u.r.n, s->u.r.n, cc->u.r.n);
    s2 = alloc(c, ty); emit(c, "MUL r%d r%d r%d", s2->u.r.n, s->u.r.n, s->u.r.n);
    c1 = kf(c, SIN_C[0]); c3 = kf(c, SIN_C[1]); c5 = kf(c, SIN_C[2]); c7 = kf(c, SIN_C[3]);
    p = alloc(c, ty);
    emit(c, "MAD r%d r%d r%d r%d", p->u.r.n, s2->u.r.n, c7->u.r.n, c5->u.r.n);
    emit(c, "MAD r%d r%d r%d r%d", p->u.r.n, p->u.r.n, s2->u.r.n, c3->u.r.n);
    emit(c, "MAD r%d r%d r%d r%d", p->u.r.n, p->u.r.n, s2->u.r.n, c1->u.r.n);
    y = alloc(c, ty); emit(c, "MUL r%d r%d r%d", y->u.r.n, p->u.r.n, s->u.r.n);
    return y;
}

static Val *bi_sin(Cg *c, Val *x) { return (c->h->noops & 2) ? sincos_lowered(c, x, 0) : un(c, "SIN", x); }
static Val *bi_cos(Cg *c, Val *x) { return (c->h->noops & 2) ? sincos_lowered(c, x, 1) : un(c, "COS", x); }

/* a / b truncated toward zero without the divider (h->noops bit 0, glslc._idiv_lowered: its comment has the reasoning): registers an, bn of type ty (int, or
 * ivecN in int representation), with only the integer ops the cores have (no IOP). |a|, |b| by compare and select, two float estimates with the exact remainder
 * after each (the second floored after taking 1/2 off), one correction, the sign by a select. 27 instructions, exact over the 24-bit range. */
static Val *idiv_lowered(Cg *c, int an, int bn, const char *ty)
{
    const char *fty = !strcmp(ty, "int") ? "float" : ty[4] == '2' ? "vec2" : ty[4] == '3' ? "vec3" : "vec4";
    Val *z, *na, *nb, *A, *B, *fa, *fb, *rb, *t, *q, *p, *r, *fq, *half, *t2, *d, *cc, *s;
    z = zeroreg(c);
    na = alloc(c, fty); emit(c, "ISLT r%d r%d r%d", na->u.r.n, an, z->u.r.n);
    nb = alloc(c, fty); emit(c, "ISLT r%d r%d r%d", nb->u.r.n, bn, z->u.r.n);
    A = alloc(c, ty);
    emit(c, "ISUB r%d r%d r%d", A->u.r.n, z->u.r.n, an);
    emit(c, "SEL r%d r%d r%d r%d", A->u.r.n, na->u.r.n, A->u.r.n, an);
    B = alloc(c, ty);
    emit(c, "ISUB r%d r%d r%d", B->u.r.n, z->u.r.n, bn);
    emit(c, "SEL r%d r%d r%d r%d", B->u.r.n, nb->u.r.n, B->u.r.n, bn);
    fa = alloc(c, fty); emit(c, "UTOF r%d r%d", fa->u.r.n, A->u.r.n);
    fb = alloc(c, fty); emit(c, "UTOF r%d r%d", fb->u.r.n, B->u.r.n);
    rb = alloc(c, fty); emit(c, "RCP r%d r%d", rb->u.r.n, fb->u.r.n);
    t = alloc(c, fty); emit(c, "MUL r%d r%d r%d", t->u.r.n, fa->u.r.n, rb->u.r.n);
    q = alloc(c, ty); emit(c, "FTOU r%d r%d", q->u.r.n, t->u.r.n);
    p = alloc(c, ty); emit(c, "IMUL r%d r%d r%d", p->u.r.n, q->u.r.n, B->u.r.n);
    r = alloc(c, ty); emit(c, "ISUB r%d r%d r%d", r->u.r.n, A->u.r.n, p->u.r.n);
    fq = alloc(c, fty); emit(c, "ITOF r%d r%d", fq->u.r.n, r->u.r.n);
    half = kf(c, 0.5);
    t2 = alloc(c, fty);
    emit(c, "MAD r%d r%d r%d -r%d", t2->u.r.n, fq->u.r.n, rb->u.r.n, half->u.r.n);
    emit(c, "FLR r%d r%d", t2->u.r.n, t2->u.r.n);
    d = alloc(c, ty); emit(c, "FTOI r%d r%d", d->u.r.n, t2->u.r.n);
    emit(c, "IADD r%d r%d r%d", q->u.r.n, q->u.r.n, d->u.r.n);
    emit(c, "IMUL r%d r%d r%d", p->u.r.n, d->u.r.n, B->u.r.n);
    emit(c, "ISUB r%d r%d r%d", r->u.r.n, r->u.r.n, p->u.r.n);
    emit(c, "ISUB r%d r%d r%d", r->u.r.n, r->u.r.n, B->u.r.n);
    cc = alloc(c, fty); emit(c, "ISGE r%d r%d r%d", cc->u.r.n, r->u.r.n, z->u.r.n);
    emit(c, "FTOI r%d r%d", d->u.r.n, cc->u.r.n);
    emit(c, "IADD r%d r%d r%d", q->u.r.n, q->u.r.n, d->u.r.n);
    s = alloc(c, fty); emit(c, "SET SNE r%d r%d r%d", s->u.r.n, na->u.r.n, nb->u.r.n);
    emit(c, "ISUB r%d r%d r%d", p->u.r.n, z->u.r.n, q->u.r.n);
    emit(c, "SEL r%d r%d r%d r%d", q->u.r.n, s->u.r.n, p->u.r.n, q->u.r.n);
    return q;
}

static Val *atan1(Cg *c, Val *x)
{
    static const double CF[4] = { -0.0851330, 0.1801410, -0.3302995, 0.9998660 };
    Val *ax = bi_abs(c, x), *axe = alloc(c, TY(c, x)), *kk, *inv, *z, *z2, *p, *at, *hp, *cmp, *full, *sg, *r, *st, *k1;
    int i;
    kk = kf(c, 1e-6);
    emit(c, "ADD r%d r%d r%d", axe->u.r.n, ax->u.r.n, kk->u.r.n);
    inv = alloc(c, TY(c, x)); emit(c, "RCP r%d r%d", inv->u.r.n, axe->u.r.n);
    z = bi_min(c, ax, inv);
    z2 = alloc(c, TY(c, x)); emit(c, "MUL r%d r%d r%d", z2->u.r.n, z->u.r.n, z->u.r.n);
    p = kf(c, 0.0208351);
    for (i = 0; i < 4; i++) {
        Val *t = alloc(c, TY(c, x)), *kc;
        emit(c, "MUL r%d r%d r%d", t->u.r.n, p->u.r.n, z2->u.r.n);
        p = alloc(c, TY(c, x));
        kc = kf(c, CF[i]);
        emit(c, "ADD r%d r%d r%d", p->u.r.n, t->u.r.n, kc->u.r.n);
    }
    at = alloc(c, TY(c, x)); emit(c, "MUL r%d r%d r%d", at->u.r.n, z->u.r.n, p->u.r.n);
    hp = kf(c, PI_2);
    cmp = alloc(c, TY(c, x)); emit(c, "SUB r%d r%d r%d", cmp->u.r.n, hp->u.r.n, at->u.r.n);
    k1 = kf(c, 1.0);
    st = bi_step(c, k1, ax);
    full = builtin_mix(c, at, cmp, st);
    sg = bi_sign(c, x);
    r = alloc(c, TY(c, x)); emit(c, "MUL r%d r%d r%d", r->u.r.n, full->u.r.n, sg->u.r.n);
    return r;
}

static Val *atan64(Cg *c, Val **a, int na)
{
    static const double CF[4] = { -0.0851330, 0.1801410, -0.3302995, 0.9998660 };
    Val *y, *x, *ay, *ax, *mn, *mx, *inv, *r, *s, *p, *at, *z, *c1, *t1, *a1, *a2, *c3, *t3, *res, *kk;
    int i;
    if (na == 1) {
        y = a[0]; x = kf(c, 1.0);
        if (strcmp(TY(c, y), "float")) {        /* a 1 of y's shape: what is worked out from x takes its type (min, max, SET) */
            Val *one = alloc(c, TY(c, y));
            emit(c, "MOV r%d r%d", one->u.r.n, x->u.r.n);
            x = one;
        }
    }
    else {
        if (na != 2) sh_py_error(c->h, "ValueError: wrong number of arguments to atan");
        y = a[0]; x = a[1];
    }
    ay = bi_abs(c, y); ax = bi_abs(c, x);
    mn = bi_min(c, ax, ay); mx = bi_max(c, ax, ay);
    inv = alloc(c, TY(c, y)); emit(c, "RCP r%d r%d", inv->u.r.n, mx->u.r.n);
    r = alloc(c, TY(c, y)); emit(c, "MUL r%d r%d r%d", r->u.r.n, mn->u.r.n, inv->u.r.n);
    s = alloc(c, TY(c, y)); emit(c, "MUL r%d r%d r%d", s->u.r.n, r->u.r.n, r->u.r.n);
    p = kf(c, 0.0208351);
    for (i = 0; i < 4; i++) {
        Val *q = alloc(c, TY(c, y));
        kk = kf(c, CF[i]);
        emit(c, "MAD r%d r%d r%d r%d", q->u.r.n, p->u.r.n, s->u.r.n, kk->u.r.n);
        p = q;
    }
    at = alloc(c, TY(c, y)); emit(c, "MUL r%d r%d r%d", at->u.r.n, p->u.r.n, r->u.r.n);
    z = zeroreg(c);
    c1 = setc(c, "SLT", ax, ay);
    t1 = alloc(c, TY(c, y));
    kk = kf(c, PI_2);
    emit(c, "SUB r%d r%d r%d", t1->u.r.n, kk->u.r.n, at->u.r.n);
    a1 = alloc(c, TY(c, y)); emit(c, "SEL r%d r%d r%d r%d", a1->u.r.n, c1->u.r.n, t1->u.r.n, at->u.r.n);
    if (na == 1) a2 = a1;
    else {
        Val *c2 = setc(c, "SLT", x, z), *t2 = alloc(c, TY(c, y));
        kk = kf(c, PI);
        emit(c, "SUB r%d r%d r%d", t2->u.r.n, kk->u.r.n, a1->u.r.n);
        a2 = alloc(c, TY(c, y)); emit(c, "SEL r%d r%d r%d r%d", a2->u.r.n, c2->u.r.n, t2->u.r.n, a1->u.r.n);
    }
    c3 = setc(c, "SLT", y, z);
    t3 = alloc(c, TY(c, y)); emit(c, "SUB r%d r%d r%d", t3->u.r.n, z->u.r.n, a2->u.r.n);
    res = alloc(c, TY(c, y)); emit(c, "SEL r%d r%d r%d r%d", res->u.r.n, c3->u.r.n, t3->u.r.n, a2->u.r.n);
    return res;
}

static Val *bi_asin(Cg *c, Val *x)
{
    Val *x2 = alloc(c, TY(c, x)), *d, *k, *rs, *t;
    emit(c, "MUL r%d r%d r%d", x2->u.r.n, N(c, x), N(c, x));
    d = alloc(c, TY(c, x));
    k = kf(c, 1.0);
    emit(c, "SUB r%d r%d r%d", d->u.r.n, k->u.r.n, x2->u.r.n);
    k = kf(c, 1e-6);
    emit(c, "ADD r%d r%d r%d", d->u.r.n, d->u.r.n, k->u.r.n);
    rs = alloc(c, TY(c, x)); emit(c, "RSQRT r%d r%d", rs->u.r.n, d->u.r.n);
    t = alloc(c, TY(c, x)); emit(c, "MUL r%d r%d r%d", t->u.r.n, N(c, x), rs->u.r.n);
    return atan1(c, t);
}

static Val *bi_acos(Cg *c, Val *x)
{
    Val *a = bi_asin(c, x), *r = alloc(c, TY(c, x)), *k = kf(c, PI_2);
    emit(c, "SUB r%d r%d r%d", r->u.r.n, k->u.r.n, a->u.r.n);
    return r;
}

static Val *bi_ceil(Cg *c, Val *x) { return un(c, "CEIL", x); }

static Val *bi_faceforward(Cg *c, Val *Nn, Val *I, Val *Nref)
{
    Val *d = bi_dot(c, Nref, I), *z = zeroreg(c), *s = alloc(c, "float"), *two, *m, *one, *r;
    emit(c, "SET SLT r%d r%d r%d", s->u.r.n, d->u.r.n, z->u.r.n);
    two = kf(c, 2.0); m = alloc(c, "float"); emit(c, "MUL r%d r%d r%d", m->u.r.n, two->u.r.n, s->u.r.n);
    one = kf(c, 1.0); emit(c, "SUB r%d r%d r%d", m->u.r.n, m->u.r.n, one->u.r.n);
    r = alloc(c, TY(c, Nn)); emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, Nn), m->u.r.n);
    return r;
}

static Val *tex1(Cg *c, Val *samp, Val *coord, const char *mode, Val *w, int cube)
{
    int unit = (samp && samp->kind == V_SAMPLER) ? samp->u.s.unit : 0;
    Val *r = alloc(c, "vec4");
    if (!c->fs) {                                       /* MAX_VERTEX_TEXTURE_IMAGE_UNITS is 0: legal to compile, the link refuses it */
        Val *z = zeroreg(c);
        c->vs_tex = 1;
        emit(c, "MOV r%d r%d", r->u.r.n, z->u.r.n);
        return r;
    }
    if (mode) {
        Val *t = alloc(c, "vec4");
        emit(c, "MOV r%d r%d", t->u.r.n, N(c, coord));
        emit(c, "MOV.w r%d r%d", t->u.r.n, N(c, w));
        if (cube) emit(c, "TEX r%d r%d #%d %s", r->u.r.n, t->u.r.n, unit, mode);
        else emit(c, "TEX r%d r%d #%d %s .xyw", r->u.r.n, t->u.r.n, unit, mode);
        return r;
    }
    emit(c, "TEX r%d r%d #%d %s", r->u.r.n, N(c, coord), unit, cube ? ".xyz" : ".xy");
    return r;
}

static Val *tex(Cg *c, Val *samp, Val *coord, const char *mode, Val *w, int cube)
{
    Val *r = tex1(c, samp, coord, mode, w, cube), *g, *b, *a, *z, *t, *o, *d;
    int unit = (samp && samp->kind == V_SAMPLER) ? samp->u.s.unit : 0;
    if (!c->fs || !(c->h->depth_units >> unit & 1)) return r;
    /* the TMU reads the D24S8 word as RGBA bytes: R the stencil, then the depth, high byte first. All ones must come to 1 whatever the additions round */
    g = extract(c, r, 1); b = extract(c, r, 2); a = extract(c, r, 3);
    z = alloc(c, "float"); emit(c, "MUL r%d r%d r%d", z->u.r.n, g->u.r.n, kf(c, 0.99609375)->u.r.n);
    t = alloc(c, "float"); emit(c, "MUL r%d r%d r%d", t->u.r.n, b->u.r.n, kf(c, 0.003891)->u.r.n); emit(c, "ADD r%d r%d r%d", z->u.r.n, z->u.r.n, t->u.r.n);
    emit(c, "MUL r%d r%d r%d", t->u.r.n, a->u.r.n, kf(c, 0.000016)->u.r.n); emit(c, "ADD r%d r%d r%d", z->u.r.n, z->u.r.n, t->u.r.n);
    o = bi_min(c, z, kf(c, 1.0));
    d = alloc(c, "vec4"); emit(c, "MOV r%d r%d", d->u.r.n, o->u.r.n); emit(c, "MOV.w r%d r%d", d->u.r.n, kf(c, 1.0)->u.r.n);
    return d;
}

static Val *bi_textureCube(Cg *c, Val *samp, Val *dir, Val *bias) { return tex(c, samp, dir, bias ? "BIAS" : NULL, bias, 1); }

static Val *bi_texture2D(Cg *c, Val *samp, Val *uv, Val *bias) { return tex(c, samp, uv, bias ? "BIAS" : NULL, bias, 0); }

static Val *bi_texture2DProj(Cg *c, Val *samp, Val *uv, Val *bias)
{
    Val *q = extract(c, uv, ncomp(TY(c, uv)) - 1), *inv = alloc(c, "float"), *st;
    emit(c, "RCP r%d r%d", inv->u.r.n, q->u.r.n);
    st = alloc(c, "vec2");
    emit(c, "MUL r%d r%d r%d", st->u.r.n, N(c, uv), inv->u.r.n);
    return bi_texture2D(c, samp, st, bias);
}

/* uv.xy (ty vec3: .xyz) / q, q the last component */
static Val *proj(Cg *c, Val *uv, const char *ty)
{
    Val *q = extract(c, uv, ncomp(TY(c, uv)) - 1), *inv = alloc(c, "float"), *st;
    emit(c, "RCP r%d r%d", inv->u.r.n, q->u.r.n);
    st = alloc(c, ty);
    emit(c, "MUL r%d r%d r%d", st->u.r.n, N(c, uv), inv->u.r.n);
    return st;
}

static Sym *sym_put(Cg *c, Dict *d, const char *name, long long slot, const char *ty, int has3, long long n);

/* (width, height) of the texture on this unit: a uniform the library fills, as the TMU takes a level, not gradients */
static Val *texsize(Cg *c, Val *samp)
{
    int unit = (samp && samp->kind == V_SAMPLER) ? samp->u.s.unit : 0;
    const char *name = sh_fmt(c->h, "_g2_ts%d", unit);
    DictEnt *e = dict_find(&c->uniforms, name);
    Val *r;
    if (!e) {
        sym_put(c, &c->uniforms, name, c->next_uniform, "vec2", 0, 0);
        c->next_uniform++;
        e = dict_find(&c->uniforms, name);
    }
    r = alloc(c, "vec2");
    emit_ldp(c, r->u.r.n, ((Sym *)e->p)->slot);
    return r;
}

/* lambda = log2(rho), rho the longer of the two gradients in texels (ES 3.0 §3.8.9) */
static Val *grad_lod(Cg *c, Val *dx, Val *dy, Val *sz)
{
    Val *x = alloc(c, TY(c, dx)), *y, *dxx, *dyy, *m, *l, *r, *k;
    emit(c, "MUL r%d r%d r%d", x->u.r.n, N(c, dx), N(c, sz));
    y = alloc(c, TY(c, dy));
    emit(c, "MUL r%d r%d r%d", y->u.r.n, N(c, dy), N(c, sz));
    dxx = bi_dot(c, x, x); dyy = bi_dot(c, y, y);
    m = bi_max(c, dxx, dyy);
    l = un(c, "LOG2", m);
    r = alloc(c, "float");
    k = kf(c, 0.5);
    emit(c, "MUL r%d r%d r%d", r->u.r.n, l->u.r.n, k->u.r.n);
    return r;
}

static Val *bi_textureCubeGradEXT(Cg *c, Val *samp, Val *dir, Val *dx, Val *dy)
{
    /* a face's s = (sc / |ma| + 1) / 2: across a pixel the coordinate on the face moves by the gradient over 2 |ma| */
    Val *ax = bi_abs(c, dir), *e0 = extract(c, ax, 0), *e1 = extract(c, ax, 1), *m01 = bi_max(c, e0, e1), *e2 = extract(c, ax, 2), *ma = bi_max(c, m01, e2);
    Val *tw = alloc(c, "float"), *inv, *sz, *k;
    emit(c, "ADD r%d r%d r%d", tw->u.r.n, ma->u.r.n, ma->u.r.n);
    inv = alloc(c, "float"); emit(c, "RCP r%d r%d", inv->u.r.n, tw->u.r.n);
    sz = texsize(c, samp);
    k = alloc(c, "float");
    emit(c, "MUL r%d r%d.x r%d", k->u.r.n, sz->u.r.n, inv->u.r.n);
    return tex(c, samp, dir, "LOD", grad_lod(c, dx, dy, k), 1);
}

static Val *bi_tan(Cg *c, Val *x)
{
    Val *s = bi_sin(c, x), *co = bi_cos(c, x), *inv = alloc(c, TY(c, x)), *r;
    emit(c, "RCP r%d r%d", inv->u.r.n, co->u.r.n);
    r = alloc(c, TY(c, x));
    emit(c, "MUL r%d r%d r%d", r->u.r.n, s->u.r.n, inv->u.r.n);
    return r;
}

static Val *bi_dFdx(Cg *c, Val *x) { return un(c, "DDX", x); }
static Val *bi_dFdy(Cg *c, Val *x) { return un(c, "DDY", x); }

static Val *bi_fwidth(Cg *c, Val *x)
{
    Val *dx = bi_dFdx(c, x), *dy = bi_dFdy(c, x), *adx = bi_abs(c, dx), *ady = bi_abs(c, dy), *r = alloc(c, TY(c, x));
    emit(c, "ADD r%d r%d r%d", r->u.r.n, adx->u.r.n, ady->u.r.n);
    return r;
}

/* the dispatch of Compiler.builtin(): getattr(self, 'bi_' + name), called with *a */
typedef struct BiEnt { const char *name; int minargs, maxargs; int code; } BiEnt;
enum {
    B_abs, B_floor, B_fract, B_sign, B_min, B_max, B_lessThan, B_lessThanEqual, B_greaterThan, B_greaterThanEqual,
    B_equal, B_notEqual, B_any, B_all, B_not, B_clamp, B_mix, B_step, B_smoothstep, B_mod, B_dot, B_length, B_distance,
    B_sqrt, B_inversesqrt, B_normalize, B_cross, B_reflect, B_refract, B_matrixCompMult, B_radians, B_degrees, B_exp2,
    B_log2, B_exp, B_log, B_pow, B_sin, B_cos, B_atan, B_asin, B_acos, B_ceil, B_faceforward, B_textureCube,
    B_texture2DProj, B_texture2DLod, B_textureCubeLod, B_tan, B_texture2D, B_dFdx, B_dFdy, B_fwidth,
    B_texture2DProjLod, B_texture2DGradEXT, B_texture2DProjGradEXT, B_textureCubeGradEXT, B_texture3DProj, B_texture3DProjLod
};
static const BiEnt BUILTINS[] = {
    {"abs",1,1,B_abs},{"floor",1,1,B_floor},{"fract",1,1,B_fract},{"sign",1,1,B_sign},{"min",2,2,B_min},{"max",2,2,B_max},
    {"lessThan",2,2,B_lessThan},{"lessThanEqual",2,2,B_lessThanEqual},{"greaterThan",2,2,B_greaterThan},
    {"greaterThanEqual",2,2,B_greaterThanEqual},{"equal",2,2,B_equal},{"notEqual",2,2,B_notEqual},{"any",1,1,B_any},
    {"all",1,1,B_all},{"not",1,1,B_not},{"clamp",3,3,B_clamp},{"mix",3,3,B_mix},{"step",2,2,B_step},
    {"smoothstep",3,3,B_smoothstep},{"mod",2,2,B_mod},{"dot",2,2,B_dot},{"length",1,1,B_length},{"distance",2,2,B_distance},
    {"sqrt",1,1,B_sqrt},{"inversesqrt",1,1,B_inversesqrt},{"normalize",1,1,B_normalize},{"cross",2,2,B_cross},
    {"reflect",2,2,B_reflect},{"refract",3,3,B_refract},{"matrixCompMult",2,2,B_matrixCompMult},{"radians",1,1,B_radians},
    {"degrees",1,1,B_degrees},{"exp2",1,1,B_exp2},{"log2",1,1,B_log2},{"exp",1,1,B_exp},{"log",1,1,B_log},{"pow",2,2,B_pow},
    {"sin",1,1,B_sin},{"cos",1,1,B_cos},{"atan",0,1000000,B_atan},{"asin",1,1,B_asin},{"acos",1,1,B_acos},{"ceil",1,1,B_ceil},
    {"faceforward",3,3,B_faceforward},{"textureCube",2,3,B_textureCube},{"texture2DProj",2,3,B_texture2DProj},
    {"texture2DLod",3,3,B_texture2DLod},{"textureCubeLod",3,3,B_textureCubeLod},{"tan",1,1,B_tan},{"texture2D",2,3,B_texture2D},
    {"dFdx",1,1,B_dFdx},{"dFdy",1,1,B_dFdy},{"fwidth",1,1,B_fwidth},{"texture2DProjLod",3,3,B_texture2DProjLod},
    {"texture2DLodEXT",3,3,B_texture2DLod},{"texture2DProjLodEXT",3,3,B_texture2DProjLod},{"textureCubeLodEXT",3,3,B_textureCubeLod},
    {"texture2DGradEXT",4,4,B_texture2DGradEXT},{"texture2DProjGradEXT",4,4,B_texture2DProjGradEXT},{"textureCubeGradEXT",4,4,B_textureCubeGradEXT},
    {"texture3D",2,3,B_textureCube},{"texture3DLod",3,3,B_textureCubeLod},{"texture3DProj",2,3,B_texture3DProj},{"texture3DProjLod",3,3,B_texture3DProjLod},{NULL,0,0,0}
};

static Val *builtin(Cg *c, const char *name, Val **args, int na)
{
    const BiEnt *b;
    Val **a;
    int i;
    for (b = BUILTINS; b->name; b++) if (!strcmp(b->name, name)) break;
    if (!b->name) sh_compile_error(c->h, "unknown function/builtin '%s'", name);
    a = new_vals(c, na + 4);
    for (i = 0; i < na; i++) a[i] = asf(c, args[i]);
    for (; i < na + 4; i++) a[i] = NULL;
    if (na < b->minargs || na > b->maxargs) sh_py_error(c->h, "TypeError: wrong number of arguments to %s()", name);
    switch (b->code) {
    case B_abs: return bi_abs(c, a[0]);
    case B_floor: return bi_floor(c, a[0]);
    case B_fract: return bi_fract(c, a[0]);
    case B_sign: return bi_sign(c, a[0]);
    case B_min: return bi_min(c, a[0], a[1]);
    case B_max: return bi_max(c, a[0], a[1]);
    case B_lessThan: return bi_lessThan(c, a[0], a[1]);
    case B_lessThanEqual: return bi_lessThanEqual(c, a[0], a[1]);
    case B_greaterThan: return bi_greaterThan(c, a[0], a[1]);
    case B_greaterThanEqual: return bi_greaterThanEqual(c, a[0], a[1]);
    case B_equal: return bi_equal(c, a[0], a[1]);
    case B_notEqual: return bi_notEqual(c, a[0], a[1]);
    case B_any: return bi_any(c, a[0]);
    case B_all: return bi_all(c, a[0]);
    case B_not: return bi_not(c, a[0]);
    case B_clamp: return bi_clamp(c, a[0], a[1], a[2]);
    case B_mix: return bi_mix(c, a[0], a[1], a[2]);
    case B_step: return bi_step(c, a[0], a[1]);
    case B_smoothstep: return bi_smoothstep(c, a[0], a[1], a[2]);
    case B_mod: return bi_mod(c, a[0], a[1]);
    case B_dot: return bi_dot(c, a[0], a[1]);
    case B_length: return bi_length(c, a[0]);
    case B_distance: return bi_distance(c, a[0], a[1]);
    case B_sqrt: return bi_sqrt(c, a[0]);
    case B_inversesqrt: return bi_inversesqrt(c, a[0]);
    case B_normalize: return bi_normalize(c, a[0]);
    case B_cross: return bi_cross(c, a[0], a[1]);
    case B_reflect: return bi_reflect(c, a[0], a[1]);
    case B_refract: return bi_refract(c, a[0], a[1], a[2]);
    case B_matrixCompMult: return bi_matrixCompMult(c, a[0], a[1]);
    case B_radians: return bi_radians(c, a[0]);
    case B_degrees: return bi_degrees(c, a[0]);
    case B_exp2: return bi_exp2(c, a[0]);
    case B_log2: return bi_log2(c, a[0]);
    case B_exp: return bi_exp(c, a[0]);
    case B_log: return bi_log(c, a[0]);
    case B_pow: return bi_powv(c, a[0], a[1]);
    case B_sin: return bi_sin(c, a[0]);
    case B_cos: return bi_cos(c, a[0]);
    case B_atan: return atan64(c, a, na);
    case B_asin: return bi_asin(c, a[0]);
    case B_acos: return bi_acos(c, a[0]);
    case B_ceil: return bi_ceil(c, a[0]);
    case B_faceforward: return bi_faceforward(c, a[0], a[1], a[2]);
    case B_textureCube: return bi_textureCube(c, a[0], a[1], a[2]);
    case B_texture2DProj: return bi_texture2DProj(c, a[0], a[1], a[2]);
    case B_texture2DLod: return tex(c, a[0], a[1], "LOD", a[2], 0);
    case B_textureCubeLod: return tex(c, a[0], a[1], "LOD", a[2], 1);
    case B_tan: return bi_tan(c, a[0]);
    case B_texture2D: return bi_texture2D(c, a[0], a[1], a[2]);
    case B_dFdx: return bi_dFdx(c, a[0]);
    case B_dFdy: return bi_dFdy(c, a[0]);
    case B_texture2DProjLod: return tex(c, a[0], proj(c, a[1], "vec2"), "LOD", a[2], 0);
    case B_texture3DProj: return bi_textureCube(c, a[0], proj(c, a[1], "vec3"), a[2]);
    case B_texture3DProjLod: return tex(c, a[0], proj(c, a[1], "vec3"), "LOD", a[2], 1);
    case B_texture2DGradEXT: { Val *sz = texsize(c, a[0]); return tex(c, a[0], a[1], "LOD", grad_lod(c, a[2], a[3], sz), 0); }
    case B_texture2DProjGradEXT: { Val *st = proj(c, a[1], "vec2"), *sz = texsize(c, a[0]); return tex(c, a[0], st, "LOD", grad_lod(c, a[2], a[3], sz), 0); }
    case B_textureCubeGradEXT: return bi_textureCubeGradEXT(c, a[0], a[1], a[2], a[3]);
    default: return bi_fwidth(c, a[0]);
    }
}

/* ---------------- calls ---------------- */
/* a compile-time scalar becomes a register, in the scope that declares it */
static Val *promote(Cg *c, const char *name)
{
    DictEnt *d = const_home(c, name);
    double val = d->d;
    Dict *home = NULL;
    Val *r, *k;
    int i;
    for (i = c->nlocals - 1; i >= 0 && !home; i--) if (dict_find(c->locals[i], name) == d) home = c->locals[i];
    if (!home) { dict_del(&c->const_env, name); home = c->locals[c->nlocals - 1]; }
    r = alloc(c, "float");
    local_set(c, home, name, r, "float");
    k = kf(c, val);
    emit(c, "MOV r%d r%d", r->u.r.n, k->u.r.n);
    return r;
}

static Val *get_lvalue_reg(Cg *c, Node *e)
{
    if (e->kind == N_VAR && const_home(c, e->s)) return promote(c, e->s);
    return eval_lvalue(c, e);
}

/* an out / inout argument without a register of its own: part of a vector */
static int no_alias(Cg *c, Node *e)
{
    if (e->kind == N_INDEX) return is_reg(eval_lvalue(c, e->a));
    return e->kind == N_SWIZZLE && !is_struct(eval_lvalue(c, e->a));
}

static const char *local_type(Cg *c, const char *name);

/* the variable an lvalue writes: v of v.xy, a[i], s.m */
static const char *lv_base(Node *e)
{
    while (e->kind == N_SWIZZLE || e->kind == N_INDEX) e = e->a;
    return e->kind == N_VAR ? e->s : NULL;
}

/* does a function body read or write `name`, also through the functions it calls (a local of the name counts too);
 * own: the function has a parameter of the name, so only the functions it calls can see the caller's */
static int sees(Cg *c, Node *e, const char *name, Dict *seen, int own)
{
    int r = 0, i;
    if (!e || e->kind == N_REG) return 0;
    SH_ENTER(c->h, 2);
    if (e->kind == N_VAR) r = !own && !strcmp(e->s, name);
    else {
        if (e->kind == N_CALL) {
            const char *fn = e->fn ? e->fn->name : e->s;
            DictEnt *fe = dict_find(&c->funcs, fn);
            if (fe && !dict_find(seen, fn)) {
                dict_put(c->h, seen, fn);
                r = sees(c, ((Func *)fe->p)->body, name, seen, 0);
            }
        }
        if (!r) r = sees(c, e->a, name, seen, own) || sees(c, e->b, name, seen, own) || sees(c, e->c, name, seen, own) || sees(c, e->d, name, seen, own);
        for (i = 0; i < e->nlist && !r; i++) r = sees(c, e->list[i], name, seen, own);
    }
    SH_LEAVE(c->h, 2);
    return r;
}

/* a local whose register holds its value in the other representation than the parameter's: a promoted int is a float */
static int repr_differs(Cg *c, Node *e, const char *pty)
{
    const char *lt = e->kind == N_VAR ? local_type(c, e->s) : NULL;
    return lt && !strcmp(lt, "int") != !strcmp(pty, "int");
}

/* a return under an if or in a loop (under: any return at all) */
static int early_return(Cg *c, Node *s, int under)
{
    int r = 0, i;
    if (!s) return 0;
    SH_ENTER(c->h, 2);
    switch (s->kind) {
    case S_RETURN: r = under; break;
    case S_BLOCK: case S_MULTI:
        for (i = 0; i < s->nlist && !r; i++) r = early_return(c, s->list[i], under);
        break;
    case S_IF: r = early_return(c, s->b, 1) || early_return(c, s->c, 1); break;
    case S_FOR: r = early_return(c, s->d, 1); break;
    case S_WHILE: case S_DOWHILE: r = early_return(c, s->b, 1); break;
    default: break;
    }
    SH_LEAVE(c->h, 2);
    return r;
}

/* A return under an if or in a loop leaves in some lanes only, so on the SIMT cores it cannot be a BRA (the others would be lost and the mask stack left open): the body runs as a LOOP of
 * one iteration and the return is a per-lane BREAK, which ELSE, ENDIF and ENDL honour. Inside a hardware loop that BREAK leaves only that loop: left marks the lanes for one more BREAK after its ENDL (hw_loop) */
static void flow_open(Cg *c, Node *body, CurRet *cr)
{
    Val *k;
    cr->flow = early_return(c, body, 0); cr->depth = cr->any = 0; cr->left = NULL;
    if (!cr->flow) return;
    cr->left = alloc(c, "float");
    k = kf(c, 0.0);
    emit(c, "MOV r%d r%d", cr->left->u.r.n, k->u.r.n);
    emit(c, "LOOP 1");
}

static void hw_loop(Cg *c, int d)
{
    CurRet *f = c->cur_ret;
    if (!f || !f->flow) return;
    f->depth += d;
    if (d < 0 && f->any) {
        Val *z = zeroreg(c);
        emit(c, "SETP SNE r%d r%d", f->left->u.r.n, z->u.r.n);
        emit(c, "BREAK");
    }
}

/* main: a return ends the program, past the code that writes the outputs kept in locals */
static void run_main(Cg *c, Node *body)
{
    CurRet cr;
    if (!early_return(c, body, 1)) { run_stmt(c, body); return; }
    cr.retreg = alloc(c, "float");
    cr.lret = newlabel(c, "Lret"); cr.rt = "void"; cr.holder = NULL;
    flow_open(c, body, &cr);
    c->cur_ret = &cr;
    run_stmt(c, body);
    if (cr.flow) emit(c, "ENDL");
    emit(c, "%s:", cr.lret);
    c->cur_ret = NULL;
}

static Val *inline_call(Cg *c, const char *name, Node **argexprs, int nargs)
{
    Func *f;
    Dict *binds;
    Val *retreg, *res, **copy, **outs;
    const char *lret;
    CurRet cr, *saved;
    Dict **outer, wrt;
    int i, nouter, capouter;
    SH_ENTER(c->h, 1);
    if (dict_find(&c->inlining, name)) sh_compile_error(c->h, "recursive call to '%s' (forbidden in GLSL ES)", name);
    f = (Func *)dict_find(&c->funcs, name)->p;
    if (f->nparams != nargs) sh_compile_error(c->h, "%s(): expected %d args, got %d", name, f->nparams, nargs);
    binds = new_dict(c);
    copy = (Val **)sh_zalloc(c->h, ((size_t)nargs + 1) * sizeof(Val *));
    outs = (Val **)sh_zalloc(c->h, ((size_t)nargs + 1) * sizeof(Val *));
    memset(&wrt, 0, sizeof wrt);
    written(c, f->body, &wrt);                  /* a parameter the body never writes keeps a known value (const, cvec) */
    for (i = 0; i < nargs; i++) {
        Param *p = &f->params[i];
        Node *ae = argexprs[i];
        if (!strcmp(p->q, "out") || !strcmp(p->q, "inout")) {
            const char *b;
            int shared = 0, k;
            Val *agg = NULL;
            if (ae->kind == N_VAR && const_home(c, ae->s)) promote(c, ae->s);         /* a register to write */
            b = lv_base(ae);                    /* §6.1.1 copies in and out: an alias would show when another argument or the function sees it */
            if (b) {
                int same = 0, own = 0;
                for (k = 0; k < nargs; k++) {
                    const char *q = f->params[k].q, *o;
                    if ((!strcmp(q, "out") || !strcmp(q, "inout")) && (o = lv_base(argexprs[k])) != NULL && !strcmp(o, b)) same++;
                }
                for (k = 0; k < f->nparams; k++) if (!strcmp(f->params[k].name, b)) own = 1;
                if (same > 1) shared = 1;
                else {
                    Dict *seen = new_dict(c);
                    dict_put(c->h, seen, name);
                    shared = sees(c, f->body, b, seen, own);
                }
            }
            if (shared && !no_alias(c, ae)) {
                Val *lv = get_lvalue_reg(c, ae);
                if (lv && (lv->kind == V_MAT || lv->kind == V_ARRAY || lv->kind == V_STRUCT)) agg = lv;
            }
            if (agg) {                          /* an array, struct or matrix of its own, written back at the end */
                Val *pr = !strcmp(p->q, "inout") ? clone_val(c, agg) : like(c, agg);
                local_set(c, binds, p->name, pr, p->ty);
                copy[i] = pr;
                if (!strcmp(p->q, "out")) outs[i] = pr;
            }
            else if (no_alias(c, ae) || repr_differs(c, ae, p->ty) || shared) {    /* no register to alias: a copy, written back at the end */
                Val *pr = alloc(c, p->ty);
                if (!strcmp(p->q, "inout")) {
                    Val *av = eval(c, ae);
                    av = !strcmp(p->ty, "int") ? asi(c, av) : asf(c, av);
                    emit(c, "MOV r%d r%d", pr->u.r.n, N(c, av));
                }
                local_set(c, binds, p->name, pr, p->ty);
                copy[i] = pr;
                if (!strcmp(p->q, "out")) outs[i] = pr;
            } else {
                Val *v = get_lvalue_reg(c, ae);
                local_set(c, binds, p->name, v, p->ty);
                if (!strcmp(p->q, "out")) outs[i] = v;
            }
        } else {
            Val *av = eval(c, ae);
            if (av && av->kind != V_REG) { av = clone_val(c, av); local_set(c, binds, p->name, av, p->ty); }
            else {
                Val *pr;
                av = !strcmp(p->ty, "int") ? asi(c, av) : asf(c, av);
                pr = alloc(c, p->ty);
                emit(c, "MOV r%d r%d", pr->u.r.n, N(c, av));
                local_set(c, binds, p->name, pr, p->ty);
                if (!dict_find(&wrt, p->name) && !strcmp(p->ty, av->ty)) {
                    pr->u.r.has_const = av->u.r.has_const; pr->u.r.cval = av->u.r.cval;
                    pr->u.r.cvec = av->u.r.cvec; pr->u.r.ncvec = av->u.r.ncvec;
                }
            }
        }
    }
    for (i = 0; i < nargs; i++) zero_val(c, outs[i]);       /* an out parameter starts at 0 as a local does (zero_val); after every argument is read: f(x, x), in and out the same variable */
    retreg = alloc(c, strcmp(f->rt, "void") ? f->rt : "float");
    lret = newlabel(c, "Lret");
    saved = c->cur_ret;
    cr.retreg = retreg; cr.lret = lret; cr.rt = f->rt; cr.holder = NULL;
    flow_open(c, f->body, &cr);
    c->cur_ret = &cr;
    dict_put(c->h, &c->inlining, name);
    outer = c->locals; nouter = c->nlocals; capouter = c->localcap;
    c->locals = NULL; c->nlocals = c->localcap = 0;
    scope_push(c, outer[0]);                    /* a function sees its own names and, in [0], the outputs kept in locals */
    scope_push(c, binds);
    run_stmt(c, f->body);
    c->locals = outer; c->nlocals = nouter; c->localcap = capouter;
    dict_del(&c->inlining, name);
    if (cr.flow) emit(c, "ENDL");
    emit(c, "%s:", lret);
    c->cur_ret = saved;
    for (i = 0; i < nargs; i++) if (copy[i]) store(c, argexprs[i], copy[i]);
    res = cr.holder ? cr.holder : retreg;
    SH_LEAVE(c->h, 1);
    return res;
}

static Val *call(Cg *c, const char *name, Node **argexprs, int nargs)
{
    Val **args, *r;
    int i;
    SH_ENTER(c->h, 1);
    if (is_cast(name) && nargs == 1) {
        double cv;
        Val *a;
        if (fold_const(c, argexprs[0], &cv)) { r = kf(c, sh_fold_cast(c->h, name, cv)); goto done; }
        a = eval(c, argexprs[0]);
        if (is_mat(a)) a = extract(c, col(c, a, 0), 0);         /* of a vector or a matrix: its first component */
        else if (is_reg(a) && ncomp(a->ty) > 1) a = extract(c, a, 0);
        a = asf(c, a);
        r = alloc(c, !strcmp(name, "bool") ? "float" : name);
        if (!strcmp(name, "int")) { emit(c, "FTOI r%d r%d", r->u.r.n, N(c, a)); goto done; }
        if (!strcmp(name, "bool")) {
            Val *z = zeroreg(c);
            emit(c, "SET SNE r%d r%d r%d", r->u.r.n, N(c, a), z->u.r.n);
        }
        else emit(c, "MOV r%d r%d", r->u.r.n, N(c, a));
        goto done;
    }
    if (vec_n(name) || mat_n(name)) {
        args = new_vals(c, nargs);
        for (i = 0; i < nargs; i++) args[i] = eval(c, argexprs[i]);
        r = mat_n(name) ? construct_mat(c, name, args, nargs) : construct(c, name, args, nargs);
        goto done;
    }
    {
        DictEnt *sd = dict_find(&c->structs, name);
        if (sd) {
            StructDef *s = (StructDef *)sd->p;
            Dict *m;
            args = new_vals(c, nargs);
            for (i = 0; i < nargs; i++) args[i] = eval(c, argexprs[i]);
            if (nargs != s->n) sh_compile_error(c->h, "%s(): expected %d members, got %d", name, s->n, nargs);
            m = new_dict(c);
            for (i = 0; i < nargs; i++) dict_put(c->h, m, s->m[i].name)->p = args[i];
            r = mk_struct(c, name, m);
            goto done;
        }
    }
    if (dict_find(&c->funcs, name)) { r = inline_call(c, name, argexprs, nargs); goto done; }
    args = new_vals(c, nargs);
    for (i = 0; i < nargs; i++) args[i] = eval(c, argexprs[i]);
    r = builtin(c, name, args, nargs);
done:
    SH_LEAVE(c->h, 1);
    return r;
}

/* ---------------- expressions ---------------- */
static int is_numv(const Node *e, double v) { return e->kind == N_NUM && e->num == v; }
static int is_integer(double v) { return v - v == 0.0 && (v >= 4503599627370496.0 || v <= -4503599627370496.0 || (double)(long long)v == v); }

/* gl_FragData[i] is gl_FragColor: one draw buffer. i may be any constant expression, a loop's index too */
static Node *fd(Cg *c, Node *e)
{
    double v;
    if (e->kind == N_INDEX && e->a->kind == N_VAR && !strcmp(e->a->s, "gl_FragData")) {
        Node *n;
        if (!fold_const(c, e->b, &v) || v != 0.0) sh_compile_error(c->h, "gl_FragData index out of range: MAX_DRAW_BUFFERS is 1");
        n = (Node *)sh_zalloc(c->h, sizeof(Node));
        n->kind = N_VAR; n->s = "gl_FragColor";
        return n;
    }
    return e;
}

/* ---- a run-time index that is not LDPX's (a vector, a matrix, a local or varying array, a uniform array in the fragment
 * core, which has no indirect load): every element read, then one SEL per element ---- */
static Val **dyn_elems(Cg *c, Val *base, long long *n)
{
    Val **el;
    long long i;
    if (is_mat(base)) { *n = base->u.m.ncols; return base->u.m.cols; }
    if (is_array(base)) {
        *n = base->u.a.size;
        el = new_vals(c, *n);
        for (i = 0; i < *n; i++) el[i] = array_elem(c, base, i);
        return el;
    }
    if (is_reg(base)) {
        *n = ncomp(base->ty);
        el = new_vals(c, *n);
        for (i = 0; i < *n; i++) el[i] = extract(c, base, i);
        return el;
    }
    sh_compile_error(c->h, "an index into this needs a constant expression");
}

static Val *ieq(Cg *c, Val *ireg, long long i)
{
    Val *cn = alloc(c, "bool"), *k = asi_lit(c, (double)i);
    emit(c, "ISEQ r%d r%d r%d", cn->u.r.n, N(c, ireg), k->u.r.n);
    return cn;
}

/* base[idx] per lane: an index past the end reads element 0 */
static Val *dyn_read(Cg *c, Val *base, Node *idx)
{
    Val *x = eval(c, idx), *ireg = asi(c, x), **el, *r;
    long long n, i;
    el = dyn_elems(c, base, &n);
    r = el[0];
    for (i = 1; i < n; i++) { Val *q = ieq(c, ireg, i); r = sel_val(c, q, el[i], r); }
    return r;
}

/* base[idx] = rv per lane: each element takes rv where the index is its own */
static void dyn_write(Cg *c, Val *base, Node *idx, Val *rv)
{
    Val *x = eval(c, idx), *ireg = asi(c, x);
    long long i;
    if (is_reg(base)) {
        for (i = 0; i < ncomp(base->ty); i++) {
            Val *n = alloc(c, "float"), *q = ieq(c, ireg, i);
            const char *src = swz_src(c, base, i);
            emit(c, "SEL r%d r%d r%d %s", n->u.r.n, q->u.r.n, N(c, rv), src);
            emit(c, "MOV.%c r%d r%d", "xyzw"[i], base->u.r.n, n->u.r.n);
        }
        return;
    }
    if (is_array(base) && base->u.a.local != 1) sh_compile_error(c->h, "cannot write this array");
    {
        long long n = is_mat(base) ? base->u.m.ncols : is_array(base) ? base->u.a.size : 0;
        Val **el = is_mat(base) ? base->u.m.cols : is_array(base) ? base->u.a.elems : NULL;
        if (!el) sh_py_error(c->h, "AttributeError: a value here has no .elems");
        for (i = 0; i < n; i++) {
            Val *q = ieq(c, ireg, i), *t = sel_val(c, q, rv, el[i]);
            copy_val(c, el[i], t);
        }
    }
}

/* evaluating e does more than give a value: it writes something, or calls something that may */
static int effects(Cg *c, Node *e)
{
    int r = 0, i;
    if (!e || e->kind == N_REG) return 0;
    SH_ENTER(c->h, 2);
    if (e->kind == N_ASSIGN || e->kind == N_POSTOP) r = 1;
    else if (e->kind == N_CALL && (e->fn || dict_find(&c->funcs, e->s))) r = 1;
    else {
        r = effects(c, e->a) || effects(c, e->b) || effects(c, e->c) || effects(c, e->d);
        for (i = 0; i < e->nlist && !r; i++) r = effects(c, e->list[i]);
    }
    SH_LEAVE(c->h, 2);
    return r;
}

/* cn ? then : other with only one side run, per lane; other is an expression, or the value otherv */
static Val *branch(Cg *c, Val *cn, Node *then, Node *other, Val *otherv)
{
    Val *z = zeroreg(c), *rv, *d;
    const char *lelse, *lend;
    emit(c, "SETP SNE r%d r%d", N(c, cn), z->u.r.n);
    lelse = newlabel(c, "Lelse"); lend = newlabel(c, "Lend");
    emit(c, "IF %s", lelse);
    rv = eval(c, then); rv = asf(c, rv);
    d = like(c, rv);
    copy_val(c, d, rv);
    emit(c, "%s:", lelse);
    emit(c, "ELSE %s", lend);
    if (other) { otherv = eval(c, other); otherv = asf(c, otherv); }
    copy_val(c, d, otherv);
    emit(c, "%s:", lend);
    emit(c, "ENDIF");
    return d;
}

static Val *eval_index(Cg *c, Node *e)
{
    Node *f = fd(c, e);
    Val *base;
    double ci;
    long long i;
    if (f != e) return eval(c, f);
    base = eval(c, e->a);
    if (!fold_const(c, e->b, &ci)) {                            /* a run-time index */
        if (is_array(base) && base->u.a.local == 0 && !c->fs) return uniform_dyn_index(c, base, e->b);
        if (base && base->kind == V_SAMPLER) sh_compile_error(c->h, "a sampler's index must be a constant expression");
        return dyn_read(c, base, e->b);
    }
    i = py_int(c, ci);
    if (base && base->kind == V_SAMPLER) {
        if (base->u.s.size < 0) sh_compile_error(c->h, "indexing a sampler that is not an array");
        if (i < 0 || i >= base->u.s.size) sh_compile_error(c->h, "array index %L out of range [0,%L)", i, base->u.s.size);
        return mk_sampler(c, base->u.s.unit + (int)i, base->ty, -1);
    }
    if (is_array(base)) {
        if (i < 0 || i >= base->u.a.size) sh_compile_error(c->h, "array index %L out of range [0,%L)", i, base->u.a.size);
        return array_elem(c, base, i);
    }
    if (is_mat(base)) return col(c, base, i);
    return extract(c, base, i);
}

static Val *eval_(Cg *c, Node *e)
{
    switch (e->kind) {
    case N_SEQ: {
        Val *r = NULL;
        int i;
        for (i = 0; i < e->nlist; i++) r = eval(c, e->list[i]);
        return r;
    }
    case N_REG: return e->reg;
    case N_NUM: return kf(c, e->num);
    case N_VAR: {
        DictEnt *d = const_home(c, e->s);
        if (d) return kf(c, d->d);
        return load_var(c, e->s);
    }
    case N_SWIZZLE: {
        Val *base = eval(c, e->a);
        if (is_struct(base)) {
            DictEnt *m = dict_find(base->u.st.members, e->s);
            if (!m) sh_compile_error(c->h, "struct %s has no member '%s'", base->u.st.sname, e->s);
            return (Val *)m->p;
        }
        return swizzle_read(c, base, e->s);
    }
    case N_INDEX: return eval_index(c, e);
    case N_UNARY: {
        Val *a = eval(c, e->a), *r;
        if (e->s[0] == '+') return a;
        if (e->s[0] == '-' && is_mat(a)) {
            Val **cols = new_vals(c, a->u.m.ncols);
            int i;
            for (i = 0; i < a->u.m.ncols; i++) {
                r = alloc(c, TY(c, a->u.m.cols[i]));
                emit(c, "MOV r%d -r%d", r->u.r.n, N(c, a->u.m.cols[i]));
                cols[i] = r;
            }
            return mk_mat(c, cols, a->u.m.ncols, a->ty);
        }
        if (e->s[0] == '-') {
            if (is_reg(a) && !strcmp(a->ty, "int")) {
                Val *z;
                r = alloc(c, "int");
                z = asi_lit(c, 0.0);
                emit(c, "ISUB r%d r%d r%d", r->u.r.n, z->u.r.n, a->u.r.n);
                return r;
            }
            r = alloc(c, TY(c, a));
            emit(c, "MOV r%d -r%d", r->u.r.n, N(c, a));
            return r;
        }
        {
            Val *one = kf(c, 1.0);
            r = alloc(c, "float");
            emit(c, "SUB r%d r%d r%d", r->u.r.n, one->u.r.n, N(c, a));
            return r;
        }
    }
    case N_BIN: return binop(c, e->s, e->a, e->b);
    case N_CALL: return call(c, e->fn ? e->fn->name : e->s, e->list, e->nlist);
    case N_TERNARY: {
        Val *cn = eval(c, e->a), *a, *b, *r;
        if (effects(c, e->b) || effects(c, e->c)) return branch(c, cn, e->b, e->c, NULL);     /* only one side runs */
        a = eval(c, e->b); a = asf(c, a);
        b = eval(c, e->c); b = asf(c, b);
        if (is_mat(a) || is_struct(a)) return sel_val(c, cn, a, b);
        r = alloc(c, ncomp(TY(c, a)) >= ncomp(TY(c, b)) ? a->ty : b->ty);
        emit(c, "SEL r%d r%d r%d r%d", r->u.r.n, N(c, cn), N(c, a), N(c, b));
        return r;
    }
    case N_ASSIGN: {
        Node *lhs = e->a, *rhs = e->b;
        const char *op = e->s;
        Val *rv;
        char op0[2];
        op0[0] = op[0]; op0[1] = 0;
        if (lhs->kind == N_VAR && const_home(c, lhs->s)) {
            double cv;
            int has;
            if (!strcmp(op, "=")) has = fold_const(c, rhs, &cv);
            else {
                Node t;
                memset(&t, 0, sizeof t);
                t.kind = N_BIN; t.s = op0; t.a = lhs; t.b = rhs;
                has = fold_const(c, &t, &cv);
            }
            if (has) {
                DictEnt *d = const_home(c, lhs->s);
                d->d = d->i ? sh_fold_cast(c->h, "int", cv) : cv;
                return kf(c, d->d);
            }
            promote(c, lhs->s);
        }
        rv = !strcmp(op, "=") ? eval(c, rhs) : binop(c, op0, lhs, rhs);
        store(c, lhs, rv);
        return rv;
    }
    case N_POSTOP: {
        Node *t = e->a;
        int inc = e->s[0] == '+';
        Val *cur, *r;
        if (t->kind == N_VAR) {
            DictEnt *d = const_home(c, t->s);
            if (d) {
                double old = d->d;
                d->d = sh_pf(c->h, old + (inc ? 1.0 : -1.0));
                return kf(c, old);
            }
        }
        cur = eval(c, t);
        if (is_mat(cur)) {
            Val *one = kf(c, 1.0), **cols = new_vals(c, cur->u.m.ncols), *old;
            int i;
            for (i = 0; i < cur->u.m.ncols; i++) {
                Val *x = alloc(c, TY(c, cur->u.m.cols[i]));
                emit(c, "%s r%d r%d r%d", inc ? "ADD" : "SUB", x->u.r.n, N(c, cur->u.m.cols[i]), one->u.r.n);
                cols[i] = x;
            }
            r = mk_mat(c, cols, cur->u.m.ncols, cur->ty);
            old = clone_val(c, cur);
            store(c, t, r);
            return old;
        }
        if (is_reg(cur) && !strcmp(cur->ty, "int")) {
            Val *k;
            r = alloc(c, "int");
            k = asi_lit(c, 1.0);
            emit(c, "%s r%d r%d r%d", inc ? "IADD" : "ISUB", r->u.r.n, cur->u.r.n, k->u.r.n);
        } else {
            Val *one = kf(c, 1.0);
            r = alloc(c, TY(c, cur));
            emit(c, "%s r%d r%d r%d", inc ? "ADD" : "SUB", r->u.r.n, N(c, cur), one->u.r.n);
        }
        {
            Val *old = alloc(c, TY(c, cur));                    /* cur is the variable itself: the value before the step */
            emit(c, "MOV r%d r%d", old->u.r.n, N(c, cur));
            store(c, t, r);
            return old;
        }
    }
    default: break;
    }
    sh_compile_error(c->h, "cannot eval this node");
}

static Val *eval(Cg *c, Node *e)
{
    Val *v;
    SH_ENTER(c->h, 1);
    v = eval_(c, e);
    SH_LEAVE(c->h, 1);
    return v;
}

/* a op b, lane by lane, with the shapes the caller settled */
static Val *arith(Cg *c, const char *op, Val *a, Val *b, const char *ty)
{
    Val *r = alloc(c, ty);
    if (op[0] == '+') emit(c, "ADD r%d r%d r%d", r->u.r.n, N(c, a), N(c, b));
    else if (op[0] == '-') emit(c, "SUB r%d r%d r%d", r->u.r.n, N(c, a), N(c, b));
    else if (op[0] == '*') emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, a), N(c, b));
    else {
        Val *inv = alloc(c, TY(c, b));
        emit(c, "RCP r%d r%d", inv->u.r.n, N(c, b));
        emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, a), inv->u.r.n);
    }
    return r;
}

/* §5.9: * is the linear product of matrices and vectors; everything else, and any matrix with a scalar, is by component */
static Val *mat_op(Cg *c, const char *op, Val *a, Val *b)
{
    int ma = is_mat(a), mb = is_mat(b), j;
    Val *m, **cols;
    if (!strcmp(op, "*") && ((ma && mb) || (ma && !is_scalar(TY(c, b))) || (mb && !is_scalar(TY(c, a))))) return mat_mul(c, a, b);
    m = ma ? a : b;
    if (strcmp(op, "+") && strcmp(op, "-") && strcmp(op, "*") && strcmp(op, "/")) sh_compile_error(c->h, "op %s unsupported", op);
    cols = new_vals(c, m->u.m.ncols);
    for (j = 0; j < m->u.m.ncols; j++) cols[j] = arith(c, op, ma ? col(c, a, j) : a, mb ? col(c, b, j) : b, m->u.m.cols[j]->ty);
    return mk_mat(c, cols, m->u.m.ncols, m->ty);
}

/* == and != of matrices and structs: one bool */
static void eq_leaves(Cg *c, Val *a, Val *b, Val **acc)
{
    int i;
    if (is_mat(a)) {
        for (i = 0; i < a->u.m.ncols; i++) eq_leaves(c, col(c, a, i), col(c, b, i), acc);
    }
    else if (is_struct(a)) {
        Dict *f = a->u.st.members;
        for (i = 0; i < f->n; i++) {
            if (!f->ent[i].live) continue;
            eq_leaves(c, (Val *)f->ent[i].p, member_get(c, b, f->ent[i].key), acc);
        }
    }
    else {
        Val *x = asf(c, a), *y = asf(c, b), *q = setc(c, "SEQ", x, y);
        if (ncomp(TY(c, x)) > 1) q = bi_all(c, q);
        if (!*acc) *acc = q;
        else {
            Val *t = alloc(c, "float");
            emit(c, "MIN r%d r%d r%d", t->u.r.n, (*acc)->u.r.n, q->u.r.n);
            *acc = t;
        }
    }
}

static Val *agg_eq(Cg *c, const char *op, Val *a, Val *b)
{
    Val *acc = NULL, *r;
    eq_leaves(c, a, b, &acc);
    r = alloc(c, "bool");
    if (op[0] == '=') emit(c, "MOV r%d r%d", r->u.r.n, N(c, acc));
    else { Val *one = kf(c, 1.0); emit(c, "SUB r%d r%d r%d", r->u.r.n, one->u.r.n, N(c, acc)); }
    return r;
}

static int fzero(const Val *v) { return is_reg(v) && !strcmp(v->ty, "float") && v->u.r.has_const && v->u.r.cval == 0.0; }       /* glslc._fzero */
static int fvec_ty(const char *ty) { return !strcmp(ty, "float") || !strcmp(ty, "vec2") || !strcmp(ty, "vec3") || !strcmp(ty, "vec4"); }

static Val *binop_(Cg *c, const char *op, Node *ea, Node *eb)
{
    int plus = !strcmp(op, "+"), minus = !strcmp(op, "-"), mul = !strcmp(op, "*"), div = !strcmp(op, "/");
    int ai, bi, n;
    Val *a, *b, *r;
    const char *rt;
    if (ea->kind == N_NUM && eb->kind == N_NUM && (plus || minus || mul || div)) {
        Node t;
        double v;
        memset(&t, 0, sizeof t);
        t.kind = N_BIN; t.s = op; t.a = ea; t.b = eb;
        fold_const(c, &t, &v);
        return kf(c, v);
    }
    if ((!strcmp(op, "&&") || !strcmp(op, "||")) && effects(c, eb)) {     /* the right side runs only where the left leaves it open */
        Val *l = eval(c, ea), *k;
        l = asf(c, l);
        if (op[0] == '&') { k = kf(c, 0.0); return branch(c, l, eb, NULL, k); }
        {
            Val *nn = alloc(c, "float"), *z = zeroreg(c);
            emit(c, "SET SEQ r%d r%d r%d", nn->u.r.n, N(c, l), z->u.r.n);
            k = kf(c, 1.0);
            return branch(c, nn, eb, NULL, k);
        }
    }
    if (mul) {
        Node *xe;
        if (is_numv(eb, 1.0)) return eval(c, ea);
        if (is_numv(ea, 1.0)) return eval(c, eb);
        if (is_numv(ea, 0.0) || is_numv(eb, 0.0)) {             /* x*0 -> 0 (ES2 exprs are side-effect-free) */
            Val *x = eval(c, is_numv(ea, 0.0) ? eb : ea);
            if (!is_mat(x)) return kf(c, 0.0);
        }
        xe = is_numv(eb, 2.0) ? ea : is_numv(ea, 2.0) ? eb : NULL;
        if (xe) {
            Val *x = eval(c, xe);
            if (is_reg(x)) {
                r = alloc(c, x->ty);
                emit(c, "%s r%d r%d r%d", !strcmp(x->ty, "int") ? "IADD" : "ADD", r->u.r.n, x->u.r.n, x->u.r.n);
                return r;
            }
        }
    }
    if ((plus || minus) && is_numv(eb, 0.0)) return eval(c, ea);
    if (plus && is_numv(ea, 0.0)) return eval(c, eb);
    a = eval(c, ea);
    b = eval(c, eb);
    if ((plus || minus) && fzero(b) && is_reg(a) && fvec_ty(a->ty)) return a;     /* a register known to hold 0.0, as the literal above (glslc.binop) */
    if (plus && fzero(a) && is_reg(b) && fvec_ty(b->ty)) return b;
    if ((!strcmp(op, "==") || !strcmp(op, "!=")) && (is_struct(a) || (is_mat(a) && is_mat(b)))) return agg_eq(c, op, a, b);
    if (ea->isint && eb->isint) { a = asi(c, a); b = asi(c, b); }
    ai = is_reg(a) && !strcmp(a->ty, "int");
    bi = is_reg(b) && !strcmp(b->ty, "int");
    if ((ai || bi) && (div || mul)) {                           /* IDIV (truncates toward zero), IMUL */
        if (!bi && eb->kind == N_NUM && is_integer(eb->num)) { b = asi_lit(c, eb->num); bi = 1; }
        if (!ai && ea->kind == N_NUM && is_integer(ea->num)) { a = asi_lit(c, ea->num); ai = 1; }
        if (ai && bi) {
            if (div && (c->h->noops & 1)) {
                int an = N(c, a), bn = N(c, b);
                return idiv_lowered(c, an, bn, "int");
            }
            r = alloc(c, "int");
            emit(c, "%s r%d r%d r%d", div ? "IDIV" : "IMUL", r->u.r.n, N(c, a), N(c, b));
            return r;
        }
    }
    {
        int rel = !strcmp(op, "<") || !strcmp(op, ">") || !strcmp(op, "<=") || !strcmp(op, ">=") || !strcmp(op, "==") || !strcmp(op, "!=");
        if ((ai || bi) && (plus || minus || rel)) {
            if (!bi && eb->kind == N_NUM && is_integer(eb->num)) { b = asi_lit(c, eb->num); bi = 1; }
            if (!ai && ea->kind == N_NUM && is_integer(ea->num)) { a = asi_lit(c, ea->num); ai = 1; }
            if (ai && bi) return int_binop(c, op, a, b);
            a = asf(c, a); b = asf(c, b);
        }
        else if (ai || bi) { a = asf(c, a); b = asf(c, b); }
    }
    if (is_mat(a) || is_mat(b)) return mat_op(c, op, a, b);
    if (div && (!strncmp(TY(c, a), "ivec", 4) || !strncmp(TY(c, b), "ivec", 4))) {     /* an int vector's lanes are floats; / truncates */
        const char *t = !strncmp(a->ty, "ivec", 4) ? a->ty : b->ty;
        Val *x = alloc(c, t), *y = alloc(c, t), *q = alloc(c, t);
        r = alloc(c, t);
        emit(c, "FTOI r%d r%d", x->u.r.n, N(c, a));
        emit(c, "FTOI r%d r%d", y->u.r.n, N(c, b));
        if (c->h->noops & 1) q = idiv_lowered(c, x->u.r.n, y->u.r.n, t);
        else emit(c, "IDIV r%d r%d r%d", q->u.r.n, x->u.r.n, y->u.r.n);
        emit(c, "ITOF r%d r%d", r->u.r.n, q->u.r.n);
        return r;
    }
    rt = ncomp(TY(c, a)) >= ncomp(TY(c, b)) ? a->ty : b->ty;
    r = alloc(c, rt);
    if (plus) emit(c, "ADD r%d r%d r%d", r->u.r.n, N(c, a), N(c, b));
    else if (minus) emit(c, "SUB r%d r%d r%d", r->u.r.n, N(c, a), N(c, b));
    else if (mul) emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, a), N(c, b));
    else if (div) {
        Val *inv = alloc(c, TY(c, b));
        emit(c, "RCP r%d r%d", inv->u.r.n, N(c, b));
        emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, a), inv->u.r.n);
    }
    else if (!strcmp(op, "<") || !strcmp(op, ">") || !strcmp(op, "<=") || !strcmp(op, ">=") || !strcmp(op, "==") || !strcmp(op, "!=")) {
        if (op[1] == '=' && (op[0] == '=' || op[0] == '!') && ncomp(a->ty) > 1) {
            Val *cw, *red;
            n = op[0] == '=';
            cw = n ? bi_equal(c, a, b) : bi_notEqual(c, a, b);
            red = n ? bi_all(c, cw) : bi_any(c, cw);
            emit(c, "MOV r%d r%d", r->u.r.n, red->u.r.n);
            r->ty = "bool";
        }
        else cmp_(c, r, a, b, op);
    }
    else if (!strcmp(op, "&&")) emit(c, "MUL r%d r%d r%d", r->u.r.n, N(c, a), N(c, b));
    else if (!strcmp(op, "||")) emit(c, "MAX r%d r%d r%d", r->u.r.n, N(c, a), N(c, b));
    else if (!strcmp(op, "^^")) cmp_(c, r, a, b, "!=");
    else sh_compile_error(c->h, "op %s unsupported", op);
    return r;
}

static Val *binop(Cg *c, const char *op, Node *ea, Node *eb)
{
    Val *v;
    SH_ENTER(c->h, 1);
    v = binop_(c, op, ea, eb);
    SH_LEAVE(c->h, 1);
    return v;
}

/* ---------------- assignment ---------------- */
static const char *local_type(Cg *c, const char *name)
{
    Local *l = local_find(c, name);
    return l ? l->ty : NULL;
}

static Val *get_local_reg(Cg *c, const char *name, const char *ty)
{
    Local *l = local_find(c, name);
    Val *r;
    if (l) return l->v;
    r = alloc(c, ty);
    local_set(c, c->locals[c->nlocals - 1], name, r, ty);
    return r;
}

static int store_special(Cg *c, const char *name, Val *rv)
{
    Val *t;
    if (!strcmp(name, "gl_Position") || !strcmp(name, "gl_FragColor")) {
        t = asf(c, rv);
        emit(c, "STO 0 r%d", N(c, t));
        return 1;
    }
    if (!strcmp(name, "gl_PointSize")) {
        if (c->fs) sh_compile_error(c->h, "gl_PointSize is only available in the vertex shader");
        t = asf(c, rv);
        emit(c, "STO 1 r%d", N(c, t));
        return 1;
    }
    if (!strcmp(name, "gl_FragDepth") || !strcmp(name, "gl_FragDepthEXT")) {
        t = asf(c, rv);
        emit(c, "STOZ r%d", N(c, t));
        return 1;
    }
    return 0;
}

static void store(Cg *c, Node *lhs, Val *rv)
{
    lhs = fd(c, lhs);
    if (lhs->kind == N_VAR) {
        const char *name = lhs->s, *tt;
        DictEnt *vy;
        Val *t, *slot;
        if (!dict_find(&c->shadow, name) && store_special(c, name, rv)) return;
        if (!c->fs && (vy = dict_find(&c->varyings, name)) != NULL && !dict_find(&c->shadow, name)) {
            t = asf(c, rv);
            emit(c, "STV %L r%d", ((Sym *)vy->p)->slot, N(c, t));
            return;
        }
        if (is_mat(rv) || is_struct(rv) || is_array(rv)) {
            copy_val(c, eval_lvalue(c, lhs), rv);
            return;
        }
        tt = local_type(c, name);
        if (tt && !strcmp(tt, "int")) rv = asi(c, rv);
        else if (tt) rv = asf(c, rv);
        slot = get_local_reg(c, name, TY(c, rv));
        emit(c, "MOV r%d r%d", N(c, slot), N(c, rv));
        return;
    }
    if (lhs->kind == N_SWIZZLE) {
        Node *base = fd(c, lhs->a);
        const char *mask = lhs->s;
        Val *target;
        while (base->kind == N_SWIZZLE && !is_struct(eval_lvalue(c, base->a))) {      /* v.wz.y = rv */
            size_t k, n = strlen(mask);
            char *m = (char *)sh_alloc(c->h, n + 1);
            for (k = 0; k < n; k++) m[k] = base->s[swz_index(c, mask[k])];
            m[n] = 0;
            mask = m;
            base = fd(c, base->a);
        }
        target = eval_lvalue(c, base);
        if (is_struct(target)) {
            DictEnt *m = dict_find(target->u.st.members, mask);
            Val *cur;
            if (!m) sh_compile_error(c->h, "struct %s has no member '%s'", target->u.st.sname, mask);
            cur = (Val *)m->p;
            if (is_reg(cur)) {
                rv = !strcmp(cur->ty, "int") ? asi(c, rv) : asf(c, rv);
                emit(c, "MOV r%d r%d", cur->u.r.n, N(c, rv));
            }
            else {
                if (!cur) m->p = cur = like(c, rv);
                copy_val(c, cur, rv);
            }
            return;
        }
        masked_write(c, target, mask, asf(c, rv));      /* a vector holds its ints as floats: i.x += 1 is an IADD */
        return;
    }
    if (lhs->kind == N_INDEX) {
        double ci;
        int known = fold_const(c, lhs->b, &ci);
        Val *tgt, *base2;
        if (lhs->a->kind == N_SWIZZLE && !is_struct(eval_lvalue(c, lhs->a->a))) {     /* v.zy[1] = rv */
            Node t;
            char m[2];
            long long i;
            if (!known) sh_compile_error(c->h, "array index must be a constant expression");
            i = py_int(c, ci);
            if (i < 0 || i >= (long long)strlen(lhs->a->s)) sh_py_error(c->h, "IndexError: string index out of range");
            memset(&t, 0, sizeof t);
            m[0] = lhs->a->s[i]; m[1] = 0;
            t.kind = N_SWIZZLE; t.a = lhs->a->a; t.s = sh_strdup(c->h, m);
            store(c, &t, rv);
            return;
        }
        base2 = eval_lvalue(c, lhs->a);
        if (!known) {
            dyn_write(c, base2, lhs->b, is_reg(rv) ? asf(c, rv) : rv);
            return;
        }
        if (is_reg(base2)) {
            char m[2];
            Val *t = asf(c, rv);
            m[0] = xyzw(c, py_int(c, ci)); m[1] = 0;
            masked_write(c, base2, m, t);
            return;
        }
        tgt = eval_lvalue(c, lhs);
        if (is_mat(rv) || is_struct(rv)) {
            if (is_mat(rv) != is_mat(tgt) || is_struct(rv) != is_struct(tgt)) sh_compile_error(c->h, "cannot assign a matrix to an array element here");
            copy_val(c, tgt, rv);
            return;
        }
        rv = !strcmp(TY(c, tgt), "int") ? asi(c, rv) : asf(c, rv);
        emit(c, "MOV r%d r%d", N(c, tgt), N(c, rv));
        return;
    }
    sh_compile_error(c->h, "unsupported assignment target");
}

/* registers for an aggregate member of a local struct, at its first write: 0, as the struct's other members */
static Val *member_new(Cg *c, const char *sname, const char *mname)
{
    StructDef *s = (StructDef *)dict_find(&c->structs, sname)->p;
    int i;
    for (i = 0; i < s->n; i++) {
        Member *m = &s->m[i];
        Val *v;
        if (strcmp(m->name, mname)) continue;
        if (m->arr) {
            long long n = arrsize(c, m->arr), j;
            Val **elems = new_vals(c, n);
            for (j = 0; j < n; j++) elems[j] = fresh(c, m->ty);
            v = mk_array(c, m->ty, n, 1, 0, elems);
        }
        else v = fresh(c, m->ty);
        zero_val(c, v);
        return v;
    }
    return NULL;
}

/* the value of a local the lvalue e names, an aggregate member of a local struct made on the way (NULL: no such local) */
static Val *lv_struct(Cg *c, Node *e)
{
    Val *r = NULL;
    SH_ENTER(c->h, 1);
    if (e->kind == N_VAR) {
        Local *l = local_find(c, e->s);
        r = l ? l->v : NULL;
    }
    else if (e->kind == N_SWIZZLE) {
        Val *st = lv_struct(c, e->a);
        DictEnt *m = is_struct(st) ? dict_find(st->u.st.members, e->s) : NULL;
        if (m) {
            if (!m->p) m->p = member_new(c, st->u.st.sname, e->s);
            r = (Val *)m->p;
        }
    }
    else if (e->kind == N_INDEX) {
        Val *a = lv_struct(c, e->a);
        double ci;
        if (is_array(a) && a->u.a.local == 1 && fold_const(c, e->b, &ci)) {
            long long i = py_int(c, ci);
            if (i >= 0 && i < a->u.a.size) r = a->u.a.elems[i];
        }
    }
    SH_LEAVE(c->h, 1);
    return r;
}

static Val *eval_lvalue_(Cg *c, Node *e)
{
    e = fd(c, e);
    if (e->kind == N_VAR) {
        Local *l = local_find(c, e->s);
        if (l) return l->v;
    }
    if (e->kind == N_INDEX) {
        double ci;
        Val *arr;
        long long i;
        if (!fold_const(c, e->b, &ci)) sh_compile_error(c->h, "array index must be a constant expression");
        lv_struct(c, e->a);                     /* s.b[0] = x: s.b gets its registers at its first write */
        arr = eval(c, e->a);
        i = py_int(c, ci);
        if (is_array(arr) && arr->u.a.local != 1 && e->a->kind == N_SWIZZLE) {
            /* a struct copied from a uniform shares its read-only array until the first write, which would otherwise
               cost a register per element in every shader that only reads it */
            Val *st = eval_lvalue(c, e->a->a);
            if (is_struct(st)) {
                Val **elems = new_vals(c, arr->u.a.size), *l;
                long long j;
                for (j = 0; j < arr->u.a.size; j++) elems[j] = fresh(c, arr->u.a.ety);
                l = mk_array(c, arr->u.a.ety, arr->u.a.size, 1, 0, elems);
                copy_val(c, l, arr);
                dict_put(c->h, st->u.st.members, e->a->s)->p = l;
                arr = l;
            }
        }
        if (is_array(arr) && arr->u.a.local == 1) {
            long long j = i < 0 ? i + arr->u.a.size : i, sz = arr->u.a.size < 0 ? 0 : arr->u.a.size;
            if (j < 0 || j >= sz) sh_py_error(c->h, "IndexError: list index out of range");
            return arr->u.a.elems[j];
        }
        if (is_mat(arr)) return col(c, arr, i);
        sh_compile_error(c->h, "cannot take address of this index target");
    }
    if (e->kind == N_SWIZZLE) {
        Val *bv = eval_lvalue(c, e->a);
        if (is_struct(bv)) return member_get(c, bv, e->s);
    }
    sh_compile_error(c->h, "lvalue must be a local variable");
}

static Val *eval_lvalue(Cg *c, Node *e)
{
    Val *v;
    SH_ENTER(c->h, 1);
    v = eval_lvalue_(c, e);
    SH_LEAVE(c->h, 1);
    return v;
}

/* ---------------- control-flow analysis ---------------- */
static int refs_runtime(Cg *c, Node *e)
{
    int r = 0, i;
    if (!e) return 0;
    SH_ENTER(c->h, 2);
    switch (e->kind) {
    case N_VAR:
        r = dict_find(&c->uniforms, e->s) || dict_find(&c->varyings, e->s) || dict_find(&c->attribs, e->s);
        break;
    case N_SWIZZLE: case N_POSTOP: case N_UNARY: r = refs_runtime(c, e->a); break;
    case N_INDEX: case N_BIN: case N_ASSIGN: r = refs_runtime(c, e->a) || refs_runtime(c, e->b); break;
    case N_TERNARY: r = refs_runtime(c, e->a) || refs_runtime(c, e->b) || refs_runtime(c, e->c); break;
    case N_CALL: case N_SEQ:
        for (i = 0; i < e->nlist && !r; i++) r = refs_runtime(c, e->list[i]);
        break;
    default: break;
    }
    SH_LEAVE(c->h, 2);
    return r;
}

static int has_loopctl(Cg *c, Node *s)
{
    int r = 0, i;
    if (!s) return 0;
    SH_ENTER(c->h, 2);
    switch (s->kind) {
    case S_BREAK: case S_CONTINUE: r = 1; break;
    case S_BLOCK: case S_MULTI:
        for (i = 0; i < s->nlist && !r; i++) r = has_loopctl(c, s->list[i]);
        break;
    case S_IF: r = has_loopctl(c, s->b) || (s->c && has_loopctl(c, s->c)); break;
    default: break;
    }
    SH_LEAVE(c->h, 2);
    return r;
}

/* A long constant-bounded for loop is a hardware loop, not an unrolled one (glslc.py: HW_FOR_CTL, _lit, _for_trips, _needs_const, _long_for — the reasons are there). Decided in the pre-pass from the
 * loop's text alone and kept in the statement (hwloop): the pre-pass's marks and the code generator must agree. */
enum { HW_FOR_CTL = 16, HW_FOR_PLAIN = 64 };

/* a literal, a signed one, or a global scalar constant that no local hides */
static int lit_num(Cg *c, Node *e, double *v)
{
    if (!e) return 0;
    if (e->kind == N_NUM) { *v = e->num; return 1; }
    if (e->kind == N_UNARY && (!strcmp(e->s, "-") || !strcmp(e->s, "+"))) {
        if (!lit_num(c, e->a, v)) return 0;
        if (e->s[0] == '-') *v = -*v;
        return 1;
    }
    if (e->kind == N_VAR) {
        ScanScope *s;
        DictEnt *d = dict_find(&c->const_env, e->s);
        if (!d) return 0;
        for (s = c->scan; s; s = s->up) if (dict_find(&s->names, e->s)) return 0;
        *v = d->d;
        return 1;
    }
    return 0;
}

/* lit_num for the code generator: the value in the scopes as they are (a parameter hides a global constant, which the pre-pass cannot see; glslc._lit_now) */
static int lit_now(Cg *c, Node *e, double *v) { return e && fold_const(c, e, v); }

/* the pass count of `for (int|float i = a; i < b; i++)` and its kin (<= > >= !=, --, += c, -= c); -1 if the loop is not of that form or runs beyond 4096 passes. now: with the code generator's values */
static int for_trips(Cg *c, Node *init, Node *cond, Node *post, int now)
{
    double v, lim, step;
    const char *name, *op;
    int n = 0;
    int (*lit)(Cg *, Node *, double *) = now ? lit_now : lit_num;
    if (!init || init->kind != S_LDECL || (strcmp(init->ty, "int") && strcmp(init->ty, "float")) || !init->a || init->b) return -1;
    name = init->s;
    if (!lit(c, init->a, &v) || !post) return -1;
    if (cond->kind != N_BIN || (strcmp(cond->s, "<") && strcmp(cond->s, "<=") && strcmp(cond->s, ">") && strcmp(cond->s, ">=") && strcmp(cond->s, "!="))
        || cond->a->kind != N_VAR || strcmp(cond->a->s, name)) return -1;
    if (post->kind == N_POSTOP && post->a->kind == N_VAR && !strcmp(post->a->s, name)) step = !strcmp(post->s, "++") ? 1.0 : -1.0;
    else if (post->kind == N_ASSIGN && (!strcmp(post->s, "+=") || !strcmp(post->s, "-=")) && post->a->kind == N_VAR && !strcmp(post->a->s, name)) {
        if (!lit(c, post->b, &step)) return -1;
        if (post->s[0] == '-') step = -step;
    }
    else return -1;
    if (!lit(c, cond->b, &lim)) return -1;
    op = cond->s;
    while (n <= 4096 && (!strcmp(op, "<") ? v < lim : !strcmp(op, "<=") ? v <= lim : !strcmp(op, ">") ? v > lim : !strcmp(op, ">=") ? v >= lim : v != lim)) { v += step; n++; }
    return n <= 4096 ? n : -1;
}

/* does the loop's body need its index as a constant: in an array index, or handed to a function of the shader (which may index with its parameter)? By name */
static int needs_const(Cg *c, Node *e, const char *name, int inside)
{
    int r = 0, i;
    if (!e || e->kind == N_REG) return 0;
    SH_ENTER(c->h, 2);
    if (e->kind == N_VAR) r = inside && !strcmp(e->s, name);
    else if (e->kind == N_INDEX) r = needs_const(c, e->a, name, inside) || needs_const(c, e->b, name, 1);
    else {
        int in2 = inside || (e->kind == N_CALL && dict_find(&c->funcs, e->fn ? e->fn->name : e->s) != NULL);
        r = needs_const(c, e->a, name, in2) || needs_const(c, e->b, name, in2) || needs_const(c, e->c, name, in2) || needs_const(c, e->d, name, in2);
        for (i = 0; i < e->nlist && !r; i++) r = needs_const(c, e->list[i], name, in2);
    }
    SH_LEAVE(c->h, 2);
    return r;
}

static int has_loopctl(Cg *c, Node *s);

/* the pass count of a for loop that is a hardware loop, 0 for any other (kept in the statement: the code generator's _lit could see other names) */
static int long_for(Cg *c, Node *s)
{
    int n = for_trips(c, s->a, s->b, s->c, 0);
    if (n < 0 || n <= (has_loopctl(c, s->d) ? HW_FOR_CTL : HW_FOR_PLAIN)) return 0;
    return needs_const(c, s->d, s->a->s, 0) ? 0 : n;
}

static void mark_decls(Node *s)
{
    int i;
    if (!s) return;
    if (s->kind == S_LDECL) s->cond = 1;
    else if (s->kind == S_MULTI) for (i = 0; i < s->nlist; i++) mark_decls(s->list[i]);
}

static void mark_base(Cg *c, Node *e)
{
    ScanScope *s;
    while (e->kind == N_SWIZZLE || e->kind == N_INDEX) e = e->a;
    if (e->kind != N_VAR) return;
    for (s = c->scan; s; s = s->up) {
        DictEnt *d = dict_find(&s->names, e->s);
        if (d) { ((Node *)d->p)->cond = 1; return; }
    }
}

/* everything an expression run under a lane mask can write */
static void cond_expr(Cg *c, Node *e)
{
    int i;
    if (!e) return;
    SH_ENTER(c->h, 1);
    if (e->kind == N_ASSIGN || e->kind == N_POSTOP) mark_base(c, e->a);
    else if (e->kind == N_CALL) {
        DictEnt *fe = dict_find(&c->funcs, e->fn ? e->fn->name : e->s);
        if (fe) {
            Func *f = (Func *)fe->p;
            for (i = 0; i < e->nlist && i < f->nparams; i++) if (!strcmp(f->params[i].q, "out") || !strcmp(f->params[i].q, "inout")) mark_base(c, e->list[i]);
        }
    }
    if (e->kind != N_REG) {
        cond_expr(c, e->a); cond_expr(c, e->b); cond_expr(c, e->c); cond_expr(c, e->d);
        for (i = 0; i < e->nlist; i++) cond_expr(c, e->list[i]);
    }
    SH_LEAVE(c->h, 1);
}

/* what a ?: side or the right of && and || writes is written under a lane mask too */
static void arms(Cg *c, Node *e)
{
    int i;
    if (!e || e->kind == N_REG) return;
    SH_ENTER(c->h, 1);
    if (e->kind == N_TERNARY) { cond_expr(c, e->b); cond_expr(c, e->c); }
    else if (e->kind == N_BIN && (!strcmp(e->s, "&&") || !strcmp(e->s, "||"))) cond_expr(c, e->b);
    arms(c, e->a); arms(c, e->b); arms(c, e->c); arms(c, e->d);
    for (i = 0; i < e->nlist; i++) arms(c, e->list[i]);
    SH_LEAVE(c->h, 1);
}

/* Mark the declarations written under RUNTIME control flow. Constant propagation is invalid across it (the write is conditional), so such scalars must be real masked registers.
 * By declaration, not by name: c->scan holds the declarations in scope, and two loops called i have nothing to do with each other */
static void scan_cond_assigns(Cg *c, Node *s, int incond)
{
    ScanScope in;
    int i;
    if (!s) return;
    SH_ENTER(c->h, 1);
    switch (s->kind) {
    case S_LDECL: case S_EXPR: case S_RETURN: case S_IF: case S_WHILE: case S_DOWHILE: arms(c, s->a); break;
    case S_FOR: arms(c, s->b); arms(c, s->c); break;
    default: break;
    }
    switch (s->kind) {
    case S_BLOCK:
        memset(&in, 0, sizeof in);
        in.up = c->scan; c->scan = &in;
        for (i = 0; i < s->nlist; i++) scan_cond_assigns(c, s->list[i], incond);
        c->scan = in.up;
        break;
    case S_MULTI:
        for (i = 0; i < s->nlist; i++) scan_cond_assigns(c, s->list[i], incond);
        break;
    case S_LDECL:
        if (incond) cond_expr(c, s->a);
        dict_put(c->h, &c->scan->names, s->s)->p = s;
        break;
    case S_IF:
        if (incond) cond_expr(c, s->a);
        scan_cond_assigns(c, s->b, 1);
        scan_cond_assigns(c, s->c, 1);
        break;
    case S_FOR: {
        int rt = refs_runtime(c, s->b) || refs_runtime(c, s->c);      /* runtime bound or step -> real (non-unrolled) loop */
        memset(&in, 0, sizeof in);
        in.up = c->scan; c->scan = &in;
        if (!rt && (s->hwloop = long_for(c, s)) != 0) rt = 1;      /* a long constant loop: the hardware loop too (hwloop = its pass count) */
        scan_cond_assigns(c, s->a, incond);
        if (rt) {
            mark_decls(s->a);
            cond_expr(c, s->c);
        }
        scan_cond_assigns(c, s->d, incond || rt || has_loopctl(c, s->d));
        c->scan = in.up;
        break;
    }
    case S_WHILE: {
        int rt = refs_runtime(c, s->a) || has_loopctl(c, s->b);
        scan_cond_assigns(c, s->b, incond || rt);
        break;
    }
    case S_DOWHILE:
        scan_cond_assigns(c, s->b, 1);
        break;
    default:
        if (incond) cond_expr(c, s);
        break;
    }
    SH_LEAVE(c->h, 1);
}

static void scan_body(Cg *c, Node *body)
{
    ScanScope top;
    memset(&top, 0, sizeof top);
    c->scan = &top;
    scan_cond_assigns(c, body, 0);
    c->scan = NULL;
}

/* ---------------- statements ---------------- */
static void loop_push(Cg *c, int hw, Val *brk, Val *cont)
{
    if (c->nloop == c->loopcap) {
        int cap = c->loopcap ? c->loopcap * 2 : 8;
        LoopCtl *nl = (LoopCtl *)sh_alloc(c->h, (size_t)cap * sizeof(LoopCtl));
        if (c->nloop) memcpy(nl, c->loopctl, (size_t)c->nloop * sizeof(LoopCtl));
        c->loopctl = nl; c->loopcap = cap;
    }
    c->loopctl[c->nloop].hw = hw; c->loopctl[c->nloop].brk = brk; c->loopctl[c->nloop].cont = cont;
    c->nloop++;
}

static Node *mk_node(Cg *c, NodeKind k) { Node *n = (Node *)sh_zalloc(c->h, sizeof(Node)); n->kind = k; return n; }
static Node *nreg(Cg *c, Val *v) { Node *n = mk_node(c, N_REG); n->reg = v; return n; }
static Node *nnum(Cg *c, double v) { Node *n = mk_node(c, N_NUM); n->num = v; return n; }
static Node *nbin(Cg *c, const char *op, Node *a, Node *b) { Node *n = mk_node(c, N_BIN); n->s = op; n->a = a; n->b = b; return n; }
static Node *nexpr(Cg *c, Node *e) { Node *n = mk_node(c, S_EXPR); n->a = e; return n; }

/* A vertex shader may read a varying back or write part of one (v = a; v.y = 1.0 - v.y). STV writes a whole varying and LDV reads attributes, so those, and arrays and matrices, live in a local until the end of main.
 * So do gl_Position, gl_FragColor and the like when read or written in part (STO too writes a whole register) */
static const char *const SPECIAL[2][3] = { { "gl_Position", "gl_PointSize", "" }, { "gl_FragColor", "gl_FragDepth", "gl_FragDepthEXT" } };      /* vec4, float, float */
static int is_special(Cg *c, const char *n) { return !strcmp(n, SPECIAL[c->fs][0]) || !strcmp(n, SPECIAL[c->fs][1]) || !strcmp(n, SPECIAL[c->fs][2]); }

static void vary_uses(Cg *c, Node *n)
{
    int i;
    if (!n) return;
    SH_ENTER(c->h, 1);
    if (n->kind == N_ASSIGN && !strcmp(n->s, "=") && n->a->kind == N_VAR) vary_uses(c, n->b);      /* a whole write: no use */
    else if (n->kind == N_VAR) {
        if (dict_find(&c->varyings, n->s) || is_special(c, n->s)) dict_put(c->h, &c->vary_used, n->s);
        else if (!strcmp(n->s, "gl_FragData")) dict_put(c->h, &c->vary_used, "gl_FragColor");
    }
    else if (n->kind != N_REG) {
        vary_uses(c, n->a); vary_uses(c, n->b); vary_uses(c, n->c); vary_uses(c, n->d);
        for (i = 0; i < n->nlist; i++) vary_uses(c, n->list[i]);
    }
    SH_LEAVE(c->h, 1);
}

static void shadow_varyings(Cg *c, Node *body)
{
    int i;
    vary_uses(c, body);
    for (i = 0; i < c->funcs.n; i++) if (c->funcs.ent[i].live) vary_uses(c, ((Func *)c->funcs.ent[i].p)->body);
    c->nozero = 1;                              /* outputs: not variables of the source's, left as they were */
    for (i = 0; i < c->varyings.n; i++) {
        const char *name;
        Sym *v;
        if (!c->varyings.ent[i].live) continue;
        name = c->varyings.ent[i].key; v = (Sym *)c->varyings.ent[i].p;
        if (!c->fs && (v->has3 || mat_n(v->ty) || dict_find(&c->vary_used, name) || c->h->vmap)) {
            Node *d = mk_node(c, S_LDECL);
            dict_put(c->h, &c->shadow, name);
            d->ty = v->ty; d->s = name;
            if (v->has3) d->b = nnum(c, (double)v->n);
            c->nozero = !(v->has3 || mat_n(v->ty));    /* an array or a matrix starts at 0: a write at a run-time index reads every element */
            run_stmt(c, d);
            c->nozero = 1;
        }
    }
    for (i = 0; i < 3; i++) {
        if (dict_find(&c->vary_used, SPECIAL[c->fs][i])) {
            Node *d = mk_node(c, S_LDECL);
            dict_put(c->h, &c->shadow, SPECIAL[c->fs][i]);
            d->ty = i ? "float" : "vec4"; d->s = SPECIAL[c->fs][i];
            run_stmt(c, d);
        }
    }
    c->nozero = 0;
}

static void flush_varyings(Cg *c)
{
    Val *rows[8] = { 0 }, *part[8][4] = { { 0 } };              /* Appendix A.7: the vectors of a row, put together here; part[row][lane] = what starts there */
    int i, j, k;
    for (i = 0; i < c->shadow.n; i++) {
        const char *name;
        Val *val;
        long long slot, ne;
        if (!c->shadow.ent[i].live) continue;
        name = c->shadow.ent[i].key;
        val = ((Local *)dict_find(c->locals[0], name)->p)->v;
        if (store_special(c, name, val)) continue;
        slot = ((Sym *)dict_find(&c->varyings, name)->p)->slot;
        ne = is_array(val) ? val->u.a.size : 1;
        for (j = 0; j < ne; j++) {
            Val *el = is_array(val) ? val->u.a.elems[j] : val;
            int nc = is_mat(el) ? el->u.m.ncols : 1;
            for (k = 0; k < nc; k++) {
                Val *t = asf(c, is_mat(el) ? el->u.m.cols[k] : el);
                if (!c->h->vmap) emit(c, "STV %L r%d", slot, N(c, t));
                else if (slot < c->h->nvmap && c->h->vmap[slot] >= 0) part[(c->h->vmap[slot] >> 2) & 7][c->h->vmap[slot] & 3] = t;
                slot++;
            }
        }
    }
    for (i = 0; i < 8; i++) {                                   /* lane x first (the row can become its register), then zeros where nothing is */
        int cover = 0, comp;
        for (comp = 0; comp < 4; comp++) {
            Val *t = part[i][comp];
            int n;
            char m[5];
            if (!t) continue;
            if (!rows[i]) rows[i] = alloc(c, "vec4");
            n = ncomp(t->ty);
            memcpy(m, "xyzw" + comp, (size_t)n); m[n] = 0;
            masked_write(c, rows[i], m, t);
            cover |= ((1 << n) - 1) << comp;
        }
        if (rows[i] && cover != 15) {
            char m[5];
            int n = 0;
            for (comp = 0; comp < 4; comp++) if (!(cover >> comp & 1)) m[n++] = "xyzw"[comp];
            m[n] = 0;
            emit(c, "MOV.%s r%d r%d", m, rows[i]->u.r.n, zeroreg(c)->u.r.n);
        }
    }
    for (i = 0; i < 8; i++) if (rows[i]) emit(c, "STV %d r%d", i, rows[i]->u.r.n);
}

/* BREAK or CONT when a statement is nothing but a break / continue (in blocks of one statement), else NULL (glslc._lone_ctl) */
static const char *lone_ctl(Node *s)
{
    while (s && s->kind == S_BLOCK && s->nlist == 1) s = s->list[0];
    if (!s) return NULL;
    return s->kind == S_BREAK ? "BREAK" : s->kind == S_CONTINUE ? "CONT" : NULL;
}

/* `if (c) break;` in a hardware loop: SETP c + BREAK, `a || b` a pair each (glslc._ctl_if, the reasons are there) */
static void ctl_if(Cg *c, Node *e, const char *op)
{
    SH_ENTER(c->h, 1);
    if (e->kind == N_BIN && !strcmp(e->s, "||")) { ctl_if(c, e->a, op); ctl_if(c, e->b, op); }
    else {
        Val *cond = eval(c, e), *z = zeroreg(c);
        emit(c, "SETP SNE r%d r%d", N(c, cond), z->u.r.n);
        emit(c, "%s", op);
    }
    SH_LEAVE(c->h, 1);
}

static void run_if(Cg *c, Node *cond_e, Node *th, Node *el)
{
    Val *cond, *z;
    const char *lelse, *lend;
    if (!el && c->nloop && c->loopctl[c->nloop - 1].hw) {
        const char *op = lone_ctl(th);
        if (op) { ctl_if(c, cond_e, op); return; }
    }
    cond = eval(c, cond_e);
    SH_ENTER(c->h, 1);
    z = zeroreg(c);
    emit(c, "SETP SNE r%d r%d", N(c, cond), z->u.r.n);
    lelse = newlabel(c, "Lelse"); lend = newlabel(c, "Lend");
    emit(c, "IF %s", lelse);
    run_stmt(c, th);
    emit(c, "%s:", lelse);
    emit(c, "ELSE %s", lend);
    if (el) run_stmt(c, el);
    emit(c, "%s:", lend);
    emit(c, "ENDIF");
    SH_LEAVE(c->h, 1);
}

static void run_guarded_body(Cg *c, Node *body, Val *brk, Val *cont)
{
    Node **stmts, *one[1], *guard;
    int n, i;
    if (body->kind == S_BLOCK) { stmts = body->list; n = body->nlist; }
    else { one[0] = body; stmts = one; n = 1; }
    guard = nbin(c, "&&", nbin(c, "==", nreg(c, brk), nnum(c, 0.0)), nbin(c, "==", nreg(c, cont), nnum(c, 0.0)));
    for (i = 0; i < n; i++) run_if(c, guard, stmts[i], NULL);
}

static void unroll(Cg *c, Node *cond, Node *post, Node *body, int ctl, const char *kind)
{
    Val *brk = NULL;
    int it = 0;
    if (ctl) {
        Val *k;
        brk = alloc(c, "float");
        k = kf(c, 0.0);
        emit(c, "MOV r%d r%d", brk->u.r.n, k->u.r.n);
    }
    while (it < 4096) {
        double v;
        if (!fold_const(c, cond, &v)) sh_compile_error(c->h, "%s loop needs constant-evaluable bounds (ES2)", kind);
        if (v == 0.0) return;
        if (ctl) {
            Val *cont = alloc(c, "float"), *k = kf(c, 0.0);
            emit(c, "MOV r%d r%d", cont->u.r.n, k->u.r.n);
            loop_push(c, 0, brk, cont);
            run_guarded_body(c, body, brk, cont);
            c->nloop--;
        }
        else run_stmt(c, body);
        if (post) { Node *p = nexpr(c, post); run_stmt(c, p); }
        it++;
    }
    sh_compile_error(c->h, "%s loop did not terminate within 4096 unrolled iterations", kind);
}

static void runtime_loop(Cg *c, Node *cond, Node *post, Node *body)
{
    Val *first = NULL, *cv, *nc, *z, *k;
    if (post) {
        first = alloc(c, "float");
        k = kf(c, 1.0);
        emit(c, "MOV r%d r%d", first->u.r.n, k->u.r.n);
    }
    emit(c, "LOOP 4096");
    loop_push(c, 1, NULL, NULL);
    hw_loop(c, 1);
    if (post) {
        run_if(c, nbin(c, "==", nreg(c, first), nnum(c, 0.0)), nexpr(c, post), NULL);
        k = kf(c, 0.0);
        emit(c, "MOV r%d r%d", first->u.r.n, k->u.r.n);
    }
    cv = eval(c, cond);
    nc = alloc(c, "float");
    z = zeroreg(c);
    emit(c, "SET SEQ r%d r%d r%d", nc->u.r.n, N(c, cv), z->u.r.n);
    z = zeroreg(c);
    emit(c, "SETP SNE r%d r%d", nc->u.r.n, z->u.r.n);
    emit(c, "BREAK");
    run_stmt(c, body);
    c->nloop--;
    emit(c, "ENDL");
    hw_loop(c, -1);
}

static void written_base(Cg *c, Node *t, Dict *out)
{
    while (t->kind == N_SWIZZLE || t->kind == N_INDEX) t = t->a;
    if (t->kind == N_VAR) dict_put(c->h, out, t->s);
}

/* the names an expression or statement assigns, in order */
static void written(Cg *c, Node *e, Dict *out)
{
    int i;
    if (!e || e->kind == N_REG) return;
    SH_ENTER(c->h, 1);
    if (e->kind == N_ASSIGN || e->kind == N_POSTOP) written_base(c, e->a, out);
    else if (e->kind == N_CALL) {
        DictEnt *fe = dict_find(&c->funcs, e->fn ? e->fn->name : e->s);
        if (fe) {
            Func *f = (Func *)fe->p;
            for (i = 0; i < e->nlist && i < f->nparams; i++)
                if (!strcmp(f->params[i].q, "out") || !strcmp(f->params[i].q, "inout")) { written_base(c, e->list[i], out); written(c, e->list[i], out); }
        }
    }
    written(c, e->a, out); written(c, e->b, out); written(c, e->c, out); written(c, e->d, out);
    for (i = 0; i < e->nlist; i++) written(c, e->list[i], out);
    SH_LEAVE(c->h, 1);
}

/* does a statement or expression read the variable `name` anywhere (glslc._names) */
static int names(Cg *c, Node *e, const char *name)
{
    int r = 0, i;
    if (!e || e->kind == N_REG) return 0;
    SH_ENTER(c->h, 2);
    if (e->kind == N_VAR) r = !strcmp(e->s, name);
    else {
        r = names(c, e->a, name) || names(c, e->b, name) || names(c, e->c, name) || names(c, e->d, name);
        for (i = 0; i < e->nlist && !r; i++) r = names(c, e->list[i], name);
    }
    SH_LEAVE(c->h, 2);
    return r;
}

/* a long constant loop whose body never reads its index: LOOP n counts the passes in the core, no index register, no first-pass flag, no exit test (glslc._counted_loop) */
static void counted_loop(Cg *c, int n, Node *body)
{
    Dict w;
    int i;
    memset(&w, 0, sizeof w);
    written(c, body, &w);
    for (i = 0; i < w.n; i++) if (const_home(c, w.ent[i].key)) promote(c, w.ent[i].key);
    emit(c, "LOOP %d", n);
    loop_push(c, 1, NULL, NULL);
    hw_loop(c, 1);
    run_stmt(c, body);
    c->nloop--;
    emit(c, "ENDL");
    hw_loop(c, -1);
}

/* hw: the pass count of a long constant-bounded for loop that is a hardware loop (long_for, from the pre-pass), else 0 */
static void run_loop(Cg *c, Node *init, Node *cond, Node *post, Node *body, const char *kind, int hw)
{
    int wh = !strcmp(kind, "while"), always, unknown;
    double cv;
    scope_push(c, new_dict(c));
    if (hw && !names(c, body, init->s) && for_trips(c, init, cond, post, 1) == hw) { counted_loop(c, hw, body); c->nlocals--; return; }     /* the count again from the values here (glslc._run_loop) */
    if (init) run_stmt(c, init);
    always = cond->kind == N_NUM && cond->num != 0.0 && (!post || !strcmp(kind, "for"));
    unknown = !fold_const(c, cond, &cv);
    if (refs_runtime(c, cond) || ((wh || always) && has_loopctl(c, body)) || unknown || hw) {     /* hw: a long constant-bounded for loop (long_for) */
        Dict w;                                                 /* what the loop writes must be real registers */
        int i;
        memset(&w, 0, sizeof w);
        written(c, cond, &w); written(c, post, &w); written(c, body, &w);     /* the condition's writes too: for (; i++ < 20.; ) */
        for (i = 0; i < w.n; i++) if (const_home(c, w.ent[i].key)) promote(c, w.ent[i].key);
        runtime_loop(c, cond, post, body);
    }
    else unroll(c, cond, post, body, has_loopctl(c, body), kind);
    c->nlocals--;
}

/* a `continue` of THIS loop in the statement (not one of a nested loop) */
static int has_continue(Cg *c, Node *s)
{
    int r = 0, i;
    if (!s) return 0;
    SH_ENTER(c->h, 2);
    switch (s->kind) {
    case S_CONTINUE: r = 1; break;
    case S_BLOCK: case S_MULTI:
        for (i = 0; i < s->nlist && !r; i++) r = has_continue(c, s->list[i]);
        break;
    case S_IF: r = has_continue(c, s->b) || (s->c && has_continue(c, s->c)); break;
    default: break;
    }
    SH_LEAVE(c->h, 2);
    return r;
}

static void do_loop(Cg *c, Node *cond, Node *body)
{
    Val *first, *k, *z, *cv, *ncond, *brk;
    const char *lelse, *lend;
    if (!has_continue(c, body)) {
        /* no `continue` in the body: the body, then the condition, once a pass, as written. The form below evaluates the condition before the FIRST body too and only ignores its result —
         * one evaluation too many when the condition writes (do { } while (i++ < n)) */
        emit(c, "LOOP 4096");
        loop_push(c, 1, NULL, NULL);
        hw_loop(c, 1);
        run_stmt(c, body);
        cv = eval(c, cond);
        z = zeroreg(c);
        ncond = alloc(c, "float");
        emit(c, "SET SEQ r%d r%d r%d", ncond->u.r.n, N(c, cv), z->u.r.n);
        emit(c, "SETP SNE r%d r%d", ncond->u.r.n, z->u.r.n);
        emit(c, "BREAK");
        c->nloop--;
        emit(c, "ENDL");
        hw_loop(c, -1);
        return;
    }
    /* with a `continue`: the test at the loop top, worked out only where a pass has run (glslc._do_loop) */
    first = alloc(c, "float");
    k = kf(c, 1.0);
    emit(c, "MOV r%d r%d", first->u.r.n, k->u.r.n);
    emit(c, "LOOP 4096");
    loop_push(c, 1, NULL, NULL);
    hw_loop(c, 1);
    z = zeroreg(c);
    brk = alloc(c, "float");
    emit(c, "MOV r%d r%d", brk->u.r.n, z->u.r.n);
    emit(c, "SETP SEQ r%d r%d", first->u.r.n, z->u.r.n);
    lelse = newlabel(c, "Lelse"); lend = newlabel(c, "Lend");
    emit(c, "IF %s", lelse);
    cv = eval(c, cond);
    emit(c, "SET SEQ r%d r%d r%d", brk->u.r.n, N(c, cv), z->u.r.n);
    emit(c, "%s:", lelse);
    emit(c, "ELSE %s", lend);
    emit(c, "%s:", lend);
    emit(c, "ENDIF");
    emit(c, "MOV r%d r%d", first->u.r.n, z->u.r.n);
    emit(c, "SETP SNE r%d r%d", brk->u.r.n, z->u.r.n);
    emit(c, "BREAK");
    run_stmt(c, body);
    c->nloop--;
    emit(c, "ENDL");
    hw_loop(c, -1);
}

static void run_ldecl(Cg *c, Node *s)
{
    const char *ty = s->ty, *name = s->s;
    Node *init = s->a, *arr = s->b;
    Dict *scope = c->locals[c->nlocals - 1];
    Val *r;
    int zero = !init && !c->nozero;             /* no value given: 0 (zero_val); not the outputs' shadows */
    if (!strcmp(ty, "sampler2D") || !strcmp(ty, "samplerCube") || !strcmp(ty, "sampler3D")) return;
    if (dict_find(&c->structs, ty) && !arr) {
        Val *iv = init ? eval(c, init) : NULL, *sv = decl_local_struct(c, ty, iv);
        if (zero) zero_val(c, sv);
        scope = c->locals[c->nlocals - 1];
        local_set(c, scope, name, sv, ty);
        return;
    }
    if (arr) {
        double d;
        long long n, i;
        Val **elems;
        if (!fold_const(c, arr, &d)) sh_py_error(c->h, "TypeError: int() argument must be a number, not 'NoneType'");
        n = py_int(c, d);
        if (n > (1LL << 26)) sh_py_error(c->h, "out of memory (local array too large)");
        elems = new_vals(c, n);
        for (i = 0; i < n; i++) elems[i] = fresh(c, ty);
        r = mk_array(c, ty, n, 1, 0, elems);
        if (zero) zero_val(c, r);
        scope = c->locals[c->nlocals - 1];
        local_set(c, scope, name, r, sh_fmt(c->h, "%s[%L]", ty, n));
        return;
    }
    if (is_scalar(ty) && init && !s->cond) {
        double cv;
        if (fold_const(c, init, &cv)) {
            DictEnt *e = dict_put(c->h, c->locals[c->nlocals - 1], name);
            int isi = !strcmp(ty, "int");
            e->p = NULL; e->d = isi ? sh_fold_cast(c->h, "int", cv) : cv; e->i = isi;
            return;
        }
    }
    if (mat_n(ty)) {
        int n, i;
        Val **cols;
        if (init) {
            Val *v = eval(c, init);
            v = clone_val(c, v);
            local_set(c, c->locals[c->nlocals - 1], name, v, ty);
            return;
        }
        n = mat_n(ty);
        cols = new_vals(c, n);
        for (i = 0; i < n; i++) cols[i] = alloc(c, vecn(c, n));
        r = mk_mat(c, cols, n, ty);
        if (zero) zero_val(c, r);
        local_set(c, c->locals[c->nlocals - 1], name, r, ty);
        return;
    }
    r = alloc(c, ty);
    if (init) {                                 /* the initializer still sees an outer x: float x = x + 1.0; */
        Val *rv = eval(c, init);
        rv = !strcmp(ty, "int") ? asi(c, rv) : asf(c, rv);
        emit(c, "MOV r%d r%d", r->u.r.n, N(c, rv));
    } else if (zero) zero_val(c, r);
    local_set(c, c->locals[c->nlocals - 1], name, r, ty);
}

static void run_stmt_(Cg *c, Node *s)
{
    int i;
    switch (s->kind) {
    case S_MULTI:
        for (i = 0; i < s->nlist; i++) run_stmt(c, s->list[i]);
        break;
    case S_BLOCK:
        scope_push(c, new_dict(c));
        for (i = 0; i < s->nlist; i++) run_stmt(c, s->list[i]);
        c->nlocals--;
        break;
    case S_LDECL: run_ldecl(c, s); break;
    case S_EXPR: eval(c, s->a); break;
    case S_IF: run_if(c, s->a, s->b, s->c); break;
    case S_FOR: run_loop(c, s->a, s->b, s->c, s->d, "for", s->hwloop); break;
    case S_WHILE: run_loop(c, NULL, s->a, NULL, s->b, "while", 0); break;
    case S_DOWHILE:
        scope_push(c, new_dict(c));
        {                                                       /* a condition that writes a tracked constant (as run_loop) */
            Dict w;
            memset(&w, 0, sizeof w);
            written(c, s->a, &w);
            for (i = 0; i < w.n; i++) if (const_home(c, w.ent[i].key)) promote(c, w.ent[i].key);
        }
        do_loop(c, s->a, s->b);
        c->nlocals--;
        break;
    case S_DISCARD: emit(c, "DISCARD"); break;
    case S_BREAK:
    case S_CONTINUE:
        if (c->nloop) {
            LoopCtl *lc = &c->loopctl[c->nloop - 1];
            Val *one = kf(c, 1.0);
            if (lc->hw) {
                Val *z = zeroreg(c);
                emit(c, "SETP SNE r%d r%d", one->u.r.n, z->u.r.n);
                emit(c, "%s", s->kind == S_BREAK ? "BREAK" : "CONT");
            }
            else emit(c, "MOV r%d r%d", s->kind == S_BREAK ? lc->brk->u.r.n : lc->cont->u.r.n, one->u.r.n);
        }
        break;
    case S_RETURN:
        if (c->cur_ret) {
            CurRet *cr = c->cur_ret;
            if (s->a) {
                Val *rv = eval(c, s->a);
                if (rv && (rv->kind == V_STRUCT || rv->kind == V_MAT || rv->kind == V_ARRAY)) {
                    if (!cr->flow) cr->holder = rv;
                    else {                              /* one result per lane: registers of its own */
                        if (!cr->holder) cr->holder = like(c, rv);
                        copy_val(c, cr->holder, rv);
                    }
                }
                else {
                    rv = !strcmp(cr->rt, "int") ? asi(c, rv) : asf(c, rv);
                    emit(c, "MOV r%d r%d", cr->retreg->u.r.n, N(c, rv));
                }
            }
            if (!cr->flow) emit(c, "BRA %s", cr->lret);
            else {
                Val *one, *z;
                if (cr->depth) {
                    one = kf(c, 1.0);
                    emit(c, "MOV r%d r%d", cr->left->u.r.n, one->u.r.n);
                    cr->any = 1;
                }
                one = kf(c, 1.0);
                z = zeroreg(c);
                emit(c, "SETP SNE r%d r%d", one->u.r.n, z->u.r.n);
                emit(c, "BREAK");
            }
        }
        break;
    default: break;
    }
}

static void run_stmt(Cg *c, Node *s)
{
    SH_ENTER(c->h, 1);
    run_stmt_(c, s);
    SH_LEAVE(c->h, 1);
}

/* ---------------- the driver: _compile_stage up to the optimizer, and the result tables ---------------- */
static Sym *sym_put(Cg *c, Dict *d, const char *name, long long slot, const char *ty, int has3, long long n)
{
    Sym *s = (Sym *)sh_alloc(c->h, sizeof(Sym));
    s->slot = slot; s->ty = ty; s->has3 = has3; s->n = n;
    dict_put(c->h, d, name)->p = s;
    return s;
}

/* a sampler in a uniform struct: a texture unit of its own, under the name a front end gives it (s.tex, a[1].tex), and the uniform
 * slot it takes, which a load of the struct turns into that unit */
static void struct_samplers(Cg *c, const char *path, const char *ty, long long asz, long long slot)
{
    StructDef *s = (StructDef *)dict_find(&c->structs, ty)->p;
    long long e, ne = asz != NONE_SZ ? asz : 1;
    int i;
    SH_ENTER(c->h, 1);
    for (e = 0; e < ne; e++) {
        const char *p = asz != NONE_SZ ? sh_fmt(c->h, "%s[%L]", path, e) : path;
        for (i = 0; i < s->n; i++) {
            Member *m = &s->m[i];
            long long n = arrsize(c, m->arr);
            const char *q = sh_fmt(c->h, "%s.%s", p, m->name);
            if (!strcmp(m->ty, "sampler2D") || !strcmp(m->ty, "samplerCube") || !strcmp(m->ty, "sampler3D")) {
                sym_put(c, &c->samplers, q, c->next_sampler, m->ty, n != NONE_SZ, n != NONE_SZ ? n : 1);
                dict_put(c->h, &c->sampler_slot, sh_fmt(c->h, "%L", slot))->i = c->next_sampler;
                c->next_sampler = (int)c_range(c, c->next_sampler + (n != NONE_SZ ? n : 1));
            }
            else if (dict_find(&c->structs, m->ty)) struct_samplers(c, q, m->ty, n, slot);
            slot += type_nslots(c, m->ty, n);
        }
    }
    SH_LEAVE(c->h, 1);
}

static ShSym *sym_table(Cg *c, const Dict *d, int *n)
{
    ShSym *t = (ShSym *)sh_alloc(c->h, (size_t)(d->n + 1) * sizeof(ShSym));
    int i, k = 0;
    for (i = 0; i < d->n; i++) {
        const DictEnt *e = &d->ent[i];
        const Sym *s = (const Sym *)e->p;
        t[k].name = e->key; t[k].loc = s->slot; t[k].ty = s->ty; t[k].has_array = s->has3; t[k].array = s->n;
        k++;
    }
    *n = k;
    return t;
}

/* the index an LDV line reads ("LDV r3 16", "LDV.xy r3 16", "LDV r3.y 16.x"): 1 and *at = where its digits start */
static int ldv_index(const char *s, const char *e, const char **at, long long *idx)
{
    const char *p;
    long long v = 0;
    if (e - s < 4 || strncmp(s, "LDV", 3) || (s[3] != ' ' && s[3] != '.')) return 0;
    p = memchr(s, ' ', (size_t)(e - s));
    if (p) p = memchr(p + 1, ' ', (size_t)(e - p - 1));
    if (!p || p + 1 >= e || p[1] < '0' || p[1] > '9') return 0;
    *at = ++p;
    while (p < e && *p >= '0' && *p <= '9' && v < 0x7FFFFFFF) v = v * 10 + (*p++ - '0');
    *idx = v;
    return 1;
}

/* glslc place_attribs: an attribute the program reads is active and takes a location (§2.10.4), a matrix one a column, in a run;
 * a bound one (h->binds) where the link binds it, also two on one, the rest in declaration order the lowest run no active one holds.
 * The code reads them there; the table keeps the active ones, in declaration order */
static void place_attribs(Cg *c, ShOutput *out)
{
    Shc *h = c->h;
    int n = c->attribs.n, nread = c->next_attr + 1, cap = 1, i, j, k, m = 0;
    unsigned char *read = (unsigned char *)sh_zalloc(h, (size_t)nread), *held;
    int *w = (int *)sh_alloc(h, (size_t)(n + 1) * sizeof(int) * 3), *loc = w + n + 1, *act = loc + n + 1;
    int *moved = (int *)sh_alloc(h, (size_t)nread * sizeof(int));
    const char *s, *e, *at;
    long long idx;
    ShSym *t = (ShSym *)sh_alloc(h, (size_t)(n + 1) * sizeof(ShSym));
    SB b;
    for (s = out->sasm; *s; s = *e ? e + 1 : e) {
        e = strchr(s, '\n');
        if (!e) e = s + strlen(s);
        if (ldv_index(s, e, &at, &idx) && idx < nread) read[idx] = 1;
    }
    for (i = 0; i < n; i++) {
        const Sym *y = (const Sym *)c->attribs.ent[i].p;
        w[i] = mat_n(y->ty) ? ncomp2(c, y->ty) : 1;
        act[i] = 0; loc[i] = -1;
        for (k = 0; k < w[i]; k++) if (y->slot + k < nread && read[y->slot + k]) act[i] = 1;
        if (!act[i]) continue;
        cap += w[i];
        for (j = h->nbinds - 1; j >= 0; j--) if (!strcmp(h->binds[j].name, c->attribs.ent[i].key)) { loc[i] = h->binds[j].loc; cap += loc[i] + w[i]; break; }
    }
    held = (unsigned char *)sh_zalloc(h, (size_t)cap);
    for (i = 0; i < n; i++) if (act[i] && loc[i] >= 0) for (k = 0; k < w[i]; k++) held[loc[i] + k] = 1;
    for (i = 0; i < n; i++) {
        int x = 0;
        if (!act[i] || loc[i] >= 0) continue;
        for (;;) {
            for (k = 0; k < w[i] && !(x + k < cap && held[x + k]); k++) ;
            if (k == w[i]) break;
            x++;
        }
        loc[i] = x;
        for (k = 0; k < w[i]; k++) held[x + k] = 1;
    }
    for (i = 0; i < nread; i++) moved[i] = -1;
    for (i = 0; i < n; i++) {
        const Sym *y = (const Sym *)c->attribs.ent[i].p;
        if (!act[i]) continue;
        for (k = 0; k < w[i]; k++) moved[y->slot + k] = loc[i] + k;
        t[m].name = c->attribs.ent[i].key; t[m].loc = loc[i]; t[m].ty = y->ty; t[m].has_array = 0; t[m].array = 0;
        m++;
    }
    sb_init(&b, h, strlen(out->sasm) + 16);
    for (s = out->sasm; *s; s = *e ? e + 1 : e) {
        e = strchr(s, '\n');
        if (!e) e = s + strlen(s);
        if (ldv_index(s, e, &at, &idx)) {
            const char *d = at;
            if (idx >= nread || moved[idx] < 0) sh_py_error(h, "KeyError: %L", idx);
            while (d < e && *d >= '0' && *d <= '9') d++;
            sb_putn(&b, s, (size_t)(at - s));
            sb_putll(&b, moved[idx]);
            sb_putn(&b, d, (size_t)(e - d));
        } else sb_putn(&b, s, (size_t)(e - s));
        if (*e) sb_putc(&b, '\n');
    }
    out->sasm = sb_str(&b);
    out->attribs = t; out->nattribs = m;
}

static void mark_struct(Cg *c, Dict *used, const char *ty)
{
    DictEnt *e = dict_find(&c->structs, ty);
    StructDef *s;
    int i;
    if (!e || dict_find(used, ty)) return;
    dict_put(c->h, used, ty);
    s = (StructDef *)e->p;
    for (i = 0; i < s->n; i++) mark_struct(c, used, s->m[i].ty);
}

static void struct_table(Cg *c, ShOutput *out)
{
    Dict used;
    int i, k = 0;
    memset(&used, 0, sizeof used);
    for (i = 0; i < c->uniforms.n; i++) mark_struct(c, &used, ((Sym *)c->uniforms.ent[i].p)->ty);
    out->structs = (ShStruct *)sh_alloc(c->h, (size_t)(c->structs.n + 1) * sizeof(ShStruct));
    for (i = 0; i < c->structs.n; i++) {
        const DictEnt *e = &c->structs.ent[i];
        StructDef *s = (StructDef *)e->p;
        ShStruct *o;
        long long off = 0;
        int m;
        if (!dict_find(&used, e->key)) continue;
        o = &out->structs[k++];
        o->name = e->key;
        o->nmembers = s->n;
        o->members = (ShMember *)sh_alloc(c->h, (size_t)(s->n + 1) * sizeof(ShMember));
        for (m = 0; m < s->n; m++) {
            long long asz = arrsize(c, s->m[m].arr), nsl = type_nslots(c, s->m[m].ty, asz);
            ShMember *om = &o->members[m];
            om->name = s->m[m].name; om->ty = s->m[m].ty; om->offset = off; om->nslots = nsl;
            om->has_array = asz != NONE_SZ; om->array = asz != NONE_SZ ? asz : 0; om->prec = s->m[m].rprec;
            off += nsl;
        }
        o->nslots = off;
    }
    out->nstructs = k;
}

void sh_compile(Shc *h, const char *src, int fs, int remat, ShOutput *out)
{
    int exts = 0;
    char *pp = sh_preprocess(h, src, fs, &exts);
    int nd, nho, i, depthrange = 0;
    Decl *ho, *ast = sh_parse(h, pp, exts, &nd, &ho, &nho), *mainf = NULL;
    Cg cg, *c = &cg;
    sh_sema(h, ast, nd, fs, exts, &depthrange);
    memset(c, 0, sizeof *c);
    c->h = h; c->fs = fs; c->remat = remat;
    c->next_vary = h->nvary_order;
    for (i = 0; i < nho; i++) {
        StructDef *s = (StructDef *)sh_alloc(h, sizeof(StructDef));
        s->m = ho[i].members; s->n = ho[i].nmembers;
        dict_put(h, &c->structs, ho[i].name)->p = s;
    }
    for (i = 0; i < SH_NMAX; i++) { DictEnt *e = dict_put(h, &c->const_env, SH_GL_MAX[i].name); e->d = SH_GL_MAX[i].v; e->i = 1; }
    for (i = 0; i < nd; i++) {
        if (ast[i].kind == D_STRUCT) {
            StructDef *s = (StructDef *)sh_alloc(h, sizeof(StructDef));
            s->m = ast[i].members; s->n = ast[i].nmembers;
            dict_put(h, &c->structs, ast[i].name)->p = s;
        }
    }
    for (i = 0; i < nd; i++) {
        Decl *d = &ast[i];
        const char *qual = d->qual;
        if (d->kind != D_GDECL) continue;
        if (!strcmp(qual, "attribute")) {
            sym_put(c, &c->attribs, d->name, c->next_attr, d->ty, 0, 0);
            c->next_attr += mat_n(d->ty) ? ncomp2(c, d->ty) : 1;
        }
        else if (!strcmp(qual, "uniform")) {
            if (!strcmp(d->ty, "sampler2D") || !strcmp(d->ty, "samplerCube") || !strcmp(d->ty, "sampler3D")) {
                long long n = d->arr ? c_range(c, arrsize(c, d->arr)) : 1;
                sym_put(c, &c->samplers, d->name, c->next_sampler, d->ty, d->arr != NULL, n);
                c->next_sampler = (int)c_range(c, c->next_sampler + n);
            }
            else if (dict_find(&c->structs, d->ty)) {
                long long asz = arrsize(c, d->arr), n = type_nslots(c, d->ty, asz);
                if (asz != NONE_SZ) sym_put(c, &c->uniforms, d->name, c->next_uniform, d->ty, 1, asz);
                else sym_put(c, &c->uniforms, d->name, c->next_uniform, d->ty, 0, 0);
                struct_samplers(c, d->name, d->ty, asz, c->next_uniform);
                c->next_uniform = c_range(c, c->next_uniform + n);
            }
            else {
                long long per = mat_n(d->ty) ? mat_n(d->ty) : 1;
                if (d->arr) {
                    double v;
                    long long n;
                    if (!fold_const(c, d->arr, &v)) sh_py_error(h, "TypeError: int() argument must be a number, not 'NoneType'");
                    n = c_range(c, py_int(c, v));
                    sym_put(c, &c->uniforms, d->name, c->next_uniform, d->ty, 1, n);
                    c->next_uniform = c_range(c, c->next_uniform + per * n);
                }
                else {
                    sym_put(c, &c->uniforms, d->name, c->next_uniform, d->ty, 0, 0);
                    c->next_uniform = c_range(c, c->next_uniform + per);
                }
            }
        }
        else if (!strcmp(qual, "varying")) {
            int k = -1, o;
            long long asz = arrsize(c, d->arr), n = (mat_n(d->ty) ? ncomp2(c, d->ty) : 1) * (asz != NONE_SZ && asz ? asz : 1);
            for (o = 0; o < h->nvary_order; o++) if (h->vary_order[o] && !strcmp(h->vary_order[o], d->name)) k = o;
            sym_put(c, &c->varyings, d->name, k >= 0 ? k : c->next_vary, d->ty, asz != NONE_SZ, asz != NONE_SZ ? asz : 0);
            if (k < 0) c->next_vary = (int)c_range(c, c->next_vary + n);
        }
        else if (!strcmp(qual, "const")) {
            double cv;
            if (!d->init) sh_compile_error(h, "global const '%s' needs an initializer", d->name);
            if (fold_const(c, d->init, &cv) && is_scalar(d->ty)) {
                DictEnt *e = dict_put(h, &c->const_env, d->name);
                e->i = !strcmp(d->ty, "int");
                e->d = e->i ? sh_fold_cast(h, "int", cv) : cv;
            }
            else {
                GConst *g = (GConst *)sh_alloc(h, sizeof(GConst));
                g->ty = d->ty; g->init = d->init;
                dict_put(h, &c->global_consts, d->name)->p = g;
            }
        }
    }
    if (depthrange) {                   /* the built-in uniform (§7.5): the library fills it */
        StructDef *s = (StructDef *)sh_alloc(h, sizeof(StructDef));
        s->m = SH_DEPTHRANGE; s->n = 3;
        dict_put(h, &c->structs, "gl_DepthRangeParameters")->p = s;
        sym_put(c, &c->uniforms, "gl_DepthRange", c->next_uniform, "gl_DepthRangeParameters", 0, 0);
        c->next_uniform += 3;
    }
    struct_table(c, out);
    for (i = 0; i < nd; i++) {
        Decl *d = &ast[i];
        if (d->kind != D_FUNC) continue;
        if (!strcmp(d->name, "main")) mainf = d;
        else {
            Func *f = (Func *)sh_alloc(h, sizeof(Func));
            f->rt = d->ty; f->params = d->params; f->nparams = d->nparams; f->body = d->body;
            if (d->fname) dict_put(h, &c->funcs, d->fname)->p = f;
            dict_put(h, &c->funcs, d->name)->p = f;
        }
    }
    if (!mainf) sh_compile_error(h, "no main()");
    scope_push(c, new_dict(c));
    scan_body(c, mainf->body);
    for (i = 0; i < c->funcs.n; i++) if (c->funcs.ent[i].live) scan_body(c, ((Func *)c->funcs.ent[i].p)->body);
    shadow_varyings(c, mainf->body);
    for (i = 0; i < nd; i++) {          /* a function called under an if writes a global under the if: never a tracked constant */
        Decl *d = &ast[i];
        if (d->kind == D_GDECL && !strcmp(d->qual, "var")) {
            Node *g = (Node *)sh_alloc(h, sizeof(Node));
            memset(g, 0, sizeof *g);
            g->kind = S_LDECL; g->ty = d->ty; g->s = d->name; g->a = d->init; g->b = d->arr;
            mark_decls(g);
            run_stmt(c, g);
        }
    }
    run_main(c, mainf->body);
    flush_varyings(c);
    emit(c, "END");
    sv_room(c, c->vcount + 1);          /* the table covers every register, scalar or not */
    out->const_base = h->umap ? h->urows : c->next_uniform;
    out->sasm = sh_optimize(h, &c->code, c->vcount, c->sv, remat, out->const_base, c->fs, c->pool, &c->npool);
    if (c->fs) out->attribs = sym_table(c, &c->attribs, &out->nattribs);
    else place_attribs(c, out);
    out->uniforms = sym_table(c, &c->uniforms, &out->nuniforms);
    out->samplers = sym_table(c, &c->samplers, &out->nsamplers);
    out->varyings = sym_table(c, &c->varyings, &out->nvaryings);
    out->pool = c->pool; out->npool = c->npool;
    out->vs_tex = c->vs_tex;
}
