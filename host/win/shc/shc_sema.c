/* shc_sema.c — glslc's fold_expr() and class Sema, function for function: GLSL ES 1.00's types, overload resolution, the
 * errors the language requires. h->strict (a program from an application) reports every error; otherwise (the programs the
 * libraries generate, and the benches) it only resolves overloads, and takes what it cannot type as ERR. */
#include <math.h>
#include <string.h>
#include "shc_internal.h"

#define PI_180   0x1.1df46a2529d39p-6     /* math.pi / 180.0 */
#define D180_PI  0x1.ca5dc1a63c1f8p+5     /* 180.0 / math.pi */

const struct ShMaxC SH_GL_MAX[SH_NMAX] = {
    { "gl_MaxVertexAttribs", 16 }, { "gl_MaxVertexUniformVectors", 224 }, { "gl_MaxVaryingVectors", 8 }, { "gl_MaxVertexTextureImageUnits", 0 },
    { "gl_MaxCombinedTextureImageUnits", 8 }, { "gl_MaxTextureImageUnits", 8 }, { "gl_MaxFragmentUniformVectors", 48 }, { "gl_MaxDrawBuffers", 1 }
};
Member SH_DEPTHRANGE[3] = { { "float", "near", NULL, NULL }, { "float", "far", NULL, NULL }, { "float", "diff", NULL, NULL } };

/* ---------------- constant expressions ---------------- */
double sh_fold_cast(Shc *h, const char *name, double a)
{
    if (!strcmp(name, "int")) {                /* float(int(a)) */
        double t;
        if (a != a) sh_py_error(h, "ValueError: cannot convert float NaN to integer");
        if (a - a != 0.0) sh_py_error(h, "OverflowError: cannot convert float infinity to integer");
        if (a >= 4503599627370496.0 || a <= -4503599627370496.0) return a;       /* already an integer */
        t = (double)(long long)a;
        return t == 0.0 ? 0.0 : t;              /* int(-0.5) is 0, not -0.0 */
    }
    if (!strcmp(name, "bool")) return a != 0.0 ? 1.0 : 0.0;
    return a;
}

static int is_cast(const char *s) { return !strcmp(s, "float") || !strcmp(s, "int") || !strcmp(s, "bool"); }
static double pfloor(double x) { return floor(x); }

/* _FOLD1: 0 = the function raises, so the expression is no constant */
static int fold1(const char *n, double x, double *r)
{
    int finite = x - x == 0.0;
    if (!strcmp(n, "radians")) *r = x * PI_180;
    else if (!strcmp(n, "degrees")) *r = x * D180_PI;
    else if (!strcmp(n, "sin")) { if (!finite) return 0; *r = sin(x); }
    else if (!strcmp(n, "cos")) { if (!finite) return 0; *r = cos(x); }
    else if (!strcmp(n, "tan")) { if (!finite) return 0; *r = tan(x); }
    else if (!strcmp(n, "asin")) { if (fabs(x) > 1.0) return 0; *r = asin(x); }
    else if (!strcmp(n, "acos")) { if (fabs(x) > 1.0) return 0; *r = acos(x); }
    else if (!strcmp(n, "atan")) *r = atan(x);
    else if (!strcmp(n, "exp")) { *r = exp(x); if (finite && *r - *r != 0.0) return 0; }
    else if (!strcmp(n, "log")) { if (x <= 0.0) return 0; *r = log(x); }
    else if (!strcmp(n, "sqrt")) { if (x < 0.0) return 0; *r = sqrt(x); }
    else if (!strcmp(n, "abs")) *r = fabs(x);
    else if (!strcmp(n, "floor")) { if (!finite) return 0; *r = pfloor(x); }
    else if (!strcmp(n, "ceil")) { if (!finite) return 0; *r = ceil(x); }
    else if (!strcmp(n, "exp2")) { *r = pow(2.0, x); if (finite && *r - *r != 0.0) return 0; }
    else if (!strcmp(n, "log2")) { if (x <= 0.0) return 0; *r = log2(x); }
    else if (!strcmp(n, "inversesqrt")) { if (x <= 0.0) return 0; *r = 1.0 / sqrt(x); }
    else if (!strcmp(n, "fract")) { if (!finite) return 0; *r = x - pfloor(x); }
    else if (!strcmp(n, "sign")) *r = (double)((x > 0) - (x < 0));
    else return -1;
    return 1;
}

/* _FOLD2 */
static int fold2(const char *n, double x, double y, double *r)
{
    if (!strcmp(n, "pow")) {
        if ((x == 0.0 && y < 0.0) || (x < 0.0 && y != floor(y))) return 0;
        *r = pow(x, y);
        if (x - x == 0.0 && y - y == 0.0 && *r - *r != 0.0) return 0;
    }
    else if (!strcmp(n, "min")) *r = y < x ? y : x;
    else if (!strcmp(n, "max")) *r = y > x ? y : x;
    else if (!strcmp(n, "atan")) *r = atan2(x, y);
    else if (!strcmp(n, "mod")) { if (y == 0.0) return 0; *r = x - y * floor(x / y); }
    else if (!strcmp(n, "step")) *r = y < x ? 0.0 : 1.0;
    else return -1;
    return 1;
}

int sh_fold(Shc *h, Node *e, ShGet get, void *ctx, double *out, int *isint)
{
    int r = 0;
    *isint = 0;
    SH_ENTER(h, 1);
    switch (e->kind) {
    case N_NUM: *out = e->num; *isint = e->lit == LIT_INT; r = 1; break;
    case N_VAR: r = get(ctx, e->s, out, isint); break;
    case N_UNARY: {
        double a;
        int ai;
        if (sh_fold(h, e->a, get, ctx, &a, &ai)) {
            if (e->s[0] == '-') { *out = sh_pf(h, -a); *isint = ai; r = 1; }
            else if (e->s[0] == '+') { *out = a; *isint = ai; r = 1; }
            else if (e->s[0] == '!') { *out = a != 0.0 ? 0.0 : 1.0; r = 1; }
        }
        break;
    }
    case N_BIN: {
        double a = 0, b = 0, v = 0;
        const char *op = e->s;
        int ai = 0, bi = 0, ha = sh_fold(h, e->a, get, ctx, &a, &ai), hb = sh_fold(h, e->b, get, ctx, &b, &bi);
        if (!ha || !hb) break;
        if (ai && bi && (!strcmp(op, "+") || !strcmp(op, "-") || !strcmp(op, "*") || !strcmp(op, "/"))) {
            if (op[0] == '/') v = b != 0.0 ? sh_fold_cast(h, "int", a / b) : 0.0;            /* toward zero */
            else v = op[0] == '+' ? a + b : op[0] == '-' ? a - b : a * b;
            *isint = 1;
        }
        else if (!strcmp(op, "+")) v = a + b;
        else if (!strcmp(op, "-")) v = a - b;
        else if (!strcmp(op, "*")) v = a * b;
        else if (!strcmp(op, "/")) v = b != 0.0 ? a / b : 0.0;
        else if (!strcmp(op, "%")) v = b != 0.0 ? sh_pymod(a, b) : 0.0;
        else if (!strcmp(op, "<")) v = a < b;
        else if (!strcmp(op, ">")) v = a > b;
        else if (!strcmp(op, "<=")) v = a <= b;
        else if (!strcmp(op, ">=")) v = a >= b;
        else if (!strcmp(op, "==")) v = a == b;
        else if (!strcmp(op, "!=")) v = a != b;
        else if (!strcmp(op, "&&")) v = a != 0.0 && b != 0.0;
        else if (!strcmp(op, "^^")) v = (a != 0.0) != (b != 0.0);
        else if (!strcmp(op, "||")) v = a != 0.0 || b != 0.0;
        else break;                              /* {...}.get(op): None */
        *out = sh_pf(h, v);
        r = 1;
        break;
    }
    case N_TERNARY: {
        double cv;
        int ci;
        if (sh_fold(h, e->a, get, ctx, &cv, &ci)) r = sh_fold(h, cv != 0.0 ? e->b : e->c, get, ctx, out, isint);
        break;
    }
    case N_CALL: {
        double a[3];
        int ai, i;
        if (is_cast(e->s) && e->nlist == 1) {
            if (sh_fold(h, e->list[0], get, ctx, &a[0], &ai)) { *out = sh_fold_cast(h, e->s, a[0]); *isint = !strcmp(e->s, "int"); r = 1; }
        }
        else if (e->nlist == 1) {
            double t;
            int k = fold1(e->s, 0.0, &t);               /* -1: not one of them */
            if (k >= 0 && sh_fold(h, e->list[0], get, ctx, &a[0], &ai) && fold1(e->s, a[0], &t) == 1) { *out = sh_pf(h, t); r = 1; }
        }
        else if (e->nlist == 2) {
            double t;
            if (fold2(e->s, 1.0, 1.0, &t) >= 0) {
                int ok = 1;
                for (i = 0; i < 2 && ok; i++) ok = sh_fold(h, e->list[i], get, ctx, &a[i], &ai);
                if (ok && fold2(e->s, a[0], a[1], &t) == 1) { *out = sh_pf(h, t); r = 1; }
            }
        }
        else if (e->nlist == 3 && (!strcmp(e->s, "clamp") || !strcmp(e->s, "mix"))) {
            int ok = 1;
            for (i = 0; i < 3 && ok; i++) ok = sh_fold(h, e->list[i], get, ctx, &a[i], &ai);
            if (ok) {
                double t;
                if (!strcmp(e->s, "clamp")) { t = a[1] > a[0] ? a[1] : a[0]; t = a[2] < t ? a[2] : t; }
                else t = a[0] * (1.0 - a[2]) + a[1] * a[2];
                *out = sh_pf(h, t);
                r = 1;
            }
        }
        break;
    }
    default: break;
    }
    SH_LEAVE(h, 1);
    return r;
}

/* ---------------- the pass ---------------- */
typedef struct SSym { ShT ty; const char *qual; int has; double val; int vint; } SSym;
typedef struct StructInfo { Member *m; int n; } StructInfo;
typedef struct S {
    Shc *h;
    int fs, strict, exts;
    Dict **scopes; int nscope, capscope;
    unsigned char *fprec; int nprec, capprec;    /* defprec: this scope's default precision for float, 0 none, 1 lowp, 2 mediump, 3 highp */
    unsigned char *iprec;                        /* the same for int */
    Dict funcs, structs;
    ShFn *cur;
    int loops, depthrange;
    Dict gnames;                                 /* a global name -> "function", "variable" or "struct": §4.2.7, they share one name space */
    int fragout;                                 /* 1 gl_FragColor, 2 gl_FragData: the ones the program writes */
} S;

static const ShT ERR = { "?", -1 };
static ShT tn(const char *b) { ShT t; t.b = b; t.arr = -1; return t; }
static int is_err(ShT t) { return t.b[0] == '?'; }
static int teq(ShT a, ShT b) { return !strcmp(a.b, b.b) && a.arr == b.arr; }
static int tsame(ShT a, ShT b) { return teq(a, b) || is_err(a) || is_err(b); }

static int is_scalar(const char *t) { return !strcmp(t, "float") || !strcmp(t, "int") || !strcmp(t, "bool"); }
static int vec_n(const char *t)
{
    const char *p = t;
    if (*p == 'b' || *p == 'i') p++;
    if (p[0] == 'v' && p[1] == 'e' && p[2] == 'c' && p[3] >= '2' && p[3] <= '4' && !p[4]) return p[3] - '0';
    return 0;
}
static int mat_n(const char *t) { return t[0] == 'm' && t[1] == 'a' && t[2] == 't' && t[3] >= '2' && t[3] <= '4' && !t[4] ? t[3] - '0' : 0; }
static int is_sampler(const char *t) { return !strcmp(t, "sampler2D") || !strcmp(t, "samplerCube") || !strcmp(t, "sampler3D"); }

static const char *fam(const char *b, int *n)    /* _fam */
{
    *n = 1;
    if (is_scalar(b)) return b;
    if (vec_n(b)) { *n = vec_n(b); return b[0] == 'i' ? "int" : b[0] == 'b' ? "bool" : "float"; }
    if (mat_n(b)) { *n = mat_n(b); return "float"; }
    *n = 0;
    return NULL;
}

static const char *vecof(S *s, const char *f, int n)
{
    if (n == 1) return f;
    return sh_fmt(s->h, "%s%d", !strcmp(f, "float") ? "vec" : !strcmp(f, "int") ? "ivec" : "bvec", n);
}

/* self.err: a program from an application stops here */
static ShT serr(S *s, const char *fmt, ...)
{
    if (s->strict) {
        SB b; va_list ap;
        sb_init(&b, s->h, 80);
        va_start(ap, fmt); sb_vfmt(&b, fmt, ap); va_end(ap);
        sh_compile_error(s->h, "%s", sb_str(&b));
    }
    return ERR;
}

static SSym *sym(S *s, const char *name)
{
    int i;
    for (i = s->nscope - 1; i >= 0; i--) {
        DictEnt *e = dict_find(s->scopes[i], name);
        if (e) return (SSym *)e->p;
    }
    return NULL;
}

static int get_val(void *ctx, const char *name, double *v, int *isint)
{
    SSym *y = sym((S *)ctx, name);
    if (!y || !y->has) return 0;
    *v = y->val; *isint = y->vint;
    return 1;
}

static int fold(S *s, Node *e, double *v, int *isint) { return sh_fold(s->h, e, get_val, s, v, isint); }

static void scope_push(S *s)
{
    if (s->nscope == s->capscope) {
        int cap = s->capscope ? s->capscope * 2 : 16;
        Dict **n = (Dict **)sh_alloc(s->h, (size_t)cap * sizeof(Dict *));
        unsigned char *f = (unsigned char *)sh_alloc(s->h, (size_t)cap), *ip = (unsigned char *)sh_alloc(s->h, (size_t)cap);
        if (s->nscope) { memcpy(n, s->scopes, (size_t)s->nscope * sizeof(Dict *)); memcpy(f, s->fprec, (size_t)s->nscope); memcpy(ip, s->iprec, (size_t)s->nscope); }
        s->scopes = n; s->fprec = f; s->iprec = ip; s->capscope = cap;
    }
    s->scopes[s->nscope] = (Dict *)sh_zalloc(s->h, sizeof(Dict));
    s->fprec[s->nscope] = 0; s->iprec[s->nscope] = 0;
    s->nscope++;
}

static SSym *new_sym(S *s, ShT ty, const char *qual, int has, double val, int vint)
{
    SSym *y = (SSym *)sh_alloc(s->h, sizeof(SSym));
    y->ty = ty; y->qual = qual; y->has = has; y->val = val; y->vint = vint;
    return y;
}

static void put(S *s, Dict *d, const char *name, ShT ty, const char *qual, int has, double val, int vint)
{
    dict_put(s->h, d, name)->p = new_sym(s, ty, qual, has, val, vint);
}

static void declare(S *s, const char *name, ShT ty, const char *qual, int has, double val, int vint)
{
    Dict *top = s->scopes[s->nscope - 1];
    if (!strncmp(name, "gl_", 3)) serr(s, "'%s': the gl_ prefix is reserved", name);
    if (*name && dict_find(top, name)) serr(s, "'%s' is already declared in this scope", name);
    put(s, top, name, ty, qual, has, val, vint);
}

static void need_prec(S *s, const char *base, const char *prec)
{
    int n, i;
    const char *f = fam(base, &n);
    if (dict_find(&s->structs, base) || !f || strcmp(f, "float") || prec) return;
    for (i = s->nscope - 1; i >= 0; i--) if (s->fprec[i]) return;
    serr(s, "no precision for a float type: the fragment language has no default");
}

static int base_ok(S *s, const char *b)
{
    return is_scalar(b) || vec_n(b) || mat_n(b) || dict_find(&s->structs, b) || is_sampler(b);
}

static ShT ty(S *s, Node *e);

/* an array's size: an integral constant expression above zero; -1 = none */
static long long size_of(S *s, Node *arr)
{
    ShT t;
    double v;
    int vi;
    if (!arr) return -1;
    t = ty(s, arr);
    if (!teq(t, tn("int")) && !is_err(t)) serr(s, "an array size must be an integer");
    if (!fold(s, arr, &v, &vi)) { serr(s, "an array size must be a constant expression"); return -2; }
    if (v <= 0) serr(s, "an array size must be above zero");
    if (v != v || v - v != 0.0) sh_py_error(s->h, "OverflowError: cannot convert float to integer");
    if (v > 1e15) sh_py_error(s->h, "array size out of range");
    return (long long)v;
}

static StructInfo *struct_get(S *s, const char *b) { DictEnt *e = dict_find(&s->structs, b); return e ? (StructInfo *)e->p : NULL; }

static int has_sampler(S *s, const char *b)
{
    StructInfo *i;
    int k;
    if (is_sampler(b)) return 1;
    if ((i = struct_get(s, b)) != NULL) for (k = 0; k < i->n; k++) if (has_sampler(s, i->m[k].ty)) return 1;
    return 0;
}

static int has_opaque(S *s, const char *b)
{
    StructInfo *i;
    int k;
    if (is_sampler(b)) return 1;
    if ((i = struct_get(s, b)) != NULL) for (k = 0; k < i->n; k++) if (i->m[k].arr || has_opaque(s, i->m[k].ty)) return 1;
    return 0;
}

/* a struct type with an array among its members, at any depth */
static int has_array(S *s, const char *b)
{
    StructInfo *i;
    int k;
    if ((i = struct_get(s, b)) != NULL) for (k = 0; k < i->n; k++) if (i->m[k].arr || has_array(s, i->m[k].ty)) return 1;
    return 0;
}

static void gname(S *s, const char *name, const char *kind)
{
    DictEnt *e = dict_find(&s->gnames, name);
    if (!e) { dict_put(s->h, &s->gnames, name)->p = (void *)kind; return; }
    if (strcmp((const char *)e->p, kind)) serr(s, "'%s' is already declared as a %s", name, (const char *)e->p);
}

/* a write to e: §7.2, a shader writes gl_FragColor or gl_FragData, not both */
static void wrote(S *s, Node *e)
{
    SH_ENTER(s->h, 1);
    while (e->kind == N_SWIZZLE || e->kind == N_INDEX) e = e->a;
    if (e->kind == N_VAR && !strcmp(e->s, "gl_FragData")) s->fragout |= 2;
    else if (e->kind == N_VAR && !strcmp(e->s, "gl_FragColor")) s->fragout |= e->fragdata ? 2 : 1;
    SH_LEAVE(s->h, 1);
}

/* the precision a declaration of base type b has here: its own, else the default in effect; NULL for bool and struct types.
 * A static string: it outlives the compile (g2sc_member.prec) */
static const char *prec_of(S *s, const char *b, const char *own)
{
    static const char *const NAME[4] = { NULL, "lowp", "mediump", "highp" };
    int n, i;
    const char *f = fam(b, &n);
    unsigned char *tab;
    if (own) own = !strcmp(own, "lowp") ? NAME[1] : !strcmp(own, "mediump") ? NAME[2] : NAME[3];
    if (is_sampler(b)) return own ? own : NAME[1];
    if (!f || !strcmp(f, "bool")) return NULL;
    if (own) return own;
    tab = !strcmp(f, "float") ? s->fprec : s->iprec;
    for (i = s->nscope - 1; i >= 0; i--) if (tab[i]) return NAME[tab[i]];
    return NULL;
}

static void do_struct(S *s, const char *name, Member *m, int n)
{
    StructInfo *i = (StructInfo *)sh_alloc(s->h, sizeof(StructInfo));
    int k;
    if (dict_find(&s->structs, name)) serr(s, "struct %s is already defined", name);
    for (k = 0; k < n; k++) {
        if (!base_ok(s, m[k].ty) || !strcmp(m[k].ty, "void")) serr(s, "struct %s: bad member type %s", name, m[k].ty);
        need_prec(s, m[k].ty, m[k].prec);
        if (m[k].arr) size_of(s, m[k].arr);
        m[k].rprec = prec_of(s, m[k].ty, m[k].prec);
    }
    i->m = m; i->n = n;
    dict_put(s->h, &s->structs, name)->p = i;
}

static void prec_stmt(S *s, const char *q, const char *t)
{
    if (strcmp(q, "highp") && strcmp(q, "mediump") && strcmp(q, "lowp")) serr(s, "bad precision statement");
    if (strcmp(t, "float") && strcmp(t, "int") && !is_sampler(t)) serr(s, "no default precision for %s", t);
    {
        unsigned char lvl = !strcmp(q, "lowp") ? 1 : !strcmp(q, "mediump") ? 2 : 3;
        if (!strcmp(t, "float")) s->fprec[s->nscope - 1] = lvl;
        if (!strcmp(t, "int")) s->iprec[s->nscope - 1] = lvl;
    }
}

static int is_const(S *s, Node *e);

static void init_check(S *s, const char *b, long long n, Node *init, int must_const)
{
    ShT t = ty(s, init), want;
    want.b = b; want.arr = n;
    if (!tsame(t, want)) serr(s, "cannot initialize a %s with a %s", b, t.b);
    if (must_const && !is_const(s, init)) serr(s, "the initializer of a const must be a constant expression");
}

static void gdecl(S *s, Decl *d)
{
    const char *qual = d->qual, *b = d->ty, *f;
    long long n;
    int w, has = 0, vint = 0;
    double val = 0;
    ShT t;
    if (!base_ok(s, b) || !strcmp(b, "void")) serr(s, "bad type %s", b);
    need_prec(s, b, d->prec);
    n = size_of(s, d->arr);
    f = fam(b, &w);
    if (!strcmp(qual, "attribute")) {
        if (s->fs) serr(s, "attribute is for vertex shaders");
        if (!f || strcmp(f, "float") || d->arr) serr(s, "an attribute is a float, a vector or a matrix of floats");
    }
    else if (!strcmp(qual, "varying")) {
        if (!f || strcmp(f, "float")) serr(s, "a varying is a float, a vector or a matrix of floats, or an array of them");
    }
    else if (!strcmp(qual, "var") || !strcmp(qual, "const")) {
        if (is_sampler(b)) serr(s, "a sampler must be a uniform");
    }
    if (!strcmp(qual, "const")) {
        if (!d->init) serr(s, "const '%s' needs an initializer", d->name);
    }
    else if ((!strcmp(qual, "attribute") || !strcmp(qual, "varying") || !strcmp(qual, "uniform")) && d->init)
        serr(s, "%s '%s' cannot have an initializer", qual, d->name);
    if (d->init) {
        init_check(s, b, n, d->init, !strcmp(qual, "const"));
        if (d->arr) serr(s, "an array cannot have an initializer");
        if (!strcmp(qual, "const") && is_scalar(b)) {
            has = fold(s, d->init, &val, &vint);
            if (has && !strcmp(b, "int")) { val = sh_fold_cast(s->h, "int", val); vint = 1; }
        }
        if (!strcmp(qual, "var") && !is_const(s, d->init)) serr(s, "a global's initializer must be a constant expression");
    }
    t.b = b; t.arr = n;
    declare(s, d->name, t, !strcmp(qual, "var") ? "" : qual, has, val, vint);
}

static void stmt(S *s, Node *n);
static ShT ty_builtin(S *s, const char *n, ShT *ts, int na);

/* a built-in function of this name takes exactly these parameters */
static int builtin_takes(S *s, const char *name, ShT *sig, int n)
{
    int strict = s->strict;
    ShT t;
    s->strict = 0;
    t = ty_builtin(s, name, sig, n);
    s->strict = strict;
    return !is_err(t);
}

static ShFn *func(S *s, Decl *d)
{
    int isdef = d->kind == D_FUNC, i;
    const char *ret = d->ty, *name = d->name;
    ShT *sig = (ShT *)sh_alloc(s->h, ((size_t)d->nparams + 1) * sizeof(ShT));
    DictEnt *e;
    ShFn *fn = NULL, *f, *last = NULL;
    if (strcmp(ret, "void") && !base_ok(s, ret)) serr(s, "bad return type %s", ret);
    if (strcmp(ret, "void")) need_prec(s, ret, d->prec);
    for (i = 0; i < d->nparams; i++) {
        Param *p = &d->params[i];
        if (!strcmp(p->ty, "void") || !base_ok(s, p->ty)) serr(s, "bad parameter type %s", p->ty);
        if (p->isconst && (!strcmp(p->q, "out") || !strcmp(p->q, "inout"))) serr(s, "const out is not a parameter");
        if (is_sampler(p->ty) && strcmp(p->q, "in")) serr(s, "a sampler is an in parameter");
        need_prec(s, p->ty, p->prec);
        sig[i].b = p->ty; sig[i].arr = size_of(s, p->arr);
    }
    if (!strncmp(name, "gl_", 3)) serr(s, "'%s': the gl_ prefix is reserved", name);
    if (strcmp(ret, "void") && has_array(s, ret)) serr(s, "%s: a function cannot return a struct with an array", name);
    if (s->strict && builtin_takes(s, name, sig, d->nparams)) serr(s, "%s: a built-in function cannot be declared again", name);
    e = dict_put(s->h, &s->funcs, name);
    for (f = (ShFn *)e->p; f; f = f->next) {
        int same = f->nparams == d->nparams;
        last = f;
        for (i = 0; same && i < d->nparams; i++) same = teq(f->sig[i], sig[i]);
        if (same) { fn = f; break; }
    }
    if (!fn) {
        fn = (ShFn *)sh_zalloc(s->h, sizeof(ShFn));
        fn->name = name; fn->ret = ret; fn->params = d->params; fn->nparams = d->nparams; fn->sig = sig;
        if (last) last->next = fn;
        else e->p = fn;
    }
    else {
        if (strcmp(fn->ret, ret)) serr(s, "%s: two return types", name);
        if (!isdef) serr(s, "%s is already declared", name);
        for (i = 0; i < d->nparams && i < fn->nparams; i++)
            if (strcmp(d->params[i].q, fn->params[i].q) || d->params[i].isconst != fn->params[i].isconst)
                serr(s, "%s: a parameter's qualifiers differ from its declaration", name);
    }
    if (isdef) {
        if (fn->defined) serr(s, "%s is already defined", name);
        fn->defined = 1; fn->node = d; fn->params = d->params;
        scope_push(s); s->cur = fn;
        for (i = 0; i < d->nparams; i++) {
            Param *p = &d->params[i];
            if (*p->name) declare(s, p->name, sig[i], p->isconst ? "ro" : p->q, 0, 0, 0);
        }
        stmt(s, d->body);
        s->cur = NULL; s->nscope--;
    }
    return fn;
}

static void visit(S *s, ShFn *f)
{
    int i;
    if (f->state == 1) sh_compile_error(s->h, "recursive call to '%s' (forbidden in GLSL ES)", f->name);
    if (f->state) return;
    f->state = 1;
    SH_ENTER(s->h, 1);
    for (i = 0; i < f->ncalls; i++) visit(s, f->calls[i]);
    SH_LEAVE(s->h, 1);
    f->state = 2;
}

/* ---- statements ---- */
static void cond(S *s, Node *e)
{
    ShT t = ty(s, e);
    if (!tsame(t, tn("bool"))) serr(s, "a condition must be a bool, not a %s", t.b);
}

static void stmt_(S *s, Node *n)
{
    int i;
    switch (n->kind) {
    case S_BLOCK:
        scope_push(s);
        for (i = 0; i < n->nlist; i++) stmt(s, n->list[i]);
        s->nscope--;
        break;
    case S_MULTI:
        for (i = 0; i < n->nlist; i++) stmt(s, n->list[i]);
        break;
    case S_LDECL: {
        const char *b = n->ty;
        long long sz;
        int has = 0, vint = 0;
        double val = 0;
        ShT t;
        if (!base_ok(s, b) || !strcmp(b, "void")) serr(s, "bad type %s", b);
        if (has_sampler(s, b)) serr(s, "a sampler must be a uniform");
        need_prec(s, b, n->prec);
        sz = size_of(s, n->b);
        if (n->isc && !n->a) serr(s, "const '%s' needs an initializer", n->s);
        if (n->a) {
            if (n->b) serr(s, "an array cannot have an initializer");
            init_check(s, b, sz, n->a, n->isc);
            if (n->isc && is_scalar(b)) {
                has = fold(s, n->a, &val, &vint);
                if (has && !strcmp(b, "int")) { val = sh_fold_cast(s->h, "int", val); vint = 1; }
            }
        }
        t.b = b; t.arr = sz;
        declare(s, n->s, t, n->isc ? "const" : "", has, val, vint);
        break;
    }
    case S_PREC: prec_stmt(s, n->prec, n->ty); break;
    case S_LSTRUCT: do_struct(s, n->s, n->mem, n->nmem); break;
    case S_EXPR: ty(s, n->a); break;
    case S_IF:
        cond(s, n->a); stmt(s, n->b);
        if (n->c) stmt(s, n->c);
        break;
    case S_FOR:
        scope_push(s);
        stmt(s, n->a); cond(s, n->b);
        if (n->c) ty(s, n->c);
        s->loops++;
        if (s->strict && n->d && n->d->kind == S_BLOCK) {           /* §6.3: the body shares the scope of the for statement */
            for (i = 0; i < n->d->nlist; i++) stmt(s, n->d->list[i]);
        }
        else stmt(s, n->d);
        s->loops--;
        s->nscope--;
        break;
    case S_WHILE:
        cond(s, n->a); s->loops++; stmt(s, n->b); s->loops--;
        break;
    case S_DOWHILE:
        s->loops++; stmt(s, n->b); s->loops--; cond(s, n->a);
        break;
    case S_RETURN: {
        const char *rt = s->cur ? s->cur->ret : "void";
        if (!n->a) { if (strcmp(rt, "void")) serr(s, "return with no value in a function that has a type"); }
        else {
            ShT t = ty(s, n->a);
            if (!strcmp(rt, "void") || !tsame(t, tn(rt))) serr(s, "return of a %s from a function of type %s", t.b, rt);
        }
        break;
    }
    case S_DISCARD: if (!s->fs) serr(s, "discard is for fragment shaders"); break;
    case S_BREAK: if (!s->loops) serr(s, "break outside a loop"); break;
    case S_CONTINUE: if (!s->loops) serr(s, "continue outside a loop"); break;
    default: break;
    }
}

static void stmt(S *s, Node *n)
{
    if (!n) return;
    SH_ENTER(s->h, 1);
    stmt_(s, n);
    SH_LEAVE(s->h, 1);
}

/* ---- expressions ---- */
static int is_lvalue(S *s, Node *e)
{
    int r = 0;
    SH_ENTER(s->h, 1);
    if (e->kind == N_VAR) {
        SSym *y = sym(s, e->s);
        if (!y) r = 1;
        else if (!strcmp(y->qual, "const") || !strcmp(y->qual, "attribute") || !strcmp(y->qual, "uniform") || !strcmp(y->qual, "ro")) r = 0;
        else if (!strcmp(y->qual, "varying") && s->fs) r = 0;
        else r = 1;
    }
    else if (e->kind == N_SWIZZLE) {
        size_t i, j, n = strlen(e->s);
        int dup = 0;
        for (i = 0; i < n; i++) for (j = i + 1; j < n; j++) if (e->s[i] == e->s[j]) dup = 1;
        if (dup && !dict_find(&s->structs, ty(s, e->a).b)) r = 0;
        else r = is_lvalue(s, e->a);
    }
    else if (e->kind == N_INDEX) r = is_lvalue(s, e->a);
    SH_LEAVE(s->h, 1);
    return r;
}

static int is_texfn(const char *n)
{
    static const char *const T[] = { "texture2D","texture2DProj","textureCube","texture2DLod","texture2DProjLod","textureCubeLod","texture3D","texture3DProj","texture3DLod","texture3DProjLod",NULL };
    int i;
    for (i = 0; T[i]; i++) if (!strcmp(T[i], n)) return 1;
    return 0;
}
static int is_deriv(const char *n) { return !strcmp(n, "dFdx") || !strcmp(n, "dFdy") || !strcmp(n, "fwidth"); }

static int is_const(S *s, Node *e)
{
    int r = 0, i;
    SH_ENTER(s->h, 1);
    switch (e->kind) {
    case N_NUM: r = 1; break;
    case N_VAR: { SSym *y = sym(s, e->s); r = !y || !strcmp(y->qual, "const"); break; }
    case N_SWIZZLE: case N_UNARY: r = is_const(s, e->a); break;
    case N_INDEX: case N_BIN: r = is_const(s, e->a) && is_const(s, e->b); break;
    case N_TERNARY: r = is_const(s, e->a) && is_const(s, e->b) && is_const(s, e->c); break;
    case N_CALL:
        if (dict_find(&s->funcs, e->s) || is_texfn(e->s) || is_deriv(e->s) || strstr(e->s, "EXT")) break;
        r = 1;
        for (i = 0; i < e->nlist && r; i++) r = is_const(s, e->list[i]);
        break;
    default: break;
    }
    SH_LEAVE(s->h, 1);
    return r;
}

static ShT ty_field(S *s, Node *e)
{
    ShT t = ty(s, e->a);
    const char *f = e->s, *fm;
    StructInfo *si;
    static const char *const NAMES[3] = { "xyzw", "rgba", "stpq" };
    int n, k;
    size_t len = strlen(f), i;
    if (is_err(t)) return t;
    if (t.arr >= 0) return serr(s, "a field of an array");
    if ((si = struct_get(s, t.b)) != NULL) {
        for (k = 0; k < si->n; k++) {
            if (!strcmp(si->m[k].name, f)) {
                ShT r;
                r.b = si->m[k].ty; r.arr = si->m[k].arr ? size_of(s, si->m[k].arr) : -1;
                return r;
            }
        }
        return serr(s, "struct has no member '%s'", f);
    }
    fm = fam(t.b, &n);
    if (!vec_n(t.b)) return serr(s, "a field of a %s", t.b);
    for (k = 0; k < 3; k++) {
        int all = 1;
        for (i = 0; i < len; i++) if (!strchr(NAMES[k], f[i])) all = 0;
        if (all) {
            int bad = len > 4;
            for (i = 0; i < len; i++) if ((int)(strchr(NAMES[k], f[i]) - NAMES[k]) >= n) bad = 1;
            if (bad) return serr(s, "swizzle .%s out of range", f);
            return tn(vecof(s, fm, (int)len));
        }
    }
    return serr(s, "bad swizzle .%s", f);
}

static ShT ty_index(S *s, Node *e)
{
    ShT t = ty(s, e->a), i = ty(s, e->b);
    double v = 0;
    int vi, hasv, n;
    const char *f;
    if (!tsame(i, tn("int"))) serr(s, "an index must be an int");
    hasv = fold(s, e->b, &v, &vi);
    if (is_err(t)) return t;
    if (t.arr >= 0) {
        if (hasv && !(v >= 0 && v < (double)t.arr)) serr(s, "index %L out of range", (long long)v);
        return tn(t.b);
    }
    if (mat_n(t.b) || vec_n(t.b)) {
        f = fam(t.b, &n);
        if (hasv && !(v >= 0 && v < n)) serr(s, "index %L out of range", (long long)v);
        return tn(mat_n(t.b) ? sh_fmt(s->h, "vec%d", n) : f);
    }
    return serr(s, "cannot index a %s", t.b);
}

static ShT ty_bin(S *s, const char *op, ShT a, ShT b)
{
    int rel = !strcmp(op, "<") || !strcmp(op, ">") || !strcmp(op, "<=") || !strcmp(op, ">=");
    int eq = !strcmp(op, "==") || !strcmp(op, "!=");
    int logic = !strcmp(op, "&&") || !strcmp(op, "||") || !strcmp(op, "^^");
    int na, nb;
    const char *fa, *fb;
    if (is_err(a) || is_err(b)) return rel || eq || logic ? tn("bool") : (!is_err(a) ? a : b);
    if (logic) {
        if (!teq(a, tn("bool")) || !teq(b, tn("bool"))) serr(s, "%s needs bools", op);
        return tn("bool");
    }
    if (eq) {
        if (!teq(a, b)) serr(s, "%s of a %s and a %s", op, a.b, b.b);
        if (a.arr >= 0 || has_opaque(s, a.b)) serr(s, "%s of this type", op);
        return tn("bool");
    }
    if (rel) {
        if (!teq(a, b) || a.arr >= 0 || (strcmp(a.b, "int") && strcmp(a.b, "float"))) serr(s, "%s of a %s and a %s", op, a.b, b.b);
        return tn("bool");
    }
    if (strcmp(op, "+") && strcmp(op, "-") && strcmp(op, "*") && strcmp(op, "/")) return serr(s, "the operator %s is reserved", op);
    fa = fam(a.b, &na); fb = fam(b.b, &nb);
    if (a.arr >= 0 || b.arr >= 0 || !fa || !fb || !strcmp(fa, "bool") || !strcmp(fb, "bool") || strcmp(fa, fb))
        return serr(s, "%s of a %s and a %s", op, a.b, b.b);
    if (!strcmp(a.b, b.b)) return a;
    if (is_scalar(a.b)) return b;
    if (is_scalar(b.b)) return a;
    if (op[0] == '*') {
        if (mat_n(a.b) && vec_n(b.b) && na == nb) return b;
        if (vec_n(a.b) && mat_n(b.b) && na == nb) return a;
    }
    return serr(s, "%s of a %s and a %s", op, a.b, b.b);
}

static ShT ty_ctor(S *s, const char *n, Node **args, ShT *ts, int na)
{
    long long total = 0, want;
    int mats = 0, i;
    (void)args;
    if (!na) return serr(s, "%s() needs arguments", n);
    want = is_scalar(n) ? 1 : vec_n(n) ? vec_n(n) : mat_n(n) * mat_n(n);
    for (i = 0; i < na; i++) {
        int c;
        const char *f;
        if (is_err(ts[i])) return tn(n);
        f = fam(ts[i].b, &c);
        if (!f || ts[i].arr >= 0) return serr(s, "%s() of a %s", n, ts[i].b);
        if (mat_n(ts[i].b)) { c = c * c; mats++; }
        if (total >= want) return serr(s, "%s(): too many arguments", n);
        total += c;
    }
    if (mats && na > 1 && mat_n(n)) return serr(s, "%s(): a matrix goes alone", n);
    if (total < want && !(na == 1 && (is_scalar(ts[0].b) || (mat_n(n) && mat_n(ts[0].b))))) return serr(s, "%s(): too few arguments", n);
    return tn(n);
}

/* SIGS: return type, name, parameters. G float or vec2..4, V vec2..4, I ivec2..4, L bvec2..4, M a matrix, all of one size; R bvec of that size */
static const char *const SIGS[] = {
    "G radians G","G degrees G","G sin G","G cos G","G tan G","G asin G","G acos G","G atan G","G atan G G","G pow G G","G exp G","G log G","G exp2 G","G log2 G",
    "G sqrt G","G inversesqrt G","G abs G","G sign G","G floor G","G ceil G","G fract G","G mod G G","G mod G float","G min G G","G min G float",
    "G max G G","G max G float","G clamp G G G","G clamp G float float","G mix G G G","G mix G G float","G step G G","G step float G",
    "G smoothstep G G G","G smoothstep float float G","float length G","float distance G G","float dot G G","vec3 cross vec3 vec3",
    "G normalize G","G faceforward G G G","G reflect G G","G refract G G float","M matrixCompMult M M",
    "R lessThan V V","R lessThan I I","R lessThanEqual V V","R lessThanEqual I I","R greaterThan V V","R greaterThan I I",
    "R greaterThanEqual V V","R greaterThanEqual I I","R equal V V","R equal I I","R equal L L","R notEqual V V","R notEqual I I","R notEqual L L",
    "bool any L","bool all L","R not L",
    "vec4 texture2D sampler2D vec2","vec4 texture2D sampler2D vec2 float","vec4 texture2DProj sampler2D vec3","vec4 texture2DProj sampler2D vec3 float",
    "vec4 texture2DProj sampler2D vec4","vec4 texture2DProj sampler2D vec4 float","vec4 textureCube samplerCube vec3","vec4 textureCube samplerCube vec3 float",
    "vec4 texture2DLod sampler2D vec2 float","vec4 texture2DProjLod sampler2D vec3 float","vec4 texture2DProjLod sampler2D vec4 float",
    "vec4 textureCubeLod samplerCube vec3 float",
    "G dFdx G","G dFdy G","G fwidth G",
    "vec4 texture2DLodEXT sampler2D vec2 float","vec4 texture2DProjLodEXT sampler2D vec3 float","vec4 texture2DProjLodEXT sampler2D vec4 float",
    "vec4 textureCubeLodEXT samplerCube vec3 float","vec4 texture2DGradEXT sampler2D vec2 vec2 vec2",
    "vec4 texture2DProjGradEXT sampler2D vec3 vec2 vec2","vec4 texture2DProjGradEXT sampler2D vec4 vec2 vec2",
    "vec4 textureCubeGradEXT samplerCube vec3 vec3 vec3",
    "vec4 texture3D sampler3D vec3","vec4 texture3D sampler3D vec3 float","vec4 texture3DProj sampler3D vec4","vec4 texture3DProj sampler3D vec4 float",
    "vec4 texture3DLod sampler3D vec3 float","vec4 texture3DProjLod sampler3D vec4 float",
    NULL
};

static int sig_split(char *buf, const char *sg, char **w)
{
    int n = 0;
    char *p = buf;
    strcpy(buf, sg);
    while (*p && n < 7) {
        w[n++] = p;
        while (*p && *p != ' ') p++;
        if (*p) *p++ = 0;
    }
    return n;
}

static const char *types_text(S *s, ShT *ts, int n)
{
    SB b;
    int i;
    sb_init(&b, s->h, 40);
    for (i = 0; i < n; i++) { if (i) sb_puts(&b, ", "); sb_puts(&b, ts[i].b); }
    return sb_str(&b);
}

static ShT ty_builtin(S *s, const char *n, ShT *ts, int na)
{
    int fs = s->fs, known = 0, i;
    size_t ln = strlen(n);
    if (s->strict) {
        if (is_deriv(n) && !(fs && (s->exts & SH_EXT_DERIV))) return serr(s, "%s needs GL_OES_standard_derivatives", n);
        if (ln > 3 && !strcmp(n + ln - 3, "EXT") && !(fs && (s->exts & SH_EXT_LOD))) return serr(s, "%s needs GL_EXT_shader_texture_lod", n);
        if ((!strcmp(n, "texture2DLod") || !strcmp(n, "texture2DProjLod") || !strcmp(n, "textureCubeLod") || !strcmp(n, "texture3DLod") || !strcmp(n, "texture3DProjLod")) && fs) return serr(s, "%s is for vertex shaders", n);
        if ((!strcmp(n, "texture2D") || !strcmp(n, "texture2DProj") || !strcmp(n, "textureCube") || !strcmp(n, "texture3D") || !strcmp(n, "texture3DProj")) && na == 3 && !fs) return serr(s, "%s with a bias is for fragment shaders", n);
    }
    for (i = 0; SIGS[i]; i++) {
        char buf[80], *w[7];
        int nw = sig_split(buf, SIGS[i], w), bound = 0, ok = 1, k;
        if (strcmp(w[1], n)) continue;
        known = 1;
        if (nw - 2 != na) continue;
        for (k = 0; k < na && ok; k++) {
            const char *p = w[k + 2], *f, *want;
            int c;
            ShT t = ts[k];
            if (is_err(t)) continue;
            if (t.arr >= 0) { ok = 0; break; }
            f = fam(t.b, &c);
            want = !strcmp(p, "G") || !strcmp(p, "V") ? "float" : !strcmp(p, "I") ? "int" : !strcmp(p, "L") ? "bool" : NULL;
            if (!strcmp(p, "M")) { if (!mat_n(t.b)) { ok = 0; break; } }
            else if (want) {
                if (!f || strcmp(f, want) || mat_n(t.b) || (strcmp(p, "G") && c == 1)) { ok = 0; break; }
            }
            else {
                if (strcmp(t.b, p)) ok = 0;
                continue;
            }
            if (!bound) bound = c;
            else if (bound != c) ok = 0;
        }
        if (!ok) continue;
        if (!strcmp(w[0], "G")) return tn(vecof(s, "float", bound ? bound : 1));
        if (!strcmp(w[0], "R")) return tn(vecof(s, "bool", bound ? bound : 2));
        if (!strcmp(w[0], "M")) return tn(sh_fmt(s->h, "mat%d", bound ? bound : 2));
        return tn(sh_strdup(s->h, w[0]));
    }
    if (known) return serr(s, "no function %s(%s)", n, types_text(s, ts, na));
    return serr(s, "unknown function/builtin '%s'", n);
}

static ShT ty_call(S *s, Node *e)
{
    const char *n = e->s;
    int na = e->nlist, i;
    ShT *ts = (ShT *)sh_alloc(s->h, ((size_t)na + 1) * sizeof(ShT));
    StructInfo *si;
    DictEnt *fe;
    for (i = 0; i < na; i++) ts[i] = ty(s, e->list[i]);
    if (is_scalar(n) || vec_n(n) || mat_n(n)) return ty_ctor(s, n, e->list, ts, na);
    if ((si = struct_get(s, n)) != NULL) {
        if (si->n != na) return serr(s, "%s: %d members, %d arguments", n, si->n, na);
        for (i = 0; i < na; i++) {
            ShT w;
            w.b = si->m[i].ty; w.arr = si->m[i].arr ? size_of(s, si->m[i].arr) : -1;
            if (!tsame(ts[i], w)) serr(s, "%s: member %s is not a %s", n, si->m[i].name, ts[i].b);
        }
        return tn(n);
    }
    if ((fe = dict_find(&s->funcs, n)) != NULL) {
        ShFn *f;
        for (f = (ShFn *)fe->p; f; f = f->next) {
            int same = f->nparams == na, loose = same;
            for (i = 0; same && i < na; i++) same = teq(f->sig[i], ts[i]);
            for (i = 0; loose && i < na; i++) loose = tsame(f->sig[i], ts[i]);
            if (same || (!s->strict && loose)) {
                e->fn = f;
                if (s->cur) {
                    ShFn *c = s->cur;
                    if (c->ncalls == c->capcalls) {
                        int cap = c->capcalls ? c->capcalls * 2 : 4;
                        ShFn **nn = (ShFn **)sh_alloc(s->h, (size_t)cap * sizeof(ShFn *));
                        if (c->ncalls) memcpy(nn, c->calls, (size_t)c->ncalls * sizeof(ShFn *));
                        c->calls = nn; c->capcalls = cap;
                    }
                    c->calls[c->ncalls++] = f;
                }
                for (i = 0; i < na; i++) {
                    const char *q = f->params[i].q;
                    if (!strcmp(q, "out") || !strcmp(q, "inout")) {
                        wrote(s, e->list[i]);
                        if (!is_lvalue(s, e->list[i])) serr(s, "%s: an %s argument must be a variable", n, q);
                    }
                }
                return tn(f->ret);
            }
        }
        return serr(s, "no function %s(%s)", n, types_text(s, ts, na));
    }
    return ty_builtin(s, n, ts, na);
}

static ShT ty1(S *s, Node *e)
{
    switch (e->kind) {
    case N_NUM:
        if (e->lit == LIT_ONE) return ERR;
        return tn(e->lit == LIT_INT ? "int" : e->lit == LIT_BOOL ? "bool" : "float");
    case N_VAR: {
        SSym *y = sym(s, e->s);
        if (!y) return serr(s, "undefined identifier '%s'", e->s);
        if (!strcmp(e->s, "gl_DepthRange")) s->depthrange = 1;
        return y->ty;
    }
    case N_SWIZZLE: return ty_field(s, e);
    case N_INDEX: return ty_index(s, e);
    case N_CALL: return ty_call(s, e);
    case N_POSTOP: {
        ShT t = ty(s, e->a);
        int n;
        const char *f;
        wrote(s, e->a);
        if (is_err(t)) return t;
        if (!is_lvalue(s, e->a)) serr(s, "%s needs a variable it can write", e->s);
        f = fam(t.b, &n);
        if (!f || (strcmp(f, "int") && strcmp(f, "float")) || t.arr >= 0) serr(s, "%s of a %s", e->s, t.b);
        return t;
    }
    case N_UNARY: {
        ShT t = ty(s, e->a);
        int n;
        const char *f;
        if (is_err(t)) return t;
        f = fam(t.b, &n);
        if (e->s[0] == '!') { if (!teq(t, tn("bool"))) serr(s, "! needs a bool"); }
        else if (!f || (strcmp(f, "int") && strcmp(f, "float")) || t.arr >= 0) serr(s, "%s of a %s", e->s, t.b);
        return t;
    }
    case N_BIN: {
        ShT a = ty(s, e->a), b = ty(s, e->b);
        return ty_bin(s, e->s, a, b);
    }
    case N_TERNARY: {
        ShT a, b;
        cond(s, e->a);
        a = ty(s, e->b); b = ty(s, e->c);
        if (!tsame(a, b)) serr(s, "?: needs two sides of one type");
        if (a.arr >= 0) serr(s, "?: of arrays");
        return !is_err(a) ? a : b;
    }
    case N_ASSIGN: {
        ShT t = ty(s, e->a), r = ty(s, e->b);
        wrote(s, e->a);
        if (!is_err(t) && !is_lvalue(s, e->a)) serr(s, "assignment to something that cannot be written");
        if (!strcmp(e->s, "=")) {
            if (!tsame(t, r)) serr(s, "cannot assign a %s to a %s", r.b, t.b);
            if (t.arr >= 0 || has_sampler(s, t.b)) serr(s, "cannot assign this type");       /* 5.8: entire structures are l-values, whole arrays are not */
            return t;
        }
        {
            char op[2];
            ShT q;
            op[0] = e->s[0]; op[1] = 0;
            q = ty_bin(s, sh_strdup(s->h, op), t, r);
            if (!tsame(q, t)) serr(s, "%s changes the type of a %s", e->s, t.b);
        }
        return t;
    }
    case N_SEQ: {
        ShT t = ERR;
        int i;
        for (i = 0; i < e->nlist; i++) t = ty(s, e->list[i]);
        return t;
    }
    default: return ERR;
    }
}

static ShT ty(S *s, Node *e)
{
    ShT t;
    SH_ENTER(s->h, 1);
    t = ty1(s, e);
    SH_LEAVE(s->h, 1);
    if (!strcmp(t.b, "int") && t.arr < 0) e->isint = 1;       /* the code generator's registers do not always say so: 1 ?: 2 */
    return t;
}

void sh_sema(Shc *h, Decl *ast, int nd, int fs, int exts, int *depthrange)
{
    S ss, *s = &ss;
    Dict *g;
    int i;
    DictEnt *me;
    memset(s, 0, sizeof *s);
    s->h = h; s->fs = fs; s->strict = h->strict; s->exts = exts;
    scope_push(s);
    g = s->scopes[0];
    if (!fs) { put(s, g, "gl_Position", tn("vec4"), "", 0, 0, 0); put(s, g, "gl_PointSize", tn("float"), "", 0, 0, 0); }
    else {
        ShT fd;
        put(s, g, "gl_FragCoord", tn("vec4"), "ro", 0, 0, 0);
        put(s, g, "gl_FrontFacing", tn("bool"), "ro", 0, 0, 0);
        put(s, g, "gl_PointCoord", tn("vec2"), "ro", 0, 0, 0);
        put(s, g, "gl_FragColor", tn("vec4"), "", 0, 0, 0);
        fd.b = "vec4"; fd.arr = 1;
        put(s, g, "gl_FragData", fd, "", 0, 0, 0);
        if (!s->strict || (exts & SH_EXT_FRAGDEPTH)) put(s, g, "gl_FragDepthEXT", tn("float"), "", 0, 0, 0);
        if (!s->strict) put(s, g, "gl_FragDepth", tn("float"), "", 0, 0, 0);
    }
    for (i = 0; i < SH_NMAX; i++) put(s, g, SH_GL_MAX[i].name, tn("int"), "const", 1, SH_GL_MAX[i].v, 1);
    put(s, g, "gl_DepthRange", tn("gl_DepthRangeParameters"), "uniform", 0, 0, 0);
    {
        StructInfo *si = (StructInfo *)sh_alloc(h, sizeof(StructInfo));
        si->m = SH_DEPTHRANGE; si->n = 3;
        dict_put(h, &s->structs, "gl_DepthRangeParameters")->p = si;
    }
    s->fprec[0] = fs ? 0 : 3; s->iprec[0] = fs ? 2 : 3;          /* §4.5.3: the vertex language highp, the fragment language int mediump */
    for (i = 0; i < nd; i++) {
        Decl *d = &ast[i];
        switch (d->kind) {
        case D_PREC: prec_stmt(s, d->qual, d->ty); break;
        case D_STRUCT: do_struct(s, d->name, d->members, d->nmembers); gname(s, d->name, "struct"); break;
        case D_GDECL: gdecl(s, d); gname(s, d->name, "variable"); break;
        default: func(s, d); gname(s, d->name, "function"); break;
        }
    }
    if (s->fragout == 3) serr(s, "a shader writes gl_FragColor or gl_FragData, not both");
    me = dict_find(&s->funcs, "main");
    {
        ShFn *f;
        int any = 0;
        for (f = me ? (ShFn *)me->p : NULL; f; f = f->next) any |= f->defined;
        if (!any) { if (s->strict) sh_compile_error(h, "no main()"); }
        else for (f = (ShFn *)me->p; f; f = f->next) if (strcmp(f->ret, "void") || f->nparams) serr(s, "main must be void main()");
    }
    for (i = 0; i < s->funcs.n; i++) {                /* an overload goes by its own name in the code generator */
        ShFn *first = (ShFn *)s->funcs.ent[i].p, *f;
        for (f = first; f; f = f->next) {
            if (first->next) {
                SB b;
                int k;
                sb_init(&b, h, 40);
                sb_puts(&b, s->funcs.ent[i].key); sb_putc(&b, '(');
                for (k = 0; k < f->nparams; k++) {
                    if (k) sb_putc(&b, ',');
                    sb_puts(&b, f->sig[k].b);
                    if (f->sig[k].arr >= 0) { sb_putc(&b, '['); sb_putll(&b, f->sig[k].arr); sb_putc(&b, ']'); }
                }
                sb_putc(&b, ')');
                f->name = sb_str(&b);
            }
            else f->name = s->funcs.ent[i].key;
            if (f->node) f->node->fname = f->name;
        }
    }
    if (s->strict) {
        for (i = 0; i < s->funcs.n; i++) {
            ShFn *f;
            for (f = (ShFn *)s->funcs.ent[i].p; f; f = f->next) visit(s, f);
        }
    }
    *depthrange = s->depthrange;
}
