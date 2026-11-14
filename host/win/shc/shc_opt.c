/* shc_opt.c — glslc's passes on the emitted lines: strip_dead_bra, dce, copy_prop, cse, hoist_consts, hoist_tex, _mad_fuse,
 * allocate_registers (with the scalar packing) and _rewrite_packed. Lines are pre-tokenized (Python re-parses the text on
 * every look); each pass makes the same decisions in the same order, so the output text is the same. */
#include <stdlib.h>
#include <string.h>
#include "shc_internal.h"

enum { F_ALU = 1, F_SIDE = 2, F_CTRL = 4 };

enum {
    OP_ADD, OP_SUB, OP_MUL, OP_MAD, OP_MIN, OP_MAX, OP_DP3, OP_DP4, OP_MOV, OP_RCP, OP_RSQRT, OP_LOG2, OP_EXP2,
    OP_FTOI, OP_ITOF, OP_IADD, OP_ISUB, OP_ISLT, OP_ISGE, OP_DDX, OP_DDY, OP_LDP, OP_LDV, OP_LDPX, OP_TEX,
    OP_SEL, OP_IDIV, OP_FLR, OP_CEIL, OP_TRUNC, OP_RNDE, OP_SIN, OP_COS,
    OP_ISEQ, OP_ISNE, OP_IMUL, OP_IMOD, OP_UDIV, OP_UMOD, OP_FTOU, OP_UTOF, OP_MOVI,
    OP_STO, OP_STV, OP_STOZ, OP_DISCARD, OP_SETP, OP_IF, OP_ELSE, OP_ENDIF, OP_BRA, OP_BRP, OP_LOOP, OP_ENDL,
    OP_END, OP_BREAK, OP_CONT, OP_NOP, OP_SET, OP_SQRT, OP_RCPM, OP_RSQM, OP_CLAMP, OP_COUNT
};

static const struct { const char *name; int flags; } OPT[OP_COUNT] = {
    {"ADD",F_ALU},{"SUB",F_ALU},{"MUL",F_ALU},{"MAD",F_ALU},{"MIN",F_ALU},{"MAX",F_ALU},{"DP3",F_ALU},{"DP4",F_ALU},
    {"MOV",F_ALU},{"RCP",F_ALU},{"RSQRT",F_ALU},{"LOG2",F_ALU},{"EXP2",F_ALU},
    {"FTOI",F_ALU},{"ITOF",F_ALU},{"IADD",F_ALU},{"ISUB",F_ALU},{"ISLT",F_ALU},{"ISGE",F_ALU},{"DDX",F_ALU},{"DDY",F_ALU},
    {"LDP",F_ALU},{"LDV",F_ALU},{"LDPX",F_ALU},{"TEX",F_ALU | F_CTRL},
    {"SEL",F_ALU},{"IDIV",F_ALU},{"FLR",F_ALU},{"CEIL",F_ALU},{"TRUNC",F_ALU},{"RNDE",F_ALU},{"SIN",F_ALU},{"COS",F_ALU},
    {"ISEQ",F_ALU},{"ISNE",F_ALU},{"IMUL",F_ALU},{"IMOD",F_ALU},{"UDIV",F_ALU},{"UMOD",F_ALU},{"FTOU",F_ALU},{"UTOF",F_ALU},{"MOVI",F_ALU},
    {"STO",F_SIDE | F_CTRL},{"STV",F_SIDE | F_CTRL},{"STOZ",F_SIDE | F_CTRL},{"DISCARD",F_SIDE | F_CTRL},{"SETP",F_SIDE | F_CTRL},
    {"IF",F_SIDE | F_CTRL},{"ELSE",F_SIDE | F_CTRL},{"ENDIF",F_SIDE | F_CTRL},{"BRA",F_SIDE | F_CTRL},{"BRP",F_SIDE | F_CTRL},
    {"LOOP",F_SIDE | F_CTRL},{"ENDL",F_SIDE | F_CTRL},{"END",F_SIDE | F_CTRL},
    {"BREAK",F_SIDE},{"CONT",F_SIDE},{"NOP",F_SIDE},{"SET",0},
    {"SQRT",F_ALU},{"RCPM",F_ALU},{"RSQM",F_ALU},{"CLAMP",F_ALU}    /* board B's fragment core, xops (glslc._ALU_DST) */
};

static int op_of(const char *base, size_t n)
{
    int i;
    for (i = 0; i < OP_COUNT; i++) if (strlen(OPT[i].name) == n && !memcmp(OPT[i].name, base, n)) return i;
    return -1;
}

static int flags_of(const Line *l) { return l->op >= 0 ? OPT[l->op].flags : 0; }

/* ---------------- tokens ---------------- */
/* a modifier as a character: 0, '-' negate, '!' abs, '~' -abs (written "-!", glslc._MODTXT); and as isa64's field m (0..3) */
static const char MODCH[4] = { 0, '-', '!', '~' };
static int mod_bits(char m) { return m == '-' ? 1 : m == '!' ? 2 : m == '~' ? 3 : 0; }
static int mod_comp(int outer, int inner) { return (outer & 2) ? outer : inner ^ outer; }      /* glslc._modcomp */
static void put_mod(SB *o, char m) { if (m == '~') sb_puts(o, "-!"); else if (m) sb_putc(o, m); }

/* _REGTOK = ^([-!]*)r(\d+)(\.[xyzw]{1,4})?$ ; the modifier as isa64.parse_src reads it ('-' toggles the negation, '!' sets abs) */
static void tok_parse(Tok *t, const char *s)
{
    const char *p = s;
    int n = 0, digits = 0, k = 0, mb = 0;
    char mod;
    memset(t, 0, sizeof *t);
    while (*p == '-' || *p == '!') { mb = *p == '-' ? mb ^ 1 : mb | 2; p++; }
    mod = MODCH[mb];
    if (*p == 'r') {
        p++;
        while (*p >= '0' && *p <= '9') { n = n * 10 + (*p - '0'); p++; digits++; }
        if (digits) {
            if (*p == 0) { t->num = n; t->mod = mod; return; }
            if (*p == '.') {
                const char *q = p + 1;
                while (*q == 'x' || *q == 'y' || *q == 'z' || *q == 'w') { if (k < 4) t->swz[k] = *q; k++; q++; }
                if (*q == 0 && k >= 1 && k <= 4) { t->num = n; t->mod = mod; return; }
            }
        }
    }
    memset(t, 0, sizeof *t);
    t->s = s;
}

static int tok_isreg(const Tok *t) { return t->s == NULL; }
static int tok_num(const Tok *t) { return t->s ? -1 : t->num; }                 /* _regnum, None = -1 */
static int tok_hasmod(const Tok *t) { return !t->s && t->mod; }
static int tok_hasswz(const Tok *t) { return !t->s && t->swz[0]; }

static int tok_eq(const Tok *a, const Tok *b)
{
    if (a->s || b->s) return a->s && b->s && !strcmp(a->s, b->s);
    return a->num == b->num && a->mod == b->mod && !memcmp(a->swz, b->swz, sizeof a->swz);
}

static void line_refresh(Line *l)
{
    int i, d;
    l->dst = -1; l->nsrc = 0;
    if (l->label) return;
    if (l->op == OP_SET) {
        if (l->ntok > 2) l->dst = tok_num(&l->tok[2]);
        for (i = 3; i < 5 && i < l->ntok; i++) if (tok_isreg(&l->tok[i])) l->src[l->nsrc++] = l->tok[i].num;
        return;
    }
    if ((flags_of(l) & F_ALU) && l->ntok > 1) l->dst = tok_num(&l->tok[1]);
    d = l->dst >= 0 ? 1 : -1;
    for (i = 1; i < l->ntok; i++) {
        if (i == d) continue;
        if (tok_isreg(&l->tok[i])) l->src[l->nsrc++] = l->tok[i].num;
    }
}

Line *line_parse(Shc *h, char *text)
{
    Line *l = (Line *)sh_alloc(h, sizeof(Line));
    size_t n = strlen(text);
    char *p = text, *dot;
    memset(l, 0, sizeof *l);
    if (n && text[n - 1] == ':') {
        text[n - 1] = 0;
        l->label = text; l->op = -1; l->mask = ""; l->dst = -1;
        return l;
    }
    while (*p) {
        char *e = p;
        while (*e && *e != ' ') e++;
        if (*e) *e++ = 0;
        if (l->ntok >= LINE_MAXTOK) sh_py_error(h, "internal: too many tokens on a line");
        if (l->ntok == 0) {
            dot = strchr(p, '.');
            l->has_dot = dot != NULL;
            l->mask = dot ? dot + 1 : "";
            l->op = op_of(p, dot ? (size_t)(dot - p) : strlen(p));
            l->tok[0].s = p;
        }
        else tok_parse(&l->tok[l->ntok], p);
        l->ntok++;
        p = e;
    }
    line_refresh(l);
    return l;
}

void lines_push(Shc *h, Lines *l, Line *ln)
{
    if (l->n == l->cap) {
        int cap = l->cap ? l->cap * 2 : 256;
        Line **nv = (Line **)sh_alloc(h, (size_t)cap * sizeof(Line *));
        if (l->n) memcpy(nv, l->v, (size_t)l->n * sizeof(Line *));
        l->v = nv; l->cap = cap;
    }
    l->v[l->n++] = ln;
}

/* ---------------- strip_dead_bra ---------------- */
static void strip_dead_bra(Lines *L)
{
    int i, k = 0, n = L->n;
    for (i = 0; i < n; i++) {
        Line *l = L->v[i];
        if (!l->label && l->op == OP_BRA && i + 1 < n && L->v[i + 1]->label && !strcmp(L->v[i + 1]->label, l->tok[1].s)) continue;
        L->v[k++] = l;
    }
    L->n = k;
}

/* glslc.drop_labels: a label nothing jumps to goes (an inlined function's epilogue once strip_dead_bra has dropped the BRA to it) */
static void drop_labels(Shc *h, Lines *L)
{
    Dict refs;
    int i, t, k = 0, n = L->n;
    memset(&refs, 0, sizeof refs);
    for (i = 0; i < n; i++) {
        const Line *l = L->v[i];
        if (!l->label) for (t = 1; t < l->ntok; t++) if (l->tok[t].s) dict_put(h, &refs, l->tok[t].s);
    }
    for (i = 0; i < n; i++) if (!L->v[i]->label || dict_find(&refs, L->v[i]->label)) L->v[k++] = L->v[i];
    L->n = k;
}

/* ---------------- _loop_regions ---------------- */
typedef struct Loop { int lo, hi; } Loop;

static int loop_regions(Shc *h, const Lines *L, Loop **out)
{
    Dict lab;
    int i, n = L->n, nl = 0, cap = 0, sp = 0, stcap = 0, hasbra = 0;
    int *st = NULL;
    Loop *loops = NULL;
    memset(&lab, 0, sizeof lab);
    for (i = 0; i < n; i++) if (L->v[i]->op == OP_BRA) { hasbra = 1; break; }
    if (hasbra) for (i = 0; i < n; i++) if (L->v[i]->label) dict_put(h, &lab, L->v[i]->label)->i = i;
    for (i = 0; i < n; i++) {
        Line *l = L->v[i];
        int lo = -1;
        if (l->label) continue;
        if (l->op == OP_BRA) {
            DictEnt *e = dict_find(&lab, l->tok[1].s);
            if (e && e->i < i) lo = (int)e->i;
        }
        else if (l->op == OP_LOOP) {
            if (sp == stcap) {
                int nc = stcap ? stcap * 2 : 8;
                int *ns = (int *)sh_alloc(h, (size_t)nc * sizeof(int));
                if (sp) memcpy(ns, st, (size_t)sp * sizeof(int));
                st = ns; stcap = nc;
            }
            st[sp++] = i;
        }
        else if (l->op == OP_ENDL && sp) lo = st[--sp];
        if (lo >= 0) {
            if (nl == cap) {
                int nc = cap ? cap * 2 : 8;
                Loop *nv = (Loop *)sh_alloc(h, (size_t)nc * sizeof(Loop));
                if (nl) memcpy(nv, loops, (size_t)nl * sizeof(Loop));
                loops = nv; cap = nc;
            }
            loops[nl].lo = lo; loops[nl].hi = i; nl++;
        }
    }
    *out = loops;
    return nl;
}

/* ---------------- dce ---------------- */
static int any_in_range(const int *v, int n, int lo, int hi)      /* any(lo <= x < hi): v ascending */
{
    int a = 0, b = n;
    while (a < b) { int m = (a + b) / 2; if (v[m] < lo) a = m + 1; else b = m; }
    return a < n && v[a] < hi;
}

static void dce(Shc *h, Lines *L, int nv)
{
    int changed = 1;
    while (changed) {
        ShMark mk = sh_mark(h);
        Loop *loops;
        int nloops = loop_regions(h, L, &loops), n = L->n, i, j, ndead = 0;
        int *start = (int *)sh_zalloc(h, (size_t)(nv + 3) * sizeof(int)), *reads;
        unsigned char *dead = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
        changed = 0;
        for (i = 0; i < n; i++) {
            Line *l = L->v[i];
            if (l->label) continue;
            for (j = 0; j < l->nsrc; j++) start[l->src[j] + 2]++;
            if (l->dst >= 0 && l->has_dot) start[l->dst + 2]++;
        }
        for (i = 2; i < nv + 3; i++) start[i] += start[i - 1];
        reads = (int *)sh_alloc(h, (size_t)(start[nv + 2] + 1) * sizeof(int));
        for (i = 0; i < n; i++) {                 /* start[r + 1] walks from the start of r's run to its end */
            Line *l = L->v[i];
            if (l->label) continue;
            for (j = 0; j < l->nsrc; j++) reads[start[l->src[j] + 1]++] = i;
            if (l->dst >= 0 && l->has_dot) reads[start[l->dst + 1]++] = i;
        }
        for (i = 0; i < n; i++) {
            Line *l = L->v[i];
            int d, used, rb, re;
            if (l->label || (flags_of(l) & F_SIDE)) continue;
            d = l->dst;
            if (d < 0) continue;
            rb = start[d]; re = start[d + 1];
            used = re > rb && reads[re - 1] > i;
            if (!used) {
                int k;
                for (k = 0; k < nloops; k++) {
                    if (loops[k].lo <= i && i <= loops[k].hi && any_in_range(reads + rb, re - rb, loops[k].lo, i)) { used = 1; break; }
                }
            }
            if (!used) { dead[i] = 1; ndead++; }
        }
        if (ndead) {
            int k = 0;
            for (i = 0; i < n; i++) if (!dead[i]) L->v[k++] = L->v[i];
            L->n = k;
            changed = 1;
        }
        sh_release(h, mk);
    }
}

/* the batched _replace_src: the chosen registers are pairwise disjoint, so one pass with a map equals Python's sequence */
static void apply_map(Lines *L, const int *remap, const unsigned char *del)
{
    int i, k = 0, n = L->n;
    for (i = 0; i < n; i++) {
        Line *l = L->v[i];
        int t, touched = 0;
        if (del[i]) continue;
        L->v[k++] = l;
        if (l->label) continue;
        for (t = 1; t < l->ntok; t++) {
            Tok *x = &l->tok[t];
            if (tok_isreg(x) && remap[x->num]) { x->num = remap[x->num]; touched = 1; }
        }
        if (touched) line_refresh(l);
    }
    L->n = k;
}

/* ---------------- copy_prop ---------------- */
static void copy_prop(Shc *h, Lines *L, int nv)
{
    int changed = 1;
    while (changed) {
        ShMark mk = sh_mark(h);
        int n = L->n, i, j, nchosen = 0;
        int *defc = (int *)sh_zalloc(h, (size_t)(nv + 2) * sizeof(int));
        int *lastread = (int *)sh_alloc(h, (size_t)(nv + 2) * sizeof(int));
        int *dstart = (int *)sh_zalloc(h, (size_t)(nv + 3) * sizeof(int)), *defs;
        int *remap = (int *)sh_zalloc(h, (size_t)(nv + 2) * sizeof(int));
        unsigned char *used = (unsigned char *)sh_zalloc(h, (size_t)nv + 2);
        unsigned char *del = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
        changed = 0;
        memset(lastread, 0xFF, (size_t)(nv + 2) * sizeof(int));
        for (i = 0; i < n; i++) {
            Line *l = L->v[i];
            if (l->label) continue;
            if (l->dst >= 0) { defc[l->dst]++; dstart[l->dst + 2]++; }
            for (j = 0; j < l->nsrc; j++) lastread[l->src[j]] = i;
        }
        for (i = 2; i < nv + 3; i++) dstart[i] += dstart[i - 1];
        defs = (int *)sh_alloc(h, (size_t)(dstart[nv + 2] + 1) * sizeof(int));
        for (i = 0; i < n; i++) {
            Line *l = L->v[i];
            if (!l->label && l->dst >= 0) defs[dstart[l->dst + 1]++] = i;
        }
        for (i = 0; i < n; i++) {
            Line *l = L->v[i];
            int dv, sv, lastuse;
            if (l->label) continue;
            if (l->op != OP_MOV || l->has_dot || l->ntok != 3 || tok_hasmod(&l->tok[2]) || tok_hasswz(&l->tok[2])) continue;
            dv = tok_num(&l->tok[1]); sv = tok_num(&l->tok[2]);
            if (dv < 0 || sv < 0 || dv == sv || defc[dv] != 1) continue;
            if (used[dv] || used[sv]) continue;
            lastuse = lastread[dv] > i ? lastread[dv] : -1;
            if (lastuse >= 0 && any_in_range(defs + dstart[sv], dstart[sv + 1] - dstart[sv], i + 1, lastuse + 1)) continue;
            del[i] = 1; remap[dv] = sv; used[dv] = 1; used[sv] = 1; nchosen++;
        }
        if (nchosen) { apply_map(L, remap, del); changed = 1; }
        sh_release(h, mk);
    }
}

/* ---------------- cse ---------------- */
static int rhs_start(const Line *l) { return l->op == OP_SET ? 3 : 2; }

static int rhs_eq(const Line *a, const Line *b)               /* _rhs(a) == _rhs(b) */
{
    int i, s;
    if (a->op != b->op || strcmp(a->mask, b->mask) || a->has_dot != b->has_dot || a->ntok != b->ntok) return 0;
    if (a->op == OP_SET && !tok_eq(&a->tok[1], &b->tok[1])) return 0;
    s = rhs_start(a);
    for (i = s; i < a->ntok; i++) if (!tok_eq(&a->tok[i], &b->tok[i])) return 0;
    return 1;
}

static unsigned tok_hash(const Tok *t)
{
    unsigned h = 2166136261u;
    if (t->s) {
        const char *p;
        for (p = t->s; *p; p++) h = (h ^ (unsigned char)*p) * 16777619u;
    } else {
        int i;
        h = (h ^ (unsigned)t->num) * 16777619u;
        h = (h ^ (unsigned char)t->mod) * 16777619u;
        for (i = 0; i < 4; i++) h = (h ^ (unsigned char)t->swz[i]) * 16777619u;
    }
    return h;
}

static unsigned rhs_hash(const Line *l)
{
    unsigned h = (unsigned)l->op * 2654435761u;
    int i;
    if (l->op == OP_SET) h ^= tok_hash(&l->tok[1]);
    for (i = rhs_start(l); i < l->ntok; i++) h = (h * 31u) ^ tok_hash(&l->tok[i]);
    return h;
}

/* ldv_tex (a fragment program): an LDV looks past a TEX for its twin, the two varyings of one row the link packed */
static void cse(Shc *h, Lines *L, int nv, int skip_ldp, int ldv_tex)
{
    int changed = 1;
    while (changed) {
        ShMark mk = sh_mark(h);
        int n = L->n, i, k, nchosen = 0;
        int *defc = (int *)sh_zalloc(h, (size_t)(nv + 2) * sizeof(int));
        int *remap = (int *)sh_zalloc(h, (size_t)(nv + 2) * sizeof(int));
        unsigned char *used = (unsigned char *)sh_zalloc(h, (size_t)nv + 2);
        unsigned char *del = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
        unsigned *hash = (unsigned *)sh_alloc(h, ((size_t)n + 1) * sizeof(unsigned));
        changed = 0;
        for (i = 0; i < n; i++) {
            Line *l = L->v[i];
            if (l->label) continue;
            if (l->dst >= 0) defc[l->dst]++;
            hash[i] = rhs_hash(l);
        }
        for (i = 0; i < n; i++) {
            Line *li = L->v[i];
            int di, hit = -1, dk = -1;
            if (li->label) continue;
            if (skip_ldp && li->op == OP_LDP) continue;
            if ((flags_of(li) & (F_SIDE | F_CTRL)) || li->has_dot) continue;
            di = li->dst;
            if (di < 0 || defc[di] != 1 || used[di]) continue;
            for (k = i + 1; k < n; k++) {
                Line *lk = L->v[k];
                int fl, s, stop = 0;
                if (lk->label) continue;
                fl = flags_of(lk);
                if ((fl & F_CTRL) && !(ldv_tex && lk->op == OP_TEX && li->op == OP_LDV)) break;
                dk = lk->dst;
                if (dk == di) break;
                if (dk >= 0) for (s = 0; s < li->nsrc; s++) if (li->src[s] == dk) { stop = 1; break; }
                if (stop) break;
                if (hash[k] == hash[i] && !(fl & F_SIDE) && !lk->has_dot && dk >= 0 && defc[dk] == 1 && rhs_eq(lk, li)) { hit = k; break; }
            }
            if (hit >= 0 && !used[dk] && dk != di) {
                del[hit] = 1; remap[dk] = di; used[di] = 1; used[dk] = 1; nchosen++;
            }
        }
        if (nchosen) { apply_map(L, remap, del); changed = 1; }
        sh_release(h, mk);
    }
}

/* ---------------- hoist_consts ---------------- */
static int is_const_ld(const Line *l)
{
    return !l->label && l->op == OP_LDP && !l->has_dot && l->ntok == 3 && l->tok[2].s && l->tok[2].s[0] == '@';
}

static void hoist_consts(Shc *h, Lines *L)
{
    int i, k = 0, any = 0, nc = 0, n = L->n;
    Line **nv;
    for (i = 0; i < n; i++) {
        int op = L->v[i]->op;
        if (!L->v[i]->label && !L->v[i]->has_dot && (op == OP_BRP || op == OP_BRA || op == OP_IF || op == OP_ELSE || op == OP_LOOP)) { any = 1; break; }
    }
    if (!any) return;
    for (i = 0; i < n; i++) if (is_const_ld(L->v[i])) nc++;
    if (!nc) return;
    nv = (Line **)sh_alloc(h, ((size_t)L->cap + 1) * sizeof(Line *));
    for (i = 0; i < n; i++) if (is_const_ld(L->v[i])) nv[k++] = L->v[i];
    for (i = 0; i < n; i++) if (!is_const_ld(L->v[i])) nv[k++] = L->v[i];
    L->v = nv;
}

/* ---------------- hoist_tex ---------------- */
/* a line reads r: a source, or the old lanes of its masked write */
static int reads_reg(const Line *l, int r)
{
    int s;
    if (r < 0) return 0;
    for (s = 0; s < l->nsrc; s++) if (l->src[s] == r) return 1;
    return l->dst == r && l->has_dot && (flags_of(l) & F_ALU);
}

/* _interleave: the work every TEX coordinate of the front needs first, then the TEXes, each as soon as what it reads is there */
static void interleave(Shc *h, Line **v, int n)
{
    ShMark mk = sh_mark(h);
    unsigned char *pred = (unsigned char *)sh_zalloc(h, (size_t)n * (size_t)n + 1), *done = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    Line **o = (Line **)sh_alloc(h, ((size_t)n + 1) * sizeof(Line *));
    int a, b, k, x;
    for (b = 0; b < n; b++) {
        for (a = 0; a < b; a++) {
            int da = v[a]->dst, db = v[b]->dst;
            if ((da >= 0 && (reads_reg(v[b], da) || da == db)) || (db >= 0 && reads_reg(v[a], db))) pred[(size_t)b * n + a] = 1;
        }
    }
    for (k = 0; k < n; k++) {
        int pick = -1, pass;
        for (pass = 0; pass < 2 && pick < 0; pass++) {
            for (x = 0; x < n && pick < 0; x++) {
                int ready = 1;
                if (done[x] || (pass == 0 && v[x]->op == OP_TEX)) continue;
                for (a = 0; a < x; a++) if (pred[(size_t)x * n + a] && !done[a]) { ready = 0; break; }
                if (ready) pick = x;
            }
        }
        o[k] = v[pick]; done[pick] = 1;
    }
    memcpy(v, o, (size_t)n * sizeof(Line *));
    sh_release(h, mk);
}

static void sched_region(Shc *h, Line **region, int m, int *wr_head, int nv, Line **out)
{
    ShMark mk;
    int k, has = 0, sp = 0, ins = 0;
    int *wr_next, *stack;
    unsigned char *inslice;
    (void)nv;
    for (k = 0; k < m; k++) if (region[k]->op == OP_TEX) has = 1;
    if (m < 2 || !has) { memcpy(out, region, (size_t)m * sizeof(Line *)); return; }
    mk = sh_mark(h);
    wr_next = (int *)sh_alloc(h, (size_t)m * sizeof(int));
    stack = (int *)sh_alloc(h, ((size_t)m + 1) * sizeof(int));
    inslice = (unsigned char *)sh_zalloc(h, (size_t)m);
    for (k = 0; k < m; k++) {
        int d = region[k]->dst;
        if (d >= 0) { wr_next[k] = wr_head[d]; wr_head[d] = k; }
    }
    for (k = 0; k < m; k++) if (region[k]->op == OP_TEX) { stack[sp++] = k; inslice[k] = 1; ins++; }
    while (sp) {
        int q = stack[--sp], s;
        Line *l = region[q];
        int nr = l->nsrc + 1;
        for (s = 0; s < nr; s++) {                /* the sources, and the old lanes of a masked write */
            int r, w;
            if (s < l->nsrc) r = l->src[s];
            else {
                if (l->dst < 0 || !l->has_dot || !(flags_of(l) & F_ALU)) break;
                r = l->dst;
            }
            for (w = wr_head[r]; w >= 0; w = wr_next[w]) {
                if (w < q && !inslice[w]) { inslice[w] = 1; ins++; stack[sp++] = w; }
            }
        }
    }
    for (k = 0; k < m; k++) if (region[k]->dst >= 0) wr_head[region[k]->dst] = -1;
    if (ins == m) memcpy(out, region, (size_t)m * sizeof(Line *));
    else {
        int o = 0;
        for (k = 0; k < m; k++) if (inslice[k]) out[o++] = region[k];
        for (k = 0; k < m; k++) if (!inslice[k]) out[o++] = region[k];
    }
    sh_release(h, mk);
    interleave(h, out, ins);                    /* glslc's interleave=True (forward) */
}

static void hoist_tex(Shc *h, Lines *L, int nv)
{
    int i = 0, n = L->n, o = 0;
    Line **out = (Line **)sh_alloc(h, ((size_t)L->cap + 1) * sizeof(Line *));
    int *wr_head = (int *)sh_alloc(h, (size_t)(nv + 2) * sizeof(int));
    memset(wr_head, 0xFF, (size_t)(nv + 2) * sizeof(int));
    while (i < n) {
        Line *l = L->v[i];
        int lo;
        if (l->label || (flags_of(l) & F_SIDE)) { out[o++] = l; i++; continue; }
        lo = i;
        while (i < n && !(L->v[i]->label || (flags_of(L->v[i]) & F_SIDE))) i++;
        sched_region(h, L->v + lo, i - lo, wr_head, nv, out + o);
        o += i - lo;
    }
    L->v = out;
}

/* ---------------- _mad_fuse (the isa64 form: on virtual registers, no addend limits) ---------------- */
static void mad_fuse(Shc *h, Lines *L)
{
    int changed = 1;
    while (changed) {
        int i = 0;
        changed = 0;
        while (i < L->n) {
            Line *ln = L->v[i];
            int fused = 0, pass;
            if (ln->label || (ln->op != OP_ADD && ln->op != OP_SUB) || ln->ntok != 4) { i++; continue; }
            for (pass = 0; pass < 2 && !fused; pass++) {
                const Tok *tmp = &ln->tok[pass ? 3 : 2], *other = &ln->tok[pass ? 2 : 3];
                int temp_first = !pass, pneg, oc, tv, j, ok = 1, k, dead, an, bn;
                Tok a, b;
                int have = 0;
                if (!tok_isreg(tmp) || tmp->swz[0]) continue;
                /* _mad_signs: an abs on the temp ('!' or '-!') is not a sign of the product */
                if (tmp->mod == '!' || tmp->mod == '~') continue;
                pneg = tmp->mod == '-';
                if (ln->op == OP_SUB) {
                    if (temp_first) continue;
                    pneg = !pneg;
                }
                oc = tok_num(other); tv = tmp->num;
                if (oc < 0 || oc == tv) continue;
                for (j = i - 1; j >= 0; j--) {
                    Line *lj = L->v[j];
                    int s;
                    if (lj->label || (flags_of(lj) & F_CTRL)) { ok = 0; break; }
                    for (s = 0; s < lj->nsrc; s++) if (lj->src[s] == tv) { ok = 0; break; }
                    if (!ok) break;
                    if (lj->dst == tv) {
                        if (lj->op == OP_MUL && !lj->has_dot && lj->ntok == 4 && !tok_hasmod(&lj->tok[2]) && !tok_hasmod(&lj->tok[3])) {
                            a = lj->tok[2]; b = lj->tok[3]; have = 1;
                        }
                        break;
                    }
                }
                if (!ok || !have) continue;
                an = tok_num(&a); bn = tok_num(&b);
                ok = 1;
                for (k = j + 1; k < i; k++) {
                    Line *lk = L->v[k];
                    if (lk->label) continue;
                    if (lk->dst == an || lk->dst == bn) { ok = 0; break; }
                }
                if (!ok) continue;
                dead = 1;
                for (k = i + 1; k < L->n; k++) {
                    Line *lk = L->v[k];
                    int s, rd = 0;
                    if (lk->label) continue;
                    for (s = 0; s < lk->nsrc; s++) if (lk->src[s] == tv) { rd = 1; break; }
                    if (rd) { dead = 0; break; }
                    if (lk->dst == tv) { if (lk->has_dot) dead = 0; break; }     /* virtual registers: a masked write keeps tmp's other lanes */
                }
                if (!dead) continue;
                {
                    Line *m = (Line *)sh_alloc(h, sizeof(Line));
                    memset(m, 0, sizeof *m);
                    m->op = OP_MAD; m->mask = ln->mask; m->has_dot = ln->has_dot; m->ntok = 5;
                    m->tok[1] = ln->tok[1];
                    m->tok[2] = a;
                    m->tok[3] = b;
                    if (pneg) {
                        if (tok_isreg(&b)) m->tok[3].mod = '-';
                        else m->tok[3].s = sh_fmt(h, "-%s", b.s);
                    }
                    m->tok[4] = *other;
                    line_refresh(m);
                    L->v[i] = m;
                    memmove(L->v + j, L->v + j + 1, (size_t)(L->n - j - 1) * sizeof(Line *));
                    L->n--;
                    changed = 1; fused = 1;
                }
            }
            if (!fused) i++;
        }
    }
}

/* ---------------- allocate_registers + _rewrite_packed ---------------- */
typedef struct Act { int end, ph, cp, v; } Act;
typedef struct Ord { int first, last, seq, v; } Ord;

static int ord_cmp(const void *pa, const void *pb)
{
    const Ord *a = (const Ord *)pa, *b = (const Ord *)pb;
    if (a->first != b->first) return a->first < b->first ? -1 : 1;
    if (a->last != b->last) return a->last < b->last ? -1 : 1;
    return a->seq < b->seq ? -1 : a->seq > b->seq;
}

static SHC_NORETURN void spill(Shc *h, int s, int live)
{
    sh_compile_error(h, "register spill needed at instruction %d (%d live > 31) "
                        "\xE2\x80\x94 shader exceeds 31 live registers (split it or reduce temporaries)", s, live);
}

/* one line of the output: Python's new[] joined with ' ', @N resolved */
static void put_tok_text(SB *o, const char *s, long long base)
{
    if (s[0] == '@') {
        const char *p = s + 1;
        long long n = 0;
        while (*p >= '0' && *p <= '9') { n = n * 10 + (*p - '0'); p++; }
        sb_putll(o, base + n);
        sb_puts(o, p);
    }
    else sb_puts(o, s);
}

static char *allocate_registers(Shc *h, Lines *L, int nv, const unsigned char *svregs, long long base)
{
    enum { NPHYS = 31 };
    int n = L->n, i, t, nord = 0, nloops, nactive = 0;
    int *first = (int *)sh_alloc(h, (size_t)(nv + 2) * sizeof(int));
    int *last = (int *)sh_alloc(h, (size_t)(nv + 2) * sizeof(int));
    int *mapping = (int *)sh_zalloc(h, (size_t)(nv + 2) * sizeof(int));
    signed char *comp = (signed char *)sh_alloc(h, (size_t)nv + 2);
    Ord *order = (Ord *)sh_alloc(h, (size_t)(nv + 2) * sizeof(Ord));
    Act active[NPHYS * 4 + 2];
    uint32_t freep = 0xFFFFFFFEu;                  /* r1..r31: the list + sort + pop of the lowest */
    unsigned char partial[32], inpartial[32];
    Loop *loops;
    SB o;
    memset(first, 0xFF, (size_t)(nv + 2) * sizeof(int));
    memset(comp, 0xFF, (size_t)nv + 2);
    memset(inpartial, 0, sizeof inpartial);
    memset(partial, 0, sizeof partial);
    for (i = 0; i < n; i++) {
        Line *l = L->v[i];
        if (l->label) continue;
        for (t = 1; t < l->ntok; t++) {
            if (tok_isreg(&l->tok[t])) {
                int v = l->tok[t].num;
                if (first[v] < 0) { first[v] = i; order[nord].seq = nord; order[nord].v = v; nord++; }
                last[v] = i;
            }
        }
    }
    nloops = loop_regions(h, L, &loops);
    for (i = 0; i < nloops; i++) {
        int lo = loops[i].lo, hi = loops[i].hi, k;
        for (k = 0; k < nord; k++) {
            int v = order[k].v;
            if (first[v] <= lo && last[v] >= lo) { if (hi > last[v]) last[v] = hi; }
        }
    }
    for (i = 0; i < nord; i++) { order[i].first = first[order[i].v]; order[i].last = last[order[i].v]; }
    qsort(order, (size_t)nord, sizeof(Ord), ord_cmp);
    for (i = 0; i < nord; i++) {
        int v = order[i].v, s = first[v], k, keep = 0, ph, cp = -1;
        for (k = 0; k < nactive; k++) {
            Act *a = &active[k];
            if (a->end < s) {
                if (a->cp < 0) freep |= (uint32_t)1 << a->ph;
                else {
                    partial[a->ph] |= (unsigned char)(1 << a->cp);
                    if (partial[a->ph] == 0xF) { inpartial[a->ph] = 0; freep |= (uint32_t)1 << a->ph; }
                }
            }
            else active[keep++] = *a;
        }
        nactive = keep;
        if (svregs[v]) {
            ph = -1;
            for (k = 1; k < 32; k++) if (inpartial[k] && partial[k]) { ph = k; break; }
            if (ph < 0) {
                if (!freep) spill(h, s, nactive + 1);
                for (ph = 1; !((freep >> ph) & 1); ph++) { }
                freep &= ~((uint32_t)1 << ph);
                inpartial[ph] = 1; partial[ph] = 0xF;
            }
            for (cp = 0; !((partial[ph] >> cp) & 1); cp++) { }
            partial[ph] &= (unsigned char)~(1 << cp);
            comp[v] = (signed char)cp;
        } else {
            if (!freep) spill(h, s, nactive + 1);
            for (ph = 1; !((freep >> ph) & 1); ph++) { }
            freep &= ~((uint32_t)1 << ph);
        }
        mapping[v] = ph;
        active[nactive].end = last[v]; active[nactive].ph = ph; active[nactive].cp = cp; active[nactive].v = v;
        nactive++;
    }
    /* _rewrite_packed, then the @N resolution of _compile_stage */
    sb_init(&o, h, (size_t)n * 24 + 16);
    for (i = 0; i < n; i++) {
        Line *l = L->v[i];
        int dp, dst_packed = 0, oplen;
        if (i) sb_putc(&o, '\n');
        if (l->label) { sb_puts(&o, l->label); sb_putc(&o, ':'); continue; }
        dp = l->op == OP_SET ? 2 : (flags_of(l) & F_ALU) ? 1 : -1;
        oplen = (int)strlen(l->mask);
        for (t = 1; t < l->ntok; t++) {
            const Tok *x = &l->tok[t];
            if (tok_isreg(x) && comp[x->num] >= 0) {
                if (t == dp) {
                    if (x->swz[1] || oplen > 1) sh_compile_error(h, "packed scalar r%d written with a multi-component mask", x->num);
                    dst_packed = 1;
                }
            }
        }
        sb_puts(&o, OPT[l->op].name);
        if (l->has_dot && !(dst_packed && oplen)) { sb_putc(&o, '.'); sb_puts(&o, l->mask); }
        for (t = 1; t < l->ntok; t++) {
            const Tok *x = &l->tok[t];
            sb_putc(&o, ' ');
            if (tok_isreg(x)) {
                int v = x->num, p = mapping[v];
                if (comp[v] < 0) {
                    put_mod(&o, x->mod);
                    sb_putc(&o, 'r'); sb_putll(&o, p);
                    if (x->swz[0]) { sb_putc(&o, '.'); sb_puts(&o, x->swz); }
                } else {
                    if (t != dp) put_mod(&o, x->mod);
                    sb_putc(&o, 'r'); sb_putll(&o, p); sb_putc(&o, '.'); sb_putc(&o, "xyzw"[comp[v]]);
                }
            } else {
                put_tok_text(&o, x->s, base);
                if (t == 2 && dst_packed && (l->op == OP_LDP || l->op == OP_LDV || l->op == OP_LDPX) && !l->has_dot && !strchr(x->s, '.')) sb_puts(&o, ".x");
            }
        }
    }
    return sb_str(&o);
}

/* ---------------- block_forward + dead_stores + forward_opt: copies and dead writes per component inside a basic block ---------------- */
static int block_end(const Line *l)
{
    switch (l->op) {
    case OP_IF: case OP_ELSE: case OP_ENDIF: case OP_BRP: case OP_BRA: case OP_LOOP: case OP_ENDL: case OP_BREAK: case OP_CONT: case OP_SETP: case OP_DISCARD: case OP_END: return 1;
    default: return 0;
    }
}

static int neg_ok(int op)                                       /* the readers a copy with a modifier may reach (glslc._NEG_OK: not SEL, integer ops, TEX, LDPX) */
{
    switch (op) {
    case OP_ADD: case OP_SUB: case OP_MUL: case OP_MAD: case OP_MIN: case OP_MAX: case OP_DP3: case OP_DP4: case OP_MOV: case OP_RCP: case OP_RSQRT: case OP_LOG2: case OP_EXP2: case OP_STO: case OP_STV:
    case OP_SET: case OP_SETP: case OP_STOZ: case OP_SIN: case OP_COS: case OP_FLR: case OP_CEIL: case OP_TRUNC: case OP_RNDE:
    case OP_SQRT: case OP_RCPM: case OP_RSQM: case OP_CLAMP: return 1;
    default: return 0;
    }
}

static int comp_of(char ch) { return ch == 'x' ? 0 : ch == 'y' ? 1 : ch == 'z' ? 2 : 3; }

static void swz4(const Tok *t, int *q)                          /* isa64.enc_swz: 1 letter splats, of 2-3 the last repeats */
{
    int i, n = (int)strlen(t->swz);
    if (!n) { for (i = 0; i < 4; i++) q[i] = i; return; }
    for (i = 0; i < 4; i++) q[i] = comp_of(t->swz[i < n ? i : n - 1]);
}

static void swz_text(Tok *t, const int *q)
{
    memset(t->swz, 0, sizeof t->swz);
    if (q[0] == 0 && q[1] == 1 && q[2] == 2 && q[3] == 3) return;
    t->swz[0] = "xyzw"[q[0]];
    if (q[1] != q[0] || q[2] != q[0] || q[3] != q[0]) { t->swz[1] = "xyzw"[q[1]]; t->swz[2] = "xyzw"[q[2]]; t->swz[3] = "xyzw"[q[3]]; }
}

static int wmask(const Line *l)
{
    const char *m;
    int w = 0;
    if (!l->has_dot) return 15;
    for (m = l->mask; *m; m++) w |= 1 << comp_of(*m);
    return w;
}

static int dst_pos(const Line *l) { return l->op == OP_SET ? 2 : (flags_of(l) & F_ALU) ? 1 : -1; }

typedef struct Copy { int s; signed char c, mod; } Copy;        /* s < 0: no copy; mod: the copy's modifier, 0..3 */

static int read_pos(const Line *l);
static int read_mask(const Line *l, const Tok *t);

/* only the swizzle positions the instruction reads count: a TEX's .xy reads two of a vec4 a constructor built (gl_ff.c's tightened coordinates) */
static int block_forward(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    ShMark mk = sh_mark(h);
    size_t size = ((size_t)nv + 2) * 4;
    Copy *cp = (Copy *)sh_alloc(h, size * sizeof(Copy));
    int i, k, j, kept = 0, changed = 0, n = L->n;
    size_t e;
#define CP_RESET() do { for (e = 0; e < size; e++) cp[e].s = -1; } while (0)
    CP_RESET();
    for (i = 0; i < n; i++) {
        Line *l = L->v[i];
        int dp, d;
        if (l->label) { CP_RESET(); L->v[kept++] = l; continue; }
        dp = dst_pos(l);
        d = l->dst;
        for (k = 1; k < l->ntok; k++) {
            Tok *t = &l->tok[k];
            int q[4], nq[4], xs[4], xn[4], s = -1, cm = 0, same = 1, diff = 0, pos, last = -1, ident = 1;
            char nmod;
            if (k == dp || !tok_isreg(t)) continue;
            swz4(t, q);
            pos = read_pos(l);
            for (j = 0; j < 4; j++) {
                const Copy *x = &cp[(size_t)t->num * 4 + (size_t)q[j]];
                xs[j] = x->s >= 0 ? x->s : t->num; nq[j] = x->s >= 0 ? x->c : q[j]; xn[j] = x->s >= 0 ? x->mod : 0;
                if (!(pos >> j & 1)) continue;
                if (last < 0) { s = xs[j]; cm = xn[j]; }
                else if (xs[j] != s || xn[j] != cm) same = 0;
                if (nq[j] != q[j]) diff = 1;
                if (nq[j] != j) ident = 0;
                last = j;
            }
            if (!same || last < 0) continue;
            for (j = 0; j < 4; j++) if (!(pos >> j & 1) && (xs[j] != s || xn[j] != cm)) nq[j] = ident ? j : nq[last];   /* a position nothing reads: as its own, or the last read one's */
            if (s == t->num && !diff && !cm) continue;
            if (cm && !neg_ok(l->op)) continue;
            if (s == d && !(nq[0] == 0 && nq[1] == 1 && nq[2] == 2 && nq[3] == 3)) continue;
            nmod = cm ? MODCH[mod_comp(mod_bits(t->mod), cm)] : t->mod;
            t->num = s; t->mod = nmod;
            if (sv[s]) memset(t->swz, 0, sizeof t->swz); else swz_text(t, nq);
            changed = 1;
        }
        line_refresh(l);
        if (l->op == OP_MOV && l->ntok == 3 && d >= 0 && tok_isreg(&l->tok[2]) && l->tok[2].num == d && !l->tok[2].mod && !l->tok[2].swz[0]) { changed = 1; continue; }   /* the identity, under any mask */
        L->v[kept++] = l;
        if (block_end(l)) { CP_RESET(); continue; }
        if (d < 0) continue;
        {
            int wm = sv[d] ? 15 : wmask(l);
            for (e = 0; e < size; e++) {
                Copy *x = &cp[e];
                if (x->s < 0) continue;
                if (((int)(e / 4) == d && (wm >> (e & 3) & 1)) || (x->s == d && (wm >> x->c & 1))) x->s = -1;
            }
            if (l->op == OP_MOV && l->ntok == 3 && tok_isreg(&l->tok[2]) && l->tok[2].num != d) {
                int q[4], c;
                swz4(&l->tok[2], q);
                for (c = 0; c < 4; c++) {
                    if (wm >> c & 1) {
                        Copy *x = &cp[(size_t)d * 4 + (size_t)c];
                        x->s = l->tok[2].num; x->c = (signed char)q[c]; x->mod = (signed char)mod_bits(l->tok[2].mod);
                    }
                }
            }
        }
    }
#undef CP_RESET
    L->n = kept;
    sh_release(h, mk);
    return changed;
}

/* the reads of a register, ascending; read_from: read at or after `end`, or in a loop around it (the back-edge) */
typedef struct Reads { const int *start, *v; const Loop *loops; int nloops; } Reads;
static int read_from(const Reads *R, int r, int end)
{
    int a = R->start[r], b = R->start[r + 1], i;
    if (a == b) return 0;
    if (R->v[b - 1] >= end) return 1;
    for (i = 0; i < R->nloops; i++) if (R->loops[i].lo <= end && end <= R->loops[i].hi && any_in_range(R->v + a, b - a, R->loops[i].lo, end)) return 1;
    return 0;
}

static int dead_stores(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    ShMark mk = sh_mark(h);
    Loop *loops;
    int nloops = loop_regions(h, L, &loops), n = L->n, i, j, k, end = n, kept = 0, changed = 0, total = 0;
    int *start = (int *)sh_zalloc(h, ((size_t)nv + 3) * sizeof(int)), *fill = (int *)sh_alloc(h, ((size_t)nv + 3) * sizeof(int)), *reads;
    signed char *over = (signed char *)sh_alloc(h, (size_t)nv + 2);
    unsigned char *del = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    Reads R;
    for (i = 0; i < n; i++) if (!L->v[i]->label) for (j = 0; j < L->v[i]->nsrc; j++) { start[L->v[i]->src[j] + 1]++; total++; }
    for (i = 1; i < nv + 3; i++) start[i] += start[i - 1];
    memcpy(fill, start, ((size_t)nv + 3) * sizeof(int));
    reads = (int *)sh_alloc(h, ((size_t)total + 1) * sizeof(int));
    for (i = 0; i < n; i++) if (!L->v[i]->label) for (j = 0; j < L->v[i]->nsrc; j++) reads[fill[L->v[i]->src[j]]++] = i;
    R.start = start; R.v = reads; R.loops = loops; R.nloops = nloops;
    memset(over, -1, (size_t)nv + 2);
#define OV(r) (over[r] >= 0 ? over[r] : (over[r] = read_from(&R, (r), end) ? 0 : 15))
    for (i = n - 1; i >= 0; i--) {
        Line *l = L->v[i];
        int dp, d;
        if (l->label) continue;                                 /* a jump here cannot reach what precedes: every way out of that code ends its block */
        if (block_end(l)) { memset(over, -1, (size_t)nv + 2); end = i; }
        dp = dst_pos(l);
        d = l->dst;
        if (d >= 0) {
            int wm = sv[d] ? 15 : wmask(l), o = OV(d);
            if (!(wm & ~o)) { del[i] = 1; changed = 1; continue; }
            over[d] = (signed char)(o | wm);
        }
        for (k = 1; k < l->ntok; k++) {
            const Tok *t = &l->tok[k];
            if (k != dp && tok_isreg(t)) {
                int r = t->num, o = OV(r);
                over[r] = (signed char)(o & ~read_mask(l, t));
            }
        }
    }
#undef OV
    for (i = 0; i < n; i++) if (!del[i]) L->v[kept++] = L->v[i];
    L->n = kept;
    sh_release(h, mk);
    return changed;
}

/* narrow_masks (after mad_fuse): a component-wise write loses the components nothing reads before they are written again */
static const char *const MASKTXT[16] = {"", "x", "y", "xy", "z", "xz", "yz", "xyz", "w", "xw", "yw", "xyw", "zw", "xzw", "yzw", "xyzw"};

static int narrow_op(int op)
{
    switch (op) {
    case OP_ADD: case OP_SUB: case OP_MUL: case OP_MAD: case OP_MIN: case OP_MAX: case OP_MOV: case OP_RCP: case OP_RSQRT: case OP_LOG2: case OP_EXP2:
    case OP_SIN: case OP_COS: case OP_FLR: case OP_CEIL: case OP_TRUNC: case OP_RNDE: case OP_SQRT: case OP_RCPM: case OP_RSQM: case OP_CLAMP: return 1;
    default: return 0;
    }
}

/* glslc._exposed: is register r read at the top of the loop lo..hi before a whole write at the loop's own level; a LOOP 1 has no next pass.
 * odot: each line's mask as it was before narrow_masks began (it narrows in place; Python looks at the lines it was given) */
static int exposed(const Lines *L, const unsigned char *odot, int r, int lo, int hi)
{
    const Line *top = L->v[lo];
    int k, j, depth = 0;
    if (top->ntok > 1 && top->tok[1].s && !strcmp(top->tok[1].s, "1")) return 0;
    for (k = lo + 1; k < hi; k++) {
        const Line *l = L->v[k];
        if (l->label) continue;
        for (j = 0; j < l->nsrc; j++) if (l->src[j] == r) return 1;
        if (l->dst == r) return odot[k] || depth ? 1 : 0;
        if (l->op == OP_IF || l->op == OP_LOOP) depth++;
        else if (l->op == OP_ENDIF || l->op == OP_ENDL) depth--;
    }
    return 0;
}

/* read_from for narrow_masks: a read earlier in a loop counts only when the register goes round it */
static int read_from_nm(const Reads *R, const Lines *L, const unsigned char *odot, int r, int end)
{
    int a = R->start[r], b = R->start[r + 1], i;
    if (a == b) return 0;
    if (R->v[b - 1] >= end) return 1;
    for (i = 0; i < R->nloops; i++) if (R->loops[i].lo <= end && end <= R->loops[i].hi && any_in_range(R->v + a, b - a, R->loops[i].lo, end) && exposed(L, odot, r, R->loops[i].lo, R->loops[i].hi)) return 1;
    return 0;
}

static void narrow_masks(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    ShMark mk = sh_mark(h);
    Loop *loops;
    int nloops = loop_regions(h, L, &loops), n = L->n, i, j, k, end = n, total = 0;
    int *start = (int *)sh_zalloc(h, ((size_t)nv + 3) * sizeof(int)), *fill = (int *)sh_alloc(h, ((size_t)nv + 3) * sizeof(int)), *reads;
    signed char *over = (signed char *)sh_alloc(h, (size_t)nv + 2);
    unsigned char *odot;
    Reads R;
    for (i = 0; i < n; i++) if (!L->v[i]->label) for (j = 0; j < L->v[i]->nsrc; j++) { start[L->v[i]->src[j] + 1]++; total++; }
    for (i = 1; i < nv + 3; i++) start[i] += start[i - 1];
    memcpy(fill, start, ((size_t)nv + 3) * sizeof(int));
    reads = (int *)sh_alloc(h, ((size_t)total + 1) * sizeof(int));
    for (i = 0; i < n; i++) if (!L->v[i]->label) for (j = 0; j < L->v[i]->nsrc; j++) reads[fill[L->v[i]->src[j]]++] = i;
    R.start = start; R.v = reads; R.loops = loops; R.nloops = nloops;
    odot =(unsigned char *)sh_alloc(h, (size_t)n + 1);
    for (i = 0; i < n; i++) odot[i] = (unsigned char)(L->v[i]->label ? 0 : L->v[i]->has_dot);
    memset(over, -1, (size_t)nv + 2);
#define OV(r) (over[r] >= 0 ? over[r] : (over[r] = read_from_nm(&R, L, odot, (r), end) ? 0 : 15))
    for (i = n - 1; i >= 0; i--) {
        Line *l = L->v[i];
        int dp, d;
        if (l->label) continue;
        if (block_end(l)) { memset(over, -1, (size_t)nv + 2); end = i; }
        dp = dst_pos(l);
        d = l->dst;
        if (d >= 0) {
            int wm = sv[d] ? 15 : wmask(l), o = OV(d), nm = wm & ~o;
            if (nm && nm != wm && !sv[d] && narrow_op(l->op) && !l->tok[dp].swz[0]) { l->has_dot = 1; l->mask = MASKTXT[nm]; }
            over[d] = (signed char)(o | wm);
        }
        for (k = 1; k < l->ntok; k++) {
            const Tok *t = &l->tok[k];
            if (k != dp && tok_isreg(t)) { int r = t->num, o = OV(r); over[r] = (signed char)(o & ~read_mask(l, t)); }
        }
    }
#undef OV
    sh_release(h, mk);
}

static void forward_opt(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    for (;;) {
        int before, changed = block_forward(h, L, nv, sv);
        changed |= dead_stores(h, L, nv, sv);
        before = L->n;
        dce(h, L, nv);
        if (!changed && L->n == before) return;
    }
}

/* ---------------- shape_opt (after hoist_tex): merged copies, masked results, direct SETPs, no empty ELSE ---------------- */
/* _tab: per register the lines that write it (ascending), how many tokens read it and the last line, first and last mention */
typedef struct Tab { int *first, *last, *wcnt, *wst, *wpos, *rcnt, *rmax; } Tab;

static void tab_build(Shc *h, const Lines *L, int nv, Tab *T)
{
    int i, j, n = L->n, *fill;
    size_t sz = ((size_t)nv + 3) * sizeof(int);
    T->first = (int *)sh_alloc(h, sz); T->last = (int *)sh_alloc(h, sz); T->rmax = (int *)sh_alloc(h, sz);
    T->wcnt = (int *)sh_zalloc(h, sz); T->rcnt = (int *)sh_zalloc(h, sz); T->wst = (int *)sh_zalloc(h, sz);
    memset(T->first, 0xFF, sz); memset(T->last, 0xFF, sz); memset(T->rmax, 0xFF, sz);
    for (i = 0; i < n; i++) {
        const Line *l = L->v[i];
        if (l->label) continue;
        if (l->dst >= 0) { T->wcnt[l->dst]++; if (T->first[l->dst] < 0) T->first[l->dst] = i; T->last[l->dst] = i; }
        for (j = 0; j < l->nsrc; j++) { int r = l->src[j]; T->rcnt[r]++; T->rmax[r] = i; if (T->first[r] < 0) T->first[r] = i; T->last[r] = i; }
    }
    for (i = 0; i <= nv + 1; i++) T->wst[i + 1] = T->wst[i] + T->wcnt[i];
    T->wpos = (int *)sh_alloc(h, ((size_t)T->wst[nv + 2] + 1) * sizeof(int));
    fill = (int *)sh_alloc(h, sz);
    memcpy(fill, T->wst, sz);
    for (i = 0; i < n; i++) if (!L->v[i]->label && L->v[i]->dst >= 0) T->wpos[fill[L->v[i]->dst]++] = i;
}

static int tok_plain(const Tok *t) { return tok_isreg(t) && !t->mod && !t->swz[0]; }
static int tok_splat(const Tok *t) { return tok_isreg(t) && t->swz[0] && !t->swz[1]; }
static int ends_block(const Line *l) { return l->label || block_end(l); }

static int coalesce(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    ShMark mk = sh_mark(h);
    Tab T;
    Loop *loops;
    int nloops, i, k, n = L->n, nchosen = 0;
    int *remap = (int *)sh_zalloc(h, ((size_t)nv + 2) * sizeof(int));
    unsigned char *used = (unsigned char *)sh_zalloc(h, (size_t)nv + 2), *del = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    tab_build(h, L, nv, &T);
    nloops = loop_regions(h, L, &loops);
    for (i = 0; i < n; i++) {
        Line *l = L->v[i];
        int d, s, ok = 1;
        if (l->label || l->op != OP_MOV || l->ntok != 3 || !tok_plain(&l->tok[1]) || !tok_plain(&l->tok[2])) continue;
        d = l->tok[1].num; s = l->tok[2].num;
        if (d == s || used[d] || used[s] || (sv[d] != 0) != (sv[s] != 0) || T.first[d] != i) continue;
        for (k = T.wst[s]; k < T.wst[s + 1]; k++) if (T.wpos[k] > i) { ok = 0; break; }
        if (!ok) continue;
        if (any_in_range(T.wpos + T.wst[d], T.wst[d + 1] - T.wst[d], i + 1, T.rmax[s] + 1)) continue;
        for (k = 0; k < nloops; k++) if (loops[k].lo <= T.last[d] && loops[k].hi >= i) { ok = 0; break; }
        if (!ok) continue;
        remap[d] = s; del[i] = 1; used[d] = 1; used[s] = 1; nchosen++;
    }
    if (nchosen) apply_map(L, remap, del);
    sh_release(h, mk);
    return nchosen > 0;
}

static int fold_op(int op) { return op == OP_ADD || op == OP_SUB || op == OP_MUL || op == OP_MAD || op == OP_MIN || op == OP_MAX; }

static int mask_fold(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    ShMark mk = sh_mark(h);
    Tab T;
    int i, k, n = L->n, nchosen = 0;
    unsigned char *used = (unsigned char *)sh_zalloc(h, (size_t)nv + 2), *del = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    unsigned char *done = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    tab_build(h, L, nv, &T);
    for (i = 0; i < n; i++) {
        Line *l = L->v[i], *lj;
        int d, t, j, ok = 1;
        if (l->label || l->op != OP_MOV || !l->has_dot || l->ntok != 3 || !tok_plain(&l->tok[1]) || !tok_plain(&l->tok[2])) continue;
        d = l->tok[1].num; t = l->tok[2].num;
        if (d == t || sv[d] || used[d] || used[t] || T.wcnt[t] != 1 || T.rcnt[t] != 1) continue;
        j = T.wpos[T.wst[t]];
        if (j >= i || done[j]) continue;
        lj = L->v[j];
        if (!fold_op(lj->op) || lj->has_dot || !tok_plain(&lj->tok[1])) continue;
        for (k = 2; k < lj->ntok; k++) {
            const Tok *x = &lj->tok[k];
            if (!tok_isreg(x) || x->num == t || used[x->num]) { ok = 0; break; }
        }
        if (!ok) continue;
        for (k = 2; k < lj->ntok; k++) if (lj->tok[k].num == d && lj->tok[k].swz[0]) { ok = 0; break; }
        if (!ok) continue;
        if (sv[t]) for (k = 2; k < lj->ntok; k++) if (!sv[lj->tok[k].num] && !tok_splat(&lj->tok[k])) { ok = 0; break; }   /* a scalar from splats only */
        if (!ok) continue;
        for (k = j + 1; k < i; k++) {
            const Line *lk = L->v[k];
            int s;
            if (ends_block(lk) || lk->dst == d) { ok = 0; break; }
            for (s = 0; s < lk->nsrc; s++) if (lk->src[s] == d) { ok = 0; break; }
            if (!ok) break;
        }
        if (!ok) continue;
        /* in place: the batch's registers are disjoint, so the checks of the candidates after it read what they would have */
        for (k = 2; k < lj->ntok; k++) used[lj->tok[k].num] = 1;
        used[d] = 1; used[t] = 1;
        lj->has_dot = 1; lj->mask = l->mask; lj->tok[1] = l->tok[1];
        line_refresh(lj);
        done[j] = 1; del[i] = 1; nchosen++;
    }
    if (nchosen) {
        int kept = 0;
        for (i = 0; i < n; i++) if (!del[i]) L->v[kept++] = L->v[i];
        L->n = kept;
    }
    sh_release(h, mk);
    return nchosen > 0;
}

/* _const_of: the pool constant a register holds (written once, by an LDP of the pool), NULL = none */
static const double *const_of(const Lines *L, const Tab *T, int r, const double *pool, int npool)
{
    const Line *l;
    const char *p;
    int idx = 0;
    if (r < 0 || T->wcnt[r] != 1) return NULL;
    l = L->v[T->wpos[T->wst[r]]];
    if (l->op != OP_LDP || l->has_dot || l->ntok != 3 || !l->tok[2].s || l->tok[2].s[0] != '@' || !l->tok[2].s[1]) return NULL;
    for (p = l->tok[2].s + 1; *p; p++) { if (*p < '0' || *p > '9') return NULL; idx = idx * 10 + (*p - '0'); }
    return idx < npool ? pool + (size_t)idx * 4 : NULL;
}

static int all_of(const double *k, double v) { return k && k[0] == v && k[1] == v && k[2] == v && k[3] == v; }

static Tok splat_tok(const Tok *x, int comp, const unsigned char *sv)
{
    Tok t = *x;
    int q[4];
    swz4(x, q);
    memset(t.swz, 0, sizeof t.swz);
    if (!sv[x->num]) t.swz[0] = "xyzw"[q[comp]];
    return t;
}

static int between_ok(const Lines *L, int j, int i, int r0, int r1)
{
    int k;
    for (k = j + 1; k < i; k++) {
        const Line *lk = L->v[k];
        if (ends_block(lk)) return 0;
        if (lk->dst >= 0 && (lk->dst == r0 || lk->dst == r1)) return 0;
    }
    return 1;
}

static int setp_fold(Shc *h, Lines *L, int nv, const unsigned char *sv, const double *pool, int npool)
{
    ShMark mk = sh_mark(h);
    Tab T;
    int i, n = L->n, changed = 0;
    tab_build(h, L, nv, &T);
    for (i = 0; i < n; i++) {
        Line *l = L->v[i], *lj;
        const Tok *A, *Z = NULL;
        int a, s0, j, q[4];
        if (l->label || l->op != OP_SETP || l->ntok != 4) continue;
        Z = &l->tok[3];
        if (!tok_isreg(Z) || Z->mod || !all_of(const_of(L, &T, Z->num, pool, npool), 0.0)) continue;
        A = &l->tok[2];
        if (!tok_isreg(A) || A->mod) continue;
        a = A->num; swz4(A, q); s0 = q[0];
        if (T.wcnt[a] != 1 || T.wpos[T.wst[a]] >= i) continue;
        j = T.wpos[T.wst[a]]; lj = L->v[j];
        if (!strcmp(l->tok[1].s, "SNE") && lj->op == OP_SUB && !lj->has_dot && lj->ntok == 4 && tok_plain(&lj->tok[1])) {
            const Tok *one = &lj->tok[2], *x = &lj->tok[3];
            const Line *ls;
            int sl;
            if (!tok_isreg(one) || one->mod || !all_of(const_of(L, &T, one->num, pool, npool), 1.0) || !tok_isreg(x) || x->mod) continue;
            if (T.wcnt[x->num] != 1) continue;
            sl = T.wpos[T.wst[x->num]]; ls = L->v[sl];
            if (ls->op != OP_SET || ls->has_dot || !tok_plain(&ls->tok[2])) continue;
            if (sl >= j || !between_ok(L, sl, i, x->num, -1)) continue;
            l->tok[1].s = "SEQ"; l->tok[2] = splat_tok(x, s0, sv);
            line_refresh(l); changed = 1;
        } else if ((!strcmp(l->tok[1].s, "SNE") || !strcmp(l->tok[1].s, "SEQ")) && lj->op == OP_SET && !lj->has_dot && tok_plain(&lj->tok[2])) {
            const char *cc = lj->tok[1].s;
            int seq = !strcmp(l->tok[1].s, "SEQ"), eqne = !strcmp(cc, "SEQ") || !strcmp(cc, "SNE");
            if (seq && !eqne) continue;
            if (!tok_isreg(&lj->tok[3]) || !tok_isreg(&lj->tok[4])) continue;
            if (!between_ok(L, j, i, lj->tok[3].num, lj->tok[4].num)) continue;
            l->tok[1].s = !seq ? cc : !strcmp(cc, "SEQ") ? "SNE" : "SEQ";
            l->tok[2] = splat_tok(&lj->tok[3], s0, sv); l->tok[3] = splat_tok(&lj->tok[4], s0, sv);
            line_refresh(l); changed = 1;
        }
    }
    sh_release(h, mk);
    return changed;
}

static int else_drop(Shc *h, Lines *L)
{
    ShMark mk = sh_mark(h);
    int i, k, t, n = L->n, kept = 0, changed = 0;
    unsigned char *del = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    for (i = 0; i + 2 < n; i++) {
        Line *l = L->v[i], *lb = L->v[i + 1], *le = L->v[i + 2];
        int refs = 0;
        if (l->label || l->op != OP_ELSE || l->ntok != 2 || !l->tok[1].s) continue;
        if (!lb->label || strcmp(lb->label, l->tok[1].s) || le->label || le->op != OP_ENDIF || le->ntok != 1) continue;
        for (k = 0; k < n; k++) {
            const Line *lk = L->v[k];
            if (lk->label) continue;
            for (t = 1; t < lk->ntok; t++) if (lk->tok[t].s && !strcmp(lk->tok[t].s, l->tok[1].s)) refs++;
        }
        if (refs != 1) continue;
        del[i] = del[i + 1] = 1; changed = 1;
    }
    for (i = 0; i < n; i++) if (!del[i]) L->v[kept++] = L->v[i];
    L->n = kept;
    sh_release(h, mk);
    return changed;
}

/* glslc._copy_back: `OP t, a, b` then `MOV d, t`, t written only there: `OP d, a, b` and t's reads read d (the conditions are there).
 * Decided on the lines as they are, applied after (Python builds the new lines apart) */
static int copy_back(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    ShMark mk = sh_mark(h);
    Tab T;
    Loop *loops;
    int nloops, i, k, n = L->n, nchosen = 0;
    int *remap = (int *)sh_zalloc(h, ((size_t)nv + 2) * sizeof(int));
    unsigned char *used = (unsigned char *)sh_zalloc(h, (size_t)nv + 2), *del = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    unsigned char *done = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    tab_build(h, L, nv, &T);
    nloops = loop_regions(h, L, &loops);
    for (i = 0; i < n; i++) {
        Line *l = L->v[i], *lj;
        int d, t, j, ok = 1, lr, dn;
        if (l->label || l->op != OP_MOV || l->has_dot || l->ntok != 3 || !tok_plain(&l->tok[1]) || !tok_plain(&l->tok[2])) continue;
        d = l->tok[1].num; t = l->tok[2].num;
        if (d == t || (sv[d] != 0) != (sv[t] != 0) || used[d] || used[t] || T.wcnt[t] != 1) continue;
        j = T.wpos[T.wst[t]];
        if (j >= i || done[j]) continue;
        lj = L->v[j];
        if (!fold_op(lj->op) || lj->has_dot || !tok_plain(&lj->tok[1])) continue;
        for (k = 2; k < lj->ntok; k++) {
            const Tok *x = &lj->tok[k];
            if (!tok_isreg(x) || x->num == t || used[x->num]) { ok = 0; break; }
        }
        if (!ok) continue;
        for (k = 2; k < lj->ntok; k++) if (lj->tok[k].num == d && lj->tok[k].swz[0]) { ok = 0; break; }
        if (!ok) continue;
        for (k = j + 1; k < i; k++) {
            const Line *lk = L->v[k];
            int s;
            if (ends_block(lk) || lk->dst == d) { ok = 0; break; }
            for (s = 0; s < lk->nsrc; s++) if (lk->src[s] == d) { ok = 0; break; }
            if (!ok) break;
        }
        if (!ok) continue;
        if (T.first[t] < j) continue;                           /* a read of t before its only write: round a loop */
        lr = T.rmax[t];
        dn = T.wst[d + 1] - T.wst[d];
        if (any_in_range(T.wpos + T.wst[d], dn, i + 1, lr + 1)) continue;
        for (k = 0; k < nloops && ok; k++) {
            int lo = loops[k].lo, hi = loops[k].hi, x, s, rt = 0;
            if (lo <= j) continue;
            for (x = lo; x <= hi && !rt; x++) { const Line *lx = L->v[x]; if (!lx->label) for (s = 0; s < lx->nsrc; s++) if (lx->src[s] == t) { rt = 1; break; } }
            if (rt && any_in_range(T.wpos + T.wst[d], dn, lo, hi + 1)) ok = 0;
        }
        if (!ok) continue;
        remap[t] = d; del[i] = 1; done[j] = 1; nchosen++;
        used[d] = 1; used[t] = 1;
        for (k = 2; k < lj->ntok; k++) used[lj->tok[k].num] = 1;
    }
    if (nchosen) apply_map(L, remap, del);
    sh_release(h, mk);
    return nchosen > 0;
}

static void shape_opt(Shc *h, Lines *L, int nv, const unsigned char *sv, const double *pool, int npool)
{
    for (;;) {
        int a = coalesce(h, L, nv, sv);
        int b = mask_fold(h, L, nv, sv);
        int c = setp_fold(h, L, nv, sv, pool, npool);
        int d = else_drop(h, L);
        int e = copy_back(h, L, nv, sv);
        if (!(a || b || c || d || e)) break;
    }
    dce(h, L, nv);
}

/* ---------------- ldv_masks: a fragment shader's LDV writes only the components something reads ---------------- */
static int read_pos(const Line *l)                              /* the positions of a source's swizzle the instruction reads */
{
    int pos = 15;
    if (l->op == OP_DP3) pos = 7;
    else if (l->op == OP_TEX) {
        const Tok *e = &l->tok[l->ntok - 1];
        if (e->s && e->s[0] == '.') {
            const char *p;
            pos = 0;
            for (p = e->s + 1; *p; p++) pos |= 1 << comp_of(*p);
        }
    }
    else if (l->op != OP_DP4 && (flags_of(l) & F_ALU)) pos = wmask(l);
    return pos;
}

static int read_mask(const Line *l, const Tok *t)
{
    int q[4], pos = read_pos(l), c, m = 0;
    swz4(t, q);
    for (c = 0; c < 4; c++) if (pos >> c & 1) m |= 1 << q[c];
    return m;
}

static void ldv_masks(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    static const char *const MASK[16] = {"", "x", "y", "xy", "z", "xz", "yz", "xyz", "w", "xw", "yw", "xyw", "zw", "xzw", "yzw", ""};
    ShMark mk = sh_mark(h);
    unsigned char *read = (unsigned char *)sh_zalloc(h, (size_t)nv + 2);
    int i, k;
    for (i = 0; i < L->n; i++) {
        const Line *l = L->v[i];
        int dp;
        if (l->label) continue;
        dp = dst_pos(l);
        for (k = 1; k < l->ntok; k++) if (k != dp && tok_isreg(&l->tok[k])) read[l->tok[k].num] |= (unsigned char)read_mask(l, &l->tok[k]);
    }
    for (i = 0; i < L->n; i++) {
        Line *l = L->v[i];
        if (l->label || l->op != OP_LDV || l->has_dot || l->dst < 0 || sv[l->dst]) continue;
        if (read[l->dst] == 0 || read[l->dst] == 15) continue;
        l->has_dot = 1; l->mask = MASK[read[l->dst]];
    }
    sh_release(h, mk);
}

/* g2sc_compile_vs: the varyings the linked fragment stage never reads */
static void drop_unread(Shc *h, Lines *L)
{
    int i, kept = 0;
    for (i = 0; i < L->n; i++) {
        Line *l = L->v[i];
        if (!l->label && l->op == OP_STV) {
            int s = atoi(l->tok[1].s);
            if (s < 32 && (h->unread >> s & 1)) continue;
        }
        L->v[kept++] = l;
    }
    L->n = kept;
}

/* Only the constants the final code loads take a parameter vector, renumbered in their order (glslc._compile_stage): a constant whose load the optimizer removed, or an unrolled loop's index nobody
 * reads, stayed in the pool — of the fragment core's 64 vectors, the uniforms included */
static void compact_pool(Shc *h, Lines *L, double *pool, int *npool)
{
    int n = *npool, i, t, k = 0;
    int *slot = (int *)sh_alloc(h, (size_t)(n + 1) * sizeof(int));
    for (i = 0; i < n; i++) slot[i] = -1;
    for (i = 0; i < L->n; i++) {
        Line *l = L->v[i];
        if (l->label) continue;
        for (t = 1; t < l->ntok; t++) {
            const char *s = l->tok[t].s;
            if (s && s[0] == '@' && s[1] >= '0' && s[1] <= '9') { long v = strtol(s + 1, NULL, 10); if (v >= 0 && v < n) slot[v] = 0; }
        }
    }
    for (i = 0; i < n; i++) if (slot[i] == 0) {
        if (k != i) memcpy(pool + (size_t)k * 4, pool + (size_t)i * 4, 4 * sizeof(double));
        slot[i] = k++;
    }
    if (k != n) for (i = 0; i < L->n; i++) {
        Line *l = L->v[i];
        if (l->label) continue;
        for (t = 1; t < l->ntok; t++) {
            const char *s = l->tok[t].s;
            if (s && s[0] == '@' && s[1] >= '0' && s[1] <= '9') {
                char *e;
                long v = strtol(s + 1, &e, 10);
                if (v >= 0 && v < n && slot[v] != v) l->tok[t].s = sh_fmt(h, "@%d%s", slot[v], e);
            }
        }
    }
    *npool = k;
}

/* ---------------- sfu_fuse (xops bit 0): `RCP t, x` / `RSQRT t, x` then `MUL d, y, t`, t and d scalars, t read only there: RCPM / RSQM d, x, y (glslc.sfu_fuse) ---------------- */
static void sfu_fuse(Shc *h, Lines *L, int nv, const unsigned char *sv)
{
    ShMark mk = sh_mark(h);
    Tab T;
    int i, n = L->n, kept = 0;
    unsigned char *used = (unsigned char *)sh_zalloc(h, (size_t)nv + 2), *del = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
    tab_build(h, L, nv, &T);
    for (i = 0; i < n; i++) {
        Line *l = L->v[i];
        int d, pass;
        if (l->label || l->op != OP_MUL || l->has_dot || l->ntok != 4 || !tok_plain(&l->tok[1])) continue;
        d = l->tok[1].num;
        if (!sv[d]) continue;
        for (pass = 0; pass < 2; pass++) {
            int k = pass ? 2 : 3, ok = pass ? 3 : 2, t, j, xr, yr, q, bad = 0;
            Tok xt, yt;
            const Line *lj;
            if (!tok_plain(&l->tok[k])) continue;
            t = l->tok[k].num; yt = l->tok[ok]; yr = tok_num(&yt);
            if (!sv[t] || t == d || used[t] || used[d] || T.wcnt[t] != 1 || T.rcnt[t] != 1 || yr == t) continue;
            j = T.wpos[T.wst[t]];
            if (j >= i) continue;
            lj = L->v[j];
            if ((lj->op != OP_RCP && lj->op != OP_RSQRT) || lj->has_dot || lj->ntok != 3 || !tok_plain(&lj->tok[1])) continue;
            xt = lj->tok[2]; xr = tok_num(&xt);
            if (xr < 0 || used[xr] || (yr >= 0 && used[yr])) continue;
            if ((xr == d && xt.swz[0]) || (yr == d && yt.swz[0])) continue;
            for (q = j + 1; q < i; q++) if (ends_block(L->v[q]) || L->v[q]->dst == xr) { bad = 1; break; }
            if (bad) continue;
            l->op = lj->op == OP_RCP ? OP_RCPM : OP_RSQM;
            l->tok[2] = xt; l->tok[3] = yt;
            line_refresh(l);
            del[j] = 1;
            used[d] = used[t] = used[xr] = 1;
            if (yr >= 0) used[yr] = 1;
            break;
        }
    }
    for (i = 0; i < n; i++) if (!del[i]) L->v[kept++] = L->v[i];
    L->n = kept;
    sh_release(h, mk);
}

/* ---------------- licm: loop-invariant code motion out of hardware loops (glslc.licm / _licm_loop: the conditions are there) ---------------- */
static int licm_op(int op)
{
    switch (op) {
    case OP_ADD: case OP_SUB: case OP_MUL: case OP_MAD: case OP_MIN: case OP_MAX: case OP_DP3: case OP_DP4: case OP_MOV: case OP_RCP: case OP_RSQRT:
    case OP_LOG2: case OP_EXP2: case OP_SIN: case OP_COS: case OP_FLR: case OP_CEIL: case OP_TRUNC: case OP_RNDE: case OP_SEL: case OP_SET:
    case OP_FTOI: case OP_ITOF: case OP_FTOU: case OP_UTOF: case OP_IADD: case OP_ISUB: case OP_IMUL: case OP_ISLT: case OP_ISGE: case OP_ISEQ: case OP_ISNE:
    case OP_LDP: case OP_MOVI: case OP_SQRT: case OP_RCPM: case OP_RSQM: case OP_CLAMP: return 1;
    default: return 0;
    }
}

static int src_has(const Line *l, int r) { int j; for (j = 0; j < l->nsrc; j++) if (l->src[j] == r) return 1; return 0; }

/* every write of r in the loop is in cand and before `before` */
static int writes_in(const Lines *L, int lo, int hi, int r, const unsigned char *cand, int before)
{
    int k;
    for (k = lo + 1; k < hi; k++) { const Line *l = L->v[k]; if (!l->label && l->dst == r && (!cand[k] || k > before)) return 0; }
    return 1;
}

static int written_in(const Lines *L, int lo, int hi, int r)
{
    int k;
    for (k = lo + 1; k < hi; k++) if (!L->v[k]->label && L->v[k]->dst == r) return 1;
    return 0;
}

/* the lines of LOOP lo .. ENDL hi that run once before it: cand[k] = 1; how many */
static int licm_loop(Shc *h, const Lines *L, int nv, int lo, int hi, unsigned char *cand)
{
    unsigned char *outside = (unsigned char *)sh_zalloc(h, (size_t)nv + 2);
    int *firstw = (int *)sh_alloc(h, ((size_t)nv + 2) * sizeof(int)), *firstr = (int *)sh_alloc(h, ((size_t)nv + 2) * sizeof(int));
    signed char *dep = (signed char *)sh_zalloc(h, (size_t)L->n + 1);
    int k, j, depth = 0, n = L->n, count = 0, bad;
    for (k = 0; k < nv + 2; k++) firstw[k] = firstr[k] = -1;
    for (k = lo + 1; k < hi; k++) {
        const Line *l = L->v[k];
        if (l->label) continue;
        dep[k] = (signed char)(depth > 0);
        if (l->dst >= 0 && firstw[l->dst] < 0) firstw[l->dst] = k;
        for (j = 0; j < l->nsrc; j++) if (firstr[l->src[j]] < 0) firstr[l->src[j]] = k;
        if (l->dst >= 0 && l->has_dot && firstr[l->dst] < 0) firstr[l->dst] = k;
        if (l->op == OP_IF || l->op == OP_LOOP) depth++;
        else if (l->op == OP_ENDIF || l->op == OP_ENDL) depth--;
    }
    for (k = 0; k < n; k++) {
        const Line *l = L->v[k];
        if ((k >= lo && k <= hi) || l->label) continue;
        for (j = 0; j < l->nsrc; j++) outside[l->src[j]] = 1;
        if (l->dst >= 0 && l->has_dot) outside[l->dst] = 1;
    }
    for (k = lo + 1; k < hi; k++) {
        const Line *l = L->v[k];
        int d = l->dst;
        if (l->label || dep[k] || d < 0 || !licm_op(l->op) || src_has(l, d) || outside[d]) continue;
        if ((firstr[d] < 0 ? hi : firstr[d]) < firstw[d]) continue;
        cand[k] = 1;
    }
    do {
        bad = 0;
        for (k = lo + 1; k < hi; k++) {
            const Line *l = L->v[k];
            int d, ok = 1;
            if (!cand[k]) continue;
            d = l->dst;
            if (!writes_in(L, lo, hi, d, cand, hi)) ok = 0;
            for (j = 0; j < l->nsrc && ok; j++) { int s = l->src[j]; if (s != d && written_in(L, lo, hi, s) && !writes_in(L, lo, hi, s, cand, k)) ok = 0; }
            if (!ok) { cand[k] = 0; bad = 1; }
        }
    } while (bad);
    for (k = lo + 1; k < hi; k++) count += cand[k];
    return count;
}

static void licm(Shc *h, Lines *L, int nv)
{
    for (;;) {
        ShMark mk = sh_mark(h);
        int *stk = (int *)sh_alloc(h, ((size_t)L->n + 1) * sizeof(int)), sp = 0, i, moved = 0;
        for (i = 0; i < L->n && !moved; i++) {
            Line *l = L->v[i];
            if (l->label) continue;
            if (l->op == OP_LOOP) stk[sp++] = i;
            else if (l->op == OP_ENDL && sp) {
                int lo = stk[--sp], k, m = 0, n = L->n;
                unsigned char *cand;
                Line **nv2;
                const Line *top = L->v[lo];
                if (top->ntok > 1 && top->tok[1].s && !strcmp(top->tok[1].s, "1")) continue;
                cand = (unsigned char *)sh_zalloc(h, (size_t)n + 1);
                if (!licm_loop(h, L, nv, lo, i, cand)) continue;
                nv2 = (Line **)sh_alloc(h, (size_t)n * sizeof(Line *));
                for (k = 0; k < lo; k++) nv2[m++] = L->v[k];
                for (k = lo + 1; k < i; k++) if (cand[k]) nv2[m++] = L->v[k];
                nv2[m++] = L->v[lo];
                for (k = lo + 1; k < n; k++) if (!cand[k]) nv2[m++] = L->v[k];
                memcpy(L->v, nv2, (size_t)n * sizeof(Line *));
                moved = 1;
            }
        }
        sh_release(h, mk);
        if (!moved) return;
    }
}

/* ---------------- the pipeline of _compile_stage ---------------- */
char *sh_optimize(Shc *h, Lines *code, int vcount, const unsigned char *svregs, int remat, long long const_base, int fs, double *pool, int *npool_io)
{
    int npool = *npool_io;
    if (h->unread) drop_unread(h, code);
    strip_dead_bra(code);
    drop_labels(h, code);
    dce(h, code, vcount);
    copy_prop(h, code, vcount);
    forward_opt(h, code, vcount, svregs);
    cse(h, code, vcount, remat, fs);
    dce(h, code, vcount);
    if (fs) ldv_masks(h, code, vcount, svregs);
    if (!remat) hoist_consts(h, code);
    if (!h->nohoist) licm(h, code, vcount);
    hoist_tex(h, code, vcount);
    shape_opt(h, code, vcount, svregs, pool, npool);
    mad_fuse(h, code);
    if (h->xops & 1) sfu_fuse(h, code, vcount, svregs);
    narrow_masks(h, code, vcount, svregs);
    compact_pool(h, code, pool, npool_io);
    return allocate_registers(h, code, vcount, svregs, const_base);
}
