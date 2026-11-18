/* shc_pp.c — glslc.preprocess(), line for line: GLSL ES 1.00 §3.4, which is C++'s preprocessor. Preprocessing tokens; macros
 * replaced token by token (a token carries the macros it came out of, so none expands inside itself); #if expressions of
 * 32-bit ints with C's precedence, short-circuited. h->strict (a program from an application) reports every error §3.4 names;
 * otherwise what cannot be read is let pass. */
#include <string.h>
#include "shc_internal.h"

enum { PT_ID = 1, PT_NUM, PT_OP, PT_OTHER };
typedef struct Hide { const char *name; const struct Hide *next; } Hide;      /* the frozenset of macros a token came out of */
typedef struct PT { int k; const char *v; int line, ws; const Hide *hide; } PT; /* kind, text, physical line, white space before it */
typedef struct PTV { PT *t; int n, cap; } PTV;
typedef struct PLine { int start; PTV t; } PLine;                               /* a logical line: its first physical line, its tokens */
typedef struct Define { int funclike; const char **params; int nparams; PTV body; } Define;
typedef struct PP { Shc *h; int strict, fs; Dict defines; long long lineoff, file; } PP;

/* ---------------- token lists ---------------- */
static void ptv_push(Shc *h, PTV *v, PT t)
{
    if (v->n == v->cap) {
        int cap = v->cap ? v->cap * 2 : 16;
        PT *n = (PT *)sh_alloc(h, (size_t)cap * sizeof(PT));
        if (v->n) memcpy(n, v->t, (size_t)v->n * sizeof(PT));
        v->t = n; v->cap = cap;
    }
    v->t[v->n++] = t;
}

/* v[i:j] = rep */
static void ptv_splice(Shc *h, PTV *v, int i, int j, const PTV *rep)
{
    int n = v->n - (j - i) + rep->n;
    PT *t = (PT *)sh_alloc(h, ((size_t)n + 1) * sizeof(PT));
    memcpy(t, v->t, (size_t)i * sizeof(PT));
    if (rep->n) memcpy(t + i, rep->t, (size_t)rep->n * sizeof(PT));
    memcpy(t + i + rep->n, v->t + j, (size_t)(v->n - j) * sizeof(PT));
    v->t = t; v->n = n; v->cap = n + 1;
}

static PTV ptv_copy(Shc *h, const PT *t, int n)
{
    PTV v;
    v.cap = n + 1; v.n = n;
    v.t = (PT *)sh_alloc(h, (size_t)v.cap * sizeof(PT));
    if (n) memcpy(v.t, t, (size_t)n * sizeof(PT));
    return v;
}

static int hidden(const Hide *s, const char *name)
{
    for (; s; s = s->next) if (!strcmp(s->name, name)) return 1;
    return 0;
}

static const Hide *hide_add(Shc *h, const Hide *s, const char *name)
{
    Hide *n;
    if (hidden(s, name)) return s;
    n = (Hide *)sh_alloc(h, sizeof(Hide));
    n->name = name; n->next = s;
    return n;
}

static const Hide *hide_union(Shc *h, const Hide *a, const Hide *b)
{
    for (; b; b = b->next) a = hide_add(h, a, b->name);
    return a;
}

/* _pp_text: the tokens, one space apart */
static char *pp_text(Shc *h, const PT *t, int n)
{
    SB b;
    int i;
    sb_init(&b, h, 64);
    for (i = 0; i < n; i++) { if (i) sb_putc(&b, ' '); sb_puts(&b, t[i].v); }
    return sb_str(&b);
}

static int is_ws(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\f' || c == '\v'; }
static int is_alpha(unsigned char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; }

/* _pp_lines: the source as logical lines; a comment is white space, so one that runs over lines joins them */
static PLine *pp_lines(PP *pp, const char *src, int *nl, int *nphys)
{
    static const char *const OPS3[] = { "<<=", ">>=", NULL };
    static const char *const OPS2[] = { "++","--","<<",">>","<=",">=","==","!=","&&","||","^^","+=","-=","*=","/=","%=","&=","^=","|=","##", NULL };
    Shc *h = pp->h;
    size_t i = 0, n = strlen(src);
    int line = 1, ws = 0, cap = 64, nlines = 0;
    PLine *ls = (PLine *)sh_alloc(h, (size_t)cap * sizeof(PLine));
    PTV cur; memset(&cur, 0, sizeof cur);
    ls[0].start = 1;
    while (i < n) {
        unsigned char c = (unsigned char)src[i];
        size_t j;
        PT t;
        if (c == '\n') {
            ls[nlines].t = cur; nlines++;
            if (nlines == cap) {
                PLine *nn = (PLine *)sh_alloc(h, (size_t)cap * 2 * sizeof(PLine));
                memcpy(nn, ls, (size_t)nlines * sizeof(PLine)); ls = nn; cap *= 2;
            }
            memset(&cur, 0, sizeof cur);
            line++; ls[nlines].start = line; ws = 0; i++;
            continue;
        }
        if (is_ws((char)c)) { ws = 1; i++; continue; }
        if (c == '/' && src[i + 1] == '/') {
            while (i < n && src[i] != '\n') i++;
            ws = 1;
            continue;
        }
        if (c == '/' && src[i + 1] == '*') {
            const char *q = strstr(src + i + 2, "*/"), *p;
            if (!q) sh_compile_error(h, "a comment does not end");
            for (p = src + i; p < q; p++) if (*p == '\n') line++;
            i = (size_t)(q - src) + 2; ws = 1;
            continue;
        }
        t.line = line; t.ws = ws; t.hide = NULL;
        if (is_alpha(c)) {
            for (j = i + 1; j < n && sh_is_word((unsigned char)src[j]); j++) { }
            t.k = PT_ID;
        }
        else if (sh_is_digit(c) || (c == '.' && sh_is_digit((unsigned char)src[i + 1]))) {      /* \.?\d(?:[eE][+-]|[\w.])* */
            j = i + (c == '.' ? 2 : 1);
            for (;;) {
                if (j + 1 < n && (src[j] == 'e' || src[j] == 'E') && (src[j + 1] == '+' || src[j + 1] == '-')) j += 2;
                else if (j < n && (sh_is_word((unsigned char)src[j]) || src[j] == '.')) j++;
                else break;
            }
            t.k = PT_NUM;
        }
        else {
            int k, len = 0;
            for (k = 0; OPS3[k]; k++) if (!strncmp(src + i, OPS3[k], 3)) { len = 3; break; }
            for (k = 0; !len && OPS2[k]; k++) if (!strncmp(src + i, OPS2[k], 2)) { len = 2; break; }
            if (!len && strchr("-+*/%<>=!&|^~?:.,;(){}[]#", (char)c)) len = 1;
            if (len) t.k = PT_OP;
            else {                                                  /* any other character, a UTF-8 sequence whole */
                len = 1;
                if (c >= 0xC0) while (i + (size_t)len < n && ((unsigned char)src[i + len] & 0xC0) == 0x80) len++;
                t.k = PT_OTHER;
            }
            j = i + (size_t)len;
        }
        t.v = sh_strndup(h, src + i, j - i);
        ptv_push(h, &cur, t);
        ws = 0; i = j;
    }
    ls[nlines].t = cur; nlines++;
    *nl = nlines; *nphys = line;
    return ls;
}

/* _pp_int: an integer literal of a #if (0 = none); *v its value, 32 bits */
static int pp_int(const char *s, int32_t *v)
{
    uint32_t u = 0;
    size_t n = strlen(s), i;
    if (n > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
        for (i = 2; i < n; i++) {
            char c = s[i];
            int d = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 : c >= 'A' && c <= 'F' ? c - 'A' + 10 : -1;
            if (d < 0) return 0;
            u = u * 16 + (uint32_t)d;
        }
    }
    else if (s[0] == '0') {
        for (i = 1; i < n; i++) { if (s[i] < '0' || s[i] > '7') return 0; u = u * 8 + (uint32_t)(s[i] - '0'); }
    }
    else if (s[0] >= '1' && s[0] <= '9') {
        for (i = 0; i < n; i++) { if (!sh_is_digit((unsigned char)s[i])) return 0; u = u * 10 + (uint32_t)(s[i] - '0'); }
    }
    else return 0;
    *v = (int32_t)u;
    return 1;
}

static int32_t w32(long long x) { return (int32_t)(uint32_t)(unsigned long long)x; }

static Define *def_of(PP *pp, const char *name) { DictEnt *e = dict_find(&pp->defines, name); return e ? (Define *)e->p : NULL; }

/* call_end: toks[i] names a function-like macro: 1 and the indices of its '(' and the ')' that closes it; 0 when no '(' follows;
 * -1 when the call does not end in toks */
static int call_end(const PTV *t, int i, int *lp, int *rp)
{
    int j = i + 1, depth = 0, k;
    if (j >= t->n) return -1;
    if (strcmp(t->t[j].v, "(")) return 0;
    for (k = j; k < t->n; k++) {
        if (!strcmp(t->t[k].v, "(")) depth++;
        else if (!strcmp(t->t[k].v, ")") && --depth == 0) { *lp = j; *rp = k; return 1; }
    }
    return -1;
}

static PT num_tok(Shc *h, const char *v, int line, int ws) { PT t; t.k = PT_NUM; t.v = sh_strdup(h, v); t.line = line; t.ws = ws; t.hide = NULL; return t; }

/* expand: macro replacement of a token list; ifexpr: `defined X` and `defined ( X )` become 1 or 0, also where a replacement
 * brings them */
static PTV expand(PP *pp, const PT *in, int nin, int ifexpr)
{
    Shc *h = pp->h;
    PTV toks = ptv_copy(h, in, nin), out;
    int i = 0;
    memset(&out, 0, sizeof out);
    SH_ENTER(h, 1);
    while (i < toks.n) {
        PT t = toks.t[i];
        Define *d;
        if (ifexpr && t.k == PT_ID && !strcmp(t.v, "defined")) {
            int j = i + 1, paren = j < toks.n && !strcmp(toks.t[j].v, "(");
            const char *nm;
            if (paren) j++;
            if (j >= toks.n || toks.t[j].k != PT_ID) sh_compile_error(h, "defined needs a macro name");
            nm = toks.t[j].v; j++;
            if (paren) {
                if (j >= toks.n || strcmp(toks.t[j].v, ")")) sh_compile_error(h, "defined( needs a )");
                j++;
            }
            ptv_push(h, &out, num_tok(h, def_of(pp, nm) || !strcmp(nm, "__LINE__") || !strcmp(nm, "__FILE__") ? "1" : "0", t.line, t.ws));
            i = j;
            continue;
        }
        if (t.k != PT_ID || hidden(t.hide, t.v)) { ptv_push(h, &out, t); i++; continue; }
        if (!strcmp(t.v, "__LINE__") || !strcmp(t.v, "__FILE__")) {
            SB b;
            sb_init(&b, h, 24);
            sb_putll(&b, t.v[2] == 'L' ? (long long)t.line + pp->lineoff : pp->file);
            ptv_push(h, &out, num_tok(h, sb_str(&b), t.line, t.ws));
            i++;
            continue;
        }
        if ((d = def_of(pp, t.v)) == NULL) { ptv_push(h, &out, t); i++; continue; }
        if (!d->funclike) {
            PTV rep; int n;
            const Hide *hd = hide_add(h, t.hide, t.v);
            memset(&rep, 0, sizeof rep);
            for (n = 0; n < d->body.n; n++) {
                PT b = d->body.t[n];
                b.line = t.line; b.ws = n == 0 ? t.ws : b.ws; b.hide = hd;
                ptv_push(h, &rep, b);
            }
            ptv_splice(h, &toks, i, i + 1, &rep);
            continue;
        }
        {
            int lp, rp, ce = call_end(&toks, i, &lp, &rp), depth = 0, na = 1, cap = 4, k, n, line;
            PTV *args, rep;
            const Hide *hd;
            if (ce != 1) { ptv_push(h, &out, t); i++; continue; }
            args = (PTV *)sh_zalloc(h, (size_t)cap * sizeof(PTV));
            for (k = lp + 1; k < rp; k++) {
                const char *v = toks.t[k].v;
                if (!strcmp(v, "(")) depth++;
                else if (!strcmp(v, ")")) depth--;
                if (!strcmp(v, ",") && depth == 0) {
                    if (na == cap) {
                        PTV *nn = (PTV *)sh_zalloc(h, (size_t)cap * 2 * sizeof(PTV));
                        memcpy(nn, args, (size_t)na * sizeof(PTV)); args = nn; cap *= 2;
                    }
                    na++;
                    continue;
                }
                ptv_push(h, &args[na - 1], toks.t[k]);
            }
            if (na == 1 && args[0].n == 0 && d->nparams == 0) na = 0;
            if (na != d->nparams) {
                if (pp->strict) sh_compile_error(h, "macro %s takes %d arguments, not %d", t.v, d->nparams, na);
                if (d->nparams > cap) {
                    PTV *nn = (PTV *)sh_zalloc(h, (size_t)d->nparams * sizeof(PTV));
                    memcpy(nn, args, (size_t)(na < d->nparams ? na : d->nparams) * sizeof(PTV)); args = nn;
                }
                for (k = na; k < d->nparams; k++) memset(&args[k], 0, sizeof(PTV));
                na = d->nparams;
            }
            for (k = 0; k < na; k++) args[k] = expand(pp, args[k].t, args[k].n, 0);
            hd = hide_add(h, t.hide, t.v);
            line = toks.t[rp].line;
            memset(&rep, 0, sizeof rep);
            for (n = 0; n < d->body.n; n++) {
                PT b = d->body.t[n];
                int ws = n == 0 ? t.ws : b.ws, p = -1, m;
                if (b.k == PT_ID) for (m = 0; m < d->nparams; m++) if (!strcmp(d->params[m], b.v)) { p = m; break; }
                if (p >= 0) {
                    for (m = 0; m < args[p].n; m++) {
                        PT x = args[p].t[m];
                        if (m == 0) x.ws = ws;
                        x.hide = hide_union(h, x.hide, hd);
                        ptv_push(h, &rep, x);
                    }
                }
                else { b.line = line; b.ws = ws; b.hide = hd; ptv_push(h, &rep, b); }
            }
            ptv_splice(h, &toks, i, rp + 1, &rep);
        }
    }
    SH_LEAVE(h, 1);
    return out;
}

/* ---------------- #if ---------------- */
typedef struct EV { PP *pp; PTV x; int pos; } EV;
static const char *const LEVELS[10][4] = {
    { "||", NULL }, { "&&", NULL }, { "|", NULL }, { "^", NULL }, { "&", NULL }, { "==", "!=", NULL },
    { "<", ">", "<=", ">=" }, { "<<", ">>", NULL }, { "+", "-", NULL }, { "*", "/", "%", NULL }
};

static const char *peek_v(EV *e) { return e->pos < e->x.n ? e->x.t[e->pos].v : NULL; }

static int at_level(EV *e, int lvl)
{
    const char *v = peek_v(e);
    int k;
    if (!v) return 0;
    for (k = 0; k < 4 && LEVELS[lvl][k]; k++) if (!strcmp(LEVELS[lvl][k], v)) return 1;
    return 0;
}

static int32_t binary(EV *e, int lvl, int ev);

static int32_t unary(EV *e, int ev)
{
    Shc *h = e->pp->h;
    PT t;
    int32_t a = 0;
    SH_ENTER(h, 1);
    if (e->pos >= e->x.n) sh_compile_error(h, "#if: the expression ends early");
    t = e->x.t[e->pos++];
    if (!strcmp(t.v, "+") || !strcmp(t.v, "-") || !strcmp(t.v, "~") || !strcmp(t.v, "!")) {
        a = unary(e, ev);
        a = t.v[0] == '+' ? a : t.v[0] == '-' ? w32(-(long long)a) : t.v[0] == '~' ? w32(~(long long)a) : !a;
    }
    else if (!strcmp(t.v, "(")) {
        const char *v;
        a = binary(e, 0, ev);
        v = peek_v(e);
        if (!v || strcmp(v, ")")) sh_compile_error(h, "#if: ) expected");
        e->pos++;
    }
    else if (t.k == PT_NUM) { if (!pp_int(t.v, &a)) sh_compile_error(h, "#if: %s is no integer", t.v); }
    else if (t.k == PT_ID) { if (ev) sh_compile_error(h, "#if: '%s' is not defined", t.v); a = 0; }
    else sh_compile_error(h, "#if: unexpected '%s'", t.v);
    SH_LEAVE(h, 1);
    return a;
}

static int32_t binary(EV *e, int lvl, int ev)
{
    Shc *h = e->pp->h;
    int32_t a;
    SH_ENTER(h, 1);
    if (lvl == 10) a = unary(e, ev);
    else {
        a = binary(e, lvl + 1, ev);
        while (at_level(e, lvl)) {
            const char *op = peek_v(e);
            int32_t b;
            e->pos++;
            if (!strcmp(op, "||")) { b = binary(e, lvl + 1, ev && !a); a = a || b; continue; }
            if (!strcmp(op, "&&")) { b = binary(e, lvl + 1, ev && a); a = a && b; continue; }
            b = binary(e, lvl + 1, ev);
            if (!strcmp(op, "/") || !strcmp(op, "%")) {
                long long q, la = a, lb = b;
                if (b == 0) {
                    if (ev) sh_compile_error(h, "#if: division by zero");
                    a = 0;
                    continue;
                }
                q = (la < 0 ? -la : la) / (lb < 0 ? -lb : lb);
                if ((la < 0) != (lb < 0)) q = -q;
                a = op[0] == '/' ? w32(q) : w32(la - lb * q);
                continue;
            }
            if (!strcmp(op, "<<")) a = w32((long long)((uint32_t)a << (b & 31)));
            else if (!strcmp(op, ">>")) a = a >> (b & 31);
            else if (!strcmp(op, "==")) a = a == b;
            else if (!strcmp(op, "!=")) a = a != b;
            else if (!strcmp(op, "<")) a = a < b;
            else if (!strcmp(op, ">")) a = a > b;
            else if (!strcmp(op, "<=")) a = a <= b;
            else if (!strcmp(op, ">=")) a = a >= b;
            else if (!strcmp(op, "|")) a = a | b;
            else if (!strcmp(op, "^")) a = a ^ b;
            else if (!strcmp(op, "&")) a = a & b;
            else if (!strcmp(op, "+")) a = w32((long long)a + b);
            else if (!strcmp(op, "-")) a = w32((long long)a - b);
            else a = w32((long long)a * b);
        }
    }
    SH_LEAVE(h, 1);
    return a;
}

/* eval_if: a #if / #elif expression, 1 or 0 */
static int eval_if(PP *pp, const PT *t, int n)
{
    EV e;
    int32_t r;
    e.pp = pp; e.pos = 0;
    e.x = expand(pp, t, n, 1);
    if (!e.x.n) sh_compile_error(pp->h, "#if needs an expression");
    r = binary(&e, 0, 1);
    if (e.pos != e.x.n) sh_compile_error(pp->h, "#if: unexpected '%s' after the expression", e.x.t[e.pos].v);
    return r != 0;
}

/* if_value: a CompileError of a library's own program reads as 0; any other error goes on */
static int if_value(PP *pp, const PT *t, int n)
{
    Shc *h = pp->h;
    jmp_buf save;
    volatile int depth = h->depth;
    int r;
    char err[sizeof h->err];
    if (pp->strict) return eval_if(pp, t, n);
    memcpy(save, h->jb, sizeof(jmp_buf));
    memcpy(err, h->err, sizeof err);
    if (setjmp(h->jb)) {
        memcpy(h->jb, save, sizeof(jmp_buf));
        if (!h->compile_error) longjmp(h->jb, 1);
        memcpy(h->err, err, sizeof err);
        h->compile_error = 0;
        h->depth = depth;
        return 0;
    }
    r = eval_if(pp, t, n);
    memcpy(h->jb, save, sizeof(jmp_buf));
    return r;
}

/* ---------------- directives ---------------- */
static int is_fixed(const char *nm) { return !strcmp(nm, "__LINE__") || !strcmp(nm, "__FILE__") || !strcmp(nm, "__VERSION__") || !strcmp(nm, "GL_ES"); }

/* name_of: the one name a directive takes */
static const char *name_of(PP *pp, const PT *t, int n, const char *what)
{
    if (pp->strict && (n != 1 || t[0].k != PT_ID)) sh_compile_error(pp->h, "#%s takes one macro name", what);
    return n ? t[0].v : "";
}

static void no_more(PP *pp, const PT *t, int n, const char *what)
{
    if (pp->strict && n) sh_compile_error(pp->h, "#%s: unexpected '%s'", what, t[0].v);
}

static int same_body(const PTV *a, const PTV *b)
{
    int i;
    if (a->n != b->n) return 0;
    for (i = 0; i < a->n; i++)
        if (a->t[i].k != b->t[i].k || strcmp(a->t[i].v, b->t[i].v) || (i && !a->t[i].ws != !b->t[i].ws)) return 0;
    return 1;
}

static int same_params(const Define *d, int funclike, const char **params, int np)
{
    int i;
    if (d->funclike != funclike || d->nparams != np) return 0;
    for (i = 0; i < np; i++) if (strcmp(d->params[i], params[i])) return 0;
    return 1;
}

static void define(PP *pp, const PT *t, int n)
{
    Shc *h = pp->h;
    const char *nm, **params = NULL;
    int funclike = 0, np = 0, cap = 4, i = 1;
    Define *d, *was;
    if (!n || t[0].k != PT_ID) sh_compile_error(h, "#define needs a macro name");
    nm = t[0].v;
    if (i < n && !strcmp(t[i].v, "(") && !t[i].ws) {               /* a function-like macro: '(' right after the name */
        funclike = 1; i++;
        params = (const char **)sh_alloc(h, (size_t)cap * sizeof(char *));
        if (i < n && !strcmp(t[i].v, ")")) i++;
        else {
            for (;;) {
                int k;
                if (i >= n || t[i].k != PT_ID) {
                    if (pp->strict) sh_compile_error(h, "#define %s: a parameter must be a name", nm);
                    while (i < n && strcmp(t[i].v, ")")) i++;
                    i++;
                    break;
                }
                if (pp->strict) for (k = 0; k < np; k++) if (!strcmp(params[k], t[i].v)) sh_compile_error(h, "#define %s: the parameter %s twice", nm, t[i].v);
                if (np == cap) {
                    const char **nn = (const char **)sh_alloc(h, (size_t)cap * 2 * sizeof(char *));
                    memcpy(nn, params, (size_t)np * sizeof(char *)); params = nn; cap *= 2;
                }
                params[np++] = t[i].v; i++;
                if (i < n && !strcmp(t[i].v, ",")) { i++; continue; }
                if (i < n && !strcmp(t[i].v, ")")) { i++; break; }
                if (pp->strict) sh_compile_error(h, "#define %s: ) expected", nm);
                break;
            }
        }
    }
    if (i > n) i = n;
    d = (Define *)sh_alloc(h, sizeof(Define));
    d->funclike = funclike; d->params = params; d->nparams = np;
    d->body = ptv_copy(h, t + i, n - i);
    if (pp->strict && (!strncmp(nm, "GL_", 3) || is_fixed(nm))) sh_compile_error(h, "macro name '%s' is reserved", nm);
    was = def_of(pp, nm);
    if (pp->strict && was && (!same_params(was, funclike, params, np) || !same_body(&was->body, &d->body)))
        sh_compile_error(h, "macro %s is defined again otherwise", nm);
    dict_put(h, &pp->defines, nm)->p = d;
}

static void pp_const(PP *pp, const char *name, const char *v)
{
    Define *d = (Define *)sh_zalloc(pp->h, sizeof(Define));
    ptv_push(pp->h, &d->body, num_tok(pp->h, v, 0, 0));
    dict_put(pp->h, &pp->defines, name)->p = d;
}

static int ext_bit(const char *nm)
{
    if (!strcmp(nm, "GL_OES_standard_derivatives")) return SH_EXT_DERIV;
    if (!strcmp(nm, "GL_EXT_shader_texture_lod")) return SH_EXT_LOD;
    if (!strcmp(nm, "GL_EXT_frag_depth")) return SH_EXT_FRAGDEPTH;
    if (!strcmp(nm, "GL_OES_texture_3D")) return SH_EXT_3D;
    return 0;
}

static int word_tok(const PT *t)
{
    const char *p;
    for (p = t->v; *p; p++) if (!sh_is_word((unsigned char)*p)) return 0;
    return t->v[0] != 0;
}

/* #extension name : behavior */
static void do_extension(PP *pp, const PT *t, int n, int code, int *exts)
{
    Shc *h = pp->h;
    const char *nm, *beh;
    if (n != 3 || !word_tok(&t[0]) || strcmp(t[1].v, ":") || !word_tok(&t[2]) ||
        (strcmp(t[2].v, "require") && strcmp(t[2].v, "enable") && strcmp(t[2].v, "warn") && strcmp(t[2].v, "disable")))
        sh_compile_error(h, "bad #extension: %s", pp_text(h, t, n));
    if (pp->strict && code) sh_compile_error(h, "#extension comes before the code");
    nm = t[0].v; beh = t[2].v;
    if (!strcmp(nm, "all")) {
        if (!strcmp(beh, "require") || !strcmp(beh, "enable")) sh_compile_error(h, "#extension all : %s", beh);
        if (!strcmp(beh, "disable")) *exts = 0;
        else *exts = pp->fs ? SH_EXT_DERIV | SH_EXT_LOD | SH_EXT_FRAGDEPTH | SH_EXT_3D : SH_EXT_3D;
    }
    else if (!(ext_bit(nm) == SH_EXT_3D || (pp->fs && ext_bit(nm)))) {
        if (!strcmp(beh, "require")) sh_compile_error(h, "extension %s is not supported", nm);
    }
    else if (!strcmp(beh, "disable")) *exts &= ~ext_bit(nm);
    else *exts |= ext_bit(nm);
}

/* #line line [source]: the next line is line */
static void do_line(PP *pp, const PT *t, int n, int next)
{
    Shc *h = pp->h;
    PTV x = expand(pp, t, n, 0);
    long long v[2];
    int i;
    if (x.n < 1 || x.n > 2) sh_compile_error(h, "bad #line: %s", pp_text(h, t, n));
    for (i = 0; i < x.n; i++) {
        const char *p = x.t[i].v;
        if (x.t[i].k != PT_NUM || !*p) sh_compile_error(h, "bad #line: %s", pp_text(h, t, n));
        v[i] = 0;
        for (; *p; p++) {
            if (!sh_is_digit((unsigned char)*p)) sh_compile_error(h, "bad #line: %s", pp_text(h, t, n));
            v[i] = v[i] * 10 + (*p - '0');
            if (v[i] > 0x7FFFFFFF) v[i] = 0x7FFFFFFF;
        }
    }
    pp->lineoff = v[0] - next;
    if (x.n == 2) pp->file = v[1];
}

typedef struct Ifs { int pa, taken, any, el; } Ifs;

char *sh_preprocess(Shc *h, const char *src, int fs, int *exts)
{
    PP pp;
    PLine *ls;
    int nl, nphys, li = 0, sp = 0, spcap = 16, begun = 0, code = 0, i;
    char **out;
    Ifs *stack;
    SB o;
    memset(&pp, 0, sizeof pp);
    pp.h = h; pp.strict = h->strict; pp.fs = fs;
    ls = pp_lines(&pp, src, &nl, &nphys);
    pp_const(&pp, "GL_ES", "1");
    pp_const(&pp, "__VERSION__", "100");
    if (fs) {
        pp_const(&pp, "GL_FRAGMENT_PRECISION_HIGH", "1");
        pp_const(&pp, "GL_OES_standard_derivatives", "1");
        pp_const(&pp, "GL_EXT_shader_texture_lod", "1");
        pp_const(&pp, "GL_EXT_frag_depth", "1");
    }
    pp_const(&pp, "GL_OES_texture_3D", "1");                  /* glslc.EXT_ANY */
    out = (char **)sh_zalloc(h, (size_t)nphys * sizeof(char *));
    stack = (Ifs *)sh_alloc(h, (size_t)spcap * sizeof(Ifs));
    while (li < nl) {
        int start = ls[li].start, active = 1, k;
        PTV toks = ls[li].t;
        li++;
        for (k = 0; k < sp; k++) if (!stack[k].taken) active = 0;
        if (toks.n && !strcmp(toks.t[0].v, "#")) {
            const PT *d = toks.t + 1, *rest;
            int nd = toks.n - 1, nr;
            const char *key = nd && d[0].k == PT_ID ? d[0].v : "";
            rest = d + 1; nr = nd ? nd - 1 : 0;
            if (nd && !*key && pp.strict && active) sh_compile_error(h, "bad directive #%s", d[0].v);
#define KEY(w) (!strcmp(key, w))
            if (KEY("if") || KEY("ifdef") || KEY("ifndef")) {
                Ifs f;
                int t;
                if (!active) t = 0;
                else if (KEY("if")) t = if_value(&pp, rest, nr);
                else {
                    const char *nm = name_of(&pp, rest, nr, key);
                    t = (def_of(&pp, nm) || !strcmp(nm, "__LINE__") || !strcmp(nm, "__FILE__")) == KEY("ifdef");
                }
                f.pa = active; f.taken = t; f.any = t; f.el = 0;
                if (sp == spcap) {
                    Ifs *ns = (Ifs *)sh_alloc(h, (size_t)spcap * 2 * sizeof(Ifs));
                    memcpy(ns, stack, (size_t)sp * sizeof(Ifs)); stack = ns; spcap *= 2;
                }
                stack[sp++] = f;
            }
            else if (KEY("elif")) {
                Ifs *f;
                int t;
                if (!sp) sh_compile_error(h, "#elif without #if");
                f = &stack[sp - 1];
                if (f->el && pp.strict) sh_compile_error(h, "#elif after #else");
                t = f->pa && !f->any && if_value(&pp, rest, nr);
                f->taken = t; f->any = f->any || t;
            }
            else if (KEY("else")) {
                Ifs *f;
                if (!sp) sh_compile_error(h, "#else without #if");
                f = &stack[sp - 1];
                if (f->el && pp.strict) sh_compile_error(h, "#else after #else");
                no_more(&pp, rest, nr, "else");
                f->taken = f->pa && !f->any; f->any = 1; f->el = 1;
            }
            else if (KEY("endif")) {
                if (!sp) sh_compile_error(h, "#endif without #if");
                no_more(&pp, rest, nr, "endif");
                sp--;
            }
            else if (active) {
                if (KEY("version")) {
                    char *v = pp_text(h, rest, nr);
                    if (begun || strcmp(v, "100")) {
                        if (!strcmp(v, "100")) sh_compile_error(h, "#version must come first");
                        sh_compile_error(h, "#version %s is not supported: GLSL ES 1.00 only", v);
                    }
                }
                else if (KEY("define")) define(&pp, rest, nr);
                else if (KEY("undef")) {
                    const char *nm = name_of(&pp, rest, nr, "undef");
                    if (pp.strict && (!strncmp(nm, "GL_", 3) || is_fixed(nm))) sh_compile_error(h, "macro name '%s' is reserved", nm);
                    dict_del(&pp.defines, nm);
                }
                else if (KEY("error")) sh_compile_error(h, "#error %s", pp_text(h, rest, nr));
                else if (KEY("extension")) do_extension(&pp, rest, nr, code, exts);
                else if (KEY("line")) do_line(&pp, rest, nr, li < nl ? ls[li].start : nphys + 1);
                else if (KEY("pragma") || !nd) { }
                else if (pp.strict) sh_compile_error(h, "unknown directive #%s", key);
            }
#undef KEY
            if (*key) begun = 1;
            continue;
        }
        if (!active) continue;
        if (toks.n) { begun = 1; code = 1; }
        while (li < nl && !(ls[li].t.n && !strcmp(ls[li].t.t[0].v, "#"))) {     /* a call of a function-like macro may go on over lines */
            int open = 0, lp, rp, m;
            PTV nx;
            for (m = 0; m < toks.n && !open; m++) {
                Define *df;
                if (toks.t[m].k == PT_ID && (df = def_of(&pp, toks.t[m].v)) != NULL && df->funclike && call_end(&toks, m, &lp, &rp) == -1) open = 1;
            }
            if (!open) break;
            nx = ptv_copy(h, ls[li].t.t, ls[li].t.n);
            if (nx.n) nx.t[0].ws = 1;
            {
                PTV j = ptv_copy(h, toks.t, toks.n);
                for (m = 0; m < nx.n; m++) ptv_push(h, &j, nx.t[m]);
                toks = j;
            }
            li++;
        }
        {
            PTV x = expand(&pp, toks.t, toks.n, 0);
            out[start - 1] = pp_text(h, x.t, x.n);
        }
    }
    if (sp) sh_compile_error(h, "#if without #endif");
    sb_init(&o, h, strlen(src) + 64);
    for (i = 0; i < nphys; i++) {
        if (i) sb_putc(&o, '\n');
        if (out[i]) sb_puts(&o, out[i]);
    }
    return sb_str(&o);
}
