/* shc_pp.c — glslc.preprocess(), line for line. The quirks are glslc's and kept on purpose: the comment passes run in
 * this order, a directive is recognized by its first whitespace-separated word (so `#if(X)` is no directive),
 * parameters substitute one after the other, unknown directives are ignored. */
#include <string.h>
#include "shc_internal.h"

typedef struct Define { char **params; int nparams; int funclike; const char *body; } Define;
typedef struct Hide { const char *name; struct Hide *next; } Hide;       /* the frozenset `hide` */
typedef struct PP { Shc *h; Dict defines; } PP;

static int hidden(const Hide *hd, const char *name)
{
    for (; hd; hd = hd->next) if (!strcmp(hd->name, name)) return 1;
    return 0;
}

static char *strip_copy(Shc *h, const char *s, size_t n)
{
    const char *b = s, *e = s + n;
    sh_strip(&b, &e);
    return sh_strndup(h, b, (size_t)(e - b));
}

/* _pp_split_args: text[i] == '('. Returns the index after the closing paren (len + 1 when there is none). */
static size_t split_args(PP *pp, const char *text, size_t n, size_t i, char ***pargs, int *pn)
{
    Shc *h = pp->h;
    SB cur; int depth = 1, na = 0, cap = 8;
    char **args = (char **)sh_alloc(h, (size_t)cap * sizeof(char *));
    const char *b, *e;
    i++;
    sb_init(&cur, h, 64);
    while (i < n && depth > 0) {
        char c = text[i];
        if (c == '(') { depth++; sb_putc(&cur, c); }
        else if (c == ')') {
            depth--;
            if (depth == 0) break;
            sb_putc(&cur, c);
        }
        else if (c == ',' && depth == 1) {
            if (na == cap) {
                char **na2 = (char **)sh_alloc(h, (size_t)cap * 2 * sizeof(char *));
                memcpy(na2, args, (size_t)na * sizeof(char *)); args = na2; cap *= 2;
            }
            args[na++] = strip_copy(h, cur.p, cur.n);
            cur.n = 0;
        }
        else sb_putc(&cur, c);
        i++;
    }
    b = cur.p; e = cur.p + cur.n;
    sh_strip(&b, &e);
    if (b != e || na) {
        if (na == cap) {
            char **na2 = (char **)sh_alloc(h, (size_t)cap * 2 * sizeof(char *));
            memcpy(na2, args, (size_t)na * sizeof(char *)); args = na2; cap *= 2;
        }
        args[na++] = sh_strndup(h, b, (size_t)(e - b));
    }
    *pargs = args; *pn = na;
    return i + 1;
}

static int word_at(const char *s, size_t n, size_t pos) { return pos < n && sh_is_word((unsigned char)s[pos]); }
static int boundary(const char *s, size_t n, size_t pos) { return (pos > 0 && word_at(s, n, pos - 1)) != word_at(s, n, pos); }

/* re.sub(r'\b' + re.escape(p) + r'\b', a, rep), with p possibly empty (an empty match at every word boundary) */
static char *sub_param(Shc *h, const char *rep, const char *p, const char *a)
{
    size_t n = strlen(rep), lp = strlen(p), pos = 0, m;
    SB o;
    sb_init(&o, h, n + 16);
    while (pos <= n) {
        for (m = pos; m + lp <= n; m++)
            if (!memcmp(rep + m, p, lp) && boundary(rep, n, m) && boundary(rep, n, m + lp)) break;
        if (m + lp > n) break;
        sb_putn(&o, rep + pos, m - pos);
        sb_puts(&o, a);
        if (lp == 0) {
            if (m < n) sb_putc(&o, rep[m]);
            pos = m + 1;
        }
        else pos = m + lp;
    }
    if (pos < n) sb_putn(&o, rep + pos, n - pos);
    return sb_str(&o);
}

static void expand(PP *pp, const char *text, size_t n, Hide *hide, SB *out)
{
    Shc *h = pp->h;
    size_t i = 0;
    SH_ENTER(h, 1);
    while (i < n) {
        unsigned char ch = (unsigned char)text[i];
        size_t j;
        ShMark mk;
        char *name;
        DictEnt *de;
        if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_')) { sb_putc(out, (char)ch); i++; continue; }
        for (j = i + 1; j < n && sh_is_word((unsigned char)text[j]); j++) { }
        mk = sh_mark(h);
        name = sh_strndup(h, text + i, j - i);
        de = dict_find(&pp->defines, name);
        if (de && !hidden(hide, name)) {
            Define *d = (Define *)de->p;
            Hide hn; hn.name = name; hn.next = hide;
            if (!d->funclike) {
                expand(pp, d->body, strlen(d->body), &hn, out);
                i = j;
            } else {
                size_t k = j;
                while (k < n && (text[k] == ' ' || text[k] == '\t' || text[k] == '\n')) k++;
                if (k < n && text[k] == '(') {
                    char **args, *rep; int na, a;
                    k = split_args(pp, text, n, k, &args, &na);
                    for (a = 0; a < na; a++) {
                        SB t; sb_init(&t, h, 64);
                        expand(pp, args[a], strlen(args[a]), hide, &t);
                        args[a] = sb_str(&t);
                    }
                    rep = (char *)d->body;
                    for (a = 0; a < na && a < d->nparams; a++) rep = sub_param(h, rep, d->params[a], args[a]);
                    expand(pp, rep, strlen(rep), &hn, out);
                    i = k;
                } else {
                    sb_putn(out, name, j - i);
                    i = j;
                }
            }
        } else {
            sh_release(h, mk);                  /* before the write: `out` may grow */
            sb_putn(out, text + i, j - i);
            i = j;
        }
    }
    SH_LEAVE(h, 1);
}

/* text ends inside a call of a function-like macro: the arguments go on over the next line, as in C++ */
static int open_call(PP *pp, const char *text, size_t n)
{
    size_t i = 0;
    while (i < n) {
        unsigned char ch = (unsigned char)text[i];
        size_t j, k;
        ShMark mk;
        DictEnt *de;
        int depth = 0;
        if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || ch == '_')) { i++; continue; }
        for (j = i + 1; j < n && sh_is_word((unsigned char)text[j]); j++) { }
        mk = sh_mark(pp->h);
        de = dict_find(&pp->defines, sh_strndup(pp->h, text + i, j - i));
        sh_release(pp->h, mk);
        i = j;
        if (!de || !((Define *)de->p)->funclike) continue;
        for (k = j; k < n && (text[k] == ' ' || text[k] == '\t' || text[k] == '\n'); k++) { }
        if (k == n) return 1;                   /* the '(' may be on the next line */
        if (text[k] != '(') continue;
        for (; k < n; k++) {
            if (text[k] == '(') depth++;
            else if (text[k] == ')' && --depth == 0) break;
        }
        if (k == n) return 1;
        i = k + 1;
    }
    return 0;
}

/* ---------------- eval_if ---------------- */
static const char *skip_ws(const char *s, const char *e)
{
    int k;
    while ((k = sh_ws_len(s, e)) != 0) s += k;
    return s;
}

static int is_defined(PP *pp, const char *b, const char *e)
{
    ShMark mk = sh_mark(pp->h);
    int r = dict_find(&pp->defines, sh_strndup(pp->h, b, (size_t)(e - b))) != NULL;
    sh_release(pp->h, mk);
    return r;
}

/* re.sub(r'defined\s*\(\s*(\w+)\s*\)', ...), then re.sub(r'defined\s+(\w+)', ...) */
static char *sub_defined(PP *pp, const char *s, int paren)
{
    const char *p = s, *e = s + strlen(s);
    SB o;
    sb_init(&o, pp->h, (size_t)(e - s) + 16);
    while (p < e) {
        if ((size_t)(e - p) >= 7 && !memcmp(p, "defined", 7)) {
            const char *q = p + 7, *wb, *we;
            if (paren) {
                q = skip_ws(q, e);
                if (q < e && *q == '(') {
                    q = skip_ws(q + 1, e);
                    wb = q;
                    while (q < e && sh_is_word((unsigned char)*q)) q++;
                    we = q;
                    q = skip_ws(q, e);
                    if (we > wb && q < e && *q == ')') {
                        sb_putc(&o, is_defined(pp, wb, we) ? '1' : '0');
                        p = q + 1;
                        continue;
                    }
                }
            } else {
                const char *r = skip_ws(q, e);
                if (r > q) {
                    wb = r;
                    while (r < e && sh_is_word((unsigned char)*r)) r++;
                    if (r > wb) {
                        sb_putc(&o, is_defined(pp, wb, r) ? '1' : '0');
                        p = r;
                        continue;
                    }
                }
            }
        }
        sb_putc(&o, *p++);
    }
    return sb_str(&o);
}

/* re.sub(r'\b[A-Za-z_]\w*\b', '0', expr) */
static char *sub_idents(Shc *h, const char *s)
{
    size_t n = strlen(s), p = 0;
    SB o;
    sb_init(&o, h, n + 16);
    while (p < n) {
        unsigned char c = (unsigned char)s[p];
        if (((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_') && !(p > 0 && sh_is_word((unsigned char)s[p - 1]))) {
            while (p < n && sh_is_word((unsigned char)s[p])) p++;
            sb_putc(&o, '0');
            continue;
        }
        sb_putc(&o, s[p++]);
    }
    return sb_str(&o);
}

static char *replace_all(Shc *h, const char *s, const char *from, const char *to)
{
    size_t n = strlen(s), lf = strlen(from), p = 0;
    SB o;
    sb_init(&o, h, n + 16);
    while (p < n) {
        if (n - p >= lf && !memcmp(s + p, from, lf)) { sb_puts(&o, to); p += lf; }
        else sb_putc(&o, s[p++]);
    }
    return sb_str(&o);
}

/* re.sub(r'\bnot=', '!=', expr) */
static char *sub_not_eq(Shc *h, const char *s)
{
    size_t n = strlen(s), p = 0;
    SB o;
    sb_init(&o, h, n + 16);
    while (p < n) {
        if (n - p >= 4 && !memcmp(s + p, "not=", 4) && !(p > 0 && sh_is_word((unsigned char)s[p - 1]))) { sb_puts(&o, "!="); p += 4; }
        else sb_putc(&o, s[p++]);
    }
    return sb_str(&o);
}

static int eval_if(PP *pp, const char *b, const char *e)
{
    Shc *h = pp->h;
    char *x = sh_strndup(h, b, (size_t)(e - b));
    SB t;
    x = sub_defined(pp, x, 1);
    x = sub_defined(pp, x, 0);
    sb_init(&t, h, strlen(x) + 16);
    expand(pp, x, strlen(x), NULL, &t);
    x = sub_idents(h, sb_str(&t));
    x = replace_all(h, x, "||", " or ");
    x = replace_all(h, x, "&&", " and ");
    x = replace_all(h, x, "!", "not ");
    x = sub_not_eq(h, x);
    return sh_pyeval_truth(h, x);
}

/* ---------------- directives ---------------- */
static void do_define(PP *pp, const char *rest, const char *rend)
{
    /* re.match(r'(\w+)(\(([^)]*)\))?\s*(.*)', rest, re.S) */
    Shc *h = pp->h;
    const char *p = rest, *gb = NULL, *ge = NULL, *bb, *be;
    Define *d;
    char *nm;
    int funclike = 0;
    while (p < rend && sh_is_word((unsigned char)*p)) p++;
    if (p == rest) sh_py_error(h, "AttributeError: bad #define");        /* m is None */
    nm = sh_strndup(h, rest, (size_t)(p - rest));
    if (p < rend && *p == '(') {
        const char *q = p + 1;
        while (q < rend && *q != ')') q++;
        if (q < rend) { gb = p + 1; ge = q; p = q + 1; funclike = 1; }
    }
    p = skip_ws(p, rend);
    bb = p; be = rend;
    sh_strip(&bb, &be);
    d = (Define *)sh_alloc(h, sizeof(Define));
    d->funclike = funclike; d->nparams = 0; d->params = NULL;
    d->body = sh_strndup(h, bb, (size_t)(be - bb));
    if (funclike) {
        int n = 1;
        const char *q;
        for (q = gb; q < ge; q++) if (*q == ',') n++;
        d->params = (char **)sh_alloc(h, (size_t)n * sizeof(char *));
        n = 0;
        for (q = gb; ; ) {
            const char *c = q;
            while (c < ge && *c != ',') c++;
            d->params[n++] = strip_copy(h, q, (size_t)(c - q));
            if (c >= ge) break;
            q = c + 1;
        }
        if (n == 1 && !d->params[0][0]) n = 0;      /* params == [''] -> [] */
        d->nparams = n;
    }
    if (h->strict && !strncmp(nm, "GL_", 3)) sh_compile_error(h, "macro name '%s' is reserved", nm);
    dict_put(h, &pp->defines, nm)->p = d;
}

typedef struct Ifs { int pa, taken, any; } Ifs;
int snprintf(char *s, size_t n, const char *fmt, ...);

static void pp_const(PP *pp, const char *name, const char *body)
{
    Define *d = (Define *)sh_alloc(pp->h, sizeof(Define));
    d->params = NULL; d->nparams = 0; d->funclike = 0; d->body = body;
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

/* #extension name : behavior */
static void do_extension(Shc *h, const char *r, const char *re, int fs, int *exts)
{
    const char *a = r, *ae, *b, *be;
    char *nm, *beh;
    while (r < re && sh_is_word((unsigned char)*r)) r++;
    ae = r;
    r = skip_ws(r, re);
    b = NULL;
    if (ae > a && r < re && *r == ':') {
        b = r = skip_ws(r + 1, re);
        while (r < re && sh_is_word((unsigned char)*r)) r++;
        be = r;
        r = skip_ws(r, re);
        if (be == b || r != re) b = NULL;
    }
    nm = b ? sh_strndup(h, a, (size_t)(ae - a)) : NULL;
    beh = b ? sh_strndup(h, b, (size_t)(be - b)) : NULL;
    if (!b || (strcmp(beh, "require") && strcmp(beh, "enable") && strcmp(beh, "warn") && strcmp(beh, "disable")))
        sh_compile_error(h, "bad #extension: %s", sh_strndup(h, a, (size_t)(re - a)));
    if (!strcmp(nm, "all")) {
        if (!strcmp(beh, "require") || !strcmp(beh, "enable")) sh_compile_error(h, "#extension all : %s", beh);
        if (!strcmp(beh, "disable")) *exts = 0;
        else *exts = fs ? SH_EXT_DERIV | SH_EXT_LOD | SH_EXT_FRAGDEPTH | SH_EXT_3D : SH_EXT_3D;
    }
    else if (!(ext_bit(nm) == SH_EXT_3D || (fs && ext_bit(nm)))) {
        if (!strcmp(beh, "require")) sh_compile_error(h, "extension %s is not supported", nm);
    }
    else if (!strcmp(beh, "disable")) *exts &= ~ext_bit(nm);
    else *exts |= ext_bit(nm);
}

char *sh_preprocess(Shc *h, const char *src, int fs, int *exts)
{
    PP pp;
    SB s1, s2, out;
    const char *p, *e, *line;
    size_t n = strlen(src), i;
    Ifs *stack; int sp = 0, spcap = 16, lineno = 0, lineoff = 0, begun = 0;
    pp.h = h; memset(&pp.defines, 0, sizeof pp.defines);
    /* re.sub(r'/\*.*?\*\/', newlines, src, flags=re.S): the block comments first, so one can start inside a // comment */
    sb_init(&s1, h, n + 16);
    for (i = 0; i < n; ) {
        if (src[i] == '/' && src[i + 1] == '*') {
            const char *q = strstr(src + i + 2, "*/");
            if (q) {
                const char *c;
                for (c = src + i; c < q; c++) if (*c == '\n') sb_putc(&s1, '\n');
                i = (size_t)(q - src) + 2;
                continue;
            }
        }
        sb_putc(&s1, src[i++]);
    }
    /* re.sub(r'//[^\n]*', '', src) */
    sb_init(&s2, h, s1.n + 16);
    sb_str(&s1);
    for (i = 0; i < s1.n; ) {
        if (s1.p[i] == '/' && s1.p[i + 1] == '/') {
            while (i < s1.n && s1.p[i] != '\n') i++;
            continue;
        }
        sb_putc(&s2, s1.p[i++]);
    }
    pp_const(&pp, "GL_ES", "1");
    pp_const(&pp, "__VERSION__", "100");
    pp_const(&pp, "__FILE__", "0");
    if (fs) {
        pp_const(&pp, "GL_FRAGMENT_PRECISION_HIGH", "1");
        pp_const(&pp, "GL_OES_standard_derivatives", "1");
        pp_const(&pp, "GL_EXT_shader_texture_lod", "1");
        pp_const(&pp, "GL_EXT_frag_depth", "1");
    }
    pp_const(&pp, "GL_OES_texture_3D", "1");                  /* glslc.EXT_ANY */
    stack = (Ifs *)sh_alloc(h, (size_t)spcap * sizeof(Ifs));
    sb_init(&out, h, s2.n + 16);
    p = s2.p; e = s2.p + s2.n;
    for (line = p; ; ) {
        const char *le = line, *st, *ste;
        int first = line == p, active, k;
        char *ln = (char *)sh_alloc(h, 16);
        lineno++;
        snprintf(ln, 16, "%d", lineno + lineoff);
        pp_const(&pp, "__LINE__", ln);
        while (le < e && *le != '\n') le++;
        if (!first) sb_putc(&out, '\n');
        st = line; ste = le;
        sh_strip(&st, &ste);
        active = 1;
        for (k = 0; k < sp; k++) if (!stack[k].taken) active = 0;
        if (st < ste && *st == '#') {
            const char *d = st + 1, *de = ste, *ke, *rest, *rb, *re_;
            sh_strip(&d, &de);
            ke = d;
            while (ke < de && !sh_ws_len(ke, de)) ke++;
            rest = skip_ws(ke, de);
            rb = rest; re_ = de;
            sh_strip(&rb, &re_);
#define KEY(w) ((size_t)(ke - d) == sizeof(w) - 1 && !memcmp(d, w, sizeof(w) - 1))
            if (KEY("version") && active) {
                char *v = sh_strndup(h, rb, (size_t)(re_ - rb));
                if (begun || strcmp(v, "100")) {
                    if (!strcmp(v, "100")) sh_compile_error(h, "#version must come first");
                    sh_compile_error(h, "#version %s is not supported: GLSL ES 1.00 only", v);
                }
            }
            if (ke > d) begun = 1;
            if (KEY("define") && active) do_define(&pp, rest, de);
            else if (KEY("undef") && active) { dict_del(&pp.defines, sh_strndup(h, rb, (size_t)(re_ - rb))); }
            else if (KEY("error") && active) sh_compile_error(h, "#error %s", sh_strndup(h, rb, (size_t)(re_ - rb)));
            else if (KEY("ifdef") || KEY("ifndef") || KEY("if")) {
                Ifs f; f.pa = active;
                if (KEY("if")) f.taken = active && eval_if(&pp, rest, de);
                else {
                    int def = dict_find(&pp.defines, sh_strndup(h, rb, (size_t)(re_ - rb))) != NULL;
                    f.taken = active && (KEY("ifdef") ? def : !def);
                }
                f.any = f.taken;
                if (sp == spcap) {
                    Ifs *ns = (Ifs *)sh_alloc(h, (size_t)spcap * 2 * sizeof(Ifs));
                    memcpy(ns, stack, (size_t)sp * sizeof(Ifs)); stack = ns; spcap *= 2;
                }
                stack[sp++] = f;
            }
            else if (KEY("elif")) {
                int t;
                if (!sp) sh_py_error(h, "IndexError: #elif without #if");
                t = stack[sp - 1].pa && !stack[sp - 1].any && eval_if(&pp, rest, de);
                stack[sp - 1].taken = t;
                stack[sp - 1].any = stack[sp - 1].any || t;
            }
            else if (KEY("else")) {
                if (!sp) sh_py_error(h, "IndexError: #else without #if");
                stack[sp - 1].taken = stack[sp - 1].pa && !stack[sp - 1].any;
                stack[sp - 1].any = 1;
            }
            else if (KEY("endif")) {
                if (!sp) sh_py_error(h, "IndexError: #endif without #if");
                sp--;
            }
            else if (KEY("extension") && active) do_extension(h, rb, re_, fs, exts);
            else if (KEY("line") && active) {
                SB x;
                const char *q, *qe, *w;
                long long v = 0;
                sb_init(&x, h, 32);
                expand(&pp, rb, (size_t)(re_ - rb), NULL, &x);
                q = sb_str(&x); qe = q + x.n;
                w = q;
                while (w < qe && sh_is_digit((unsigned char)*w)) { v = v * 10 + (*w - '0'); if (v > 0x7FFFFFFF) v = 0x7FFFFFFF; w++; }
                if (w == q) sh_compile_error(h, "bad #line: %s", sh_strndup(h, rb, (size_t)(re_ - rb)));
                w = skip_ws(w, qe);
                while (w < qe && sh_is_digit((unsigned char)*w)) w++;
                w = skip_ws(w, qe);
                if (w != qe) sh_compile_error(h, "bad #line: %s", sh_strndup(h, rb, (size_t)(re_ - rb)));
                lineoff = (int)v - lineno;              /* ES 1.00: the next line counts as line + 1 */
            }
#undef KEY
        }
        else if (active) {
            SB j;
            const char *t = line;
            size_t tn = (size_t)(le - line);
            int joined = 0, m;
            if (st < ste) begun = 1;
            while (le < e && open_call(&pp, t, tn)) {
                const char *nb = le + 1, *ne = nb, *sb_, *se;
                while (ne < e && *ne != '\n') ne++;
                sb_ = nb; se = ne;
                sh_strip(&sb_, &se);
                if (sb_ < se && *sb_ == '#') break;
                if (!joined) { sb_init(&j, h, tn + (size_t)(ne - nb) + 16); sb_putn(&j, line, tn); }
                sb_putc(&j, ' ');
                sb_putn(&j, nb, (size_t)(ne - nb));
                t = j.p; tn = j.n;
                le = ne; lineno++; joined++;
            }
            expand(&pp, t, tn, NULL, &out);
            for (m = 0; m < joined; m++) sb_putc(&out, '\n');   /* the lines joined: blank, the line numbers stay */
        }
        if (le >= e) break;
        line = le + 1;
    }
    if (sp) sh_compile_error(h, "#if without #endif");
    return sb_str(&out);
}
