/* shc_parse.c — glslc's lex() and class Parser, function for function. The AST mirrors the Python tuples:
 *   N_NUM num  N_VAR s  N_SWIZZLE a,s  N_INDEX a,b  N_CALL s,list  N_POSTOP a,s  N_UNARY s,a  N_BIN s,a,b
 *   N_TERNARY a,b,c  N_ASSIGN s,a,b  N_SEQ list
 *   S_BLOCK list  S_IF a(cond),b(then),c(else)  S_FOR a(init),b(cond),c(post),d(body)  S_WHILE a(cond),b(body)
 *   S_DOWHILE a(cond),b(body)  S_RETURN a  S_LDECL isc,ty,s,a(init),b(arr)  S_MULTI list  S_EXPR a */
#include <string.h>
#include "shc_internal.h"

/* ================================================================ lexer */
static const char *const KEYWORDS[] = {
    "attribute","uniform","varying","const","void","float","int","bool",
    "vec2","vec3","vec4","bvec2","bvec3","bvec4","ivec2","ivec3","ivec4",
    "mat2","mat3","mat4","sampler2D","samplerCube",
    "if","else","for","return","discard","true","false","in","out","inout","precision",
    "highp","mediump","lowp","struct","while","do","continue","break","invariant", NULL
};

static int is_keyword(const char *s)
{
    int i;
    for (i = 0; KEYWORDS[i]; i++) if (!strcmp(KEYWORDS[i], s)) return 1;
    return 0;
}

/* §3.6: a program that names one is in error */
static const char *const RESERVED[] = {
    "asm","class","union","enum","typedef","template","this","packed","goto","switch","default","inline","noinline",
    "volatile","public","static","extern","external","interface","flat","long","short","double","half","fixed",
    "unsigned","superp","input","output","hvec2","hvec3","hvec4","dvec2","dvec3","dvec4","fvec2","fvec3","fvec4",
    "sampler1D","sampler3D","sampler1DShadow","sampler2DShadow","sampler2DRect","sampler3DRect",
    "sampler2DRectShadow","sizeof","cast","namespace","using", NULL
};

static const char *const OPS2[] = { "<<",">>","<=",">=","==","!=","&&","||","++","--","+=","-=","*=","/=","^^", NULL };

static size_t digits(const char *s, size_t i, size_t n) { while (i < n && sh_is_digit((unsigned char)s[i])) i++; return i; }
static int is_hex(unsigned char c) { return sh_is_digit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }

static size_t exp_part(const char *s, size_t i, size_t n)   /* ([eE][+-]?\d+)? */
{
    size_t j = i;
    if (j < n && (s[j] == 'e' || s[j] == 'E')) {
        j++;
        if (j < n && (s[j] == '+' || s[j] == '-')) j++;
        if (j < n && sh_is_digit((unsigned char)s[j])) return digits(s, j, n);
    }
    return i;
}

/* the float alternatives, leftmost first: \d+\.\d*(exp)? | \.\d+(exp)? | \d+[eE][+-]?\d+ | \d+\. ; 0 = no match */
static size_t match_float(const char *s, size_t i, size_t n)
{
    size_t d = digits(s, i, n);
    if (d > i && d < n && s[d] == '.') return exp_part(s, digits(s, d + 1, n), n);
    if (d == i && i < n && s[i] == '.' && i + 1 < n && sh_is_digit((unsigned char)s[i + 1])) return exp_part(s, digits(s, i + 1, n), n);
    if (d > i) { size_t e = exp_part(s, d, n); if (e > d) return e; }
    return 0;
}

static Tk *lex(Shc *h, const char *src, int exts, int *ntok)
{
    size_t i = 0, n = strlen(src);
    int nt = 0, cap = 256;
    Tk *t = (Tk *)sh_alloc(h, (size_t)cap * sizeof(Tk));
    while (i < n) {
        size_t j;
        int lit, base = 10;
        int ws = sh_ws_len(src + i, src + n);
        unsigned char c = (unsigned char)src[i];
        if (ws) {
            i += (size_t)ws;
            while ((ws = sh_ws_len(src + i, src + n)) != 0) i += (size_t)ws;
            continue;
        }
        if (c == '/' && src[i + 1] == '/') {
            while (i < n && src[i] != '\n') i++;
            continue;
        }
        if (c == '/' && src[i + 1] == '*') {
            const char *q = strstr(src + i + 2, "*/");
            if (q) { i = (size_t)(q - src) + 2; continue; }
        }
        if (nt + 1 >= cap) {
            Tk *nn = (Tk *)sh_alloc(h, (size_t)cap * 2 * sizeof(Tk));
            memcpy(nn, t, (size_t)nt * sizeof(Tk)); t = nn; cap *= 2;
        }
        j = match_float(src, i, n);
        lit = j ? LIT_FLOAT : LIT_INT;
        if (j == 0 && c == '0' && (src[i + 1] == 'x' || src[i + 1] == 'X') && is_hex((unsigned char)src[i + 2])) {
            base = 16;
            j = i + 2;
            while (j < n && is_hex((unsigned char)src[j])) j++;
        }
        else if (j == 0 && sh_is_digit(c)) {
            j = digits(src, i, n);
            base = j - i > 1 && c == '0' ? 8 : 10;
        }
        if (j) {
            double v;
            uint64_t u;
            char *key = (char *)sh_alloc(h, 18);
            int k, ovf = 0;
            if (lit == LIT_FLOAT || base == 10) v = sh_str2d(h, src + i, j - i);
            else if (base == 16) v = bi_to_double(h, bi_from_digits(h, src + i + 2, j - i - 2, 16), &ovf);
            else {
                size_t q;
                for (q = i; q < j; q++) if (src[q] > '7') sh_compile_error(h, "bad octal literal %s", sh_strndup(h, src + i, j - i));
                v = bi_to_double(h, bi_from_digits(h, src + i, j - i, 8), &ovf);
            }
            memcpy(&u, &v, 8);
            key[0] = 1;                 /* the value as a canonical key: equal floats = equal text; no name looks like it */
            for (k = 0; k < 16; k++) key[1 + k] = "0123456789abcdef"[(u >> (60 - 4 * k)) & 15];
            key[17] = 0;
            t[nt].k = TK_NUM; t[nt].v = key; t[nt].num = v; t[nt].lit = lit; nt++;
            i = j;
            continue;
        }
        if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_') {
            char *w;
            j = i + 1;
            while (j < n && sh_is_word((unsigned char)src[j])) j++;
            w = sh_strndup(h, src + i, j - i);
            t[nt].k = is_keyword(w) || ((exts & SH_EXT_3D) && !strcmp(w, "sampler3D")) ? TK_KW : TK_ID; t[nt].v = w; t[nt].num = 0; nt++;
            i = j;
            continue;
        }
        {
            int k, len = 0;
            for (k = 0; OPS2[k]; k++) if (!memcmp(src + i, OPS2[k], 2)) { len = 2; break; }
            if (!len && c && strchr("-+*/%<>=!&|^~?:.,;(){}[]", c)) len = 1;
            if (!len) {
                size_t e = n - i < 16 ? n : i + 16;
                sh_compile_error(h, "lex error at '%s'", sh_strndup(h, src + i, e - i));
            }
            t[nt].k = TK_OP; t[nt].v = sh_strndup(h, src + i, (size_t)len); t[nt].num = 0; nt++;
            i += (size_t)len;
        }
    }
    t[nt].k = TK_EOF; t[nt].v = ""; t[nt].num = 0; nt++;
    *ntok = nt;
    return t;
}

/* ================================================================ parser */
typedef struct DV { Decl *v; int n, cap; } DV;
typedef struct P {
    Shc *h; Tk *t; int n, i;
    Dict **sscope; int nscope, capscope;     /* struct names by scope: name -> the name its definition goes by (an inner struct may hide an outer one) */
    int nstruct;                             /* definitions so far: a hidden name and an anonymous struct get a unique one */
    DV hoisted;                              /* struct definitions met inside functions: every struct is global to the code generator */
} P;

/* the name a struct type goes by, NULL if v names no struct */
static const char *sname_of(P *p, const char *v)
{
    int i;
    for (i = p->nscope - 1; i >= 0; i--) {
        DictEnt *e = dict_find(p->sscope[i], v);
        if (e) return (const char *)e->p;
    }
    return NULL;
}

static void sscope_push(P *p)
{
    if (p->nscope == p->capscope) {
        int cap = p->capscope ? p->capscope * 2 : 8;
        Dict **n = (Dict **)sh_alloc(p->h, (size_t)cap * sizeof(Dict *));
        if (p->nscope) memcpy(n, p->sscope, (size_t)p->nscope * sizeof(Dict *));
        p->sscope = n; p->capscope = cap;
    }
    p->sscope[p->nscope++] = (Dict *)sh_zalloc(p->h, sizeof(Dict));
}

static int vec_or_mat(const char *v)
{
    static const char *const T[] = { "vec2","vec3","vec4","bvec2","bvec3","bvec4","ivec2","ivec3","ivec4","mat2","mat3","mat4",NULL };
    int i;
    for (i = 0; T[i]; i++) if (!strcmp(T[i], v)) return 1;
    return 0;
}

static Tk *peek(P *p)
{
    if (p->i >= p->n) sh_py_error(p->h, "IndexError: read past the end of the token list");
    return &p->t[p->i];
}

static Tk *next(P *p) { Tk *k = peek(p); p->i++; return k; }
static int at(P *p, const char *v) { return !strcmp(peek(p)->v, v); }

static const char *show(const Tk *k) { return k->k == TK_NUM ? "<number>" : k->v; }

static void eat(P *p, const char *v)
{
    Tk *k = peek(p);
    if (strcmp(k->v, v)) sh_compile_error(p->h, "expected '%s' got '%s'", v, show(k));
    p->i++;
}

static int accept_(P *p, const char *v)
{
    if (at(p, v)) { p->i++; return 1; }
    return 0;
}

/* the name a declaration gives: an identifier (§3.7); a program from an application that names a keyword or a number fails */
static const char *ident(P *p)
{
    Tk *k = next(p);
    if (p->h->strict && k->k != TK_ID) {
        if (k->k == TK_NUM) sh_compile_error(p->h, "expected a name, got <number>");
        sh_compile_error(p->h, "expected a name, got '%s'", k->v);
    }
    return k->v;
}

static Node *node(P *p, NodeKind k)
{
    Node *n = (Node *)sh_zalloc(p->h, sizeof(Node));
    n->kind = k;
    return n;
}

typedef struct NV { Node **v; int n, cap; } NV;
static void nv_push(P *p, NV *l, Node *e)
{
    if (l->n == l->cap) {
        int cap = l->cap ? l->cap * 2 : 4;
        Node **nn = (Node **)sh_alloc(p->h, (size_t)cap * sizeof(Node *));
        if (l->n) memcpy(nn, l->v, (size_t)l->n * sizeof(Node *));
        l->v = nn; l->cap = cap;
    }
    l->v[l->n++] = e;
}

static const char *type_name(P *p)
{
    const char *v = next(p)->v, *s = sname_of(p, v);
    return s ? s : v;
}

static const char *skip_precision(P *p)
{
    const char *q = NULL;
    for (;;) {
        const char *v = peek(p)->v;
        if (strcmp(v, "highp") && strcmp(v, "mediump") && strcmp(v, "lowp")) return q;
        q = next(p)->v;
    }
}

static Node *expr(P *p);
static Node *assign(P *p);
static Node *stmt(P *p);

static Node **arglist(P *p, int *pn)
{
    NV l; memset(&l, 0, sizeof l);
    SH_ENTER(p->h, 1);
    eat(p, "(");
    if (!at(p, ")")) {
        for (;;) {
            nv_push(p, &l, assign(p));
            if (!accept_(p, ",")) break;
        }
    }
    eat(p, ")");
    SH_LEAVE(p->h, 1);
    *pn = l.n;
    return l.v;
}

static Node *primary(P *p)
{
    Tk *k = peek(p);
    const char *v = k->v;
    Node *n;
    SH_ENTER(p->h, 1);
    if (k->k == TK_NUM) { next(p); n = node(p, N_NUM); n->num = k->num; n->lit = k->lit; }
    else if (!strcmp(v, "true")) { next(p); n = node(p, N_NUM); n->num = 1.0; n->lit = LIT_BOOL; }
    else if (!strcmp(v, "false")) { next(p); n = node(p, N_NUM); n->num = 0.0; n->lit = LIT_BOOL; }
    else if (vec_or_mat(v)) {
        next(p);
        n = node(p, N_CALL); n->s = v;
        n->list = arglist(p, &n->nlist);
    }
    else if (k->k == TK_ID || !strcmp(v, "float") || !strcmp(v, "int") || !strcmp(v, "bool")) { next(p); n = node(p, N_VAR); n->s = v; }
    else if (!strcmp(v, "(")) {
        next(p);
        n = expr(p);
        eat(p, ")");
    }
    else { sh_compile_error(p->h, "unexpected token '%s' in expression", show(k)); }
    SH_LEAVE(p->h, 1);
    return n;
}

static Node *postfix(P *p)
{
    Node *e;
    SH_ENTER(p->h, 1);
    e = primary(p);
    for (;;) {
        const char *v = peek(p)->v;
        if (!strcmp(v, ".")) {
            Node *n = node(p, N_SWIZZLE);
            next(p);
            n->s = next(p)->v; n->a = e; e = n;
        }
        else if (!strcmp(v, "[")) {
            Node *n = node(p, N_INDEX);
            next(p);
            n->a = e; n->b = expr(p);
            eat(p, "]");
            if (e->kind == N_VAR && !strcmp(e->s, "gl_FragData") && n->b->kind == N_NUM && n->b->num == 0.0) { e->s = "gl_FragColor"; e->fragdata = 1; }   /* MAX_DRAW_BUFFERS is 1 */
            else e = n;
        }
        else if (!strcmp(v, "(") && e->kind == N_VAR) {
            Node *n = node(p, N_CALL);
            const char *sn = sname_of(p, e->s);
            n->s = sn ? sn : e->s;
            n->list = arglist(p, &n->nlist);
            e = n;
        }
        else if (!strcmp(v, "++") || !strcmp(v, "--")) {
            Node *n = node(p, N_POSTOP);
            next(p);
            n->a = e; n->s = v; e = n;
        }
        else break;
    }
    SH_LEAVE(p->h, 1);
    return e;
}

static Node *unary(P *p)
{
    const char *v = peek(p)->v;
    Node *n;
    SH_ENTER(p->h, 1);
    if (!strcmp(v, "-") || !strcmp(v, "!") || !strcmp(v, "+")) {
        next(p);
        n = node(p, N_UNARY); n->s = v;
        n->a = unary(p);
    }
    else if (!strcmp(v, "++") || !strcmp(v, "--")) {        /* ++x is x += 1, and its value */
        next(p);
        n = node(p, N_ASSIGN); n->s = v[0] == '+' ? "+=" : "-=";
        n->a = unary(p);
        n->b = node(p, N_NUM); n->b->num = 1.0; n->b->lit = LIT_ONE;
    }
    else n = postfix(p);
    SH_LEAVE(p->h, 1);
    return n;
}

static int prec_has(int lvl, const char *v)
{
    static const char *const PREC[7][5] = {
        { "||", NULL }, { "^^", NULL }, { "&&", NULL }, { "==", "!=", NULL }, { "<", ">", "<=", ">=", NULL }, { "+", "-", NULL }, { "*", "/", "%", NULL }
    };
    int i;
    for (i = 0; PREC[lvl][i]; i++) if (!strcmp(PREC[lvl][i], v)) return 1;
    return 0;
}

static Node *binary(P *p, int lvl)
{
    Node *left;
    SH_ENTER(p->h, 1);
    if (lvl >= 7) left = unary(p);
    else {
        left = binary(p, lvl + 1);
        while (prec_has(lvl, peek(p)->v)) {
            Node *n = node(p, N_BIN);
            n->s = next(p)->v;
            n->a = left; n->b = binary(p, lvl + 1);
            left = n;
        }
    }
    SH_LEAVE(p->h, 1);
    return left;
}

static Node *ternary(P *p)
{
    Node *c;
    SH_ENTER(p->h, 1);
    c = binary(p, 0);
    if (accept_(p, "?")) {
        Node *n = node(p, N_TERNARY);
        n->a = c;
        n->b = assign(p);
        eat(p, ":");
        n->c = assign(p);
        c = n;
    }
    SH_LEAVE(p->h, 1);
    return c;
}

static Node *assign(P *p)
{
    Node *left;
    const char *v;
    SH_ENTER(p->h, 1);
    left = ternary(p);
    v = peek(p)->v;
    if (!strcmp(v, "=") || !strcmp(v, "+=") || !strcmp(v, "-=") || !strcmp(v, "*=") || !strcmp(v, "/=")) {
        Node *n = node(p, N_ASSIGN);
        next(p);
        n->s = v; n->a = left;
        n->b = assign(p);
        left = n;
    }
    SH_LEAVE(p->h, 1);
    return left;
}

static Node *expr(P *p)
{
    Node *e;
    SH_ENTER(p->h, 1);
    e = assign(p);
    if (at(p, ",")) {
        NV l; Node *n;
        memset(&l, 0, sizeof l);
        nv_push(p, &l, e);
        while (accept_(p, ",")) nv_push(p, &l, assign(p));
        n = node(p, N_SEQ);
        n->list = l.v; n->nlist = l.n;
        e = n;
    }
    SH_LEAVE(p->h, 1);
    return e;
}

static Node *block(P *p)
{
    NV l; Node *n;
    memset(&l, 0, sizeof l);
    SH_ENTER(p->h, 1);
    eat(p, "{");
    sscope_push(p);
    while (!at(p, "}")) nv_push(p, &l, stmt(p));
    eat(p, "}");
    p->nscope--;
    SH_LEAVE(p->h, 1);
    n = node(p, S_BLOCK);
    n->list = l.v; n->nlist = l.n;
    return n;
}

static int starts_decl(P *p, const char *v)
{
    static const char *const T[] = { "const","highp","mediump","lowp","float","int","bool","sampler2D","samplerCube","sampler3D",NULL };
    int i;
    for (i = 0; T[i]; i++) if (!strcmp(T[i], v)) return 1;
    return vec_or_mat(v) || sname_of(p, v) != NULL;
}

static Decl *dv_new(P *p, DV *l, DeclKind k);
static const char *parse_struct(P *p, Member **pm, int *pn);

/* §6.2: the statement of an if or an else is a scope of its own: a declaration there goes in a block */
static Node *scoped(P *p, Node *s)
{
    Node *b;
    if (s->kind != S_LDECL && s->kind != S_MULTI && s->kind != S_LSTRUCT) return s;
    b = node(p, S_BLOCK);
    b->list = (Node **)sh_alloc(p->h, sizeof(Node *));
    b->list[0] = s; b->nlist = 1;
    return b;
}

/* the local declarators after a type: name[size] = init, ... ; */
static Node *ldecl_names(P *p, int isc, const char *ty, const char *prec, Node *first)
{
    NV l;
    Node *n;
    memset(&l, 0, sizeof l);
    if (first) nv_push(p, &l, first);
    for (;;) {
        Node *d = node(p, S_LDECL);
        d->isc = isc; d->ty = ty; d->prec = prec;
        d->s = ident(p);
        if (accept_(p, "[")) { d->b = assign(p); eat(p, "]"); }
        if (accept_(p, "=")) d->a = assign(p);
        nv_push(p, &l, d);
        if (!accept_(p, ",")) break;
    }
    eat(p, ";");
    if (l.n == 1) return l.v[0];
    n = node(p, S_MULTI); n->list = l.v; n->nlist = l.n;
    return n;
}

static Node *stmt(P *p)
{
    const char *v = peek(p)->v;
    Node *n;
    SH_ENTER(p->h, 1);
    if (!strcmp(v, "{")) n = block(p);
    else if (!strcmp(v, ";")) { next(p); n = node(p, S_BLOCK); }       /* the empty statement */
    else if (!strcmp(v, "precision")) {
        next(p);
        n = node(p, S_PREC);
        n->prec = next(p)->v; n->ty = next(p)->v;
        eat(p, ";");
    }
    else if (!strcmp(v, "struct")) {                                   /* a struct defined in a function: known to every function, its name to this scope */
        Member *m; int nm;
        const char *sname = parse_struct(p, &m, &nm);
        Decl *d = dv_new(p, &p->hoisted, D_STRUCT);
        d->name = sname; d->members = m; d->nmembers = nm;
        n = node(p, S_LSTRUCT);
        n->s = sname; n->mem = m; n->nmem = nm;
        if (!accept_(p, ";")) {
            Node *r = ldecl_names(p, 0, sname, NULL, n);
            n = r;
        }
    }
    else if (!strcmp(v, "if")) {
        next(p); eat(p, "(");
        n = node(p, S_IF);
        n->a = expr(p);
        eat(p, ")");
        n->b = scoped(p, stmt(p));
        if (accept_(p, "else")) n->c = scoped(p, stmt(p));
    }
    else if (!strcmp(v, "for")) {
        next(p); eat(p, "(");
        n = node(p, S_FOR);
        sscope_push(p);
        n->a = stmt(p);
        if (at(p, ";")) { n->b = node(p, N_NUM); n->b->num = 1.0; n->b->lit = LIT_BOOL; }       /* for (;;): true */
        else n->b = expr(p);
        eat(p, ";");
        if (!at(p, ")")) n->c = expr(p);
        eat(p, ")");
        n->d = stmt(p);
        p->nscope--;
    }
    else if (!strcmp(v, "while")) {
        next(p); eat(p, "(");
        n = node(p, S_WHILE);
        n->a = expr(p);
        eat(p, ")");
        n->b = stmt(p);
    }
    else if (!strcmp(v, "do")) {
        next(p);
        n = node(p, S_DOWHILE);
        n->b = stmt(p);
        eat(p, "while"); eat(p, "(");
        n->a = expr(p);
        eat(p, ")"); eat(p, ";");
    }
    else if (!strcmp(v, "return")) {
        next(p);
        n = node(p, S_RETURN);
        if (!at(p, ";")) n->a = expr(p);
        eat(p, ";");
    }
    else if (!strcmp(v, "discard")) { next(p); eat(p, ";"); n = node(p, S_DISCARD); }
    else if (!strcmp(v, "break")) { next(p); eat(p, ";"); n = node(p, S_BREAK); }
    else if (!strcmp(v, "continue")) { next(p); eat(p, ";"); n = node(p, S_CONTINUE); }
    else if (starts_decl(p, v) && !strcmp(p->t[p->i + 1].v, "(")) {    /* a constructor starts an expression: float(x) + 1.0; */
        Node *e = expr(p);
        eat(p, ";");
        n = node(p, S_EXPR); n->a = e;
    }
    else if (starts_decl(p, v)) {
        int isc = accept_(p, "const");
        const char *ty, *pr;
        pr = skip_precision(p);
        ty = type_name(p);
        n = ldecl_names(p, isc, ty, pr, NULL);
    }
    else {
        Node *e = expr(p);
        eat(p, ";");
        n = node(p, S_EXPR); n->a = e;
    }
    SH_LEAVE(p->h, 1);
    return n;
}

/* ---- top level ---- */
static Decl *dv_new(P *p, DV *l, DeclKind k)
{
    Decl *d;
    if (l->n == l->cap) {
        int cap = l->cap ? l->cap * 2 : 16;
        Decl *nn = (Decl *)sh_alloc(p->h, (size_t)cap * sizeof(Decl));
        if (l->n) memcpy(nn, l->v, (size_t)l->n * sizeof(Decl));
        l->v = nn; l->cap = cap;
    }
    d = &l->v[l->n++];
    memset(d, 0, sizeof *d);
    d->kind = k;
    return d;
}

static const char *parse_struct(P *p, Member **pm, int *pn)
{
    const char *name = NULL, *real;
    int nm = 0, cap = 8;
    Member *m = (Member *)sh_alloc(p->h, (size_t)cap * sizeof(Member));
    Dict *top;
    next(p);
    p->nstruct++;
    if (!at(p, "{")) name = ident(p);
    eat(p, "{");
    while (!at(p, "}")) {
        const char *mty, *pr;
        pr = skip_precision(p);
        mty = type_name(p);
        for (;;) {
            const char *mname = ident(p);
            Node *marr = NULL;
            if (accept_(p, "[")) { marr = assign(p); eat(p, "]"); }
            if (nm == cap) {
                Member *nn = (Member *)sh_alloc(p->h, (size_t)cap * 2 * sizeof(Member));
                memcpy(nn, m, (size_t)nm * sizeof(Member)); m = nn; cap *= 2;
            }
            m[nm].ty = mty; m[nm].name = mname; m[nm].arr = marr; m[nm].prec = pr; nm++;
            if (!accept_(p, ",")) break;
        }
        eat(p, ";");
    }
    eat(p, "}");
    top = p->sscope[p->nscope - 1];
    if (p->h->strict && name && dict_find(top, name)) sh_compile_error(p->h, "struct %s is already defined", name);
    real = name && !sname_of(p, name) ? name : sh_fmt(p->h, "%s@%d", name ? name : "_anon", p->nstruct);
    if (name) dict_put(p->h, top, name)->p = (void *)real;
    *pm = m; *pn = nm;
    return real;
}

/* the global declarators after a type */
static void gdecl_names(P *p, DV *l, const char *qual, const char *ty, const char *prec)
{
    for (;;) {
        const char *name = ident(p);
        Node *arr = NULL, *init = NULL;
        Decl *d;
        if (accept_(p, "[")) { arr = assign(p); eat(p, "]"); }
        if (accept_(p, "=")) init = assign(p);
        d = dv_new(p, l, D_GDECL);
        d->qual = qual; d->ty = ty; d->name = name; d->arr = arr; d->init = init; d->prec = prec;
        if (!accept_(p, ",")) break;
    }
    eat(p, ";");
}

static void parse_params(P *p, Decl *d)
{
    int np = 0, cap = 4;
    Param *ps = (Param *)sh_alloc(p->h, (size_t)cap * sizeof(Param));
    eat(p, "(");
    if (at(p, "void") && !strcmp(p->t[p->i + 1].v, ")")) next(p);      /* f(void) */
    if (!at(p, ")")) {
        for (;;) {
            const char *q = "in", *ty, *nm = "", *pr = NULL;
            Node *arr = NULL;
            int isc = 0, last = -1;
            for (;;) {
                const char *v = peek(p)->v;
                int o;
                if (strcmp(v, "in") && strcmp(v, "out") && strcmp(v, "inout") && strcmp(v, "const") &&
                    strcmp(v, "highp") && strcmp(v, "mediump") && strcmp(v, "lowp")) break;
                next(p);
                o = !strcmp(v, "const") ? 0 : !strcmp(v, "in") || !strcmp(v, "out") || !strcmp(v, "inout") ? 1 : 2;   /* §4.6.4: const, then in / out / inout, then a precision */
                if (p->h->strict && o <= last) sh_compile_error(p->h, "parameter qualifier '%s' out of order", v);
                last = o;
                if (!strcmp(v, "in") || !strcmp(v, "out") || !strcmp(v, "inout")) q = v;
                else if (!strcmp(v, "const")) isc = 1;
                else pr = v;
            }
            ty = type_name(p);
            if (!at(p, ",") && !at(p, ")") && !at(p, "[")) nm = ident(p);       /* a prototype may leave the name out */
            if (accept_(p, "[")) { arr = assign(p); eat(p, "]"); }
            if (np == cap) {
                Param *nn = (Param *)sh_alloc(p->h, (size_t)cap * 2 * sizeof(Param));
                memcpy(nn, ps, (size_t)np * sizeof(Param)); ps = nn; cap *= 2;
            }
            ps[np].q = q; ps[np].ty = ty; ps[np].name = nm; ps[np].prec = pr; ps[np].arr = arr; ps[np].isconst = isc; np++;
            if (!accept_(p, ",")) break;
        }
    }
    eat(p, ")");
    d->params = ps; d->nparams = np;
}

static void top_decl(P *p, DV *l)
{
    const char *v = peek(p)->v;
    if (!strcmp(v, "precision")) {
        Decl *d;
        next(p);
        d = dv_new(p, l, D_PREC);
        d->qual = next(p)->v; d->ty = next(p)->v;
        eat(p, ";");
        return;
    }
    if (!strcmp(v, "struct")) {
        Member *m; int nm;
        const char *sname = parse_struct(p, &m, &nm);
        Decl *d = dv_new(p, l, D_STRUCT);
        d->name = sname; d->members = m; d->nmembers = nm;
        if (accept_(p, ";")) return;
        gdecl_names(p, l, "var", sname, NULL);                  /* struct S{..} a, b; */
        return;
    }
    if (!strcmp(v, "invariant")) {           /* every program here is computed one way: nothing to do */
        next(p);
        if (!at(p, "varying")) {             /* invariant gl_Position, v; : names declared before */
            while (!accept_(p, ";")) {
                Tk *k = next(p);
                if (p->h->strict && k->k != TK_ID && strcmp(k->v, ",")) sh_compile_error(p->h, "invariant qualifies varyings and the outputs gl_Position and gl_PointSize");
            }
            return;
        }
        v = peek(p)->v;
    }
    if (!strcmp(v, "attribute") || !strcmp(v, "uniform") || !strcmp(v, "varying") || !strcmp(v, "const")) {
        const char *qual = next(p)->v, *ty, *pr;
        pr = skip_precision(p);
        if (at(p, "struct")) {                                  /* uniform struct S {..} u; */
            Member *m; int nm;
            Decl *d;
            ty = parse_struct(p, &m, &nm);
            d = dv_new(p, l, D_STRUCT);
            d->name = ty; d->members = m; d->nmembers = nm;
        }
        else ty = type_name(p);
        gdecl_names(p, l, qual, ty, pr);
        return;
    }
    {
        const char *pr = skip_precision(p);
        const char *ty = type_name(p), *name = ident(p);
        if (at(p, "(")) {
            Decl tmp, *d;
            Node *body = NULL;
            int proto;
            memset(&tmp, 0, sizeof tmp);
            parse_params(p, &tmp);
            proto = accept_(p, ";");
            if (!proto) body = block(p);
            d = dv_new(p, l, proto ? D_PROTO : D_FUNC);
            d->ty = ty; d->name = name; d->params = tmp.params; d->nparams = tmp.nparams; d->body = body; d->prec = pr;
            return;
        }
        for (;;) {                                              /* a plain global: kept with the outputs, which main and every function see */
            Node *arr = NULL, *init = NULL;
            Decl *d;
            if (accept_(p, "[")) { arr = assign(p); eat(p, "]"); }
            if (accept_(p, "=")) init = assign(p);
            d = dv_new(p, l, D_GDECL);
            d->qual = "var"; d->ty = ty; d->name = name; d->arr = arr; d->init = init; d->prec = pr;
            if (!accept_(p, ",")) break;
            name = ident(p);
        }
        eat(p, ";");
    }
}

Decl *sh_parse(Shc *h, const char *src, int exts, int *ndecl, Decl **hoisted, int *nhoisted)
{
    P p;
    DV l;
    memset(&p, 0, sizeof p);
    memset(&l, 0, sizeof l);
    p.h = h;
    p.t = lex(h, src, exts, &p.n);
    sscope_push(&p);
    if (h->strict) {
        int i, k;
        for (i = 0; i < p.n; i++)
            if (p.t[i].k == TK_ID)
                for (k = 0; RESERVED[k]; k++) if (!strcmp(RESERVED[k], p.t[i].v)) sh_compile_error(h, "'%s' is a reserved word", p.t[i].v);
    }
    while (peek(&p)->k != TK_EOF) top_decl(&p, &l);
    *ndecl = l.n;
    *hoisted = p.hoisted.v; *nhoisted = p.hoisted.n;
    return l.v;
}
