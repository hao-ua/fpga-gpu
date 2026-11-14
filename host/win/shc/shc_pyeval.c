/* shc_pyeval.c — what `eval(expr, {'__builtins__': {}}, {})` does for the text glslc's eval_if hands it.
 *
 * That text is Python, not the GLSL #if language, and the difference shows: / is true division, % and // floor, ints are
 * unbounded, ** exists, comparisons chain, `1 != 2` arrives here as `1 not = 2` and is a SyntaxError (false), `010` is a
 * SyntaxError (false). So this is a real tokenizer + parser + evaluator of Python's numeric expressions: int (big) and
 * float with Python's rules. Any Python exception, syntax errors included, means 0: eval_if catches Exception.
 * A construct outside that numeric subset that Python would accept (strings, tuples, lists, dicts, sets, complex,
 * Ellipsis, `is`) fails the compile loudly when it is evaluated: this is the one place where the port refuses. */
#include <math.h>
#include <string.h>
#include "shc_internal.h"

enum { T_EOF, T_NEWLINE, T_NUM, T_OP, T_KW, T_NAME, T_STR, T_GAP };
enum { PV_INT, PV_FLOAT };
typedef struct PV { int kind; BI *i; double f; } PV;

typedef struct Tok_ { int t; char op[4]; PV v; } Tok_;

typedef enum { E_NUM, E_NEG, E_POS, E_INV, E_NOT, E_BIN, E_AND, E_OR, E_CMP, E_COND, E_RAISE, E_GAP } EKind;
typedef struct E {
    EKind k;
    PV v;
    char op[4];
    struct E *a, *b, *c;
    struct E **items; char (*ops)[4]; int n;     /* E_CMP: n operands, n - 1 ops */
} E;

typedef struct X {
    Shc *h;
    const char *p, *e;
    int level;                  /* open brackets: the tokenizer's MAXLEVEL */
    int depth;                  /* parser recursion */
    Tok_ tk;
    jmp_buf jb;                 /* a Python exception, syntax errors included: eval_if returns 0 */
} X;

#define MAXLEVEL 200
#define MAXPARSE 3000

SHC_NORETURN static void raise_(X *x) { longjmp(x->jb, 1); }

/* ---------------- tokenizer ---------------- */
static int isxd(int c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }
static int peekc(X *x, const char *p) { return p < x->e ? (unsigned char)*p : -1; }
static int isdg(int c) { return c >= '0' && c <= '9'; }
static int is_idstart(int c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_' || c >= 128; }
static int is_idchar(int c) { return is_idstart(c) || isdg(c); }

static const char *dec_tail(X *x, const char *p)          /* tok_decimal_tail: the first digit is already consumed */
{
    for (;;) {
        while (isdg(peekc(x, p))) p++;
        if (peekc(x, p) != '_') return p;
        p++;
        if (!isdg(peekc(x, p))) raise_(x);
    }
}

static int look(X *x, const char *p, const char *w)
{
    size_t n = strlen(w);
    return (size_t)(x->e - p) >= n && !memcmp(p, w, n);
}

static void verify_end(X *x, const char *p)               /* verify_end_of_number */
{
    int c = peekc(x, p), r = 0;
    if (c == 'a') r = look(x, p + 1, "nd");
    else if (c == 'e') r = look(x, p + 1, "lse");
    else if (c == 'f') r = look(x, p + 1, "or");
    else if (c == 'i') { int c2 = peekc(x, p + 1); r = c2 == 'f' || c2 == 'n' || c2 == 's'; }
    else if (c == 'o') r = look(x, p + 1, "r");
    else if (c == 'n') r = look(x, p + 1, "ot");
    if (!r && c >= 0 && c < 128 && is_idchar(c)) raise_(x);
}

static void set_int(X *x, const char *s, const char *e, int base)
{
    size_t digits = 0;
    const char *q;
    x->tk.t = T_NUM; x->tk.v.kind = PV_INT;
    for (q = s; q < e; q++) if (*q != '_') digits++;
    if (base == 10 && digits > 4300) raise_(x);           /* sys.int_info.default_max_str_digits: a SyntaxError */
    x->tk.v.i = bi_from_digits(x->h, s, (size_t)(e - s), base);
}

static void set_float(X *x, const char *s, const char *e)
{
    x->tk.t = T_NUM; x->tk.v.kind = PV_FLOAT;
    x->tk.v.f = sh_str2d(x->h, s, (size_t)(e - s));
}

static void radix(X *x, const char *s, const char *p, int base)
{
    int c;
    do {
        if (peekc(x, p) == '_') p++;
        c = peekc(x, p);
        if (base == 16 ? !isxd(c) : !(c >= '0' && c < '0' + base)) raise_(x);
        do { p++; c = peekc(x, p); } while (base == 16 ? isxd(c) : (c >= '0' && c < '0' + base));
        if (base != 16 && isdg(c)) raise_(x);
    } while (c == '_');
    verify_end(x, p);
    set_int(x, s + 2, p, base);
    x->p = p;
}

static void tok_number(X *x)
{
    const char *s = x->p, *p = s;
    int c = peekc(x, p), isf = 0;
    if (c == '0') {
        int nonzero = 0;
        p++; c = peekc(x, p);
        if (c == 'x' || c == 'X') { radix(x, s, p + 1, 16); return; }
        if (c == 'o' || c == 'O') { radix(x, s, p + 1, 8); return; }
        if (c == 'b' || c == 'B') { radix(x, s, p + 1, 2); return; }
        for (;;) {
            if (peekc(x, p) == '_') { p++; if (!isdg(peekc(x, p))) raise_(x); }
            if (peekc(x, p) != '0') break;
            p++;
        }
        if (isdg(peekc(x, p))) { nonzero = 1; p = dec_tail(x, p + 1); }
        c = peekc(x, p);
        if (c == '.') { p++; isf = 1; goto fraction; }
        if (c == 'e' || c == 'E') goto exponent;
        if (c == 'j' || c == 'J') goto imaginary;
        if (nonzero) raise_(x);                           /* old-style octal */
        verify_end(x, p);
        set_int(x, s, p, 10);
        x->p = p;
        return;
    }
    p = dec_tail(x, p + 1);
    if (peekc(x, p) == '.') {
        p++; isf = 1;
fraction:
        if (isdg(peekc(x, p))) p = dec_tail(x, p + 1);
    }
    c = peekc(x, p);
    if (c == 'e' || c == 'E') {
        const char *q;
exponent:
        q = p + 1;
        c = peekc(x, q);
        if (c == '+' || c == '-') {
            q++;
            if (!isdg(peekc(x, q))) raise_(x);
        } else if (!isdg(c)) {
            verify_end(x, p);                             /* `1else`: the number ends before the e */
            if (isf) set_float(x, s, p); else set_int(x, s, p, 10);
            x->p = p;
            return;
        }
        p = dec_tail(x, q + 1);
        isf = 1;
    }
    c = peekc(x, p);
    if (c == 'j' || c == 'J') {
imaginary:
        p++;
        verify_end(x, p);
        x->tk.t = T_GAP;                                  /* complex */
        x->p = p;
        return;
    }
    verify_end(x, p);
    if (isf) set_float(x, s, p); else set_int(x, s, p, 10);
    x->p = p;
}

static void tok_string(X *x, int q)
{
    const char *p = x->p + 1;
    int triple = 0;
    if (peekc(x, p) == q && peekc(x, p + 1) == q) { triple = 1; p += 2; }
    for (;;) {
        int c = peekc(x, p);
        if (c < 0) raise_(x);                             /* unterminated */
        if (c == '\\') { p += 2; continue; }
        if (c == q) {
            if (!triple) { p++; break; }
            if (peekc(x, p + 1) == q && peekc(x, p + 2) == q) { p += 3; break; }
        }
        if ((c == '\n' || c == '\r') && !triple) raise_(x);
        p++;
    }
    x->p = p;
    x->tk.t = T_STR;
}

static const char *const OPS3[] = { "**=", "//=", ">>=", "<<=", "...", NULL };
static const char *const OPS2[] = { "**", "//", ">>", "<<", "<=", ">=", "==", "!=", "<>", "->", "+=", "-=", "*=", "/=", "%=", "@=", "&=", "|=", "^=", ":=", NULL };
static const char OPS1[] = "+-*/%@&|^~<>()[]{},:.;=";

static void next_tok(X *x)
{
    int c, i;
    for (;;) {
        c = peekc(x, x->p);
        if (c == ' ' || c == '\t' || c == '\f') { x->p++; continue; }
        if (c == '#') {                                   /* a Python comment */
            while (x->p < x->e && *x->p != '\n' && *x->p != '\r') x->p++;
            continue;
        }
        if (c == '\\') {                                  /* line continuation */
            int d = peekc(x, x->p + 1);
            if (d == '\n') { x->p += 2; continue; }
            if (d == '\r') { x->p += 2; if (peekc(x, x->p) == '\n') x->p++; continue; }
            raise_(x);
        }
        if (c == '\n' || c == '\r') {
            x->p++;
            if (x->level > 0) continue;                   /* inside brackets a newline is nothing */
            x->tk.t = T_NEWLINE;
            return;
        }
        break;
    }
    if (c < 0) { x->tk.t = T_EOF; return; }
    if (isdg(c) || (c == '.' && isdg(peekc(x, x->p + 1)))) {
        if (c == '.') {                                   /* `.5`: the fraction path of the tokenizer */
            const char *s = x->p, *p = dec_tail(x, x->p + 2);
            int isj;
            c = peekc(x, p);
            if (c == 'e' || c == 'E') {
                const char *q = p + 1;
                c = peekc(x, q);
                if (c == '+' || c == '-') { q++; if (!isdg(peekc(x, q))) raise_(x); p = dec_tail(x, q + 1); }
                else if (isdg(c)) p = dec_tail(x, q + 1);
                else { verify_end(x, p); set_float(x, s, p); x->p = p; return; }
            }
            c = peekc(x, p);
            isj = c == 'j' || c == 'J';
            if (isj) p++;
            verify_end(x, p);
            if (isj) x->tk.t = T_GAP; else set_float(x, s, p);
            x->p = p;
            return;
        }
        tok_number(x);
        return;
    }
    if (c == '"' || c == '\'') { tok_string(x, c); return; }
    if (is_idstart(c)) {
        const char *s = x->p;
        if (c >= 128) raise_(x);                          /* not an identifier we know about: invalid character */
        while (is_idchar(peekc(x, x->p))) {
            if (peekc(x, x->p) >= 128) raise_(x);
            x->p++;
        }
        {
            size_t n = (size_t)(x->p - s);
            static const char *const KW[] = { "or", "and", "not", "if", "else", "in", "is", "for", NULL };
            x->tk.t = T_NAME;
            for (i = 0; KW[i]; i++)
                if (strlen(KW[i]) == n && !memcmp(KW[i], s, n)) { x->tk.t = T_KW; strcpy(x->tk.op, KW[i]); }
        }
        return;
    }
    for (i = 0; OPS3[i]; i++) if (look(x, x->p, OPS3[i])) { x->tk.t = T_OP; strcpy(x->tk.op, OPS3[i]); x->p += 3; return; }
    for (i = 0; OPS2[i]; i++) if (look(x, x->p, OPS2[i])) { x->tk.t = T_OP; strcpy(x->tk.op, OPS2[i]); x->p += 2; return; }
    if (c < 128 && strchr(OPS1, c)) {
        if (c == '(' || c == '[' || c == '{') { if (x->level >= MAXLEVEL) raise_(x); x->level++; }
        if (c == ')' || c == ']' || c == '}') { if (x->level == 0) raise_(x); x->level--; }
        x->tk.t = T_OP; x->tk.op[0] = (char)c; x->tk.op[1] = 0; x->p++;
        return;
    }
    raise_(x);                                            /* $ ? ` ! ... */
}

/* ---------------- parser: Python's expression grammar over the tokens above ---------------- */
static int is_op(X *x, const char *o) { return x->tk.t == T_OP && !strcmp(x->tk.op, o); }
static int is_kw(X *x, const char *k) { return x->tk.t == T_KW && !strcmp(x->tk.op, k); }
static E *mk(X *x, EKind k) { E *e = (E *)sh_zalloc(x->h, sizeof(E)); e->k = k; return e; }
static E *p_or(X *x);
static E *p_expression(X *x);

static E *p_bin_tail(X *x, E *a, const char *op, E *b)
{
    E *e = mk(x, E_BIN);
    strcpy(e->op, op); e->a = a; e->b = b;
    return e;
}

/* skip a bracketed display whose contents are outside the numeric subset */
static void skip_brackets(X *x)
{
    int d = 1;
    while (d > 0) {
        next_tok(x);
        if (x->tk.t == T_EOF || x->tk.t == T_NEWLINE) raise_(x);
        if (is_op(x, "(") || is_op(x, "[") || is_op(x, "{")) d++;
        if (is_op(x, ")") || is_op(x, "]") || is_op(x, "}")) d--;
    }
    next_tok(x);
}

static E *p_atom(X *x)
{
    E *e;
    if (x->tk.t == T_NUM) { e = mk(x, E_NUM); e->v = x->tk.v; next_tok(x); return e; }
    if (x->tk.t == T_GAP) { next_tok(x); return mk(x, E_GAP); }
    if (x->tk.t == T_STR) {
        while (x->tk.t == T_STR) next_tok(x);             /* adjacent strings concatenate */
        return mk(x, E_GAP);
    }
    if (is_op(x, "...")) { next_tok(x); return mk(x, E_GAP); }
    if (is_op(x, "(")) {
        next_tok(x);
        if (is_op(x, ")")) { next_tok(x); return mk(x, E_GAP); }        /* () */
        e = p_expression(x);
        if (is_op(x, ",")) {                              /* a tuple: parse it for syntax, refuse to evaluate it */
            while (is_op(x, ",")) {
                next_tok(x);
                if (is_op(x, ")")) break;
                p_expression(x);
            }
            e = mk(x, E_GAP);
        }
        if (!is_op(x, ")")) raise_(x);
        next_tok(x);
        return e;
    }
    if (is_op(x, "[") || is_op(x, "{")) { skip_brackets(x); return mk(x, E_GAP); }
    raise_(x);
    return NULL;
}

static E *p_primary(X *x)
{
    E *e = p_atom(x);
    for (;;) {
        if (is_op(x, "(")) {                              /* calling a number: a TypeError when evaluated */
            E *r = mk(x, E_RAISE);
            next_tok(x);
            if (!is_op(x, ")")) {
                for (;;) {
                    p_expression(x);
                    if (!is_op(x, ",")) break;
                    next_tok(x);
                    if (is_op(x, ")")) break;
                }
            }
            if (!is_op(x, ")")) raise_(x);
            next_tok(x);
            r->a = e;
            e = r;
        }
        else if (is_op(x, "[")) {                         /* subscripting a number: likewise */
            E *r = mk(x, E_RAISE);
            next_tok(x);
            p_expression(x);
            if (!is_op(x, "]")) { r->k = E_GAP; while (!is_op(x, "]")) { if (x->tk.t == T_EOF || x->tk.t == T_NEWLINE) raise_(x); next_tok(x); } }
            next_tok(x);
            r->a = e;
            e = r;
        }
        else break;
    }
    return e;
}

static E *p_factor(X *x);

static E *p_power(X *x)
{
    E *a = p_primary(x);
    if (is_op(x, "**")) {
        next_tok(x);
        return p_bin_tail(x, a, "**", p_factor(x));
    }
    return a;
}

static E *p_factor(X *x)
{
    E *e;
    if (++x->depth > MAXPARSE) raise_(x);                 /* the parser's stack limit: MemoryError */
    if (is_op(x, "-") || is_op(x, "+") || is_op(x, "~")) {
        EKind k = is_op(x, "-") ? E_NEG : is_op(x, "+") ? E_POS : E_INV;
        next_tok(x);
        e = mk(x, k);
        e->a = p_factor(x);
    }
    else e = p_power(x);
    x->depth--;
    return e;
}

static E *p_term(X *x)
{
    E *a = p_factor(x);
    while (is_op(x, "*") || is_op(x, "/") || is_op(x, "//") || is_op(x, "%") || is_op(x, "@")) {
        char op[4];
        strcpy(op, x->tk.op);
        next_tok(x);
        a = p_bin_tail(x, a, op, p_factor(x));
    }
    return a;
}

static E *p_sum(X *x)
{
    E *a = p_term(x);
    while (is_op(x, "+") || is_op(x, "-")) {
        char op[4];
        strcpy(op, x->tk.op);
        next_tok(x);
        a = p_bin_tail(x, a, op, p_term(x));
    }
    return a;
}

static E *p_shift(X *x)
{
    E *a = p_sum(x);
    while (is_op(x, "<<") || is_op(x, ">>")) {
        char op[4];
        strcpy(op, x->tk.op);
        next_tok(x);
        a = p_bin_tail(x, a, op, p_sum(x));
    }
    return a;
}

static E *p_band(X *x)
{
    E *a = p_shift(x);
    while (is_op(x, "&")) { next_tok(x); a = p_bin_tail(x, a, "&", p_shift(x)); }
    return a;
}

static E *p_bxor(X *x)
{
    E *a = p_band(x);
    while (is_op(x, "^")) { next_tok(x); a = p_bin_tail(x, a, "^", p_band(x)); }
    return a;
}

static E *p_bor(X *x)
{
    E *a = p_bxor(x);
    while (is_op(x, "|")) { next_tok(x); a = p_bin_tail(x, a, "|", p_bxor(x)); }
    return a;
}

static E *p_compare(X *x)
{
    E *a = p_bor(x), *e;
    E **items = NULL; char (*ops)[4] = NULL; int n = 1, cap = 0;
    for (;;) {
        char op[4];
        if (is_op(x, "<") || is_op(x, ">") || is_op(x, "<=") || is_op(x, ">=") || is_op(x, "==") || is_op(x, "!=")) {
            strcpy(op, x->tk.op);
            next_tok(x);
        }
        else if (is_kw(x, "in")) { strcpy(op, "in"); next_tok(x); }
        else if (is_kw(x, "is")) {
            strcpy(op, "is");
            next_tok(x);
            if (is_kw(x, "not")) next_tok(x);
        }
        else if (is_kw(x, "not")) {
            next_tok(x);
            if (!is_kw(x, "in")) raise_(x);               /* `1 not = 2`: what `1 != 2` has become */
            strcpy(op, "in");
            next_tok(x);
        }
        else break;
        if (n >= cap) {
            int nc = cap ? cap * 2 : 4;
            E **ni = (E **)sh_alloc(x->h, (size_t)nc * sizeof(E *));
            char (*no)[4] = (char (*)[4])sh_alloc(x->h, (size_t)nc * 4);
            if (cap) { memcpy(ni, items, (size_t)n * sizeof(E *)); memcpy(no, ops, (size_t)(n - 1) * 4); }
            else ni[0] = a;
            items = ni; ops = no; cap = nc;
        }
        strcpy(ops[n - 1], op);
        items[n++] = p_bor(x);
    }
    if (n == 1) return a;
    e = mk(x, E_CMP);
    e->n = n; e->items = items; e->ops = ops;
    return e;
}

static E *p_not(X *x)
{
    E *e;
    if (++x->depth > MAXPARSE) raise_(x);
    if (is_kw(x, "not")) {
        next_tok(x);
        e = mk(x, E_NOT);
        e->a = p_not(x);
    }
    else e = p_compare(x);
    x->depth--;
    return e;
}

static E *p_and(X *x)
{
    E *a = p_not(x);
    while (is_kw(x, "and")) {
        E *e = mk(x, E_AND);
        next_tok(x);
        e->a = a; e->b = p_not(x);
        a = e;
    }
    return a;
}

static E *p_or(X *x)
{
    E *a = p_and(x);
    while (is_kw(x, "or")) {
        E *e = mk(x, E_OR);
        next_tok(x);
        e->a = a; e->b = p_and(x);
        a = e;
    }
    return a;
}

static E *p_expression(X *x)
{
    E *a;
    if (++x->depth > MAXPARSE) raise_(x);
    a = p_or(x);
    if (is_kw(x, "if")) {
        E *e = mk(x, E_COND);
        next_tok(x);
        e->c = p_or(x);
        if (!is_kw(x, "else")) raise_(x);
        next_tok(x);
        e->a = a; e->b = p_expression(x);
        a = e;
    }
    x->depth--;
    return a;
}

/* ---------------- evaluation ---------------- */
static int truth(const PV *v) { return v->kind == PV_INT ? !bi_is_zero(v->i) : v->f != 0.0; }
static PV pv_int(BI *i) { PV v; v.kind = PV_INT; v.i = i; v.f = 0; return v; }
static PV pv_float(double f) { PV v; v.kind = PV_FLOAT; v.i = NULL; v.f = f; return v; }
static PV pv_bool(X *x, int b) { return pv_int(bi_from_u64(x->h, (uint64_t)(b != 0))); }

static double to_f(X *x, const PV *v)
{
    int ovf;
    double d;
    if (v->kind == PV_FLOAT) return v->f;
    d = bi_to_double(x->h, v->i, &ovf);
    if (ovf) raise_(x);                                   /* OverflowError: int too large to convert to float */
    return d;
}

/* <0, 0, >0, or 2 = unordered. An int against a float compares exactly (float_richcompare). */
static int cmp_nf(X *x, const BI *a, double b)
{
    BI *t;
    int c;
    double tr;
    if (b != b) return 2;
    if (b == HUGE_VAL) return -1;
    if (b == -HUGE_VAL) return 1;
    tr = b < 0 ? ceil(b) : floor(b);
    t = bi_from_double(x->h, tr);
    c = bi_cmp(a, t);
    if (c) return c;
    return b > tr ? -1 : b < tr ? 1 : 0;
}

static int cmp_pv(X *x, const PV *a, const PV *b)
{
    int c;
    if (a->kind == PV_INT && b->kind == PV_INT) return bi_cmp(a->i, b->i);
    if (a->kind == PV_FLOAT && b->kind == PV_FLOAT) {
        if (a->f != a->f || b->f != b->f) return 2;
        return a->f < b->f ? -1 : a->f > b->f ? 1 : 0;
    }
    if (a->kind == PV_INT) return cmp_nf(x, a->i, b->f);
    c = cmp_nf(x, b->i, a->f);
    return c == 2 ? 2 : -c;
}

static int cmp_true(int c, const char *op)
{
    if (!strcmp(op, "!=")) return c != 0;
    if (c == 2) return 0;
    if (!strcmp(op, "<")) return c < 0;
    if (!strcmp(op, ">")) return c > 0;
    if (!strcmp(op, "<=")) return c <= 0;
    if (!strcmp(op, ">=")) return c >= 0;
    return c == 0;                                        /* == */
}

static int odd_f(double w)                                /* DOUBLE_IS_ODD_INTEGER */
{
    return sh_fmod(fabs(w), 2.0) == 1.0;
}

static PV float_pow(X *x, double iv, double iw)
{
    int neg = 0;
    double ix;
    if (iw == 0) return pv_float(1.0);
    if (iv != iv) return pv_float(iv);
    if (iw != iw) return pv_float(iv == 1.0 ? 1.0 : iw);
    if (iw == HUGE_VAL || iw == -HUGE_VAL) {
        iv = fabs(iv);
        if (iv == 1.0) return pv_float(1.0);
        if ((iw > 0.0) == (iv > 1.0)) return pv_float(fabs(iw));
        return pv_float(0.0);
    }
    if (iv == HUGE_VAL || iv == -HUGE_VAL) {
        int odd = odd_f(iw);
        if (iw > 0.0) return pv_float(odd ? iv : fabs(iv));
        return pv_float(odd ? copysign(0.0, iv) : 0.0);
    }
    if (iv == 0.0) {
        if (iw < 0.0) raise_(x);                          /* ZeroDivisionError */
        return pv_float(odd_f(iw) ? iv : 0.0);
    }
    if (iv < 0.0) {
        if (iw != floor(iw)) sh_compile_error(x->h, "#if expression outside the numeric subset (complex result)");
        iv = -iv;
        neg = odd_f(iw);
    }
    if (iv == 1.0) return pv_float(neg ? -1.0 : 1.0);
    ix = pow(iv, iw);
    if (ix == HUGE_VAL) raise_(x);                        /* OverflowError */
    return pv_float(neg ? -ix : ix);
}

static PV eval_e(X *x, E *e);

static PV eval_bin(X *x, E *e)
{
    PV a = eval_e(x, e->a), b = eval_e(x, e->b);
    const char *op = e->op;
    Shc *h = x->h;
    if (!strcmp(op, "@")) raise_(x);                      /* TypeError */
    if (!strcmp(op, "<<") || !strcmp(op, ">>") || !strcmp(op, "&") || !strcmp(op, "|") || !strcmp(op, "^")) {
        if (a.kind != PV_INT || b.kind != PV_INT) raise_(x);   /* TypeError */
        if (op[0] == '<' || op[0] == '>') {
            uint64_t k;
            if (b.i->neg) raise_(x);                      /* ValueError: negative shift count */
            if (op[0] == '<') {
                if (bi_is_zero(a.i)) return a;
                if (!bi_fits_u64(b.i, &k)) sh_compile_error(h, "integer too large in a #if expression");
                return pv_int(bi_shl(h, a.i, k));
            }
            if (!bi_fits_u64(b.i, &k)) return pv_int(a.i->neg ? bi_neg(h, bi_from_u64(h, 1)) : bi_from_u64(h, 0));
            return pv_int(bi_shr(h, a.i, k));
        }
        return pv_int(bi_bitop(h, a.i, b.i, op[0]));
    }
    if (a.kind == PV_INT && b.kind == PV_INT) {
        BI *q, *r;
        int ovf;
        double d;
        if (!strcmp(op, "+")) return pv_int(bi_add(h, a.i, b.i));
        if (!strcmp(op, "-")) return pv_int(bi_sub(h, a.i, b.i));
        if (!strcmp(op, "*")) return pv_int(bi_mul(h, a.i, b.i));
        if (!strcmp(op, "**")) {
            uint64_t k, m;
            if (b.i->neg) return float_pow(x, to_f(x, &a), to_f(x, &b));
            if (bi_fits_u64(a.i, &m) && m <= 1) return bi_is_zero(b.i) ? pv_bool(x, 1) : a;      /* 0 ** n, 1 ** n */
            if (a.i->n == 1 && a.i->d[0] == 1) return bi_is_odd(b.i) ? a : pv_bool(x, 1);                /* (-1) ** n */
            if (!bi_fits_u64(b.i, &k) || k > (1u << 21)) sh_compile_error(h, "integer too large in a #if expression");
            return pv_int(bi_pow(h, a.i, k));
        }
        if (bi_is_zero(b.i)) raise_(x);                   /* ZeroDivisionError */
        if (!strcmp(op, "/")) {
            d = bi_ratio(h, a.i, b.i, &ovf);
            if (ovf) raise_(x);
            return pv_float(d);
        }
        bi_divmod(h, a.i, b.i, &q, &r);
        return pv_int(!strcmp(op, "//") ? q : r);
    }
    {
        double u = to_f(x, &a), v = to_f(x, &b);
        if (!strcmp(op, "+")) return pv_float(u + v);
        if (!strcmp(op, "-")) return pv_float(u - v);
        if (!strcmp(op, "*")) return pv_float(u * v);
        if (!strcmp(op, "**")) return float_pow(x, u, v);
        if (v == 0.0) raise_(x);                          /* ZeroDivisionError */
        if (!strcmp(op, "/")) return pv_float(u / v);
        if (!strcmp(op, "%")) return pv_float(sh_pymod(u, v));
        {                                                 /* float_floor_div */
            double mod = sh_fmod(u, v), div = (u - mod) / v, fd;
            if (mod != 0.0) { if ((v < 0) != (mod < 0)) div -= 1.0; }
            if (div != 0.0) {
                fd = floor(div);
                if (div - fd > 0.5) fd += 1.0;
            }
            else fd = copysign(0.0, u / v);
            return pv_float(fd);
        }
    }
}

static PV eval_e(X *x, E *e)
{
    PV a;
    int i;
    switch (e->k) {
    case E_NUM: return e->v;
    case E_GAP: sh_compile_error(x->h, "#if expression outside the numeric subset of Python's eval()");
    case E_RAISE: raise_(x);                              /* TypeError */
    case E_NEG:
        a = eval_e(x, e->a);
        if (a.kind == PV_INT) return pv_int(bi_neg(x->h, a.i));
        return pv_float(-a.f);
    case E_POS: return eval_e(x, e->a);
    case E_INV:
        a = eval_e(x, e->a);
        if (a.kind != PV_INT) raise_(x);                  /* TypeError */
        return pv_int(bi_invert(x->h, a.i));
    case E_NOT:
        a = eval_e(x, e->a);
        return pv_bool(x, !truth(&a));
    case E_AND:
        a = eval_e(x, e->a);
        return truth(&a) ? eval_e(x, e->b) : a;
    case E_OR:
        a = eval_e(x, e->a);
        return truth(&a) ? a : eval_e(x, e->b);
    case E_COND:
        a = eval_e(x, e->c);
        return truth(&a) ? eval_e(x, e->a) : eval_e(x, e->b);
    case E_BIN: return eval_bin(x, e);
    case E_CMP:
        a = eval_e(x, e->items[0]);
        for (i = 0; i + 1 < e->n; i++) {
            PV b = eval_e(x, e->items[i + 1]);
            if (!strcmp(e->ops[i], "in")) raise_(x);      /* TypeError: an int is not iterable */
            if (!strcmp(e->ops[i], "is")) sh_compile_error(x->h, "#if expression outside the numeric subset of Python's eval()");
            if (!cmp_true(cmp_pv(x, &a, &b), e->ops[i])) return pv_bool(x, 0);
            a = b;
        }
        return pv_bool(x, 1);
    }
    return pv_bool(x, 0);
}

static int run(X *x)
{
    E *e;
    PV v;
    int more_comma = 0;
    next_tok(x);
    e = p_expression(x);
    while (is_op(x, ",")) {                               /* `1, 2` is a tuple */
        more_comma = 1;
        next_tok(x);
        if (x->tk.t == T_EOF || x->tk.t == T_NEWLINE) break;
        p_expression(x);
    }
    while (x->tk.t == T_NEWLINE) next_tok(x);
    if (x->tk.t != T_EOF) raise_(x);
    if (more_comma) sh_compile_error(x->h, "#if expression outside the numeric subset of Python's eval()");
    v = eval_e(x, e);
    return truth(&v);
}

int sh_pyeval_truth(Shc *h, const char *expr)
{
    X x;
    volatile int r = 0;
    ShMark mk = sh_mark(h);
    memset(&x, 0, sizeof x);
    x.h = h;
    x.p = expr; x.e = expr + strlen(expr);
    while (x.p < x.e && (*x.p == ' ' || *x.p == '\t')) x.p++;      /* eval() strips these */
    if (setjmp(x.jb) == 0) r = run(&x);
    else r = 0;
    sh_release(h, mk);
    return r;
}
