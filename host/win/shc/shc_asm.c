/* shc_asm.c — the assembler half of tools/isa64.py: enc_swz, encode, _reg, parse_src, parse_dst, parse_idx, parse_fp,
 * assemble (vs= rules, max_if / max_loop) and check_overlap, with the constants of shader_ref.py they use. The word
 * format is in isa64.py's header. Every AsmError (and every ValueError / IndexError / KeyError the Python raises on a
 * malformed line) is a failure here. */
#include <stdlib.h>
#include <string.h>
#include "shc_internal.h"

enum { NREG = 32, NPARAM = 64, NVARY = 16, SWZ_ID = 0xE4, IF_DEPTH = 8, LOOP_DEPTH = 4 };

typedef struct OpDef { const char *name; int op; } OpDef;
static const OpDef OPS[] = {
    {"NOP",0},{"MOV",1},{"MOVI",2},{"ADD",3},{"SUB",4},{"MUL",5},{"MAD",6},{"MIN",7},{"MAX",8},{"DP3",9},{"DP4",10},
    {"RCP",11},{"RSQRT",12},{"LOG2",13},{"EXP2",14},{"SETP",15},{"LDP",16},{"LDV",17},{"STV",18},{"STO",19},
    {"BRA",20},{"BRP",21},{"LOOP",22},{"ENDL",23},{"IF",24},{"ELSE",25},{"ENDIF",26},
    {"BREAK",27},{"CONT",28},{"DISCARD",29},{"FTOI",30},{"ITOF",31},
    {"STOZ",21},{"DDX",32},{"DDY",33},{"TEX",34},{"SET",35},{"IADD",36},{"ISUB",37},{"ISLT",38},{"ISGE",39},
    {"SIN",40},{"COS",41},{"TRUNC",42},{"FLR",43},{"CEIL",44},{"RNDE",45},{"SEL",46},{"IMUL",47},{"IDIV",48},
    {"ISEQ",49},{"ISNE",50},{"IOP",51},{"IMOD",52},{"UDIV",53},{"UMOD",54},{"FTOU",55},{"UTOF",56},{"LDPX",57},
    {"SQRT",58},{"RCPM",59},{"RSQM",60},{"CLAMP",61},{"END",63},{NULL,0}      /* 58..61: board B's fragment core (isa64 XOPS) */
};
static const OpDef IOP_FN[] = {
    {"IAND",0},{"IOR",1},{"IXOR",2},{"INOT",3},{"ISHL",4},{"ISHR",5},{"USHR",6},{"IMIN",7},{"IMAX",8},{"UMIN",9},{"UMAX",10},
    {"IABS",11},{"INEG",12},{"ISIGN",13},{"ULT",14},{"UGE",15},{NULL,0}
};

static int lookup(const OpDef *t, const char *name)
{
    for (; t->name; t++) if (!strcmp(t->name, name)) return t->op;
    return -1;
}

static int in_list(const char *const *l, const char *s)
{
    for (; *l; l++) if (!strcmp(*l, s)) return 1;
    return 0;
}

static const char *const ARITH2[] = { "ADD","SUB","MUL","MIN","MAX","DP3","DP4","RCPM","RSQM",NULL };
static const char *const ARITH1[] = { "MOV","RCP","RSQRT","LOG2","EXP2","SIN","COS","FTOI","ITOF","FTOU","UTOF","DDX","DDY","SQRT",
                                      "TRUNC","FLR","CEIL","RNDE",NULL };
static const char *const INT2[] = { "IADD","ISUB","ISLT","ISGE","ISEQ","ISNE","IMUL","IDIV","IMOD","UDIV","UMOD",NULL };
static const char *const CTRL_IMM[] = { "BRA","BRP","LOOP","IF","ELSE",NULL };
static const char *const CTRL0[] = { "NOP","ENDL","ENDIF","BREAK","CONT","DISCARD","END",NULL };
static const char *const FS_ONLY[] = { "TEX","DDX","DDY","DISCARD","STOZ","SQRT","RCPM","RSQM","CLAMP",NULL };

typedef struct Src { int reg, swz, mod; } Src;

static uint64_t encode(int op, int wm, int rd, Src s0, Src s1, Src s2, uint64_t imm, int immw)
{
    uint64_t w = ((uint64_t)(op & 0x7F) << 57) | ((uint64_t)(wm & 0xF) << 53) | ((uint64_t)(rd & 0x1F) << 48);
    w |= ((uint64_t)(s0.reg & 0x1F) << 43) | ((uint64_t)(s0.swz & 0xFF) << 35) | ((uint64_t)(s0.mod & 3) << 33);
    w |= ((uint64_t)(s1.reg & 0x1F) << 28) | ((uint64_t)(s1.swz & 0xFF) << 20) | ((uint64_t)(s1.mod & 3) << 18);
    w |= ((uint64_t)(s2.reg & 0x1F) << 13) | ((uint64_t)(s2.swz & 0xFF) << 5) | ((uint64_t)(s2.mod & 3) << 3);
    if (immw) {
        uint64_t m = ((uint64_t)1 << immw) - 1;
        w = (w & ~m) | (imm & m);
    }
    return w;
}

static int lower(int c) { return c >= 'A' && c <= 'Z' ? c + 32 : c; }

static int enc_swz(Shc *h, const char *s0)
{
    char s[5];
    size_t n = strlen(s0), i;
    int v = 0;
    if (n == 0) { s0 = "xyzw"; n = 4; }
    if (n > 4) sh_py_error(h, "bad swizzle .%s", s0);
    for (i = 0; i < n; i++) {
        s[i] = (char)lower((unsigned char)s0[i]);
        if (!s[i] || !strchr("xyzw", s[i])) sh_py_error(h, "bad swizzle .%s", s0);
    }
    for (; i < 4; i++) s[i] = s[i - 1];
    for (i = 0; i < 4; i++) v |= (int)(strchr("xyzw", s[i]) - "xyzw") << (2 * i);
    return v;
}

static int maskbits(Shc *h, const char *suf)                     /* shader_ref.maskbits */
{
    int b = 0;
    for (; *suf; suf++) {
        char c = (char)lower((unsigned char)*suf);
        if (!strchr("xyzw", c)) sh_py_error(h, "ValueError: bad mask");
        b |= 1 << (int)(strchr("xyzw", c) - "xyzw");
    }
    return b;
}

/* int(text, 0): sign, 0x / 0o / 0b, '_' between digits, no leading zeros on a decimal; saturates (every caller range-checks) */
static long long py_int0(Shc *h, const char *t)
{
    const char *e, *b;
    int neg = 0, base = 10, last_us = 1;
    unsigned long long v = 0;
    int ndig = 0, allzero = 1;
    b = t; e = t + strlen(t);
    sh_strip(&b, &e);
    if (b < e && (*b == '+' || *b == '-')) neg = *b++ == '-';
    if (e - b >= 2 && b[0] == '0') {
        int c = lower((unsigned char)b[1]);
        if (c == 'x') base = 16; else if (c == 'o') base = 8; else if (c == 'b') base = 2;
        if (base != 10) { b += 2; last_us = 0; }
    }
    if (base != 10 && b < e && *b == '_') b++;
    if (b >= e) sh_py_error(h, "ValueError: invalid literal for int()");
    {
        const char *p = b;
        int first_digit = -1;
        for (; p < e; p++) {
            int c = (unsigned char)*p, d;
            if (c == '_') {
                if (last_us) sh_py_error(h, "ValueError: invalid literal for int()");
                last_us = 1;
                continue;
            }
            c = lower(c);
            if (c >= '0' && c <= '9') d = c - '0';
            else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
            else d = 99;
            if (d >= base) sh_py_error(h, "ValueError: invalid literal for int()");
            if (first_digit < 0) first_digit = d;
            if (d) allzero = 0;
            last_us = 0;
            ndig++;
            if (v < (1ULL << 58)) v = v * (unsigned)base + (unsigned)d;
            else v = 1ULL << 62;
        }
        if (last_us || !ndig) sh_py_error(h, "ValueError: invalid literal for int()");
        if (base == 10 && first_digit == 0 && !allzero) sh_py_error(h, "ValueError: invalid literal for int()");
    }
    return neg ? -(long long)v : (long long)v;
}

static int reg_(Shc *h, const char *t)
{
    const char *p = t;
    long long n = 0;
    if (*p != 'r' && *p != 'R') sh_py_error(h, "bad register %s", t);
    p++;
    if (!*p) sh_py_error(h, "bad register %s", t);
    for (; *p; p++) {
        if (*p < '0' || *p > '9') sh_py_error(h, "bad register %s", t);
        if (n < 1000) n = n * 10 + (*p - '0');
    }
    if (n >= NREG) sh_py_error(h, "bad register %s", t);
    return (int)n;
}

static Src parse_src(Shc *h, const char *tok)
{
    Src s;
    const char *t = tok, *dot;
    char *r;
    int mod = 0;
    while (*t == '-' || *t == '!') {
        if (*t == '-') mod ^= 1; else mod |= 2;
        t++;
    }
    dot = strchr(t, '.');
    r = sh_strndup(h, t, dot ? (size_t)(dot - t) : strlen(t));
    s.reg = reg_(h, r);
    s.swz = enc_swz(h, dot ? dot + 1 : "");
    s.mod = mod & 3;
    return s;
}

/* parse_dst: -> register; *wm = the mask or -1 for None */
static int parse_dst(Shc *h, const char *tok, int *wm)
{
    const char *dot = strchr(tok, '.');
    int r = reg_(h, sh_strndup(h, tok, dot ? (size_t)(dot - tok) : strlen(tok)));
    *wm = (dot && dot[1]) ? maskbits(h, dot + 1) : -1;
    return r;
}

static long long parse_idx(Shc *h, const char *tok, int *swz)
{
    const char *t = tok, *dot;
    long long v;
    while (*t == '#') t++;
    dot = strchr(t, '.');
    v = py_int0(h, sh_strndup(h, t, dot ? (size_t)(dot - t) : strlen(t)));
    *swz = enc_swz(h, dot ? dot + 1 : "");
    return v;
}

/* parse_fp: '#0x...' = raw bits, else fr.encode_real(Fraction(text), FP24): RNE, FTZ, overflow -> Inf */
static uint32_t encode_fp24(Shc *h, const BI *n0, const BI *d0)
{
    enum { E = 7, M = 16, EMIN = 1 - 63, EMAX = 63, INF_EXP = 127 };
    BI n = *n0, d = *d0;
    int s = n0->neg != d0->neg;
    long long e = 0;
    BI *q, *r, *a, *b;
    uint64_t fl;
    uint32_t sbit;
    if (bi_is_zero(n0)) return 0;
    sbit = (uint32_t)s << (E + M);
    n.neg = d.neg = 0;
    a = &n; b = &d;
    /* a / b in [1, 2) after scaling by powers of two, as the loops of encode_real */
    while (bi_cmp(a, bi_shl(h, b, 1)) >= 0) { b = bi_shl(h, b, 1); e++; if (e > EMAX + 2) return sbit | ((uint32_t)INF_EXP << M); }
    while (bi_cmp(a, b) < 0) { a = bi_shl(h, a, 1); e--; if (e < EMIN - 2) return sbit; }
    if (e < EMIN) return sbit;
    bi_divmod(h, bi_shl(h, a, M), b, &q, &r);
    if (!bi_fits_u64(q, &fl)) sh_py_error(h, "internal: encode_real");
    {
        BI *twice = bi_shl(h, r, 1);
        int c = bi_cmp(twice, b);
        if (c > 0 || (c == 0 && (fl & 1))) fl++;
    }
    if (fl == ((uint64_t)1 << (M + 1))) { fl >>= 1; e++; }
    if (e > EMAX) return sbit | ((uint32_t)INF_EXP << M);
    return sbit | ((uint32_t)(e + 63) << M) | (uint32_t)(fl & ((1u << M) - 1));
}

static int digits_us(const char *p, const char *e, const char **out)   /* \d+(_\d+)* */
{
    const char *q = p;
    while (q < e && sh_is_digit((unsigned char)*q)) q++;
    if (q == p) return 0;
    for (;;) {
        const char *r = q;
        if (r < e && *r == '_') {
            const char *s = r + 1;
            while (s < e && sh_is_digit((unsigned char)*s)) s++;
            if (s > r + 1) { q = s; continue; }
        }
        break;
    }
    *out = q;
    return 1;
}

static uint32_t parse_fp(Shc *h, const char *tok)
{
    const char *t = tok, *p, *e;
    int neg = 0;
    BI *num, *den;
    while (*t == '#') t++;
    if (t[0] == '0' && (t[1] == 'x' || t[1] == 'X')) {
        const char *q = t + 2;
        uint32_t v = 0;
        int nd = 0, c;
        for (; *q; q++) {
            c = lower((unsigned char)*q);
            if (c == '_') continue;
            if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) sh_py_error(h, "ValueError: bad hex literal");
            v = (v << 4) | (uint32_t)(c <= '9' ? c - '0' : c - 'a' + 10);
            nd++;
        }
        if (!nd) sh_py_error(h, "ValueError: bad hex literal");
        return v & 0xFFFFFF;
    }
    /* Fraction(text) */
    p = t; e = t + strlen(t);
    sh_strip(&p, &e);
    if (p < e && (*p == '-' || *p == '+')) neg = *p++ == '-';
    {
        const char *nb = p, *ne = p, *fb = NULL, *fe = NULL, *ex = NULL, *q;
        long long exp10 = 0;
        if (!(p < e && (sh_is_digit((unsigned char)*p) || (*p == '.' && p + 1 < e && sh_is_digit((unsigned char)p[1]))))) sh_py_error(h, "ValueError: bad fraction");
        if (sh_is_digit((unsigned char)*p)) digits_us(p, e, &ne);
        q = ne;
        den = bi_from_u64(h, 1);
        if (q < e && *q == '/') {
            const char *de;
            q++;
            if (!digits_us(q, e, &de) || de != e) sh_py_error(h, "ValueError: bad fraction");
            den = bi_from_digits(h, q, (size_t)(de - q), 10);
            if (bi_is_zero(den)) sh_py_error(h, "ZeroDivisionError");
            q = e;
        } else {
            if (q < e && *q == '.') {
                q++; fb = q; fe = q;
                if (q < e && sh_is_digit((unsigned char)*q)) digits_us(q, e, &fe);
                q = fe;
            }
            if (q < e && (*q == 'e' || *q == 'E')) {
                const char *eb;
                int eneg = 0;
                ex = q + 1;
                if (ex < e && (*ex == '-' || *ex == '+')) eneg = *ex++ == '-';
                if (!digits_us(ex, e, &eb) || eb != e) sh_py_error(h, "ValueError: bad fraction");
                for (; ex < e; ex++) if (*ex != '_') { if (exp10 < 100000) exp10 = exp10 * 10 + (*ex - '0'); }
                if (eneg) exp10 = -exp10;
                q = e;
            }
        }
        if (q != e) sh_py_error(h, "ValueError: bad fraction");
        num = ne > nb ? bi_from_digits(h, nb, (size_t)(ne - nb), 10) : bi_from_u64(h, 0);
        if (fb && fe > fb) {
            size_t nf = 0;
            const char *c;
            for (c = fb; c < fe; c++) if (*c != '_') nf++;
            num = bi_add(h, bi_mul(h, num, bi_pow(h, bi_from_u64(h, 10), nf)), bi_from_digits(h, fb, (size_t)(fe - fb), 10));
            exp10 -= (long long)nf;
        }
        if (exp10 >= 0) num = bi_mul(h, num, bi_pow(h, bi_from_u64(h, 10), (uint64_t)exp10));
        else den = bi_mul(h, den, bi_pow(h, bi_from_u64(h, 10), (uint64_t)-exp10));
    }
    if (neg) num = bi_neg(h, num);
    return encode_fp24(h, num, den);
}

/* decode + check_overlap: no source component may be one its own instruction's destination has already written */
static const char *opname(int op)
{
    const OpDef *t;
    for (t = OPS; t->name; t++)
        if (t->op == op && strcmp(t->name, "STOZ") && strcmp(t->name, "BRP")) return t->name;
    return "?";
}

static void check_overlap(Shc *h, uint64_t w, const char *ln)
{
    const char *name = opname((int)((w >> 57) & 0x7F));
    int wm = (int)((w >> 53) & 0xF), rd = (int)((w >> 48) & 0x1F), nsrc, c, s, written = 0;
    int reg[3], swz[3];
    if (!(in_list(ARITH2, name) || in_list(ARITH1, name) || in_list(INT2, name) || !strcmp(name, "MAD") || !strcmp(name, "SEL") ||
          !strcmp(name, "SET") || !strcmp(name, "LDPX"))) return;       /* name not in WRITES, or the LDP / LDV / TEX / MOVI group */
    if (!strcmp(name, "DP3") || !strcmp(name, "DP4")) return;
    nsrc = (!strcmp(name, "MAD") || !strcmp(name, "SEL")) ? 3 : in_list(ARITH1, name) ? 1 : 2;
    reg[0] = (int)((w >> 43) & 0x1F); swz[0] = (int)((w >> 35) & 0xFF);
    reg[1] = (int)((w >> 28) & 0x1F); swz[1] = (int)((w >> 20) & 0xFF);
    reg[2] = (int)((w >> 13) & 0x1F); swz[2] = (int)((w >> 5) & 0xFF);
    if (!strcmp(name, "LDPX")) { nsrc = 1; reg[0] = reg[1]; swz[0] = swz[1]; }      /* the index register (rs1), read per micro-op */
    for (c = 0; c < 4; c++) {
        if (!((wm >> c) & 1)) continue;
        for (s = 0; s < nsrc; s++) {
            if (reg[s] == rd && ((written >> ((swz[s] >> (2 * c)) & 3)) & 1))
                sh_py_error(h, "dest/source overlap (component %c written before it is read for .%c): %s",
                            "xyzw"[(swz[s] >> (2 * c)) & 3], "xyzw"[c], ln);
        }
        written |= 1 << c;
    }
}

/* ---------------- assemble ---------------- */
static int is_sep(const char *s, const char *e)                  /* [\s,] */
{
    if (s < e && *s == ',') return 1;
    return sh_ws_len(s, e);
}

static const char *arg_(Shc *h, char **args, int na, int i)
{
    if (i >= na) sh_py_error(h, "IndexError: missing operand");
    return args[i];
}

uint64_t *sh_assemble(Shc *h, const char *sasm, int vs, int max_if, int max_loop, int *nwords)
{
    Dict labels;
    int nprog = 0, cap = 256, pc, ifd = 0, lpd = 0;
    char **prog = (char **)sh_alloc(h, (size_t)cap * sizeof(char *));
    const char *p = sasm;
    uint64_t *words;
    Src z0;
    z0.reg = 0; z0.swz = SWZ_ID; z0.mod = 0;
    memset(&labels, 0, sizeof labels);
    for (;;) {                                                   /* the label pass */
        const char *le = p, *b, *e, *q;
        while (*le && *le != '\n' && *le != '\r') le++;
        b = p; e = le;
        /* ln.split('//')[0].split(';')[0].strip() */
        for (q = b; q < e; q++) {
            if (*q == ';' || (*q == '/' && q + 1 < e && q[1] == '/')) { e = q; break; }
        }
        sh_strip(&b, &e);
        if (e > b) {
            if (e[-1] == ':') {
                dict_put(h, &labels, sh_strndup(h, b, (size_t)(e - b - 1)))->i = nprog;
            } else {
                if (nprog == cap) {
                    char **np = (char **)sh_alloc(h, (size_t)cap * 2 * sizeof(char *));
                    memcpy(np, prog, (size_t)nprog * sizeof(char *)); prog = np; cap *= 2;
                }
                prog[nprog++] = sh_strndup(h, b, (size_t)(e - b));
            }
        }
        if (!*le) break;
        p = le + 1;
        if (*le == '\r' && *p == '\n') p++;
    }
    words = (uint64_t *)sh_alloc(h, ((size_t)nprog + 1) * sizeof(uint64_t));
    for (pc = 0; pc < nprog; pc++) {
        const char *ln = prog[pc], *e = ln + strlen(ln), *s = ln, *mdot;
        char *t[8], *mn, *msuf;
        char **args = t;
        int nt = 0, na, i, op, wm_mn = -1;
        uint64_t w = 0;
        ShMark mk = sh_mark(h);
        Src nosrc;
        nosrc = z0;
        /* re.split(r'[\s,]+', ln.strip()), the empty strings dropped from the arguments */
        {
            int ntcap = 8;
            char **tt = t;
            for (;;) {
                const char *b = s;
                while (s < e && !is_sep(s, e)) s++;
                if (nt == ntcap) {
                    char **g = (char **)sh_alloc(h, (size_t)ntcap * 2 * sizeof(char *));
                    memcpy(g, tt, (size_t)nt * sizeof(char *)); tt = g; ntcap *= 2;
                }
                if (nt == 0 || s > b) tt[nt++] = sh_strndup(h, b, (size_t)(s - b));
                if (s >= e) break;
                {
                    int k;
                    while ((k = is_sep(s, e)) != 0) s += k;
                }
                if (s >= e) { break; }
            }
            args = tt + 1;
            na = nt - 1;
            mn = tt[0];
        }
        /* mn_full = t[0].upper(); mn, _, msuf = mn_full.partition('.') */
        for (i = 0; mn[i]; i++) if (mn[i] >= 'a' && mn[i] <= 'z') mn[i] = (char)(mn[i] - 32);
        mdot = strchr(mn, '.');
        msuf = NULL;
        if (mdot) { msuf = (char *)mdot + 1; *(char *)mdot = 0; }
        if (msuf && *msuf) wm_mn = maskbits(h, msuf);
        if (!strcmp(mn, "DIV")) {                               /* DIV rd, y, x = y / x: RCPM with its operands in the core's order */
            char *sw;
            if (na < 3) sh_py_error(h, "DIV rd, y, x: %s", ln);
            mn = "RCPM"; sw = args[1]; args[1] = args[2]; args[2] = sw;
        }
        if (lookup(IOP_FN, mn) >= 0) op = lookup(OPS, "IOP");
        else if (vs && in_list(FS_ONLY, mn)) sh_py_error(h, "%s in a vertex program", mn);
        else if (vs && !strcmp(mn, "BRP")) sh_py_error(h, "BRP in a vertex program: the 16-lane vertex core has no scalar branch, use IF / ELSE");
        else if (!vs && (!strcmp(mn, "BRP") || !strcmp(mn, "STV"))) sh_py_error(h, "%s in a fragment program", mn);
        else if ((op = lookup(OPS, mn)) < 0) sh_py_error(h, "bad op %s", mn);

#define DST(i_, rd_, wm_) do { \
            int m_; \
            (rd_) = parse_dst(h, arg_(h, args, na, (i_)), &m_); \
            (wm_) = m_ >= 0 ? m_ : (wm_mn >= 0 ? wm_mn : 0xF); \
            if ((wm_) == 0) sh_py_error(h, "empty write mask in: %s", ln); \
        } while (0)
        if (in_list(ARITH2, mn) || in_list(INT2, mn)) {
            int rd, wm;
            Src s0, s1;
            DST(0, rd, wm);
            s0 = parse_src(h, arg_(h, args, na, 1)); s1 = parse_src(h, arg_(h, args, na, 2));
            if (in_list(INT2, mn) && (s0.mod || s1.mod)) sh_py_error(h, "no modifiers on integer ops: %s", ln);
            w = encode(op, wm, rd, s0, s1, nosrc, 0, 0);
        }
        else if (lookup(IOP_FN, mn) >= 0) {
            int rd, wm;
            Src s0, s1 = nosrc;
            DST(0, rd, wm);
            s0 = parse_src(h, arg_(h, args, na, 1));
            if (na > 2) s1 = parse_src(h, args[2]);
            w = encode(op, wm, rd, s0, s1, nosrc, 0, 0) | ((uint64_t)lookup(IOP_FN, mn) << 13);
        }
        else if (!strcmp(mn, "MAD") || !strcmp(mn, "CLAMP")) {     /* isa64 ARITH3 */
            int rd, wm;
            Src s0, s1, s2;
            DST(0, rd, wm);
            s0 = parse_src(h, arg_(h, args, na, 1)); s1 = parse_src(h, arg_(h, args, na, 2)); s2 = parse_src(h, arg_(h, args, na, 3));
            w = encode(op, wm, rd, s0, s1, s2, 0, 0);
        }
        else if (in_list(ARITH1, mn)) {
            int rd, wm;
            DST(0, rd, wm);
            w = encode(op, wm, rd, parse_src(h, arg_(h, args, na, 1)), nosrc, nosrc, 0, 0);
        }
        else if (!strcmp(mn, "SEL")) {
            int rd, wm;
            Src ss[3];
            DST(0, rd, wm);
            for (i = 0; i < 3; i++) ss[i] = parse_src(h, arg_(h, args, na, i + 1));
            if (ss[0].mod || ss[1].mod || ss[2].mod) sh_py_error(h, "no modifiers on SEL: %s", ln);
            w = encode(op, wm, rd, ss[0], ss[1], ss[2], 0, 0);
        }
        else if (!strcmp(mn, "MOVI")) {
            int rd, wm;
            Src z; z.reg = 0; z.swz = 0; z.mod = 0;
            DST(0, rd, wm);
            w = encode(op, wm, rd, nosrc, z, z, 0, 0) | ((uint64_t)parse_fp(h, arg_(h, args, na, 1)) << 4);
        }
        else if (!strcmp(mn, "TEX")) {
            int rd, wm, mode = 0, cm = 0, a;
            long long samp;
            const char *st;
            Src s0;
            DST(0, rd, wm);
            for (a = 3; a < na; a++) {
                char *u = sh_strdup(h, args[a]);
                for (i = 0; u[i]; i++) if (u[i] >= 'a' && u[i] <= 'z') u[i] = (char)(u[i] - 32);
                if (!strcmp(u, "BIAS")) mode = 1;
                else if (!strcmp(u, "LOD")) mode = 2;
                else if (args[a][0] == '.') cm = maskbits(h, args[a] + 1);
                else sh_py_error(h, "bad TEX option in: %s", ln);
            }
            st = arg_(h, args, na, 2);
            while (*st == '#') st++;
            samp = py_int0(h, st);
            if (samp < 0 || samp >= 16) sh_py_error(h, "bad sampler in: %s", ln);
            s0 = parse_src(h, arg_(h, args, na, 1));
            w = encode(op, wm, rd, s0, nosrc, nosrc, (uint64_t)((cm << 6) | (mode << 4) | (int)samp), 10);
        }
        else if (!strcmp(mn, "STOZ")) {
            w = encode(op, 0, 0, parse_src(h, arg_(h, args, na, 0)), nosrc, nosrc, 0, 0);
        }
        else if (!strcmp(mn, "SETP") || !strcmp(mn, "SET")) {
            int setp = !strcmp(mn, "SETP"), rd = 0, wm = 0, cc;
            char *u = sh_strdup(h, arg_(h, args, na, 0));
            Src s0, s1;
            for (i = 0; u[i]; i++) if (u[i] >= 'a' && u[i] <= 'z') u[i] = (char)(u[i] - 32);
            if (!setp) DST(1, rd, wm);
            s0 = parse_src(h, arg_(h, args, na, setp ? 1 : 2));
            s1 = parse_src(h, arg_(h, args, na, setp ? 2 : 3));
            if (!strcmp(u, "SLT")) cc = 2;
            else if (!strcmp(u, "SGE")) cc = 3;
            else if (!strcmp(u, "SEQ")) cc = 4;
            else if (!strcmp(u, "SNE")) cc = 5;
            else sh_py_error(h, "KeyError: bad compare");
            w = encode(op, wm, rd, s0, s1, nosrc, 0, 0) | ((uint64_t)cc << 13);
        }
        else if (!strcmp(mn, "LDP") || !strcmp(mn, "LDV")) {
            int rd, wm, swz, ldp = !strcmp(mn, "LDP");
            long long idx;
            Src s0;
            DST(0, rd, wm);
            idx = parse_idx(h, arg_(h, args, na, 1), &swz);
            if ((ldp && !(idx >= 0 && idx < (vs ? 256 : NPARAM))) || (!ldp && !(idx >= 0 && idx < NVARY)))
                sh_py_error(h, "index out of range in: %s", ln);
            s0.reg = 0; s0.swz = swz; s0.mod = 0;
            w = encode(op, wm, rd, s0, nosrc, nosrc, (uint64_t)idx, 8);
        }
        else if (!strcmp(mn, "LDPX")) {                         /* rd.c = params[(idx + rN[swz1[c]]) mod 256][swz0[c]] per lane; vertex programs only */
            int rd, wm, swz;
            long long idx;
            Src s0, ix;
            if (!vs) sh_py_error(h, "LDPX in a fragment program: %s", ln);
            DST(0, rd, wm);
            idx = parse_idx(h, arg_(h, args, na, 1), &swz);
            ix = parse_src(h, arg_(h, args, na, 2));
            if (!(idx >= 0 && idx < 256)) sh_py_error(h, "index out of range in: %s", ln);
            if (ix.mod) sh_py_error(h, "no modifier on LDPX's index register: %s", ln);
            s0.reg = 0; s0.swz = swz; s0.mod = 0;
            w = encode(op, wm, rd, s0, ix, nosrc, (uint64_t)idx, 8);
        }
        else if (!strcmp(mn, "STV") || !strcmp(mn, "STO")) {
            long long outidx = 0;
            const char *src;
            int stv = !strcmp(mn, "STV");
            if (na == 2) { outidx = py_int0(h, args[0]); src = args[1]; }
            else src = arg_(h, args, na, 0);
            if ((stv && !(outidx >= 0 && outidx < NVARY)) || (!stv && !(outidx >= 0 && outidx <= (vs ? 1 : 0))))
                sh_py_error(h, "output index out of range in: %s", ln);
            w = encode(op, 0, 0, parse_src(h, src), nosrc, nosrc, (uint64_t)outidx, 8);
        }
        else if (in_list(CTRL_IMM, mn)) {
            const char *a0 = arg_(h, args, na, 0);
            DictEnt *lb = dict_find(&labels, a0);
            long long tgt = lb ? lb->i : py_int0(h, a0);
            w = encode(op, 0, 0, nosrc, nosrc, nosrc, (uint64_t)tgt, 16);
            if (!strcmp(mn, "BRP") && na > 1 && (args[1][0] == 'F' || args[1][0] == 'f') && !args[1][1]) w |= (uint64_t)1 << 52;
            if (!strcmp(mn, "IF")) ifd++;
            else if (!strcmp(mn, "LOOP")) lpd++;
            if (max_if >= 0 && ifd > max_if) sh_py_error(h, "IF nesting deeper than %d", max_if);
            if (max_loop >= 0 && lpd > max_loop) sh_py_error(h, "LOOP nesting deeper than %d", max_loop);
        }
        else if (in_list(CTRL0, mn)) {
            w = encode(op, 0, 0, nosrc, nosrc, nosrc, 0, 0);
            if (!strcmp(mn, "ENDIF")) ifd--;
            else if (!strcmp(mn, "ENDL")) lpd--;
        }
        else sh_py_error(h, "bad op %s", mn);
#undef DST
        check_overlap(h, w, ln);
        words[pc] = w;
        sh_release(h, mk);
    }
    *nwords = nprog;
    return words;
}
