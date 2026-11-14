/* a user's vertex program on the PC, one vertex at a time. The fixed pipeline's position is a matter of matrices; a shader's is known only by running it, and some of what this library does on the PC needs it: the
 * facing of a polygon drawn as lines, a stippled line's distance in pixels, the corners of an antialiased line, selection and feedback. This runs the compiled text the board runs, in doubles: positions to compare and
 * to measure, not pixels. Control flow as isa64.simulate_vs2 on one lane; values are numbers, an int is a whole number in its double. */
#include "gl_internal.h"

enum { O_MOV, O_ADD, O_SUB, O_MUL, O_MAD, O_DP3, O_DP4, O_MIN, O_MAX, O_RCP, O_RSQRT, O_LOG2, O_EXP2, O_SIN, O_COS, O_FTOI, O_ITOF, O_FTOU, O_UTOF, O_TRUNC, O_FLR, O_CEIL, O_RNDE, O_SEL, O_SET, O_SETP,
       O_LDP, O_LDPX, O_LDV, O_MOVI, O_STO, O_STV, O_IADD, O_ISUB, O_IMUL, O_IDIV, O_IMOD, O_UDIV, O_UMOD, O_ISLT, O_ISGE, O_ISEQ, O_ISNE, O_IAND, O_IOR, O_IXOR, O_INOT, O_ISHL, O_ISHR, O_USHR, O_IMIN, O_IMAX, O_IABS, O_INEG, O_ISIGN,
       O_IF, O_ELSE, O_ENDIF, O_LOOP, O_BREAK, O_CONT, O_ENDL, O_BRA, O_NOP, O_END };

static const struct { const char *n; int op; } NAMES[] = {
    { "MOV", O_MOV }, { "ADD", O_ADD }, { "SUB", O_SUB }, { "MUL", O_MUL }, { "MAD", O_MAD }, { "DP3", O_DP3 }, { "DP4", O_DP4 }, { "MIN", O_MIN }, { "MAX", O_MAX }, { "RCP", O_RCP }, { "RSQRT", O_RSQRT }, { "LOG2", O_LOG2 },
    { "EXP2", O_EXP2 }, { "SIN", O_SIN }, { "COS", O_COS }, { "FTOI", O_FTOI }, { "ITOF", O_ITOF }, { "FTOU", O_FTOU }, { "UTOF", O_UTOF }, { "TRUNC", O_TRUNC }, { "FLR", O_FLR }, { "CEIL", O_CEIL }, { "RNDE", O_RNDE },
    { "SEL", O_SEL }, { "SET", O_SET }, { "SETP", O_SETP }, { "LDP", O_LDP }, { "LDPX", O_LDPX }, { "LDV", O_LDV }, { "MOVI", O_MOVI }, { "STO", O_STO }, { "STV", O_STV }, { "IADD", O_IADD }, { "ISUB", O_ISUB }, { "IMUL", O_IMUL },
    { "IDIV", O_IDIV }, { "IMOD", O_IMOD }, { "UDIV", O_UDIV }, { "UMOD", O_UMOD }, { "ISLT", O_ISLT }, { "ISGE", O_ISGE }, { "ISEQ", O_ISEQ }, { "ISNE", O_ISNE }, { "IAND", O_IAND }, { "IOR", O_IOR }, { "IXOR", O_IXOR },
    { "INOT", O_INOT }, { "ISHL", O_ISHL }, { "ISHR", O_ISHR }, { "USHR", O_USHR }, { "IMIN", O_IMIN }, { "IMAX", O_IMAX }, { "UMIN", O_IMIN }, { "UMAX", O_IMAX }, { "ULT", O_ISLT }, { "UGE", O_ISGE },
    { "IABS", O_IABS }, { "INEG", O_INEG }, { "ISIGN", O_ISIGN }, { "IF", O_IF }, { "ELSE", O_ELSE }, { "ENDIF", O_ENDIF }, { "LOOP", O_LOOP }, { "BREAK", O_BREAK },
    { "CONT", O_CONT }, { "ENDL", O_ENDL }, { "BRA", O_BRA }, { "NOP", O_NOP }, { "END", O_END },
};

typedef struct vsrc { uint8_t r, swz[4], mod; } vsrc;             /* mod: 1 negate, 2 absolute */
typedef struct vop { uint8_t op, wm, rd, cc, nsrc; vsrc s[3]; int idx, target; uint8_t iswz[4]; double imm; } vop;
typedef struct vprog { vop *ops; int n, bad; } vprog;

static int comp_of(char c) { return c == 'x' ? 0 : c == 'y' ? 1 : c == 'z' ? 2 : c == 'w' ? 3 : -1; }

static void swz_of(const char *s, uint8_t *o)
{
    int i, n = (int)strlen(s);
    for (i = 0; i < 4; i++) { int c = n ? comp_of(s[i < n ? i : n - 1]) : i; o[i] = (uint8_t)(c < 0 ? i : c); }
}

/* -r1.xy, !r2, -!r3 */
static void parse_src(const char *t, vsrc *o)
{
    o->mod = 0;
    while (*t == '-' || *t == '!') { o->mod |= *t == '-' ? 1 : 2; t++; }
    if (*t == 'r') t++;
    o->r = (uint8_t)atoi(t);
    t = strchr(t, '.');
    swz_of(t ? t + 1 : "", o->swz);
}

static int mask_of(const char *s) { int m = 0; for (; *s; s++) { int c = comp_of(*s); if (c >= 0) m |= 1 << c; } return m; }

static int parse_dst(const char *t, vop *o)
{
    const char *d;
    if (*t == 'r') t++;
    o->rd = (uint8_t)atoi(t);
    d = strchr(t, '.');
    if (d) o->wm = (uint8_t)mask_of(d + 1);
    return 1;
}

static int cc_of(const char *t) { static const char *const N[4] = { "SEQ", "SNE", "SLT", "SGE" }; int i; for (i = 0; i < 4; i++) if (!strcmp(t, N[i])) return i; return 1; }

/* the compiled text to operations; labels at the next operation's number */
static vprog *parse(const char *sasm)
{
    vprog *p = (vprog *)calloc(1, sizeof *p);
    typedef struct lab { char n[24]; int pc; } lab;
    lab *lb = NULL;
    char **tg = NULL;
    int nl = 0, cap = 0, pass;
    if (!p) return NULL;
    for (pass = 0; pass < 2; pass++) {
        const char *s = sasm;
        int pc = 0;
        while (*s) {
            char line[160], *t[6], *q;
            const char *e = strchr(s, '\n');
            size_t len = e ? (size_t)(e - s) : strlen(s);
            int nt = 0, i, op = -1;
            if (len >= sizeof line) len = sizeof line - 1;
            memcpy(line, s, len); line[len] = 0;
            s = e ? e + 1 : s + strlen(s);
            for (q = strtok(line, " ,\t"); q && nt < 6; q = strtok(NULL, " ,\t")) t[nt++] = q;
            if (!nt) continue;
            if (t[0][strlen(t[0]) - 1] == ':') {
                if (!pass) { lb = (lab *)realloc(lb, ((size_t)nl + 1) * sizeof *lb); snprintf(lb[nl].n, sizeof lb[nl].n, "%.*s", (int)strlen(t[0]) - 1, t[0]); lb[nl].pc = pc; nl++; }
                continue;
            }
            if (pass) {
                vop *o = &p->ops[pc];
                char mn[16], *dot;
                int a = 1;
                snprintf(mn, sizeof mn, "%s", t[0]);
                dot = strchr(mn, '.');
                o->wm = 15;
                if (dot) { o->wm = (uint8_t)mask_of(dot + 1); *dot = 0; }
                for (i = 0; i < (int)(sizeof NAMES / sizeof NAMES[0]); i++) if (!strcmp(NAMES[i].n, mn)) op = NAMES[i].op;
                if (op < 0) { p->bad = 1; g2e_log("gl: a vertex program the PC cannot run: %s", t[0]); op = O_NOP; }
                o->op = (uint8_t)op;
                switch (op) {
                case O_IF: case O_ELSE: case O_BRA: case O_LOOP:
                    if (nt > 1) {
                        o->target = isdigit((unsigned char)t[1][0]) ? atoi(t[1]) : -1;
                        for (i = 0; i < nl && o->target < 0; i++) if (!strcmp(lb[i].n, t[1])) o->target = lb[i].pc;
                    }
                    break;
                case O_ENDIF: case O_BREAK: case O_CONT: case O_ENDL: case O_NOP: case O_END: break;
                case O_SETP: o->cc = (uint8_t)cc_of(t[1]); parse_src(t[2], &o->s[0]); parse_src(t[3], &o->s[1]); break;
                case O_SET: o->cc = (uint8_t)cc_of(t[1]); parse_dst(t[2], o); parse_src(t[3], &o->s[0]); parse_src(t[4], &o->s[1]); break;
                case O_LDP: case O_LDV: case O_LDPX: {
                    char *d2;
                    parse_dst(t[1], o);
                    o->idx = atoi(t[2]);
                    d2 = strchr(t[2], '.');
                    swz_of(d2 ? d2 + 1 : "", o->s[0].swz);
                    if (op == O_LDPX) { char *d3 = strchr(t[3], '.'); o->s[1].r = (uint8_t)atoi(t[3] + 1); swz_of(d3 ? d3 + 1 : "", o->iswz); }
                    break;
                }
                case O_MOVI: parse_dst(t[1], o); o->imm = atof(t[2]); break;
                case O_STO: case O_STV: o->idx = atoi(t[1]); parse_src(t[2], &o->s[0]); break;
                default:
                    parse_dst(t[a++], o);
                    for (i = 0; a < nt && i < 3; i++) parse_src(t[a++], &o->s[i]);
                    o->nsrc = (uint8_t)i;
                    break;
                }
            }
            pc++;
        }
        if (!pass) { cap = pc; p->ops = (vop *)calloc((size_t)cap + 1, sizeof *p->ops); p->n = pc; }
    }
    free(lb); free(tg);
    return p;
}

static double fetch(const vsrc *s, double (*g)[4], int c)
{
    double v = g[s->r][s->swz[c]];
    if (s->mod & 2) v = fabs(v);
    if (s->mod & 1) v = -v;
    return v;
}

static int cmp(int cc, double a, double b)
{
    switch (cc) {
    case 0: return a == b;
    case 1: return a != b;
    case 2: return a < b;
    default: return a >= b;
    }
}

static double one(int op, double a, double b, double c)
{
    switch (op) {
    case O_MOV: case O_ITOF: case O_UTOF: return a;
    case O_ADD: return a + b;
    case O_SUB: return a - b;
    case O_MUL: return a * b;
    case O_MAD: return a * b + c;
    case O_MIN: return a < b ? a : b;
    case O_MAX: return a > b ? a : b;
    case O_RCP: return 1.0 / a;
    case O_RSQRT: return 1.0 / sqrt(a);
    case O_LOG2: return log2(a);
    case O_EXP2: return pow(2.0, a);
    case O_SIN: return sin(a);
    case O_COS: return cos(a);
    case O_FTOI: case O_FTOU: case O_TRUNC: return trunc(a);
    case O_FLR: return floor(a);
    case O_CEIL: return ceil(a);
    case O_RNDE: return rint(a);
    case O_SEL: return a != 0.0 ? b : c;
    case O_IADD: return a + b;
    case O_ISUB: return a - b;
    case O_IMUL: return a * b;
    case O_IDIV: case O_UDIV: return b != 0.0 ? trunc(a / b) : 0.0;
    case O_IMOD: case O_UMOD: return b != 0.0 ? fmod(a, b) : 0.0;
    case O_ISLT: return a < b;
    case O_ISGE: return a >= b;
    case O_ISEQ: return a == b;
    case O_ISNE: return a != b;
    case O_IAND: return (double)((int)a & (int)b);
    case O_IOR: return (double)((int)a | (int)b);
    case O_IXOR: return (double)((int)a ^ (int)b);
    case O_INOT: return (double)~(int)a;
    case O_ISHL: return (double)((int)a << ((int)b & 31));
    case O_ISHR: return (double)((int)a >> ((int)b & 31));
    case O_USHR: return (double)((unsigned)(int)a >> ((int)b & 31));
    case O_IMIN: return a < b ? a : b;
    case O_IMAX: return a > b ? a : b;
    case O_IABS: return fabs(a);
    case O_INEG: return -a;
    case O_ISIGN: return a > 0.0 ? 1.0 : a < 0.0 ? -1.0 : 0.0;
    default: return 0.0;
    }
}

/* attr: by the program's attribute number; par: 256 vec4; o: position, then the varyings */
static void run(const vprog *p, const double (*attr)[4], const double (*par)[4], double *pos, double (*vary)[4])
{
    double g[32][4];
    int ms_e[8], ms_t[8], ls_cnt[4], ls_top[4], ls_ent[4], ls_live[4], ls_cont[4], nm = 0, nl = 0, active = 1, pred = 0, pc = 0, steps = 0, c;
    memset(g, 0, sizeof g);
    while (pc < p->n && steps++ < 200000) {
        const vop *o = &p->ops[pc];
        int next = pc + 1;
        double (*dg)[4] = g;
        switch (o->op) {
        case O_END: return;
        case O_NOP: break;
        case O_DP3: case O_DP4: {
            int n = o->op == O_DP3 ? 3 : 4;
            double s = 0.0;
            if (active) {
                for (c = 0; c < n; c++) s += fetch(&o->s[0], g, c) * fetch(&o->s[1], g, c);
                for (c = 0; c < 4; c++) if (o->wm >> c & 1) dg[o->rd][c] = s;
            }
            break;
        }
        case O_LDP:
            if (active) for (c = 0; c < 4; c++) if (o->wm >> c & 1) dg[o->rd][c] = par[o->idx & 255][o->s[0].swz[c]];
            break;
        case O_LDPX:
            if (active) for (c = 0; c < 4; c++) if (o->wm >> c & 1) dg[o->rd][c] = par[(o->idx + (int)g[o->s[1].r][o->iswz[c]]) & 255][o->s[0].swz[c]];
            break;
        case O_LDV:
            if (active) for (c = 0; c < 4; c++) if (o->wm >> c & 1) dg[o->rd][c] = attr[o->idx & 15][o->s[0].swz[c]];
            break;
        case O_MOVI:
            if (active) for (c = 0; c < 4; c++) if (o->wm >> c & 1) dg[o->rd][c] = o->imm;
            break;
        case O_STO:
            if (active && o->idx == 0) for (c = 0; c < 4; c++) pos[c] = fetch(&o->s[0], g, c);
            break;
        case O_STV:
            if (active && vary && o->idx < 16) for (c = 0; c < 4; c++) vary[o->idx][c] = fetch(&o->s[0], g, c);
            break;
        case O_SET:
            if (active) for (c = 0; c < 4; c++) if (o->wm >> c & 1) dg[o->rd][c] = cmp(o->cc, fetch(&o->s[0], g, c), fetch(&o->s[1], g, c)) ? 1.0 : 0.0;
            break;
        case O_SETP: pred = cmp(o->cc, fetch(&o->s[0], g, 0), fetch(&o->s[1], g, 0)); break;
        case O_IF:
            ms_e[nm] = active; ms_t[nm] = active && pred; nm++;
            active = active && pred;
            if (!active) next = o->target;
            break;
        case O_ELSE:
            active = ms_e[nm - 1] && !ms_t[nm - 1] && (nl ? ls_live[nl - 1] && !ls_cont[nl - 1] : 1);
            if (!active) next = o->target;
            break;
        case O_ENDIF:
            nm--;
            active = ms_e[nm] && (nl ? ls_live[nl - 1] && !ls_cont[nl - 1] : 1);
            break;
        case O_LOOP:
            ls_cnt[nl] = o->target; ls_top[nl] = pc + 1; ls_ent[nl] = active; ls_live[nl] = active; ls_cont[nl] = 0; nl++;
            break;
        case O_BREAK: if (active && pred) { ls_live[nl - 1] = 0; active = 0; } break;
        case O_CONT: if (active && pred) { ls_cont[nl - 1] = 1; active = 0; } break;
        case O_ENDL:
            if (--ls_cnt[nl - 1] > 0 && ls_live[nl - 1]) { active = 1; ls_cont[nl - 1] = 0; next = ls_top[nl - 1]; }
            else { active = ls_ent[nl - 1]; nl--; }
            break;
        case O_BRA: next = o->target; break;
        default: {
            double r[4];
            int w = o->wm;
            if (!active) break;
            for (c = 0; c < 4; c++) if (w >> c & 1) r[c] = one(o->op, fetch(&o->s[0], g, c), o->nsrc > 1 ? fetch(&o->s[1], g, c) : 0.0, o->nsrc > 2 ? fetch(&o->s[2], g, c) : 0.0);
            for (c = 0; c < 4; c++) if (w >> c & 1) dg[o->rd][c] = r[c];
            break;
        }
        }
        pc = next;
    }
}

void *gl_vs_parse(const char *sasm) { return parse(sasm); }
void gl_vs_free(void *p) { vprog *v = (vprog *)p; if (v) { free(v->ops); free(v); } }
void gl_vs_exec(const void *p, const double (*attr)[4], const double (*par)[4], double *pos, double (*vary)[4]) { run((const vprog *)p, attr, par, pos, vary); }
