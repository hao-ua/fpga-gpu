/* shc_internal.h — shared by the host/win/shc/ sources; nothing here is part of the API (g2_shc.h is). */
#ifndef SHC_INTERNAL_H
#define SHC_INTERNAL_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <setjmp.h>
#include "g2_shc.h"

#define SHC_NORETURN __attribute__((noreturn))

/* ---------------- one compile attempt: arena, error exit, Python's recursion limit ---------------- */
/* The recursion count stands in for sys.getrecursionlimit() (1000 frames, a few of them taken by the caller): a shader
 * that recurses as deep as RecursionError in Python fails here too, and the C stack stays bounded. */
#define SHC_DEPTH_LIMIT 990
#define SHC_ARENA_LIMIT ((size_t)1 << 30)
#define IF_DEPTH_DEFAULT 8              /* isa64.IF_DEPTH, LOOP_DEPTH */
#define LOOP_DEPTH_DEFAULT 4

typedef struct ShChunk { struct ShChunk *next; size_t cap, used; } ShChunk;
typedef struct ShMark { ShChunk *chunk; size_t used, total; } ShMark;

typedef struct Shc {
    jmp_buf jb;
    char err[512];
    int compile_error;          /* the failure was a CompileError (else: any other Python exception, or a C limit) */
    const char *const *vary_order; int nvary_order;   /* g2sc_compile_ex: survives sh_init, the fallback compiles again */
    uint32_t unread;            /* g2sc_compile_vs: survives sh_init too */
    int strict;                 /* g2sc_compile_strict: every error the language requires; survives sh_init too */
    const int *umap, *vmap;     /* g2sc_compile_packed: slot -> row * 4 + lane; survive sh_init too */
    uint32_t depth_units;       /* bit per sampler: a depth texture is bound, a read gives (z, z, z, 1) */
    uint32_t xops;              /* glslc xops (g2sc_set_target), a fragment stage only: 1 SQRT / RCPM / RSQM, 2 CLAMP; survives sh_init too */
    uint32_t noops;             /* glslc noops (g2sc_set_absent), this stage's core: 1 no integer divider, 2 no SIN / COS; survives sh_init too */
    int nohoist;                /* glslc hoist=False: the register-spill retries compile without licm */
    int urows, nvmap;
    int depth;
    ShChunk *chunk;
    size_t total;
    uint64_t nan_id;            /* every Python NaN object has an identity: see sh_pf() */
} Shc;

void   sh_init(Shc *h);
void   sh_done(Shc *h);
void  *sh_alloc(Shc *h, size_t n);                 /* 8-aligned, uninitialized */
void  *sh_zalloc(Shc *h, size_t n);
ShMark sh_mark(Shc *h);
void   sh_release(Shc *h, ShMark m);               /* frees everything allocated since the mark */
char  *sh_strndup(Shc *h, const char *s, size_t n);
char  *sh_strdup(Shc *h, const char *s);
char  *sh_fmt(Shc *h, const char *fmt, ...);       /* %s %d %L (long long) %c %% */
SHC_NORETURN void sh_compile_error(Shc *h, const char *fmt, ...);   /* raise CompileError */
SHC_NORETURN void sh_py_error(Shc *h, const char *fmt, ...);        /* any other Python exception (AttributeError, ...) */

#define SH_ENTER(h, frames) do { if (((h)->depth += (frames)) > SHC_DEPTH_LIMIT) sh_py_error((h), "RecursionError: maximum recursion depth exceeded"); } while (0)
#define SH_LEAVE(h, frames) ((h)->depth -= (frames))

/* the Python float object model: a float that comes out of an operation is a new object. Everything compares by value
 * but a NaN, which is equal only to itself as the SAME object (dict keys); the arena gives each new NaN a unique payload */
double sh_pf(Shc *h, double v);
int    sh_key_eq(double a, double b);              /* dict-key equality of two floats */

/* ---------------- string builder ---------------- */
typedef struct SB { Shc *h; char *p; size_t n, cap; } SB;
void sb_init(SB *b, Shc *h, size_t cap);
void sb_putc(SB *b, char c);
void sb_putn(SB *b, const char *s, size_t n);
void sb_puts(SB *b, const char *s);
void sb_putll(SB *b, long long v);
void sb_vfmt(SB *b, const char *fmt, va_list ap);  /* the formatter of sh_fmt */
char *sb_str(SB *b);                               /* NUL-terminated, stays valid (the arena owns it) */

/* ---------------- Python str semantics on UTF-8 bytes ---------------- */
int  sh_ws_len(const char *s, const char *end);      /* bytes of the str.isspace() character at s, 0 if none */
void sh_strip(const char **s, const char **end);     /* str.strip() */
int  sh_is_word(unsigned char c);                    /* \w (ASCII: see the report for what that leaves out) */
int  sh_is_digit(unsigned char c);

/* ---------------- ordered hash map (Python dict: insertion order, key -> value) ---------------- */
typedef struct DictEnt { const char *key; unsigned hash; int live; void *p; double d; long long i; } DictEnt;
typedef struct Dict { DictEnt *ent; int n, cap, nlive; int *tab; int tabcap; } Dict;
DictEnt *dict_find(const Dict *d, const char *key);
DictEnt *dict_put(Shc *h, Dict *d, const char *key);  /* find or append (a kept key keeps its position) */
int      dict_del(Dict *d, const char *key);          /* 1 if it was there */

/* ---------------- big integers (Python int) ---------------- */
typedef struct BI { int neg; int n; uint32_t *d; } BI;
BI    *bi_from_u64(Shc *h, uint64_t v);
BI    *bi_from_digits(Shc *h, const char *s, size_t n, int base);  /* '_' skipped; no sign */
int    bi_is_zero(const BI *a);
int    bi_cmp(const BI *a, const BI *b);
BI    *bi_add(Shc *h, const BI *a, const BI *b);
BI    *bi_sub(Shc *h, const BI *a, const BI *b);
BI    *bi_mul(Shc *h, const BI *a, const BI *b);
void   bi_divmod(Shc *h, const BI *a, const BI *b, BI **q, BI **r);  /* floor semantics, b != 0 */
BI    *bi_neg(Shc *h, const BI *a);
BI    *bi_invert(Shc *h, const BI *a);
BI    *bi_shl(Shc *h, const BI *a, uint64_t k);
BI    *bi_shr(Shc *h, const BI *a, uint64_t k);
BI    *bi_bitop(Shc *h, const BI *a, const BI *b, int op);  /* '&' '|' '^' */
BI    *bi_pow(Shc *h, const BI *a, uint64_t e);
int    bi_fits_u64(const BI *a, uint64_t *v);              /* non-negative and < 2^64 */
int    bi_is_odd(const BI *a);
double bi_to_double(Shc *h, const BI *a, int *overflow);   /* float(int): round half even */
double bi_ratio(Shc *h, const BI *a, const BI *b, int *overflow); /* int / int, correctly rounded */
BI    *bi_from_double(Shc *h, double d);                   /* int(d), finite */

double sh_str2d(Shc *h, const char *s, size_t n);          /* float(str) of digits[.digits][e[+-]digits], correctly rounded */
double sh_fmod(double x, double y);                        /* exact, the C99 fmod */
double sh_pymod(double x, double y);                       /* Python float %, y != 0 */

/* ---------------- preprocessor ---------------- */
enum { SH_EXT_DERIV = 1, SH_EXT_LOD = 2, SH_EXT_FRAGDEPTH = 4, SH_EXT_3D = 8 };   /* glslc.EXTENSIONS; *exts = the ones #extension enabled */
char *sh_preprocess(Shc *h, const char *src, int fs, int *exts);

/* ---------------- lexer, AST ---------------- */
enum { TK_KW, TK_ID, TK_NUM, TK_OP, TK_EOF };
enum { LIT_FLOAT, LIT_INT, LIT_BOOL, LIT_ONE };             /* IntLit, BoolLit, OneLit: the 1 of ++x, of the type of x */
typedef struct Tk { int k; const char *v; double num; int lit; } Tk;  /* a number's v is a canonical key (never a valid name) */

typedef enum {
    N_NUM, N_VAR, N_SWIZZLE, N_INDEX, N_CALL, N_POSTOP, N_UNARY, N_BIN, N_TERNARY, N_ASSIGN, N_SEQ, N_REG,
    S_BLOCK, S_IF, S_FOR, S_WHILE, S_DOWHILE, S_RETURN, S_DISCARD, S_BREAK, S_CONTINUE, S_LDECL, S_MULTI, S_EXPR, S_PREC, S_LSTRUCT
} NodeKind;

struct Val;
struct Member;
struct ShFn;
typedef struct Node {
    NodeKind kind;
    double num;                  /* N_NUM */
    int lit;                     /* N_NUM: LIT_* */
    int isint;                   /* Sema: an expression of type int */
    const char *s;               /* N_VAR name, N_SWIZZLE field, N_CALL name, N_UNARY / N_BIN / N_ASSIGN / N_POSTOP op, S_LDECL name, S_LSTRUCT name */
    const char *ty;              /* S_LDECL, S_PREC */
    const char *prec;            /* S_LDECL, S_PREC: the precision qualifier */
    int isc;                     /* S_LDECL const */
    int cond;                    /* S_LDECL: written under runtime control flow, so a real masked register */
    int hwloop;                  /* S_FOR: the pass count of a long constant-bounded loop that is a hardware loop, else 0 (glslc.hw_for; shc_gen.c long_for) */
    int fragdata;                /* N_VAR gl_FragColor that was gl_FragData[0] (Parser.fragdata) */
    struct Node *a, *b, *c, *d;  /* children: see sh_parse (the field order of the Python tuples) */
    struct Node **list; int nlist;
    struct Val *reg;             /* N_REG: a pre-materialized value in a synthetic AST */
    struct ShFn *fn;             /* N_CALL: the function Sema resolved it to */
    struct Member *mem; int nmem;   /* S_LSTRUCT */
} Node;

typedef struct Member { const char *ty, *name; Node *arr; const char *prec; } Member;
typedef struct Param { const char *q, *ty, *name, *prec; Node *arr; int isconst; } Param;

typedef enum { D_STRUCT, D_GDECL, D_FUNC, D_PROTO, D_PREC } DeclKind;
typedef struct Decl {
    DeclKind kind;
    const char *name;            /* struct / function / variable name */
    Member *members; int nmembers;              /* D_STRUCT */
    const char *qual, *ty; Node *arr, *init;    /* D_GDECL; ty = return type of a D_FUNC; D_PREC: qual = the qualifier */
    Param *params; int nparams; Node *body;     /* D_FUNC, D_PROTO */
    const char *prec;            /* its precision qualifier */
    const char *fname;           /* D_FUNC: the name the code generator knows it by: an overload's is its own */
} Decl;

/* preprocess's output -> lex + Parser.parse; *hoisted: the structs defined in functions */
Decl *sh_parse(Shc *h, const char *src, int exts, int *ndecl, Decl **hoisted, int *nhoisted);

/* ---------------- semantic pass, constant expressions ---------------- */
typedef struct ShT { const char *b; long long arr; } ShT;   /* a type; arr < 0: not an array */
typedef struct ShFn {
    const char *name, *ret;
    Param *params; int nparams;
    ShT *sig;
    int defined, state;
    Decl *node;
    struct ShFn **calls; int ncalls, capcalls;
    struct ShFn *next;           /* the next overload of the name */
} ShFn;

void sh_sema(Shc *h, Decl *ast, int nd, int fs, int exts, int *depthrange);     /* Sema(...).run(ast) */
/* get: the value of a scalar known at compile time. fold_expr: 1 and *out, *isint if e is a constant expression */
typedef int (*ShGet)(void *ctx, const char *name, double *v, int *isint);
int  sh_fold(Shc *h, Node *e, ShGet get, void *ctx, double *out, int *isint);
double sh_fold_cast(Shc *h, const char *name, double a);    /* float(int(a)), float(bool(a)), a */

#define SH_NMAX 8
extern const struct ShMaxC { const char *name; int v; } SH_GL_MAX[SH_NMAX];    /* glslc.GL_MAX */
extern Member SH_DEPTHRANGE[3];

/* ---------------- passes: the compiler's .sasm lines, pre-tokenized ---------------- */
#define LINE_MAXTOK 8
typedef struct Tok {
    const char *s;               /* the text of a non-register token; NULL for a register */
    int num;                     /* register number */
    char mod;                    /* 0, '-', '!' */
    char swz[5];                 /* the swizzle letters, 0 = none */
} Tok;

typedef struct Line {
    const char *label;           /* a label line: its name */
    int op;                      /* index into the mnemonic table */
    const char *mask;            /* the mnemonic's suffix ("" = none) */
    int has_dot;                 /* '.' in toks[0] */
    int ntok;                    /* including the mnemonic, tok[0] is unused */
    Tok tok[LINE_MAXTOK];
    int dst;                     /* _dst(), -1 = None */
    int nsrc; int src[LINE_MAXTOK];   /* _srcs() */
} Line;

typedef struct Lines { Line **v; int n, cap; } Lines;
Line *line_parse(Shc *h, char *text);                 /* tokenizes in place: text stays owned by the arena */
void  lines_push(Shc *h, Lines *l, Line *ln);

/* the whole post-codegen pipeline -> the final .sasm text (physical registers, packed scalars, @N resolved); pool: the constants, 4 doubles each (shape_opt reads the 0 and 1) */
char *sh_optimize(Shc *h, Lines *code, int vcount, const unsigned char *svregs, int remat, long long const_base, int fs, double *pool, int *npool_io);   /* the pool comes back with the loaded constants alone */

/* ---------------- code generator ---------------- */
typedef struct ShSym { const char *name; long long loc; const char *ty; int has_array; long long array; } ShSym;
typedef struct ShMember { const char *name, *ty; long long offset, nslots, array; int has_array; } ShMember;
typedef struct ShStruct { const char *name; long long nslots; ShMember *members; int nmembers; } ShStruct;

typedef struct ShOutput {
    char *sasm;
    ShSym *attribs, *uniforms, *samplers, *varyings;
    int nattribs, nuniforms, nsamplers, nvaryings;
    long long const_base;
    double *pool; int npool;     /* 4 doubles each */
    ShStruct *structs; int nstructs;
    int vs_tex;                  /* a vertex program samples a texture: it compiles, the link refuses it */
} ShOutput;

void sh_compile(Shc *h, const char *src, int fs, int remat, ShOutput *out);   /* _compile_stage(..., isa64=True) */

/* ---------------- assembler ---------------- */
uint64_t *sh_assemble(Shc *h, const char *sasm, int vs, int max_if, int max_loop, int *nwords);  /* AsmError = sh_py_error */

#endif
