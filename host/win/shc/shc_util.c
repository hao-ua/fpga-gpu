/* shc_util.c — arena, error exit, strings, ordered dict, big integers, exact decimal -> double. */
#include <stdlib.h>
#include <string.h>
#include "shc_internal.h"

/* ================================================================ arena + errors */
#define CHUNK_MIN ((size_t)64 * 1024)

void sh_init(Shc *h)
{
    h->err[0] = 0; h->compile_error = 0; h->depth = 0; h->chunk = NULL; h->total = 0; h->nan_id = 0;
}

void sh_done(Shc *h)
{
    ShChunk *c = h->chunk;
    while (c) { ShChunk *n = c->next; free(c); c = n; }
    h->chunk = NULL; h->total = 0;
}

void *sh_alloc(Shc *h, size_t n)
{
    ShChunk *c = h->chunk;
    size_t hdr = (sizeof(ShChunk) + 7) & ~(size_t)7, need = (n + 7) & ~(size_t)7;
    if (need < n) sh_py_error(h, "out of memory");
    if (!c || c->cap - c->used < need) {
        size_t cap = need > CHUNK_MIN ? need : CHUNK_MIN;
        if (cap > SHC_ARENA_LIMIT || h->total + cap > SHC_ARENA_LIMIT) sh_py_error(h, "out of memory (shader too large)");
        c = (ShChunk *)malloc(hdr + cap);
        if (!c) sh_py_error(h, "out of memory");
        c->next = h->chunk; c->cap = cap; c->used = 0;
        h->chunk = c; h->total += cap;
    }
    {
        void *p = (char *)c + hdr + c->used;
        c->used += need;
        return p;
    }
}

void *sh_zalloc(Shc *h, size_t n) { void *p = sh_alloc(h, n); memset(p, 0, n); return p; }

ShMark sh_mark(Shc *h)
{
    ShMark m; m.chunk = h->chunk; m.used = h->chunk ? h->chunk->used : 0; m.total = h->total;
    return m;
}

void sh_release(Shc *h, ShMark m)
{
    while (h->chunk != m.chunk) { ShChunk *n = h->chunk->next; free(h->chunk); h->chunk = n; }
    if (h->chunk) h->chunk->used = m.used;
    h->total = m.total;
}

char *sh_strndup(Shc *h, const char *s, size_t n)
{
    char *p = (char *)sh_alloc(h, n + 1);
    memcpy(p, s, n); p[n] = 0;
    return p;
}

char *sh_strdup(Shc *h, const char *s) { return sh_strndup(h, s, strlen(s)); }

/* the formatter behind every message and emitted line: %s %d %L %c %% only, no locale, no C runtime */
void sb_vfmt(SB *b, const char *fmt, va_list ap)
{
    for (; *fmt; fmt++) {
        if (*fmt != '%') { sb_putc(b, *fmt); continue; }
        fmt++;
        switch (*fmt) {
        case 's': { const char *s = va_arg(ap, const char *); sb_puts(b, s ? s : "(null)"); break; }
        case 'd': sb_putll(b, va_arg(ap, int)); break;
        case 'L': sb_putll(b, va_arg(ap, long long)); break;
        case 'c': sb_putc(b, (char)va_arg(ap, int)); break;
        case '%': sb_putc(b, '%'); break;
        default:  sb_putc(b, '%'); if (*fmt) sb_putc(b, *fmt); else fmt--; break;
        }
    }
}

char *sh_fmt(Shc *h, const char *fmt, ...)
{
    SB b; va_list ap;
    sb_init(&b, h, 32);
    va_start(ap, fmt); sb_vfmt(&b, fmt, ap); va_end(ap);
    return sb_str(&b);
}

/* the message goes into a fixed buffer: the error path must work when the arena cannot grow */
static void err_v(Shc *h, const char *fmt, va_list ap)
{
    size_t n = 0, cap = sizeof h->err - 1;
    for (; *fmt && n < cap; fmt++) {
        if (*fmt != '%') { h->err[n++] = *fmt; continue; }
        fmt++;
        if (*fmt == 's') {
            const char *s = va_arg(ap, const char *);
            if (!s) s = "(null)";
            while (*s && n < cap) h->err[n++] = *s++;
        } else if (*fmt == 'd' || *fmt == 'L') {
            long long v = *fmt == 'd' ? va_arg(ap, int) : va_arg(ap, long long);
            char t[24]; int k = 0; unsigned long long u = v < 0 ? 0ULL - (unsigned long long)v : (unsigned long long)v;
            do { t[k++] = (char)('0' + u % 10); u /= 10; } while (u);
            if (v < 0 && n < cap) h->err[n++] = '-';
            while (k && n < cap) h->err[n++] = t[--k];
        } else if (*fmt == 'c') {
            h->err[n++] = (char)va_arg(ap, int);
        } else if (*fmt == '%') {
            h->err[n++] = '%';
        } else if (!*fmt) {
            break;
        }
    }
    h->err[n] = 0;
}

void sh_compile_error(Shc *h, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt); err_v(h, fmt, ap); va_end(ap);
    h->compile_error = 1;
    longjmp(h->jb, 1);
}

void sh_py_error(Shc *h, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt); err_v(h, fmt, ap); va_end(ap);
    h->compile_error = 0;
    longjmp(h->jb, 1);
}

static uint64_t dbits(double d) { uint64_t u; memcpy(&u, &d, 8); return u; }
static double bitsd(uint64_t u) { double d; memcpy(&d, &u, 8); return d; }
static int d_isnan(double d) { return (dbits(d) & 0x7FFFFFFFFFFFFFFFULL) > 0x7FF0000000000000ULL; }

double sh_pf(Shc *h, double v)
{
    if (d_isnan(v)) return bitsd((dbits(v) & 0x8000000000000000ULL) | 0x7FF8000000000000ULL | (++h->nan_id & 0x0007FFFFFFFFFFFFULL));
    return v;
}

int sh_key_eq(double a, double b)
{
    return a == b || (d_isnan(a) && d_isnan(b) && dbits(a) == dbits(b));
}

/* ================================================================ string builder */
void sb_init(SB *b, Shc *h, size_t cap)
{
    b->h = h; b->n = 0; b->cap = cap < 16 ? 16 : cap;
    b->p = (char *)sh_alloc(h, b->cap);
}

static void sb_room(SB *b, size_t more)
{
    if (b->cap - b->n < more + 1) {
        size_t cap = b->cap * 2;
        char *p;
        while (cap - b->n < more + 1) cap *= 2;
        p = (char *)sh_alloc(b->h, cap);
        memcpy(p, b->p, b->n);
        b->p = p; b->cap = cap;
    }
}

void sb_putc(SB *b, char c) { sb_room(b, 1); b->p[b->n++] = c; }
void sb_putn(SB *b, const char *s, size_t n) { sb_room(b, n); memcpy(b->p + b->n, s, n); b->n += n; }
void sb_puts(SB *b, const char *s) { sb_putn(b, s, strlen(s)); }

void sb_putll(SB *b, long long v)
{
    char t[24]; int k = 0;
    unsigned long long u = v < 0 ? 0ULL - (unsigned long long)v : (unsigned long long)v;
    do { t[k++] = (char)('0' + u % 10); u /= 10; } while (u);
    if (v < 0) sb_putc(b, '-');
    while (k) sb_putc(b, t[--k]);
}

char *sb_str(SB *b) { sb_room(b, 1); b->p[b->n] = 0; return b->p; }

/* ================================================================ Python str semantics */
int sh_ws_len(const char *cs, const char *cend)
{
    const unsigned char *s = (const unsigned char *)cs, *e = (const unsigned char *)cend;
    unsigned char c;
    if (s >= e) return 0;
    c = s[0];
    if (c < 0x80) return c == ' ' || (c >= 9 && c <= 13) || (c >= 0x1C && c <= 0x1F);
    if (c == 0xC2 && e - s >= 2 && (s[1] == 0x85 || s[1] == 0xA0)) return 2;
    if (e - s >= 3) {
        if (c == 0xE1 && s[1] == 0x9A && s[2] == 0x80) return 3;
        if (c == 0xE2 && s[1] == 0x80 && ((s[2] >= 0x80 && s[2] <= 0x8A) || s[2] == 0xA8 || s[2] == 0xA9 || s[2] == 0xAF)) return 3;
        if (c == 0xE2 && s[1] == 0x81 && s[2] == 0x9F) return 3;
        if (c == 0xE3 && s[1] == 0x80 && s[2] == 0x80) return 3;
    }
    return 0;
}

void sh_strip(const char **ps, const char **pe)
{
    const char *s = *ps, *e = *pe;
    int k;
    while ((k = sh_ws_len(s, e)) != 0) s += k;
    while (e > s) {
        if (sh_ws_len(e - 1, e) == 1) { e -= 1; continue; }
        if (e - s >= 2 && sh_ws_len(e - 2, e) == 2) { e -= 2; continue; }
        if (e - s >= 3 && sh_ws_len(e - 3, e) == 3) { e -= 3; continue; }
        break;
    }
    *ps = s; *pe = e;
}

int sh_is_digit(unsigned char c) { return c >= '0' && c <= '9'; }
int sh_is_word(unsigned char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_'; }

/* ================================================================ ordered dict */
static unsigned str_hash(const char *s)
{
    unsigned h = 2166136261u;
    for (; *s; s++) { h ^= (unsigned char)*s; h *= 16777619u; }
    return h;
}

DictEnt *dict_find(const Dict *d, const char *key)
{
    unsigned h, i;
    if (!d->tabcap) return NULL;
    h = str_hash(key);
    for (i = h & (unsigned)(d->tabcap - 1); ; i = (i + 1) & (unsigned)(d->tabcap - 1)) {
        int k = d->tab[i];
        if (k < 0) return NULL;
        if (d->ent[k].hash == h && d->ent[k].live && !strcmp(d->ent[k].key, key)) return &d->ent[k];
    }
}

static void dict_index(Dict *d, int k)
{
    unsigned i;
    for (i = d->ent[k].hash & (unsigned)(d->tabcap - 1); d->tab[i] >= 0; i = (i + 1) & (unsigned)(d->tabcap - 1)) { }
    d->tab[i] = k;
}

DictEnt *dict_put(Shc *h, Dict *d, const char *key)
{
    DictEnt *e = dict_find(d, key);
    if (e) return e;
    if (d->n == d->cap) {                  /* tombstones are compacted here, the order is kept */
        int cap = d->nlive * 2 < 8 ? 8 : d->nlive * 2, i, k = 0;
        DictEnt *ne = (DictEnt *)sh_alloc(h, (size_t)cap * sizeof(DictEnt));
        for (i = 0; i < d->n; i++) if (d->ent[i].live) ne[k++] = d->ent[i];
        d->ent = ne; d->n = k; d->cap = cap;
        d->tabcap = 16;
        while (d->tabcap < cap * 2) d->tabcap *= 2;
        d->tab = (int *)sh_alloc(h, (size_t)d->tabcap * sizeof(int));
        memset(d->tab, 0xFF, (size_t)d->tabcap * sizeof(int));
        for (i = 0; i < k; i++) dict_index(d, i);
    }
    e = &d->ent[d->n];
    e->key = key; e->hash = str_hash(key); e->live = 1; e->p = NULL; e->d = 0; e->i = 0;
    dict_index(d, d->n);
    d->n++; d->nlive++;
    return e;
}

int dict_del(Dict *d, const char *key)
{
    DictEnt *e = dict_find(d, key);
    if (!e) return 0;
    e->live = 0; d->nlive--;           /* the probe chain stays: a dead entry is skipped by dict_find */
    return 1;
}

/* ================================================================ big integers */
#define BI_MAX_LIMBS (1 << 16)         /* 2 Mbit: beyond this #if arithmetic fails loudly (schoolbook multiply) */

static BI *bi_new(Shc *h, int n)
{
    BI *r = (BI *)sh_alloc(h, sizeof(BI));
    if (n > BI_MAX_LIMBS) sh_compile_error(h, "integer too large in a #if expression");
    r->neg = 0; r->n = n;
    r->d = (uint32_t *)sh_alloc(h, (size_t)(n ? n : 1) * 4);
    return r;
}

static BI *bi_trim(BI *r)
{
    while (r->n > 0 && r->d[r->n - 1] == 0) r->n--;
    if (r->n == 0) r->neg = 0;
    return r;
}

BI *bi_from_u64(Shc *h, uint64_t v)
{
    BI *r = bi_new(h, 2);
    r->d[0] = (uint32_t)v; r->d[1] = (uint32_t)(v >> 32);
    return bi_trim(r);
}

BI *bi_from_digits(Shc *h, const char *s, size_t n, int base)
{
    size_t cap = n / (base == 2 ? 32 : base == 8 ? 10 : base == 16 ? 8 : 9) + 2, i;
    BI *r;
    if (cap > BI_MAX_LIMBS) sh_compile_error(h, "integer too large in a #if expression");
    r = bi_new(h, (int)cap);
    r->n = 0;
    for (i = 0; i < n; i++) {
        unsigned c = (unsigned char)s[i], dg;
        uint64_t carry;
        int k;
        if (c == '_') continue;
        dg = c <= '9' ? c - '0' : (c | 32) - 'a' + 10;
        carry = dg;
        for (k = 0; k < r->n; k++) {
            uint64_t t = (uint64_t)r->d[k] * (unsigned)base + carry;
            r->d[k] = (uint32_t)t; carry = t >> 32;
        }
        if (carry) r->d[r->n++] = (uint32_t)carry;
    }
    return r;
}

int bi_is_zero(const BI *a) { return a->n == 0; }
int bi_is_odd(const BI *a) { return a->n > 0 && (a->d[0] & 1); }

static int cmp_mag(const BI *a, const BI *b)
{
    int i;
    if (a->n != b->n) return a->n < b->n ? -1 : 1;
    for (i = a->n - 1; i >= 0; i--) if (a->d[i] != b->d[i]) return a->d[i] < b->d[i] ? -1 : 1;
    return 0;
}

int bi_cmp(const BI *a, const BI *b)
{
    if (a->neg != b->neg) return a->neg ? -1 : 1;
    return a->neg ? -cmp_mag(a, b) : cmp_mag(a, b);
}

static BI *add_mag(Shc *h, const BI *a, const BI *b)
{
    const BI *x = a->n >= b->n ? a : b, *y = a->n >= b->n ? b : a;
    BI *r = bi_new(h, x->n + 1);
    uint64_t c = 0;
    int i;
    for (i = 0; i < x->n; i++) {
        c += (uint64_t)x->d[i] + (i < y->n ? y->d[i] : 0);
        r->d[i] = (uint32_t)c; c >>= 32;
    }
    r->d[x->n] = (uint32_t)c;
    return bi_trim(r);
}

static BI *sub_mag(Shc *h, const BI *a, const BI *b)     /* |a| >= |b| */
{
    BI *r = bi_new(h, a->n);
    int64_t c = 0;
    int i;
    for (i = 0; i < a->n; i++) {
        c += (int64_t)a->d[i] - (i < b->n ? b->d[i] : 0);
        r->d[i] = (uint32_t)c; c >>= 32;
    }
    return bi_trim(r);
}

BI *bi_neg(Shc *h, const BI *a)
{
    BI *r = bi_new(h, a->n);
    memcpy(r->d, a->d, (size_t)a->n * 4);
    r->neg = a->n ? !a->neg : 0;
    return r;
}

BI *bi_add(Shc *h, const BI *a, const BI *b)
{
    BI *r;
    if (a->neg == b->neg) { r = add_mag(h, a, b); r->neg = r->n ? a->neg : 0; return r; }
    if (cmp_mag(a, b) >= 0) { r = sub_mag(h, a, b); r->neg = r->n ? a->neg : 0; return r; }
    r = sub_mag(h, b, a); r->neg = r->n ? b->neg : 0;
    return r;
}

BI *bi_sub(Shc *h, const BI *a, const BI *b)
{
    BI t = *b;
    t.neg = b->n ? !b->neg : 0;
    return bi_add(h, a, &t);
}

BI *bi_mul(Shc *h, const BI *a, const BI *b)
{
    BI *r = bi_new(h, a->n + b->n);
    int i, j;
    memset(r->d, 0, (size_t)(a->n + b->n) * 4);
    for (i = 0; i < a->n; i++) {
        uint64_t c = 0;
        for (j = 0; j < b->n; j++) {
            c += (uint64_t)a->d[i] * b->d[j] + r->d[i + j];
            r->d[i + j] = (uint32_t)c; c >>= 32;
        }
        r->d[i + b->n] = (uint32_t)c;
    }
    r->neg = a->neg != b->neg;
    return bi_trim(r);
}

static int bitlen32(uint32_t v) { int n = 0; while (v) { n++; v >>= 1; } return n; }
static long long bitlen(const BI *a) { return a->n ? (long long)(a->n - 1) * 32 + bitlen32(a->d[a->n - 1]) : 0; }

/* magnitude division, Knuth algorithm D */
static void divmod_mag(Shc *h, const BI *a, const BI *b, BI **pq, BI **pr)
{
    if (cmp_mag(a, b) < 0) {
        BI *r = bi_new(h, a->n);
        memcpy(r->d, a->d, (size_t)a->n * 4);
        *pq = bi_new(h, 0); *pr = r;
        return;
    }
    if (b->n == 1) {
        BI *q = bi_new(h, a->n), *r = bi_new(h, 1);
        uint64_t rem = 0;
        int i;
        for (i = a->n - 1; i >= 0; i--) {
            uint64_t cur = (rem << 32) | a->d[i];
            q->d[i] = (uint32_t)(cur / b->d[0]); rem = cur % b->d[0];
        }
        r->d[0] = (uint32_t)rem;
        *pq = bi_trim(q); *pr = bi_trim(r);
        return;
    }
    {
        int s = 32 - bitlen32(b->d[b->n - 1]), n = b->n, m = a->n - b->n, i, j;
        uint32_t *u = (uint32_t *)sh_alloc(h, (size_t)(a->n + 1) * 4);
        uint32_t *v = (uint32_t *)sh_alloc(h, (size_t)n * 4);
        BI *q = bi_new(h, m + 1), *r = bi_new(h, n);
        for (i = n - 1; i > 0; i--) v[i] = (b->d[i] << s) | (s ? b->d[i - 1] >> (32 - s) : 0);
        v[0] = b->d[0] << s;
        u[a->n] = s ? a->d[a->n - 1] >> (32 - s) : 0;
        for (i = a->n - 1; i > 0; i--) u[i] = (a->d[i] << s) | (s ? a->d[i - 1] >> (32 - s) : 0);
        u[0] = a->d[0] << s;
        for (j = m; j >= 0; j--) {
            uint64_t num = ((uint64_t)u[j + n] << 32) | u[j + n - 1];
            uint64_t qh = num / v[n - 1], rh = num % v[n - 1];
            int64_t t, k;
            while (qh >= ((uint64_t)1 << 32) || qh * v[n - 2] > ((rh << 32) | u[j + n - 2])) {
                qh--; rh += v[n - 1];
                if (rh >= ((uint64_t)1 << 32)) break;
            }
            k = 0;
            for (i = 0; i < n; i++) {
                uint64_t p = qh * v[i];
                t = (int64_t)u[i + j] - k - (int64_t)(p & 0xFFFFFFFFu);
                u[i + j] = (uint32_t)t;
                k = (int64_t)(p >> 32) - (t >> 32);
            }
            t = (int64_t)u[j + n] - k;
            u[j + n] = (uint32_t)t;
            if (t < 0) {
                uint64_t c = 0;
                qh--;
                for (i = 0; i < n; i++) {
                    c += (uint64_t)u[i + j] + v[i];
                    u[i + j] = (uint32_t)c; c >>= 32;
                }
                u[j + n] += (uint32_t)c;
            }
            q->d[j] = (uint32_t)qh;
        }
        for (i = 0; i < n; i++) r->d[i] = (u[i] >> s) | (s ? (uint32_t)((uint64_t)u[i + 1] << (32 - s)) : 0);
        *pq = bi_trim(q); *pr = bi_trim(r);
    }
}

void bi_divmod(Shc *h, const BI *a, const BI *b, BI **pq, BI **pr)
{
    BI *q, *r;
    divmod_mag(h, a, b, &q, &r);
    q->neg = q->n ? a->neg != b->neg : 0;
    r->neg = r->n ? a->neg : 0;
    if (r->n && a->neg != b->neg) {        /* the remainder takes the divisor's sign */
        r = bi_add(h, r, b);
        q = bi_sub(h, q, bi_from_u64(h, 1));
    }
    *pq = q; *pr = r;
}

BI *bi_invert(Shc *h, const BI *a) { return bi_sub(h, bi_neg(h, a), bi_from_u64(h, 1)); }  /* ~x = -x - 1 */

BI *bi_shl(Shc *h, const BI *a, uint64_t k)
{
    BI *r;
    int ls, bs, i;
    if (!a->n) return bi_new(h, 0);
    if (k / 32 > BI_MAX_LIMBS) sh_compile_error(h, "integer too large in a #if expression");
    ls = (int)(k / 32); bs = (int)(k % 32);
    r = bi_new(h, a->n + ls + 1);
    memset(r->d, 0, (size_t)r->n * 4);
    for (i = 0; i < a->n; i++) {
        uint64_t t = (uint64_t)a->d[i] << bs;
        r->d[i + ls] |= (uint32_t)t;
        r->d[i + ls + 1] |= (uint32_t)(t >> 32);
    }
    r->neg = a->neg;
    return bi_trim(r);
}

BI *bi_shr(Shc *h, const BI *a, uint64_t k)
{
    BI *r;
    int ls, bs, i;
    if (a->neg) {                          /* floor: -((|a| - 1) >> k) - 1 */
        BI t = *a;
        t.neg = 0;
        return bi_invert(h, bi_shr(h, bi_sub(h, &t, bi_from_u64(h, 1)), k));
    }
    if (k / 32 >= (uint64_t)a->n) return bi_new(h, 0);
    ls = (int)(k / 32); bs = (int)(k % 32);
    r = bi_new(h, a->n - ls);
    for (i = 0; i < r->n; i++) {
        uint64_t t = a->d[i + ls];
        if (i + ls + 1 < a->n) t |= (uint64_t)a->d[i + ls + 1] << 32;
        r->d[i] = (uint32_t)(t >> bs);
    }
    return bi_trim(r);
}

static void to_twos(const BI *a, uint32_t *d, int n)
{
    int i;
    uint64_t c = 1;
    for (i = 0; i < n; i++) {
        uint32_t v = i < a->n ? a->d[i] : 0;
        if (a->neg) { c += (uint32_t)~v; d[i] = (uint32_t)c; c >>= 32; }
        else d[i] = v;
    }
}

BI *bi_bitop(Shc *h, const BI *a, const BI *b, int op)
{
    int n = (a->n > b->n ? a->n : b->n) + 1, i;
    uint32_t *x = (uint32_t *)sh_alloc(h, (size_t)n * 4), *y = (uint32_t *)sh_alloc(h, (size_t)n * 4);
    BI *r = bi_new(h, n);
    to_twos(a, x, n); to_twos(b, y, n);
    for (i = 0; i < n; i++) r->d[i] = op == '&' ? x[i] & y[i] : op == '|' ? x[i] | y[i] : x[i] ^ y[i];
    if (r->d[n - 1] >> 31) {               /* negative: back to sign + magnitude */
        uint64_t c = 1;
        for (i = 0; i < n; i++) { c += (uint32_t)~r->d[i]; r->d[i] = (uint32_t)c; c >>= 32; }
        r->neg = 1;
    }
    return bi_trim(r);
}

BI *bi_pow(Shc *h, const BI *a, uint64_t e)
{
    BI *r = bi_from_u64(h, 1), *b;
    b = bi_add(h, a, bi_new(h, 0));
    while (e) {
        if (e & 1) r = bi_mul(h, r, b);
        e >>= 1;
        if (e) b = bi_mul(h, b, b);
    }
    return r;
}

int bi_fits_u64(const BI *a, uint64_t *v)
{
    if (a->neg || a->n > 2) return 0;
    *v = (a->n > 0 ? a->d[0] : 0) | ((uint64_t)(a->n > 1 ? a->d[1] : 0) << 32);
    return 1;
}

/* |a| / |b| to the nearest double (ties to even), subnormals included; the integer division is exact, no FP unit used */
static double ratio_mag(Shc *h, const BI *a, const BI *b, int *overflow)
{
    long long bl = bitlen(a) - bitlen(b), s = 63 - bl, e;
    BI *q, *r;
    BI an = *a, bn = *b;
    uint64_t Q, kept, rem, half;
    int qb, prec, drop, sticky;
    an.neg = bn.neg = 0;
    *overflow = 0;
    if (!a->n) return 0.0;
    if (bl > 2000) { *overflow = 1; return 0.0; }          /* far beyond the double range: no need to divide */
    if (bl < -2000) return 0.0;
    if (s >= 0) divmod_mag(h, bi_shl(h, &an, (uint64_t)s), &bn, &q, &r);
    else        divmod_mag(h, &an, bi_shl(h, &bn, (uint64_t)-s), &q, &r);
    if (!bi_fits_u64(q, &Q)) sh_py_error(h, "internal: ratio quotient");
    sticky = r->n != 0;
    qb = 0;
    while (qb < 64 && (Q >> qb)) qb++;
    e = (long long)qb - 1 - s;                             /* the value is in [2^e, 2^(e+1)) */
    if (e > 1023) { *overflow = 1; return 0.0; }
    prec = e >= -1022 ? 53 : (e + 1075 < 0 ? -1 : (int)(e + 1075));
    if (prec < 0) return 0.0;
    drop = qb - prec;
    if (drop >= 64) { kept = 0; rem = Q; half = (uint64_t)1 << 63; if (drop > 64) return 0.0; }
    else { kept = Q >> drop; rem = Q & (((uint64_t)1 << drop) - 1); half = (uint64_t)1 << (drop - 1); }
    if (rem > half || (rem == half && (sticky || (kept & 1)))) kept++;
    if (e >= -1022) {
        if (kept == (uint64_t)1 << 53) { kept >>= 1; e++; if (e > 1023) { *overflow = 1; return 0.0; } }
        return bitsd(((uint64_t)(e + 1023) << 52) | (kept & (((uint64_t)1 << 52) - 1)));
    }
    return bitsd(kept);                                    /* a subnormal (2^52 rounds into the first normal) */
}

double bi_ratio(Shc *h, const BI *a, const BI *b, int *overflow)
{
    double d = ratio_mag(h, a, b, overflow);
    return a->neg != b->neg ? -d : d;
}

double bi_to_double(Shc *h, const BI *a, int *overflow)
{
    double d = ratio_mag(h, a, bi_from_u64(h, 1), overflow);
    return a->neg ? -d : d;
}

BI *bi_from_double(Shc *h, double d)
{
    uint64_t u = dbits(d), m = (u & (((uint64_t)1 << 52) - 1)) | ((uint64_t)1 << 52);
    int e = (int)((u >> 52) & 0x7FF) - 1075;
    BI *r;
    if (((u >> 52) & 0x7FF) < 1023) return bi_new(h, 0);   /* |d| < 1 */
    r = e >= 0 ? bi_shl(h, bi_from_u64(h, m), (uint64_t)e) : bi_from_u64(h, m >> -e);
    r->neg = r->n ? (int)(u >> 63) : 0;
    return r;
}

/* ================================================================ float(str): correctly rounded, integer arithmetic only */
double sh_str2d(Shc *h, const char *s, size_t n)
{
    ShMark mk = sh_mark(h);
    char *dig = (char *)sh_alloc(h, n + 2);
    size_t nd = 0, i;
    long long e10 = 0;
    int dot = 0, ovf;
    double r;
    BI *D, *N, *M;
    for (i = 0; i < n; i++) {
        char c = s[i];
        if (sh_is_digit((unsigned char)c)) {
            if (nd > 0 || c != '0') dig[nd++] = c;
            if (dot) e10--;
        }
        else if (c == '.') dot = 1;
        else if (c != '_') break;
    }
    if (i < n) {                           /* e[+-]digits */
        int neg = 0;
        long long e = 0;
        i++;
        if (i < n && (s[i] == '+' || s[i] == '-')) neg = s[i++] == '-';
        for (; i < n; i++) if (s[i] != '_') { if (e < 100000000000LL) e = e * 10 + (s[i] - '0'); }
        e10 += neg ? -e : e;
    }
    while (nd > 0 && dig[nd - 1] == '0') { nd--; e10++; }
    if (nd == 0) { sh_release(h, mk); return 0.0; }
    if ((long long)nd + e10 > 400) { sh_release(h, mk); return bitsd(0x7FF0000000000000ULL); }
    if ((long long)nd + e10 < -400) { sh_release(h, mk); return 0.0; }
    if (nd > 800) {                        /* beyond this the digits only say "not a tie" */
        e10 += (long long)nd - 801;
        nd = 800;
        dig[nd++] = '1';
    }
    D = bi_from_digits(h, dig, nd, 10);
    N = D; M = bi_from_u64(h, 1);
    {
        long long k = e10 < 0 ? -e10 : e10;
        BI *p = bi_from_u64(h, 1), *t = bi_from_u64(h, 1000000000);
        for (; k >= 9; k -= 9) p = bi_mul(h, p, t);
        for (; k > 0; k--) p = bi_mul(h, p, bi_from_u64(h, 10));
        if (e10 >= 0) N = bi_mul(h, N, p); else M = p;
    }
    r = ratio_mag(h, N, M, &ovf);
    if (ovf) r = bitsd(0x7FF0000000000000ULL);
    sh_release(h, mk);
    return r;
}

/* ================================================================ float %, exact (the algorithm of musl's fmod) */
double sh_fmod(double x, double y)
{
    uint64_t ux = dbits(x), uy = dbits(y), sx = ux >> 63, i;
    int ex = (int)((ux >> 52) & 0x7FF), ey = (int)((uy >> 52) & 0x7FF);
    double t;
    if ((uy << 1) == 0 || (uy & 0x7FFFFFFFFFFFFFFFULL) > 0x7FF0000000000000ULL || ex == 0x7FF) {
        t = (x * y) / (x * y);             /* NaN, as C99 requires */
        return t;
    }
    if ((ux << 1) <= (uy << 1)) {
        if ((ux << 1) == (uy << 1)) return bitsd(sx << 63);
        return x;
    }
    if (!ex) { for (i = ux << 12; (i >> 63) == 0; ex--, i <<= 1) { } ux <<= 1 - ex; }
    else { ux &= ~(uint64_t)0 >> 12; ux |= (uint64_t)1 << 52; }
    if (!ey) { for (i = uy << 12; (i >> 63) == 0; ey--, i <<= 1) { } uy <<= 1 - ey; }
    else { uy &= ~(uint64_t)0 >> 12; uy |= (uint64_t)1 << 52; }
    for (; ex > ey; ex--) {
        i = ux - uy;
        if ((i >> 63) == 0) { if (i == 0) return bitsd(sx << 63); ux = i; }
        ux <<= 1;
    }
    i = ux - uy;
    if ((i >> 63) == 0) { if (i == 0) return bitsd(sx << 63); ux = i; }
    for (; (ux >> 52) == 0; ux <<= 1, ex--) { }
    if (ex > 0) { ux -= (uint64_t)1 << 52; ux |= (uint64_t)ex << 52; }
    else ux >>= 1 - ex;
    return bitsd(ux | (sx << 63));
}

double sh_pymod(double x, double y)
{
    double m = sh_fmod(x, y);
    if (m != 0.0) {                        /* NaN included, as float_rem's `if (mod)` */
        if ((y < 0) != (m < 0)) m += y;
    }
    else m = bitsd(dbits(y) & 0x8000000000000000ULL);
    return m;
}
