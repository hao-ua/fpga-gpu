/* gpu2core: reliability layer, batching, reads and every message packer — a port of host/fb/fbproto.py (class Board).
 * Where a function has a counterpart in fbproto it keeps its name, so the two can be read side by side. Differences on
 * purpose: Win32 threads and locks; a 1 ms maintenance period (Python slept 2 ms); READ_DATA chunks copied straight
 * into the caller's buffer by the receive thread.
 */
#define _WIN32_WINNT 0x0601
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <mmsystem.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gpu2core.h"

#define MAGIC 0x4246
#define VERSION 1
#define WINDOW_MAX 64
#define DGRAM_MAX 1472
#define RX_MAX 4096
#define EVENT_RING 1024
#define ERROR_RING 64
#define READ_REQS 8
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif

typedef struct slot { uint32_t seq, len; uint8_t data[DGRAM_MAX]; } slot;
typedef struct ev_entry { uint32_t seq, idx, valid; g2_event ev; } ev_entry;
typedef struct rd_req { size_t s, n; uint32_t tag; double t_sent, t_last; long long top; int has_data; } rd_req;
typedef struct rd_range { size_t s, n; } rd_range;

struct g2_board {
    SOCKET sock;
    struct sockaddr_in addr;
    FILE *rec;                              /* recording sink (parity): replaces the socket and the ACK machinery */
    int window, b_sub_index;
    double drop_rx;
    uint32_t rng;
    SRWLOCK lock;
    CONDITION_VARIABLE cv;
    HANDLE rx_thread, mt_thread;
    volatile LONG running;
    int timer_period;

    /* send state */
    uint32_t next_seq;
    slot ring[WINDOW_MAX];                  /* the unacknowledged datagrams, oldest at ring_head */
    int ring_head, ring_count;
    uint64_t inflight;                      /* _inflight(): board bytes of the unacknowledged datagrams */
    uint32_t ack_exp, ack_frame;
    int ack_valid;
    uint32_t ack_count, credit;
    int resend_req;                         /* 0 none, 1 now, 2 after the FULL wait */
    double last_progress, last_resend;
    uint32_t suspect_seq;                   /* hb_suspect */
    double suspect_t;
    int suspect_valid;

    /* the open BATCH datagram */
    int batch_open;
    uint32_t batch_count, batch_size;
    uint8_t batch[G2_BATCH_MAX];

    /* receive state */
    ev_entry events[EVENT_RING];            /* newest overwrites the oldest: nobody waits on events that old */
    int ev_next;
    g2_error errors[ERROR_RING];
    int err_count;
    uint32_t status_words[G2_ST_WORDS];
    int status_n;
    uint32_t status_count;
    uint32_t ev_base, rx_base;              /* A's counts of relayed events and of replies minus ours, at the first STATUS */
    int ev_missing, rx_missing;             /* sent by A and never arrived, as of the last STATUS */
    uint32_t lreply_count, lreply_w2, lreply_w3;

    /* the one read in progress: filled by the receive thread */
    int rd_active, rd_rel;
    uint8_t *rd_out, *rd_missing;
    size_t rd_total, rd_base, rd_nmissing;
    size_t *rd_starts;                      /* tag - rd_tag0 -> start of that request */
    size_t rd_nstarts, rd_capstarts;
    uint32_t rd_tag0;
    rd_req rd_reqs[READ_REQS];              /* outstanding, oldest first */
    int rd_nreqs;

    uint32_t fence_id;
    g2_counters c;
    int no_xga;                             /* board B refused SET_MODE 4 (a board B without mode 4): g2_mode_select stays below it */
    /* image handles as a cache of board B's table (vimg_*) */
    struct vimg *vi;                        /* [G2_HANDLES_MAX], by the handle's index */
    uint16_t *owner;                        /* [table entry] -> the image index that holds it, VIMG_NONE: free */
    uint32_t vstamp;                       /* counts the commands that name images: an entry named by the current one is never taken back */
    g2_ref last_ref;                        /* the last sequenced message's reference */
    char errtext[256];
};

/* an image's descriptor as the library gave it, and its entry in board B's table (VIMG_NONE: none now) */
#define VIMG_NONE 0xFFFF
#define B_IMAGES  1024                      /* board B's image table: handle bits 9:0 */
typedef struct vimg { uint32_t base, width, height, fmt, pitch, levels, layers, usage, vol, stamp; uint8_t origin, samples, live; uint16_t phys; } vimg;
static uint16_t phys_of(g2_board *b, uint16_t handle);

static void (*g_log)(const char *);
static uint32_t g_tag = 0x1000;             /* Board._tag */

void g2_set_log(void (*fn)(const char *line)) { g_log = fn; }

static void fail(g2_board *b, const char *fmt, ...)
{
    va_list ap;
    char tmp[256];
    va_start(ap, fmt);
    vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (b) memcpy(b->errtext, tmp, sizeof tmp);
    if (g_log) g_log(tmp);
}

const char *g2_error_text(const g2_board *b) { return b->errtext; }

static double now_s(void)
{
    static double period;
    LARGE_INTEGER t;
    if (period == 0.0) {
        LARGE_INTEGER f;
        QueryPerformanceFrequency(&f);
        period = 1.0 / (double)f.QuadPart;
    }
    QueryPerformanceCounter(&t);
    return (double)t.QuadPart * period;
}

/* little-endian packing */
typedef struct pk { uint8_t *p; } pk;
static void p8(pk *k, uint32_t v) { *k->p++ = (uint8_t)v; }
static void p16(pk *k, uint32_t v) { k->p[0] = (uint8_t)v; k->p[1] = (uint8_t)(v >> 8); k->p += 2; }
static void p32(pk *k, uint32_t v)
{
    k->p[0] = (uint8_t)v; k->p[1] = (uint8_t)(v >> 8); k->p[2] = (uint8_t)(v >> 16); k->p[3] = (uint8_t)(v >> 24);
    k->p += 4;
}
static uint32_t r32(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static uint32_t board_bytes(size_t body) { return (uint32_t)(32 + ((body + 31) / 32) * 32); }   /* need = 32 + ceil32(body) */

static void write_header(uint8_t *d, uint8_t typ, uint32_t seq)
{
    pk k = { d };
    p16(&k, MAGIC); p8(&k, VERSION); p8(&k, typ); p32(&k, seq);
}

static void rec_write(g2_board *b, const uint8_t *d, uint32_t len)
{
    uint8_t n[4];
    pk k = { n };
    p32(&k, len);
    fwrite(n, 1, 4, b->rec);
    fwrite(d, 1, len, b->rec);
}

static void sock_send(g2_board *b, const uint8_t *d, uint32_t len)
{
    sendto(b->sock, (const char *)d, (int)len, 0, (const struct sockaddr *)&b->addr, sizeof b->addr);
}

/* ---------------------------------------------------------------- receive */
static void on_ack(g2_board *b, uint32_t exp, uint32_t flags, uint32_t free_bytes, uint32_t frame)
{
    int progressed = 0;
    double now;
    b->ack_count++;
    while (b->ring_count) {
        slot *s = &b->ring[b->ring_head];
        if ((uint32_t)(s->seq - exp) >= 0x80000000u) {          /* s < exp (wrap-safe) */
            b->inflight -= board_bytes(s->len - 8);
            b->ring_head = (b->ring_head + 1) % WINDOW_MAX;
            b->ring_count--;
            progressed = 1;
        } else {
            break;
        }
    }
    now = now_s();
    if (progressed) b->last_progress = now;
    b->ack_exp = exp; b->ack_valid = 1;
    b->credit = free_bytes;
    b->ack_frame = frame;
    if (b->ring_count && (flags & G2_ACKF_NAK) && b->resend_req < 1) b->resend_req = 1;
    if (b->ring_count && (flags & G2_ACKF_FULL)) b->resend_req = 2;
    /* the heartbeat rule (fbproto._on_ack): an ACK that still expects our oldest datagram after 2 ms without progress
       marks a suspect; it may be stale, so _maint_loop resends only if nothing progressed for 2 ms after the mark */
    if (progressed) {
        b->suspect_valid = 0;
    } else if (b->ring_count && !b->suspect_valid && exp == b->ring[b->ring_head].seq && now - b->last_progress > 0.002) {
        b->suspect_valid = 1; b->suspect_seq = exp; b->suspect_t = now;
    }
}

static void store_event(g2_board *b, uint32_t seq, uint32_t idx, uint32_t code, uint32_t a0, uint32_t a1, uint32_t a2)
{
    ev_entry *e = &b->events[b->ev_next];
    b->ev_next = (b->ev_next + 1) % EVENT_RING;
    e->seq = seq; e->idx = idx; e->valid = 1;
    e->ev.code = code; e->ev.a0 = a0; e->ev.a1 = a1; e->ev.a2 = a2;
    if (code == G2_EV_ERROR) {
        g2_error *r;
        if (b->err_count == ERROR_RING) {
            memmove(b->errors, b->errors + 1, (ERROR_RING - 1) * sizeof b->errors[0]);
            b->err_count--;
        }
        r = &b->errors[b->err_count++];
        r->seq = seq; r->code = a0; r->type = a1; r->detail = a2; r->idx = idx;
    }
}

static uint32_t xorshift(uint32_t *s)
{
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return *s = x;
}

static void on_read_data(g2_board *b, const uint8_t *body, size_t len)
{
    uint32_t tag = r32(body), off = r32(body + 4);
    size_t dlen = len - 8, p;
    if (!b->rd_active || (uint32_t)(tag - b->rd_tag0) >= b->rd_nstarts) return;   /* chunks of finished / unknown reads */
    if (b->drop_rx > 0.0 && (double)(xorshift(&b->rng) >> 8) / 16777216.0 < b->drop_rx) return;
    {
        size_t base = b->rd_starts[tag - b->rd_tag0];
        long long q = b->rd_rel ? (long long)base + off : (long long)off - (long long)b->rd_base;
        if (q < 0 || (size_t)q >= b->rd_total) return;
        p = (size_t)q;
    }
    if (p % G2_RD_CHUNK == 0) {
        size_t i = p / G2_RD_CHUNK, n = b->rd_total - p;
        if (n > dlen) n = dlen;
        if (b->rd_missing[i]) { b->rd_missing[i] = 0; b->rd_nmissing--; }
        else b->c.rd_dup++;                                     /* arrived twice: late original or re-request */
        memcpy(b->rd_out + p, body + 8, n);
    }
    {
        int i;
        for (i = 0; i < b->rd_nreqs; i++) {
            rd_req *r = &b->rd_reqs[i];
            if (r->tag == tag) {
                r->t_last = now_s(); r->has_data = 1;
                if ((long long)p > r->top) r->top = (long long)p;
                break;
            }
        }
    }
}

static DWORD WINAPI rx_loop(LPVOID arg)
{
    g2_board *b = (g2_board *)arg;
    uint8_t data[RX_MAX];
    while (b->running) {
        int n = recvfrom(b->sock, (char *)data, sizeof data, 0, NULL, NULL);
        const uint8_t *body = data + 8;
        size_t blen;
        uint32_t seq;
        uint8_t typ;
        if (n == SOCKET_ERROR) {
            int e = WSAGetLastError();
            if (e == WSAETIMEDOUT || e == WSAECONNRESET || e == WSAEMSGSIZE) continue;
            if (!b->running) break;
            Sleep(1);
            continue;
        }
        if (n < 8 || data[0] != (MAGIC & 255) || data[1] != (MAGIC >> 8) || data[2] != VERSION) continue;
        typ = data[3];
        seq = r32(data + 4);
        blen = (size_t)n - 8;
        AcquireSRWLockExclusive(&b->lock);
        b->c.n_rx++;
        if (typ == G2_R_ACK && blen >= 16) {
            on_ack(b, r32(body), r32(body + 4), r32(body + 8), r32(body + 12));
        } else if (typ == G2_R_STATUS) {
            int i, nw = (int)(blen / 4);
            if (nw > G2_ST_WORDS) nw = G2_ST_WORDS;
            for (i = 0; i < nw; i++) b->status_words[i] = r32(body + 4 * i);
            b->status_n = nw;
            if (!b->status_count) {
                b->ev_base = b->status_words[G2_ST_B_EVENTS] - (uint32_t)b->c.n_events;
                b->rx_base = b->status_words[G2_ST_REPLIES] - (uint32_t)b->c.n_rx;
            }
            /* replies arrive in the order A sent them, so here, not later, A's count and ours describe the same moment */
            b->ev_missing = (int)(b->status_words[G2_ST_B_EVENTS] - b->ev_base - (uint32_t)b->c.n_events);
            b->rx_missing = (int)(b->status_words[G2_ST_REPLIES] - b->rx_base - (uint32_t)b->c.n_rx);
            b->status_count++;
        } else if (typ == G2_R_EVENT && blen >= 16) {
            uint32_t codew = r32(body);
            b->c.n_events++;
            store_event(b, seq, (codew >> 8) & 0xFF, codew & 0xFF, r32(body + 4), r32(body + 8), r32(body + 12));
        } else if (typ == G2_R_READ_DATA && blen >= 8) {
            on_read_data(b, body, blen);
        } else if ((typ == G2_R_PHY_REG || typ == G2_R_REG) && blen >= 8) {
            b->lreply_w2 = r32(body); b->lreply_w3 = r32(body + 4);
            b->lreply_count++;
        }
        ReleaseSRWLockExclusive(&b->lock);
        WakeAllConditionVariable(&b->cv);
    }
    return 0;
}

/* ---------------------------------------------------------------- send / resend */
static void hb_check(g2_board *b, double now)
{
    if (!b->suspect_valid) return;
    if (!b->ring_count || b->ring[b->ring_head].seq != b->suspect_seq) {
        b->suspect_valid = 0;
    } else if (now - b->suspect_t >= 0.002 && now - b->last_resend > 0.004) {
        b->suspect_valid = 0;
        if (b->resend_req < 1) b->resend_req = 1;
        b->c.n_hb_resend++;
    }
}

static DWORD WINAPI maint_loop(LPVOID arg)
{
    g2_board *b = (g2_board *)arg;
    while (b->running) {
        Sleep(1);
        AcquireSRWLockExclusive(&b->lock);
        if (!b->ring_count) {
            b->resend_req = 0;
            b->suspect_valid = 0;
        } else {
            double now = now_s(), wait;
            int stalled;
            hb_check(b, now);
            wait = b->resend_req == 1 ? 0.002 : 0.005;
            stalled = now - b->last_progress > 0.01;            /* the stall timer: 10 ms */
            if ((b->resend_req && now - b->last_resend > wait) || (stalled && now - b->last_resend > 0.01)) {
                /* the burst goes out under the lock, unlike fbproto: a new seq sent into the middle of it reaches the board
                   ahead of an older resent one = a gap, a NAK, the next burst, ... a Go-Back-N storm that sustains itself with
                   nothing lost */
                int i, n = b->ring_count;
                for (i = 0; i < n; i++) {
                    slot *s = &b->ring[(b->ring_head + i) % WINDOW_MAX];
                    sock_send(b, s->data, s->len);
                }
                b->c.n_resent += (uint64_t)n;
                b->resend_req = 0;
                b->last_resend = now;
            }
        }
        ReleaseSRWLockExclusive(&b->lock);
    }
    return 0;
}

int g2_send_unseq(g2_board *b, uint8_t type, const void *body, size_t len)
{
    uint8_t d[DGRAM_MAX];
    if (len > DGRAM_MAX - 8) return -1;
    write_header(d, type, 0);
    if (len) memcpy(d + 8, body, len);
    if (b->rec) rec_write(b, d, (uint32_t)len + 8);
    else sock_send(b, d, (uint32_t)len + 8);
    return 0;
}

/* _send_seq: wait for room in the window and the credit, assign the seq, keep a copy until acknowledged, send */
static uint32_t send_raw(g2_board *b, uint8_t typ, const void *hdr, size_t hlen, const void *data, size_t dlen, double timeout)
{
    size_t body = hlen + dlen;
    uint32_t need = board_bytes(body), seq;
    double t0;
    slot *s;
    if (body > DGRAM_MAX - 8) { fail(b, "datagram of %u bytes", (unsigned)body); return 0; }
    if (b->rec) {
        uint8_t d[DGRAM_MAX];
        seq = b->next_seq;
        b->next_seq = (b->next_seq + 1) ? b->next_seq + 1 : 1;
        write_header(d, typ, seq);
        if (hlen) memcpy(d + 8, hdr, hlen);
        if (dlen) memcpy(d + 8 + hlen, data, dlen);
        rec_write(b, d, (uint32_t)body + 8);
        b->c.dgrams++;
        return seq;
    }
    t0 = now_s();
    AcquireSRWLockExclusive(&b->lock);
    while (b->ring_count >= b->window || (b->ring_count && b->inflight + need > b->credit)) {
        if (!SleepConditionVariableSRW(&b->cv, &b->lock, 50, 0) && now_s() - t0 > timeout) {
            uint32_t exp = b->ack_exp;
            int n = b->ring_count;
            ReleaseSRWLockExclusive(&b->lock);
            fail(b, "send window stuck (expected %u, %d unacked)", exp, n);
            return 0;
        }
    }
    seq = b->next_seq;
    b->next_seq = (b->next_seq + 1) ? b->next_seq + 1 : 1;
    s = &b->ring[(b->ring_head + b->ring_count) % WINDOW_MAX];
    s->seq = seq; s->len = (uint32_t)body + 8;
    write_header(s->data, typ, seq);
    if (hlen) memcpy(s->data + 8, hdr, hlen);
    if (dlen) memcpy(s->data + 8 + hlen, data, dlen);
    if (!b->ring_count) b->last_progress = now_s();
    b->ring_count++;
    b->inflight += need;
    b->c.dgrams++;
    /* sent under the lock: the slot cannot be recycled by an ACK while sendto reads it */
    sock_send(b, s->data, s->len);
    ReleaseSRWLockExclusive(&b->lock);
    return seq;
}

static int flush_batch(g2_board *b, double timeout)
{
    uint32_t seq;
    if (!b->batch_open || !b->batch_count) return 0;
    seq = send_raw(b, G2_T_BATCH, b->batch, b->batch_size, NULL, 0, timeout);
    b->batch_count = 0; b->batch_size = 0;
    if (!seq) return -1;
    b->c.n_batches++;
    return 0;
}

static int is_b_event_type(uint8_t typ)
{
    return typ == G2_T_FENCE || typ == G2_T_PRESENT || typ == G2_T_SET_MODE || typ == G2_T_MEMTEST;
}

g2_ref g2_send_seq2(g2_board *b, uint8_t type, const void *hdr, size_t hlen, const void *data, size_t dlen)
{
    g2_ref ref = { 0, 0 };
    size_t len = hlen + dlen;
    if (b->batch_open) {
        if (!is_b_event_type(type) || b->b_sub_index) {
            size_t n = 4 + ((len + 3) & ~(size_t)3);
            if (n <= G2_BATCH_MAX) {
                pk k;
                if (b->batch_size + n > G2_BATCH_MAX && flush_batch(b, 10.0) < 0) return ref;
                /* the batch datagram takes next_seq when it is sent: nothing else sends before that (see g2_wait_event) */
                ref.seq = b->next_seq; ref.idx = b->batch_count;
                k.p = b->batch + b->batch_size;
                p8(&k, type); p8(&k, 0); p16(&k, (uint32_t)len);
                if (hlen) memcpy(k.p, hdr, hlen);
                if (dlen) memcpy(k.p + hlen, data, dlen);
                memset(k.p + len, 0, n - 4 - len);
                b->batch_size += (uint32_t)n;
                b->batch_count++;
                b->c.n_batched++;
                return ref;
            }
            /* too large for a BATCH datagram (full WRITE_IMAGE chunks): alone, after what the batch collected */
        }
        if (flush_batch(b, 10.0) < 0) return ref;
    }
    ref.seq = send_raw(b, type, hdr, hlen, data, dlen, 10.0);
    return ref;
}

g2_ref g2_send_seq(g2_board *b, uint8_t type, const void *body, size_t len) { return b->last_ref = g2_send_seq2(b, type, body, len, NULL, 0); }

int g2_batch_begin(g2_board *b)
{
    if (b->batch_open) { fail(b, "batch blocks do not nest"); return -1; }
    b->batch_open = 1; b->batch_count = 0; b->batch_size = 0;
    return 0;
}

int g2_batch_end(g2_board *b)
{
    int r = flush_batch(b, 10.0);
    b->batch_open = 0;
    return r;
}

int g2_batch_flush(g2_board *b) { return flush_batch(b, 10.0); }

int g2_wait_acked(g2_board *b, double timeout)
{
    double t0;
    if (flush_batch(b, timeout) < 0) return -1;
    if (b->rec) return 0;
    t0 = now_s();
    AcquireSRWLockExclusive(&b->lock);
    while (b->ring_count) {
        if (now_s() - t0 > timeout) {
            int n = b->ring_count;
            uint32_t exp = b->ack_exp;
            ReleaseSRWLockExclusive(&b->lock);
            fail(b, "not acknowledged: %d datagrams, board expects %u", n, exp);
            return -1;
        }
        SleepConditionVariableSRW(&b->cv, &b->lock, 50, 0);
    }
    ReleaseSRWLockExclusive(&b->lock);
    return 0;
}

const char *g2_error_name(uint32_t code)
{
    static const char *const names[] = { "?", "unknown type", "bad handle", "out of range", "outside heap", "board B not connected",
        "bad length/alignment", "bad board", "video mode did not lock",
        "render frame aborted (B did not take the render stream in time)",
        "fragment program stack violation (quads of the FLUSH's frame were killed)" };
    return code < sizeof names / sizeof names[0] ? names[code] : "?";
}

static int take_event(g2_board *b, g2_ref ref, g2_event *ev)
{
    int i;
    for (i = 0; i < EVENT_RING; i++) {
        ev_entry *e = &b->events[(b->ev_next + EVENT_RING - 1 - i) % EVENT_RING];
        if (!e->valid) return 0;
        if (e->seq == ref.seq && e->idx == ref.idx) {
            e->valid = 2;                                       /* taken (still a stop for the scan above when overwritten) */
            *ev = e->ev;
            e->seq = 0;
            return 1;
        }
    }
    return 0;
}

static int event_result(g2_board *b, g2_ref ref, const g2_event *ev)
{
    if (ev->code == G2_EV_ERROR) {
        fail(b, "seq %u sub-message %u: %s (type %02X, detail %08X)", ref.seq, ref.idx, g2_error_name(ev->a0), ev->a1, ev->a2);
        return -1;
    }
    return 0;
}

int g2_poll_event(g2_board *b, g2_ref ref, g2_event *ev)
{
    int got;
    if (ref.seq == 0) return -1;
    if (b->batch_open && b->batch_count && ref.seq == b->next_seq && flush_batch(b, 10.0) < 0) return -1;
    AcquireSRWLockExclusive(&b->lock);
    got = take_event(b, ref, ev);
    ReleaseSRWLockExclusive(&b->lock);
    if (!got) return 0;
    return event_result(b, ref, ev) < 0 ? -1 : 1;
}

/* where a missing EVENT got to: under the lock. All of it as of A's last STATUS */
static void event_counts(g2_board *b, char *o, size_t n)
{
    snprintf(o, n, "sent by A and not here: %d events, %d datagrams in all | %u dropped by A",
             b->ev_missing, b->rx_missing, b->status_words[G2_ST_REPLIES_DROPPED]);
}

int g2_wait_event(g2_board *b, g2_ref ref, g2_event *ev, double timeout)
{
    double t0;
    int got;
    if (ref.seq == 0) return -1;
    if (b->rec) {                                               /* recording: nothing answers, nothing waits */
        memset(ev, 0, sizeof *ev);
        return 0;
    }
    if (b->batch_open && b->batch_count && ref.seq == b->next_seq && flush_batch(b, timeout) < 0) return -1;
    t0 = now_s();
    AcquireSRWLockExclusive(&b->lock);
    while (!(got = take_event(b, ref, ev))) {
        if (now_s() - t0 > timeout) {
            char why[160];
            event_counts(b, why, sizeof why);
            ReleaseSRWLockExclusive(&b->lock);
            fail(b, "no EVENT for seq %u sub-message %u: %s", ref.seq, ref.idx, why);
            return -1;
        }
        SleepConditionVariableSRW(&b->cv, &b->lock, 50, 0);
    }
    ReleaseSRWLockExclusive(&b->lock);
    return event_result(b, ref, ev);
}

int g2_wait_done(g2_board *b, g2_ref ref, g2_event *ev, double patience, double timeout)
{
    enum { ASKS = 16 };
    g2_ref ask[ASKS];
    int nask = 0, i, r;
    double t0, next;
    if (ref.seq == 0) return -1;
    if (b->rec) {
        memset(ev, 0, sizeof *ev);
        return 0;
    }
    t0 = now_s();
    next = t0 + patience;
    for (;;) {
        double now;
        if ((r = g2_poll_event(b, ref, ev)) != 0) return r < 0 ? -1 : 0;
        for (i = 0; i < nask && i < ASKS; i++) {
            g2_event e;
            if (g2_poll_event(b, ask[i], &e) > 0) {             /* ref's EVENT never came, and what it asked for is done */
                char why[160], line[220];
                AcquireSRWLockExclusive(&b->lock);
                b->c.n_lost_events++;
                event_counts(b, why, sizeof why);
                ReleaseSRWLockExclusive(&b->lock);
                snprintf(line, sizeof line, "gpu2core: the EVENT of seq %u sub-message %u was lost, %.0f ms: %s", ref.seq, ref.idx, (now_s() - t0) * 1000.0, why);
                if (g_log) g_log(line);
                memset(ev, 0, sizeof *ev);
                return 1;
            }
        }
        now = now_s();
        if (now - t0 > timeout) {
            char why[160];
            AcquireSRWLockExclusive(&b->lock);
            event_counts(b, why, sizeof why);
            ReleaseSRWLockExclusive(&b->lock);
            fail(b, "no EVENT for seq %u sub-message %u, nor for %d fences: %s", ref.seq, ref.idx, nask, why);
            return -1;
        }
        if (now >= next) {
            g2_ref f = g2_fence_async(b, NULL);
            if (f.seq) ask[nask++ % ASKS] = f;
            next = now + patience;
        }
        AcquireSRWLockExclusive(&b->lock);
        SleepConditionVariableSRW(&b->cv, &b->lock, 5, 0);
        ReleaseSRWLockExclusive(&b->lock);
    }
}

int g2_take_errors(g2_board *b, g2_error *out, int max)
{
    int n;
    AcquireSRWLockExclusive(&b->lock);
    n = b->err_count < max ? b->err_count : max;
    if (n) memcpy(out, b->errors, (size_t)n * sizeof out[0]);
    b->err_count = 0;
    ReleaseSRWLockExclusive(&b->lock);
    return n;
}

void g2_get_counters(g2_board *b, g2_counters *c)
{
    AcquireSRWLockExclusive(&b->lock);
    *c = b->c;
    c->next_seq = b->next_seq; c->ack_frame = b->ack_frame;
    ReleaseSRWLockExclusive(&b->lock);
}

/* ---------------------------------------------------------------- session */
/* the board's address: a dotted IPv4 address or a host name (resolved once, IPv4 only) */
static int resolve_ipv4(const char *name, struct in_addr *out)
{
    struct addrinfo hints, *res = NULL;
    unsigned long a = inet_addr(name);
    if (a != INADDR_NONE || strcmp(name, "255.255.255.255") == 0) { out->s_addr = a; return 1; }
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    if (getaddrinfo(name, NULL, &hints, &res) != 0 || !res) return 0;
    *out = ((struct sockaddr_in *)res->ai_addr)->sin_addr;
    freeaddrinfo(res);
    return 1;
}

g2_board *g2_open(const char *ip, int port, const g2_options *opt)
{
    static LONG wsa_done;
    g2_board *b = (g2_board *)calloc(1, sizeof *b);
    if (!b) return NULL;
    b->sock = INVALID_SOCKET;
    b->window = opt && opt->window ? opt->window : WINDOW_MAX;
    if (b->window > WINDOW_MAX) b->window = WINDOW_MAX;
    b->b_sub_index = opt ? opt->b_sub_index : 1;
    b->drop_rx = opt ? opt->drop_rx : 0.0;
    b->rng = 1;
    b->next_seq = 1;
    b->credit = 1u << 20;
    b->last_progress = now_s();
    InitializeSRWLock(&b->lock);
    InitializeConditionVariable(&b->cv);
    if (opt && opt->record_path) {
        b->rec = fopen(opt->record_path, "wb");
        if (!b->rec) { free(b); return NULL; }
        b->credit = 1u << 30;
        return b;
    }
    if (InterlockedExchange(&wsa_done, 1) == 0) {
        WSADATA w;
        if (WSAStartup(MAKEWORD(2, 2), &w) != 0) { free(b); return NULL; }
    }
    b->sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (b->sock == INVALID_SOCKET) { fail(NULL, "socket: %d", WSAGetLastError()); free(b); return NULL; }
    {
        int rcv = 8 << 20, snd = 4 << 20;
        DWORD tmo = 50, ret = 0;
        BOOL off = FALSE;
        struct sockaddr_in me;
        setsockopt(b->sock, SOL_SOCKET, SO_RCVBUF, (const char *)&rcv, sizeof rcv);
        setsockopt(b->sock, SOL_SOCKET, SO_SNDBUF, (const char *)&snd, sizeof snd);
        setsockopt(b->sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof tmo);
        /* an ICMP port unreachable must not fail the blocking receive */
        WSAIoctl(b->sock, SIO_UDP_CONNRESET, &off, sizeof off, NULL, 0, &ret, NULL, NULL);
        memset(&me, 0, sizeof me);
        me.sin_family = AF_INET;
        me.sin_port = htons((u_short)(opt ? opt->bind_port : 0));
        if (bind(b->sock, (struct sockaddr *)&me, sizeof me) == SOCKET_ERROR) {
            fail(NULL, "bind: %d", WSAGetLastError());
            closesocket(b->sock); free(b);
            return NULL;
        }
    }
    memset(&b->addr, 0, sizeof b->addr);
    b->addr.sin_family = AF_INET;
    b->addr.sin_port = htons((u_short)port);
    if (!resolve_ipv4(ip, &b->addr.sin_addr)) {
        fail(NULL, "board address %s: neither an IPv4 address nor a name that resolves", ip);
        closesocket(b->sock); free(b);
        return NULL;
    }
    b->timer_period = timeBeginPeriod(1) == TIMERR_NOERROR;
    b->running = 1;
    {
        /* The threads below run this module's code until the process ends. The module stays loaded from here on: a FreeLibrary of the library (dEQP unloads libEGL
           after each session) would leave them running in unmapped code. They cannot be stopped from DllMain, which holds the loader lock they need to end */
        HMODULE self;
        GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_PIN, (LPCWSTR)(void *)rx_loop, &self);
    }
    b->rx_thread = CreateThread(NULL, 0, rx_loop, b, 0, NULL);
    b->mt_thread = CreateThread(NULL, 0, maint_loop, b, 0, NULL);
    SetThreadPriority(b->rx_thread, THREAD_PRIORITY_ABOVE_NORMAL);
    SetThreadPriority(b->mt_thread, THREAD_PRIORITY_ABOVE_NORMAL);
    return b;
}

void g2_close(g2_board *b)
{
    if (!b) return;
    free(b->vi); free(b->owner);
    if (b->rec) { fclose(b->rec); free(b); return; }
    InterlockedExchange(&b->running, 0);
    WaitForSingleObject(b->rx_thread, 1000);
    WaitForSingleObject(b->mt_thread, 1000);
    CloseHandle(b->rx_thread);
    CloseHandle(b->mt_thread);
    closesocket(b->sock);
    if (b->timer_period) timeEndPeriod(1);
    free(b);
}

int g2_hello(g2_board *b, int new_session)
{
    uint8_t body[4];
    uint32_t n0;
    int tries;
    pk k = { body };
    p32(&k, new_session ? 1 : 0);
    AcquireSRWLockExclusive(&b->lock);
    b->ring_count = 0; b->ring_head = 0; b->inflight = 0;
    b->next_seq = 1;
    memset(b->events, 0, sizeof b->events);
    b->ev_next = 0;
    n0 = b->ack_count;
    ReleaseSRWLockExclusive(&b->lock);
    for (tries = 0; tries < 10; tries++) {
        g2_send_unseq(b, G2_T_HELLO, body, 4);
        AcquireSRWLockExclusive(&b->lock);
        SleepConditionVariableSRW(&b->cv, &b->lock, 200, 0);
        if (b->ack_count != n0) {
            int exp = (int)b->ack_exp;
            ReleaseSRWLockExclusive(&b->lock);
            return exp;
        }
        ReleaseSRWLockExclusive(&b->lock);
    }
    fail(b, "no ACK to HELLO");
    return -1;
}

int g2_status(g2_board *b, uint32_t out[G2_ST_WORDS], int *nwords)
{
    uint32_t n0;
    int tries;
    AcquireSRWLockExclusive(&b->lock);
    n0 = b->status_count;
    ReleaseSRWLockExclusive(&b->lock);
    for (tries = 0; tries < 3; tries++) {
        double t0 = now_s();
        g2_send_unseq(b, G2_T_STATUS_REQ, NULL, 0);
        AcquireSRWLockExclusive(&b->lock);
        while (b->status_count == n0 && now_s() - t0 < 0.333)
            SleepConditionVariableSRW(&b->cv, &b->lock, 20, 0);
        if (b->status_count != n0) {
            memset(out, 0, G2_ST_WORDS * 4);
            memcpy(out, b->status_words, (size_t)b->status_n * 4);
            if (nwords) *nwords = b->status_n;
            ReleaseSRWLockExclusive(&b->lock);
            return 0;
        }
        ReleaseSRWLockExclusive(&b->lock);
    }
    fail(b, "no STATUS");
    return -1;
}

int g2_last_status(g2_board *b, uint32_t out[G2_ST_WORDS])
{
    int ok;
    AcquireSRWLockExclusive(&b->lock);
    ok = b->status_count != 0;
    if (ok) {
        memset(out, 0, G2_ST_WORDS * 4);
        memcpy(out, b->status_words, (size_t)b->status_n * 4);
    }
    ReleaseSRWLockExclusive(&b->lock);
    return ok ? 0 : -1;
}

int g2_check_boards(g2_board *b, int min_a_build)
{
    uint32_t st[G2_ST_WORDS];
    if (g2_status(b, st, NULL) < 0) return -1;
    if (st[G2_ST_LAYOUT] < 7) { fail(b, "STATUS layout %u: the boards run bitstreams older than A 57 / B 55", st[G2_ST_LAYOUT]); return -1; }
    if ((int)st[G2_ST_BUILD] < min_a_build) { fail(b, "board A build %u, need %d", st[G2_ST_BUILD], min_a_build); return -1; }
    if (!G2_LINK_TRAINED(st[G2_ST_LINK_FLAGS]) || !G2_LINK_B_ALIVE(st[G2_ST_LINK_FLAGS])) {
        fail(b, "the board link is not up / B is not alive");
        return -1;
    }
    return 0;
}

static int local_msg(g2_board *b, uint8_t typ, const void *body, size_t len, uint32_t *w3)
{
    int tries;
    for (tries = 0; tries < 3; tries++) {
        uint32_t n0;
        double t0;
        AcquireSRWLockExclusive(&b->lock);
        n0 = b->lreply_count;
        ReleaseSRWLockExclusive(&b->lock);
        g2_send_unseq(b, typ, body, len);
        t0 = now_s();
        AcquireSRWLockExclusive(&b->lock);
        while (b->lreply_count == n0 && now_s() - t0 < 0.5)
            SleepConditionVariableSRW(&b->cv, &b->lock, 20, 0);
        if (b->lreply_count != n0) {
            *w3 = b->lreply_w3;
            ReleaseSRWLockExclusive(&b->lock);
            return 0;
        }
        ReleaseSRWLockExclusive(&b->lock);
    }
    fail(b, "no reply to local message %02X", typ);
    return -1;
}

int g2_reg_read(g2_board *b, uint32_t addr, uint32_t *value)
{
    uint8_t d[8];
    pk k = { d };
    p8(&k, 0); p8(&k, 0); p16(&k, addr); p32(&k, 0);
    return local_msg(b, G2_T_REG, d, 8, value);
}

int g2_reg_write(g2_board *b, uint32_t addr, uint32_t value)
{
    uint8_t d[8];
    uint32_t dummy;
    pk k = { d };
    p8(&k, 1); p8(&k, 0); p16(&k, addr); p32(&k, value);
    return local_msg(b, G2_T_REG, d, 8, &dummy);
}

/* ---------------------------------------------------------------- reads (fbproto._read) */
typedef void (*make_req)(void *ctx, size_t s, size_t n, uint32_t tag, uint8_t *body, size_t *blen);

static int cmp_size(const void *a, const void *b)
{
    size_t x = *(const size_t *)a, y = *(const size_t *)b;
    return x < y ? -1 : x > y;
}

static int read_stream(g2_board *b, uint8_t typ, make_req make, void *ctx, void *out, size_t total, int rel, size_t base)
{
    const size_t CH = G2_RD_CHUNK;
    const size_t max_req = 256 * 1024 / CH * CH;
    const int inflight = 2;
    const double tail = 0.02, quiet = 1.0, timeout = 10.0;
    size_t nch = (total + CH - 1) / CH, ntodo = 0, todo_head, cap, *lost = NULL, i, seen = nch;
    rd_range *todo = NULL;
    int rc = 0;
    double t_data;

    if (total == 0) return 0;
    cap = nch + 2 * (total / max_req + 2) + 16;
    todo = (rd_range *)malloc(cap * 2 * sizeof *todo);
    lost = (size_t *)malloc(nch * sizeof *lost);
    if (!todo || !lost) { free(todo); free(lost); fail(b, "read: out of memory"); return -1; }
    /* the deque: head in the middle so that re-requests can go first */
    todo_head = cap;
    {
        size_t s;
        for (s = 0; s < total; s += max_req) {
            todo[todo_head + ntodo].s = s;
            todo[todo_head + ntodo].n = total - s < max_req ? total - s : max_req;
            ntodo++;
        }
    }
    if (b->rec) {                                               /* recording: the requests of a read without losses */
        for (i = 0; i < ntodo; i++) {
            uint8_t body[32];
            size_t blen = 0;
            g_tag = (g_tag + 1) ? g_tag + 1 : 1;
            make(ctx, todo[todo_head + i].s, todo[todo_head + i].n, g_tag, body, &blen);
            g2_send_seq(b, typ, body, blen);
            flush_batch(b, 10.0);
        }
        memset(out, 0, total);
        free(todo); free(lost);
        return 0;
    }
    AcquireSRWLockExclusive(&b->lock);
    b->rd_out = (uint8_t *)out;
    b->rd_total = total; b->rd_base = base; b->rd_rel = rel;
    b->rd_missing = (uint8_t *)malloc(nch);
    memset(b->rd_missing, 1, nch);
    b->rd_nmissing = nch;
    b->rd_capstarts = 64;
    b->rd_starts = (size_t *)malloc(b->rd_capstarts * sizeof(size_t));
    b->rd_nstarts = 0;
    g_tag = (g_tag + 1) ? g_tag + 1 : 1;
    b->rd_tag0 = g_tag;
    b->rd_nreqs = 0;
    b->rd_active = 1;
    ReleaseSRWLockExclusive(&b->lock);
    t_data = now_s();

    while (rc == 0) {
        double now;
        size_t nlost = 0;
        int later_has_data = 0, k;
        while (ntodo) {
            uint8_t body[32];
            size_t blen = 0;
            uint32_t tag;
            rd_range t;
            int full;
            AcquireSRWLockExclusive(&b->lock);
            full = b->rd_nreqs >= inflight;
            if (!full) {
                rd_req *r = &b->rd_reqs[b->rd_nreqs];
                t = todo[todo_head++]; ntodo--;
                tag = b->rd_tag0 + (uint32_t)b->rd_nstarts;
                if (b->rd_nstarts == b->rd_capstarts) {
                    b->rd_capstarts *= 2;
                    b->rd_starts = (size_t *)realloc(b->rd_starts, b->rd_capstarts * sizeof(size_t));
                }
                b->rd_starts[b->rd_nstarts++] = t.s;
                r->s = t.s; r->n = t.n; r->tag = tag; r->t_sent = now_s(); r->t_last = 0.0; r->top = -1; r->has_data = 0;
                b->rd_nreqs++;
            }
            ReleaseSRWLockExclusive(&b->lock);
            if (full) break;
            make(ctx, t.s, t.n, tag, body, &blen);
            if (!g2_send_seq(b, typ, body, blen).seq || flush_batch(b, 10.0) < 0) { rc = -1; break; }
        }
        if (rc) break;
        AcquireSRWLockExclusive(&b->lock);
        if (b->rd_nmissing == 0) { ReleaseSRWLockExclusive(&b->lock); break; }
        SleepConditionVariableSRW(&b->cv, &b->lock, 2, 0);
        now = now_s();
        if (b->rd_nmissing != seen) { seen = b->rd_nmissing; t_data = now; }
        /* loss detection, newest request first: know whether a later one produced */
        for (k = b->rd_nreqs - 1; k >= 0; k--) {
            rd_req *r = &b->rd_reqs[k];
            size_t s = r->s, n = r->n, p, first_lost = nlost, maxbelow = 0;
            int mine = 0, below = 0, had = r->has_data;
            double t_last = r->t_last;
            /* the last term: all it produced was lost, and with nothing later to show that up only time can tell */
            int late = later_has_data || (had && now - t_last > tail) || (!had && now - r->t_sent > quiet);
            for (p = s; p < s + n; p += CH) {
                if (!b->rd_missing[p / CH]) continue;
                mine++;
                if (late) {
                    lost[nlost++] = p;
                } else if ((long long)p < r->top) {
                    lost[nlost++] = p;
                    below++;
                    maxbelow = p;
                }
            }
            if (!mine) {
                memmove(r, r + 1, (size_t)(b->rd_nreqs - k - 1) * sizeof *r);
                b->rd_nreqs--;
            } else if (late) {
                if (later_has_data) b->c.rd_later += nlost - first_lost; else b->c.rd_tail += nlost - first_lost;
                memmove(r, r + 1, (size_t)(b->rd_nreqs - k - 1) * sizeof *r);
                b->rd_nreqs--;
            } else if (below) {
                b->c.rd_below += (uint64_t)below;
                r->s = maxbelow + CH;                           /* the request now only expects what follows */
                r->n = s + n - r->s;
            }
            if (had) later_has_data = 1;
        }
        if (now - t_data > timeout) {
            ReleaseSRWLockExclusive(&b->lock);
            fail(b, "read: no data for %.0f s (%u of %u chunks still to come)", timeout, (unsigned)seen, (unsigned)nch);
            rc = -1;
            break;
        }
        ReleaseSRWLockExclusive(&b->lock);
        if (nlost) {
            size_t a, e, nruns = 0;
            b->c.n_rerequested += nlost;
            qsort(lost, nlost, sizeof *lost, cmp_size);
            a = e = lost[0];
            /* runs of adjacent chunks; re-requests go first, in position order */
            {
                rd_range *runs = (rd_range *)malloc((nlost + 1) * sizeof *runs);
                for (i = 1; i < nlost; i++) {
                    if (lost[i] == e + CH && lost[i] + CH - a <= max_req) {
                        e = lost[i];
                    } else {
                        runs[nruns].s = a; runs[nruns].n = (e + CH < total ? e + CH : total) - a; nruns++;
                        a = e = lost[i];
                    }
                }
                runs[nruns].s = a; runs[nruns].n = (e + CH < total ? e + CH : total) - a; nruns++;
                if (todo_head < nruns) {                        /* room in front of the deque */
                    size_t nh = cap;
                    memmove(todo + nh, todo + todo_head, ntodo * sizeof *todo);
                    todo_head = nh;
                }
                for (i = nruns; i-- > 0;) todo[--todo_head] = runs[i], ntodo++;
                free(runs);
            }
        }
    }
    AcquireSRWLockExclusive(&b->lock);
    b->rd_active = 0;
    free(b->rd_missing); b->rd_missing = NULL;
    free(b->rd_starts); b->rd_starts = NULL;
    b->rd_nstarts = 0;
    g_tag = b->rd_tag0 + (uint32_t)b->rd_capstarts;             /* late chunks of this read can never match the next one */
    ReleaseSRWLockExclusive(&b->lock);
    free(todo); free(lost);
    return rc;
}

/* ---------------------------------------------------------------- resources and transfers */
g2_ref g2_create_buffer(g2_board *b, uint16_t handle, uint32_t base, uint32_t size, uint16_t usage)
{
    uint8_t d[12];
    pk k = { d };
    p16(&k, handle); p16(&k, usage); p32(&k, base); p32(&k, size);
    return g2_send_seq(b, G2_T_CREATE_BUFFER, d, sizeof d);
}

g2_ref g2_destroy(g2_board *b, uint16_t handle)
{
    uint8_t d[4];
    pk k = { d };
    g2_ref none = { 0, 0 };
    if ((handle & 0xC000) == 0xC000 && b->vi && b->vi[handle & (G2_HANDLES_MAX - 1)].live) {     /* an image: its table entry, if it has one */
        vimg *v = &b->vi[handle & (G2_HANDLES_MAX - 1)];
        v->live = 0;
        if (v->phys == VIMG_NONE) return none;
        b->owner[v->phys] = VIMG_NONE;
        handle = (uint16_t)(0xC000u | v->phys);
        v->phys = VIMG_NONE;
    }
    p16(&k, handle); p16(&k, 0);
    return g2_send_seq(b, G2_T_DESTROY, d, sizeof d);
}

int g2_write_buffer(g2_board *b, uint16_t handle, uint32_t offset, const void *data, size_t len)
{
    size_t i;
    const uint8_t *p = (const uint8_t *)data;
    for (i = 0; i < len; i += G2_MAX_DATA) {
        uint8_t d[8];
        pk k = { d };
        size_t n = len - i < G2_MAX_DATA ? len - i : G2_MAX_DATA;
        p16(&k, handle); p16(&k, 0); p32(&k, offset + (uint32_t)i);
        if (!g2_send_seq2(b, G2_T_WRITE_BUFFER, d, 8, p + i, n).seq) return -1;
    }
    return 0;
}

int g2_mem_write(g2_board *b, uint32_t addr, const void *data, size_t len, int board)
{
    size_t i;
    const uint8_t *p = (const uint8_t *)data;
    for (i = 0; i < len; i += G2_MAX_DATA) {
        uint8_t d[8];
        pk k = { d };
        size_t n = len - i < G2_MAX_DATA ? len - i : G2_MAX_DATA;
        p8(&k, (uint32_t)board); p8(&k, 0); p16(&k, 0); p32(&k, addr + (uint32_t)i);
        if (!g2_send_seq2(b, G2_T_MEM_WRITE, d, 8, p + i, n).seq) return -1;
    }
    return 0;
}

typedef struct rb_ctx { uint16_t handle; uint32_t offset; } rb_ctx;
static void make_read_buffer(void *ctx, size_t s, size_t n, uint32_t tag, uint8_t *body, size_t *blen)
{
    rb_ctx *c = (rb_ctx *)ctx;
    pk k = { body };
    p16(&k, c->handle); p16(&k, 0); p32(&k, c->offset + (uint32_t)s); p32(&k, (uint32_t)n); p32(&k, tag);
    *blen = 16;
}

int g2_read_buffer(g2_board *b, uint16_t handle, uint32_t offset, void *out, size_t len)
{
    rb_ctx c = { handle, offset };
    return read_stream(b, G2_T_READ_BUFFER, make_read_buffer, &c, out, len, 1, 0);
}

typedef struct mr_ctx { uint32_t addr; int board; } mr_ctx;
static void make_mem_read(void *ctx, size_t s, size_t n, uint32_t tag, uint8_t *body, size_t *blen)
{
    mr_ctx *c = (mr_ctx *)ctx;
    pk k = { body };
    p8(&k, (uint32_t)c->board); p8(&k, 0); p16(&k, 0); p32(&k, c->addr + (uint32_t)s); p32(&k, (uint32_t)n); p32(&k, tag);
    *blen = 16;
}

int g2_mem_read(g2_board *b, uint32_t addr, void *out, size_t len, int board)
{
    mr_ctx c = { addr, board };
    return read_stream(b, G2_T_MEM_READ, make_mem_read, &c, out, len, 1, 0);
}

int g2_memtest(g2_board *b, uint32_t addr, uint32_t len, uint32_t seed, uint32_t flags, int board, double timeout,
               uint32_t *errors, uint32_t *first_bad)
{
    uint8_t d[20];
    pk k = { d };
    g2_ref r;
    g2_event ev;
    p8(&k, (uint32_t)board); p8(&k, 0); p16(&k, 0); p32(&k, addr); p32(&k, len); p32(&k, seed); p32(&k, flags);
    r = g2_send_seq(b, G2_T_MEMTEST, d, sizeof d);
    if (g2_wait_event(b, r, &ev, timeout) < 0) return -1;
    if (errors) *errors = ev.a0;
    if (first_bad) *first_bad = ev.a1;
    return 0;
}

/* ---------------------------------------------------------------- display */
int g2_set_mode(g2_board *b, uint32_t mode, double timeout)
{
    uint8_t d[4];
    pk k = { d };
    g2_ref r;
    g2_event ev;
    p32(&k, mode);
    r = g2_send_seq(b, G2_T_SET_MODE, d, sizeof d);
    if (b->rec) return 0;
    if (g2_wait_event(b, r, &ev, timeout) < 0) return -1;
    if (ev.code != G2_EV_MODE || ev.a0 != mode) { fail(b, "set_mode %u: unexpected event %u", mode, ev.code); return -1; }
    return 0;
}

const int G2_MODE_W[G2_NMODES] = { 640, 800, 1280, 1280, 1024 }, G2_MODE_H[G2_NMODES] = { 480, 600, 720, 800, 768 };
const double G2_MODE_HZ[G2_NMODES] = { 59.896, 60.317, 60.000, 59.910, 60.004 };   /* pixel clock / (h total x v total) */

int g2_mode_of_flags(uint32_t b_flags)
{
    uint32_t m = G2_B_FLAG_MODE(b_flags);
    return m < G2_NMODES ? (int)m : 2;
}

int g2_mode_fit(int w, int h, int with_xga)
{
    static const int BY_SIZE[G2_NMODES] = { 0, 1, 4, 2, 3 };
    int i;
    for (i = 0; i < G2_NMODES; i++) {
        int m = BY_SIZE[i];
        if ((m != 4 || with_xga) && G2_MODE_W[m] >= w && G2_MODE_H[m] >= h) return m;
    }
    return 3;
}

/* SET_MODE 4 is the probe: a B before it answers with an error EVENT of type SET_MODE, which is taken from the error list (as g2r_has_3d's probe) and
   remembered. Any other failure of mode 4 (no answer) falls back as well, for this call only */
int g2_mode_select(g2_board *b, int w, int h, int *mode)
{
    int m = g2_mode_fit(w, h, !b->no_xga);
    if (m == *mode) return 0;
    if (m == 4) {
        g2_error e[8];
        int n, i;
        g2_take_errors(b, e, 8);
        if (g2_set_mode(b, 4, 5.0) == 0) { *mode = 4; return 0; }
        n = g2_take_errors(b, e, 8);
        for (i = 0; i < n; i++) if (e[i].type == G2_T_SET_MODE) b->no_xga = 1;
        if (g_log) g_log(b->no_xga ? "set_mode 4: board B has no 1024 x 768 mode (B 124 or older); 1024 x 768 shows 1:1 in 1280 x 800"
                                   : "set_mode 4: no answer; 1280 x 800 this time");
        m = g2_mode_fit(w, h, 0);
        if (m == *mode) return 0;
    }
    if (g2_set_mode(b, (uint32_t)m, 5.0) < 0) return -1;
    *mode = m;
    return 0;
}

g2_ref g2_present(g2_board *b, uint16_t handle, uint32_t frame_id, int at_once, int hold)
{
    uint8_t d[8];
    pk k = { d };
    b->vstamp++;
    p16(&k, phys_of(b, handle)); p16(&k, (at_once ? G2_PRES_AT_ONCE : 0u) | (hold ? G2_PRES_HOLD : 0u)); p32(&k, frame_id);
    return g2_send_seq(b, G2_T_PRESENT, d, sizeof d);
}

int g2_present_wait(g2_board *b, g2_ref ref, uint32_t *fid, uint32_t *vsyncs, uint32_t *dropped, double timeout)
{
    g2_event ev;
    if (g2_wait_event(b, ref, &ev, timeout) < 0) return -1;
    if (b->rec) { ev.code = G2_EV_PRESENT; }
    if (ev.code != G2_EV_PRESENT) { fail(b, "present: unexpected event %u", ev.code); return -1; }
    if (fid) *fid = ev.a0;
    if (vsyncs) *vsyncs = ev.a1;
    if (dropped) *dropped = ev.a2;
    return 0;
}

/* ---------------------------------------------------------------- board A's geometry */
g2_ref g2_frame_begin(g2_board *b)
{
    uint8_t d[4] = { 0, 0, 0, 0 };
    return g2_send_seq(b, G2_T_FRAME_BEGIN, d, sizeof d);
}

g2_ref g2_vs_program(g2_board *b, uint16_t handle, uint32_t offset, uint32_t n_instr)
{
    uint8_t d[12];
    pk k = { d };
    p16(&k, handle); p16(&k, 0); p32(&k, offset); p32(&k, n_instr);
    return g2_send_seq(b, G2_T_VS_PROGRAM, d, sizeof d);
}

g2_ref g2_vs_uniforms(g2_board *b, uint32_t first, const uint32_t *v, uint32_t count)
{
    uint8_t d[4];
    pk k = { d };
    p16(&k, first); p16(&k, count);
    return g2_send_seq2(b, G2_T_VS_UNIFORMS, d, 4, v, (size_t)count * 16);
}

g2_ref g2_vertex_attr(g2_board *b, int index, int enabled, int size, uint16_t handle, uint32_t stride, uint32_t offset,
                      const uint32_t c[4], int type, int normalized)
{
    uint8_t d[32];
    pk k = { d };
    /* word 0 = {normalized[24], type[22:20], size[18:16], enabled[8], index[3:0]} */
    p32(&k, (uint32_t)(index & 0xF) | (uint32_t)(enabled ? 1 : 0) << 8 | (uint32_t)(size & 7) << 16 | (uint32_t)(type & 7) << 20 |
             (uint32_t)(normalized ? 1 : 0) << 24);
    p16(&k, handle); p16(&k, 0); p32(&k, stride); p32(&k, offset);
    p32(&k, c[0]); p32(&k, c[1]); p32(&k, c[2]); p32(&k, c[3]);
    return g2_send_seq(b, G2_T_VERTEX_ATTR, d, sizeof d);
}

/* rect: the draw's rectangle x0, y0, x1, y1 in pixels — its scissor box, the same as its DRAW_STATE's and only when the scissor test is on there — or NULL for the whole target; words 13, 14.
   Board A drops what has no pixel in it and walks inside it. A board A without the rectangle reads the first 13 words and drops
   the two as the unread tail of the sub-message, so every board gets the same bytes */
g2_ref g2_vs_state(g2_board *b, const double vp[4], int cull, int front_cw, uint32_t fsmask, uint32_t width, uint32_t height,
                   uint32_t draw_id, double line_width, double depth_near, double depth_far, const uint32_t rect[4])
{
    return g2_vs_state2(b, vp, cull, front_cw, fsmask, width, height, draw_id, line_width, depth_near, depth_far, 0, rect);
}

/* z01 (board A builds from G2_A_ZCLIP0 on): the program's z runs 0 .. w, not -w .. w — board A clips at z >= 0 (word 4 bit 24) and window z = depth_near + ndc_z (depth_far - depth_near).
   For a library that keeps depth reversed (far at 0): a float is finest near 0, and every stage that carries a far vertex's depth is FP24. A board A without the flag ignores the bit and would clip at z >= -w */
g2_ref g2_vs_state2(g2_board *b, const double vp[4], int cull, int front_cw, uint32_t fsmask, uint32_t width, uint32_t height,
                    uint32_t draw_id, double line_width, double depth_near, double depth_far, int z01, const uint32_t rect[4])
{
    return g2_vs_state3(b, vp, cull, front_cw, fsmask, width, height, draw_id, line_width, depth_near, depth_far, z01, 0, rect);
}

/* a_flags (board A builds with the coarse depth; earlier ones ignore the bits): word 4 [29:25] = {LAZY, INV, STRICT, CULL, OCC} — the draw's
   coarse-depth flags (g2_render.c zc_flags: a promise about its depth state) and, from the last VS_STATE before a FLUSH, the frame's lazy TRI items */
g2_ref g2_vs_state3(g2_board *b, const double vp[4], int cull, int front_cw, uint32_t fsmask, uint32_t width, uint32_t height,
                    uint32_t draw_id, double line_width, double depth_near, double depth_far, int z01, uint32_t a_flags, const uint32_t rect[4])
{
    uint8_t d[60];
    pk k = { d };
    int i, lw = (int)rint(line_width);
    uint32_t gb[4];
    if (lw < 1) lw = 1;
    if (lw > 64) lw = 64;
    for (i = 0; i < 2; i++) {                                   /* the guard band's NDC bounds: (+-G - o) / s, ordered */
        double s = vp[2 * i], o = vp[2 * i + 1], lo = (-G2_GUARD_BAND - o) / s, hi = (G2_GUARD_BAND - o) / s;
        gb[2 * i] = g2_fp24(lo < hi ? lo : hi);
        gb[2 * i + 1] = g2_fp24(lo < hi ? hi : lo);
    }
    for (i = 0; i < 4; i++) p32(&k, g2_fp24(vp[i]));
    p16(&k, (uint32_t)cull | (uint32_t)(front_cw ? 1 : 0) << 8 | (uint32_t)lw << 9);
    p8(&k, fsmask); p8(&k, (z01 ? 1u : 0u) | (a_flags & 0x1Fu) << 1); p16(&k, width); p16(&k, height); p32(&k, draw_id);
    if (z01) { p32(&k, g2_fp24(depth_far - depth_near)); p32(&k, g2_fp24(depth_near)); }
    else { p32(&k, g2_fp24((depth_far - depth_near) / 2)); p32(&k, g2_fp24((depth_near + depth_far) / 2)); }
    for (i = 0; i < 4; i++) p32(&k, gb[i]);
    p16(&k, rect ? rect[0] : 0); p16(&k, rect ? rect[1] : 0); p16(&k, rect ? rect[2] : width); p16(&k, rect ? rect[3] : height);
    return g2_send_seq(b, G2_T_VS_STATE, d, sizeof d);
}

g2_ref g2_draw(g2_board *b, int mode, uint32_t count, uint32_t first, int indexed, uint16_t index_handle, uint32_t index_offset,
               int index_u8)
{
    uint8_t d[20];
    pk k = { d };
    p8(&k, (uint32_t)mode); p8(&k, indexed ? 1 : 0); p8(&k, index_u8 ? 1 : 0); p8(&k, 0);
    p16(&k, indexed ? index_handle : 0); p16(&k, 0); p32(&k, index_offset); p32(&k, first); p32(&k, count);
    return g2_send_seq(b, G2_T_DRAW, d, sizeof d);
}

/* ---------------------------------------------------------------- board B's per-draw state and frames */
g2_ref g2_draw_state(g2_board *b, uint32_t draw_id, const uint32_t h[23], const uint32_t *uniforms, uint32_t n)
{
    uint8_t d[4 + 23 * 4];
    pk k = { d };
    int i;
    p32(&k, draw_id & 0x1FFF);
    b->vstamp++;
    for (i = 0; i < 23; i++) p32(&k, i >= 15 && (h[i] & 0xFFFF) ? (h[i] & 0xFFFF0000u) | phys_of(b, (uint16_t)h[i]) : h[i]);   /* h15..h22: samplers */
    return g2_send_seq2(b, G2_T_DRAW_STATE, d, sizeof d, uniforms, (size_t)n * 16);
}

g2_ref g2_flush(g2_board *b, uint16_t handle, uint32_t flags, const uint32_t c[4], uint32_t clear_depth, uint32_t clear_stencil,
                const uint32_t sc[4], uint16_t ds_handle, uint32_t stencil_mask)
{
    uint8_t d[28];
    pk k = { d };
    b->vstamp++;
    handle = phys_of(b, handle);
    if (ds_handle) ds_handle = phys_of(b, ds_handle);
    p32(&k, (uint32_t)handle | (flags & 0xFFFF) << 16);
    p32(&k, (c[0] & 0x1FFF) | (c[1] & 0x1FFF) << 13);
    p32(&k, (c[2] & 0x1FFF) | (c[3] & 0x1FFF) << 13);
    p32(&k, (clear_depth & 0xFFFFFF) | (clear_stencil & 0xFF) << 24);
    p32(&k, (sc[0] & 0x1FFF) | (sc[1] & 0x1FFF) << 16);
    p32(&k, (sc[2] & 0x1FFF) | (sc[3] & 0x1FFF) << 16);
    p32(&k, (uint32_t)ds_handle | (stencil_mask & 0xFF) << 16);
    return g2_send_seq(b, G2_T_FLUSH, d, sizeof d);
}

g2_ref g2_fence_async(g2_board *b, uint32_t *fid)
{
    uint8_t d[4];
    pk k = { d };
    uint32_t id = ++b->fence_id;
    p32(&k, id);
    if (fid) *fid = id;
    return g2_send_seq(b, G2_T_FENCE, d, sizeof d);
}

int g2_fence(g2_board *b, double timeout)
{
    uint32_t id;
    g2_ref r = g2_fence_async(b, &id);
    g2_event ev;
    int got;
    if (b->rec) return r.seq ? 0 : -1;
    got = g2_wait_done(b, r, &ev, 0.25, timeout);
    if (got < 0) return -1;
    if (!got && (ev.code != G2_EV_FENCE || ev.a0 != id)) { fail(b, "fence %u: unexpected event %u (%u)", id, ev.code, ev.a0); return -1; }
    return 0;
}

/* ---------------------------------------------------------------- images on board B */
uint32_t g2_storage_bpp(uint32_t fmt)
{
    switch (fmt) {
    case G2_FMT_RGB565: case G2_FMT_RGBA4444: case G2_FMT_RGBA5551: return 2;
    case G2_FMT_P8: return 1;
    default: return 4;
    }
}

uint32_t g2_transfer_bpp(uint32_t tfmt, uint32_t storage_fmt)
{
    if (tfmt == G2_TF_RAW) return g2_storage_bpp(storage_fmt);
    if (tfmt == G2_TF_RGB565) return 2;
    if (tfmt == G2_TF_RGB888 || tfmt == G2_TF_BGR888) return 3;
    return 4;
}

/* ---------------------------------------------------------------- image handles as a cache of board B's table
 * The libraries' image handles are virtual: 0xC000 | index, index < G2_HANDLES_MAX. Board B's descriptor table holds B_IMAGES entries, so this
 * layer keeps every image's descriptor here and gives it a table entry when a command names it. When the
 * table is full, the entry of the image named longest ago is taken back (DESTROY, its memory untouched) and the image gets one again when it is next
 * named. That is safe at any time: board B turns a sampler into an address when the DRAW_STATE arrives, a FLUSH looks its images up when it runs,
 * and commands run in order. An image keeps its own index as its entry while that is free, so a program within the table sends the same bytes as
 * without this layer. Handles never created here (board B's built-in test images 0..15) pass unchanged. */

static int vimg_ready(g2_board *b)
{
    if (b->vi) return 1;
    b->vi = (vimg *)calloc(G2_HANDLES_MAX, sizeof *b->vi);
    b->owner = (uint16_t *)malloc(G2_HANDLES_MAX * sizeof *b->owner);
    if (!b->vi || !b->owner) { free(b->vi); free(b->owner); b->vi = NULL; b->owner = NULL; return 0; }
    memset(b->owner, 0xFF, G2_HANDLES_MAX * sizeof *b->owner);
    return 1;
}

static g2_ref send_create(g2_board *b, uint16_t phys, const vimg *v)
{
    uint8_t d[20];
    pk k = { d };
    p16(&k, 0xC000u | phys); p16(&k, v->usage); p32(&k, v->base); p16(&k, v->pitch); p8(&k, v->fmt);
    p8(&k, (uint32_t)(v->origin & 1) | (uint32_t)(v->samples & 3) << 3);
    p16(&k, v->width); p16(&k, v->height); p8(&k, v->levels); p8(&k, v->layers); p16(&k, v->vol);
    return g2_send_seq(b, G2_T_CREATE_IMAGE, d, sizeof d);
}

static void send_destroy(g2_board *b, uint16_t handle)
{
    uint8_t d[4];
    pk k = { d };
    p16(&k, handle); p16(&k, 0);
    g2_send_seq(b, G2_T_DESTROY, d, sizeof d);
}

/* a table entry for image idx: its own index if free, else any free one, else the one named longest ago (not by the command being built) */
static int vimg_place(g2_board *b, uint32_t idx)
{
    vimg *v = &b->vi[idx];
    uint32_t i, best = VIMG_NONE, oldest = 0;
    if (idx < B_IMAGES && b->owner[idx] == VIMG_NONE) best = idx;
    for (i = 16; best == VIMG_NONE && i < B_IMAGES; i++) if (b->owner[i] == VIMG_NONE) best = i;
    if (best == VIMG_NONE) {
        for (i = 16; i < B_IMAGES; i++) {
            const vimg *o = &b->vi[b->owner[i]];
            if (o->stamp == b->vstamp) continue;
            if (best == VIMG_NONE || b->vstamp - o->stamp > oldest) { best = i; oldest = b->vstamp - o->stamp; }
        }
        if (best == VIMG_NONE) return -1;                           /* every entry is named by this one command: cannot happen with 8 samplers */
        send_destroy(b, (uint16_t)(0xC000u | best));
        b->vi[b->owner[best]].phys = VIMG_NONE;
        b->c.img_evictions++;
    }
    b->owner[best] = (uint16_t)idx;
    v->phys = (uint16_t)best;
    send_create(b, (uint16_t)best, v);
    return 0;
}

/* the handle board B knows an image by, giving it a table entry if it has none */
static uint16_t phys_of(g2_board *b, uint16_t handle)
{
    uint32_t idx = handle & (G2_HANDLES_MAX - 1);
    vimg *v;
    if ((handle & 0xC000) != 0xC000 || !b->vi || !b->vi[idx].live) return handle;
    v = &b->vi[idx];
    v->stamp = b->vstamp;
    if (v->phys == VIMG_NONE && vimg_place(b, idx) < 0) return handle;
    return (uint16_t)(0xC000u | v->phys);
}

static g2_ref vimg_create(g2_board *b, uint16_t handle, uint32_t base, uint32_t width, uint32_t height, uint32_t fmt, uint32_t pitch,
                          int origin, uint32_t levels, uint32_t layers, int samples_log2, uint32_t usage, uint32_t vol)
{
    g2_ref none = { 0, 0 };
    uint32_t idx = handle & (G2_HANDLES_MAX - 1);
    vimg *v;
    b->vstamp++;
    if ((handle & 0xC000) != 0xC000 || !vimg_ready(b)) {
        vimg t;
        memset(&t, 0, sizeof t);
        t.base = base; t.width = width; t.height = height; t.fmt = fmt; t.pitch = pitch ? pitch : width * g2_storage_bpp(fmt); t.levels = levels; t.layers = layers;
        t.usage = usage; t.vol = vol; t.origin = (uint8_t)origin; t.samples = (uint8_t)samples_log2;
        return send_create(b, (uint16_t)(handle & (G2_HANDLES_MAX - 1)), &t);
    }
    v = &b->vi[idx];
    if (v->live && v->phys != VIMG_NONE) { b->owner[v->phys] = VIMG_NONE; v->phys = VIMG_NONE; }   /* created again over itself: B takes the new descriptor */
    v->base = base; v->width = width; v->height = height; v->fmt = fmt; v->pitch = pitch ? pitch : width * g2_storage_bpp(fmt);
    v->levels = levels; v->layers = layers; v->usage = usage; v->vol = vol; v->origin = (uint8_t)origin; v->samples = (uint8_t)samples_log2;
    v->live = 1; v->stamp = b->vstamp;
    if (vimg_place(b, idx) < 0) return none;
    return b->last_ref;
}

static g2_ref create_image(g2_board *b, uint16_t handle, uint32_t base, uint32_t width, uint32_t height, uint32_t fmt, uint32_t pitch,
                           int origin, uint32_t levels, uint32_t layers, int samples_log2, uint32_t usage, uint32_t vol)
{
    return vimg_create(b, handle, base, width, height, fmt, pitch, origin, levels, layers, samples_log2, usage, vol);
}

g2_ref g2_create_image(g2_board *b, uint16_t handle, uint32_t base, uint32_t width, uint32_t height, uint32_t fmt, uint32_t pitch,
                       int origin, uint32_t levels, uint32_t layers, int samples_log2, uint32_t usage)
{
    return create_image(b, handle, base, width, height, fmt, pitch, origin, levels, layers, samples_log2, usage, 0);
}

g2_ref g2_create_volume(g2_board *b, uint16_t handle, uint32_t base, uint32_t width, uint32_t height, uint32_t depth, uint32_t fmt, uint32_t levels, uint32_t usage)
{
    return create_image(b, handle, base, width, height, fmt, 0, 1, levels, depth, 0, usage, 1);
}

static void wi_header(uint8_t *d, uint16_t handle, uint32_t level, uint32_t layer, uint32_t tfmt, uint32_t x, uint32_t y, uint32_t w,
                      uint32_t h, uint32_t first)
{
    pk k = { d };
    p16(&k, handle); p8(&k, level); p8(&k, layer); p8(&k, tfmt); p8(&k, 0); p16(&k, 0);
    p16(&k, x); p16(&k, y); p16(&k, w); p16(&k, h); p32(&k, first); p32(&k, 0);
}

int g2_write_image(g2_board *b, uint16_t handle, const void *data, size_t len, uint32_t tfmt, uint32_t x, uint32_t y, uint32_t w,
                   uint32_t h, uint32_t level, uint32_t layer, uint32_t first_pixel, uint32_t storage_fmt)
{
    uint32_t bpp = g2_transfer_bpp(tfmt, storage_fmt), per = G2_MAX_DATA / bpp;
    size_t npix = len / bpp, p;
    const uint8_t *src = (const uint8_t *)data;
    for (p = 0; p < npix; p += per) {
        uint8_t hdr[24];
        size_t n = npix - p < per ? npix - p : per;
        b->vstamp++;
        wi_header(hdr, phys_of(b, handle), level, layer, tfmt, x, y, w, h, first_pixel + (uint32_t)p);
        if (!g2_send_seq2(b, G2_T_WRITE_IMAGE, hdr, 24, src + p * bpp, n * bpp).seq) return -1;
    }
    return 0;
}

int g2_write_image_rect(g2_board *b, uint16_t handle, const void *src, int32_t src_pitch, uint32_t tfmt, uint32_t x, uint32_t y,
                        uint32_t w, uint32_t h, uint32_t level, uint32_t layer, uint32_t storage_fmt)
{
    uint32_t bpp = g2_transfer_bpp(tfmt, storage_fmt), per = G2_MAX_DATA / bpp, p = 0, npix = w * h;
    uint8_t d[G2_MAX_DATA];
    while (p < npix) {
        uint8_t hdr[24];
        uint32_t n = npix - p < per ? npix - p : per, done = 0;
        while (done < n) {                                      /* gather whole pixels across the row ends */
            uint32_t q = p + done, row = q / w, col = q % w, run = w - col;
            if (run > n - done) run = n - done;
            memcpy(d + done * bpp, (const uint8_t *)src + (ptrdiff_t)row * src_pitch + col * bpp, run * bpp);
            done += run;
        }
        b->vstamp++;
        wi_header(hdr, phys_of(b, handle), level, layer, tfmt, x, y, w, h, p);
        if (!g2_send_seq2(b, G2_T_WRITE_IMAGE, hdr, 24, d, n * bpp).seq) return -1;
        p += n;
    }
    return 0;
}

typedef struct ri_ctx { uint16_t handle; uint32_t level, layer, tfmt, x, y, w, h, first, bpp; } ri_ctx;
static void make_read_image(void *ctx, size_t s, size_t nbytes, uint32_t tag, uint8_t *body, size_t *blen)
{
    ri_ctx *c = (ri_ctx *)ctx;
    pk k = { body };
    p16(&k, c->handle); p8(&k, c->level); p8(&k, c->layer); p8(&k, c->tfmt); p8(&k, 0); p16(&k, 0);
    p16(&k, c->x); p16(&k, c->y); p16(&k, c->w); p16(&k, c->h);
    p32(&k, c->first + (uint32_t)(s / c->bpp)); p32(&k, (uint32_t)(nbytes / c->bpp)); p32(&k, tag);
    *blen = 28;
}

int g2_read_image(g2_board *b, uint16_t handle, uint32_t tfmt, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t level, uint32_t layer,
                  uint32_t storage_fmt, uint32_t first, uint32_t count, void *out)
{
    ri_ctx c;
    uint32_t bpp = g2_transfer_bpp(tfmt, storage_fmt), n = count ? count : w * h - first;
    b->vstamp++;
    c.handle = phys_of(b, handle); c.level = level; c.layer = layer; c.tfmt = tfmt; c.x = x; c.y = y; c.w = w; c.h = h; c.first = first; c.bpp = bpp;
    return read_stream(b, G2_T_READ_IMAGE, make_read_image, &c, out, (size_t)n * bpp, 0, (size_t)first * bpp);
}

g2_ref g2_fill_image(g2_board *b, uint16_t handle, uint32_t tfmt, uint32_t value, uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                     uint32_t mask, uint32_t level, uint32_t layer)
{
    uint8_t d[20];
    pk k = { d };
    b->vstamp++;
    p16(&k, phys_of(b, handle)); p8(&k, level); p8(&k, layer); p8(&k, tfmt); p8(&k, mask & 0xF); p16(&k, 0);
    p16(&k, x); p16(&k, y); p16(&k, w); p16(&k, h); p32(&k, value);
    return g2_send_seq(b, G2_T_FILL_IMAGE, d, sizeof d);
}

g2_ref g2_copy_image(g2_board *b, uint16_t src, uint16_t dst, uint32_t sx, uint32_t sy, uint32_t sw, uint32_t sh, uint32_t dx, uint32_t dy,
                     uint32_t dw, uint32_t dh, int mirror_x, int mirror_y, int use_key, uint32_t key_lo, uint32_t key_hi, uint32_t slevel,
                     uint32_t slayer, uint32_t dlevel, uint32_t dlayer)
{
    uint8_t d[36];
    pk k = { d };
    uint32_t flags = (mirror_x ? G2_COPY_MIRROR_X : 0u) | (mirror_y ? G2_COPY_MIRROR_Y : 0u) | (use_key ? G2_COPY_KEY : 0u);
    b->vstamp++;
    src = phys_of(b, src); dst = phys_of(b, dst);
    p16(&k, src); p8(&k, slevel); p8(&k, slayer); p16(&k, dst); p8(&k, dlevel); p8(&k, dlayer);
    p16(&k, sx); p16(&k, sy); p16(&k, sw); p16(&k, sh); p16(&k, dx); p16(&k, dy); p16(&k, dw); p16(&k, dh);
    p32(&k, flags); p32(&k, use_key ? key_lo : 0); p32(&k, use_key ? key_hi : 0);
    return g2_send_seq(b, G2_T_COPY_IMAGE, d, sizeof d);
}

/* the 36-byte COPY_IMAGE body with the options' flags; the tile's phase in the key-low word */
static void copy_head(pk *k, g2_board *b, uint16_t src, uint16_t dst, uint32_t sx, uint32_t sy, uint32_t sw, uint32_t sh, uint32_t dx,
                      uint32_t dy, uint32_t dw, uint32_t dh, const g2_copy_opts *o, uint32_t extra)
{
    uint32_t f = (o->flags & 0x7Fu) | extra, lo = 0, hi = 0, c;
    if (f & G2_COPY_SWIZZLE) for (c = 0; c < 4; c++) f |= (uint32_t)(o->swizzle[c] & 3u) << (8 + 2 * c);
    if (f & G2_COPY_KEY) { lo = o->key_lo; hi = o->key_hi; }
    if (f & G2_COPY_TILE) { lo = (o->tile_px & 0x1FFFu) | (o->tile_py & 0x1FFFu) << 16; hi = 0; }
    src = phys_of(b, src); dst = phys_of(b, dst);
    p16(k, src); p8(k, o->slevel); p8(k, o->slayer); p16(k, dst); p8(k, o->dlevel); p8(k, o->dlayer);
    p16(k, sx); p16(k, sy); p16(k, sw); p16(k, sh); p16(k, dx); p16(k, dy); p16(k, dw); p16(k, dh);
    p32(k, f); p32(k, lo); p32(k, hi);
}

g2_ref g2_copy_image_ex(g2_board *b, uint16_t src, uint16_t dst, uint32_t sx, uint32_t sy, uint32_t sw, uint32_t sh, uint32_t dx, uint32_t dy,
                        uint32_t dw, uint32_t dh, const g2_copy_opts *o)
{
    uint8_t d[36];
    pk k = { d };
    b->vstamp++;
    copy_head(&k, b, src, dst, sx, sy, sw, sh, dx, dy, dw, dh, o, 0);
    return g2_send_seq(b, G2_T_COPY_IMAGE, d, sizeof d);
}

#define G2_COPY_LIST_MAX 160                       /* pairs per COPY_IMAGE: 36 + 8 * 159 bytes stay inside one datagram */
g2_ref g2_copy_rects(g2_board *b, uint16_t src, uint16_t dst, int32_t ox, int32_t oy, const uint32_t *rects, uint32_t n, const g2_copy_opts *o)
{
    uint8_t d[36 + 8 * G2_COPY_LIST_MAX];
    g2_ref ref = { 0, 0 };
    uint32_t i = 0;
    b->vstamp++;
    while (i < n) {
        uint32_t m = (n - i > G2_COPY_LIST_MAX) ? G2_COPY_LIST_MAX : n - i, j;
        const uint32_t *r = rects + 4 * i;
        pk k = { d };
        copy_head(&k, b, src, dst, (uint32_t)((int32_t)r[0] + ox), (uint32_t)((int32_t)r[1] + oy), r[2], r[3], r[0], r[1], r[2], r[3], o,
                  m > 1 ? (uint32_t)G2_COPY_LIST : 0u);
        for (j = 1; j < m; j++) { p16(&k, r[4 * j]); p16(&k, r[4 * j + 1]); p16(&k, r[4 * j + 2]); p16(&k, r[4 * j + 3]); }
        ref = g2_send_seq(b, G2_T_COPY_IMAGE, d, (size_t)(k.p - d));
        i += m;
    }
    return ref;
}

/* ---------------------------------------------------------------- palette and hardware cursor */
g2_ref g2_set_palette(g2_board *b, uint32_t first, const uint32_t *colors, uint32_t count)
{
    uint8_t d[4 + 256 * 4];
    pk k = { d };
    uint32_t i;
    g2_ref none = { 0, 0 };
    if (count > 256) return none;
    p16(&k, first); p16(&k, count);
    for (i = 0; i < count; i++) p32(&k, colors[i] & 0xFFFFFF);
    return g2_send_seq(b, G2_T_SET_PALETTE, d, 4 + (size_t)count * 4);
}

int g2_cursor_shape(g2_board *b, const uint32_t *words, uint32_t count)
{
    uint32_t i;
    for (i = 0; i < count; i += 340) {
        uint8_t d[8];
        pk k = { d };
        uint32_t n = count - i < 340 ? count - i : 340;
        p8(&k, G2_CUR_SHAPE); p8(&k, 0); p16(&k, 0); p16(&k, i); p16(&k, 0);
        if (!g2_send_seq2(b, G2_T_CURSOR, d, 8, words + i, (size_t)n * 4).seq) return -1;
    }
    return 0;
}

g2_ref g2_cursor_show(g2_board *b, int mode, int hot_x, int hot_y, int has_pos, int x, int y, int take)
{
    uint8_t d[12];
    pk k = { d };
    p8(&k, G2_CUR_SHOW); p8(&k, (uint32_t)mode); p8(&k, (take ? 1u : 0u) | (has_pos ? 2u : 0u)); p8(&k, 0);
    p8(&k, (uint32_t)hot_x); p8(&k, (uint32_t)hot_y); p16(&k, 0);
    p16(&k, (uint32_t)(has_pos ? x : 0)); p16(&k, (uint32_t)(has_pos ? y : 0));
    return g2_send_seq(b, G2_T_CURSOR, d, 12);
}

g2_ref g2_cursor_hide(g2_board *b)
{
    uint8_t d[4];
    pk k = { d };
    p8(&k, G2_CUR_HIDE); p8(&k, 0); p16(&k, 0);
    return g2_send_seq(b, G2_T_CURSOR, d, 4);
}

g2_ref g2_cursor_move(g2_board *b, int x, int y)
{
    uint8_t d[8];
    pk k = { d };
    p8(&k, G2_CUR_MOVE); p8(&k, 0); p16(&k, 0); p16(&k, (uint32_t)x); p16(&k, (uint32_t)y);
    return g2_send_seq(b, G2_T_CURSOR, d, 8);
}

int g2_cursor_pos(g2_board *b, int x, int y)
{
    uint8_t d[4];
    pk k = { d };
    p16(&k, (uint32_t)x); p16(&k, (uint32_t)y);
    return g2_send_unseq(b, G2_T_CURSOR_POS, d, 4);
}
