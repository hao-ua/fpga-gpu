/* fake_board: see fake_board.h. One thread; the model memory is linear, a buffer handle maps to its CREATE_BUFFER base. */
#define _WIN32_WINNT 0x0601
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fake_board.h"
#include "gpu2core.h"

#define MEM_MASK (FAKE_MEM_BYTES - 1)

struct fake_board {
    SOCKET s;
    HANDLE thread;
    volatile LONG running;
    uint8_t *mem;
    uint32_t buf_base[1024];
    double drop_h2b, drop_b2h;
    uint32_t rng;
    uint32_t expected, since_ack, frame;
    volatile LONG full_left;
    volatile LONG drop_ev_every, drop_rd_first, drop_rd_n, drop_rd_keep;
    uint32_t ev_count;
    volatile LONG rd_count;
    volatile LONG max_mode, mode_silent, mode_n[8];
    struct sockaddr_in host;
    int have_host;
    SRWLOCK lock;
    fake_stats st;
};

static uint32_t rnd(fake_board *f)
{
    uint32_t x = f->rng;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return f->rng = x;
}

static int chance(fake_board *f, double p) { return p > 0.0 && (double)(rnd(f) >> 8) / 16777216.0 < p; }
static uint32_t g32(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }
static uint32_t g16(const uint8_t *p) { return p[0] | (uint32_t)p[1] << 8; }
static void s32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }

static void tx(fake_board *f, uint8_t typ, uint32_t seq, const void *b1, size_t n1, const void *b2, size_t n2)
{
    uint8_t d[4096];
    if (!f->have_host) return;
    if (chance(f, f->drop_b2h)) { f->st.dropped_b2h++; return; }
    d[0] = 0x46; d[1] = 0x42; d[2] = 1; d[3] = typ;
    s32(d + 4, seq);
    memcpy(d + 8, b1, n1);
    if (n2) memcpy(d + 8 + n1, b2, n2);
    sendto(f->s, (const char *)d, (int)(8 + n1 + n2), 0, (const struct sockaddr *)&f->host, sizeof f->host);
}

static void ack(fake_board *f, uint32_t flags)
{
    uint8_t b[16];
    s32(b, f->expected); s32(b + 4, flags); s32(b + 8, 64u << 20); s32(b + 12, f->frame);
    f->st.acks++;
    f->since_ack = 0;
    tx(f, G2_R_ACK, 0, b, 16, NULL, 0);
}

static void event(fake_board *f, uint32_t seq, uint32_t idx, uint32_t code, uint32_t a0, uint32_t a1, uint32_t a2)
{
    uint8_t b[16];
    s32(b, code | idx << 8); s32(b + 4, a0); s32(b + 8, a1); s32(b + 12, a2);
    f->st.events++;
    if (f->drop_ev_every && ++f->ev_count % (uint32_t)f->drop_ev_every == 0) { f->st.dropped_b2h++; return; }
    tx(f, G2_R_EVENT, seq, b, 16, NULL, 0);
}

static void read_data(fake_board *f, uint32_t tag, uint32_t addr, uint32_t n)
{
    uint32_t off;
    LONG nr = InterlockedIncrement(&f->rd_count);
    if (nr >= f->drop_rd_first && nr - f->drop_rd_first < f->drop_rd_n && !(f->drop_rd_keep && nr % f->drop_rd_keep == 0)) { f->st.dropped_b2h += n / G2_RD_CHUNK + 1; return; }
    for (off = 0; off < n; off += G2_RD_CHUNK) {
        uint8_t h[8];
        uint32_t c = n - off < G2_RD_CHUNK ? n - off : G2_RD_CHUNK;
        s32(h, tag); s32(h + 4, off);
        f->st.chunks++;
        tx(f, G2_R_READ_DATA, 0, h, 8, f->mem + ((addr + off) & MEM_MASK), c);
    }
}

static void command(fake_board *f, uint8_t typ, uint32_t seq, uint32_t idx, const uint8_t *b, size_t n)
{
    switch (typ) {
    case G2_T_CREATE_BUFFER:
        f->buf_base[g16(b) & 1023] = g32(b + 4);
        break;
    case G2_T_WRITE_BUFFER:
        memcpy(f->mem + ((f->buf_base[g16(b) & 1023] + g32(b + 4)) & MEM_MASK), b + 8, n - 8);
        break;
    case G2_T_MEM_WRITE:
        memcpy(f->mem + (g32(b + 4) & MEM_MASK), b + 8, n - 8);
        break;
    case G2_T_READ_BUFFER:
        read_data(f, g32(b + 12), f->buf_base[g16(b) & 1023] + g32(b + 4), g32(b + 8));
        break;
    case G2_T_MEM_READ:
        read_data(f, g32(b + 12), g32(b + 4), g32(b + 8));
        break;
    case G2_T_FENCE:
        event(f, seq, idx, G2_EV_FENCE, g32(b), 0, 0);
        break;
    case G2_T_PRESENT:
        f->frame = g32(b + 4);
        event(f, seq, idx, G2_EV_PRESENT, g32(b + 4), f->frame, 0);
        break;
    case G2_T_SET_MODE:
        InterlockedIncrement(&f->mode_n[g32(b) & 7]);
        if (g32(b) <= (uint32_t)f->max_mode) event(f, seq, idx, G2_EV_MODE, g32(b), 0, 0);
        else if (!f->mode_silent) event(f, seq, idx, G2_EV_ERROR, 3, G2_T_SET_MODE, g32(b));   /* fb_cmd_b: ERR_RANGE, the type, the id */
        break;
    default:
        break;
    }
}

static void datagram(fake_board *f, const uint8_t *d, size_t n)
{
    uint32_t seq = g32(d + 4);
    uint8_t typ = d[3];
    const uint8_t *b = d + 8;
    size_t blen = n - 8;
    f->st.rx++;
    if (seq == 0) {
        if (typ == G2_T_HELLO) {
            if (g32(b)) f->expected = 1;
            ack(f, 0);
        } else if (typ == G2_T_STATUS_REQ) {
            static uint32_t st[G2_ST_WORDS];
            uint8_t w[G2_ST_WORDS * 4];
            int i;
            st[G2_ST_LAYOUT] = 7; st[G2_ST_BUILD] = 57; st[G2_ST_LINK_FLAGS] = 5;
            st[G2_ST_GAP] = (uint32_t)f->st.gaps; st[G2_ST_DUP] = (uint32_t)f->st.dups;
            for (i = 0; i < G2_ST_WORDS; i++) s32(w + 4 * i, st[i]);
            tx(f, G2_R_STATUS, 0, w, sizeof w, NULL, 0);
        }
        return;
    }
    if (seq == f->expected) {
        if (f->full_left > 0) {                                 /* the ingress queue is full: not accepted */
            InterlockedDecrement(&f->full_left);
            f->st.full_sent++;
            ack(f, G2_ACKF_FULL);
            return;
        }
        f->expected = f->expected + 1 ? f->expected + 1 : 1;
        f->st.accepted++;
        if (typ == G2_T_BATCH) {
            size_t o = 0;
            uint32_t idx = 0;
            while (o + 4 <= blen) {
                uint32_t len = g16(b + o + 2);
                command(f, b[o], seq, idx, b + o + 4, len);
                o += 4 + ((len + 3) & ~3u);
                idx++;
            }
        } else {
            command(f, typ, seq, 0, b, blen);
        }
        if (++f->since_ack >= 16) ack(f, 0);
    } else if ((uint32_t)(seq - f->expected) >= 0x80000000u) {
        f->st.dups++;
        ack(f, 0);
    } else {
        f->st.gaps++;
        ack(f, G2_ACKF_NAK);
    }
}

static DWORD WINAPI loop(LPVOID arg)
{
    fake_board *f = (fake_board *)arg;
    uint8_t d[2048];
    LARGE_INTEGER freq, t0, t;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&t0);
    while (f->running) {
        struct sockaddr_in from;
        int fl = sizeof from;
        int n = recvfrom(f->s, (char *)d, sizeof d, 0, (struct sockaddr *)&from, &fl);
        if (n >= 8) {
            f->host = from;
            f->have_host = 1;
            if (chance(f, f->drop_h2b)) f->st.dropped_h2b++;
            else datagram(f, d, (size_t)n);
        }
        QueryPerformanceCounter(&t);
        if ((double)(t.QuadPart - t0.QuadPart) / (double)freq.QuadPart >= 0.001) {  /* the heartbeat */
            t0 = t;
            ack(f, 0);
        }
    }
    return 0;
}

void fake_drop_events(fake_board *f, int every) { InterlockedExchange(&f->drop_ev_every, every); }
void fake_drop_reads(fake_board *f, int first, int n, int keep)
{
    InterlockedExchange(&f->rd_count, 0); InterlockedExchange(&f->drop_rd_first, first); InterlockedExchange(&f->drop_rd_keep, keep); InterlockedExchange(&f->drop_rd_n, n);
}
void fake_modes(fake_board *f, int max_mode, int silent) { InterlockedExchange(&f->max_mode, max_mode); InterlockedExchange(&f->mode_silent, silent); }
int fake_mode_count(fake_board *f, int mode) { return (int)f->mode_n[mode & 7]; }

fake_board *fake_start(int port, double drop_h2b, double drop_b2h, uint32_t seed)
{
    WSADATA w;
    fake_board *f = (fake_board *)calloc(1, sizeof *f);
    struct sockaddr_in me;
    DWORD tmo = 1;
    int rcv = 16 << 20;
    WSAStartup(MAKEWORD(2, 2), &w);
    f->mem = (uint8_t *)calloc(1, FAKE_MEM_BYTES);
    f->drop_h2b = drop_h2b; f->drop_b2h = drop_b2h;
    f->rng = seed ? seed : 1;
    f->expected = 1;
    f->max_mode = 255;
    f->s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    setsockopt(f->s, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tmo, sizeof tmo);
    setsockopt(f->s, SOL_SOCKET, SO_RCVBUF, (const char *)&rcv, sizeof rcv);
    memset(&me, 0, sizeof me);
    me.sin_family = AF_INET;
    me.sin_port = htons((u_short)port);
    me.sin_addr.s_addr = inet_addr("127.0.0.1");
    if (bind(f->s, (struct sockaddr *)&me, sizeof me) == SOCKET_ERROR) return NULL;
    f->running = 1;
    f->thread = CreateThread(NULL, 0, loop, f, 0, NULL);
    return f;
}

void fake_stop(fake_board *f)
{
    InterlockedExchange(&f->running, 0);
    WaitForSingleObject(f->thread, 2000);
    closesocket(f->s);
    free(f->mem);
    free(f);
}

void fake_get_stats(fake_board *f, fake_stats *s) { *s = f->st; }
const uint8_t *fake_memory(fake_board *f) { return f->mem; }
void fake_full_burst(fake_board *f, int n) { InterlockedExchange(&f->full_left, n); }
