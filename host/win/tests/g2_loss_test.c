/* g2_loss_test — gpu2core's reliability layer against fake_board with injected loss (the mark: 1 % loss, 10 000 frames, no stuck
 * window). No hardware. Every check compares data end to end, not just counters. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>
#include "fake_board.h"
#include "gpu2core.h"

#define PORT 5601
#define FAIL(...) do { printf("FAIL: " __VA_ARGS__); printf("\n"); return 1; } while (0)

static void fill(uint8_t *p, size_t n, uint32_t seed)
{
    size_t i;
    uint32_t x = seed;
    for (i = 0; i < n; i++) {
        x = x * 1664525u + 1013904223u;
        p[i] = (uint8_t)(x >> 24);
    }
}

static void report(const char *what, fake_board *f, g2_board *b)
{
    fake_stats s;
    g2_counters c;
    fake_get_stats(f, &s);
    g2_get_counters(b, &c);
    printf("ok: %s\n    fake: rx %llu accepted %llu gaps %llu dups %llu dropped h2b %llu b2h %llu acks %llu events %llu chunks %llu full %llu | "
           "host: resent %llu heartbeat %llu re-requested %llu (later %llu below %llu tail %llu dup %llu)\n", what,
           (unsigned long long)s.rx, (unsigned long long)s.accepted, (unsigned long long)s.gaps, (unsigned long long)s.dups, (unsigned long long)s.dropped_h2b,
           (unsigned long long)s.dropped_b2h, (unsigned long long)s.acks, (unsigned long long)s.events, (unsigned long long)s.chunks, (unsigned long long)s.full_sent,
           (unsigned long long)c.n_resent, (unsigned long long)c.n_hb_resend, (unsigned long long)c.n_rerequested, (unsigned long long)c.rd_later,
           (unsigned long long)c.rd_below, (unsigned long long)c.rd_tail, (unsigned long long)c.rd_dup);
}

/* write + read with `drop` in both directions: every byte must arrive, in both directions */
static int test_transfer(double drop, size_t bytes)
{
    fake_board *f = fake_start(PORT, drop, drop, 12345);
    g2_board *b = g2_open("127.0.0.1", PORT, NULL);
    uint8_t *src = (uint8_t *)malloc(bytes), *dst = (uint8_t *)malloc(bytes);
    char what[96];
    if (!f || !b) FAIL("open");
    if (g2_hello(b, 1) < 0) FAIL("hello: %s", g2_error_text(b));
    fill(src, bytes, 7);
    g2_create_buffer(b, 0x0021, 0x01000000u, (uint32_t)bytes, 0);
    if (g2_write_buffer(b, 0x0021, 0, src, bytes) < 0) FAIL("write: %s", g2_error_text(b));
    if (g2_fence(b, 30.0) < 0) FAIL("fence: %s", g2_error_text(b));
    if (memcmp(fake_memory(f) + 0x01000000u, src, bytes)) FAIL("write: the board's memory differs");
    memset(dst, 0, bytes);
    if (g2_read_buffer(b, 0x0021, 0, dst, bytes) < 0) FAIL("read: %s", g2_error_text(b));
    if (memcmp(dst, src, bytes)) FAIL("read: the data differs");
    snprintf(what, sizeof what, "%.1f %% loss both ways: %u MB written and read back intact", drop * 100, (unsigned)(bytes >> 20));
    report(what, f, b);
    g2_close(b);
    fake_stop(f);
    free(src); free(dst);
    return 0;
}

/* frames as the front ends send them: BATCH + PRESENT with two in flight; every event must arrive and no window may stick */
static int test_frames(double drop, int frames)
{
    fake_board *f = fake_start(PORT, drop, drop, 777);
    g2_board *b = g2_open("127.0.0.1", PORT, NULL);
    g2_ref pending[2];
    int np = 0, i, lost = 0;
    uint32_t u[4] = { 0, 0, 0, 0 }, cc[4] = { 0, 0, 0, 0 }, sc[4] = { 0, 0, 640, 480 };
    char what[96];
    if (!f || !b) FAIL("open");
    if (g2_hello(b, 1) < 0) FAIL("hello: %s", g2_error_text(b));
    for (i = 0; i < frames; i++) {
        int k;
        g2_ref r;
        while (np >= 2) {
            uint32_t fid = 0;
            /* a lost EVENT is a real outcome at this drop rate: the board never repeats an event, so the wait must end, not hang */
            if (g2_present_wait(b, pending[0], &fid, NULL, NULL, 0.05) < 0) lost++;
            else if (fid != (uint32_t)(i - 2)) FAIL("frame %d: EV_PRESENT says %u", i, fid);
            pending[0] = pending[1]; np--;
        }
        g2_batch_begin(b);
        g2_frame_begin(b);
        for (k = 0; k < 20; k++) {
            g2_vs_uniforms(b, 0, u, 1);
            g2_draw(b, 0, 36, 0, 1, 0x22, 0, 0);
        }
        g2_flush(b, 0xC030, G2_FL_CLEAR_COLOR, cc, 0xFFFFFF, 0, sc, 0, 0xFF);
        g2_batch_end(b);
        r = g2_present(b, 0xC030, (uint32_t)i, 0, 0);
        if (!r.seq) FAIL("frame %d: send: %s", i, g2_error_text(b));
        pending[np++] = r;
    }
    if (g2_wait_acked(b, 30.0) < 0) FAIL("final ack: %s", g2_error_text(b));
    {
        fake_stats s;
        g2_counters c;
        fake_get_stats(f, &s);
        g2_get_counters(b, &c);
        if (s.accepted != c.dgrams) FAIL("the board accepted %llu of %llu datagrams", (unsigned long long)s.accepted, (unsigned long long)c.dgrams);
    }
    snprintf(what, sizeof what, "%.1f %% loss: %d frames, every datagram accepted in order, %d EVENTs lost on the wire", drop * 100, frames, lost);
    report(what, f, b);
    if (drop == 0.0 && lost) FAIL("events lost without injected loss");
    g2_close(b);
    fake_stop(f);
    return 0;
}

static int test_full(void)
{
    fake_board *f = fake_start(PORT, 0.0, 0.0, 5);
    g2_board *b = g2_open("127.0.0.1", PORT, NULL);
    size_t bytes = 4u << 20;
    uint8_t *src = (uint8_t *)malloc(bytes);
    if (!f || !b) FAIL("open");
    if (g2_hello(b, 1) < 0) FAIL("hello");
    fill(src, bytes, 3);
    g2_create_buffer(b, 0x0021, 0x02000000u, (uint32_t)bytes, 0);
    fake_full_burst(f, 40);                                    /* the board refuses 40 datagrams with FULL */
    if (g2_write_buffer(b, 0x0021, 0, src, bytes) < 0) FAIL("write: %s", g2_error_text(b));
    if (g2_fence(b, 30.0) < 0) FAIL("fence: %s", g2_error_text(b));
    if (memcmp(fake_memory(f) + 0x02000000u, src, bytes)) FAIL("data differs after FULL");
    report("FULL back-off: 40 refused datagrams resent, 4 MB intact", f, b);
    g2_close(b);
    fake_stop(f);
    free(src);
    return 0;
}

/* the board never repeats an EVENT: a lost one must cost a moment, not a timeout. Every third lost, then every second, so the FENCE that asks is lost too, then all of them */
static int test_lost_events(void)
{
    fake_board *f = fake_start(PORT, 0.0, 0.0, 9);
    g2_board *b = g2_open("127.0.0.1", PORT, NULL);
    LARGE_INTEGER hz, t0, t1;
    double worst = 0.0;
    int every, i, recovered = 0, r;
    g2_counters c;
    g2_event ev;
    char what[120];
    if (!f || !b) FAIL("open");
    if (g2_hello(b, 1) < 0) FAIL("hello: %s", g2_error_text(b));
    QueryPerformanceFrequency(&hz);
    for (every = 3; every >= 2; every--) {
        fake_drop_events(f, every);
        for (i = 0; i < 300; i++) {
            double d;
            g2_ref q = g2_present(b, 0xC030, (uint32_t)i, 0, 0);
            if (!q.seq) FAIL("send: %s", g2_error_text(b));
            QueryPerformanceCounter(&t0);
            r = g2_wait_done(b, q, &ev, 0.01, 2.0);
            QueryPerformanceCounter(&t1);
            d = (double)(t1.QuadPart - t0.QuadPart) / (double)hz.QuadPart;
            if (d > worst) worst = d;
            if (r < 0) FAIL("present %d, every %d lost: %s", i, every, g2_error_text(b));
            if (r == 0 && ev.a0 != (uint32_t)i) FAIL("present %d: its EVENT says %u", i, ev.a0);
            recovered += r;
        }
        for (i = 0; i < 40; i++) {
            QueryPerformanceCounter(&t0);
            if (g2_fence(b, 2.0) < 0) FAIL("fence %d, every %d lost: %s", i, every, g2_error_text(b));
            QueryPerformanceCounter(&t1);
        }
    }
    if (worst > 0.25) FAIL("a lost EVENT took %.0f ms", worst * 1000.0);
    if (recovered < 300) FAIL("only %d of 600 waits recovered: the test lost too little", recovered);
    g2_get_counters(b, &c);
    if (c.n_lost_events < (uint64_t)recovered) FAIL("%d recovered, %llu counted", recovered, (unsigned long long)c.n_lost_events);
    /* nothing answers at all: the wait must end at its timeout, and say so */
    fake_drop_events(f, 1);
    {
        g2_ref q = g2_present(b, 0xC030, 1000, 0, 0);
        QueryPerformanceCounter(&t0);
        r = g2_wait_done(b, q, &ev, 0.01, 0.3);
        QueryPerformanceCounter(&t1);
        if (r >= 0) FAIL("nothing answered, and the wait says %d", r);
        if ((double)(t1.QuadPart - t0.QuadPart) / (double)hz.QuadPart < 0.29) FAIL("the wait gave up early");
    }
    fake_drop_events(f, 0);
    snprintf(what, sizeof what, "lost EVENTs: %d waits recovered, the longest %.0f ms; nothing answering ends at the timeout", recovered, worst * 1000.0);
    report(what, f, b);
    g2_close(b);
    fake_stop(f);
    return 0;
}

/* all the answers to all the requests in flight lost: nothing later shows that up, so a read must notice by itself; it may only give up on silence */
static int test_lost_reads(void)
{
    fake_board *f = fake_start(PORT, 0.0, 0.0, 11);
    g2_board *b = g2_open("127.0.0.1", PORT, NULL);
    size_t bytes = 3u << 20;
    uint8_t *src = (uint8_t *)malloc(bytes), *dst = (uint8_t *)malloc(bytes);
    LARGE_INTEGER hz, t0, t1;
    double d;
    int r;
    if (!f || !b) FAIL("open");
    if (g2_hello(b, 1) < 0) FAIL("hello: %s", g2_error_text(b));
    QueryPerformanceFrequency(&hz);
    fill(src, bytes, 5);
    g2_create_buffer(b, 0x0021, 0x01000000u, (uint32_t)bytes, 0);
    if (g2_write_buffer(b, 0x0021, 0, src, bytes) < 0) FAIL("write: %s", g2_error_text(b));
    if (g2_fence(b, 30.0) < 0) FAIL("fence: %s", g2_error_text(b));

#define TIMED_READ(n) (memset(dst, 0, bytes), QueryPerformanceCounter(&t0), r = g2_read_buffer(b, 0x0021, 0, dst, (n)), QueryPerformanceCounter(&t1), \
                       d = (double)(t1.QuadPart - t0.QuadPart) / (double)hz.QuadPart)
    fake_drop_reads(f, 3, 2, 0);                                /* the two a read keeps in flight */
    TIMED_READ(bytes);
    if (r < 0) FAIL("read, two requests lost: %s", g2_error_text(b));
    if (memcmp(dst, src, bytes)) FAIL("read, two requests lost: the data differs");
    if (d > 2.5) FAIL("two lost requests took %.2f s", d);
    fake_drop_reads(f, 1, 1, 0);                                /* a read of one request */
    TIMED_READ(1000);
    if (r < 0) FAIL("read, its only request lost: %s", g2_error_text(b));
    if (memcmp(dst, src, 1000)) FAIL("read, its only request lost: the data differs");
    if (d > 1.5) FAIL("the only request lost took %.2f s", d);
    fake_drop_reads(f, 1, 1000000, 4);                          /* slow but alive, past the 10 s of the timeout */
    TIMED_READ(bytes);
    if (r < 0) FAIL("slow read: %s", g2_error_text(b));
    if (memcmp(dst, src, bytes)) FAIL("slow read: the data differs");
    if (d < 10.5) FAIL("the slow read took only %.2f s: it proves nothing about the timeout", d);
    printf("ok: a read that kept going for %.1f s\n", d);
    fake_drop_reads(f, 1, 1000000, 0);                          /* nothing answers: it must end at its timeout, and say so */
    TIMED_READ(bytes);
    if (r >= 0) FAIL("nothing answered, and the read says %d", r);
    if (d < 9.5 || d > 12.0) FAIL("nothing answered: the read gave up after %.2f s", d);
#undef TIMED_READ
    fake_drop_reads(f, 0, 0, 0);
    report("lost reads: asked again after 1 s; silence ends a read at 10 s, slowness does not", f, b);
    g2_close(b);
    fake_stop(f);
    free(src); free(dst);
    return 0;
}

int main(void)
{
    if (test_lost_events()) return 1;
    if (test_lost_reads()) return 1;
    if (test_transfer(0.0, 8u << 20)) return 1;
    if (test_transfer(0.01, 8u << 20)) return 1;
    if (test_transfer(0.05, 2u << 20)) return 1;
    if (test_full()) return 1;
    if (test_frames(0.0, 2000)) return 1;
    if (test_frames(0.01, 10000)) return 1;
    printf("OK g2_loss_test\n");
    return 0;
}
