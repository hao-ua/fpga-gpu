/* g2_mode_test — the HDMI mode table and g2_mode_select against fake_board, no hardware: the smallest mode that holds a size (1 024 x 768 is mode 4,
 * between 800 x 600 and 1 280 x 720), a board B that has mode 4, one without it, which refuses it (ERR_RANGE: 1 280 x 800 instead, the refusal
 * remembered, its error taken), and one that does not answer (1 280 x 800 this time, asked again next time). */
#include <stdio.h>
#include <windows.h>
#include "fake_board.h"
#include "gpu2core.h"

#define PORT 5602
#define FAIL(...) do { printf("FAIL: " __VA_ARGS__); printf("\n"); return 1; } while (0)

static int test_fit(void)
{
    static const struct { int w, h, xga, want; } T[] = {
        { 640, 480, 1, 0 }, { 320, 200, 1, 0 }, { 512, 384, 1, 0 }, { 641, 480, 1, 1 }, { 800, 600, 1, 1 }, { 801, 600, 1, 4 },
        { 1024, 768, 1, 4 }, { 1024, 768, 0, 3 }, { 1024, 720, 1, 4 }, { 1024, 720, 0, 2 }, { 1024, 600, 1, 4 }, { 1024, 769, 1, 3 },
        { 1025, 600, 1, 2 }, { 1280, 720, 1, 2 }, { 1280, 721, 1, 3 }, { 1280, 800, 1, 3 }, { 1281, 800, 1, 3 }, { 4000, 3000, 0, 3 } };
    int i;
    for (i = 0; i < (int)(sizeof T / sizeof T[0]); i++) {
        int m = g2_mode_fit(T[i].w, T[i].h, T[i].xga);
        if (m != T[i].want) FAIL("g2_mode_fit(%d, %d, %d) = %d, want %d", T[i].w, T[i].h, T[i].xga, m, T[i].want);
    }
    for (i = 0; i < 8; i++) {
        int want = i < 5 ? i : 2;
        if (g2_mode_of_flags((uint32_t)i << 4 | 0x0Bu) != want) FAIL("g2_mode_of_flags(mode %d) = %d, want %d", i, g2_mode_of_flags((uint32_t)i << 4), want);
    }
    if (G2_MODE_W[4] != 1024 || G2_MODE_H[4] != 768) FAIL("mode 4 is %d x %d", G2_MODE_W[4], G2_MODE_H[4]);
    printf("ok: g2_mode_fit by size 0, 1, 4, 2, 3 (%d cases), g2_mode_of_flags 0..7\n", (int)(sizeof T / sizeof T[0]));
    return 0;
}

static int sets(fake_board *f) { int n = 0, m; for (m = 0; m < 8; m++) n += fake_mode_count(f, m); return n; }

/* select w x h from *mode; expect `want`, `n_sets` SET_MODEs in all so far, and no error left in the list */
static int pick(g2_board *b, fake_board *f, int w, int h, int *mode, int want, int n_sets)
{
    g2_error e[8];
    if (g2_mode_select(b, w, h, mode) < 0) FAIL("select %d x %d: %s", w, h, g2_error_text(b));
    if (*mode != want) FAIL("select %d x %d: mode %d, want %d", w, h, *mode, want);
    if (sets(f) != n_sets) FAIL("select %d x %d: %d SET_MODEs so far, want %d", w, h, sets(f), n_sets);
    if (g2_take_errors(b, e, 8)) FAIL("select %d x %d: an error left in the list (code %u type %02X)", w, h, e[0].code, e[0].type);
    return 0;
}

static int test_board(int max_mode, int silent)
{
    fake_board *f = fake_start(PORT, 0.0, 0.0, 3);
    g2_board *b = g2_open("127.0.0.1", PORT, NULL);
    int mode = 0, r = 1;
    double t0 = GetTickCount() / 1000.0;
    if (!f || !b) FAIL("open");
    if (g2_hello(b, 1) < 0) FAIL("hello: %s", g2_error_text(b));
    fake_modes(f, max_mode, silent);
    if (max_mode >= 4) {                                            /* a B with 1 024 x 768 */
        if (pick(b, f, 1024, 768, &mode, 4, 1) || pick(b, f, 1024, 768, &mode, 4, 1) || pick(b, f, 1000, 700, &mode, 4, 1) ||
            pick(b, f, 1280, 720, &mode, 2, 2) || pick(b, f, 640, 480, &mode, 0, 3) || pick(b, f, 1024, 768, &mode, 4, 4)) goto out;
        printf("ok: B with mode 4: 1 024 x 768 natively, no SET_MODE when the mode holds the size already\n");
    } else if (!silent) {                                           /* a board B without mode 4: ERR_RANGE */
        if (pick(b, f, 1024, 768, &mode, 3, 2) || fake_mode_count(f, 4) != 1 || pick(b, f, 1024, 700, &mode, 2, 3) ||
            pick(b, f, 1024, 768, &mode, 3, 4) || fake_mode_count(f, 4) != 1 || pick(b, f, 800, 600, &mode, 1, 5)) goto out;
        printf("ok: B without mode 4: 1 280 x 800 (1:1), one probe for the board's life, its ERR_RANGE taken\n");
    } else {                                                        /* no answer: this time only */
        if (pick(b, f, 1024, 768, &mode, 3, 2) || pick(b, f, 640, 480, &mode, 0, 3) || pick(b, f, 1024, 768, &mode, 3, 5) ||
            fake_mode_count(f, 4) != 2) goto out;
        printf("ok: SET_MODE 4 unanswered: 1 280 x 800 that time, asked again the next (%.0f s)\n", GetTickCount() / 1000.0 - t0);
    }
    r = 0;
out:
    g2_close(b);
    fake_stop(f);
    return r;
}

int main(void)
{
    if (test_fit()) return 1;
    if (test_board(255, 0)) return 1;
    if (test_board(3, 0)) return 1;
    if (test_board(3, 1)) return 1;
    printf("OK g2_mode_test\n");
    return 0;
}
