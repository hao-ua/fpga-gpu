/* fake_board — board A's protocol in software on a loopback socket: Go-Back-N acceptance,
 * ACK every 16 datagrams + the 1 ms heartbeat ACK, NAK on a gap, FULL on request, EVENTs with the sub-message index, READ_DATA
 * chunks, datagram loss injected in both directions. */
#ifndef FAKE_BOARD_H
#define FAKE_BOARD_H
#include <stdint.h>

typedef struct fake_board fake_board;
typedef struct fake_stats {
    uint64_t rx, accepted, gaps, dups, dropped_h2b, dropped_b2h, acks, events, chunks, full_sent;
} fake_stats;

fake_board *fake_start(int port, double drop_h2b, double drop_b2h, uint32_t seed);
void fake_stop(fake_board *f);
void fake_get_stats(fake_board *f, fake_stats *s);
const uint8_t *fake_memory(fake_board *f);              /* 256 MB of model memory: byte address & mask */
void fake_full_burst(fake_board *f, int n);             /* answer the next n datagrams with FULL instead of accepting them */
void fake_drop_events(fake_board *f, int every);        /* lose every n-th EVENT, whatever else arrives; 0 = none */
void fake_drop_reads(fake_board *f, int first, int n, int keep);    /* lose all the answers to read requests first.. of the next n, except every keep-th (0: none kept) */
void fake_modes(fake_board *f, int max_mode, int silent);  /* SET_MODE above max_mode: an ERR_RANGE EVENT as board B sends (silent: no answer at all); default all accepted */
int fake_mode_count(fake_board *f, int mode);           /* SET_MODEs of this id received */
#define FAKE_MEM_BYTES (1u << 28)
#endif
