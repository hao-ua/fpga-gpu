/* g2_env — what every API library shares: configuration (environment, then gpu2.ini beside the game's .exe), a log file (no debugger on
 * this toolchain: logs + a crash report with module-relative addresses for addr2line) and the connection to the boards. */
#ifndef G2_ENV_H
#define G2_ENV_H
#include "gpu2core.h"
#ifdef __cplusplus
extern "C" {
#endif

/* GPU2_IP (board A's address or host name; 192.168.1.10), GPU2_PORT (5000), GPU2_LOG (a path; empty = no log), GPU2_HOLD (1: PRESENT waits for vsync), GPU2_SCALE
   (fit | stretch), GPU2_MIN_A (62: the first with LDPX), GPU2_FRAMES_AHEAD (1, or 2: frames Present leaves unfinished, more is more lag),
   GPU2_RECORD (a folder: d3d9 keeps the last 48 frames of draws, F12 writes them), GPU2_RECORD_AFTER (the frame to start from),
   GPU2_KEEP_MB (256: room on board A for buffers that stop changing, 0 = stream everything) */
const char *g2e_get(const char *name, const char *dflt);    /* returned pointer stays valid */
int g2e_get_int(const char *name, int dflt);
void g2e_log(const char *fmt, ...);
g2_board *g2e_connect(const char *who);                     /* open + HELLO + board check; NULL (logged) when unusable */
void g2e_crash_handler(void);

#ifdef __cplusplus
}
#endif
#endif
