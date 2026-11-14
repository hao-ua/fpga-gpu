/* gpu2core — the host side of the two-board GPU for the Windows API libraries.
 *
 * A C port of host/fb/fbproto.py with the same structure, so behaviour can be compared line by line and the datagrams
 * byte by byte (host/win/tests/parity.ps1). Every function that sends takes the board's send path: one application
 * thread at a time (the front ends hold their own API lock); the receive and maintenance threads are internal.
 */
#ifndef GPU2CORE_H
#define GPU2CORE_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* host -> board */
enum {
    G2_T_HELLO = 0x01, G2_T_STATUS_REQ = 0x02, G2_T_PHY_REG = 0x03, G2_T_MEM_WRITE = 0x04, G2_T_MEM_READ = 0x05,
    G2_T_MEMTEST = 0x06, G2_T_REG = 0x07,
    G2_T_CREATE_BUFFER = 0x10, G2_T_CREATE_IMAGE = 0x11, G2_T_DESTROY = 0x12,
    G2_T_WRITE_BUFFER = 0x20, G2_T_READ_BUFFER = 0x21, G2_T_WRITE_IMAGE = 0x22, G2_T_READ_IMAGE = 0x23,
    G2_T_FILL_IMAGE = 0x24, G2_T_COPY_IMAGE = 0x25, G2_T_GEN_MIPMAP = 0x26, G2_T_COPY_BUFFER = 0x27,
    G2_T_SET_MODE = 0x30, G2_T_PRESENT = 0x31, G2_T_CURSOR = 0x32, G2_T_CURSOR_POS = 0x33, G2_T_SET_PALETTE = 0x34,
    G2_T_FENCE = 0x40, G2_T_FLUSH = 0x41,
    G2_T_DRAW_STATE = 0x50,
    G2_T_VS_PROGRAM = 0x51, G2_T_VS_UNIFORMS = 0x52, G2_T_VERTEX_ATTR = 0x53, G2_T_VS_STATE = 0x54, G2_T_DRAW = 0x55,
    G2_T_FRAME_BEGIN = 0x56,
    G2_T_BATCH = 0x60
};
/* board -> host */
enum { G2_R_ACK = 0x81, G2_R_STATUS = 0x82, G2_R_EVENT = 0x83, G2_R_READ_DATA = 0x84, G2_R_PHY_REG = 0x85, G2_R_REG = 0x86 };
enum { G2_EV_FENCE = 0x01, G2_EV_PRESENT = 0x02, G2_EV_MODE = 0x03, G2_EV_MEMTEST = 0x04, G2_EV_ERROR = 0x10 };
enum { G2_ACKF_NAK = 1, G2_ACKF_FULL = 2 };

#define G2_BATCH_MAX 1464u            /* body bytes of a BATCH datagram: 8-byte header + body <= 1472 */
#define G2_MAX_DATA  1440u            /* data bytes per WRITE_* datagram */
#define G2_RD_CHUNK  1440u            /* READ_DATA chunk size used by the board */
#define G2_HEAP_A     0x18000000u     /* board A host heap: [384 MB, 1 GB) */
#define G2_HEAP_A_END 0x40000000u
#define G2_HEAP_B     0x10000000u     /* board B host heap: [256 MB, 1 GB) */
#define G2_HEAP_B_END 0x40000000u
#define G2_GUARD_BAND 32640.0         /* board A clips x / y at +-2040 px, in 1/16 px */

enum { G2_AT_BYTE, G2_AT_UBYTE, G2_AT_SHORT, G2_AT_USHORT, G2_AT_FIXED, G2_AT_FLOAT, G2_AT_FP24 };   /* VERTEX_ATTR formats */
enum { G2_CUR_SHAPE = 1, G2_CUR_SHOW = 2, G2_CUR_HIDE = 3, G2_CUR_MOVE = 4 };
enum { G2_CUR_COLOR = 0, G2_CUR_ANDXOR = 1, G2_CUR_PREMUL = 2 };
enum { G2_COPY_MIRROR_X = 1, G2_COPY_MIRROR_Y = 2, G2_COPY_KEY = 4 };
enum { G2_MASK_R = 1, G2_MASK_G = 2, G2_MASK_B = 4, G2_MASK_A = 8 };
enum { G2_PRES_AT_ONCE = 1, G2_PRES_HOLD = 2 };
/* FLUSH flags */
enum { G2_FL_CLEAR_COLOR = 1, G2_FL_CLEAR_SCISSOR = 2, G2_FL_DITHER = 64, G2_FL_CLEAR_DEPTH = 128, G2_FL_CLEAR_STENCIL = 256,
       G2_FL_DEPTH16 = 512 };
#define G2_FL_CMASK(m) (((m) & 0xFu) << 2)
enum { G2_FN_NEVER, G2_FN_LESS, G2_FN_EQUAL, G2_FN_LEQUAL, G2_FN_GREATER, G2_FN_NOTEQUAL, G2_FN_GEQUAL, G2_FN_ALWAYS };
/* image storage formats */
enum { G2_FMT_RGBA8 = 0x00, G2_FMT_RGB565 = 0x04, G2_FMT_RGBA4444 = 0x05, G2_FMT_RGBA5551 = 0x06, G2_FMT_P8 = 0x0C,
       G2_FMT_D24S8 = 0x10 };
/* transfer formats */
enum { G2_TF_RAW = 0x00, G2_TF_RGBA8888 = 0x01, G2_TF_BGRA8888 = 0x02, G2_TF_BGRX8888 = 0x03, G2_TF_RGB565 = 0x04,
       G2_TF_RGB888 = 0x0A, G2_TF_BGR888 = 0x0B };
enum { G2_USE_SAMPLED = 1, G2_USE_RT = 2, G2_USE_SCANOUT = 4, G2_USE_TRANSFER = 8 };
enum { G2_REG_IDELAY = 0x0000, G2_REG_PHYOPT = 0x0001, G2_REG_PHYAD = 0x0002, G2_REG_REINIT = 0x0003, G2_REG_SCRATCH = 0x0004,
       G2_REG_LINKCTL = 0x0005, G2_REG_BUILD = 0x0010 };

/* STATUS word positions, fbproto.STATUS_NAMES order (layout 9, 101 words; A builds before layout 8 send 100, the rest reads 0) */
enum {
    G2_ST_LAYOUT, G2_ST_BUILD, G2_ST_UPTIME_MS, G2_ST_FLAGS, G2_ST_PHY, G2_ST_RX_OK, G2_ST_RX_FCS, G2_ST_RX_LEN, G2_ST_RX_OVF,
    G2_ST_RX_ER, G2_ST_ARP, G2_ST_ICMP, G2_ST_UDP, G2_ST_IPCSUM, G2_ST_OTHER, G2_ST_PFDROP, G2_ST_ICMPDROP, G2_ST_TX,
    G2_ST_TX_RESYNC, G2_ST_EXP_SEQ, G2_ST_ACC, G2_ST_DUP, G2_ST_GAP, G2_ST_FULL, G2_ST_BAD, G2_ST_ACKS, G2_ST_Q_USED, G2_ST_Q_FREE,
    G2_ST_CMDS, G2_ST_ERRS, G2_ST_LAST_ERR, G2_ST_LAST_ERR_SEQ, G2_ST_MT_ERR, G2_ST_MDIO_POLLS, G2_ST_SNAP_AGE, G2_ST_REPLIES,
    G2_ST_REPLIES_DROPPED,
    G2_ST_B_FLAGS, G2_ST_B_FRAMES, G2_ST_B_PIX_KHZ, G2_ST_B_UNDERRUNS_MODERR, G2_ST_B_CP, G2_ST_LINK_FLAGS, G2_ST_FWD, G2_ST_B_EVENTS,
    G2_ST_B_STATUS_PKTS, G2_ST_LINK_CRC_PAR, G2_ST_LINK_DISC_RETX,
    G2_ST_LINKCTL, G2_ST_LINK_PARMAP, G2_ST_LD_BURSTS, G2_ST_LD_BURSTS_LG, G2_ST_LD_HIST, G2_ST_LD_LAST_ERR, G2_ST_LD_LAST_WORD,
    G2_ST_LD_FILLS, G2_ST_LD_FILL_PERR, G2_ST_LD_PERRS, G2_ST_LINK_DROPS, G2_ST_LINK_FRM_TRN, G2_ST_B_LINK_CRC_PAR, G2_ST_B_LINK_INFO,
    G2_ST_B_IMAGES, G2_ST_B_PIXELS,
    G2_ST_B_LD_HIST, G2_ST_B_LD_LAST_ERR, G2_ST_B_LINK_PARMAP, G2_ST_B_XOFF, G2_ST_B_RETX, G2_ST_B_DISC, G2_ST_B_CMD_PKTS, G2_ST_B_RX_WORDS,
    G2_ST_A_XOFF, G2_ST_A_NAKS_TO, G2_ST_A_RX_WORDS, G2_ST_IP,       /* layout 11: board A's IPv4 address in use, 0 = none yet */
    G2_ST_CR_SENT, G2_ST_CR_CONSUMED, G2_ST_CR_WAIT, G2_ST_CUR_SENT_SYNC, G2_ST_B_CB_TAKEN, G2_ST_B_CUR_PAL, G2_ST_B_CURCMD_XOPS,
    G2_ST_B_X_PIXELS,
    G2_ST_GEOM_BUSY_TRIS, G2_ST_GEOM_ERR, G2_ST_RENDER_QUADS, G2_ST_RS_PKTS_INFLIGHT,
    G2_ST_B_CYC_FRAME, G2_ST_B_CYC_FLUSH, G2_ST_B_CYC_BAND, G2_ST_B_CYC_DRAW, G2_ST_B_CYC_FS, G2_ST_B_CYC_BANDLOAD, G2_ST_B_CYC_STARVED,
    G2_ST_B_CYC_ROPSTALL, G2_ST_A_CYC_GEOM, G2_ST_A_CYC_RENDER, G2_ST_A_CYC_RS_CREDIT, G2_ST_A_CYC_RS_FULL,
    G2_ST_A_VISITS,                                  /* layout 9: primitive-band visits of A's walk, cumulative (layout 8: culled quads) */
    /* layout 10: the geometry side's profile, cumulative cycle counters (host/fb/fbproto.py lists them): loader,
       shadow copy, feed, feed's refused reads, vertex core, assembler, vertex waiting for setup, setup, push waiting for the
       binner, binner, bin clear, {loads[31:16], draws[15:0]}, primitives into setup; the dispatcher's idle and D_GWAIT cycles */
    G2_ST_A_PROF_LOAD, G2_ST_A_PROF_COPY, G2_ST_A_PROF_FEED, G2_ST_A_PROF_FETCH_WAIT, G2_ST_A_PROF_CORE, G2_ST_A_PROF_ASM,
    G2_ST_A_PROF_VO_STALL, G2_ST_A_PROF_SETUP, G2_ST_A_PROF_PUSH_STALL, G2_ST_A_PROF_BIN, G2_ST_A_PROF_CLEAR,
    G2_ST_A_PROF_LOADS_DRAWS, G2_ST_A_PROF_PRIMS, G2_ST_A_DISP_IDLE, G2_ST_A_DISP_GWAIT,
    G2_ST_WORDS
};
#define G2_B_FLAG_MODE(f)      (((f) >> 4) & 7u)     /* decode_b_flags: the video mode */
#define G2_B_FLAG_XOPS(f)      (((f) >> 28) & 3u)    /* board B's fragment core: bit 0 SQRT / RCPM / RSQM, bit 1 CLAMP (0 on B builds without them): g2sc_set_target */
/* the units a core lacks, STATUS flags bits 30 / 31 (0 on every build that has them): bit 0 the integer divider, bit 1 SIN / COS — g2sc_set_absent */
#define G2_A_FLAG_ABSENT(f)    (((f) >> 30) & 3u)    /* board A's flags (G2_ST_FLAGS): the vertex core */
#define G2_B_FLAG_ABSENT(f)    (((f) >> 30) & 3u)    /* board B's flags (G2_ST_B_FLAGS): the fragment core */
#define G2_LINK_B_ALIVE(f)     ((f) & 1u)            /* decode_link_flags */
#define G2_LINK_TRAINED(f)     (((f) >> 2) & 1u)

typedef struct g2_board g2_board;

/* a sequenced command: its datagram's seq and its sub-message index (0 outside a batch); seq == 0: not sent (error) */
typedef struct g2_ref { uint32_t seq, idx; } g2_ref;
typedef struct g2_event { uint32_t code, a0, a1, a2; } g2_event;
typedef struct g2_error { uint32_t seq, code, type, detail, idx; } g2_error;

typedef struct g2_options {
    int window;                  /* 0 = 64 datagrams */
    int b_sub_index;             /* B echoes sub-message indices: commands B answers stay inside batches. default 1 */
    double drop_rx;              /* test only: probability of discarding a received READ_DATA */
    const char *record_path;     /* non-NULL: nothing goes to a socket, every datagram is appended to this file (parity) */
    int bind_port;
} g2_options;

g2_board *g2_open(const char *ip, int port, const g2_options *opt);
void g2_close(g2_board *b);
const char *g2_error_text(const g2_board *b);        /* the last failure in words */
void g2_set_log(void (*fn)(const char *line));       /* process-wide; every library error goes here */

/* session */
int g2_hello(g2_board *b, int new_session);          /* <0 on failure, else the expected seq */
int g2_status(g2_board *b, uint32_t out[G2_ST_WORDS], int *nwords);
int g2_check_boards(g2_board *b, int min_a_build);   /* build / link / B alive; <0 + error text when not usable */
int g2_reg_read(g2_board *b, uint32_t addr, uint32_t *value);
int g2_reg_write(g2_board *b, uint32_t addr, uint32_t value);

/* the send path */
g2_ref g2_send_seq(g2_board *b, uint8_t type, const void *body, size_t len);
g2_ref g2_send_seq2(g2_board *b, uint8_t type, const void *hdr, size_t hlen, const void *data, size_t dlen);
int g2_send_unseq(g2_board *b, uint8_t type, const void *body, size_t len);
int g2_batch_begin(g2_board *b);
int g2_batch_end(g2_board *b);
int g2_batch_flush(g2_board *b);                     /* send what the open batch holds and keep it open */
int g2_wait_acked(g2_board *b, double timeout_s);
int g2_wait_event(g2_board *b, g2_ref ref, g2_event *ev, double timeout_s);
/* An EVENT has no retry, and one lost must not stop a game for a whole timeout. Commands run in order, so a FENCE that answers proves what came before it is done: after patience_s with no answer this asks with one,
 * and again every patience_s. 0 = ref's EVENT in *ev, 1 = it was lost and the work is done (*ev says nothing), < 0 = an error EVENT, or nothing answered in timeout_s */
int g2_wait_done(g2_board *b, g2_ref ref, g2_event *ev, double patience_s, double timeout_s);
int g2_poll_event(g2_board *b, g2_ref ref, g2_event *ev);   /* 1: taken, 0: not arrived, <0: its EVENT was an error */
int g2_take_errors(g2_board *b, g2_error *out, int max);
const char *g2_error_name(uint32_t code);

/* counters (fbproto's n_*) */
typedef struct g2_counters {
    uint64_t n_resent, n_hb_resend, n_batched, n_batches, n_rerequested, dgrams, rd_later, rd_below, rd_tail, rd_dup, n_events, n_rx, n_lost_events;
    uint64_t img_evictions;                 /* image descriptors taken back from board B's full table, to be given again when named */
    uint32_t next_seq, ack_frame;
} g2_counters;
void g2_get_counters(g2_board *b, g2_counters *c);

/* resources and transfers */
g2_ref g2_create_buffer(g2_board *b, uint16_t handle, uint32_t base, uint32_t size, uint16_t usage);
g2_ref g2_destroy(g2_board *b, uint16_t handle);
int g2_write_buffer(g2_board *b, uint16_t handle, uint32_t offset, const void *data, size_t len);
int g2_mem_write(g2_board *b, uint32_t addr, const void *data, size_t len, int board);
int g2_read_buffer(g2_board *b, uint16_t handle, uint32_t offset, void *out, size_t len);
int g2_mem_read(g2_board *b, uint32_t addr, void *out, size_t len, int board);
int g2_memtest(g2_board *b, uint32_t addr, uint32_t len, uint32_t seed, uint32_t flags, int board, double timeout_s,
               uint32_t *errors, uint32_t *first_bad);

/* display. Board B's video modes = SET_MODE ids (src/rtl/disp/video_modes.svh): 0 640 x 480, 1 800 x 600, 2 1 280 x 720, 3 1 280 x 800,
 * 4 1 024 x 768 (VESA DMT); a board B without mode 4 answers SET_MODE 4 with an error. Ids from 5 are B's simulation modes, never set here */
#define G2_NMODES 5
extern const int G2_MODE_W[G2_NMODES], G2_MODE_H[G2_NMODES];
extern const double G2_MODE_HZ[G2_NMODES];
int g2_set_mode(g2_board *b, uint32_t mode, double timeout_s);
int g2_mode_of_flags(uint32_t b_flags);              /* the mode in STATUS word G2_ST_B_FLAGS; 2 (B's boot mode) for an id that is no mode here */
int g2_mode_fit(int w, int h, int with_xga);         /* the smallest mode holding w x h, by size 0, 1, 4, 2, 3 (4 only with_xga); 3 when none does */
/* *mode = the mode B runs; when g2_mode_fit's mode for w x h is another, SET_MODE it. A B that refuses mode 4 gets the smallest other mode that holds
 * w x h (1 024 x 768 then shows 1:1 in 1 280 x 800), and the refusal is remembered for this board. 0 = *mode holds w x h (or is 3), -1 = SET_MODE
 * failed (*mode unchanged, g2_error_text says why) */
int g2_mode_select(g2_board *b, int w, int h, int *mode);
g2_ref g2_present(g2_board *b, uint16_t handle, uint32_t frame_id, int at_once, int hold);
int g2_present_wait(g2_board *b, g2_ref ref, uint32_t *fid, uint32_t *vsyncs, uint32_t *dropped, double timeout_s);

/* board A's geometry */
g2_ref g2_frame_begin(g2_board *b);
g2_ref g2_vs_program(g2_board *b, uint16_t handle, uint32_t offset, uint32_t n_instr);
g2_ref g2_vs_uniforms(g2_board *b, uint32_t first, const uint32_t *fp24_vec4s, uint32_t count);   /* 4 words per vec4, FP24 bits */
g2_ref g2_vertex_attr(g2_board *b, int index, int enabled, int size, uint16_t handle, uint32_t stride, uint32_t offset,
                      const uint32_t const_fp24[4], int type, int normalized);
/* viewport = (sxs, sxo, sys, syo): window = ndc * s + o in 1/16 pixel */
g2_ref g2_vs_state(g2_board *b, const double viewport[4], int cull, int front_cw, uint32_t fsmask, uint32_t width, uint32_t height,   /* (+ rect: the draw's scissor box or NULL, g2_core.c) */
                   uint32_t draw_id, double line_width, double depth_near, double depth_far, const uint32_t rect[4]);
#define G2_A_ZCLIP0 147                             /* the first board A build with VS_STATE's z01: the clip volume 0 <= z <= w */
g2_ref g2_vs_state2(g2_board *b, const double viewport[4], int cull, int front_cw, uint32_t fsmask, uint32_t width, uint32_t height,
                    uint32_t draw_id, double line_width, double depth_near, double depth_far, int z01, const uint32_t rect[4]);
g2_ref g2_draw(g2_board *b, int mode, uint32_t count, uint32_t first, int indexed, uint16_t index_handle, uint32_t index_offset,
               int index_u8);

/* board B's per-draw state and frames */
g2_ref g2_draw_state(g2_board *b, uint32_t draw_id, const uint32_t h[23], const uint32_t *uniforms_fp24, uint32_t n_uniforms);
g2_ref g2_flush(g2_board *b, uint16_t handle, uint32_t flags, const uint32_t clear_c16[4], uint32_t clear_depth, uint32_t clear_stencil,
                const uint32_t scissor[4], uint16_t ds_handle, uint32_t stencil_mask);
int g2_fence(g2_board *b, double timeout_s);         /* FENCE + wait for it */
g2_ref g2_fence_async(g2_board *b, uint32_t *fid);

/* images on board B */
g2_ref g2_create_image(g2_board *b, uint16_t handle, uint32_t base, uint32_t width, uint32_t height, uint32_t fmt, uint32_t pitch,
                       int origin, uint32_t levels, uint32_t layers, int samples_log2, uint32_t usage);
g2_ref g2_create_volume(g2_board *b, uint16_t handle, uint32_t base, uint32_t width, uint32_t height, uint32_t depth, uint32_t fmt, uint32_t levels, uint32_t usage);  /* a 3D texture: w, h <= 256, depth <= 255; WRITE_IMAGE's layer is the slice */
int g2_write_image(g2_board *b, uint16_t handle, const void *data, size_t len, uint32_t tfmt, uint32_t x, uint32_t y, uint32_t w,
                   uint32_t h, uint32_t level, uint32_t layer, uint32_t first_pixel, uint32_t storage_fmt);
/* a rectangle of a larger bitmap: w x h pixels from `src`, rows src_pitch bytes apart */
int g2_write_image_rect(g2_board *b, uint16_t handle, const void *src, int32_t src_pitch, uint32_t tfmt, uint32_t x, uint32_t y,
                        uint32_t w, uint32_t h, uint32_t level, uint32_t layer, uint32_t storage_fmt);
int g2_read_image(g2_board *b, uint16_t handle, uint32_t tfmt, uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t level, uint32_t layer,
                  uint32_t storage_fmt, uint32_t first, uint32_t count, void *out);
g2_ref g2_fill_image(g2_board *b, uint16_t handle, uint32_t tfmt, uint32_t value, uint32_t x, uint32_t y, uint32_t w, uint32_t h,
                     uint32_t mask, uint32_t level, uint32_t layer);
g2_ref g2_copy_image(g2_board *b, uint16_t src, uint16_t dst, uint32_t sx, uint32_t sy, uint32_t sw, uint32_t sh, uint32_t dx, uint32_t dy,
                     uint32_t dw, uint32_t dh, int mirror_x, int mirror_y, int use_key, uint32_t key_lo, uint32_t key_hi, uint32_t slevel,
                     uint32_t slayer, uint32_t dlevel, uint32_t dlayer);

/* palette and hardware cursor */
g2_ref g2_set_palette(g2_board *b, uint32_t first, const uint32_t *colors, uint32_t count);
int g2_cursor_shape(g2_board *b, const uint32_t *words, uint32_t count);
g2_ref g2_cursor_show(g2_board *b, int mode, int hot_x, int hot_y, int has_pos, int x, int y, int take);
g2_ref g2_cursor_hide(g2_board *b);
g2_ref g2_cursor_move(g2_board *b, int x, int y);
int g2_cursor_pos(g2_board *b, int x, int y);        /* the unsequenced fast path */

/* numbers */
uint32_t g2_fp24(double x);                          /* FP24 bits: s1 e7 m16, bias 63, round to nearest even, flush to zero */
double g2_fp24_to_double(uint32_t bits);
uint32_t g2_c16(double f, int bits);                 /* a colour in the target's channel units: round(f * 16 * (2^bits - 1)) */
uint32_t g2_storage_bpp(uint32_t fmt);
uint32_t g2_transfer_bpp(uint32_t tfmt, uint32_t storage_fmt);
uint32_t g2_mip_chain_bytes(uint32_t width, uint32_t height, uint32_t bpp, uint32_t levels);   /* levels rounded to 4 B */
uint32_t g2_image_bytes(uint32_t width, uint32_t height, uint32_t bpp, uint32_t levels, uint32_t layers);   /* layers: 32 B */
uint32_t g2_vol_chain_bytes(uint32_t width, uint32_t height, uint32_t depth, uint32_t bpp, uint32_t levels);  /* the first levels of a 3D chain: where the next level starts */

/* heaps (first fit, host-owned) */
typedef struct g2_heap g2_heap;
g2_heap *g2_heap_create(uint32_t base, uint32_t end);
void g2_heap_destroy(g2_heap *h);
uint32_t g2_heap_alloc(g2_heap *h, uint32_t size, uint32_t align);   /* 0 = out of memory */
void g2_heap_free(g2_heap *h, uint32_t addr);
uint32_t g2_heap_free_bytes(const g2_heap *h);

/* descriptor handles: `limit` per table (0: 1 024 — the buffer tables). Image handles are virtual, up to G2_HANDLES_MAX: gpu2core maps them onto
   board B's table as a cache (g2_core.c vimg). B's image indices 0..9 are built-in test images, hosts use >= 16 */
#define G2_HANDLES_MAX 16384
typedef struct g2_handles { uint32_t used[G2_HANDLES_MAX / 32]; int limit; } g2_handles;
enum { G2_HK_BUF_A = 0x0000, G2_HK_BUF_B = 0x8000, G2_HK_IMG_B = 0xC000 };
int g2_handle_alloc(g2_handles *t, uint16_t kind, uint16_t *out);    /* <0 = table full */
void g2_handle_free(g2_handles *t, uint16_t handle);

#ifdef __cplusplus
}
#endif
#endif
