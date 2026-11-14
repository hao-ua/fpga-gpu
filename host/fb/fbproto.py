"""fbproto — host side of the remote-display / GPU resource protocol.

Wire format and message set: the RTL constants in
src/rtl/proto/fb_pkg.sv. Standard library only (Python 3.8+).

Datagram = 8-byte header (magic u16 0x4246, version u8 1, type u8, seq u32, little-endian) + body.
seq == 0: unsequenced (HELLO, STATUS_REQ, PHY_REG, REG). seq != 0: Go-Back-N on board A — the board
accepts only the expected seq; ACKs carry the expected seq, NAK / FULL flags and the free ingress-
queue bytes (credit). This module resends from the expected seq on NAK, FULL (after a short wait), on an
ACK (board A sends a heartbeat ACK every ~1 ms) that still expects the oldest unacknowledged datagram
after 2 ms without progress, or when nothing progressed for 10 ms.

BATCH (type 0x60): `with board.batch():` collects the sequenced commands sent inside the
block into datagrams of up to BATCH_MAX body bytes, each a sequence of sub-messages {u8 type, u8 flags,
u16 bytes, body padded to 4} executed in order under the datagram's one seq. Every EVENT carries the
sub-message index in its code word (bits 15..8), so events are keyed by (seq, index): send_* inside a
batch return a SubRef that wait_event() accepts (it sends the batch first if it is still open).
"""
import collections
import os
import random
import socket
import struct
import threading
import time

MAGIC, VERSION = 0x4246, 1

# host -> board
T_HELLO, T_STATUS_REQ, T_PHY_REG, T_MEM_WRITE, T_MEM_READ, T_MEMTEST, T_REG = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
T_CREATE_BUFFER, T_CREATE_IMAGE, T_DESTROY = 0x10, 0x11, 0x12
T_WRITE_BUFFER, T_READ_BUFFER, T_WRITE_IMAGE, T_READ_IMAGE = 0x20, 0x21, 0x22, 0x23
T_FILL_IMAGE, T_COPY_IMAGE, T_GEN_MIPMAP, T_COPY_BUFFER = 0x24, 0x25, 0x26, 0x27
T_SET_MODE, T_PRESENT, T_CURSOR, T_CURSOR_POS, T_SET_PALETTE = 0x30, 0x31, 0x32, 0x33, 0x34
T_FENCE, T_FLUSH = 0x40, 0x41
T_DRAW_STATE = 0x50                                            # board B's per-draw state
T_VS_PROGRAM, T_VS_UNIFORMS, T_VERTEX_ATTR, T_VS_STATE, T_DRAW, T_FRAME_BEGIN = 0x51, 0x52, 0x53, 0x54, 0x55, 0x56   # board A's geometry
T_BATCH = 0x60                                                 # several commands in one datagram
BATCH_MAX = 1464                                               # body bytes of a BATCH datagram (8-byte header + body <= 1472, the UDP payload of a 1500-byte MTU)
B_EVENT_TYPES = (T_FENCE, T_PRESENT, T_SET_MODE, T_MEMTEST)    # commands board B may answer with an EVENT (see Board.b_sub_index)
AT_BYTE, AT_UBYTE, AT_SHORT, AT_USHORT, AT_FIXED, AT_FLOAT, AT_FP24 = range(7)   # VERTEX_ATTR component formats (attr_conv.sv)
AT_GL = {0x1400: AT_BYTE, 0x1401: AT_UBYTE, 0x1402: AT_SHORT, 0x1403: AT_USHORT, 0x140C: AT_FIXED, 0x1406: AT_FLOAT}   # glVertexAttribPointer type -> code
CUR_SHAPE, CUR_SHOW, CUR_HIDE, CUR_MOVE = 1, 2, 3, 4          # CURSOR ops
CUR_COLOR, CUR_ANDXOR, CUR_PREMUL = 0, 1, 2                   # cursor modes
COPY_MIRROR_X, COPY_MIRROR_Y, COPY_KEY = 1, 2, 4               # COPY_IMAGE flags
MASK_R, MASK_G, MASK_B, MASK_A = 1, 2, 4, 8                    # FILL_IMAGE channel mask
# board -> host
R_ACK, R_STATUS, R_EVENT, R_READ_DATA, R_PHY_REG, R_REG = 0x81, 0x82, 0x83, 0x84, 0x85, 0x86
EV_FENCE, EV_PRESENT, EV_MODE, EV_MEMTEST, EV_ERROR = 0x01, 0x02, 0x03, 0x04, 0x10
ERR_NAMES = {1: "unknown type", 2: "bad handle", 3: "out of range", 4: "outside heap",
             5: "board B not connected", 6: "bad length/alignment", 7: "bad board", 8: "video mode did not lock",
             9: "render frame aborted (B did not take the render stream in time)",
             10: "fragment program stack violation (quads of the FLUSH's frame were killed)"}
# board B video modes (src/rtl/disp/video_modes.svh, tools/mmcm_drp_calc.py)
MODE_NAMES = {0: "640x480@60", 1: "800x600@60", 2: "1280x720@60", 3: "1280x800@60",
              4: "1024x768@60"}
PRES_AT_ONCE, PRES_HOLD = 1, 2          # PRESENT flags
# FLUSH flags: clear colour, clear scissor, colour mask {R,G,B,A} << 2, dither the clear, clear depth,
# clear stencil, the depth attachment has 16 bits
FL_CLEAR_COLOR, FL_CLEAR_SCISSOR, FL_DITHER, FL_CLEAR_DEPTH, FL_CLEAR_STENCIL, FL_DEPTH16 = 1, 2, 64, 128, 256, 512
def FL_CMASK(m): return (m & 0xF) << 2
# depth / stencil functions of DRAW_STATE (GL order), depth func in h0 [4:2]
FN_NEVER, FN_LESS, FN_EQUAL, FN_LEQUAL, FN_GREATER, FN_NOTEQUAL, FN_GEQUAL, FN_ALWAYS = range(8)
# image formats: storage codes, transfer codes, usage bits (img_pkg.sv)
FMT_RGBA8, FMT_RGB565, FMT_P8 = 0x00, 0x04, 0x0C
STORAGE_BPP = {FMT_RGBA8: 4, FMT_RGB565: 2, FMT_P8: 1}
TF_RAW, TF_RGBA8888, TF_BGRA8888, TF_BGRX8888, TF_RGB565, TF_RGB888, TF_BGR888 = 0x00, 0x01, 0x02, 0x03, 0x04, 0x0A, 0x0B
TF_NAMES = {"raw": TF_RAW, "rgba": TF_RGBA8888, "bgra": TF_BGRA8888, "bgrx": TF_BGRX8888, "rgb565": TF_RGB565,
            "rgb": TF_RGB888, "bgr": TF_BGR888}
USE_SAMPLED, USE_RT, USE_SCANOUT, USE_TRANSFER = 1, 2, 4, 8




def image_handle(index):
    """Handle of a board-B image: bit 15 = board B, bit 14 = image, bits 9..0 = table index.
    Entries 0..9 are board B's built-in test images."""
    return 0xC000 | (index & 0x3FF)
ACKF_NAK, ACKF_FULL = 1, 2
REG_IDELAY, REG_PHYOPT, REG_PHYAD, REG_REINIT, REG_SCRATCH, REG_LINKCTL, REG_BUILD = 0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0010
IDELAY_TRIAL = 0x100

# board A's address: GPU2_IP (an address or a host name), else the address board A falls back to without a DHCP lease
BOARD_IP = os.environ.get("GPU2_IP") or "192.168.1.10"

MAX_DATA = 1440            # data bytes per WRITE_* datagram with a 1500-byte MTU (8 header + 8 fields + 1440)
RD_CHUNK = 1440            # READ_DATA chunk size used by the board
HEAP_A = 0x1800_0000       # board A host heap: [384 MB, 1 GB) (below: the ingress ring, two banks of records and bin lists)
HEAP_A_END = 0x4000_0000

STATUS_NAMES = [
    "layout", "build", "uptime_ms", "flags", "phy", "rx_ok", "rx_fcs", "rx_len", "rx_ovf", "rx_er", "arp", "icmp",
    "udp", "ipcsum", "other", "pfdrop", "icmpdrop", "tx", "tx_resync", "exp_seq", "acc", "dup", "gap", "full", "bad",
    "acks", "q_used", "q_free", "cmds", "errs", "last_err", "last_err_seq", "mt_err", "mdio_polls", "snap_age",
    "replies", "replies_dropped",
    # layout 2: board B (from its link STATUS packets) and the board link (board_a_core.sv)
    "b_flags", "b_frames", "b_pix_khz", "b_underruns_moderr", "b_cp", "link_flags", "fwd", "b_events",
    "b_status_pkts", "link_crc_par", "link_disc_retx",
    # layout 3: link diagnostics of A's receiver (link_rx_diag), B's receiver, B's transfer counters
    "linkctl", "link_parmap", "ld_bursts", "ld_bursts_lg", "ld_hist", "ld_last_err", "ld_last_word", "ld_fills",
    "ld_fill_perr", "ld_perrs", "link_drops", "link_frm_trn", "b_link_crc_par", "b_link_info", "b_images", "b_pixels",
    # layout 4: B's receive diagnostics (histogram, last error, parity map), XOFF events of both RX FIFOs, B's
    # transmitter {timeouts, rewinds}, B's receiver {NAKs, discards}, CMD packets B received, words received by
    # both receivers, A's {NAKs, timeouts}
    "b_ld_hist", "b_ld_last_err", "b_link_parmap", "b_xoff", "b_retx", "b_disc", "b_cmd_pkts", "b_rx_words",
    "a_xoff", "a_naks_to", "a_rx_words",
    # layout 11: the IPv4 address in use (0 = none yet)
    "ip",
    # layout 5: credit flow control (A words sent / B words taken as reported / A cycles waiting),
    # {cursor fast-path packets sent, credit SYNC packets sent}, B words taken, B {cursor packets, palette publishes},
    # B {cursor commands, READ / FILL / COPY operations}, B READ / FILL / COPY pixels
    "cr_sent", "cr_consumed", "cr_wait", "cur_sent_sync", "b_cb_taken", "b_cur_pal", "b_curcmd_xops", "b_x_pixels",
    # layout 6: A's geometry stage {busy[20], triangle records since FRAME_BEGIN}, the
    # sticky flags {bin pool / band count overflow[2], record region overflow[1] (both per frame), the vertex core's
    # stack violation[0]}, the render side's quads sent, {render beats in flight[31:16], RSTREAM packets sent[15:0]}
    "geom_busy_tris", "geom_err", "render_quads", "rs_pkts_inflight",
    # layout 7: B's render engine, cycles of its LAST frame (clk_sys): the frame, the band
    # flush engine busy, the sequencer at band changes (drain + flush wait), at draw changes (drain + bucket / program
    # load), the fragment core busy, band set-up (clear / wipe / load), the render stream empty (A / link bound), the
    # back end stalled by the ROP; then A's CUMULATIVE cycle counters (take deltas): geometry side busy, render side
    # (band walk) busy, render beats waiting for B's credit, a_render stalled by the full render FIFO
    "b_cyc_frame", "b_cyc_flush", "b_cyc_band", "b_cyc_draw", "b_cyc_fs", "b_cyc_bandload", "b_cyc_starved", "b_cyc_ropstall",
    "a_cyc_geom", "a_cyc_render", "a_cyc_rs_credit", "a_cyc_rs_full",
    # layout 9: the primitive-band visits of A's walk, cumulative (layout 8 is not used)
    "a_visits",
    # layout 10: the geometry side's profile, cumulative cycle counters (a_geom; take deltas, divide by
    # a_cyc_geom's delta; the stages overlap, the shares do not add up): the program loader, the uniform / descriptor
    # shadow copy, the vertex feed busy, the feed's DDR read request refused, the vertex core not idle, the assembler
    # busy, a vertex waiting for the setup, the setup busy, the setup's push waiting for the binner, the binner busy,
    # the frame's bin clear, {program loads[31:16], draws[15:0]}, primitives presented to the setup; then the
    # dispatcher's idle cycles (no command queued: host-bound) and its cycles waiting for a geometry command (D_GWAIT)
    "a_prof_load", "a_prof_copy", "a_prof_feed", "a_prof_fetch_wait", "a_prof_core", "a_prof_asm", "a_prof_vo_stall",
    "a_prof_setup", "a_prof_push_stall", "a_prof_bin", "a_prof_clear", "a_prof_loads_draws", "a_prof_prims",
    "a_disp_idle", "a_disp_gwait"]
B_CYC_NAMES = ["frame", "flush", "band change", "draw change", "FS busy", "band set-up", "stream empty", "ROP stall"]
# the boards' clk_sys in kHz, for cycle counters -> ms (GPU2_CLK_MHZ, default 120)
CLK_KHZ = float(os.environ.get("GPU2_CLK_MHZ", "120")) * 1e3
A_PROF_NAMES = [("load", "a_prof_load"), ("copy", "a_prof_copy"), ("feed", "a_prof_feed"), ("fetch wait", "a_prof_fetch_wait"),
                ("core", "a_prof_core"), ("asm", "a_prof_asm"), ("vo stall", "a_prof_vo_stall"), ("setup", "a_prof_setup"),
                ("push stall", "a_prof_push_stall"), ("bin", "a_prof_bin"), ("clear", "a_prof_clear")]


def a_geom_split(st, st0, frames=1):
    """A's geometry side between two STATUS snapshots: the busy cycles per frame and each stage's share of them, the
    program loads / draws / primitives per frame, the dispatcher's idle and geometry-wait cycles per frame"""
    if st.get("layout", 0) < 10: return "no A profile counters (STATUS layout < 10)"
    d = lambda k: (st[k] - st0[k]) & 0xFFFFFFFF
    g = d("a_cyc_geom")
    if g == 0 or frames <= 0: return "A geometry idle"
    parts = ", ".join("%s %.0f%%" % (n, 100.0 * d(k) / g) for n, k in A_PROF_NAMES)
    ld = (d("a_prof_loads_draws") >> 16) & 0xFFFF; dr = d("a_prof_loads_draws") & 0xFFFF
    return ("A geometry %.0f kcycles/frame: %s | per frame: loads %.1f draws %.1f prims %.0f | dispatcher idle %.0f k, "
            "geometry wait %.0f k cycles/frame" % (g / 1e3 / frames, parts, ld / frames, dr / frames, d("a_prof_prims") / frames,
                                                   d("a_disp_idle") / 1e3 / frames, d("a_disp_gwait") / 1e3 / frames))


def b_frame_split(st):
    """B's last frame in ms with the share of each counter: 'frame 9.07 ms: flush 94.1%, band change 3.2%, ...'"""
    if "b_cyc_frame" not in st: return "no B cycle counters (STATUS layout < 7)"
    f = st["b_cyc_frame"]
    if f == 0: return "no frame yet"
    parts = ", ".join("%s %.1f%%" % (n, 100.0 * st["b_cyc_" + k] / f) for n, k in
                      zip(B_CYC_NAMES[1:], ["flush", "band", "draw", "fs", "bandload", "starved", "ropstall"]))
    return "B frame %.2f ms: %s" % (f / CLK_KHZ, parts)


def c16(f, bits=8):
    """a colour component f in [0, 1] in the render target's channel units: round(f * 16 * (2^bits - 1)), the
    unit of DRAW_STATE's blend constant and FLUSH's clear colour (4080 = 1.0 for RGBA8)"""
    return round(min(max(f, 0.0), 1.0) * 16 * ((1 << bits) - 1))


GUARD_BAND = 32640.0       # board A clips x / y at +-2040 px (1/16-pixel units; vtx_setup2 saturates at +-2048 px)


def fp24_to_float(bits):
    """FP24 bits -> float (Inf / NaN as such; subnormal fields read as zero, as the hardware does)"""
    s = -1.0 if (bits >> 23) & 1 else 1.0
    e = (bits >> 16) & 0x7F; m = bits & 0xFFFF
    if e == 0x7F: return s * float('inf') if m == 0 else float('nan')
    if e == 0: return 0.0 * s
    return s * (1.0 + m / 65536.0) * 2.0 ** (e - 63)


def fp24(x):
    """float -> FP24 bits (s1 e7 m16, bias 63, round to nearest even, flush to zero), the GPU's number format"""
    import math
    if x == 0: return 0
    if math.isinf(x): return (0x7F << 16) | (0x800000 if x < 0 else 0)
    if math.isnan(x): return (0x7F << 16) | 0x8000
    s = 0x800000 if x < 0 else 0
    m, e = math.frexp(abs(x))                    # abs(x) = m * 2^e, m in [0.5, 1)
    e -= 1; m *= 2                               # m in [1, 2)
    q = round(m * (1 << 16))                     # round half to even (Python 3 round)
    if q == (1 << 17): q >>= 1; e += 1
    if e + 63 <= 0: return s                     # FTZ
    if e + 63 >= 0x7F: return s | (0x7F << 16)
    return s | ((e + 63) << 16) | (q & 0xFFFF)


class ProtoError(Exception):
    pass


class SubRef:
    """a command inside a BATCH: (its datagram's seq once sent, its sub-message index); wait_event() takes it"""
    __slots__ = ("dgram", "idx")

    def __init__(self, dgram, idx):
        self.dgram, self.idx = dgram, idx

    @property
    def seq(self):
        return self.dgram.seq

    def __repr__(self):
        return "SubRef(seq=%s, idx=%d)" % (self.dgram.seq, self.idx)


class _BatchDgram:
    """one BATCH datagram being filled: the packed sub-messages, their byte count, the seq once sent"""
    __slots__ = ("parts", "size", "seq")

    def __init__(self):
        self.parts, self.size, self.seq = [], 0, None


class _BatchCtx:
    def __init__(self, board):
        self.board = board

    def __enter__(self):
        if self.board._batch is not None:
            raise ProtoError("batch() blocks do not nest")
        self.board._batch = _BatchDgram()
        return self

    def __exit__(self, *exc):
        try:
            if exc[0] is None:
                self.board._flush_batch()
        finally:
            self.board._batch = None


DHCP_STATES = ["idle", "delay", "selecting", "requesting", "ack", "probing", "bound", "renewing", "rebinding", "hold"]


def decode_flags(f):
    return {"calib": f & 1, "link": (f >> 1) & 1, "speed": ("10M", "100M", "1000M", "?")[(f >> 2) & 3],
            "fdx": (f >> 4) & 1, "mdio_init": (f >> 5) & 1, "host": (f >> 6) & 1, "b_present": (f >> 7) & 1,
            "tap": (f >> 8) & 31, "rxdly": (f >> 13) & 1, "adv1000": (f >> 14) & 1, "phyad": (f >> 16) & 31,
            # layout 11: the DHCP client
            "dhcp": (f >> 21) & 15, "ip_valid": (f >> 25) & 1, "ip_leased": (f >> 26) & 1,
            # the vertex core lacks the integer divider / SIN / COS (glslc noops for the vertex stage: (f >> 30) & 3)
            "no_idiv": (f >> 30) & 1, "no_sincos": (f >> 31) & 1}


def format_ip(st):
    """the address in use and where it came from: '192.168.0.57 (DHCP lease, bound)' (STATUS layout 11)"""
    if st.get("layout", 0) < 11: return "address not reported (STATUS layout < 11)"
    fl = decode_flags(st["flags"]); a = st["ip"]
    ds = DHCP_STATES[fl["dhcp"]] if fl["dhcp"] < len(DHCP_STATES) else "?"
    if not fl["ip_valid"]: return "no address yet (DHCP %s)" % ds
    return "%d.%d.%d.%d (%s, DHCP %s)" % (a >> 24, (a >> 16) & 255, (a >> 8) & 255, a & 255,
                                          "DHCP lease" if fl["ip_leased"] else "fallback", ds)


def decode_b_flags(f):
    return {"calib": f & 1, "mmcm_locked": (f >> 1) & 1, "video": (f >> 2) & 1, "mode_busy": (f >> 3) & 1,
            "mode": (f >> 4) & 7, "image_shown": (f >> 7) & 1, "fill_done": (f >> 8) & 1, "lock_fail": (f >> 11) & 1,
            "cp_busy": (f >> 12) & 1, "cur_img": (f >> 16) & 1023, "render_busy": (f >> 26) & 1,
            "fs_stack_err": (f >> 27) & 1, "xsfu": (f >> 28) & 1, "xclamp": (f >> 29) & 1,
            # the fragment core lacks the integer divider / SIN / COS (glslc noops for the fragment stage: (f >> 30) & 3)
            "no_idiv": (f >> 30) & 1, "no_sincos": (f >> 31) & 1}


def decode_link_flags(f):
    return {"b_alive": f & 1, "lock": (f >> 1) & 1, "trained": (f >> 2) & 1, "far_up": (f >> 3) & 1,
            "training": (f >> 4) & 1, "tx_enabled": (f >> 5) & 1, "guard_fail": (f >> 6) & 1,
            "phase": (f >> 16) & 255, "eye": (f >> 24) & 255}


def format_status(st):
    fl = decode_flags(st["flags"])
    s = ("build %d up %.1fs calib %d link %d %s fdx %d tap %d rxdly %d adv1000 %d PHYSR %04X BMSR %04X\n"
         % (st["build"], st["uptime_ms"] / 1000.0, fl["calib"], fl["link"], fl["speed"], fl["fdx"], fl["tap"],
            fl["rxdly"], fl["adv1000"], st["phy"] & 0xFFFF, st["phy"] >> 16))
    s += "  address %s\n" % format_ip(st)
    s += "  rx ok %(rx_ok)d fcs %(rx_fcs)d len %(rx_len)d ovf %(rx_ovf)d er %(rx_er)d | arp %(arp)d icmp %(icmp)d " \
         "udp %(udp)d ipcsum %(ipcsum)d other %(other)d pfdrop %(pfdrop)d | tx %(tx)d\n" % st
    s += "  proto exp %(exp_seq)d acc %(acc)d dup %(dup)d gap %(gap)d full %(full)d bad %(bad)d acks %(acks)d | " \
         "queue used %(q_used)d free %(q_free)d | cmds %(cmds)d errs %(errs)d last_err %(last_err)08X@%(last_err_seq)d " \
         "| memtest %(mt_err)d | replies %(replies)d dropped %(replies_dropped)d" % st
    if "link_flags" in st:
        lf, bf = decode_link_flags(st["link_flags"]), decode_b_flags(st["b_flags"])
        s += ("\n  link trained %d far_up %d lock %d B alive %d eye %d | forwarded %d B events %d B status %d "
              "| crc %d par %d disc %d rewinds %d"
              % (lf["trained"], lf["far_up"], lf["lock"], lf["b_alive"], lf["eye"], st["fwd"], st["b_events"],
                 st["b_status_pkts"], st["link_crc_par"] >> 16, st["link_crc_par"] & 0xFFFF,
                 st["link_disc_retx"] >> 16, st["link_disc_retx"] & 0xFFFF))
        if "ld_bursts" in st:
            h, le = st["ld_hist"], st["ld_last_err"]
            s += ("\n  link diag: fill A %d B %d | A rx parity %d (per half %02X/%02X) fill words %d (parity %d) lock losses %d "
                  "trainings %d | errors at burst pos 0 / 1-3 / 4+ : %d / %d / %d, after long gap %d | bursts %d (long gap %d)"
                  " | last error gap %d pos %d last %d par %X word %08X"
                  % (st["linkctl"] & 1, (st["b_link_info"] >> 16) & 1, st["ld_perrs"], (st["link_parmap"] >> 8) & 255,
                     st["link_parmap"] & 255, st["ld_fills"], st["ld_fill_perr"], st["link_drops"], st["link_frm_trn"] & 255,
                     h >> 24, (h >> 16) & 255, (h >> 8) & 255, h & 255, st["ld_bursts"], st["ld_bursts_lg"],
                     le >> 16, (le >> 8) & 255, (le >> 4) & 1, le & 15, st["ld_last_word"]))
            s += ("\n  B receiver: crc %d par %d parity (real words) %d trainings %d | B images created %d, pixels written %d"
                  % (st["b_link_crc_par"] >> 16, st["b_link_crc_par"] & 0xFFFF, st["b_link_info"] & 0xFFFF,
                     st["b_link_info"] >> 24, st["b_images"], st["b_pixels"]))
        if "b_ld_hist" in st:
            bh, ble = st["b_ld_hist"], st["b_ld_last_err"]
            s += ("\n  B rx diag: errors at burst pos 0 / 1-3 / 4+ : %d / %d / %d, after long gap %d | last error gap %d pos %d "
                  "last %d par %X | per half %08X | words %d | B xoff %d | B tx timeouts %d rewinds %d | B rx naks %d disc %d "
                  "| CMD packets %d (A forwarded %d)"
                  % (bh >> 24, (bh >> 16) & 255, (bh >> 8) & 255, bh & 255, ble >> 16, (ble >> 8) & 255, (ble >> 4) & 1,
                     ble & 15, st["b_link_parmap"], st["b_rx_words"], st["b_xoff"], st["b_retx"] >> 16, st["b_retx"] & 0xFFFF,
                     st["b_disc"] >> 16, st["b_disc"] & 0xFFFF, st["b_cmd_pkts"], st["fwd"]))
            s += ("\n  A rx: words %d | A xoff %d | A rx naks %d | A tx timeouts %d"
                  % (st["a_rx_words"], st["a_xoff"], st["a_naks_to"] >> 16, st["a_naks_to"] & 0xFFFF))
        if "cr_sent" in st:
            s += ("\n  credit: A sent %d, B reported %d, B taken %d | A waited %d cycles | credit SYNCs %d | cursor packets "
                  "A %d B %d" % (st["cr_sent"], st["cr_consumed"], st["b_cb_taken"], st["cr_wait"],
                                 st["cur_sent_sync"] & 0xFFFF, st["cur_sent_sync"] >> 16, st["b_cur_pal"] >> 16))
        if lf["b_alive"]:
            s += ("\n  B video %d mode %d (%s) image %d shown %d | frames %d pixel %d kHz | underruns %d mode errors %d "
                  "| commands %d errors %d"
                  % (bf["video"], bf["mode"], MODE_NAMES.get(bf["mode"], "?"), bf["cur_img"], bf["image_shown"],
                     st["b_frames"], st["b_pix_khz"], st["b_underruns_moderr"] >> 16, st["b_underruns_moderr"] & 0xFFFF,
                     st["b_cp"] & 0xFFFF, st["b_cp"] >> 16))
            # B's flags bits 28 / 29: the fragment core has SQRT / RCPM / RSQM, CLAMP
            xo = [n for b, n in ((28, "SQRT RCPM RSQM"), (29, "CLAMP")) if st["b_flags"] >> b & 1]
            s += "\n  B fragment core operations beyond the base set: %s" % (", ".join(xo) if xo else "none")
    return s


class Board:
    """Board A reached over UDP; commands for board B go through it over the board link."""

    def __init__(self, ip=None, port=5000, bind_port=0, window=64, verbose=False, drop_rx=0.0, b_sub_index=False):
        # an address or a host name (resolved once); default GPU2_IP, else 192.168.1.10
        self.addr = (socket.gethostbyname(ip or BOARD_IP), port)
        self.verbose = verbose
        self.window = window
        self._batch = None                            # the BATCH datagram being filled inside a batch() block
        # a board B that echoes a command's sub-message index in its EVENTs (b_sub_index=True) takes it in a batch;
        # otherwise a command B answers with an EVENT (B_EVENT_TYPES) is sent alone, after the batch collected so far
        self.b_sub_index = b_sub_index
        self.n_batched = 0                            # commands sent inside BATCH datagrams
        self.n_batches = 0                            # BATCH datagrams sent
        self.drop_rx = drop_rx                        # test only: probability of discarding a received READ_DATA
        self._rng = random.Random(1)
        self.n_rerequested = 0                        # READ_DATA chunks re-requested after a loss (all reads)
        self.read_loss = collections.Counter()        # chunks declared lost, by rule: later / below / tail
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
        self.sock.bind(("0.0.0.0", bind_port))
        self.sock.settimeout(0.05)
        self.cv = threading.Condition()
        self.next_seq = 1
        self.unacked = collections.OrderedDict()      # seq -> datagram bytes
        self.ack_exp = None
        self.ack_count = 0
        self.credit = 1 << 20
        self.resend_req = 0                           # 0 none, 1 now, 2 after FULL wait
        self.last_progress = time.monotonic()
        self.last_resend = 0.0
        self.n_resent = 0
        self.n_hb_resend = 0                          # resends triggered by the heartbeat rule (not by a NAK / FULL / the stall timer)
        self.hb_suspect = None                        # (seq, time) the heartbeat rule suspects lost, confirmed by _hb_check
        self.status_words = None
        self.status_count = 0
        self.events = {}                              # (seq, sub-message index) -> (code, a0, a1, a2)
        self.errors = []                              # (seq, code, type, detail, sub-message index) of EV_ERROR events
        self.reads = {}                               # tag -> dict(offset -> bytes)
        self.lreplies = collections.deque()           # (type, w2, w3)
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.mt_thread = threading.Thread(target=self._maint_loop, daemon=True)
        self.mt_thread.start()

    def close(self):
        self.running = False
        self.rx_thread.join(1)
        self.mt_thread.join(1)
        self.sock.close()

    # ------------------------------------------------------------------ receive
    def _rx_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < 8:
                continue
            magic, ver, typ, seq = struct.unpack_from("<HBBI", data, 0)
            if magic != MAGIC or ver != VERSION:
                continue
            body = data[8:]
            with self.cv:
                if typ == R_ACK and len(body) >= 16:
                    exp, flags, free, _frame = struct.unpack_from("<IIII", body, 0)
                    self._on_ack(exp, flags, free)
                elif typ == R_STATUS:
                    self.status_words = list(struct.unpack_from("<%dI" % (len(body) // 4), body, 0))
                    self.status_count += 1
                elif typ == R_EVENT and len(body) >= 16:
                    codew, a0, a1, a2 = struct.unpack_from("<IIII", body, 0)
                    code, idx = codew & 0xFF, (codew >> 8) & 0xFF
                    self.events[(seq, idx)] = (code, a0, a1, a2)
                    if code == EV_ERROR:
                        self.errors.append((seq, a0, a1, a2, idx))
                elif typ == R_READ_DATA and len(body) >= 8:
                    tag, off = struct.unpack_from("<II", body, 0)
                    if self.drop_rx and self._rng.random() < self.drop_rx:
                        continue
                    if tag in self.reads:                 # chunks of finished / unknown reads are ignored
                        self.reads[tag][off] = body[8:]
                elif typ in (R_PHY_REG, R_REG) and len(body) >= 8:
                    w2, w3 = struct.unpack_from("<II", body, 0)
                    self.lreplies.append((typ, w2, w3))
                self.cv.notify_all()

    def _on_ack(self, exp, flags, free):
        self.ack_count += 1
        progressed = False
        while self.unacked:
            s = next(iter(self.unacked))
            if ((s - exp) & 0xFFFFFFFF) >= 0x80000000:   # s < exp (wrap-safe)
                self.unacked.popitem(last=False)
                progressed = True
            else:
                break
        now = time.monotonic()
        if progressed:
            self.last_progress = now
        self.ack_exp = exp
        self.credit = free
        if self.unacked and (flags & ACKF_NAK):
            self.resend_req = max(self.resend_req, 1)
        if self.unacked and (flags & ACKF_FULL):
            self.resend_req = 2
        # the heartbeat rule: an ACK (A sends one every ~1 ms) that still expects our oldest unacknowledged datagram
        # although nothing progressed for 2 ms means that datagram (or its NAK) was lost. The ACK may be STALE, though:
        # generated before the datagram was even sent and processed late by this thread (Python's GIL hands the receive
        # thread over every 5 ms while the main thread computes, so such triggers are frequent and mostly false), so it
        # only marks a suspect; _maint_loop resends if nothing progressed for 2 ms after the mark
        if progressed:
            self.hb_suspect = None
        elif (self.unacked and self.hb_suspect is None and exp == next(iter(self.unacked))
                and now - self.last_progress > 0.002):
            self.hb_suspect = (exp, now)

    def _hb_check(self, now):
        """_maint_loop: resend if the suspect of the heartbeat rule is still the oldest datagram 2 ms later (no progress
        meanwhile, no resend in the last 4 ms); called with the lock held"""
        s = self.hb_suspect
        if s is None:
            return
        if not self.unacked or next(iter(self.unacked)) != s[0]:
            self.hb_suspect = None
        elif now - s[1] >= 0.002 and now - self.last_resend > 0.004:
            self.hb_suspect = None
            self.resend_req = max(self.resend_req, 1)
            self.n_hb_resend += 1

    # ------------------------------------------------------------------ send / resend
    def _maint_loop(self):
        while self.running:
            time.sleep(0.002)
            with self.cv:
                if not self.unacked:
                    self.resend_req = 0
                    self.hb_suspect = None
                    continue
                now = time.monotonic()
                self._hb_check(now)
                wait = 0.002 if self.resend_req == 1 else 0.005
                stalled = now - self.last_progress > 0.01      # the stall timer: 10 ms
                if (self.resend_req and now - self.last_resend > wait) or (stalled and now - self.last_resend > 0.01):
                    pkts = list(self.unacked.values())
                    self.resend_req = 0
                    self.last_resend = now
                else:
                    pkts = None
            if pkts:
                for p in pkts:
                    self.sock.sendto(p, self.addr)
                self.n_resent += len(pkts)

    @staticmethod
    def _msg(typ, seq, body=b""):
        return struct.pack("<HBBI", MAGIC, VERSION, typ, seq) + body

    def send_unseq(self, typ, body=b""):
        self.sock.sendto(self._msg(typ, 0, body), self.addr)

    def send_seq(self, typ, body=b"", timeout=10.0):
        """send one sequenced command; inside a batch() block it is collected instead and a SubRef is returned
        (a command B answers with an EVENT goes alone unless b_sub_index, after the batch collected so far)"""
        b = self._batch
        if b is not None:
            if typ in B_EVENT_TYPES and not self.b_sub_index:
                self._flush_batch(timeout)
            else:
                n = 4 + ((len(body) + 3) & ~3)
                if n > BATCH_MAX:
                    raise ProtoError("command type %02X of %d bytes does not fit a BATCH datagram" % (typ, len(body)))
                if b.size + n > BATCH_MAX:
                    self._flush_batch(timeout)
                    b = self._batch
                ref = SubRef(b, len(b.parts))
                b.parts.append(struct.pack("<BBH", typ, 0, len(body)) + body + b"\0" * (-len(body) & 3))
                b.size += n
                self.n_batched += 1
                return ref
        return self._send_seq(typ, body, timeout)

    def _flush_batch(self, timeout=10.0):
        """send the BATCH datagram being filled (if any) and start a new one"""
        b = self._batch
        if b is None or not b.parts:
            return
        self._batch = _BatchDgram()
        b.seq = self._send_seq(T_BATCH, b"".join(b.parts), timeout)
        self.n_batches += 1

    def batch(self):
        """`with board.batch():` — the sequenced commands sent inside go out as BATCH datagrams (see the module doc)"""
        return _BatchCtx(self)

    def _event_key(self, ref):
        """(seq, index) of an EVENT to wait for: a plain seq, or a SubRef (its batch is sent first if still open)"""
        if isinstance(ref, SubRef):
            if ref.dgram.seq is None:
                if ref.dgram is not self._batch:
                    raise ProtoError("wait on a sub-message of a batch that was never sent")
                self._flush_batch()
            return (ref.dgram.seq, ref.idx)
        return (ref, 0)

    def _send_seq(self, typ, body=b"", timeout=10.0):
        need = 32 + ((len(body) + 31) // 32) * 32
        t0 = time.monotonic()
        with self.cv:
            while len(self.unacked) >= self.window or (self.unacked and self._inflight() + need > self.credit):
                if not self.cv.wait(0.05) and time.monotonic() - t0 > timeout:
                    raise ProtoError("send window stuck (expected %s, %d unacked)" % (self.ack_exp, len(self.unacked)))
            seq = self.next_seq
            self.next_seq = (self.next_seq + 1) & 0xFFFFFFFF or 1
            pkt = self._msg(typ, seq, body)
            if not self.unacked:
                self.last_progress = time.monotonic()
            self.unacked[seq] = pkt
        self.sock.sendto(pkt, self.addr)
        return seq

    def _inflight(self):
        return sum(32 + ((len(p) - 8 + 31) // 32) * 32 for p in self.unacked.values())

    def wait_acked(self, timeout=10.0):
        self._flush_batch(timeout)
        t0 = time.monotonic()
        with self.cv:
            while self.unacked:
                if time.monotonic() - t0 > timeout:
                    raise ProtoError("not acknowledged: %d datagrams, board expects %s" % (len(self.unacked), self.ack_exp))
                self.cv.wait(0.05)

    def wait_event(self, ref, timeout=10.0):
        """the EVENT of a sequenced command: ref = its seq, or the SubRef of a command sent inside a batch"""
        key = self._event_key(ref)
        t0 = time.monotonic()
        with self.cv:
            while key not in self.events:
                if time.monotonic() - t0 > timeout:
                    raise ProtoError("no EVENT for seq %d sub-message %d" % key if key[1] else "no EVENT for seq %d" % key[0])
                self.cv.wait(0.05)
            ev = self.events.pop(key)
        if ev[0] == EV_ERROR:
            raise ProtoError("seq %d%s: %s (type %02X, detail %08X)" % (key[0], " sub-message %d" % key[1] if key[1] else "",
                                                                     ERR_NAMES.get(ev[1], ev[1]), ev[2], ev[3]))
        return ev

    def take_errors(self):
        with self.cv:
            e, self.errors = self.errors, []
        return e

    # ------------------------------------------------------------------ session / status / registers
    def hello(self, new_session=True, timeout=2.0):
        with self.cv:
            self.unacked.clear()
            self.next_seq = 1
            self.events.clear()
            n0 = self.ack_count
        for _ in range(int(timeout / 0.2)):
            self.send_unseq(T_HELLO, struct.pack("<I", 1 if new_session else 0))
            with self.cv:
                self.cv.wait_for(lambda: self.ack_count > n0, 0.2)
                if self.ack_count > n0:
                    return self.ack_exp
        raise ProtoError("no ACK to HELLO from %s:%d" % self.addr)

    def status(self, timeout=1.0):
        with self.cv:
            n0 = self.status_count
        for _ in range(3):
            self.send_unseq(T_STATUS_REQ)
            with self.cv:
                if self.cv.wait_for(lambda: self.status_count > n0, timeout / 3):
                    return dict(zip(STATUS_NAMES, self.status_words))
        raise ProtoError("no STATUS from %s:%d" % self.addr)

    def _local(self, typ, body, timeout=0.5, tries=3):
        for _ in range(tries):
            with self.cv:
                self.lreplies.clear()
            self.send_unseq(typ, body)
            with self.cv:
                if self.cv.wait_for(lambda: len(self.lreplies) > 0, timeout):
                    return self.lreplies.popleft()
        raise ProtoError("no reply to local message %02X" % typ)

    def reg_read(self, addr):
        return self._local(T_REG, struct.pack("<BBHI", 0, 0, addr, 0))[2]

    def reg_write(self, addr, value):
        return self._local(T_REG, struct.pack("<BBHI", 1, 0, addr, value & 0xFFFFFFFF))[2]

    def phy_read(self, reg, page=0):
        return self._local(T_PHY_REG, struct.pack("<BBHHH", 0, reg, page, 0, 0), timeout=0.5)[2] & 0xFFFF

    def phy_write(self, reg, value, page=0):
        return self._local(T_PHY_REG, struct.pack("<BBHHH", 1, reg, page, value, 0), timeout=0.5)[2] & 0xFFFF

    # ------------------------------------------------------------------ resources and transfers
    def create_buffer(self, handle, base, size, usage=0):
        return self.send_seq(T_CREATE_BUFFER, struct.pack("<HHII", handle, usage, base, size))

    def destroy(self, handle):
        return self.send_seq(T_DESTROY, struct.pack("<HH", handle, 0))

    def write_buffer(self, handle, offset, data):
        mv = memoryview(data)
        for i in range(0, len(data), MAX_DATA):
            self.send_seq(T_WRITE_BUFFER, struct.pack("<HHI", handle, 0, offset + i) + bytes(mv[i:i + MAX_DATA]))

    def mem_write(self, addr, data, board=0):
        mv = memoryview(data)
        for i in range(0, len(data), MAX_DATA):
            self.send_seq(T_MEM_WRITE, struct.pack("<BBHI", board, 0, 0, addr + i) + bytes(mv[i:i + MAX_DATA]))

    _tag = 0x1000

    def _read(self, make, total, rel=True, base=0, max_req=256 * 1024, inflight=2, tail=0.02, quiet=1.0, timeout=10.0):
        """Read `total` bytes as READ_DATA chunks. make(start, n, tag) -> (type, body) builds the request
        for bytes [start, start + n) of the stream; rel: chunk offsets count from the request's start (MEM_READ,
        READ_BUFFER), else they are absolute and `base` is the offset of byte 0 of this read (READ_IMAGE: offsets
        count from pixel 0 of the rectangle, base = first pixel x bytes per pixel). Requests of max_req bytes (a multiple of RD_CHUNK),
        up to `inflight` outstanding. The board answers in order, so a chunk that is still missing when a later one
        has arrived (in its own request or a later one) is lost, as is a producing request's tail after `tail` s
        without data: lost chunks are re-requested at once, as exact sub-ranges. A request that has produced nothing
        for `quiet` s is asked for again: all its answers may be lost, which nothing later would show up, or it may
        sit behind other commands (a held PRESENT, a long copy), and then the second answer only counts as "dup".
        `timeout` s with no data at all end the read."""
        CH = RD_CHUNK
        max_req = max(CH, max_req // CH * CH)
        out = bytearray(total)
        missing = set(range(0, total, CH))                  # chunk positions not received yet
        todo = collections.deque((s, min(max_req, total - s)) for s in range(0, total, max_req))
        reqs = collections.OrderedDict()                    # outstanding: tag -> [start, n, t_sent, t_last, top]
        starts = {}                                         # every tag of this read -> its start (late chunks)
        nch = seen = len(missing)
        t_data = time.monotonic()
        try:
            while missing:
                while todo and len(reqs) < inflight:
                    s, n = todo.popleft()
                    Board._tag = (Board._tag + 1) & 0xFFFFFFFF or 1
                    tag = Board._tag
                    with self.cv:
                        self.reads[tag] = {}
                    starts[tag] = s
                    typ, body = make(s, n, tag)
                    self.send_seq(typ, body)
                    self._flush_batch()                     # a read inside a batch: the request must be on its way
                    reqs[tag] = [s, n, time.monotonic(), None, -1]
                with self.cv:
                    self.cv.wait(0.002)
                    arrived = []
                    for tag in starts:
                        d = self.reads.get(tag)
                        if d:
                            arrived.extend((tag, off, data) for off, data in d.items())
                            d.clear()
                now = time.monotonic()
                for tag, off, data in arrived:
                    p = starts[tag] + off if rel else off - base
                    if 0 <= p < total and p % CH == 0:
                        if p not in missing:
                            self.read_loss["dup"] += 1              # arrived twice: late original or re-request
                        out[p:p + len(data)] = data[:total - p]
                        missing.discard(p)
                    r = reqs.get(tag)
                    if r is not None:
                        r[3] = now
                        r[4] = max(r[4], p)
                if len(missing) != seen:
                    seen, t_data = len(missing), now
                # loss detection, oldest request first
                lost, later_has_data = [], False
                for tag in reversed(list(reqs)):                # newest first: know whether a later one produced
                    r = reqs[tag]
                    s, n, t_sent, t_last, top = r
                    mine = [p for p in range(s, s + n, CH) if p in missing]
                    if not mine:
                        del reqs[tag]
                    elif later_has_data or (t_last is not None and now - t_last > tail) or (t_last is None and now - t_sent > quiet):
                        lost.extend(mine)
                        self.read_loss["later" if later_has_data else "tail"] += len(mine)
                        del reqs[tag]
                    else:
                        below = [p for p in mine if p < top]
                        if below:
                            lost.extend(below)
                            self.read_loss["below"] += len(below)
                            r[0] = max(below) + CH                  # the request now only expects what follows
                            r[1] = s + n - r[0]
                    if t_last is not None:
                        later_has_data = True
                if now - t_data > timeout:
                    raise ProtoError("read: no data for %.0f s (%d of %d chunks still to come)" % (timeout, seen, nch))
                if lost:
                    self.n_rerequested += len(lost)
                    lost.sort()
                    runs, a, b = [], lost[0], lost[0]
                    for p in lost[1:]:
                        if p == b + CH and p + CH - a <= max_req:
                            b = p
                        else:
                            runs.append((a, b)); a = b = p
                    runs.append((a, b))
                    for a, b in reversed(runs):                 # re-requests go first
                        todo.appendleft((a, min(b + CH, total) - a))
        finally:
            with self.cv:
                for tag in starts:
                    self.reads.pop(tag, None)
        return bytes(out)

    def read_buffer(self, handle, offset, length, **kw):
        return self._read(lambda s, n, tag: (T_READ_BUFFER, struct.pack("<HHIII", handle, 0, offset + s, n, tag)),
                          length, **kw)

    def mem_read(self, addr, length, board=0, **kw):
        return self._read(lambda s, n, tag: (T_MEM_READ, struct.pack("<BBHIII", board, 0, 0, addr + s, n, tag)),
                          length, **kw)

    def memtest(self, addr, length, seed=0x5EED1234, flags=3, board=0, timeout=30.0):
        seq = self.send_seq(T_MEMTEST, struct.pack("<BBHIIII", board, 0, 0, addr, length, seed, flags))
        code, errors, first_bad, tested = self.wait_event(seq, timeout)
        return errors, first_bad

    # ------------------------------------------------------------------ display (board B, over the link)
    def set_mode(self, mode, timeout=5.0):
        """Switch board B's video mode (0..4, MODE_NAMES); returns when B reports the change (EV_MODE)."""
        seq = self.send_seq(T_SET_MODE, struct.pack("<I", mode))
        code, a0, _, _ = self.wait_event(seq, timeout)
        if code != EV_MODE or a0 != mode:
            raise ProtoError("set_mode %d: unexpected event %r" % (mode, (code, a0)))
        return mode

    def present(self, handle, frame_id=0, at_once=False, hold=False, wait=True, timeout=2.0):
        """Show a board-B image (handle, see image_handle). With wait, returns (frame_id, vsync count, dropped)
        from EV_PRESENT once the image is on screen (dropped = 1: a later present replaced it first)."""
        flags = (PRES_AT_ONCE if at_once else 0) | (PRES_HOLD if hold else 0)
        seq = self.send_seq(T_PRESENT, struct.pack("<HHI", handle, flags, frame_id & 0xFFFFFFFF))
        if not wait:
            return seq
        code, fid, vsyncs, dropped = self.wait_event(seq, timeout)
        if code != EV_PRESENT:
            raise ProtoError("present %04X: unexpected event %r" % (handle, (code, fid)))
        return fid, vsyncs, dropped

    # ------------------------------------------------------------------ board A's geometry
    def frame_begin(self):
        """clear the bins, triangle ids -> 0 (before a frame's draws)"""
        return self.send_seq(T_FRAME_BEGIN, struct.pack("<I", 0))

    def vs_program(self, handle, offset, n_instr):
        """load the vertex program: n_instr 64-bit instructions from byte offset of an A buffer (32-byte aligned)"""
        return self.send_seq(T_VS_PROGRAM, struct.pack("<HHII", handle, 0, offset, n_instr))

    def vs_uniforms(self, first, vec4s):
        """vec4s: list of 4-tuples of FP24 bit patterns (or floats: converted with fp24) into uniforms first.."""
        body = struct.pack("<HH", first, len(vec4s))
        for v in vec4s:
            body += struct.pack("<IIII", *[c if isinstance(c, int) else fp24(c) for c in v])
        return self.send_seq(T_VS_UNIFORMS, body)

    def vertex_attr(self, index, enabled, size, handle, stride, offset, const=(0, 0, 0, 1.0), type=6, normalized=False):
        """attribute descriptor (glVertexAttribPointer): enabled -> `size` components of client format `type` (AT_BYTE ..
        AT_FLOAT as glVertexAttribPointer's types, AT_FP24 = the GPU's own 24-bit format in the low bits of a 32-bit word)
        from the A buffer at byte offset `offset` + index x `stride` bytes, converted to FP24 by the hardware (normalized:
        ES 2.0 §2.1.2); disabled -> the constant vec4 (ES 2.0 §2.7: (0, 0, 0, 1) until set)"""
        # word 0 = {normalized[24], type[22:20], size[18:16], enabled[8], index[3:0]} (fb_dispatch_a)
        body = struct.pack("<IHHII", (index & 0xF) | ((1 if enabled else 0) << 8) | ((size & 7) << 16) | ((type & 7) << 20) | ((1 if normalized else 0) << 24),
                           handle, 0, stride, offset)
        body += struct.pack("<IIII", *[c if isinstance(c, int) else fp24(c) for c in const])
        return self.send_seq(T_VERTEX_ATTR, body)

    def vs_state(self, viewport, cull, front_cw, fsmask, width, height, draw_id, line_width=1, depth_range=(0.0, 1.0), guard=None, rect=None, z01=False):
        """viewport = (sxs, sxo, sys, syo) with window = ndc * s + o in 1/16 PIXEL (FP24 bits or floats); cull 0 none,
        1 back, 2 front, 3 front and back; fsmask = the fragment shader's read varyings (every varying the fragment
        program reads MUST be in it: an omitted one is undefined); the render target's size in pixels (1..2048 each,
        B's FLUSH limits the width to 1280); line_width =
        glLineWidth rounded to the nearest integer (ES 2.0 §3.4: the host rounds; 1 .. 64, word 4 bits [15:9]);
        depth_range = glDepthRangef's (n, f) as floats (window z = ndc_z * (f - n) / 2 + (n + f) / 2; words 7, 8 carry
        the two FP24 constants) — or the two constants themselves as FP24 bit patterns (ints). Words 9..12
        are the GUARD BAND's NDC bounds (gxlo, gxhi, gylo, gyhi) = (+-GUARD_BAND - o) / s per axis, ordered:
        A clips triangles and lines against them (exactly) instead of saturating the 16-bit window coordinates; guard =
        the four as FP24 bit patterns to send them as given (a model's exact values), else computed here.
        Words 13, 14 are the draw's RECTANGLE {y0, x0}, {y1, x1} in pixels: rect = (x0, y0, x1, y1), the
        draw's scissor box (the SAME box as its DRAW_STATE's, and only with the scissor test on there), default the whole
        target. Board A drops what has no pixel in it and walks inside it; a board A without it takes the command's
        first 13 words and drops the two (a sub-body's or an entry's unread tail), so both get the same bytes.
        z01 (word 4 bit 24, on a board A that has the flag): the program's z runs 0 .. w, not -w .. w — A clips at z >= 0 and window z =
        n + ndc_z (f - n). For a host that keeps depth reversed (far at 0, where FP24 is fine); a board A without the flag ignores the bit"""
        lw = max(1, min(64, int(round(line_width))))
        n, f = depth_range
        zs, zo = (n, f) if isinstance(n, int) else (fp24(f - n), fp24(n)) if z01 else (fp24((f - n) / 2), fp24((n + f) / 2))
        if guard is None:
            vpf = [c if not isinstance(c, int) else fp24_to_float(c) for c in viewport]
            gb = []
            for s, o in ((vpf[0], vpf[1]), (vpf[2], vpf[3])):
                gb += sorted(((-GUARD_BAND - o) / s, (GUARD_BAND - o) / s))
            guard = [fp24(c) for c in gb]
        body = struct.pack("<IIII", *[c if isinstance(c, int) else fp24(c) for c in viewport])
        body += struct.pack("<HBBHHI", cull | ((1 if front_cw else 0) << 8) | (lw << 9), fsmask, 1 if z01 else 0, width, height, draw_id)
        body += struct.pack("<II", zs, zo)
        body += struct.pack("<IIII", *guard)
        x0, y0, x1, y1 = rect if rect is not None else (0, 0, width, height)
        body += struct.pack("<HHHH", x0, y0, x1, y1)
        return self.send_seq(T_VS_STATE, body)

    def draw(self, mode, count, first=0, index_handle=None, index_offset=0, index_u8=False):
        """mode 0 triangles, 1 strip, 2 fan, 3 points, 4 lines, 5 line strip, 6 line loop; indexed when index_handle
        is given (u16, or u8 with index_u8; index_offset = DrawElements' byte offset, any alignment); first / count 32 bits"""
        indexed = index_handle is not None
        body = struct.pack("<BBBBHHIII", mode, 1 if indexed else 0, 1 if index_u8 else 0, 0,
                           index_handle or 0, 0, index_offset, first, count)
        return self.send_seq(T_DRAW, body)

    # ------------------------------------------------------------------ board B's per-draw state and frames
    def draw_state(self, draw_id, h, uniforms=()):
        """DRAW_STATE for draw id `draw_id` (the id A's VS_STATE gives the draw's records): h = the 23 words h0..h22
        (h0 flags, h1 / h2 stencil, h3 ops, h4 blend, h5 / h6 blend constant, h7 / h8 scissor,
        h9 / h10 polygon offset, h11 {program length [27:16], B program buffer handle [15:0]}, h12 byte offset,
        h13 uniform count, h14 constant alpha, h15..h22 samplers), uniforms = 4-tuples (FP24 bits or floats)"""
        assert len(h) == 23, "DRAW_STATE takes h0..h22"
        body = struct.pack("<I", draw_id & 0x1FFF) + struct.pack("<23I", *[w & 0xFFFFFFFF for w in h])
        for v in uniforms:
            body += struct.pack("<IIII", *[c if isinstance(c, int) else fp24(c) for c in v])
        return self.send_seq(T_DRAW_STATE, body)

    def flush(self, handle, flags, clear_c16=(0, 0, 0, 0), clear_depth=0xFFFFFF, clear_stencil=0,
              scissor=(0, 0, 0, 0), ds_handle=0, stencil_mask=0xFF):
        """FLUSH: board B renders one frame of A's render stream into the colour image `handle`
        (RGBA8 / RGBA4444 / RGBA5551 / RGB565, width <= 1280). flags = FL_*; clear_c16 = (R, G, B, A) in the target's
        channel units (c16(f, bits)); clear_depth in the attachment's units (24 or 16 bits); scissor = (x0, y0, x1, y1)
        of the clear; ds_handle = the D24S8 depth-stencil image that keeps depth / stencil between FLUSHes (0 = none)"""
        R, G, B, A = clear_c16
        x0, y0, x1, y1 = scissor
        body = struct.pack("<7I", (handle & 0xFFFF) | ((flags & 0xFFFF) << 16),
                           (R & 0x1FFF) | ((G & 0x1FFF) << 13), (B & 0x1FFF) | ((A & 0x1FFF) << 13),
                           (clear_depth & 0xFFFFFF) | ((clear_stencil & 0xFF) << 24),
                           (x0 & 0x1FFF) | ((y0 & 0x1FFF) << 16), (x1 & 0x1FFF) | ((y1 & 0x1FFF) << 16),
                           (ds_handle & 0xFFFF) | ((stencil_mask & 0xFF) << 16))
        return self.send_seq(T_FLUSH, body)

    _fence_id = 0

    def fence(self, fid=None, timeout=10.0):
        if fid is None:
            Board._fence_id += 1
            fid = Board._fence_id
        seq = self.send_seq(T_FENCE, struct.pack("<I", fid))
        code, a0, _, _ = self.wait_event(seq, timeout)
        if code != EV_FENCE or a0 != fid:
            raise ProtoError("fence %d: unexpected event %r" % (fid, (code, a0)))
        return fid


    # ------------------------------------------------------------------ images on board B
    def create_image(self, handle, base, width, height, fmt=0, pitch=None, origin=0, levels=1, layers=1,
                     samples_log2=0, usage=USE_SAMPLED | USE_SCANOUT | USE_TRANSFER, vol=False):
        """CREATE_IMAGE: the image descriptor (fmt FMT_RGBA8 / FMT_RGB565; pitch default width * bpp). No event on
        success: errors arrive as EV_ERROR (check with fence() / take_errors()). vol: a 3D texture of depth = layers
        (width, height <= 256, the depth halves with the level, WRITE_IMAGE's layer is the slice)."""
        if pitch is None:
            pitch = width * STORAGE_BPP[fmt]
        flags = (origin & 1) | ((samples_log2 & 3) << 3)
        return self.send_seq(T_CREATE_IMAGE, struct.pack("<HHIHBBHHBBH", handle, usage, base, pitch, fmt, flags,
                                                          width, height, levels, layers, 1 if vol else 0))

    def write_image(self, handle, data, tfmt, x, y, w, h, level=0, layer=0, first_pixel=0, storage_fmt=FMT_RGBA8):
        """WRITE_IMAGE of `data` (pixels in transfer format tfmt, row-major from first_pixel of the rectangle), cut into
        datagrams of whole pixels. storage_fmt is only needed for RAW (bytes per pixel)."""
        bpp = transfer_bpp(tfmt, storage_fmt)
        per = (MAX_DATA // bpp)
        mv = memoryview(data)
        npix = len(data) // bpp
        for p in range(0, npix, per):
            n = min(per, npix - p)
            hdr = struct.pack("<HBBBBHHHHHII", handle, level, layer, tfmt, 0, 0, x, y, w, h, first_pixel + p, 0)
            self.send_seq(T_WRITE_IMAGE, hdr + bytes(mv[p * bpp:(p + n) * bpp]))
        return npix

    # ------------------------------------------------------------------ 2D: READ / FILL / COPY_IMAGE
    def read_image(self, handle, tfmt, x, y, w, h, level=0, layer=0, storage_fmt=FMT_RGBA8, first=0, count=0, **kw):
        """READ_IMAGE: pixels first .. first + count - 1 (count 0 = to the end) of the rectangle, packed in tfmt
        (RAW = the storage bytes; P8 images: RAW only). Lost READ_DATA chunks are re-requested as exact pixel
        ranges. Returns the bytes."""
        bpp = transfer_bpp(tfmt, storage_fmt)
        n = (w * h - first) if count == 0 else count

        def make(s, nbytes, tag):                    # s, nbytes: multiples of RD_CHUNK (a multiple of every bpp)
            return T_READ_IMAGE, struct.pack("<HBBBBHHHHHIII", handle, level, layer, tfmt, 0, 0, x, y, w, h,
                                             first + s // bpp, nbytes // bpp, tag)
        return self._read(make, n * bpp, rel=False, base=first * bpp, **kw)

    def fill_image(self, handle, tfmt, value, x, y, w, h, mask=0xF, level=0, layer=0):
        """FILL_IMAGE: value = one pixel in tfmt (its low bytes; RAW = the storage value), channel mask bits
        MASK_R / G / B / A. Exact glClear of RGB565: pass RAW with round(f * 31) / round(f * 63)."""
        return self.send_seq(T_FILL_IMAGE, struct.pack("<HBBBBHHHHHI", handle, level, layer, tfmt, mask & 0xF, 0,
                                                       x, y, w, h, value & 0xFFFFFFFF))

    def copy_image(self, src, dst, sx, sy, sw, sh, dx, dy, dw=None, dh=None, mirror_x=False, mirror_y=False,
                   key=None, slevel=0, slayer=0, dlevel=0, dlayer=0):
        """COPY_IMAGE (DirectDraw Blt): stretch when (dw, dh) != (sw, sh) (nearest, pixel centres), mirrors, source
        colour key = (low, high) stored values, per channel inclusive (alpha ignored)."""
        dw = sw if dw is None else dw
        dh = sh if dh is None else dh
        flags = (COPY_MIRROR_X if mirror_x else 0) | (COPY_MIRROR_Y if mirror_y else 0) | (COPY_KEY if key else 0)
        lo, hi = key if key else (0, 0)
        return self.send_seq(T_COPY_IMAGE, struct.pack("<HBBHBBHHHHHHHHIII", src, slevel, slayer, dst, dlevel, dlayer,
                                                       sx, sy, sw, sh, dx, dy, dw, dh, flags, lo, hi))

    # ------------------------------------------------------------------ palette and hardware cursor
    def set_palette(self, first, colors):
        """SET_PALETTE: colors = list of 0xRRGGBB for entries first .. first + len - 1; shown from the next frame on."""
        return self.send_seq(T_SET_PALETTE, struct.pack("<HH", first, len(colors)) +
                             b"".join(struct.pack("<I", c & 0xFFFFFF) for c in colors))

    def cursor_shape(self, words):
        """CURSOR SHAPE: 4096 words (64 x 64, row-major) into the hidden shape buffer; shown by cursor_show(take=True).
        Mode 0 / 2: 0xAARRGGBB (straight / premultiplied alpha), mode 1: bit 24 AND, bits 23..0 XOR colour."""
        for k in range(0, len(words), 340):
            part = words[k:k + 340]
            self.send_seq(T_CURSOR, struct.pack("<BBHHH", CUR_SHAPE, 0, 0, k, 0) +
                          b"".join(struct.pack("<I", v & 0xFFFFFFFF) for v in part))

    def cursor_show(self, mode=CUR_COLOR, hot=(0, 0), pos=None, take=True):
        flags = (1 if take else 0) | (2 if pos is not None else 0)
        x, y = pos if pos is not None else (0, 0)
        return self.send_seq(T_CURSOR, struct.pack("<BBBBBBHhh", CUR_SHOW, mode, flags, 0, hot[0], hot[1], 0, x, y))

    def cursor_hide(self):
        return self.send_seq(T_CURSOR, struct.pack("<BBH", CUR_HIDE, 0, 0))

    def cursor_move(self, x, y):
        """Sequenced move (in order with the other commands)."""
        return self.send_seq(T_CURSOR, struct.pack("<BBHhh", CUR_MOVE, 0, 0, x, y))

    def cursor_pos(self, x, y):
        """Fast path (unsequenced): board A sends it to B ahead of every queued command; latest wins, a lost one is
        healed by the next, so repeat the last position when the mouse stops."""
        self.send_unseq(T_CURSOR_POS, struct.pack("<hh", x, y))


def image_handle_b(index):
    """Handle of a host-created board-B image (same encoding as image_handle)."""
    return 0xC000 | (index & 0x3FF)


def transfer_bpp(tfmt, storage_fmt=FMT_RGBA8):
    if tfmt == TF_RAW:
        return STORAGE_BPP[storage_fmt]
    return {TF_RGB565: 2, TF_RGB888: 3, TF_BGR888: 3}.get(tfmt, 4)


def _r(v, bits):
    """8-bit channel -> bits, rounded to nearest: round(v * (2^bits - 1) / 255) (no ties exist)."""
    return (v * ((1 << bits) - 1) * 2 + 255) // 510


def _e(c, bits):
    """bits -> 8-bit channel, ES 2.0 exact: round(c * 255 / (2^bits - 1)) (no ties exist)."""
    m = (1 << bits) - 1
    return (c * 510 + m) // (2 * m)


def to_storage(tfmt, storage_fmt, pix):
    """Reference conversion of one pixel's transfer bytes into the bytes board B stores (img_write.sv):
    RGBA8 = little-endian 0xRRGGBBAA (bytes A, B, G, R), RGB565 = little-endian {R5, G6, B5}, P8 = the index.
    RGB565 transfer data is expanded exactly (round(c * 255 / 31))."""
    if tfmt == TF_RAW:
        return bytes(pix[:STORAGE_BPP[storage_fmt]])
    if tfmt == TF_RGBA8888:
        r, g, b, a = pix[0], pix[1], pix[2], pix[3]
    elif tfmt == TF_BGRA8888:
        b, g, r, a = pix[0], pix[1], pix[2], pix[3]
    elif tfmt == TF_BGRX8888:
        b, g, r, a = pix[0], pix[1], pix[2], 255
    elif tfmt == TF_RGB888:
        r, g, b, a = pix[0], pix[1], pix[2], 255
    elif tfmt == TF_BGR888:
        b, g, r, a = pix[0], pix[1], pix[2], 255
    else:
        v = pix[0] | (pix[1] << 8)
        r5, g6, b5 = v >> 11, (v >> 5) & 63, v & 31
        r, g, b, a = _e(r5, 5), _e(g6, 6), _e(b5, 5), 255
    if storage_fmt == FMT_RGB565:
        v = (_r(r, 5) << 11) | (_r(g, 6) << 5) | _r(b, 5)
        return bytes((v & 255, v >> 8))
    return bytes((a, b, g, r))


def from_storage(storage_fmt, tfmt, stored):
    """Reference of READ_IMAGE: one pixel's stored bytes -> transfer bytes (RGB565 expanded exactly, alpha 255;
    BGRX: X = 255)."""
    if tfmt == TF_RAW:
        return bytes(stored[:STORAGE_BPP[storage_fmt]])
    if storage_fmt == FMT_RGB565:
        v = stored[0] | (stored[1] << 8)
        r, g, b, a = _e(v >> 11, 5), _e((v >> 5) & 63, 6), _e(v & 31, 5), 255
    else:
        a, b, g, r = stored[0], stored[1], stored[2], stored[3]
    if tfmt == TF_RGBA8888:
        return bytes((r, g, b, a))
    if tfmt == TF_BGRA8888:
        return bytes((b, g, r, a))
    if tfmt == TF_BGRX8888:
        return bytes((b, g, r, 255))
    if tfmt == TF_RGB888:
        return bytes((r, g, b))
    if tfmt == TF_BGR888:
        return bytes((b, g, r))
    v = (_r(r, 5) << 11) | (_r(g, 6) << 5) | _r(b, 5)
    return bytes((v & 255, v >> 8))


def memtest_pattern(seed, addr):
    """Mirror of xfer_lin's pattern(a) for one 32-bit word at byte address addr."""
    a = addr & 0xFFFFFFFF
    return (seed ^ a ^ ((a << 11) & 0xFFFFFFFF) ^ (a >> 7)) & 0xFFFFFFFF
