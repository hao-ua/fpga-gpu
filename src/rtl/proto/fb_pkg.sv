// fb_pkg — host protocol constants. Mirrored by host/fb/fbproto.py.
// Datagram = 8-byte header {magic u16 = 0x4246 ("FB"), version u8 = 1, type u8, seq u32} + body,
// all little-endian. In 32-bit words (byte k of a word = bits [8k+7:8k]):
//   word0 = {type, version, 16'h4246}, word1 = seq.
// Ingress-queue entry (board A DDR ring): one 32-byte header burst
//   w0 = {8'h51, type, body_len[15:0]}, w1 = seq, w2..w7 = 0
// followed by the body bytes packed little-endian into ceil(body_len/32) bursts.
`ifndef FB_PKG_SV
`define FB_PKG_SV
package fb_pkg;
    localparam [15:0] FB_MAGIC   = 16'h4246;
    localparam [7:0]  FB_VERSION = 8'd1;
    localparam [7:0]  Q_MARK     = 8'h51;

    // host -> board
    localparam [7:0] T_HELLO = 8'h01, T_STATUS_REQ = 8'h02, T_PHY_REG = 8'h03, T_MEM_WRITE = 8'h04,
                     T_MEM_READ = 8'h05, T_MEMTEST = 8'h06, T_REG = 8'h07,
                     T_CREATE_BUFFER = 8'h10, T_CREATE_IMAGE = 8'h11, T_DESTROY = 8'h12,
                     T_WRITE_BUFFER = 8'h20, T_READ_BUFFER = 8'h21, T_WRITE_IMAGE = 8'h22, T_READ_IMAGE = 8'h23,
                     T_FILL_IMAGE = 8'h24, T_COPY_IMAGE = 8'h25, T_GEN_MIPMAP = 8'h26, T_COPY_BUFFER = 8'h27,
                     T_SET_MODE = 8'h30, T_PRESENT = 8'h31, T_CURSOR = 8'h32, T_CURSOR_POS = 8'h33,
                     T_SET_PALETTE = 8'h34,
                     T_FENCE = 8'h40, T_FLUSH = 8'h41,
                     T_DRAW_STATE = 8'h50,                       // 3D on B
                     // 3D on A: the vertex program from an A buffer, uniforms, the 16 attribute
                     // descriptors, the per-draw state of the setup stage, a draw, the frame's bin clear
                     T_VS_PROGRAM = 8'h51, T_VS_UNIFORMS = 8'h52, T_VERTEX_ATTR = 8'h53, T_VS_STATE = 8'h54,
                     T_DRAW = 8'h55, T_FRAME_BEGIN = 8'h56,
                     // BATCH: body = sub-messages {u8 type, u8 flags (0), u16 body bytes,
                     // body padded to a word} under the datagram's one seq, executed in order by fb_dispatch_a as if each
                     // had arrived alone (B's go out as separate CMD packets); every EVENT carries the sub-message index in
                     // its code word bits [15:8] (0 outside a batch; the CMD header's byte 3 carries it to B, which echoes
                     // it); an error found on A ends the batch (the rest is dropped)
                     T_BATCH = 8'h60;
    // CURSOR sub-operations (body byte 0)
    localparam [7:0] CUR_SHAPE = 8'd1, CUR_SHOW = 8'd2, CUR_HIDE = 8'd3, CUR_MOVE = 8'd4;
    // board -> host
    localparam [7:0] R_ACK = 8'h81, R_STATUS = 8'h82, R_EVENT = 8'h83, R_READ_DATA = 8'h84,
                     R_PHY_REG = 8'h85, R_REG = 8'h86;
    // EVENT codes
    localparam [7:0] EV_FENCE = 8'h01, EV_PRESENT = 8'h02, EV_MODE = 8'h03, EV_MEMTEST = 8'h04, EV_ERROR = 8'h10;
    // EVENT error codes
    localparam [7:0] ERR_TYPE = 8'd1, ERR_HANDLE = 8'd2, ERR_RANGE = 8'd3, ERR_HEAP = 8'd4, ERR_NO_B = 8'd5,
                     ERR_LEN = 8'd6, ERR_BOARD = 8'd7, ERR_MODE = 8'd8,   // ERR_MODE: video MMCM did not lock
                     ERR_RENDER = 8'd9,                                   // a FLUSH's frame was aborted: B did not take the render stream in time
                     ERR_SHADER = 8'd10;                                  // a fragment program's stack violation killed quads of a FLUSH's frame (B)
    // ACK flags
    localparam int ACKF_NAK = 0, ACKF_FULL = 1;
    // board registers (T_REG)
    localparam [15:0] REG_IDELAY = 16'h0000, REG_PHYOPT = 16'h0001, REG_PHYAD = 16'h0002, REG_REINIT = 16'h0003,
                      REG_SCRATCH = 16'h0004, REG_LINKCTL = 16'h0005, REG_BUILD = 16'h0010;   // LINKCTL b0: link fill words
    // READ_DATA chunk size (bytes): 8 header + 8 (tag, offset) + 1440 = 1456 <= 1472 (1500-byte MTU)
    localparam int RD_CHUNK = 1440;
    // STATUS layout version and length (words after the 8-byte header); layout 2 adds board B and the
    // board link (words 37-47), layout 3 the link diagnostics, B's receiver and B's transfer counters
    // (words 48-63), layout 4 B's receive diagnostics, the XOFF / ARQ / word counters of both boards
    // (words 64-75, see board_a_core)
    localparam [31:0] STATUS_LAYOUT = 32'd11;         // layout 5: credit, cursor, palette, transfer counters (76-83);
    localparam int    STATUS_WORDS  = 116;            // layout 6: A's geometry stage (84-87);
                                                      // layout 7: B's per-frame cycle counters (88-95, its STATUS words
                                                      // 24-31) and A's cumulative ones (96-99);
                                                      // layout 8: quads A's coarse depth culled (100, cumulative);
                                                      // layout 9 (A's coarse depth removed): word 100 = the
                                                      // primitive-band visits of A's walk, cumulative;
                                                      // layout 10: the geometry side's profile, 13 cumulative
                                                      // counters from a_geom (101-113) + the dispatcher's idle and D_GWAIT
                                                      // cycles (114, 115) — board_a_core / a_geom list them;
                                                      // layout 11: word 75 = the IPv4 address in use, word 3
                                                      // (flags) bits 21-26 = DHCP state, address valid, leased;
                                                      // flags bits 30 / 31 (A's and B's): the core lacks the
                                                      // integer divider / SIN / COS (0 on these boards)

    // board link packets: header word {type, seq (stamped by the
    // link ARQ layer), len = payload words}
    localparam [7:0] L_IDLE = 8'h00, L_CMD = 8'h10, L_STATUS = 8'h11, L_EVENT = 8'h12, L_READ_DATA = 8'h13,
                     L_CURSOR = 8'h14, L_CREDIT = 8'h15, L_SYNC = 8'h16,
                     L_RSTREAM = 8'h17, L_RCREDIT = 8'h18;   // 3D render stream A -> B, its credit B -> A
    // RSTREAM payload: whole render items in 64-bit beats (the header beat's hi word is padding); B buffers RS_BUF_BEATS.
    // RCREDIT payload: w0 = {24'h0, restarted, sync id}, w1 = render beats b_render has taken (A's origin since the SYNC).
    // A SYNC with a third word {render beats sent before it} re-bases the render counters as w1 does the CMD ones.
    localparam int   RS_BUF_BEATS = 2048;
    // CMD payload: w0 = {sub-message index (BATCH, else 0), host message type, body bytes}, w1 = host seq, then the body words
    // CURSOR payload (A -> B, fast path): {y s16, x s16}; CREDIT payload (B -> A): w0 =
    // {24'h0, restarted (B's first CREDIT since its reset), sync id of the last SYNC B took (0 = none / flush)},
    // w1 = the cumulative number of CMD words B's command processor has taken from its CMD_BUF_WORDS-word command
    // buffer, counted in A's origin since that SYNC; SYNC payload (A -> B): w0 = {25'h0, sync id (0 = flush,
    // 1..127)}, w1 = A's count of CMD words sent before it. (Header bits [23:16] belong to the ARQ sequence number.)
    localparam int   CMD_BUF_WORDS = 4096;
    // STATUS payload (B -> A, every 1 ms): B_LSTAT_WORDS words, see board_b_core (12 + 8 receive / ARQ words +
    // 4 credit / cursor / palette / transfer-engine words + 8 render-engine cycle counters); A accepts 1..B_LSTAT_WORDS
    localparam int   B_LSTAT_WORDS = 32;
    // EVENT payload (B -> A): {host seq, {sub-message index, code}, a0, a1, a2}, forwarded to the host as an EVENT datagram

    function automatic [31:0] hdr_w0(input [7:0] t);
        hdr_w0 = {t, FB_VERSION, FB_MAGIC};
    endfunction
endpackage
`endif
