// xfer_lin — linear transfer engine for one mem_arb read + write port pair. One operation at a time:
//   OP_WRITE   : write cmd_len bytes from the wd_* word stream to byte address cmd_addr. Any alignment:
//                the stream is realigned by addr[1:0], packed into 32-byte bursts at word slot addr[4:2],
//                and the unwritten bytes of the first/last burst are masked (app_wdf_mask = 1).
//   OP_READ    : read cmd_len bytes from cmd_addr and emit them as a little-endian word stream on rd_*_o
//                (the last word may carry fewer valid bytes).
//   OP_MEMTEST : cmd_addr/cmd_len 32-byte aligned. flags[0]: write the pattern, flags[1]: read back and
//                compare; reports mt_errors (mismatching 32-bit words) and the first bad byte address.
//                pattern(a) = seed ^ a ^ (a << 11) ^ (a >> 7) for every 32-bit word at byte address a.
// Byte order: stream byte k of a word is bits [8k+7:8k]; DDR byte address order matches app data lanes.
// `busy` falls only after every write command and data beat has been accepted by the arbiter (MIG
// strict ordering then makes the data visible to any later read).
// SHALLOW_RAM: the style of the shallow, wide FIFOs (u_bf, u_st) — block RAM, or LUTs in the one-board build
// (ONE_CHIP), where block RAM is the scarcer resource; the behaviour is the same
`ifndef SHALLOW_RAM
`ifdef ONE_CHIP
`define SHALLOW_RAM "distributed"
`else
`define SHALLOW_RAM "block"
`endif
`endif
`default_nettype none
module xfer_lin #(
    parameter int RD_AW = 5,                  // read staging FIFO depth 2**RD_AW bursts
    parameter bit MT_PIPE = 1'b0              // memtest verify: 1 = the word compares registered, the count and the first
                                              //   bad address a cycle later, mt_done a cycle later (both board cores, for
                                              //   timing); 0 = count in the compare cycle
)(
    input  wire        clk, rst,
    input  wire        cmd_valid,
    input  wire [1:0]  cmd_op,
    input  wire [29:0] cmd_addr,
    input  wire [29:0] cmd_len,
    input  wire [31:0] cmd_seed,
    input  wire [1:0]  cmd_flags,
    output wire        cmd_ready,
    output wire        busy,
    // write data stream
    input  wire        wd_valid,
    input  wire [31:0] wd_data,
    output wire        wd_ready,
    // read data stream
    output reg         rd_valid_o,
    output reg  [31:0] rd_data_o,
    input  wire        rd_ready_o,
    // memtest result
    output wire        mt_done,
    output reg  [31:0] mt_errors,
    output reg  [29:0] mt_first_bad,
    // mem_arb ports
    output wire        w_req, w_en,
    output wire [28:0] w_addr,
    output wire [255:0] w_data,
    output wire [31:0] w_mask,
    output wire        w_wren, w_wend,
    input  wire        w_rdy, w_wdf_rdy,
    output wire        r_en,
    output wire [28:0] r_addr,
    input  wire        r_rdy, r_valid,
    input  wire [255:0] rdata
);
    localparam [1:0] OP_WRITE = 2'd0, OP_READ = 2'd1, OP_MEMTEST = 2'd2;

    function automatic [31:0] pattern(input [31:0] seed, input [31:0] a);
        pattern = seed ^ a ^ (a << 11) ^ (a >> 7);
    endfunction

    typedef enum logic [2:0] {X_IDLE, X_WRITE, X_READ, X_MTW, X_MTV, X_DONE} xst_t;
    xst_t       xs;
    reg [29:0]  a_addr, a_len;
    reg [31:0]  a_seed;
    reg [1:0]   a_flags;
    reg [1:0]   s;                            // addr[1:0]
    reg [2:0]   slot0;                        // addr[4:2]
    reg [24:0]  b0;                           // first burst index (addr[29:5])
    reg [24:0]  nb;                           // number of bursts
    reg [28:0]  nin, naw, nout;               // input words, aligned words, output words

    assign cmd_ready = (xs == X_IDLE);
    assign busy      = (xs != X_IDLE);

    // ---------------- burst FIFO (write side) ----------------
    reg          bf_wen; reg [287:0] bf_wd;
    wire         bf_v; wire [287:0] bf_d; wire [5:0] bf_cnt; wire bf_pop;
    sfifo #(.W(288), .AW(4), .RAM_STYLE(`SHALLOW_RAM)) u_bf (.clk(clk), .rst(rst), .wen(bf_wen), .wdata(bf_wd),  // 192 LUT as LUTRAM; same cycles
        .full(), .count(bf_cnt), .ren(bf_pop), .rdata(bf_d), .rvalid(bf_v));
    wire bf_room = (bf_cnt < 6'd14);

    reg  [24:0] wr_dat, wr_cmd;
    wire        data_turn = (wr_dat == wr_cmd);
    assign w_wren = bf_v && data_turn;
    assign w_wend = w_wren;
    assign w_data = bf_d[255:0];
    assign w_mask = bf_d[287:256];
    assign bf_pop = w_wren && w_wdf_rdy;
    assign w_en   = !data_turn;
    assign w_addr = {1'b0, b0 + wr_cmd, 3'b000};
    assign w_req  = bf_v || !data_turn;

    // ---------------- read staging ----------------
    reg  [24:0] rd_cmd, rd_ret;
    wire        st_v; wire [255:0] st_d; wire [RD_AW+1:0] st_cnt; reg st_pop;
    wire [24:0] outstanding = rd_cmd - rd_ret;
    wire        rd_phase = (xs == X_READ) || (xs == X_MTV);
    // room in the staging for another command, registered with one burst of margin: the sum grows by at most one per
    // cycle (timing: the count adders -> r_en -> the arbiter's read-first grant -> b_rop's writer does not fit one cycle)
    reg         rd_room;
    always @(posedge clk) rd_room <= (st_cnt + outstanding) < ((1 << RD_AW) - 1);
    assign r_en   = rd_phase && (rd_cmd != nb) && rd_room;
    assign r_addr = {1'b0, b0 + rd_cmd, 3'b000};
    sfifo #(.W(256), .AW(RD_AW), .RAM_STYLE(`SHALLOW_RAM)) u_st (.clk(clk), .rst(rst), .wen(r_valid), .wdata(rdata), .full(),
        .count(st_cnt), .ren(st_pop), .rdata(st_d), .rvalid(st_v));

    // ---------------- write align + pack state ----------------
    reg  [28:0]  j;                           // aligned word index
    reg  [31:0]  prev;
    reg  [255:0] bur_d;
    reg  [31:0]  bur_be;
    reg          wpack_done;
    wire [2:0]   wslot   = slot0 + j[2:0];
    wire         need_in = (j < nin);
    wire [31:0]  cur     = need_in ? wd_data : 32'h0;
    wire [63:0]  cp      = {cur, prev};
    wire [31:0]  aligned = (s == 2'd0) ? cur : (cp >> (32 - 8*s));
    wire         wstep   = (xs == X_WRITE) && !wpack_done && bf_room && (!need_in || wd_valid);
    assign wd_ready = wstep && need_in;
    reg  [3:0]   be;
    integer l;
    always @(*) begin
        for (l = 0; l < 4; l = l + 1)
            be[l] = ({j, 2'b00} + l >= s) && ({j, 2'b00} + l < s + a_len);
    end

    // memtest write generator
    reg  [24:0]  mk;                          // memtest burst index (write gen / verify)
    reg  [255:0] pat;
    integer w;
    always @(*) begin
        for (w = 0; w < 8; w = w + 1) pat[w*32 +: 32] = pattern(a_seed, {2'b00, b0 + mk, 5'b00000} + 4*w);
    end

    // read extract state
    reg  [28:0]  a;                           // aligned word index (read)
    reg  [31:0]  rprev;
    wire [2:0]   rslot  = slot0 + a[2:0];
    wire [31:0]  aw     = st_d[rslot*32 +: 32];
    wire         can_out = !rd_valid_o || rd_ready_o;
    wire [63:0]  rcp    = {aw, rprev};
    wire [63:0]  fcp    = {32'h0, rprev};

    // memtest verify, the counting half (MT_PIPE = 1): mt_ne = the burst's mismatching words (registered with the
    // compares), counted into mt_errors a cycle later; the first bad address = the first mismatching word of the first
    // bad burst. mt_done_i = the done event registered (the whole mt_done of MT_PIPE = 0); with MT_PIPE it is delayed
    // once more so that a reader at done sees the final count.
    reg [7:0] mt_ne; reg [24:0] mt_nb; reg mt_cv; reg mt_done_i, mt_done_q;
    assign mt_done = MT_PIPE ? mt_done_q : mt_done_i;
    function automatic [3:0] popc8(input [7:0] b);
        popc8 = 4'(b[0]) + 4'(b[1]) + 4'(b[2]) + 4'(b[3]) + 4'(b[4]) + 4'(b[5]) + 4'(b[6]) + 4'(b[7]);
    endfunction
    function automatic [2:0] first8(input [7:0] b);
        first8 = 3'd0;
        for (int q = 7; q >= 0; q--) if (b[q]) first8 = 3'(q);
    endfunction
    always @(posedge clk) begin
        bf_wen <= 1'b0; st_pop <= 1'b0; mt_cv <= 1'b0;
        mt_done_i <= 1'b0; mt_done_q <= mt_done_i;   // done follows the last burst's count by the cycle the count takes
        if (rd_valid_o && rd_ready_o) rd_valid_o <= 1'b0;
        if (rst) begin
            xs <= X_IDLE; wr_dat <= 0; wr_cmd <= 0; rd_cmd <= 0; rd_ret <= 0; rd_valid_o <= 1'b0;
            mt_errors <= 0; mt_first_bad <= 0; mt_cv <= 1'b0; mt_done_i <= 1'b0; mt_done_q <= 1'b0;
        end else begin
            if (MT_PIPE && mt_cv && mt_ne != 8'd0) begin
                if (mt_errors == 0) mt_first_bad <= {mt_nb, 5'b00000} + 4 * first8(mt_ne);
                mt_errors <= mt_errors + 32'(popc8(mt_ne));
            end
            if (bf_pop)        wr_dat <= wr_dat + 1'b1;
            if (w_en && w_rdy) wr_cmd <= wr_cmd + 1'b1;
            if (r_en && r_rdy) rd_cmd <= rd_cmd + 1'b1;
            if (r_valid)       rd_ret <= rd_ret + 1'b1;
            case (xs)
                X_IDLE: if (cmd_valid) begin
                    a_addr <= cmd_addr; a_len <= cmd_len; a_seed <= cmd_seed; a_flags <= cmd_flags;
                    s <= cmd_addr[1:0]; slot0 <= cmd_addr[4:2]; b0 <= cmd_addr[29:5];
                    nb   <= (cmd_len == 0) ? 25'd0 : (((cmd_addr + cmd_len - 30'd1) >> 5) - (cmd_addr >> 5) + 25'd1);
                    nin  <= (cmd_len + 30'd3) >> 2;
                    naw  <= ({1'b0, cmd_addr[1:0]} + cmd_len + 30'd3) >> 2;
                    nout <= (cmd_len + 30'd3) >> 2;
                    wr_dat <= 0; wr_cmd <= 0; rd_cmd <= 0; rd_ret <= 0;
                    j <= 0; prev <= 0; bur_be <= 0; wpack_done <= (cmd_len == 0);
                    a <= 0; rprev <= 0; mk <= 0; mt_errors <= 0; mt_first_bad <= 0;
                    if (cmd_len == 0) begin
                        xs <= X_DONE;
                        if (cmd_op == OP_MEMTEST) mt_done_i <= 1'b1;
                    end else case (cmd_op)
                        OP_WRITE:   xs <= X_WRITE;
                        OP_READ:    xs <= X_READ;
                        default:    xs <= cmd_flags[0] ? X_MTW : X_MTV;
                    endcase
                end
                // ---------------- write ----------------
                X_WRITE: begin
                    if (wstep) begin
                        bur_d[wslot*32 +: 32] <= aligned;
                        if (wslot == 3'd7 || j == naw - 1) begin
                            bf_wen <= 1'b1;
                            bf_wd  <= {~(bur_be | ({28'h0, be} << (wslot*4))),
                                       (bur_d & ~(256'hFFFFFFFF << (wslot*32))) | ({224'h0, aligned} << (wslot*32))};
                            bur_be <= 0;
                        end else bur_be <= bur_be | ({28'h0, be} << (wslot*4));
                        prev <= cur;
                        j <= j + 1'b1;
                        if (j == naw - 1) wpack_done <= 1'b1;
                    end
                    if (wpack_done && !bf_v && !bf_wen && wr_cmd == nb && wr_dat == nb) xs <= X_DONE;
                end
                // ---------------- memtest write ----------------
                X_MTW: begin
                    if (mk != nb && bf_room) begin
                        bf_wen <= 1'b1; bf_wd <= {32'h0, pat}; mk <= mk + 1'b1;
                    end
                    if (mk == nb && !bf_v && !bf_wen && wr_cmd == nb && wr_dat == nb) begin
                        wr_dat <= 0; wr_cmd <= 0; mk <= 0;
                        if (a_flags[1]) xs <= X_MTV; else begin mt_done_i <= 1'b1; xs <= X_DONE; end
                    end
                end
                // ---------------- memtest verify ----------------
                X_MTV: begin
                    if (st_v && !st_pop) begin
                        st_pop <= 1'b1;
                        if (MT_PIPE) begin : cmp_p      // the 8 word compares now; the count and the first bad address next
                            integer q;                  // cycle (in one cycle it is a 21-level path)
                            for (q = 0; q < 8; q = q + 1) mt_ne[q] <= (st_d[q*32 +: 32] != pat[q*32 +: 32]);
                            mt_nb <= b0 + mk; mt_cv <= 1'b1;
                        end else begin : cmp            // compare, count and first bad address in one cycle
                            integer q; reg [31:0] nerr; reg found;
                            nerr = 0; found = 1'b0;
                            for (q = 0; q < 8; q = q + 1)
                                if (st_d[q*32 +: 32] != pat[q*32 +: 32]) begin
                                    nerr = nerr + 1;
                                    if (!found && mt_errors == 0) begin
                                        mt_first_bad <= {b0 + mk, 5'b00000} + 4*q; found = 1'b1;
                                    end
                                end
                            mt_errors <= mt_errors + nerr;
                        end
                        mk <= mk + 1'b1;
                        if (mk == nb - 1) begin mt_done_i <= 1'b1; xs <= X_DONE; end
                    end
                    if (nb == 0) begin mt_done_i <= 1'b1; xs <= X_DONE; end
                end
                // ---------------- read ----------------
                X_READ: begin
                    if (a < naw) begin
                        if (st_v && !st_pop && (((s == 2'd0) || (a != 0)) ? can_out : 1'b1)) begin
                            if (s == 2'd0) begin rd_valid_o <= 1'b1; rd_data_o <= aw; end
                            else if (a != 0) begin rd_valid_o <= 1'b1; rd_data_o <= rcp >> (8*s); end
                            rprev <= aw;
                            if (rslot == 3'd7 || a == naw - 1) st_pop <= 1'b1;
                            a <= a + 1'b1;
                        end
                    end else if (a == naw && s != 2'd0 && naw == nout) begin
                        if (can_out) begin rd_valid_o <= 1'b1; rd_data_o <= fcp >> (8*s); a <= a + 1'b1; end
                    end else if (!rd_valid_o || rd_ready_o) begin
                        if (rd_cmd == nb && rd_ret == nb) xs <= X_DONE;
                    end
                end
                X_DONE: if (!rd_valid_o) xs <= X_IDLE;
                default: xs <= X_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
