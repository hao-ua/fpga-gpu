// tb_b_render — board B's render engine end to end: b_render + mem_arb (R0 buckets / programs,
// R1 textures, R2 band load; W1 band flush) + the
// behavioural MIG (random ready, read latency).
// tools/b_ref.py frame <seed> <ntri> writes the render stream (br_stream.hex, one beat per line), the DDR contents
// (br_ddr.hex: buckets, programs, the render target's initial image), the pass parameters (br_params.hex) and the
// expected image (br_expect.hex: the frame rendered with the bit-exact models — quad setup, LDV, the SIMT shader at
// FP24, the b_rop rules). The stream is offered with random gaps; after `done` the target must equal the expected
// image, bit for bit.
`timescale 1ns/1ps
`default_nettype none
module tb_b_render;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    // render stream
    reg rs_valid = 0, rs_valid2 = 0; reg [63:0] rs_data = 0, rs_data2 = 0; wire rs_ready, rs_take2;
    // pass
    reg go = 0; reg [28:0] rt_base = 0; reg [10:0] rt_bpl = 0, rt_width = 0; reg [11:0] rt_height = 0; reg [51:0] cl_c16 = 0;
    reg cl_dither = 0, cl_en = 0, cl_sc = 0; reg [3:0] cl_cmask = 0; reg [12:0] cl_x0 = 0, cl_y0 = 0, cl_x1 = 0, cl_y1 = 0;
    reg [23:0] cl_depth = 0; reg [7:0] cl_sten = 0, cl_smask = 8'hff;
    reg [1:0] rt_fmt = 0; reg depth16 = 0, ds_en = 0, cl_den = 1, cl_sen = 1; reg [28:0] ds_base = 0; reg [10:0] ds_bpl = 0;
    wire done, busy, err_stack, frame_err;
    wire [31:0] c_quads, c_tris, c_ezrej, c_frames, c_errs; wire [255:0] perf;
    // memory ports
    wire r0_en, r1_en, r2_en, r0_rdy, r1_rdy, r2_rdy, r0_v, r1_v, r2_v; wire [28:0] r0_a, r1_a, r2_a; wire [255:0] rd_data;
    wire w1_req, w1_en, w1_wren, w1_wend, w1_rdy, w1_wdf;
    wire [28:0] w1_a; wire [255:0] w1_d; wire [31:0] w1_m;
`ifdef TB_NWARP16
    localparam int TB_FSW = 16;                // -Defines TB_NWARP12 / TB_NWARP16: a warp-count what-if
`elsif TB_NWARP12
    localparam int TB_FSW = 12;
`elsif TB_NWARP8
    localparam int TB_FSW = 8;                 // -Defines TB_NWARP8: the core with 8 warps (a latency-hiding what-if)
`else
    localparam int TB_FSW = 4;
`endif
    // the texture cache's variants: -Defines TB_TC256 (32 KB), TB_TC_HASH (XOR-hashed index),
    // TB_TC_NOPF; the default is the RTL's (512 sets = 64 KB, plain index, next-burst prefetch)
`ifdef TB_TC256
    defparam dut.u_tmu.TC_SETS = 256;
`endif
`ifdef TB_TC1024
    defparam dut.u_tmu.TC_SETS = 1024;                // 128 KB: +16 RAMB36 on the board
`endif
`ifdef TB_TC_HASH
    defparam dut.u_tmu.TC_HASH = 1'b1;
`endif
`ifdef TB_TC_NOPF
    defparam dut.u_tmu.TC_PF = 1'b0;
`endif
    // one lever at a time beside TB_BOARD: the cache's next-burst prefetch, the texture unit's
    // two-pixel front stage, 16 TEX entries in the core
`ifdef TB_PF1
    defparam dut.u_tmu.TC_PF = 1'b1;
`endif
`ifdef TB_F2D1
    defparam dut.u_tmu.F2_DUAL = 1'b1;
`endif
`ifdef TB_TEXQ16
    defparam dut.u_fs.TEXQ = 16;
`endif
    // MAXW 1280 = the board's (B3D_MAXW) and CELLS 4 096 (B3D_CELLS): the band buffer's depth is what lets a target up to
    // 1024 wide use 8-line bands (a bench MAXW of 256 overflows the buffer with a 256-wide frame at 8 lines)
    // -Defines TB_NL8: the band buffer doubled to 640 x 8 cells, 8-line bands at every width up to 1280 (a what-if;
    // generate the frame with BR_NLINES=8)
`ifdef TB_NL8
    localparam int TB_NL = 8;
`else
    localparam int TB_NL = 4;
`endif
    // -Defines TB_BOARD: the board's texture-unit options (board_b_core B3D_F2_DUAL = 0, B3D_TC_PF = 0);
    // the default is b_render's (F2 at two pixels per cycle, cache prefetch)
`ifdef TB_BOARD
    localparam bit TB_F2D = 1'b0, TB_TPF = 1'b0;
`else
    localparam bit TB_F2D = 1'b1, TB_TPF = 1'b1;
`endif
    // 16 fragment-core triangle slots = the board's; -Defines TB_NTS8: 8 slots
`ifdef TB_NTS8
    localparam int TB_NTS = 8;
`else
    localparam int TB_NTS = 16;
`endif
    // CELLS: the board's 4 096 (8-line bands up to 1024 wide); the TB_NL8 what-if needs 640 x 8 = 5 120
`ifdef TB_FSPRE
    localparam bit TB_PRE = 1'b1;              // -Defines TB_FSPRE: the program fetched ahead of the launch
`else
    localparam bit TB_PRE = 1'b0;              // the board's: fetched at the launch
`endif
`ifdef TB_NOSKIP
    localparam bit TB_SKIP = 1'b0;             // -Defines TB_NOSKIP: every pixel of every quad sampled
`else
    localparam bit TB_SKIP = 1'b1;             // the board's: uncovered pixels unsampled where the program allows
`endif
`ifdef TB_TEXW1
    localparam bit TB_W2 = 1'b0;               // -Defines TB_TEXW1: one texture-result writer
`else
    localparam bit TB_W2 = 1'b1;
`endif
`ifdef TB_STO4
    localparam bit TB_STO2 = 1'b0;             // -Defines TB_STO4: the colour store in 4 micro-ops
`else
    localparam bit TB_STO2 = 1'b1;             // the board's: 2 micro-ops
`endif
    // three what-ifs (off on the boards)
`ifdef TB_TEX2
    localparam bit TB_TEX2 = 1'b1;             // -Defines TB_TEX2: TEX coordinates one micro-op per pair
`else
    localparam bit TB_TEX2 = 1'b0;             // the board's: one coordinate per micro-op
`endif
`ifdef TB_LFAST
    localparam bit TB_LFAST = 1'b1;            // -Defines TB_LFAST: a full warp starts with its last row, 1 idle cycle between warps
`else
    localparam bit TB_LFAST = 1'b0;            // the board's: through L_GO, 3 idle cycles
`endif
`ifdef TB_BENOZ
    localparam bit TB_BENOZ = 1'b1;            // -Defines TB_BENOZ: no depth row for a warp that wrote no gl_FragDepth
`else
    localparam bit TB_BENOZ = 1'b0;            // the board's: the depth row of every quad is read
`endif
`ifdef TB_LDV2
    localparam int TB_LDV = 2;                 // -Defines TB_LDV2: one issued micro-op, a second multiplier and adder per lane
`elsif TB_LDV1
    localparam int TB_LDV = 1;                 // -Defines TB_LDV1: one issued micro-op, the second step by itself in the same lanes
`else
    localparam int TB_LDV = 0;                 // the board's (also -Defines TB_LDV0): two issued micro-ops per component
`endif
    b_render #(.NLINES(TB_NL), .MAXW(1280), .CELLS((TB_NL == 8) ? 5120 : 4096), .NTS(TB_NTS), .FS_NWARP(TB_FSW), .F2_DUAL(TB_F2D), .TC_PF(TB_TPF), .FS_PRE(TB_PRE),
               .TEX_SKIP(TB_SKIP), .TEX_W2(TB_W2), .FS_STO2(TB_STO2), .FS_TEX2(TB_TEX2), .FS_LDV(TB_LDV),
`ifdef TB_ADDDSP
               .FS_ADD_DSP(1'b1),
`endif
`ifdef TB_HWPROF
               .PROF(1'b1),                    // -Defines TB_HWPROF: the cumulative profile counters (+HWPROF prints them beside the bench's)
`endif
               .L_FAST(TB_LFAST), .BE_NOZ(TB_BENOZ)) dut (.clk, .rst,
        .rs_valid, .rs_valid2, .rs_data, .rs_data2, .rs_ready, .rs_take2, .go, .discard(1'b0), .abort(1'b0), .rt_base, .rt_bpl, .rt_width, .rt_height, .rt_fmt, .depth16, .ds_en, .ds_base, .ds_bpl,
        .clear_c16(cl_c16), .clear_dither(cl_dither),
        .clear_color_en(cl_en), .clear_sc_en(cl_sc), .clear_cmask(cl_cmask), .clear_x0(cl_x0), .clear_y0(cl_y0),
        .clear_x1(cl_x1), .clear_y1(cl_y1), .clear_depth(cl_depth), .clear_stencil(cl_sten),
        .clear_depth_en(cl_den), .clear_stencil_en(cl_sen), .clear_smask(cl_smask), .done, .busy,
        .r_en(r0_en), .r_addr(r0_a), .r_rdy(r0_rdy), .r_valid(r0_v), .rd_data,
        .t_en(r1_en), .t_addr(r1_a), .t_rdy(r1_rdy), .t_valid(r1_v),
        .l_en(r2_en), .l_addr(r2_a), .l_rdy(r2_rdy), .l_valid(r2_v),
        .fw_req(w1_req), .fw_en(w1_en), .fw_addr(w1_a), .fw_data(w1_d), .fw_mask(w1_m), .fw_wren(w1_wren),
        .fw_wend(w1_wend), .fw_rdy(w1_rdy), .fw_wdf_rdy(w1_wdf),
        .c_quads, .c_tris, .c_ezrej, .c_frames, .c_errs, .err_stack, .perf, .frame_err);
    wire app_en, app_rdy, app_wdf_rdy, app_rd_data_valid, app_wdf_wren, app_wdf_end; wire [2:0] app_cmd;
    wire [28:0] app_addr; wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask;
    mem_arb #(.NR(3), .NW(1), .RO_AW(5), .RO_RESERVE(4), .MUST_RUN(3'b010)) arb (.clk, .rst,
        .r_en({r2_en, r1_en, r0_en}), .r_addr({r2_a, r1_a, r0_a}), .r_rdy({r2_rdy, r1_rdy, r0_rdy}),
        .r_valid({r2_v, r1_v, r0_v}), .rd_data,
        .w_req(w1_req), .w_en(w1_en), .w_addr(w1_a), .w_data(w1_d),
        .w_mask(w1_m), .w_wren(w1_wren), .w_wend(w1_wend),
        .w_rdy(w1_rdy), .w_wdf_rdy(w1_wdf),
        .app_en, .app_cmd, .app_addr, .app_wdf_data, .app_wdf_mask, .app_wdf_wren, .app_wdf_end,
        .app_rdy, .app_wdf_rdy, .app_rd_data_valid, .app_rd_data);
    int mig_err; longint n_wr, n_rd;
`ifdef TB_FASTDDR
    localparam int TB_RDLAT = 2, TB_RDYPCT = 100;   // -Defines TB_FASTDDR: a near-ideal DDR (an upper bound for the texture miss path)
`else
    localparam int TB_RDLAT = 20, TB_RDYPCT = 80;
`endif
    mig_app_model #(.RD_LAT(TB_RDLAT), .RDY_PCT(TB_RDYPCT), .WDF_PCT(85), .SEED(3)) mig (.ui_clk(clk), .rst, .app_en, .app_cmd,
        .app_addr, .app_wdf_data, .app_wdf_mask, .app_wdf_wren, .app_wdf_end, .app_rdy, .app_wdf_rdy,
        .app_rd_data, .app_rd_data_valid, .errors(mig_err), .n_wr, .n_rd);

    // ---- +BESTALL: the back end stalls in random bursts (the ROP input held), so finished warps pile up behind the core's
    // output latch and slots are reused while the latch still reads them. The core asserts that no STO reaches the store
    // being read; the frame's bit-exact compare shows the metadata ring and the latched kill / depth flags kept up (and
    // that the draw-state bank is not reloaded under fragments waiting at the ROP, b_render's bank_free). +STALLSEED_<n>
    // another pattern, +STALLMAX_<n> the longest hold (default 120)
    int n_reuse = 0, n_stostall = 0;
    initial if ($test$plusargs("BESTALL")) begin
        int sd = 1, hmax = 120;
        void'($value$plusargs("STALLSEED_%d", sd)); void'($value$plusargs("STALLMAX_%d", hmax));
        forever begin
            repeat (1 + $unsigned($random(sd)) % 60) @(negedge clk);
            force dut.rop_hold = 1'b1;
            repeat (1 + $unsigned($random(sd)) % hmax) @(negedge clk);
            release dut.rop_hold;
        end
    end
    always @(posedge clk) if (dut.u_fs.launch && dut.u_fs.ol_v && dut.u_fs.ol_w == dut.u_fs.ap) n_reuse++;
    generate for (genvar g = 0; g < TB_FSW; g++) begin : CV
        always @(posedge clk) if (dut.u_fs.WRP[g].hv && (dut.u_fs.WRP[g].cls[4] || dut.u_fs.WRP[g].cls[21]) && dut.u_fs.ol_own[g]) n_stostall++;
    end endgenerate

    // ---- render stream source: the head beat and the one after it (the engine may take both); the second
    //      beat is withheld now and then so the single-beat paths run too ----
    reg [63:0] beats [$];
    always @(posedge clk) begin
        if (rs_valid && (rs_ready || rs_take2)) begin void'(beats.pop_front()); if (rs_take2 && rs_valid2) void'(beats.pop_front()); end
    end
    // +NOGAP: no random gaps — both beats whenever the stream has them (a full render buffer on the board), so
    // the frame's cycles are the engine's own
    bit nogap; initial nogap = $test$plusargs("NOGAP");
    bit sk_chk, sk_all; int sk_n = 0, sk_seen = 0, sk_err = 0; reg [31:0] sk_tab [0:63];   // +SKIPCHK / +SKIPALL (below, beside +TMUSTAT)
    always @(negedge clk) begin
        if (beats.size() != 0 && (nogap || $urandom % 8 != 0)) begin
            rs_valid <= 1'b1; rs_data <= beats[0];
            if (beats.size() > 1 && (nogap || $urandom % 5 != 0)) begin rs_valid2 <= 1'b1; rs_data2 <= beats[1]; end else rs_valid2 <= 1'b0;
        end else begin rs_valid <= 1'b0; rs_valid2 <= 1'b0; end
    end
    // +MSPROF: the sequencer's cycles in each state while the frame
    // runs, and why it waits — M_ITEM without a beat / with the early queue full, M_TRI without its two beats, M_TRIQS for
    // the quad-setup slot, the depth lane or the quad pipe (checked in that order)
    int msp [string]; int msp_it_empty = 0, msp_it_blk = 0, msp_tri_wait = 0, msp_tq_qs = 0, msp_tq_zl = 0, msp_tq_qp = 0;
    bit msprof; initial msprof = $test$plusargs("MSPROF");
    always @(posedge clk) if (msprof && dut.busy) begin
        string k; k = dut.ms.name();
        if (msp.exists(k)) msp[k] = msp[k] + 1; else msp[k] = 1;
        if (dut.ms == dut.M_ITEM) begin if (!dut.sb_valid) msp_it_empty++; else if (dut.quad_blk) msp_it_blk++; end
        if (dut.ms == dut.M_TRI && !(dut.sb_valid && dut.sb2_valid)) msp_tri_wait++;
        if (dut.ms == dut.M_TRIQS) begin
            if (dut.qs_cnt[dut.cur_qs + 2'd1] != 6'd0) msp_tq_qs++; else if (dut.zl_busy) msp_tq_zl++; else if (dut.qp_busy) msp_tq_qp++;
        end
    end
    final if (msprof) begin
        foreach (msp[k]) $display("MSPROF state %-9s %0d", k, msp[k]);
        $display("MSPROF waits: ITEM no beat %0d, ITEM early queue full %0d, TRI beats missing %0d, TRIQS quad-setup slot %0d, TRIQS depth lane %0d, TRIQS quad pipe %0d",
                 msp_it_empty, msp_it_blk, msp_tri_wait, msp_tq_qs, msp_tq_zl, msp_tq_qp);
    end
    // +WFILL: how full the fragment warps start (1..4 quads) and what started a
    // part-full one — the head quad is of another draw, the quad path ran empty while the sequencer waited (per sequencer
    // state: M_TSWAIT = a fragment-core triangle slot), or 32 cycles without a launchable quad; the cycles the pixel
    // issuer runs, and the quads whose pixel 3 is not covered (an issuer cycle a program without derivatives would not need)
    int wf_n [1:4]; int wf_draw = 0, wf_idle32 = 0, wf_pi = 0, wf_q = 0, wf_p3 = 0, wf_cov = 0; int wf_seq [string];
    bit wfill; initial begin wfill = $test$plusargs("WFILL"); for (int i = 1; i <= 4; i++) wf_n[i] = 0; end
    always @(posedge clk) if (wfill && dut.busy) begin
        if (dut.fs_start) wf_n[$countones(dut.fs_in_active) / 4]++;
        if (dut.pi_run) wf_pi++;
        if (dut.ls == dut.L_IDLE && !dut.rs_drop) begin
            if (dut.hd_pass && !dut.hd_bank) wf_draw++;
            else if (!(dut.hd_pass && dut.ctx_ok) && !dut.hd_ok && dut.lf_force) begin
                if (!(dut.qpath_empty && dut.seq_wait)) wf_idle32++;
                else begin string k; k = dut.ms.name(); if (wf_seq.exists(k)) wf_seq[k] = wf_seq[k] + 1; else wf_seq[k] = 1; end
            end
        end
        if (dut.ls == dut.L_ROW && dut.lrow == 2'd3) begin
            wf_q++; wf_cov += $countones(dut.qf_cov[dut.qh]); if (!dut.qf_cov[dut.qh][3]) wf_p3++;
        end
    end
    // +WFILL also: a warp's life in the core, averaged — launch to its first issue, launch to finished, finished to
    // retired (the output latch's turn), and how long its slot then stood empty before the next launch
    longint wl_n = 0, wl_first = 0, wl_fin = 0, wl_ret = 0, wl_idle = 0, wl_bub = 0, wl_bub8 = 0, wl_c = 0;
    longint wl_tl [0:15], wl_tf [0:15], wl_tr [0:15]; bit wl_got [0:15], wl_fd [0:15], wl_ever [0:15];
    initial for (int g = 0; g < 16; g++) begin wl_got[g] = 0; wl_fd[g] = 0; wl_ever[g] = 0; wl_tr[g] = 0; end
    always @(posedge clk) begin
      wl_c++;
      if (wfill && dut.busy) begin
        if (!dut.u_fs.issue && (|dut.u_fs.busy_v)) begin wl_bub++; if (&dut.u_fs.busy_v[TB_FSW-1:0]) wl_bub8++; end
        for (int g = 0; g < TB_FSW; g++) begin
            if (dut.u_fs.launch && dut.u_fs.ap == g) begin
                if (wl_ever[g]) wl_idle += wl_c - wl_tr[g];
                wl_tl[g] = wl_c; wl_got[g] = 0; wl_fd[g] = 0; wl_ever[g] = 1; wl_n++;
            end
            if (dut.u_fs.grant[g] && !wl_got[g]) begin wl_got[g] = 1; wl_first += wl_c - wl_tl[g]; end
            if (dut.u_fs.fin_v[g] && !wl_fd[g]) begin wl_fd[g] = 1; wl_tf[g] = wl_c; wl_fin += wl_c - wl_tl[g]; end
            if (dut.u_fs.retire && dut.u_fs.rp == g) begin wl_ret += wl_c - wl_tf[g]; wl_tr[g] = wl_c; end
        end
      end
    end
    // why no warp issues in a cycle with every slot busy: per warp, the first reason that applies
    longint wb_fin = 0, wb_noiv = 0, wb_fill = 0, wb_flush = 0, wb_end = 0, wb_sto = 0, wb_hz = 0, wb_hl = 0, wb_acc = 0, wb_wbc = 0, wb_other = 0;
    generate for (genvar g = 0; g < TB_FSW; g++) begin : WB
        always @(posedge clk) if (wfill && dut.busy && !dut.u_fs.issue && (&dut.u_fs.busy_v[TB_FSW-1:0])) begin
            if (dut.u_fs.WRP[g].fin) wb_fin++;
            else if (!dut.u_fs.WRP[g].iv) wb_noiv++;
            else if (dut.u_fs.WRP[g].fill_d) wb_fill++;
            else if (dut.u_fs.WRP[g].q_flush_d) wb_flush++;
            else if (dut.u_fs.WRP[g].is_end) wb_end++;
            else if ((dut.u_fs.WRP[g].cls[4] || dut.u_fs.WRP[g].cls[21]) && dut.u_fs.ol_own[g]) wb_sto++;
            else if (dut.u_fs.WRP[g].hl) wb_hl++;
            else if (|(dut.u_fs.WRP[g].hz)) wb_hz++;
            else if (dut.u_fs.WRP[g].stall) wb_acc++;
            else if (dut.u_fs.WRP[g].wbc) wb_wbc++;
            else wb_other++;
        end
    end endgenerate
    final if (wfill) begin
        $display("WFILL warps in a cycle without an issue, every slot busy: finished %0d, queue empty %0d, head loading %0d, flush %0d, at END (draining) %0d, STO waits for the latch %0d, LDV chain %0d, operand pending %0d, accumulator / other stall %0d, write slot taken %0d, other %0d",
                 wb_fin, wb_noiv, wb_fill, wb_flush, wb_end, wb_sto, wb_hl, wb_hz, wb_acc, wb_wbc, wb_other);
        if (wl_n != 0)
            $display("WFILL warp life (%0d warps): launch -> first issue %0.1f, launch -> finished %0.1f, finished -> retired %0.1f, slot empty before the next launch %0.1f cycles; cycles without an issue while a warp runs %0d (%0d with every slot busy)",
                     wl_n, real'(wl_first) / wl_n, real'(wl_fin) / wl_n, real'(wl_ret) / wl_n, real'(wl_idle) / wl_n, wl_bub, wl_bub8);
        $display("WFILL warps started with 1 / 2 / 3 / 4 quads: %0d / %0d / %0d / %0d (%0.2f quads per warp)", wf_n[1], wf_n[2], wf_n[3], wf_n[4],
                 real'(wf_n[1] + 2*wf_n[2] + 3*wf_n[3] + 4*wf_n[4]) / ((wf_n[1] + wf_n[2] + wf_n[3] + wf_n[4]) == 0 ? 1 : (wf_n[1] + wf_n[2] + wf_n[3] + wf_n[4])));
        $display("WFILL part-full starts: another draw %0d, 32 idle cycles %0d", wf_draw, wf_idle32);
        foreach (wf_seq[k]) $display("WFILL part-full start, quad path empty, sequencer in %-9s %0d", k, wf_seq[k]);
        $display("WFILL launched quads %0d: %0.2f covered pixels per quad, pixel 3 not covered in %0d; pixel issuer runs %0d cycles",
                 wf_q, real'(wf_cov) / (wf_q == 0 ? 1 : wf_q), wf_p3, wf_pi);
    end

    int err = 0; int W, H, bpp;
    initial begin
        int fd, r; reg [31:0] a; reg [255:0] d; reg [63:0] b; reg [31:0] pv [0:23]; reg [31:0] e; longint t0;
        // pv: rt base, bpl, width, height, clear colour (52 bits: read below), dither, en, sc, cmask, x0, y0, x1, y1, depth,
        //     stencil, beats, format, depth16, ds image, its base, its bpl, clear depth, clear stencil, stencil clear mask
        fd = $fopen("tools/gen/br_params.hex", "r");
        if (fd == 0) begin $display("TB_FAIL tb_b_render: no tools/gen/br_params.hex"); $finish; end
        r = $fscanf(fd, "%h %h %h %h %h", pv[0], pv[1], pv[2], pv[3], cl_c16);
        for (int k = 5; k < 24; k++) r = $fscanf(fd, "%h", pv[k]);
        $fclose(fd);
        rt_base = pv[0]; rt_bpl = pv[1]; rt_width = pv[2]; rt_height = pv[3]; W = pv[2]; H = pv[3];
        cl_dither = pv[5]; cl_en = pv[6]; cl_sc = pv[7]; cl_cmask = pv[8]; cl_x0 = pv[9]; cl_y0 = pv[10];
        cl_x1 = pv[11]; cl_y1 = pv[12]; cl_depth = pv[13]; cl_sten = pv[14];
        rt_fmt = pv[16]; depth16 = pv[17]; ds_en = pv[18]; ds_base = pv[19]; ds_bpl = pv[20]; cl_den = pv[21]; cl_sen = pv[22];
        cl_smask = pv[23]; bpp = (pv[16] == 0) ? 4 : 2;
        $display("tb_b_render: target format %0d, depth %0d bits, depth-stencil image %0d", rt_fmt, depth16 ? 16 : 24, ds_en);
        fd = $fopen("tools/gen/br_ddr.hex", "r");
        begin
            reg [63:0] d3, d2, d1, d0; int nr;
            forever begin
                nr = $fscanf(fd, "%h %h %h %h %h", a, d3, d2, d1, d0);
                if (nr != 5) break;
                mig.mem[longint'(a >> 3)] = {d3, d2, d1, d0};
            end
        end
        $fclose(fd);
        $display("tb_b_render: %0d DDR bursts preloaded (bucket of draw 1: %0d)", mig.mem.num(),
                 mig.mem.exists(longint'(32'h02000200 >> 3)));
        fd = $fopen("tools/gen/br_stream.hex", "r");
        while ($fscanf(fd, "%h", b) == 1) beats.push_back(b);
        $fclose(fd);
        $display("tb_b_render: %0d beats, target %0d x %0d", beats.size(), W, H);
        repeat (5) @(posedge clk); rst <= 0; repeat (5) @(posedge clk);
        t0 = $time;
        @(negedge clk); go = 1; @(negedge clk); go = 0;
        while (!done) @(posedge clk);
        $display("tb_b_render: frame done in %0d cycles: %0d quads launched, %0d early-depth drops after quad_setup, %0d in the early queue%s, %0d triangles, %0d errors",
                 ($time - t0) / 10, c_quads, c_ezrej, dut.c_ezrej1, dut.ez_audit ? " (+EZAUDIT: checked, not dropped)" : "", c_tris, c_errs);
        // the engine's own counters against the bench's frame time; the split for the record
        $display("tb_b_render: perf frame %0d cycles: flush %0d, band change %0d, draw change %0d, FS busy %0d, band set-up %0d, stream empty %0d, ROP stall %0d; frame_err %b",
                 perf[0 +: 32], perf[32 +: 32], perf[64 +: 32], perf[96 +: 32], perf[128 +: 32], perf[160 +: 32], perf[192 +: 32], perf[224 +: 32], frame_err);
        if (perf[0 +: 32] > ($time - t0) / 10 || perf[0 +: 32] + 8 < ($time - t0) / 10) begin
            err++; $display("  perf frame cycles %0d do not match the bench's %0d", perf[0 +: 32], ($time - t0) / 10);
        end
        for (int i = 1; i < 8; i++) if (perf[i*32 +: 32] > perf[0 +: 32]) begin err++; $display("  perf counter %0d exceeds the frame", i); end
        if (frame_err) begin err++; $display("  frame_err set (a stack violation)"); end
        if (c_errs != 0 || c_frames != 1) begin err++; $display("  c_errs %0d c_frames %0d", c_errs, c_frames); end
        $display("tb_b_render: band-buffer cross-port collisions (a peek of the cell written in the same cycle): bank 0 %0d, 1 %0d, 2 %0d",
                 dut.u_rop.BANK[0].INF.n_col, dut.u_rop.BANK[1].INF.n_col, dut.u_rop.BANK[2].INF.n_col);
        $display("tb_b_render: output latch covered: %0d launches into the slot being read, %0d cycles a STO waited for it", n_reuse, n_stostall);
        repeat (50) @(posedge clk);
        fd = $fopen("tools/gen/br_expect.hex", "r");
        for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) begin
            reg [255:0] bur; reg [31:0] got; longint wa;
            r = $fscanf(fd, "%h", e);
            wa = longint'(rt_base) + y * rt_bpl * 8 + (bpp == 4 ? x : x / 2);
            bur = mig.mem.exists(wa >> 3) ? mig.mem[wa >> 3] : 256'h0;
            got = (bpp == 4) ? bur[(wa & 7) * 32 +: 32] : {16'd0, bur[(wa & 7) * 32 + (x & 1) * 16 +: 16]};
            if (got !== e) begin
                err++;
                if (err < 12 || $test$plusargs("ALLDIFF")) $display("  pixel (%0d, %0d): %h, expected %h", x, y, got, e);
            end
        end
        $fclose(fd);
        if (ds_en) begin                                     // the depth-stencil image after the frame
            fd = $fopen("tools/gen/br_expect_ds.hex", "r");
            for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) begin
                reg [255:0] bur; reg [31:0] got; longint wa;
                r = $fscanf(fd, "%h", e);
                wa = longint'(ds_base) + y * ds_bpl * 8 + x;
                bur = mig.mem.exists(wa >> 3) ? mig.mem[wa >> 3] : 256'h0;
                got = bur[(wa & 7) * 32 +: 32];
                if (got !== e) begin
                    err++;
                    if (err < 12) $display("  depth-stencil (%0d, %0d): %h, expected %h", x, y, got, e);
                end
            end
            $fclose(fd);
        end
        if (mig_err != 0) begin err++; $display("  MIG model errors: %0d", mig_err); end
        if (beats.size() != 0) begin err++; $display("  %0d beats not consumed", beats.size()); end
        if (sk_chk) begin
            $display("tb_b_render: +SKIPCHK %0d program loads checked against br_skip.hex (%0d programs), %0d wrong verdicts", sk_seen, sk_n, sk_err);
            err += sk_err; if (sk_seen == 0) begin err++; $display("  +SKIPCHK: no program load seen"); end
        end
        $display("tb_b_render: errors = %0d", err);
        if (err == 0) $display("TB_PASS tb_b_render"); else $display("TB_FAIL tb_b_render");
        $finish;
    end
    // FRAGLOG: every fragment the back end gives b_rop and every early-depth drop -> br_rtl_frags.txt (compare with the
    // model's br_model_frags.txt: tools/b_ref.py fragcmp)
`ifdef FRAGLOG
    int flf;
    initial flf = $fopen("br_rtl_frags.txt", "w");
    always @(posedge clk) begin
        // (two pixels of a row per beat: one F line per valid pixel, x and x + 1)
        if (dut.br_valid && dut.br_ready) for (int p = 0; p < 2; p++) if (dut.br_pv[p])
            $fdisplay(flf, "F %0h %0h %0h %0h", dut.br_x + p, dut.br_y, dut.br_c16[p*52 +: 52], dut.br_z[p*24 +: 24]);
        if (dut.rs_drop) $fdisplay(flf, "D %0h %0h", dut.qf_qx[dut.qh], dut.qf_qy[dut.qh]);
        // the early-depth peeks (late: after quad_setup, exact; early: the early queue's bounds) as their depths land
        if (dut.pd3_v && !dut.pd3_e) $fdisplay(flf, "PL %0h %0h row %0d stored %h d24 %h cov %b func %0d hit %b slot %0d next %b rs_slot %0d",
            dut.qf_qx[dut.pd3_s], dut.qf_qy[dut.pd3_s], dut.pd3_p, dut.pk_z, dut.qf_d24[dut.pd3_s], dut.qf_cov[dut.pd3_s], dut.ds_pv.depth_func,
            dut.pk_hit, dut.qf_slot[dut.pd3_s], dut.u_rop.pk_next_q, dut.rs_slot);
        if (dut.pd3_v && dut.pd3_e) $fdisplay(flf, "PE %0h %0h row %0d stored %h bounds %h cov %b func %0d rej %b",
            dut.eq_qx[dut.pd3_s], dut.eq_qy[dut.pd3_s], dut.pd3_p, dut.pk_z, dut.pd_bd, dut.pd_cov, dut.pd_f, dut.pd_rej);
        if (dut.pk_en) $fdisplay(flf, "K x %0h y %0h next %b band_y0 %0h", dut.pk_x, dut.pk_y, dut.pk_next, dut.band_y0);
        if (dut.ls == dut.L_ROW && dut.lrow == 2'd3) $fdisplay(flf, "L %0h %0h %h", dut.qf_qx[dut.qh], dut.qf_qy[dut.qh], dut.qf_lam[dut.qh]);
        if (dut.fs_vw_we) $fdisplay(flf, "R%0d quad %0d %h", dut.fs_vw_row, dut.fs_vw_quad, dut.fs_vw_data);   // the launch rows as written
        if (dut.bs == dut.B_ROW) $fdisplay(flf, "O quad %0d row %0d %h", dut.bqi, dut.brow, dut.fs_out_row);     // the FS output rows
        if (dut.ls == dut.L_ROW && dut.lrow == 2'd3)
            for (int vi = 0; vi < 2; vi++) for (int c = 0; c < 4; c++)
                $fdisplay(flf, "T slot %0d v%0d.%0d {D2, D1, f0} %h", dut.qf_ts[dut.qh], vi, c, dut.u_fs.TS[0].tmem[{dut.qf_ts[dut.qh], 3'(vi), 2'(c)}]);
        if (dut.u_qs.o_valid) $fdisplay(flf, "S tag %0d z %h fw %h l1 %h l2 %h mu1 %h mu2 %h d24 %h fc %h %h %h", dut.u_qs.o_tag, dut.u_qs.o_z,
            dut.u_qs.o_fw, dut.u_qs.o_l1, dut.u_qs.o_l2, dut.u_qs.o_mu1, dut.u_qs.o_mu2, dut.u_qs.o_d24, dut.u_qs.o_fcx, dut.u_qs.o_fcy, dut.u_qs.o_fcz);
        if (dut.ls == dut.L_ROW && dut.lrow == 2'd3) $fdisplay(flf, "# u0 %h u1 %h u2 %h front %b ts %0d warp %0d quad %0d iram0 %h tri %h", dut.u_fs.params[0],
            dut.u_fs.params[1], dut.u_fs.params[2], dut.qf_front[dut.qh], dut.qf_ts[dut.qh], dut.fs_in_warp, dut.lq, dut.u_fs.iram[0],
            dut.u_fs.TS[0].tmem[{dut.qf_ts[dut.qh], 5'd0}]);
    end
`endif
    longint cyc = 0;
`ifdef PIXDBG
    // trace one pixel (+PX_x,PY_y defaults 15, 29): ROP accept / write, flush burst
    int px = 15, py = 29;
    always @(posedge clk) begin
        if (dut.br_valid && dut.br_ready && dut.br_x == px && dut.br_y == py)
            $display("  [%0d] ROP accept (%0d,%0d) c16 %h z %h band_y0 %0d st %0d rbk %0d", cyc, px, py, dut.br_c16, dut.br_z,
                     dut.band_y0, dut.u_rop.st, dut.u_rop.rbk);
        if (dut.u_rop.v6 && dut.u_rop.s6_addr == (py % 4) * W + px && dut.band_y0 == py / 4 * 4)
            $display("  [%0d] ROP write cell %0d word %h cw %b rbk %0d", cyc, dut.u_rop.s6_addr, dut.u_rop.pipe_wr_word,
                     dut.u_rop.s6_cw, dut.u_rop.rbk);
        if (w1_en && w1_rdy && w1_a == rt_base + py * W + (px / 8) * 8)
            $display("  [%0d] flush burst for (%0d,%0d): %h", cyc, px, py, w1_d[(px % 8) * 32 +: 32]);
        if (dut.band_start) $display("  [%0d] band start %0d (b_rop st %0d pf_ok %b%b%b pf_y0 %0d %0d %0d rbk %0d pbk %0d)", cyc,
            dut.band_y0, dut.u_rop.st, dut.u_rop.pf_ok[0], dut.u_rop.pf_ok[1], dut.u_rop.pf_ok[2], dut.u_rop.pf_y0[0],
            dut.u_rop.pf_y0[1], dut.u_rop.pf_y0[2], dut.u_rop.rbk, dut.u_rop.pbk);
    end
`endif
`ifdef PROF
    // where the cycles go (counted while the frame runs)
    longint p_run = 0, p_parse_wait = 0, p_q_full = 0, p_qf_full = 0, p_launch_wait_ctx = 0, p_ls_busy = 0, p_bq_busy = 0,
            p_br_stall = 0, p_fs_busy4 = 0, p_fs_idle = 0, p_rs_empty = 0, p_ms_item = 0;
    longint p_eq_full = 0, p_eq_lane = 0, p_eq_band = 0, p_eq_port = 0, p_eq_head = 0, p_eq_credit = 0, p_eq_drop = 0;
    final $display("PROF early queue: full (the sequencer waits) %0d, peek waits for the bounds %0d / for its band %0d / for the port %0d, head without verdict %0d, head waits for the quad path %0d, drops %0d (cycles)",
        p_eq_full, p_eq_lane, p_eq_band, p_eq_port, p_eq_head, p_eq_credit, p_eq_drop);
    always @(posedge clk) if (busy) begin
        p_run++;
        if (dut.ms == dut.M_ITEM && !dut.sb_valid) p_rs_empty++;
        if (dut.ms == dut.M_ITEM) p_ms_item++;
        if (dut.eq_cnt != 0) p_q_full++;
        if (dut.qf_cnt == 5'd16) p_qf_full++;
        // the early queue
        if (dut.ms == dut.M_ITEM && dut.quad_blk) p_eq_full++;
        if (dut.ep_ne && !dut.eq_lv[dut.ep]) p_eq_lane++;
        if (dut.ep_ne && dut.eq_lv[dut.ep] && dut.ez_elig && !dut.e_req && !dut.l_req) p_eq_band++;
        if (dut.ep_ne && dut.e_req && dut.l_req) p_eq_port++;
        if (dut.eq_cnt != 0 && !dut.eq_vd[dut.eq_h]) p_eq_head++;
        if (dut.eq_hv && !dut.eq_rej[dut.eq_h] && !dut.q_take) p_eq_credit++;
        if (dut.e_drop) p_eq_drop++;
        if (dut.lw_ctx) p_launch_wait_ctx++;
        if (dut.ls != dut.L_IDLE) p_ls_busy++;
        if (dut.bq_v) p_bq_busy++;
        if (dut.br_valid && !dut.br_ready) p_br_stall++;
        if (&dut.u_fs.busy_v) p_fs_busy4++;
        if (dut.fs_idle) p_fs_idle++;
    end
    final $display("PROF run %0d: render stream empty %0.1f%%, quad waiting for the issuer %0.1f%%, quad FIFO full %0.1f%%, launch busy %0.1f%%, launch waits for a context %0.1f%%, all 4 contexts busy %0.1f%%, FS idle %0.1f%%, back end busy %0.1f%%, ROP stalls %0.1f%%",
        p_run, 100.0 * p_rs_empty / p_run, 100.0 * p_q_full / p_run, 100.0 * p_qf_full / p_run, 100.0 * p_ls_busy / p_run,
        100.0 * p_launch_wait_ctx / p_run, 100.0 * p_fs_busy4 / p_run, 100.0 * p_fs_idle / p_run, 100.0 * p_bq_busy / p_run,
        100.0 * p_br_stall / p_run);
`endif
    // ---- +TMUSTAT: where the texture path's cycles go, counted while the frame runs:
    // the core's issue slots and live warps / TEXes, the TMU's job rate and the stage that holds it, the request round
    // trip (requests and results are in order), the cache's hits / misses
    bit ts_on; initial ts_on = $test$plusargs("TMUSTAT");
    longint ts_run = 0, ts_fsbusy = 0, ts_issue = 0, ts_job = 0, ts_fl = 0, ts_astall = 0, ts_rqfull = 0, ts_f4wait = 0,
            ts_f4v = 0, ts_tmubusy = 0, ts_wffull = 0, ts_f2 = 0, ts_jfne = 0, ts_f1blk = 0, ts_texlive = 0, ts_warps = 0,
            ts_req = 0, ts_res = 0, ts_lat = 0, ts_latmax = 0, ts_rq = 0;
    // the A stage: a burst is missing (a_need), allocated (a_alloc), or the allocation is blocked because the round-robin
    // entry is still referenced by a queued job / its fill has not landed / 16 lookups are queued; the job FIFO full; the
    // filter's head job waiting for its data; the cache refusing a lookup (8 in flight)
    longint ts_need = 0, ts_alloc = 0, ts_bref = 0, ts_bref_free = 0, ts_bfill = 0, ts_blq = 0, ts_jffull = 0, ts_fldata = 0, ts_cqfull = 0, ts_dist = 0;
    longint ts_t [$]; longint tp_min = 1 << 30;
    // +SKIPCHK: the program scan's verdict (b_render tsk) against b_ref.py's own (br_skip.hex: {may skip, the program's
    // word address} per draw). +SKIPALL: the mutant — every program may skip, the scan ignored: a frame holding
    // BR_SKIPTEST's refused programs must then FAIL its image compare (the frames can see an unsound skip).
    initial begin
        sk_chk = $test$plusargs("SKIPCHK"); sk_all = $test$plusargs("SKIPALL");
        if (sk_chk) begin : skl
            int fd; fd = $fopen("tools/gen/br_skip.hex", "r");
            if (fd == 0) begin $display("TB_FAIL tb_b_render: +SKIPCHK without tools/gen/br_skip.hex"); $finish; end
            while (sk_n < 64 && $fscanf(fd, "%h", sk_tab[sk_n]) == 1) sk_n++;
            $fclose(fd);
        end
        if (sk_all) force dut.tsk = 2'b11;
    end
    always @(posedge clk) if (sk_chk && !rst && dut.ms == dut.M_PROG && !dut.fe_go && dut.pg_n == dut.dsl.plen && dut.fe_done) begin : skc
        bit found, exp, got; found = 0; exp = 0; got = !(dut.tk_bad || dut.tk_badn);
        for (int i = 0; i < sk_n; i++) if (sk_tab[i][28:0] == dut.dsl.prog) begin found = 1; exp = sk_tab[i][31]; end
        sk_seen++;
        if (!found || got !== exp) begin
            sk_err++;
            $display("  SKIPCHK: program at %h: the scan says %0d, the model %0d%s", dut.dsl.prog, got, exp, found ? "" : " (not in br_skip.hex)");
        end
    end
    // the sample skip: requests with fewer than 4 pixels to sample and the pixels left out; cycles a quad's last job waited for the
    // result spacing (lp_c); programs scanned and how many may skip
    longint ts_mreq = 0, ts_mpix = 0, ts_pace = 0, ts_scan = 0, ts_scanok = 0;
    always @(posedge clk) if (ts_on && busy) begin
        if (dut.tx_req && dut.tx_ready) begin
            if (dut.tx_mask != 4'hF) ts_mreq++;
            for (int p = 0; p < 4; p++) if (dut.tx_mask[p] !== 1'b1) ts_mpix++;
        end
        if (dut.u_tmu.jf_ne && dut.u_tmu.jq_res && dut.u_tmu.lp_n[dut.u_tmu.jq_cl] &&
            (dut.u_tmu.jq_inc || (dut.u_tmu.bd_v[dut.u_tmu.jq_e[0]] && dut.u_tmu.bd_v[dut.u_tmu.jq_e[1]] &&
                                  dut.u_tmu.bd_v[dut.u_tmu.jq_e[2]] && dut.u_tmu.bd_v[dut.u_tmu.jq_e[3]]))) ts_pace++;
        if (dut.ms == dut.M_PROG && !dut.fe_go && dut.pg_n == dut.dsl.plen && dut.fe_done) begin
            ts_scan++; if (!(dut.tk_bad || dut.tk_badn)) ts_scanok++;
        end
    end
    final if (ts_on && ts_run != 0)
        $display("TMUSTAT sample skip: programs scanned %0d, %0d may skip; requests with a partial mask %0d, pixels not sampled %0d (%0.2f per request); a result held for the 4-cycle spacing %0d cycles",
                 ts_scan, ts_scanok, ts_mreq, ts_mpix, real'(ts_mpix) / (ts_req == 0 ? 1 : ts_req), ts_pace);
    always @(posedge clk) if (ts_on && busy) begin
        ts_run++;
        if (!dut.fs_idle) ts_fsbusy++;
        if (dut.u_fs.issue) ts_issue++;
        for (int w = 0; w < TB_FSW; w++) if (dut.u_fs.busy_v[w] === 1'b1) ts_warps++;
        ts_texlive += dut.u_fs.tex_live;
        if (dut.u_tmu.a_need) ts_need++;
        if (dut.u_tmu.a_alloc) ts_alloc++;
        if (dut.u_tmu.a_need && !dut.u_tmu.a_alloc) begin
            if (dut.u_tmu.bt_ref[dut.u_tmu.bt_ap] != 0 || dut.u_tmu.pend[dut.u_tmu.bt_ap]) begin : rr
                int nf; nf = 0;                                   // free entries elsewhere (a free-list allocator would go on)
                for (int e = 0; e < 16; e++)
                    if (dut.u_tmu.bt_ref[e] == 0 && !dut.u_tmu.pend[e] && (dut.u_tmu.bd_v[e] || !dut.u_tmu.bt_v[e])) nf++;
                ts_bref++; if (nf != 0) ts_bref_free++;
            end
            else if (dut.u_tmu.bt_v[dut.u_tmu.bt_ap] && !dut.u_tmu.bd_v[dut.u_tmu.bt_ap]) ts_bfill++;
            else if (dut.u_tmu.lq_n >= 16) ts_blq++;
        end
        if (dut.u_tmu.fl_rdy) for (int k = 0; k < 4; k++) if (dut.u_tmu.fl_dist[k] === 1'b1) ts_dist++;   // (bit 0 is constant 1)
        if (dut.u_tmu.hv && dut.u_tmu.jf_full) ts_jffull++;
        if (dut.u_tmu.jf_ne && !dut.u_tmu.fl_rdy) ts_fldata++;
        if (dut.u_tmu.lu_valid && !dut.u_tmu.lu_ready) ts_cqfull++;
        if (dut.u_tmu.busy) ts_tmubusy++;
        if (dut.u_tmu.t_take) ts_job++;
        if (dut.u_tmu.fl_rdy) ts_fl++;
        if (dut.u_tmu.f4_v) ts_f4v++;
        if (dut.u_tmu.f4_v && !dut.u_tmu.t_en) ts_f4wait++;
        if (dut.u_tmu.hv && !dut.u_tmu.a_push) ts_astall++;
        if (dut.u_tmu.jf_ne) ts_jfne++;
        if (dut.u_tmu.f2_run) ts_f2++;
        if (dut.u_tmu.wf_full) ts_wffull++;
        if (dut.u_tmu.f1_v && !dut.u_tmu.f1_ltok) ts_f1blk++;
        if (dut.u_tmu.rq_ne) ts_rq++;
        if (dut.tx_req && !dut.tx_ready) ts_rqfull++;
        if (dut.tx_req && dut.tx_ready) begin ts_req++; ts_t.push_back(cyc); end
        if (dut.tx_res_v) begin
            longint l; l = cyc - ts_t.pop_front(); ts_res++; ts_lat += l; if (l > ts_latmax) ts_latmax = l; if (l < tp_min) tp_min = l;
        end
    end
    // +TMUSTAT also: where a texture request's cycles go, stage by stage. Requests and
    // results are in order, so the mean time between two points of the path = the summed count of quads between them /
    // the requests (as the board's tq_n): in the request queue; in F1 / F2 (coordinates); in the wait FIFO ahead of F3; in
    // F3 (LOD); in F4 (job emission); its jobs in the tap stages and the A stage (burst allocation, cache lookup); its
    // last job in the job FIFO (the filter's turn: the jobs ahead, the data from DDR, the result spacing); the filter and
    // the conversion. And a TEX entry's life in the core: coordinates staged and its requests sent; results outstanding;
    // complete, waiting for the write port.
    longint tp_rq = 0, tp_f12 = 0, tp_wf = 0, tp_f3 = 0, tp_f4 = 0, tp_tap = 0, tp_jf = 0, tp_out = 0;
    longint tp_e4 = 0, tp_e5 = 0, tp_e6 = 0, tp_res = 0;
    longint tc_stage = 0, tc_wait = 0, tc_drain = 0, tc_n1 = 0, tc_n2 = 0, tc_n3 = 0, tc_n4 = 0;
    always @(posedge clk) if (ts_on && busy) begin
        tp_rq  += 5'(dut.u_tmu.rq_t - dut.u_tmu.rq_h);
        tp_f12 += dut.u_tmu.f1_v;
        tp_wf  += 5'(dut.u_tmu.wf_t - dut.u_tmu.wf_r);
        tp_f3  += 5'(dut.u_tmu.wf_r - dut.u_tmu.wf_h);
        tp_f4  += dut.u_tmu.f4_v;
        tp_tap += tp_e4 - tp_e5; tp_jf += tp_e5 - tp_e6; tp_out += tp_e6 - tp_res;
        if (dut.u_tmu.f4_v && dut.u_tmu.t_take && dut.u_tmu.f4_lastjob) tp_e4++;
        if (dut.u_tmu.jw_v && dut.u_tmu.jw_last && dut.u_tmu.jw_lastpix) tp_e5++;
        if (dut.u_tmu.fl_rdy && dut.u_tmu.jq_res) tp_e6++;
        if (dut.tx_res_v) tp_res++;
        tc_stage += tc_n1 - tc_n2; tc_wait += tc_n2 - tc_n3; tc_drain += tc_n3 - tc_n4;
        if (dut.u_fs.s_tex1) tc_n1++;
        if (dut.u_fs.seq_done) tc_n2++;
        if (dut.u_fs.rr_endl) tc_n3++;
        if (dut.u_fs.null_done) tc_n3++;
        if (dut.u_fs.bw_tex && dut.u_fs.tex_last) tc_n4++;
    end
    final if (ts_on && ts_req != 0) begin
        real n; n = ts_req;
        $display("TMUSTAT round trip %0.1f cycles a request = request queue %0.1f + F1/F2 %0.1f + wait FIFO %0.1f + F3 %0.1f + F4 %0.1f + taps and A stage %0.1f + job FIFO %0.1f + filter and conversion %0.1f (sum %0.1f); the shortest %0d",
                 real'(ts_lat) / n, tp_rq / n, tp_f12 / n, tp_wf / n, tp_f3 / n, tp_f4 / n, tp_tap / n, tp_jf / n, tp_out / n,
                 (tp_rq + tp_f12 + tp_wf + tp_f3 + tp_f4 + tp_tap + tp_jf + tp_out) / n, tp_min);
        $display("TMUSTAT a TEX in the core (%0d): %0.1f cycles live = staging and requests %0.1f + results outstanding %0.1f + complete, waiting for the write port %0.1f; %0.2f requests a TEX",
                 tc_n1, real'(ts_texlive) / (tc_n1 == 0 ? 1 : tc_n1), real'(tc_stage) / (tc_n1 == 0 ? 1 : tc_n1), real'(tc_wait) / (tc_n1 == 0 ? 1 : tc_n1),
                 real'(tc_drain) / (tc_n1 == 0 ? 1 : tc_n1), n / (tc_n1 == 0 ? 1 : tc_n1));
    end
    final if (ts_on && ts_run != 0) begin
        $display("TMUSTAT run %0d cycles (%0.2f per quad): FS busy %0.1f%%, issue slots used %0.1f%% of busy, live warps %0.2f, live TEXes %0.2f",
                 ts_run, real'(ts_run) / c_quads, 100.0 * ts_fsbusy / ts_run, 100.0 * ts_issue / ts_fsbusy, real'(ts_warps) / ts_run,
                 real'(ts_texlive) / ts_run);
        $display("TMUSTAT TMU busy %0.1f%% of the frame; jobs %0d = %0.1f%% of FS-busy cycles (%0.2f per quad), filtered %0d; F4 holds a quad %0.1f%%, waits for the tap FIFO %0.1f%%; A head stalled %0.1f%%; job FIFO non-empty %0.1f%%; F2 %0.1f%%; wait FIFO full %0.1f%%; level table %0.1f%%; requests queued %0.1f%%; core blocked on req_ready %0.1f%%",
                 100.0 * ts_tmubusy / ts_run, ts_job, 100.0 * ts_job / ts_fsbusy, real'(ts_job) / c_quads, ts_fl, 100.0 * ts_f4v / ts_run,
                 100.0 * ts_f4wait / ts_run, 100.0 * ts_astall / ts_run, 100.0 * ts_jfne / ts_run, 100.0 * ts_f2 / ts_run,
                 100.0 * ts_wffull / ts_run, 100.0 * ts_f1blk / ts_run, 100.0 * ts_rq / ts_run, 100.0 * ts_rqfull / ts_run);
        $display("TMUSTAT requests %0d, results %0d, round trip avg %0.1f max %0d cycles; cache hits %0d misses %0d (%0.1f%%; merges onto prefetches %0d), prefetches %0d (dropped %0d), DDR reads %0d",
                 ts_req, ts_res, real'(ts_lat) / (ts_res == 0 ? 1 : ts_res), ts_latmax, dut.u_tmu.U_CACHE.n_hit, dut.u_tmu.U_CACHE.n_miss,
                 100.0 * dut.u_tmu.U_CACHE.n_miss / ((dut.u_tmu.U_CACHE.n_hit + dut.u_tmu.U_CACHE.n_miss) == 0 ? 1 : (dut.u_tmu.U_CACHE.n_hit + dut.u_tmu.U_CACHE.n_miss)),
                 dut.u_tmu.U_CACHE.n_merge, dut.u_tmu.U_CACHE.n_pf, dut.u_tmu.U_CACHE.n_pfdrop, n_rd);
        $display("TMUSTAT A stage: a burst missing %0.1f%% of cycles, allocated %0.1f%% (%0d, %0.2f per job; %0.2f distinct bursts per job), blocked: entry referenced %0.1f%% (free entries elsewhere %0.1f%%), fill pending %0.1f%%, 16 lookups queued %0.1f%%; job FIFO full %0.1f%%; filter head waits for data %0.1f%%; cache refuses a lookup %0.1f%%",
                 100.0 * ts_need / ts_run, 100.0 * ts_alloc / ts_run, ts_alloc, real'(ts_alloc) / (ts_fl == 0 ? 1 : ts_fl), real'(ts_dist) / (ts_fl == 0 ? 1 : ts_fl),
                 100.0 * ts_bref / ts_run, 100.0 * ts_bref_free / ts_run, 100.0 * ts_bfill / ts_run,
                 100.0 * ts_blq / ts_run, 100.0 * ts_jffull / ts_run, 100.0 * ts_fldata / ts_run, 100.0 * ts_cqfull / ts_run);
        // the board's cumulative profile counters (b_render hwprof, JTAG words 24-47) beside the bench's own counts of the
        // same things: they must agree to within the few cycles of the events' registers
        begin
            longint h [0:23];
            for (int i = 0; i < 24; i++) h[i] = dut.hwprof[i*32 +: 32];
            $display("HWPROF run %0d (bench %0d) FS busy %0d (%0d) issue %0d (%0d) busy-warp sum %0d (%0d) live-TEX sum %0d (%0d) | TMU busy %0d (%0d) filter waits %0d (%0d) requests %0d (%0d) in-TMU sum %0d (bench round-trip sum %0d) DDR reads %0d outstanding sum %0d",
                     h[0], ts_run, h[4], ts_fsbusy, h[8], ts_issue, h[12], ts_warps, h[13], ts_texlive, h[18], ts_tmubusy, h[19], ts_fldata, h[20], ts_req, h[21], ts_lat, h[22], h[23]);
            $display("HWPROF warp sums: waiting with a texture outstanding %0d, program fetch %0d, waiting otherwise %0d; warps launched %0d; pixel issuer %0d (bench %0d), early queue full %0d, a quad waits for a warp slot %0d; flush %0d band change %0d draw change %0d band set-up %0d stream empty %0d ROP stall %0d",
                     h[9], h[10], h[11], h[14], h[15], wf_pi, h[16], h[17], h[1], h[2], h[3], h[5], h[6], h[7]);
        end
    end
    // ---- +DSSTAT: where a draw-state load's cycles go: the wait for a
    // free bank, the bucket read, the uniforms, the program; loads, bank swaps (no load), program fetches skipped (the bank held it)
    bit ds_on; initial ds_on = $test$plusargs("DSSTAT");
    longint ds_wait = 0, ds_read = 0, ds_uni = 0, ds_prog = 0, ds_loads = 0, ds_swaps = 0, ds_pfetch = 0;
    reg [4:0] ds_prev = 0;
    always @(posedge clk) if (ds_on && !rst) begin
        if (dut.ms == dut.M_DSWAIT) ds_wait++;
        if (dut.ms == dut.M_DSREAD) ds_read++;
        if (dut.ms == dut.M_UNI) ds_uni++;
        if (dut.ms == dut.M_PROG) ds_prog++;
        if (dut.ms == dut.M_DSWAIT && ds_prev != dut.M_DSWAIT) ds_loads++;
        if (dut.ms == dut.M_PROG && ds_prev != dut.M_PROG) ds_pfetch++;
        if (dut.ms == dut.M_ITEM && dut.sb_valid && !dut.quad_blk && dut.bk_swap) ds_swaps++;
        ds_prev <= dut.ms;
    end
    final if (ds_on) begin
        $display("DSSTAT %0d draw-state loads (%0d bank swaps without a load), %0d program fetches; cycles: wait for a free bank %0d, bucket %0d, uniforms %0d, program %0d = %0d total (%0.0f per load, %0.1f%% of the frame)",
                 ds_loads, ds_swaps, ds_pfetch, ds_wait, ds_read, ds_uni, ds_prog, ds_wait + ds_read + ds_uni + ds_prog,
                 real'(ds_wait + ds_read + ds_uni + ds_prog) / (ds_loads == 0 ? 1 : ds_loads), 100.0 * (ds_wait + ds_read + ds_uni + ds_prog) / (cyc == 0 ? 1 : cyc));
    end
    // progress / deadlock diagnosis
`ifdef DBG
    always @(posedge clk) begin
        if (dut.ms == dut.M_DSREAD && dut.rf_nonempty && !dut.fe_go)
            $display("  [%0d] DSREAD i %0d head %h", cyc, dut.ds_i, dut.rf_head);
        if (dut.fe_go) $display("  [%0d] fetch %h n %0d (ms %0d)", cyc, dut.fe_a, dut.fe_n, dut.ms);
        if (r0_en && r0_rdy) $display("  [%0d] R0 read %h", cyc, r0_a);
    end
`endif
    // +MAXCYC_<n>: the run's cycle limit (default 3 000 000; full-size frames need more)
    int cyc_max = 3000000;
    initial void'($value$plusargs("MAXCYC_%d", cyc_max));
    always @(posedge clk) begin
        cyc++;
        if (cyc % 20000 == 0 || cyc == cyc_max)
            $display("  [%0d] ms %0d ls %0d q_live %0d qf %0d qs_infl %0d eq %0d pi %b beats left %0d quads %0d tris %0d fe %0d/%0d/%0d fs_idle %b rop_st %0d",
                cyc, dut.ms, dut.ls, dut.q_live, dut.qf_cnt, dut.qs_infl, dut.eq_cnt, dut.pi_run, beats.size(), c_quads,
                c_tris, dut.fe_iss, dut.fe_pop, dut.fe_tot, dut.fs_idle, dut.u_rop.st);
        if (cyc % 20000 == 0 || cyc == cyc_max)
            $display("      fs busy %b fin %b rp %0d ap %0d lq %0d bs %0d | tex_req %b rdy %b live %0d | bq_v %b br_v %b br_rdy %b | prog %h len %0d ok %b draw %0d",
                dut.u_fs.busy_v, dut.u_fs.fin_v, dut.u_fs.rp, dut.u_fs.ap, dut.lq, dut.bs, dut.tx_req, dut.tx_ready,
                dut.u_fs.tex_live, dut.bq_v, dut.br_valid, dut.br_ready, dut.ld_prog, dut.ld_plen, dut.prog_ok, dut.active_draw);
        if (cyc == cyc_max) begin $display("TB_FAIL tb_b_render: TIMEOUT"); $finish; end
    end
    // stall detector: no quad collected for 200 000 cycles while the frame runs -> the peek / launch / back-end state, fail
    int stall_q = -1, stall_n = 0;
    always @(posedge clk) begin
        if (dut.ms == dut.M_IDLE || c_quads != stall_q) begin stall_q = c_quads; stall_n = 0; end
        else if (++stall_n == 200000) begin
            $display("  STALL: qf_rp %0d eq_pp %0d qf_wp %0d cnt %0d | pk_row %b eq_ne %b ez_elig %b pk_block %b l_req %b e_req %b bm %0d rs_slot %0d eq slot %0d nx_ok %b | eq %0d head vd %b lv %b ez %b | pd %b%b%b pk_acc %b | ls %0d lq %0d bs %0d bq_v %b bl %b br_v %b br_rdy %b rop_hold %b",
                dut.qf_rp, dut.eq_pp, dut.qf_wp, dut.qf_cnt, dut.pk_row, dut.ep_ne, dut.ez_elig, dut.pk_block, dut.l_req, dut.e_req,
                dut.bm, dut.rs_slot, dut.ep_slot, dut.nx_ok, dut.eq_cnt, dut.eq_vd[dut.eq_h], dut.eq_lv[dut.eq_h], dut.qf_ez[dut.qh], dut.pd1_v, dut.pd2_v, dut.pd3_v,
                dut.pk_acc, dut.ls, dut.lq, dut.bs, dut.bq_v, dut.bl, dut.br_valid, dut.br_ready, dut.rop_hold);
            $display("TB_FAIL tb_b_render: STALL"); $finish;
        end
    end
endmodule
`default_nettype wire
