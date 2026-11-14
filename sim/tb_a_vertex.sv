`timescale 1ns/1ps
// tb_a_vertex — board A's vertex stage (a_vertex: shader_core_vs2 + vtx_feed + vtx_rbuf + vtx_asm) on one draw against
// tools/a_vtx_ref.py's model: the DDR image (index and attribute buffers) goes into the MIG model, the program /
// uniforms / attribute descriptors through the load ports; the stream of primitive vertices (gl_Position + 8 varyings,
// 3 per triangle, in order) must equal the golden bit for bit, and the draw must end with the stage idle.
//   +SEED_<n>  (tools/gen/av_<n>_*.mem / .hex;  python3 tools/a_vtx_ref.py <n> [--mode strip|fan] [--arrays] [--u8])
//   +STALL     random back-pressure on the vertex output (vo_ready)
//   +TRACE     av_trace.txt in the run directory: one line per cycle with activity (the core's register writes, launches
//              and retires, the feed's lane writes, the vertex stream) — two RTL versions of one configuration compared
//              cycle by cycle
//   defines (sim_gpu.ps1 -Defines): VS_NL8 / VS_NL4 the vertex core's lanes per warp (default 16); VS_ONEB the one-board
//              configuration of the core (NWARP 4, SFU_TAB2 1, INT_DIV 0, SINCOS 0)
`default_nettype none
`ifdef VS_NL4
`define TB_NLP .NL(4),
`elsif VS_NL8
`define TB_NLP .NL(8),
`else
`define TB_NLP
`endif
`ifdef VS_ONEB
`define TB_CFG .NWARP(4), .SFU_TAB2(1'b1), .INT_DIV(1'b0), .SINCOS(1'b0),
`else
`define TB_CFG
`endif
module tb_a_vertex;
    localparam int W = 24, TW = 96, NVEC = 8, VW = NVEC * TW;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg         iram_we = 0, param_we = 0, ad_we = 0, ac_we = 0, dr_start = 0, vo_ready = 1;
    reg  [11:0] iram_waddr = 0; reg [63:0] iram_wdata = 0; reg [7:0] param_idx = 0; reg [TW-1:0] param_data = 0;
    reg  [3:0]  ad_idx = 0, ac_idx = 0; reg ad_en = 0, ad_norm = 0; reg [29:0] ad_base = 0, ad_stride = 0, ad_off = 0; reg [2:0] ad_size = 0, ad_type = 0;
    reg  [TW-1:0] ac_const = 0;
    reg  [2:0]  dr_mode = 0; reg dr_indexed = 0, dr_u8 = 0; reg [29:0] dr_idx_base = 0; reg [31:0] dr_first = 0, dr_count = 0; reg [7:0] varmask = 0;
    wire        dr_busy, dr_ok, err_stack, r_en, r_rdy, r_valid, vo_valid, vo_first; wire [28:0] r_addr; wire [255:0] rd_data;
    wire [TW-1:0] vo_pos; wire [VW-1:0] vo_var; wire [1:0] vo_prim; wire [W-1:0] vo_psize;
    a_vertex #(`TB_NLP `TB_CFG .NVEC(NVEC)) dut (.clk, .rst, .iram_we, .iram_waddr, .iram_wdata, .param_we, .param_idx, .param_data,
        .ad_we, .ad_idx, .ad_en, .ad_type, .ad_norm, .ad_base, .ad_size, .ad_stride, .ad_off, .ac_we, .ac_idx, .ac_const,
        .dr_start, .dr_mode, .dr_indexed, .dr_u8, .dr_idx_base, .dr_first, .dr_count, .dr_varmask(varmask), .dr_ok, .copy_busy(), .draw_done(), .dr_busy, .err_stack,
        .r_en, .r_addr, .r_rdy, .r_valid, .rd_data, .vo_valid, .vo_ready, .vo_pos, .vo_var, .vo_first, .vo_prim, .vo_psize);
    // the MIG model on the read port (reads only)
    wire app_rdy, app_wdf_rdy, app_rd_valid; wire [255:0] app_rd_data; int mig_errors; longint n_wr, n_rd;
    mig_app_model #(.RD_LAT(24), .RDY_PCT(70)) mig (.ui_clk(clk), .rst, .app_en(r_en), .app_cmd(3'b001), .app_addr(r_addr),
        .app_wdf_data(256'd0), .app_wdf_mask(32'hFFFFFFFF), .app_wdf_wren(1'b0), .app_wdf_end(1'b0),
        .app_rdy, .app_wdf_rdy, .app_rd_data, .app_rd_data_valid(app_rd_valid), .errors(mig_errors), .n_wr, .n_rd);
    assign r_rdy = app_rdy; assign r_valid = app_rd_valid; assign rd_data = app_rd_data;
    // +TRACE (see the header)
    int tfd = 0; longint tcyc = 0;
    initial if ($test$plusargs("TRACE")) tfd = $fopen("av_trace.txt", "w");
    always @(posedge clk) begin
        tcyc++;
        if (tfd != 0 && (dut.u_core.gpr_we || dut.va_we || dut.in_start || dut.c_out_ack || r_en || r_valid || (vo_valid && vo_ready)))
            $fwrite(tfd, "%0d %b%b%b%b%b%b%b w %h %h %h %h | va %h %h %h | in %h | r %h | vo %h %h\n", tcyc,
                    dut.u_core.gpr_we, dut.va_we, dut.in_start, dut.c_out_ack, r_en, r_valid, vo_valid && vo_ready,
                    dut.u_core.gpr_we ? dut.u_core.gpr_wa : '0, dut.u_core.gpr_we ? dut.u_core.gpr_wl : '0,
                    dut.u_core.gpr_we ? dut.u_core.gpr_wd : '0, dut.u_core.issue,
                    dut.va_lane, dut.va_idx, dut.va_we ? dut.va_data : '0, dut.in_start ? dut.in_active : '0, r_en ? r_addr : '0,
                    vo_valid && vo_ready ? vo_pos : '0, vo_first);
    end

    // av_<seed>_draw.mem: {varmask[7:0], idx_base[29:0] (bytes), count[31:0], first[31:0], u8, indexed, mode[2:0]} (107 bits)
    // av_<seed>_attr.mem: {const[95:0], off[29:0], stride[29:0], norm, type[2:0], size[2:0], en, base[29:0]} (194 bits; bytes)
    reg [63:0]  iram [0:4095]; reg [TW-1:0] prm [0:255]; reg [106:0] drw [0:0]; reg [193:0] att [0:15];
    reg [10*TW-1:0] ex [0:8191];                 // per vertex: position, 8 varyings, the gl_PointSize row (Q10)
    int unsigned rs = 5;
    function automatic int unsigned rnd(); rs = rs * 1103515245 + 12345; return rs >> 8; endfunction
    initial begin
        string seed; int fd, n, nexp, err, cyc, b, psz; reg [31:0] bi; reg [255:0] bd; bit stall;
        if (!$value$plusargs("SEED_%s", seed)) seed = "1";
        stall = $test$plusargs("STALL");
        for (int k = 0; k < 8192; k++) ex[k] = 'x;
        $readmemh({"tools/gen/av_", seed, "_iram.mem"}, iram);
        $readmemh({"tools/gen/av_", seed, "_params.mem"}, prm);
        $readmemh({"tools/gen/av_", seed, "_draw.mem"}, drw);
        $readmemh({"tools/gen/av_", seed, "_attr.mem"}, att);
        $readmemh({"tools/gen/av_", seed, "_expect.mem"}, ex);
        nexp = 0; while (nexp < 8192 && !$isunknown(ex[nexp])) nexp++;
        // the DDR image
        fd = $fopen({"tools/gen/av_", seed, "_ddr.hex"}, "r");
        if (fd == 0) begin $display("TB_FAIL tb_a_vertex: no DDR image"); $finish; end
        b = 0;
        while ($fscanf(fd, "%h %h", bi, bd) == 2) begin mig.mem[longint'(bi)] = bd; b++; end
        $fclose(fd);
        repeat (4) @(posedge clk); rst <= 0; repeat (2) @(posedge clk);
        n = 0; while (n < 4096 && !$isunknown(iram[n])) n++;
        for (int k = 0; k < n; k++) begin @(posedge clk); iram_we <= 1; iram_waddr <= 12'(k); iram_wdata <= iram[k]; end
        for (int k = 0; k < 256; k++) begin @(posedge clk); iram_we <= 0; param_we <= 1; param_idx <= 8'(k); param_data <= prm[k]; end
        @(posedge clk); param_we <= 0;
        for (int k = 0; k < 16; k++) begin
            @(posedge clk); ad_we <= 1; ac_we <= 1; ad_idx <= 4'(k); ac_idx <= 4'(k);
            ad_base <= att[k][29:0]; ad_en <= att[k][30]; ad_size <= att[k][33:31]; ad_type <= att[k][36:34]; ad_norm <= att[k][37];
            ad_stride <= att[k][67:38]; ad_off <= att[k][97:68]; ac_const <= att[k][193:98];
        end
        @(posedge clk); ad_we <= 0; ac_we <= 0;
        dr_mode <= drw[0][2:0]; dr_indexed <= drw[0][3]; dr_u8 <= drw[0][4]; dr_first <= drw[0][36:5]; dr_count <= drw[0][68:37];
        dr_idx_base <= drw[0][98:69]; varmask <= drw[0][106:99];
        psz = (drw[0][2:0] <= 3'd2) ? 3 : (drw[0][2:0] == 3'd3) ? 1 : 2;     // vertices per primitive (Q10: points 1, lines 2)
        @(posedge clk); while (!dr_ok) @(posedge clk);
        dr_start <= 1; @(posedge clk); dr_start <= 0;
        // collect the vertex stream
        n = 0; err = 0; cyc = 0;
        // vo_ready for a cycle is set right after the edge; the handshake of that cycle is judged at its negedge
        while (cyc < 2_000_000 && (dr_busy || n < nexp)) begin
            @(posedge clk); #1; cyc++;
            vo_ready = stall ? (rnd() % 3 != 0) : 1'b1;
            @(negedge clk);
            if (vo_valid && vo_ready) begin
                if (n < nexp) begin
                    if ({vo_pos, vo_var[0*TW +: TW], vo_var[1*TW +: TW], vo_var[2*TW +: TW], vo_var[3*TW +: TW],
                         vo_var[4*TW +: TW], vo_var[5*TW +: TW], vo_var[6*TW +: TW], vo_var[7*TW +: TW]} !== ex[n][TW +: 9*TW]) begin
                        err++;
                        if (err < 6) $display("  vertex %0d (prim %0d, v%0d) got pos %h var0 %h var1 %h var2 %h var5 %h\n          exp %h", n, n / psz, n % psz,
                            vo_pos, vo_var[0 +: TW], vo_var[TW +: TW], vo_var[2*TW +: TW], vo_var[5*TW +: TW], ex[n][TW +: 9*TW]);
                    end
                    if (vo_first !== (n % psz == 0)) begin err++; if (err < 6) $display("  vertex %0d: vo_first %b", n, vo_first); end
                    if (vo_prim !== (psz == 3 ? 2'd0 : psz == 2 ? 2'd1 : 2'd2)) begin err++; if (err < 6) $display("  vertex %0d: vo_prim %0d", n, vo_prim); end
                    if (psz == 1 && vo_psize !== ex[n][0 +: W]) begin err++; if (err < 6) $display("  vertex %0d: point size %h expected %h", n, vo_psize, ex[n][0 +: W]); end
                end else begin err++; if (err < 6) $display("  extra vertex %0d", n); end
                n++;
            end
            if (!dr_busy && n >= nexp) break;
        end
        repeat (4) @(posedge clk);
        if (n != nexp) begin err++; $display("  %0d vertices, expected %0d", n, nexp); end
        if (dr_busy) begin
            err++;
            $display("  stage still busy after %0d cycles: feed ist %0d i %0d cl %0d cs %0d r_en %b ref_valid %b jobs %0d inflight %0d | asm st %0d consumed %0d fifo %0d vi %0d cur slot %0d | done %b free %b lq %0d core idle %b in_ready %b out_valid %b",
                     cyc, dut.u_feed.ist, dut.u_feed.i, dut.u_feed.cl, dut.u_feed.cs, dut.u_feed.r_en, dut.u_feed.ref_valid, dut.u_feed.jq_cnt, dut.u_feed.inflight,
                     dut.u_asm.st, dut.u_asm.consumed, dut.u_asm.f_cnt, dut.u_asm.vi, dut.u_asm.cur >> (dut.u_asm.RW - dut.u_asm.SW),
                     dut.u_rbuf.done, dut.u_rbuf.slot_free, dut.u_rbuf.lq_cnt, dut.u_core.idle, dut.u_core.in_ready, dut.u_core.out_valid);
        end
        if (err_stack) begin err++; $display("  core stack violation"); end
        if (mig_errors != 0) begin err++; $display("  MIG model errors %0d", mig_errors); end
        $display("AV seed %s: %0d primitive vertices in %0d cycles (%0d DDR reads, %0d bursts in the image), errors = %0d",
                 seed, n, cyc, n_rd, b, err);
        if (err == 0) $display("TB_PASS tb_a_vertex %s", seed); else $display("TB_FAIL tb_a_vertex %s", seed);
        $finish;
    end
    initial begin #40ms; $display("TB_FAIL tb_a_vertex: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
