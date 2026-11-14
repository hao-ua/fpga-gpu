`timescale 1ns/1ps
// tb_video_mmcm — checks the generated video-mode DRP ROM (src/rtl/disp/video_modes.svh, tools/mmcm_drp_calc.py)
// and video_mmcm's reconfiguration:
//   1. golden: one UNISIM MMCME2_ADV per mode with that mode's static attributes; its DRP registers (initialised
//      by the model from the attributes with Xilinx's own calculation) are read back and compared, under the
//      keep-mask, with every ROM entry of the mode (bits the model leaves unknown are skipped);
//   2. video_mmcm (static = VM_STATIC_MODE) is reconfigured through all modes in a mixed order; after each
//      change: done with ok, LOCKED, pixel and serial clock frequencies within 0.05 % of the mode's value and
//      serial = 5 x pixel.
`default_nettype none
module tb_video_mmcm;
`include "video_modes.svh"
    reg clk200 = 0, clk = 0, rst = 1;
    always #2.5 clk200 = ~clk200;
    always #5.0 clk = ~clk;
    int errors = 0;

    // ---------------- golden static MMCMs ----------------
    reg  [6:0]  g_daddr = 0;
    reg         g_den [0:VM_NUM_MODES-1];
    wire [15:0] g_do [0:VM_NUM_MODES-1];
    wire        g_drdy [0:VM_NUM_MODES-1];
    genvar gm;
    generate for (gm = 0; gm < VM_NUM_MODES; gm = gm + 1) begin : g_gold
        wire fb;
        MMCME2_ADV #(
            .BANDWIDTH("OPTIMIZED"), .CLKIN1_PERIOD(5.000),
            .DIVCLK_DIVIDE(vm_divclk(gm)), .CLKFBOUT_MULT_F(vm_mult_f(gm)), .CLKFBOUT_PHASE(0.0),
            .CLKOUT0_DIVIDE_F(1.0 * vm_o0(gm)), .CLKOUT0_PHASE(0.0), .CLKOUT0_DUTY_CYCLE(0.5),
            .CLKOUT1_DIVIDE(vm_o1(gm)), .CLKOUT1_PHASE(0.0), .CLKOUT1_DUTY_CYCLE(0.5),
            .REF_JITTER1(0.010), .STARTUP_WAIT("FALSE")
        ) u (
            .CLKIN1(clk200), .CLKIN2(1'b0), .CLKINSEL(1'b1), .CLKFBIN(fb), .CLKFBOUT(fb), .CLKFBOUTB(),
            .CLKOUT0(), .CLKOUT0B(), .CLKOUT1(), .CLKOUT1B(), .CLKOUT2(), .CLKOUT2B(), .CLKOUT3(), .CLKOUT3B(),
            .CLKOUT4(), .CLKOUT5(), .CLKOUT6(),
            .DCLK(clk), .DADDR(g_daddr), .DEN(g_den[gm]), .DWE(1'b0), .DI(16'h0), .DO(g_do[gm]), .DRDY(g_drdy[gm]),
            .PSCLK(1'b0), .PSEN(1'b0), .PSINCDEC(1'b0), .PSDONE(), .LOCKED(), .CLKINSTOPPED(), .CLKFBSTOPPED(),
            .PWRDWN(1'b0), .RST(1'b0));
        initial g_den[gm] = 1'b0;
    end endgenerate

    task automatic gold_read(input int m, input [6:0] a, output [15:0] v);
        @(posedge clk); g_daddr <= a; g_den[m] <= 1'b1;
        @(posedge clk); g_den[m] <= 1'b0;
        while (g_drdy[m] !== 1'b1) @(posedge clk);
        v = g_do[m];
    endtask

    // ---------------- DUT ----------------
    reg        req = 0; reg [2:0] req_mode = 0;
    wire       busy, done, ok, locked, locked_raw, pix_clk, ser_clk;
    video_mmcm #(.LOCK_TIMEOUT(200_000)) dut (.clk(clk), .rst(rst), .clkin(clk200), .req(req), .req_mode(req_mode),
        .busy(busy), .done(done), .ok(ok), .locked(locked), .locked_raw(locked_raw), .pix_clk(pix_clk), .ser_clk(ser_clk));

    // frequency counters
    longint n_pix = 0, n_ser = 0;
    always @(posedge pix_clk) n_pix = n_pix + 1;
    always @(posedge ser_clk) n_ser = n_ser + 1;
    task automatic measure(input int m);
        longint p0, s0; real fp, fs, exp_khz; realtime t0;
        p0 = n_pix; s0 = n_ser; t0 = $realtime;
        #(20_000.0);                                            // 20 us gate
        fp = (n_pix - p0) / (($realtime - t0) * 1e-6);         // kHz (time in ns)
        fs = (n_ser - s0) / (($realtime - t0) * 1e-6);
        exp_khz = vm_pix_khz(m);
        if (fp < exp_khz * 0.9995 || fp > exp_khz * 1.0005) begin
            errors++; $display("FAIL mode %0d: pixel clock %.1f kHz, expected %.1f", m, fp, exp_khz);
        end
        if (fs < 5.0 * fp * 0.9995 || fs > 5.0 * fp * 1.0005) begin
            errors++; $display("FAIL mode %0d: serial clock %.1f kHz is not 5 x pixel %.1f", m, fs, fp);
        end
        $display("ok: mode %0d pixel %.2f kHz (expected %.2f), serial %.2f kHz", m, fp, exp_khz, fs);
    endtask

    task automatic change(input int m);
        realtime t0;
        @(posedge clk); req <= 1'b1; req_mode <= m[2:0];
        @(posedge clk); req <= 1'b0;
        t0 = $realtime;
        while (done !== 1'b1) @(posedge clk);
        if (ok !== 1'b1) begin errors++; $display("FAIL mode %0d: not locked after reconfiguration", m); end
        else $display("ok: mode %0d locked %.1f us after the request", m, ($realtime - t0) / 1000.0);
        if (locked !== 1'b1) begin errors++; $display("FAIL mode %0d: LOCKED low after done", m); end
        #(2000.0);                                              // let the output dividers settle
        measure(m);
    endtask

    initial begin
        int m, k, nchk; reg [38:0] e; reg [15:0] v; reg [6:0] a; reg [15:0] msk, dat;
        repeat (20) @(posedge clk);
        rst <= 1'b0;
        // ---- 1. golden register check ----
        #(1000.0);
        nchk = 0;
        for (m = 0; m < VM_NUM_MODES; m++) begin
            for (k = 0; k < VM_DRP_REGS; k++) begin
                e = vm_drp(m[2:0], k[4:0]); a = e[38:32]; msk = e[31:16]; dat = e[15:0];
                gold_read(m, a, v);
                for (int b = 0; b < 16; b++) begin
                    if (msk[b] == 1'b0 && v[b] !== 1'bx) begin
                        nchk++;
                        if (v[b] !== dat[b]) begin
                            errors++;
                            $display("FAIL golden mode %0d reg %02h bit %0d: model %b, ROM %b (model %04h ROM %04h mask %04h)",
                                     m, a, b, v[b], dat[b], v, dat, msk);
                        end
                    end
                end
            end
        end
        $display("ok: golden DRP check, %0d register bits compared over %0d modes", nchk, VM_NUM_MODES);
        // ---- 2. reconfiguration ----
        while (locked !== 1'b1) @(posedge clk);                 // static configuration first
        measure(VM_STATIC_MODE);
        change(1); change(0); change(3); change(2); change(4); change(5); change(1); change(2); change(5); change(0);
        if (errors == 0) $display("TB_PASS tb_video_mmcm");
        else $display("TB_FAIL tb_video_mmcm errors=%0d", errors);
        $finish;
    end
    initial begin #(50_000_000.0); $display("TB_FAIL tb_video_mmcm timeout"); $finish; end
endmodule
`default_nettype wire
