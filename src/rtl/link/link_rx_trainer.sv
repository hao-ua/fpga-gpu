// link_rx_trainer — finds the capture-clock phase of one RX lane.
//
// One PSEN pulse moves the MMCM shifter by one step (1/56 VCO period); PSDONE answers ~12 PSCLK
// cycles later. Scan: measure at the current position, then PS_INC steps later, ... NSTEPS points
// in all (one full LCLK period = 56*D steps); walk the good/bad vector twice (circular) for the
// longest run of good points; step back to its centre; re-measure there; declare `trained`.
// The scan is relative to wherever the shifter stands, so nothing depends on the shifter's absolute
// position or wrap-around; `phase` is still reported modulo one period from the shifter's reset
// position (an MMCM reset returns the shifter to 0) so values stay comparable across retrains.
// The measurement itself lives in the lane's PHY (link_rx_meas): request/acknowledge toggles.
`default_nettype none
module link_rx_trainer #(
    parameter int  NSTEPS        = 140,            // scan points per LCLK period = 56*D/PS_INC
    parameter int  PS_INC        = 8,              // shifter steps between scan points (142.9 ps @ 1 GHz VCO)
    parameter int  TRAIN_WORDS   = 1024,           // measurement length (for the answer timeout only)
    parameter int  SETTLE_CYCLES = 64,             // clk cycles after a phase move before measuring
    parameter int  MIN_EYE       = 4,              // scan points the error-free window must span
    parameter int  RETRY_CYCLES  = 100_000         // clk cycles to wait before re-scanning after a failed scan
)(
    input  wire        clk, rst,
    input  wire        rx_ok,                      // recovered clock present (MMCM locked, synced)
    input  wire        mmcm_rst,                   // shifter was reset to position 0
    input  wire        retrain,                    // pulse: drop `trained`, scan again
    // phase shifter
    output reg         psen,
    output reg         psincdec,
    input  wire        psdone,
    // measurement (link_rx_meas)
    output reg         meas_req,                   // toggle: start a measurement
    input  wire        meas_ack,                   // toggle (lclk_rx domain): measurement done
    input  wire        meas_bad,                   // result, stable before meas_ack toggles
    // status
    output reg         trained,
    output reg  [7:0]  eye_width,                  // good points in the chosen window (last scan, even if it failed)
    output reg  [7:0]  phase,                      // scan point of the chosen centre, modulo NSTEPS from reset
    output reg  [7:0]  train_cnt
);
    localparam int MEAS_TO = 32 * TRAIN_WORDS + 1000;
    (* ASYNC_REG = "TRUE" *) reg [2:0] ack_s = 0;
    always @(posedge clk) ack_s <= {ack_s[1:0], meas_ack};
    wire meas_done = ack_s[2] ^ ack_s[1];

    localparam [3:0] S_START = 0, S_SETTLE = 1, S_MEAS = 2, S_STEP = 3, S_PSWAIT = 4,
                     S_ANALYSE = 5, S_GOTO = 6, S_VERIFY = 7, S_TRAINED = 8, S_RETRY = 9;
    reg [3:0]   st = S_START;
    reg [3:0]   ret_st = S_START;          // where to go after the pending moves are done
    reg [255:0] good = 0;                  // good[i] = scan point i was clean
    reg [7:0]   ipt = 0;                   // scan point index
    reg [15:0]  moves = 0;                 // PSEN pulses still to issue in the current move
    reg [31:0]  tmr = 0;
    reg [8:0]   ai = 0;                    // analysis index 0 .. 2*NSTEPS-1
    reg [7:0]   run = 0, best = 0, best_end = 0, centre = 0;
    reg [7:0]   abs_pt = 0;                // shifter position in scan points, modulo NSTEPS, from its reset
    wire [8:0]  ai_mod  = (ai >= NSTEPS) ? ai - NSTEPS : ai;
    wire [8:0]  c_raw   = best_end + NSTEPS - ((best - 1) >> 1);
    wire [8:0]  c_mod   = (c_raw >= NSTEPS) ? c_raw - NSTEPS : c_raw;
    wire [8:0]  ap_raw  = abs_pt + centre;
    wire [8:0]  ap_mod  = (ap_raw >= NSTEPS) ? ap_raw - NSTEPS : ap_raw;
    always @(posedge clk) begin
        psen <= 1'b0;
        if (mmcm_rst) abs_pt <= 0;
        if (rst) begin
            st <= S_START; trained <= 0; eye_width <= 0; phase <= 0; train_cnt <= 0;
            ipt <= 0; moves <= 0; tmr <= 0; good <= 0; psincdec <= 0; meas_req <= 0;
        end else if (!rx_ok) begin
            st <= S_START; trained <= 0; moves <= 0;      // no recovered clock: start over when it returns
        end else begin
            case (st)
            S_START: begin
                trained <= 0; ipt <= 0; good <= 0; tmr <= 0; st <= S_SETTLE;
            end
            S_SETTLE: begin
                if (tmr == SETTLE_CYCLES) begin tmr <= 0; meas_req <= ~meas_req; st <= S_MEAS; end
                else tmr <= tmr + 1;
            end
            S_MEAS: begin
                if (meas_done || tmr == MEAS_TO) begin         // timeout (capture domain not running) counts as bad
                    tmr <= 0;
                    good[ipt] <= meas_done & ~meas_bad;
                    if (ipt == NSTEPS - 1) begin st <= S_ANALYSE; ai <= 0; run <= 0; best <= 0; best_end <= 0; end
                    else begin ipt <= ipt + 1; moves <= PS_INC; psincdec <= 1'b1; ret_st <= S_SETTLE; st <= S_STEP; end
                end else tmr <= tmr + 1;
            end
            S_STEP: begin
                if (moves == 0) st <= ret_st;
                else begin psen <= 1'b1; moves <= moves - 1; st <= S_PSWAIT; tmr <= 0; end
            end
            S_PSWAIT: begin
                if (psdone || tmr == 32'd255) st <= S_STEP;    // PSDONE normally ~12 cycles; never hang
                else tmr <= tmr + 1;
            end
            S_ANALYSE: begin
                // longest circular run of good points; best_end = index (mod NSTEPS) of its last point
                if (ai == 2 * NSTEPS) begin
                    eye_width <= best;                             // report what was found, even if too small
                    if (best < MIN_EYE) begin
                        // we stand NSTEPS-1 points above the scan start (= one point below it, circularly)
                        abs_pt <= (abs_pt == 0) ? NSTEPS - 1 : abs_pt - 1;
                        st <= S_RETRY; tmr <= 0;
                    end else begin centre <= c_mod[7:0]; st <= S_GOTO; end
                end else begin
                    if (good[ai_mod]) begin
                        if (run != NSTEPS) begin
                            run <= run + 1;
                            if (run + 1 > best) begin best <= run + 1; best_end <= ai_mod[7:0]; end
                        end
                    end else run <= 0;
                    ai <= ai + 1;
                end
            end
            S_GOTO: begin
                // we stand (NSTEPS-1)*PS_INC steps above the scan start; step down to centre*PS_INC
                moves <= (NSTEPS - 1 - centre) * PS_INC; psincdec <= 1'b0; ret_st <= S_VERIFY; st <= S_STEP; tmr <= 0;
            end
            S_VERIFY: begin
                if (tmr < SETTLE_CYCLES) tmr <= tmr + 1;
                else if (tmr == SETTLE_CYCLES) begin tmr <= tmr + 1; meas_req <= ~meas_req; end
                else if (meas_done) begin
                    abs_pt <= ap_mod[7:0];                         // we now stand at the centre
                    phase  <= ap_mod[7:0];
                    if (meas_bad) begin st <= S_RETRY; tmr <= 0; end
                    else begin trained <= 1; train_cnt <= train_cnt + 1; st <= S_TRAINED; end
                end else if (tmr == SETTLE_CYCLES + MEAS_TO) begin abs_pt <= ap_mod[7:0]; st <= S_RETRY; tmr <= 0; end
                else tmr <= tmr + 1;
            end
            S_TRAINED: begin
                if (retrain) begin trained <= 0; st <= S_START; end
            end
            S_RETRY: begin
                // nothing usable (far end not sending TRAIN words yet, or eye too small): pause, rescan
                if (tmr == RETRY_CYCLES) st <= S_START; else tmr <= tmr + 1;
            end
            default: st <= S_START;
            endcase
        end
    end
endmodule

// link_rx_meas — measurement window in the recovered-clock domain. On a request: watch TRAIN_WORDS
// cycles of the far end's training stream (xorshift32 on a 32-bit lane; two independent streams,
// xorshift32 for the low and xorshift32b for the high word, on a 64-bit lane). A cycle is bad if no
// word is valid (TRAIN beats come back to back while we are untrained), the word does not continue
// the stream from the previous one, or a parity flag is set. The first valid word of a window only
// seeds the check. A half-swapped capture breaks the continuity (link_funcs.svh). `meas_bad` is
// settled before `meas_ack` toggles.
module link_rx_meas #(
    parameter int     W           = 32,            // 32 or 64
    parameter int     TRAIN_WORDS = 1024
)(
    input  wire         lclk_rx, rst_rx,
    input  wire         meas_req,                  // toggle from the trainer (clk domain)
    input  wire [W-1:0] word,
    input  wire         valid,
    input  wire         par_bad,
    output reg          meas_ack = 0,
    output reg          meas_bad = 0
);
    `include "link_funcs.svh"
    (* ASYNC_REG = "TRUE" *) reg [2:0] req_s = 0;
    reg          run = 0, bad_acc = 0, have_prev = 0;
    reg [15:0]   cnt = 0;
    reg [W-1:0]  prev = 0;
    wire [W-1:0] expected;
    generate
        if (W == 64) begin : g64
            assign expected = {xorshift32b(prev[63:32]), xorshift32(prev[31:0])};
        end else begin : g32
            assign expected = xorshift32(prev);
        end
    endgenerate
    wire word_ok = valid & ~par_bad & (~have_prev | (word == expected));
    always @(posedge lclk_rx) begin
        req_s <= {req_s[1:0], meas_req};
        if (valid) begin prev <= word; have_prev <= 1; end
        if (rst_rx) begin
            run <= 0; cnt <= 0; bad_acc <= 0; have_prev <= 0;
        end else if (!run) begin
            if (req_s[2] ^ req_s[1]) begin run <= 1; cnt <= 0; bad_acc <= 0; have_prev <= 0; end
        end else begin
            if (!word_ok) bad_acc <= 1;
            if (cnt == TRAIN_WORDS - 1) begin
                run <= 0; meas_bad <= bad_acc | ~word_ok; meas_ack <= ~meas_ack;
            end else cnt <= cnt + 1;
        end
    end
endmodule
`default_nettype wire
