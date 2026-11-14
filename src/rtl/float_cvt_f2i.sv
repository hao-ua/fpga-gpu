// float_cvt_f2i — float (EXP_W,MAN_W) -> signed integer, truncate toward zero
// (floor of magnitude). FTZ inputs. Latency 2, II=1. For screen-coord conversion.
`default_nettype none
module float_cvt_f2i #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int OW    = 16        // signed output width
)(
    input  wire                 clk,
    input  wire                 rst,
    input  wire                 in_valid,
    input  wire [EXP_W+MAN_W:0] fval,
    output reg                  out_valid,
    output reg  signed [OW-1:0] ival
);
    localparam int W=1+EXP_W+MAN_W, SIG=MAN_W+1, BIAS=(1<<(EXP_W-1))-1, INF_E=(1<<EXP_W)-1;

    reg v0, s0; reg signed [EXP_W+1:0] e0; reg [SIG-1:0] sig0; reg ovf0;
    always @(posedge clk) begin
        v0  <= in_valid;
        s0  <= fval[W-1];
        e0  <= $signed({2'b0,fval[W-2:MAN_W]}) - BIAS;
        sig0<= (|fval[W-2:MAN_W]) ? {1'b1, fval[MAN_W-1:0]} : '0;   // FTZ
        ovf0<= (fval[W-2:MAN_W]==INF_E);
    end

    // value = sig * 2^(e-MAN_W). Compute the signed integer, then SATURATE to the signed OW-bit range
    // (finite out-of-range -> clamp, matching the reference; Inf/NaN -> 0); keeping only the low OW bits would
    // WRAP finite overflow (e.g. 2^23 -> -2^23 for OW=24).
    // Width: `mag` only ever reaches ~2^OW — the e0>=OW guard below caps the shift at (OW-1-MAN_W),
    // so sig<<(OW-1-MAN_W) < 2^OW and IMAX+1 = 2^(OW-1). OW+3 (signed, +-2^(OW+2)) holds every value with
    // margin, and the variable shifter and the saturate compare stay that narrow.
    // BUT the pad `{{(SHW-SIG){0}},sig0}` also requires SHW>=SIG(=MAN_W+1): when OW<MAN_W-2 (e.g. the
    // vtx_setup screen-coord lane, OW=CW=12, MAN_W=16) OW+3<SIG -> a NEGATIVE replication count -> sim
    // FATAL. Take max(OW,MAN_W)+3 so the pad is always valid; identical to OW+3 for the OW>=MAN_W lanes
    // (e.g. the FP24 OW=24 lanes stay 27b, bit-exact with the A7 goldens), and 19b for the OW=12 lane.
    localparam int SHW = (OW > MAN_W ? OW : MAN_W) + 3;
    localparam signed [SHW-1:0] IMAX =  (1 <<< (OW-1)) - 1;
    localparam signed [SHW-1:0] IMIN = -(1 <<< (OW-1));
    reg signed [SHW-1:0] mag, sval;
    always @* begin
        if      (ovf0 || e0 < 0) mag = 0;                                            // Inf/NaN or |v|<1 -> 0
        else if (e0 >= OW)       mag = IMAX + 1;                                      // guaranteed overflow
        else if (e0 >= MAN_W)    mag = $signed({{(SHW-SIG){1'b0}},sig0}) <<< (e0-MAN_W);
        else                     mag = $signed({{(SHW-SIG){1'b0}},sig0}) >>> (MAN_W-e0);
        sval = s0 ? -mag : mag;                                                       // apply sign
        if      (sval > IMAX) sval = IMAX;                                            // saturate (no wrap)
        else if (sval < IMIN) sval = IMIN;
    end

    always @(posedge clk) begin
        out_valid <= v0 & ~rst;
        ival      <= sval[OW-1:0];
    end
endmodule
`default_nettype wire
