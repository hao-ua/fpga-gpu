// sfu_tab2 — the four function tables of float_sfu (RCP / RSQRT seeds, LOG2 / EXP2 lookup tables) for TWO units (float_sfu
// TAB_EXT): every table is a dual-port ROM, port A for unit 0, port B for unit 1, so a pair of units holds one copy
// instead of two. A unit's request is the four addresses of its S1 stage (from its S0 register); the words come back
// registered one cycle later, as float_sfu's own reads (the same tables, the same words).
// LOG2 / EXP2 are read at idx and idx + 1 in the same cycle (the lerp's two ends). Their 2**KLOG + 1 entries lie in two
// halves of 2**(KLOG-1): the even entries 0, 2, .. and the odd ones 1, 3, ..; idx and idx + 1 always fall in different
// halves, so each half is read once per unit and cycle. The last entry (index 2**KLOG, the second end for the largest idx)
// is a constant beside them.
`default_nettype none
module sfu_tab2 #(
    parameter int EXP_W = 7,
    parameter int MAN_W = 16,
    parameter int K     = 12,
    parameter int KLOG  = 11,
    parameter     RCP_MEM="", RSQRT_MEM="", LOG2_MEM="", EXP2_MEM=""
)(
    input  wire                 clk,
    input  wire [K-1:0]         rcp_a0, rcp_a1, rsq_a0, rsq_a1,
    input  wire [KLOG-1:0]      log_i0, log_i1, exp_i0, exp_i1,
    output reg  [MAN_W+1:0]     rcp_q0, rcp_q1, rsq_q0, rsq_q1,
    output wire [23:0]          la0, lb0, la1, lb1, ea0, eb0, ea1, eb1
);
    localparam int SW = MAN_W + 2, LFW = 24, H = 1 << (KLOG - 1);

    (* rom_style="block" *) reg [SW-1:0]  rcp_seed   [0:(1<<K)-1];
    (* rom_style="block" *) reg [SW-1:0]  rsqrt_seed [0:(1<<K)-1];
    (* rom_style="block" *) reg [LFW-1:0] log_e [0:H-1], log_o [0:H-1], exp_e [0:H-1], exp_o [0:H-1];
    reg [LFW-1:0] log_full [0:(1<<KLOG)], exp_full [0:(1<<KLOG)];
    initial begin
        if (RCP_MEM != "")   $readmemh(RCP_MEM, rcp_seed);
        if (RSQRT_MEM != "") $readmemh(RSQRT_MEM, rsqrt_seed);
        if (LOG2_MEM != "")  $readmemh(LOG2_MEM, log_full);
        if (EXP2_MEM != "")  $readmemh(EXP2_MEM, exp_full);
        for (int i = 0; i < H; i++) begin
            log_e[i] = log_full[2*i]; log_o[i] = log_full[2*i+1];
            exp_e[i] = exp_full[2*i]; exp_o[i] = exp_full[2*i+1];
        end
    end
    wire [LFW-1:0] log_top = log_full[1<<KLOG], exp_top = exp_full[1<<KLOG];

    // RCP / RSQRT: one read per unit
    always @(posedge clk) begin
        rcp_q0 <= rcp_seed[rcp_a0];   rcp_q1 <= rcp_seed[rcp_a1];
        rsq_q0 <= rsqrt_seed[rsq_a0]; rsq_q1 <= rsqrt_seed[rsq_a1];
    end

    // LOG2 / EXP2: idx = 2j + p. p = 0: idx is even (E[j]), idx + 1 odd (O[j]); p = 1: idx odd (O[j]), idx + 1 even
    // (E[j+1], or the last entry when j is the last half index)
    function automatic [KLOG-2:0] e_addr(input [KLOG-1:0] i); e_addr = i[0] ? i[KLOG-1:1] + 1'b1 : i[KLOG-1:1]; endfunction
    function automatic bit is_top(input [KLOG-1:0] i); is_top = i[0] & (&i[KLOG-1:1]); endfunction
    reg [LFW-1:0] le0, lo0, le1, lo1, xe0, xo0, xe1, xo1;
    reg lp0, lp1, lt0, lt1, xp0, xp1, xt0, xt1;
    always @(posedge clk) begin
        le0 <= log_e[e_addr(log_i0)]; lo0 <= log_o[log_i0[KLOG-1:1]];
        le1 <= log_e[e_addr(log_i1)]; lo1 <= log_o[log_i1[KLOG-1:1]];
        xe0 <= exp_e[e_addr(exp_i0)]; xo0 <= exp_o[exp_i0[KLOG-1:1]];
        xe1 <= exp_e[e_addr(exp_i1)]; xo1 <= exp_o[exp_i1[KLOG-1:1]];
        lp0 <= log_i0[0]; lt0 <= is_top(log_i0); lp1 <= log_i1[0]; lt1 <= is_top(log_i1);
        xp0 <= exp_i0[0]; xt0 <= is_top(exp_i0); xp1 <= exp_i1[0]; xt1 <= is_top(exp_i1);
    end
    assign la0 = lp0 ? lo0 : le0;  assign lb0 = lp0 ? (lt0 ? log_top : le0) : lo0;
    assign la1 = lp1 ? lo1 : le1;  assign lb1 = lp1 ? (lt1 ? log_top : le1) : lo1;
    assign ea0 = xp0 ? xo0 : xe0;  assign eb0 = xp0 ? (xt0 ? exp_top : xe0) : xo0;
    assign ea1 = xp1 ? xo1 : xe1;  assign eb1 = xp1 ? (xt1 ? exp_top : xe1) : xo1;
    wire _unused = &{1'b0, EXP_W[0]};
endmodule
`default_nettype wire
