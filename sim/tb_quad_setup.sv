// tb_quad_setup — quad_setup against tools/b_ref.py (python3 tools/b_ref.py qs <seed> <ntri> -> tools/gen/qs_vectors.hex).
// Triangles go to the 4 slots in turn (a slot is rewritten only once its pixels have left); pixels follow with random
// gaps, sometimes right behind their triangle's slot write; every output field is compared bit for bit, in order.
// A triangle carries R and the 6 edge steps, a pixel its quad's pixel-0 edge values and its index p;
// the mus the DUT forms (gl_PointCoord) are checked as well.
`timescale 1ns/1ps
module tb_quad_setup;
    localparam int W = 24;
    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    reg tri_we = 0; reg [1:0] tri_slot = 0;
    reg [W-1:0] t_z0, t_dz1, t_dz2, t_q0, t_q1, t_q2, t_dzdx, t_dzdy, t_fac, t_R; reg t_po = 0; reg signed [23:0] t_units;
    reg signed [31:0] t_ab [0:5];                          // the edge steps: A0 B0 A1 B1 A2 B2
    reg depth16 = 0;                                       // the vector file's header (b_ref qs ... d16)
    reg px_valid = 0; reg [1:0] px_slot = 0; reg signed [31:0] e0, e1, e2; reg [1:0] pp; reg [12:0] px, py; reg [7:0] tag = 0;
    wire o_valid; wire [7:0] o_tag; wire [W-1:0] o_z, o_fw, o_l1, o_l2, o_fcx, o_fcy, o_fcz, o_mu1, o_mu2; wire [23:0] o_d24;
    quad_setup #(.EXP_W(7), .MAN_W(16), .TAGW(8), .NT(4)) dut (.clk, .rst,
        .tri_we, .tri_slot, .tri_z0(t_z0), .tri_dz1(t_dz1), .tri_dz2(t_dz2), .tri_q0(t_q0), .tri_q1(t_q1),
        .tri_q2(t_q2), .tri_dzdx(t_dzdx), .tri_dzdy(t_dzdy), .tri_R(t_R),
        .tri_A0(t_ab[0]), .tri_B0(t_ab[1]), .tri_A1(t_ab[2]), .tri_B1(t_ab[3]), .tri_A2(t_ab[4]), .tri_B2(t_ab[5]),
        .po_en(t_po), .po_factor(t_fac), .po_units(t_units), .depth16,
        .px_valid, .px_slot, .px_e0(e0), .px_e1(e1), .px_e2(e2), .px_p(pp), .px_x(px), .px_y(py), .px_tag(tag),
        .o_valid, .o_tag, .o_z, .o_fw, .o_l1, .o_l2, .o_d24, .o_fcx, .o_fcy, .o_fcz, .o_mu1, .o_mu2);

    // expected results in issue order: {tag, slot, z, fw, l1, l2, d24, fcz, fcx, fcy, mu1, mu2}
    typedef struct { reg [7:0] tag; reg [1:0] slot; reg [W-1:0] z, fw, l1, l2, fcz, fcx, fcy, mu1, mu2; reg [23:0] d24; } exp_t;
    exp_t q[$];
    int inflight [0:3];
    int nin = 0, nout = 0, err = 0, ntri = 0;
    reg [26:0] o24_exp [0:3];

`ifdef DBG
    int ndbg = 0;
    always @(posedge clk) begin
        if (dut.w2_v && ndbg < 3) $display("dbg t=%0t w2 %h", $time, dut.w2_r);
        if (dut.r_v && ndbg < 3) begin $display("dbg t=%0t r %h t1 %h t2 %h", $time, dut.r_r, dut.d_t1[29], dut.d_t2[29]); ndbg++; end
    end
`endif
    always @(posedge clk) if (o_valid) begin
        exp_t e;
        if (q.size() == 0) begin $display("ERROR: output without input"); err++; end
        else begin
            e = q.pop_front();
            inflight[e.slot]--;
            if (o_tag !== e.tag || o_z !== e.z || o_fw !== e.fw || o_l1 !== e.l1 || o_l2 !== e.l2 || o_d24 !== e.d24 ||
                o_fcz !== e.fcz || o_fcx !== e.fcx || o_fcy !== e.fcy || o_mu1 !== e.mu1 || o_mu2 !== e.mu2) begin
                err++;
                if (err < 10) $display("mismatch px %0d: tag %h/%h z %h/%h fw %h/%h l1 %h/%h l2 %h/%h d24 %h/%h fcz %h/%h fcx %h/%h fcy %h/%h mu1 %h/%h mu2 %h/%h",
                    nout, o_tag, e.tag, o_z, e.z, o_fw, e.fw, o_l1, e.l1, o_l2, e.l2, o_d24, e.d24, o_fcz, e.fcz,
                    o_fcx, e.fcx, o_fcy, e.fcy, o_mu1, e.mu1, o_mu2, e.mu2);
            end
            nout++;
        end
    end

    initial begin
        int fd, r; string tok; reg [26:0] o24; reg [23:0] units;
        reg [W-1:0] a0, a1, a2, a3, a4, a5, a6, a7, a8, aR; reg ap; reg [12:0] ax, ay; reg [31:0] ab [0:5], ae [0:2]; reg [3:0] apx;
        exp_t e;
        inflight = '{0, 0, 0, 0};
        fd = $fopen("tools/gen/qs_vectors.hex", "r");
        if (fd == 0) begin $display("TB_FAIL tb_quad_setup: no tools/gen/qs_vectors.hex"); $finish; end
        repeat (5) @(posedge clk); rst <= 0; repeat (3) @(posedge clk);
        tri_slot <= 2'd3;
        forever begin
            r = $fscanf(fd, "%s", tok);
            if (r != 1 || tok == "E") break;
            if (tok == "D") begin                          // depth attachment bits: 1 = 16, 0 = 24
                reg [3:0] d; r = $fscanf(fd, "%h", d); depth16 <= d[0]; @(posedge clk);
                $display("tb_quad_setup: depth %0d bits", d[0] ? 16 : 24);
            end else if (tok == "T") begin
                // (inputs read into temporaries and driven with non-blocking assignments: no race with the DUT's sampling)
                r = $fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h", a0, a1, a2, a3, a4, a5, a6, a7, ap, a8, units, o24,
                            aR, ab[0], ab[1], ab[2], ab[3], ab[4], ab[5]);
                // next slot; wait until its previous pixels have left the pipe
                while (inflight[(tri_slot + 1) % 4] != 0) @(posedge clk);
                t_z0 <= a0; t_dz1 <= a1; t_dz2 <= a2; t_q0 <= a3; t_q1 <= a4; t_q2 <= a5; t_dzdx <= a6; t_dzdy <= a7;
                t_po <= ap; t_fac <= a8; t_units <= units; t_R <= aR;
                for (int k = 0; k < 6; k++) t_ab[k] <= ab[k];
                tri_we <= 1'b1; tri_slot <= tri_slot + 2'd1; ntri++;
                @(posedge clk); tri_we <= 1'b0;
            end else if (tok == "P") begin
                r = $fscanf(fd, "%h %h %h %h %h %h %h %h %h %h %h %h %h %h %h %h", ae[0], ae[1], ae[2], apx, ax, ay, e.z, e.fw, e.l1, e.l2,
                            e.d24, e.fcz, e.fcx, e.fcy, e.mu1, e.mu2);
                e.tag = 8'(nin); e.slot = tri_slot;
                q.push_back(e); inflight[tri_slot]++;
                e0 <= ae[0]; e1 <= ae[1]; e2 <= ae[2]; pp <= apx[1:0]; px <= ax; py <= ay;
                px_valid <= 1'b1; px_slot <= tri_slot; tag <= 8'(nin);
                @(posedge clk); px_valid <= 1'b0; nin++;
                if ($urandom % 4 == 0) repeat ($urandom % 5) @(posedge clk);
            end
        end
        $fclose(fd);
        repeat (60) @(posedge clk);
        $display("tb_quad_setup: %0d triangles, %0d pixels in, %0d out, %0d errors", ntri, nin, nout, err);
        if (err == 0 && nout == nin && nin > 0) $display("TB_PASS tb_quad_setup"); else $display("TB_FAIL tb_quad_setup");
        $finish;
    end
    initial begin #20ms; $display("TB_FAIL tb_quad_setup: TIMEOUT"); $finish; end
endmodule
