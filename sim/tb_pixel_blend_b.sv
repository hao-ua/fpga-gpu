// tb_pixel_blend_b — pixel_blend_b alone: for each render-target format (RGBA8 / RGBA4444 / RGBA5551 / RGB565) random
// sources (in channel units), destinations (stored n-bit values), constants, factors (all codes), equations, masks and
// dither thresholds, one per cycle; the result 6 cycles later must equal the model (as tb_b_rop's mblend,
// tools/b_ref.py blend_fx / dst_alpha_fac / dither8).
`timescale 1ns/1ps
`default_nettype none
module tb_pixel_blend_b;
    reg clk = 0; always #5 clk = ~clk;
    reg [1:0] fmt = 0;
    reg [51:0] src = 0, cc = 0; reg [25:0] sa2 = 0, ca2 = 0; reg [31:0] dst = 0; reg en = 0;
    reg [3:0] fs = 0, fd = 0, fsa = 0, fda = 0, mask = 0, t = 0;
    reg [2:0] op = 0, opa = 0;
    wire [31:0] out;
    pixel_blend_b dut (.clk, .fmt, .src_c16(src), .src_alpha(sa2), .dst_color(dst), .blend_c16(cc), .blend_ca(ca2),
        .enable(en), .src_factor(fs), .dst_factor(fd), .src_factor_a(fsa), .dst_factor_a(fda), .op, .op_a(opa), .mask,
        .dith(t), .final_color(out));
    function automatic int chn(input [1:0] f, input int c);
        case (f) 0: return 8; 1: return 4; 2: return (c == 3) ? 1 : 5; default: return (c == 1) ? 6 : (c == 3) ? 0 : 5; endcase
    endfunction
    function automatic int one_of(input int n); return 16 * ((1 << n) - 1); endfunction
    function automatic int mfac(input [3:0] code, input int s, input int d16, input int sa, input int da16,
                                input int c, input int ca, input bit isdst, input int one);
        case (code)
            0: return 0; 1: return one; 2: return s; 3: return one - s; 4: return d16; 5: return one - d16;
            6: return sa; 7: return one - sa; 8: return da16; 9: return one - da16; 10: return c; 11: return one - c;
            12: return ca; 13: return one - ca; 14: return isdst ? 0 : ((sa < one - da16) ? sa : one - da16);
            default: return 0;
        endcase
    endfunction
    function automatic [3:0] mafac(input [3:0] code);
        case (code) 2: return 6; 3: return 7; 4: return 8; 5: return 9; 10: return 12; 11: return 13; 14: return 1;
                    default: return code; endcase
    endfunction
    function automatic int mda(input int da, input int na, input int nc);     // destination alpha as a factor of channel c
        if (na == 0) return one_of(nc);
        if (na == nc) return 16 * da;
        return da ? one_of(nc) : 0;
    endfunction
    function automatic [31:0] model(input [1:0] f, input [51:0] sv, input [25:0] sav, input [31:0] dv, input [51:0] cv,
                                    input [25:0] cav, input bit e, input [3:0] f1, f2, f3, f4, input [2:0] o1, o2,
                                    input [3:0] m, input [3:0] th);
        for (int c = 0; c < 4; c++) begin
            int s, d, v, a, b, p1, p2, xx, one, n, sa, ca, da; bit [2:0] o;
            n = chn(f, c); one = one_of(n);
            s = sv[(3-c)*13 +: 13]; d = dv[(3-c)*8 +: 8]; o = (c == 3) ? o2 : o1;
            sa = (c == 3) ? s : (c == 1) ? sav[25:13] : sav[12:0];
            ca = (c == 3) ? cv[12:0] : (c == 1) ? cav[25:13] : cav[12:0];
            da = mda(dv[7:0], chn(f, 3), n);
            if (!e) v = s;
            else if (o == 3) v = (s < 16*d) ? s : 16*d;
            else if (o == 4) v = (s > 16*d) ? s : 16*d;
            else begin
                a = mfac(c == 3 ? mafac(f3) : f1, s, 16*d, sa, da, cv[(3-c)*13 +: 13], ca, 0, one);
                b = mfac(c == 3 ? mafac(f4) : f2, s, 16*d, sa, da, cv[(3-c)*13 +: 13], ca, 1, one);
                p1 = s * a; p2 = 16 * d * b;
                xx = (o == 0) ? p1 + p2 : (o == 1) ? p1 - p2 : p2 - p1;
                v = (xx <= 0) ? 0 : (xx + one / 2) / one;
                if (v > one) v = one;
            end
            model[(3-c)*8 +: 8] = m[3-c] ? 8'((v + th) >> 4) : 8'(d);
        end
    endfunction
    function automatic [12:0] rcn(input int n);           // a channel value in its units
        int k = $urandom % 6; int one = one_of(n);
        if (k == 0) return 0; if (k == 1) return 13'(one);
        return 13'($urandom % (one + 1));
    endfunction
    function automatic [7:0] rdn(input int n);            // a stored n-bit value
        return (n == 0) ? 8'd0 : 8'($urandom % (1 << n));
    endfunction
    // the expectation travels beside the DUT's 6 stages (valid-tagged: the drains between formats carry none)
    reg exp_v = 0; reg [31:0] exp_c; string exp_s;
    reg vd [0:5]; reg [31:0] ed [0:5]; string sd [0:5];
    int n = 0, err = 0;
    always @(posedge clk) begin
        vd[0] <= exp_v; ed[0] <= exp_c; sd[0] <= exp_s;
        for (int k = 1; k < 6; k++) begin vd[k] <= vd[k-1]; ed[k] <= ed[k-1]; sd[k] <= sd[k-1]; end
        if (vd[5]) begin                                      // out of the input sampled 6 edges ago
            if (out !== ed[5]) begin err++; if (err < 8) $display("  #%0d: %h, expected %h  (%s)", n, out, ed[5], sd[5]); end
            n++;
        end
    end
    initial begin
        for (int k = 0; k < 6; k++) vd[k] = 0;
        repeat (2) @(posedge clk);
        for (int f = 0; f < 4; f++) begin
            // the format is constant through a frame: let the pipeline drain between formats
            @(negedge clk); fmt = 2'(f); exp_v = 0; repeat (8) @(posedge clk);
            for (int i = 0; i < 20000; i++) begin
                @(negedge clk);
                src = {rcn(chn(fmt, 0)), rcn(chn(fmt, 1)), rcn(chn(fmt, 2)), rcn(chn(fmt, 3))};
                sa2 = {rcn(chn(fmt, 1)), rcn(chn(fmt, 0))};
                cc = {rcn(chn(fmt, 0)), rcn(chn(fmt, 1)), rcn(chn(fmt, 2)), rcn(chn(fmt, 3))};
                ca2 = {rcn(chn(fmt, 1)), rcn(chn(fmt, 0))};
                dst = {rdn(chn(fmt, 0)), rdn(chn(fmt, 1)), rdn(chn(fmt, 2)), rdn(chn(fmt, 3))};
                en = ($urandom % 5 != 0); fs = $urandom % 15; fd = $urandom % 15; fsa = $urandom % 15; fda = $urandom % 15;
                op = $urandom % 5; opa = $urandom % 5; mask = ($urandom % 2) ? 4'hf : $urandom; t = $urandom;
                exp_v = 1; exp_c = model(fmt, src, sa2, dst, cc, ca2, en, fs, fd, fsa, fda, op, opa, mask, t);
                exp_s = $sformatf("fmt %0d src %h sa %h dst %h cc %h ca %h en %b f %0d %0d %0d %0d op %0d %0d m %h t %0d",
                                  fmt, src, sa2, dst, cc, ca2, en, fs, fd, fsa, fda, op, opa, mask, t);
            end
            @(negedge clk); exp_v = 0;
            repeat (8) @(posedge clk);
        end
        repeat (8) @(posedge clk);
        $display("tb_pixel_blend_b: %0d results, errors = %0d", n, err);
        if (err == 0 && n > 79000) $display("TB_PASS tb_pixel_blend_b"); else $display("TB_FAIL tb_pixel_blend_b");
        $finish;
    end
endmodule
`default_nettype wire
