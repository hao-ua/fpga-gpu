// tb_b_rop — b_rop (board B's ES 2.0 output merger) against a bench model of the same rules
// (tools/b_ref.py blend_fx / dst_alpha_fac / dither8 written out again here): a 32 x 16 render target in a DDR model
// (reads in order with latency, several outstanding) in each of the four formats (RGBA8 / RGBA4444 / RGBA5551 /
// RGB565), with and without a D24S8 depth-stencil image; passes of 4 bands of 4 rows; each pass picks a format, the
// image, a clear mode (lazy fast clear, clear inside a clear-scissor box over the loaded colour, a partial clear colour
// mask, no colour clear = load), the depth / stencil clear flags and stencil clear mask, a clear colour in channel
// units, dithered or not; each band runs batches of random fragments, each batch with random state: scissor, stencil
// front / back (all functions and ops, masks), depth (all functions, mask), blending (every factor and equation,
// separate alpha, constant colour), colour mask, dither. Fragments hit the same pixels often (hazards).
// After every pass the colour image (packed pixels) and the depth-stencil image in the DDR model must equal the
// model's, bit for bit.
`timescale 1ns/1ps
`default_nettype none
module tb_b_rop;
    localparam int CW = 13, WID = 32, HGT = 16, NL = 4, NPASS = 12;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg [10:0] band_y0 = 0; reg band_start = 0, band_flush_req = 0;
    reg [1:0] fmt = 0; reg ds_en = 0;
    wire band_flush_done, band_busy, band_run, flush_busy, rmw_idle;
    reg scissor_en = 0; reg signed [CW-1:0] sc_x0 = 0, sc_y0 = 0, sc_x1 = 0, sc_y1 = 0;
    reg stencil_en = 0; reg [2:0] sfn [0:1]; reg [7:0] sref [0:1], svm [0:1], swm [0:1];
    reg [2:0] osf [0:1], odf [0:1], odp [0:1];
    reg depth_test_en = 0, depth_mask = 1; reg [2:0] depth_func = 1;
    reg blend_en = 0; reg [3:0] bsf = 1, bdf = 0, bsfa = 1, bdfa = 0; reg [2:0] bop = 0, bopa = 0; reg [51:0] bcc = 0; reg [25:0] bca = 0;
    reg [3:0] cmask = 4'hf; reg dither_en = 0;
    reg [51:0] clear_c16 = 0; reg clear_dither = 0; reg [3:0] clear_cmask = 4'hf; reg [23:0] clear_depth = 0;
    reg [7:0] clear_sten = 0, clear_smask = 8'hff; reg clear_en = 1, clear_sc_en = 0, clear_den = 1, clear_sen = 1;
    reg signed [CW-1:0] csx0 = 0, csy0 = 0, csx1 = 0, csy1 = 0;
    // fragments as pixel pairs: fx even, fpv = the pixels present, pixel p's fields at index p
    reg frag_valid = 0; reg [1:0] fpv = 0; reg signed [CW-1:0] fx = 0, fy = 0; reg [103:0] fc = 0; reg [51:0] fsa = 0; reg [47:0] fz = 0; reg ffront = 1;
    wire frag_ready;
    wire w_en, w_wren, w_wend; wire [28:0] w_addr; wire [255:0] w_data; wire [31:0] w_mask;
    reg w_rdy = 1, w_wdf_rdy = 1;
    wire lr_en; wire [28:0] lr_addr; reg lr_rdy = 1, lr_rd_valid = 0; reg [255:0] lr_rd_data = 0;
    localparam [28:0] BASE = 29'h100, DBASE = 29'h800;
    wire bpp16 = (fmt != 2'd0);
    wire [10:0] bpl = bpp16 ? 11'(WID / 16) : 11'(WID / 8);
    // the pair's cell address as b_render forms it (row base + x / 2): fx / fy are set one cycle BEFORE frag_valid, so
    // frag_addr_n (the ROP's early hazard check) names next cycle's address as b_render's does
    localparam int FAW = $clog2(((64 + 1) / 2) * NL);
    wire [FAW-1:0] fa = FAW'((int'(fy) - int'(band_y0)) * ((WID + 1) / 2) + (int'(fx) >> 1));
    b_rop #(.CW(CW), .MAXW(64), .NLINES(NL), .NLMAX(NL), .SAMPLES(1)) dut (.clk, .rst, .width(11'(WID)), .nl(4'(NL)), .band_y0, .fmt,
        .color_base(BASE), .bursts_per_line(bpl), .ds_en, .ds_base(DBASE), .ds_bpl(11'(WID / 8)),
        .band_start, .band_flush_req, .band_flush_done, .band_busy, .band_run,
        .flush_busy, .rmw_idle, .next_ok(1'b1), .next_y0(band_y0 + 11'(NL)), .pl_rdy(), .pl_rdy_y0(), .pf_clear(1'b0),
        .scissor_en, .sc_x0, .sc_y0, .sc_x1, .sc_y1,
        .stencil_en, .stencil_func(sfn[1]), .stencil_func_back(sfn[0]), .stencil_ref(sref[1]), .stencil_ref_back(sref[0]),
        .stencil_vmask(svm[1]), .stencil_vmask_back(svm[0]), .stencil_wmask(swm[1]), .stencil_wmask_back(swm[0]),
        .op_sfail(osf[1]), .op_dpfail(odf[1]), .op_dppass(odp[1]), .op_sfail_back(osf[0]), .op_dpfail_back(odf[0]),
        .op_dppass_back(odp[0]), .depth_test_en, .depth_mask, .depth_func,
        .blend_en, .blend_src_factor(bsf), .blend_dst_factor(bdf), .blend_src_factor_a(bsfa), .blend_dst_factor_a(bdfa),
        .blend_op(bop), .blend_op_a(bopa), .blend_c16(bcc), .blend_ca(bca), .color_mask(cmask), .dither_en,
        .clear_c16, .clear_dither, .clear_cmask, .clear_depth, .clear_stencil(clear_sten), .clear_color_en(clear_en),
        .clear_scissor_en(clear_sc_en), .clear_depth_en(clear_den), .clear_stencil_en(clear_sen), .clear_smask,
        .clear_sc_x0(csx0), .clear_sc_y0(csy0), .clear_sc_x1(csx1), .clear_sc_y1(csy1),
        .lr_en, .lr_addr, .lr_rdy, .lr_rd_valid, .lr_rd_data,
        .frag_valid, .frag_pv(fpv), .frag_x(fx), .frag_y(fy), .frag_c16(fc), .frag_sa(fsa), .frag_z(fz), .frag_scov(2'b11), .frag_front(ffront),
        .frag_ready, .w_en, .w_addr, .w_data, .w_mask, .w_wren, .w_wend, .w_rdy, .w_wdf_rdy,
        .pk_en(1'b0), .pk_next(1'b0), .pk_x('0), .pk_y('0), .pk_z(), .frag_addr(fa), .frag_addr_n(fa), .pk_addr('0),
        .dbg_st(), .dbg_fst(), .dbg_pst());

    // ---- DDR model: 32-bit words, writes of whole bursts, reads in order after 3..9 cycles ----
    reg [31:0] mem [0:4095];
    always @(posedge clk) if (w_en && w_rdy && w_wren && w_wdf_rdy)
        for (int k = 0; k < 8; k++) mem[(w_addr + k) & 12'hFFF] <= w_data[k*32 +: 32];
    typedef struct { reg [28:0] a; longint due; } rq_t;
    rq_t rq[$]; longint now = 0;
    always @(posedge clk) begin
        now++; lr_rd_valid <= 1'b0;
        lr_rdy <= ($urandom % 4) != 0;
        if (lr_en && lr_rdy) begin rq_t e; e.a = lr_addr; e.due = now + 3 + $urandom % 7; rq.push_back(e); end
        if (rq.size() != 0 && rq[0].due <= now) begin
            for (int k = 0; k < 8; k++) lr_rd_data[k*32 +: 32] <= mem[(rq[0].a + k) & 12'hFFF];
            lr_rd_valid <= 1'b1; void'(rq.pop_front());
        end
    end

    // ---- model ----
    int err = 0, nfr = 0, nwr = 0, nflush = 0, nconst = 0;
    // every band flush: how many used the constant pattern (an untouched, fast-cleared band)
    always @(posedge clk) if (band_flush_done) begin nflush++; if (dut.fl_const) nconst++; end
    reg [31:0] fb [0:HGT-1][0:WID-1];                     // the target as the model sees it (cell colour words)
    reg [31:0] dsi [0:HGT-1][0:WID-1];                    // the depth-stencil image {stencil, depth}
    reg [31:0] bc [0:NL-1][0:WID-1]; reg [23:0] bz [0:NL-1][0:WID-1]; reg [7:0] bs [0:NL-1][0:WID-1];
    function automatic int chn(input [1:0] f, input int c);
        case (f) 0: return 8; 1: return 4; 2: return (c == 3) ? 1 : 5; default: return (c == 1) ? 6 : (c == 3) ? 0 : 5; endcase
    endfunction
    function automatic int one_of(input int n); return 16 * ((1 << n) - 1); endfunction
    function automatic [31:0] munpack(input [31:0] v, input [1:0] f);     // stored pixel -> cell colour word
        case (f)
            0: return v;
            1: return {4'd0, v[15:12], 4'd0, v[11:8], 4'd0, v[7:4], 4'd0, v[3:0]};
            2: return {3'd0, v[15:11], 3'd0, v[10:6], 3'd0, v[5:1], 7'd0, v[0]};
            default: return {3'd0, v[15:11], 2'd0, v[10:5], 3'd0, v[4:0], 8'd0};
        endcase
    endfunction
    function automatic [31:0] mpack(input [31:0] c, input [1:0] f);       // cell colour word -> stored pixel
        case (f)
            0: return c;
            1: return {16'd0, c[27:24], c[19:16], c[11:8], c[3:0]};
            2: return {16'd0, c[28:24], c[20:16], c[12:8], c[0]};
            default: return {16'd0, c[28:24], c[21:16], c[12:8]};
        endcase
    endfunction
    function automatic [3:0] bayer(input int x, input int y);
        int t [16] = '{0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};
        return 4'(t[((y & 3) << 2) | (x & 3)]);
    endfunction
    function automatic [31:0] mdclear(input int x, input int y);
        int t = clear_dither ? bayer(x, y) : 0;
        for (int c = 0; c < 4; c++) mdclear[(3-c)*8 +: 8] = 8'((int'(clear_c16[(3-c)*13 +: 13]) + t) >> 4);
    endfunction
    function automatic bit mcmp(input [2:0] f, input int a, input int b);
        case (f) 0: return 0; 1: return a < b; 2: return a == b; 3: return a <= b;
                 4: return a > b; 5: return a != b; 6: return a >= b; default: return 1; endcase
    endfunction
    function automatic [7:0] msop(input [2:0] o, input [7:0] s, input [7:0] r);
        case (o) 0: return s; 1: return 0; 2: return r; 3: return (s == 8'hff) ? s : s + 1;
                 4: return (s == 0) ? s : s - 1; 5: return ~s; 6: return s + 1; default: return s - 1; endcase
    endfunction
    function automatic int mfac(input [3:0] code, input int s, input int d16, input int sa, input int da16,
                                input int cc, input int ca, input bit dst, input int one);
        case (code)
            0: return 0; 1: return one; 2: return s; 3: return one - s; 4: return d16; 5: return one - d16;
            6: return sa; 7: return one - sa; 8: return da16; 9: return one - da16; 10: return cc; 11: return one - cc;
            12: return ca; 13: return one - ca; 14: return dst ? 0 : ((sa < one - da16) ? sa : one - da16);
            default: return 0;
        endcase
    endfunction
    function automatic [3:0] mafac(input [3:0] code);
        case (code) 2: return 6; 3: return 7; 4: return 8; 5: return 9; 10: return 12; 11: return 13; 14: return 1;
                    default: return code; endcase
    endfunction
    function automatic int mda(input int da, input int na, input int nc);
        if (na == 0) return one_of(nc);
        if (na == nc) return 16 * da;
        return da ? one_of(nc) : 0;
    endfunction
    function automatic [31:0] mblend(input [51:0] src, input [25:0] sa2, input [31:0] dst, input int x, input int y);
        int t = dither_en ? bayer(x, y) : 0;
        for (int c = 0; c < 4; c++) begin
            int s, d, v, fs, fd, p1, p2, xx, n, one, sa, ca, da; bit [2:0] o;
            n = chn(fmt, c); one = one_of(n);
            s = src[(3-c)*13 +: 13]; d = dst[(3-c)*8 +: 8];
            o = (c == 3) ? bopa : bop;
            sa = (c == 3) ? s : (c == 1) ? sa2[25:13] : sa2[12:0];
            ca = (c == 3) ? bcc[12:0] : (c == 1) ? bca[25:13] : bca[12:0];
            da = mda(dst[7:0], chn(fmt, 3), n);
            if (!blend_en) v = s;
            else if (o == 3) v = (s < 16*d) ? s : 16*d;
            else if (o == 4) v = (s > 16*d) ? s : 16*d;
            else begin
                fs = mfac(c == 3 ? mafac(bsfa) : bsf, s, 16*d, sa, da, bcc[(3-c)*13 +: 13], ca, 0, one);
                fd = mfac(c == 3 ? mafac(bdfa) : bdf, s, 16*d, sa, da, bcc[(3-c)*13 +: 13], ca, 1, one);
                p1 = s * fs; p2 = 16 * d * fd;
                xx = (o == 0) ? p1 + p2 : (o == 1) ? p1 - p2 : p2 - p1;
                v = (xx <= 0) ? 0 : (xx + one / 2) / one;
                if (v > one) v = one;
            end
            mblend[(3-c)*8 +: 8] = cmask[3-c] ? 8'((v + t) >> 4) : 8'(d);
        end
    endfunction
    task automatic model_band_start(input int y0);
        for (int l = 0; l < NL; l++) for (int x = 0; x < WID; x++) begin
            reg [31:0] cm32 = {{8{clear_cmask[3]}}, {8{clear_cmask[2]}}, {8{clear_cmask[1]}}, {8{clear_cmask[0]}}};
            bit inbox = !clear_sc_en || (x >= csx0 && x < csx1 && y0 + l >= csy0 && y0 + l < csy1);
            reg [7:0] s0 = dsi[y0 + l][x][31:24];
            bc[l][x] = (clear_en && inbox) ? ((mdclear(x, y0 + l) & cm32) | (fb[y0 + l][x] & ~cm32)) : fb[y0 + l][x];
            bz[l][x] = ds_en ? ((clear_den && inbox) ? clear_depth : dsi[y0 + l][x][23:0]) : clear_depth;
            bs[l][x] = ds_en ? ((clear_sen && inbox) ? ((clear_sten & clear_smask) | (s0 & ~clear_smask)) : s0) : clear_sten;
        end
    endtask
    task automatic model_frag(input int x, input int y, input int y0, input [51:0] c, input [25:0] sa2, input [23:0] z, input bit front);
        int l = y - y0; bit sp, dp; reg [7:0] st, nst; int f = front ? 1 : 0;
        if (scissor_en && !(x >= sc_x0 && x < sc_x1 && y >= sc_y0 && y < sc_y1)) return;
        st = bs[l][x];
        sp = !stencil_en || mcmp(sfn[f], sref[f] & svm[f], st & svm[f]);
        dp = !depth_test_en || mcmp(depth_func, z, bz[l][x]);
        if (stencil_en) begin
            nst = msop(sp ? (dp ? odp[f] : odf[f]) : osf[f], st, sref[f]);
            bs[l][x] = (nst & swm[f]) | (st & ~swm[f]);
        end
        if (sp && dp) begin
            if (depth_mask) bz[l][x] = z;
            bc[l][x] = mblend(c, sa2, bc[l][x], x, y);
        end
    endtask
`ifdef DBG
    int nwd = 0;
    always @(posedge clk) if (dut.v6 && nwd < 12) begin
        $display("rtl  write %0d addr %0d colour %h (blend in: src %h)", nwd, dut.s6_addr, dut.blend_final_s[0], dut.s0_c16[0]);
        nwd++;
    end
`endif

    function automatic [12:0] rcn(input int n);           // a channel value in its units
        int k = $urandom % 8; int one = one_of(n);
        if (k == 0) return 13'd0;
        if (k == 1) return 13'(one);
        return 13'($urandom % (one + 1));
    endfunction
    function automatic [51:0] rcol();                     // a colour in the format's channel units
        return {rcn(chn(fmt, 0)), rcn(chn(fmt, 1)), rcn(chn(fmt, 2)), rcn(chn(fmt, 3))};
    endfunction
    function automatic [25:0] ralpha();                   // an alpha in G's and in R / B's units
        return {rcn(chn(fmt, 1)), rcn(chn(fmt, 0))};
    endfunction
    function automatic [31:0] rpix();                     // a stored pixel of the format
        return bpp16 ? 32'($urandom % 65536) : $urandom;
    endfunction
    task automatic rand_state();
        scissor_en = ($urandom % 4 == 0); sc_x0 = $urandom % 16; sc_x1 = sc_x0 + $urandom % 24; sc_y0 = $urandom % 12;
        sc_y1 = sc_y0 + $urandom % 10;
        stencil_en = ($urandom % 3 == 0);
        for (int f = 0; f < 2; f++) begin
            sfn[f] = $urandom; sref[f] = $urandom % 4; svm[f] = ($urandom % 2) ? 8'hff : $urandom; swm[f] = ($urandom % 2) ? 8'hff : $urandom;
            osf[f] = $urandom; odf[f] = $urandom; odp[f] = $urandom;
        end
        depth_test_en = ($urandom % 3 != 0); depth_mask = ($urandom % 4 != 0); depth_func = $urandom;
        blend_en = ($urandom % 3 != 0);
        bsf = $urandom % 15; bdf = $urandom % 14; bsfa = $urandom % 15; bdfa = $urandom % 14;
        bop = $urandom % 5; bopa = $urandom % 5;
        bcc = rcol(); bca = ralpha();
        cmask = ($urandom % 3 == 0) ? 4'($urandom) : 4'hf;
        dither_en = $urandom % 2;
`ifdef SIMPLE
        blend_en = 0; stencil_en = 0; depth_test_en = 0; scissor_en = 0; cmask = 4'hf; dither_en = 0;
`endif
`ifdef ONLYBLEND
        stencil_en = 0; depth_test_en = 0; scissor_en = 0; cmask = 4'hf; dither_en = 0;
`endif
`ifdef ONLYDEPTH
        blend_en = 0; stencil_en = 0; scissor_en = 0; cmask = 4'hf; dither_en = 0;
`endif
    endtask
    // the stored pixel of (x, y) in the DDR model
    function automatic [31:0] mem_px(input int x, input int y);
        longint wa;
        if (bpp16) begin wa = BASE + y * bpl * 8 + x / 2; return (x & 1) ? {16'd0, mem[wa][31:16]} : {16'd0, mem[wa][15:0]}; end
        wa = BASE + y * bpl * 8 + x; return mem[wa];
    endfunction

    initial begin
        for (int f = 0; f < 2; f++) begin sfn[f] = 7; sref[f] = 0; svm[f] = 8'hff; swm[f] = 8'hff; osf[f] = 0; odf[f] = 0; odp[f] = 0; end
        repeat (5) @(posedge clk); rst <= 0; repeat (3) @(posedge clk);
        for (int p = 0; p < NPASS; p++) begin
            // passes 0 / 1 / 4 / 7 (RGBA8, RGBA4444, RGBA8 + image, RGB565) use the lazy fast clear; every pass but 2 / 5 / 8 /
            // 11 leaves its odd bands without fragments (fast + untouched = the flush's constant pattern)
            int mode = (p == 0 || p == 1 || p == 4 || p == 7) ? 0 : $urandom % 4;
            bit empty_odd = (p % 3 != 2);
            @(negedge clk);
            fmt = 2'(p % 4); ds_en = (p >= 4) ? (p % 2) : 1'b0;
            if (p == 8) ds_en = 1;
            #1;                                                   // bpl / bpp16 follow the format
            // the target and the depth-stencil image start with known contents in both the DDR model and the model
            for (int y = 0; y < HGT; y++) for (int x = 0; x < WID; x++) begin
                reg [31:0] px = rpix();
                fb[y][x] = munpack(px, fmt);
                if (bpp16) begin
                    if (x & 1) mem[BASE + y * bpl * 8 + x / 2][31:16] = px[15:0]; else mem[BASE + y * bpl * 8 + x / 2][15:0] = px[15:0];
                end else mem[BASE + y * bpl * 8 + x] = px;
                dsi[y][x] = $urandom; mem[DBASE + y * WID + x] = dsi[y][x];
            end
            clear_en = (mode != 3); clear_sc_en = (mode == 1); clear_cmask = (mode == 2) ? 4'($urandom) : 4'hf;
            clear_den = ($urandom % 3 != 0); clear_sen = ($urandom % 3 != 0); clear_smask = ($urandom % 2) ? 8'hff : 8'($urandom);
            if (p == 4) begin clear_den = 1; clear_sen = 1; clear_smask = 8'hff; end      // the fast path with an image
            csx0 = $urandom % 16; csx1 = csx0 + 1 + $urandom % 20; csy0 = $urandom % 12; csy1 = csy0 + 1 + $urandom % 8;
            clear_c16 = rcol(); clear_dither = $urandom % 2;
            clear_depth = ($urandom % 2) ? 24'hFFFFFF : $urandom; clear_sten = $urandom % 4;
            for (int b = 0; b < HGT / NL; b++) begin
                int y0 = b * NL;
                @(negedge clk); band_y0 = y0; band_start = 1; @(negedge clk); band_start = 0;
                model_band_start(y0);
                for (int bt = 0; bt < 3; bt++) begin
                    // a batch: state stays while fragments are in the pipe
                    while (!rmw_idle) @(negedge clk);
                    rand_state();
`ifdef NOFRAG
                    for (int n = 0; n < 0; n++) begin
`else
                    for (int n = 0; n < ((empty_odd && b[0]) ? 0 : 30); n++) begin
`endif
                        // a pixel pair (a quad row): even x, often the same pixels; one, the other or both pixels present
                        int x = (($urandom % 3 == 0) ? $urandom % 4 : $urandom % WID) & ~1;
                        int y = y0 + $urandom % NL;
                        reg [1:0] pv = 2'($urandom % 3) + 2'd1;
                        reg [51:0] c0 = rcol(), c1 = rcol(); reg [25:0] sa0 = ralpha(), sa1 = ralpha();
                        reg [23:0] z0 = ($urandom % 3 == 0) ? 24'h800000 : 24'($urandom);
                        reg [23:0] z1 = ($urandom % 3 == 0) ? 24'h800000 : 24'($urandom);
                        bit front = $urandom % 2;
                        fpv = pv; fx = x; fy = y; fc = {c1, c0}; fsa = {sa1, sa0}; fz = {z1, z0}; ffront = front;
                        @(negedge clk); frag_valid = 1;                  // the address a cycle ahead of the fragment (frag_addr_n)
                        #1;                                              // let frag_ready settle on the new inputs
                        while (!frag_ready) begin @(negedge clk); #1; end
                        @(negedge clk); frag_valid = 0;                  // taken at the posedge in between
                        if (pv[0]) begin model_frag(x, y, y0, c0, sa0, z0, front); nfr++; end
                        if (pv[1]) begin model_frag(x + 1, y, y0, c1, sa1, z1, front); nfr++; end
                        if ($urandom % 3 == 0) repeat ($urandom % 3) @(negedge clk);
                    end
                end
                band_flush_req = 1;
                while (!band_flush_done) @(negedge clk);
                band_flush_req = 0;
                for (int l = 0; l < NL; l++) for (int x = 0; x < WID; x++) begin
                    fb[y0 + l][x] = bc[l][x];
                    if (ds_en) dsi[y0 + l][x] = {bs[l][x], bz[l][x]};
                end
            end
            while (flush_busy) @(negedge clk);
            repeat (4) @(negedge clk);
            for (int y = 0; y < HGT; y++) for (int x = 0; x < WID; x++) begin
                if (mem_px(x, y) !== mpack(fb[y][x], fmt)) begin
                    err++;
                    if (err < 10) $display("  pass %0d (fmt %0d ds %0d clear mode %0d) pixel (%0d, %0d): %h, expected %h", p, fmt, ds_en,
                                           mode, x, y, mem_px(x, y), mpack(fb[y][x], fmt));
                end
                if (ds_en && mem[DBASE + y * WID + x] !== dsi[y][x]) begin
                    err++;
                    if (err < 10) $display("  pass %0d (fmt %0d ds %0d clear mode %0d) depth-stencil (%0d, %0d): %h, expected %h", p, fmt,
                                           ds_en, mode, x, y, mem[DBASE + y * WID + x], dsi[y][x]);
                end
            end
        end
        $display("tb_b_rop: %0d passes, %0d fragments, %0d band flushes (%0d with the constant pattern), errors = %0d",
                 NPASS, nfr, nflush, nconst, err);
        if (nconst < 4) begin err++; $display("  the constant-pattern flush was not exercised (%0d bands)", nconst); end
        if (err == 0) $display("TB_PASS tb_b_rop"); else $display("TB_FAIL tb_b_rop");
        $finish;
    end
    initial begin #40ms; $display("TB_FAIL tb_b_rop: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
