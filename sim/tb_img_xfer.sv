`timescale 1ns/1ps
// tb_img_xfer — unit bench of img_xfer (board B READ_IMAGE / FILL_IMAGE / COPY_IMAGE engine) behind
// mem_arb with the MIG model (random back-pressure). Every case presets random bytes around the source and
// destination rows, runs one command and compares every preset byte with an independent model:
//   COPY  pixel (i, j) <- source (u, v), u = floor((2i + 1) sw / 2dw), v likewise, mirrors u = sw - 1 - u,
//         v = sh - 1 - v; source colour key per channel (inclusive range, alpha ignored); conversions through RGBA8
//         with real-number rounding: c8 = round(c5 * 255 / 31), c5 = round(c8 * 31 / 255); the source is
//         snapshotted before any write, so overlapping 1:1 copies must match the model exactly;
//   FILL  the value converted once, channel mask per byte (RGBA8) or per field (RGB565, incl. read-modify-write);
//   READ  pixels first .. first + count - 1 of the rectangle, packed transfer bytes, compared with the o_* words
//         (random o_ready back-pressure).
// Directed: all 65536 RGB565 values -> RGBA8 (COPY) and -> RGBA8888 (READ); 256 grey levels RGBA8 -> RGB565;
// same-format copies bit-exact; a 4096-pixel row (row buffer limit), stretch 4096 -> 1000 and 37 -> 4096.
// COPY's swizzle (destination channel c <- source channel sel[2c +: 2] on the RGBA8 values), alpha = 255 (RGBA8
// destinations) and the wrapped tile (destination (i, j) <- tile ((px + i) mod sw, (py + j) mod sh)) are in the model;
// random burst-path copies (one format, 1:1, no key / horizontal mirror / swizzle / tile: whole bursts) at every 2-byte
// alignment of source and destination, overlapping inside one surface in every direction, and the widest rows. The count
// of burst-path commands is reported (dut.bm at the start).
`default_nettype none
module tb_img_xfer;
    reg clk = 0, rst = 1;
    always #5.0 clk = ~clk;
    int errors = 0, nprint = 0;
    task automatic report(input string s);
        errors++;
        if (nprint < 30) begin $display("%t FAIL %s", $time, s); nprint++; end
    endtask

    // ---------------- DUT + arbiter + MIG model ----------------
    reg         cmd_valid = 0; wire cmd_ready;
    reg  [1:0]  cmd_op, cmd_s_fmt, cmd_d_fmt;
    reg  [29:0] cmd_s_row0, cmd_d_row0; reg [15:0] cmd_s_pitch, cmd_d_pitch;
    reg  [12:0] cmd_s_x, cmd_s_y, cmd_s_w, cmd_s_h, cmd_d_x, cmd_d_y, cmd_d_w, cmd_d_h;
    reg  [12:0] cmd_hq, cmd_hu0, cmd_vq, cmd_vu0; reg [13:0] cmd_hr2, cmd_ha0, cmd_vr2, cmd_va0;
    reg         cmd_mir_x, cmd_mir_y, cmd_bottom_up, cmd_key, cmd_rmw;
    reg  [31:0] cmd_key_lo, cmd_key_hi, cmd_value; reg [3:0] cmd_mask; reg [7:0] cmd_tfmt;
    reg  [12:0] cmd_j0, cmd_i0; reg [23:0] cmd_npix;
    reg         cmd_alpha1, cmd_swz, cmd_wrap; reg [7:0] cmd_swz_sel; reg [12:0] cmd_px, cmd_py;
    wire        busy, o_valid; wire [31:0] o_data; reg o_ready = 1; wire [31:0] c_ops, c_pixels;
    wire        r_en, r_rdy, r_valid; wire [28:0] r_addr; wire [255:0] rd_data;
    wire        w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [28:0] w_addr; wire [255:0] w_data; wire [31:0] w_mask;
    img_xfer dut (.clk(clk), .rst(rst), .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_op(cmd_op),
        .cmd_s_row0(cmd_s_row0), .cmd_s_pitch(cmd_s_pitch), .cmd_s_fmt(cmd_s_fmt), .cmd_s_x(cmd_s_x), .cmd_s_y(cmd_s_y),
        .cmd_s_w(cmd_s_w), .cmd_s_h(cmd_s_h), .cmd_d_row0(cmd_d_row0), .cmd_d_pitch(cmd_d_pitch), .cmd_d_fmt(cmd_d_fmt),
        .cmd_d_x(cmd_d_x), .cmd_d_y(cmd_d_y), .cmd_d_w(cmd_d_w), .cmd_d_h(cmd_d_h),
        .cmd_hq(cmd_hq), .cmd_hu0(cmd_hu0), .cmd_vq(cmd_vq), .cmd_vu0(cmd_vu0),
        .cmd_hr2(cmd_hr2), .cmd_ha0(cmd_ha0), .cmd_vr2(cmd_vr2), .cmd_va0(cmd_va0),
        .cmd_mir_x(cmd_mir_x), .cmd_mir_y(cmd_mir_y), .cmd_bottom_up(cmd_bottom_up), .cmd_key(cmd_key),
        .cmd_key_lo(cmd_key_lo), .cmd_key_hi(cmd_key_hi), .cmd_value(cmd_value), .cmd_mask(cmd_mask), .cmd_rmw(cmd_rmw),
        .cmd_alpha1(cmd_alpha1), .cmd_swz(cmd_swz), .cmd_swz_sel(cmd_swz_sel), .cmd_wrap(cmd_wrap), .cmd_px(cmd_px), .cmd_py(cmd_py),
        .cmd_tfmt(cmd_tfmt), .cmd_j0(cmd_j0), .cmd_i0(cmd_i0), .cmd_npix(cmd_npix), .busy(busy),
        .o_valid(o_valid), .o_data(o_data), .o_ready(o_ready), .c_ops(c_ops), .c_pixels(c_pixels),
        .r_en(r_en), .r_addr(r_addr), .r_rdy(r_rdy), .r_valid(r_valid), .rd_data(rd_data),
        .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren), .w_wend(w_wend),
        .w_rdy(w_rdy), .w_wdf_rdy(w_wdf_rdy));
    wire        app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [2:0]  app_cmd; wire [28:0] app_addr; wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask;
    mem_arb #(.NR(1), .NW(1)) u_arb (.clk(clk), .rst(rst), .r_en(r_en), .r_addr(r_addr), .r_rdy(r_rdy), .r_valid(r_valid),
        .rd_data(rd_data), .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren),
        .w_wend(w_wend), .w_rdy(w_rdy), .w_wdf_rdy(w_wdf_rdy),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data));
    int mig_err; longint n_wr, n_rd;
    mig_app_model #(.RD_LAT(20), .RDY_PCT(70), .WDF_PCT(75), .SEED(23)) u_mig (
        .ui_clk(clk), .rst(rst), .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy),
        .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid),
        .errors(mig_err), .n_wr(n_wr), .n_rd(n_rd));

    // ---------------- reference model ----------------
    bit [7:0] ref_mem [longint];
    longint   touched[$];                            // every preset address (compared after each case)
    int unsigned seed = 32'h0BAD_5EED;
    function automatic int unsigned urand(input int unsigned n);
        seed = seed * 1664525 + 1013904223;
        return (n == 0) ? 0 : ((seed >> 8) % n);
    endfunction
    function automatic int bpp(input int f); return (f == 0) ? 4 : (f == 1) ? 2 : 1; endfunction
    function automatic int rnd(input real f); return $rtoi(f + 0.5); endfunction
    function automatic int up5(input int c); return rnd(c * 255.0 / 31.0); endfunction
    function automatic int up6(input int c); return rnd(c * 255.0 / 63.0); endfunction
    function automatic int dn(input int v, input int bits); return rnd(v * ((1 << bits) - 1) / 255.0); endfunction
    task automatic preset(input longint a, input bit [7:0] v);
        bit [255:0] bur; longint bi;
        bi = a >> 5;
        bur = u_mig.mem.exists(bi) ? u_mig.mem[bi] : 256'h0;
        bur[(a & 31) * 8 +: 8] = v;
        u_mig.mem[bi] = bur;
        if (!ref_mem.exists(a)) touched.push_back(a);
        ref_mem[a] = v;
    endtask
    function automatic bit [31:0] get_px(input longint a, input int f);
        bit [31:0] v; v = 0;
        for (int q = 0; q < bpp(f); q++) v[q*8 +: 8] = ref_mem[a + q];
        return v;
    endfunction
    task automatic put_px(input longint a, input int f, input bit [31:0] v);
        for (int q = 0; q < bpp(f); q++) ref_mem[a + q] = v[q*8 +: 8];
    endtask
    // stored value -> r, g, b, a (exact expansion)
    task automatic dec(input int f, input bit [31:0] v, output int r, output int g, output int b, output int a);
        if (f == 1) begin r = up5(v[15:11]); g = up6(v[10:5]); b = up5(v[4:0]); a = 255; end
        else begin r = v[31:24]; g = v[23:16]; b = v[15:8]; a = v[7:0]; end
    endtask
    function automatic bit [31:0] enc(input int f, input int r, input int g, input int b, input int a);
        if (f == 1) return {16'h0, 5'(dn(r, 5)), 6'(dn(g, 6)), 5'(dn(b, 5))};
        return {8'(r), 8'(g), 8'(b), 8'(a)};
    endfunction
    // transfer bytes -> r, g, b, a
    task automatic tdec(input int tf, input bit [31:0] v, output int r, output int g, output int b, output int a);
        case (tf)
            1: begin r = v[7:0]; g = v[15:8]; b = v[23:16]; a = v[31:24]; end
            2: begin b = v[7:0]; g = v[15:8]; r = v[23:16]; a = v[31:24]; end
            3: begin b = v[7:0]; g = v[15:8]; r = v[23:16]; a = 255; end
            10: begin r = v[7:0]; g = v[15:8]; b = v[23:16]; a = 255; end
            11: begin b = v[7:0]; g = v[15:8]; r = v[23:16]; a = 255; end
            default: begin r = up5(v[15:11]); g = up6(v[10:5]); b = up5(v[4:0]); a = 255; end
        endcase
    endtask
    function automatic int tbpp_of(input int tf, input int f);
        case (tf) 0: return bpp(f); 4: return 2; 10, 11: return 3; default: return 4; endcase
    endfunction

    // ---------------- one command ----------------
    typedef struct {
        int op, sf, df; longint s0, d0; int sp, dp; int sx, sy, sw, sh, dx, dy, dw, dh;
        bit mx, my, key; bit [31:0] klo, khi, value; int mask; int tf; int first, npix;
        bit alpha1, swz, wrap; bit [7:0] sel; int px, py;
    } cmd_t;
    int unsigned n_cmd = 0, n_bm = 0; longint n_px = 0;
    always @(posedge clk) if (cmd_valid && cmd_ready && dut.bm_go) n_bm++;
    bit [7:0] rd_got[$];
    always @(posedge clk) if (o_valid && o_ready) begin
        rd_got.push_back(o_data[7:0]); rd_got.push_back(o_data[15:8]); rd_got.push_back(o_data[23:16]); rd_got.push_back(o_data[31:24]);
    end
    int rdy_pct = 100;
    always @(posedge clk) o_ready <= (urand(100) < rdy_pct);

    task automatic preset_rows(input longint row0, input int pitch, input int y0, input int nrows, input int xb0, input int xb1);
        longint lo, hi;
        lo = row0 + (y0 - 1) * pitch + xb0 - 64; hi = row0 + (y0 + nrows) * pitch + xb1 + 64;
        if (lo < 0) lo = 0;
        for (longint a = lo; a < hi; a++) if (!ref_mem.exists(a)) preset(a, 8'(urand(256)));
    endtask

    task automatic run(input cmd_t c);
        int sb, db, tb; bit same, rmw, bup; bit [7:0] exp_rd[$];
        bit [31:0] snap [][];
        sb = bpp(c.sf); db = bpp(c.df);
        // background
        if (c.op == 0 || c.op == 2) preset_rows(c.s0, c.sp, c.sy, c.sh, c.sx * sb, (c.sx + c.sw) * sb);
        if (c.op != 2)              preset_rows(c.d0, c.dp, c.dy, c.dh, c.dx * db, (c.dx + c.dw) * db);
        same = (c.op == 0) && (c.s0 == c.d0) && (c.sp == c.dp) && (c.sf == c.df);
        bup = same && c.sw == c.dw && c.sh == c.dh && !c.my && !c.wrap && c.dy > c.sy;
        rmw = (c.op == 1) && (c.df == 1) && ((c.mask & 7) != 0) && ((c.mask & 7) != 7);
        // ---- model ----
        if (c.op == 0) begin
            snap = new[c.sh];
            foreach (snap[v]) begin
                snap[v] = new[c.sw];
                foreach (snap[v][u]) snap[v][u] = get_px(c.s0 + (c.sy + v) * c.sp + (c.sx + u) * sb, c.sf);
            end
            for (int j = 0; j < c.dh; j++) for (int i = 0; i < c.dw; i++) begin
                int u, v, r, g, b, a; bit [31:0] sv, dv; bit tr;
                u = ((2 * i + 1) * c.sw) / (2 * c.dw); v = ((2 * j + 1) * c.sh) / (2 * c.dh);
                if (c.mx) u = c.sw - 1 - u;
                if (c.my) v = c.sh - 1 - v;
                if (c.wrap) begin u = (c.px + i) % c.sw; v = (c.py + j) % c.sh; end
                sv = snap[v][u];
                tr = 0;
                if (c.key) begin
                    if (c.sf == 0) tr = sv[31:24] >= c.klo[31:24] && sv[31:24] <= c.khi[31:24] && sv[23:16] >= c.klo[23:16] &&
                                        sv[23:16] <= c.khi[23:16] && sv[15:8] >= c.klo[15:8] && sv[15:8] <= c.khi[15:8];
                    else if (c.sf == 1) tr = sv[15:11] >= c.klo[15:11] && sv[15:11] <= c.khi[15:11] && sv[10:5] >= c.klo[10:5] &&
                                        sv[10:5] <= c.khi[10:5] && sv[4:0] >= c.klo[4:0] && sv[4:0] <= c.khi[4:0];
                    else tr = sv[7:0] >= c.klo[7:0] && sv[7:0] <= c.khi[7:0];
                end
                if (!tr) begin
                    if (c.sf == 2) dv = sv;
                    else begin
                        int ch[4], o[4];
                        dec(c.sf, sv, r, g, b, a);
                        ch = '{r, g, b, a};
                        for (int q = 0; q < 4; q++) o[q] = c.swz ? ch[c.sel[2*q +: 2]] : ch[q];
                        if (c.alpha1) o[3] = 255;
                        dv = enc(c.df, o[0], o[1], o[2], o[3]);
                    end
                    put_px(c.d0 + (c.dy + j) * c.dp + (c.dx + i) * db, c.df, dv);
                end
            end
        end else if (c.op == 1) begin
            int r, g, b, a; bit [31:0] fv;
            if (c.tf == 0) fv = c.value;
            else begin tdec(c.tf, c.value, r, g, b, a); fv = enc(c.df, r, g, b, a); end
            for (int j = 0; j < c.dh; j++) for (int i = 0; i < c.dw; i++) begin
                longint pa; bit [31:0] ov, nv;
                pa = c.d0 + (c.dy + j) * c.dp + (c.dx + i) * db;
                ov = get_px(pa, c.df); nv = ov;
                if (c.df == 0) begin
                    if (c.mask & 1) nv[31:24] = fv[31:24];
                    if (c.mask & 2) nv[23:16] = fv[23:16];
                    if (c.mask & 4) nv[15:8]  = fv[15:8];
                    if (c.mask & 8) nv[7:0]   = fv[7:0];
                end else if (c.df == 1) begin
                    if (c.mask & 1) nv[15:11] = fv[15:11];
                    if (c.mask & 2) nv[10:5]  = fv[10:5];
                    if (c.mask & 4) nv[4:0]   = fv[4:0];
                end else if (c.mask != 0) nv[7:0] = fv[7:0];
                put_px(pa, c.df, nv);
            end
        end else begin
            tb = tbpp_of(c.tf, c.sf);
            for (int p = c.first; p < c.first + c.npix; p++) begin
                int r, g, b, a; bit [31:0] sv, tv; longint pa;
                pa = c.s0 + (c.sy + p / c.sw) * c.sp + (c.sx + p % c.sw) * sb;
                sv = get_px(pa, c.sf);
                dec(c.sf, sv, r, g, b, a);
                case (c.tf)
                    0: tv = sv;
                    1: tv = {8'(a), 8'(b), 8'(g), 8'(r)};
                    2: tv = {8'(a), 8'(r), 8'(g), 8'(b)};
                    3: tv = {8'hFF, 8'(r), 8'(g), 8'(b)};
                    10: tv = {8'h0, 8'(b), 8'(g), 8'(r)};
                    11: tv = {8'h0, 8'(r), 8'(g), 8'(b)};
                    default: tv = {16'h0, 5'(dn(r, 5)), 6'(dn(g, 6)), 5'(dn(b, 5))};
                endcase
                for (int q = 0; q < tb; q++) exp_rd.push_back(tv[q*8 +: 8]);
            end
            while (exp_rd.size() % 4) exp_rd.push_back(8'h00);
        end
        // ---- DUT ----
        @(posedge clk); while (!cmd_ready) @(posedge clk);
        cmd_valid <= 1; cmd_op <= 2'(c.op); cmd_s_fmt <= 2'(c.sf); cmd_d_fmt <= 2'(c.df);
        cmd_s_row0 <= 30'(c.s0); cmd_s_pitch <= 16'(c.sp); cmd_d_row0 <= 30'(c.d0); cmd_d_pitch <= 16'(c.dp);
        cmd_s_x <= 13'(c.sx); cmd_s_y <= 13'(c.sy); cmd_s_w <= 13'(c.sw); cmd_s_h <= 13'(c.sh);
        cmd_d_x <= 13'(c.dx); cmd_d_y <= 13'(c.dy); cmd_d_w <= 13'(c.dw); cmd_d_h <= 13'(c.dh);
        cmd_hq <= 13'(c.sw / c.dw); cmd_hr2 <= 14'(2 * (c.sw % c.dw)); cmd_hu0 <= 13'(c.sw / (2 * c.dw)); cmd_ha0 <= 14'(c.sw % (2 * c.dw));
        cmd_vq <= 13'(c.sh / c.dh); cmd_vr2 <= 14'(2 * (c.sh % c.dh)); cmd_vu0 <= 13'(c.sh / (2 * c.dh)); cmd_va0 <= 14'(c.sh % (2 * c.dh));
        cmd_mir_x <= c.mx; cmd_mir_y <= c.my; cmd_bottom_up <= bup; cmd_key <= c.key; cmd_key_lo <= c.klo; cmd_key_hi <= c.khi;
        cmd_value <= c.value; cmd_mask <= 4'(c.mask); cmd_rmw <= rmw; cmd_tfmt <= 8'(c.tf);
        cmd_j0 <= 13'(c.first / ((c.op == 2) ? c.sw : 1)); cmd_i0 <= 13'(c.first % ((c.op == 2) ? c.sw : 1)); cmd_npix <= 24'(c.npix);
        cmd_alpha1 <= c.alpha1; cmd_swz <= c.swz; cmd_swz_sel <= c.sel; cmd_wrap <= c.wrap; cmd_px <= 13'(c.px); cmd_py <= 13'(c.py);
        rd_got.delete();
        @(posedge clk); cmd_valid <= 0;
        @(posedge clk);
        while (busy) @(posedge clk);
        n_cmd++;
        if (c.op == 2) n_px += c.npix; else n_px += c.dw * c.dh;
        // ---- compare ----
        foreach (touched[t]) begin
            longint a; a = touched[t];
            if (u_mig.peek_byte(a) !== ref_mem[a]) begin
                report($sformatf("case %0d op %0d fmt %0d->%0d s %0d,%0d %0dx%0d d %0d,%0d %0dx%0d mx %0d my %0d key %0d mask %0h: byte %0h = %02h expected %02h",
                       n_cmd, c.op, c.sf, c.df, c.sx, c.sy, c.sw, c.sh, c.dx, c.dy, c.dw, c.dh, c.mx, c.my, c.key, c.mask,
                       a, u_mig.peek_byte(a), ref_mem[a]));
                break;
            end
        end
        if (c.op == 2) begin
            if (rd_got.size() != exp_rd.size())
                report($sformatf("case %0d READ tf %0d fmt %0d: %0d bytes, expected %0d", n_cmd, c.tf, c.sf, rd_got.size(), exp_rd.size()));
            else foreach (exp_rd[q]) if (rd_got[q] !== exp_rd[q]) begin
                report($sformatf("case %0d READ tf %0d fmt %0d first %0d npix %0d: byte %0d = %02h expected %02h",
                                 n_cmd, c.tf, c.sf, c.first, c.npix, q, rd_got[q], exp_rd[q]));
                break;
            end
        end
    endtask

    function automatic cmd_t blank();
        cmd_t c;
        c.op = 0; c.sf = 0; c.df = 0; c.s0 = 64'h0100_0000; c.d0 = 64'h0200_0000; c.sp = 0; c.dp = 0;
        c.sx = 0; c.sy = 0; c.sw = 1; c.sh = 1; c.dx = 0; c.dy = 0; c.dw = 1; c.dh = 1;
        c.mx = 0; c.my = 0; c.key = 0; c.klo = 0; c.khi = 0; c.value = 0; c.mask = 15; c.tf = 0; c.first = 0; c.npix = 1;
        c.alpha1 = 0; c.swz = 0; c.wrap = 0; c.sel = 8'hE4; c.px = 0; c.py = 0;
        return c;
    endfunction
    function automatic int fmt_tf(input int k); int t[7] = '{0, 1, 2, 3, 4, 10, 11}; return t[k]; endfunction

    initial begin
        cmd_t c;
        repeat (10) @(posedge clk); rst <= 0;
        repeat (10) @(posedge clk);
        // ---- directed: every RGB565 value -> RGBA8 (exact expansion) by COPY, and by READ as RGBA8888 ----
        c = blank(); c.sf = 1; c.df = 0; c.sp = 512; c.dp = 1024; c.sw = 256; c.sh = 256; c.dw = 256; c.dh = 256;
        c.s0 = 64'h0300_0000; c.d0 = 64'h0310_0000;
        for (int v = 0; v < 65536; v++) begin
            preset(c.s0 + (v / 256) * 512 + (v % 256) * 2, 8'(v));
            preset(c.s0 + (v / 256) * 512 + (v % 256) * 2 + 1, 8'(v >> 8));
        end
        run(c);
        c.op = 2; c.tf = 1; c.first = 0; c.npix = 65536; run(c);
        c.tf = 4; run(c);                                    // RGB565 -> RGB565 transfer: identity
        $display("ok: all 65536 RGB565 values -> RGBA8 (COPY) and -> RGBA8888 / RGB565 (READ)");
        // ---- directed: 256 grey levels RGBA8 -> RGB565 (rounding), RGBA8 -> RGBA8 bit-exact ----
        c = blank(); c.sf = 0; c.df = 1; c.sp = 1024; c.dp = 512; c.sw = 256; c.dw = 256; c.s0 = 64'h0320_0000; c.d0 = 64'h0330_0000;
        for (int v = 0; v < 256; v++) begin
            preset(c.s0 + v * 4 + 0, 8'(urand(256))); preset(c.s0 + v * 4 + 1, 8'(255 - v));
            preset(c.s0 + v * 4 + 2, 8'(v ^ 8'h5A)); preset(c.s0 + v * 4 + 3, 8'(v));
        end
        run(c);
        c.df = 0; c.d0 = 64'h0340_0000; run(c);
        $display("ok: 256 levels RGBA8 -> RGB565 and RGBA8 -> RGBA8");
        // ---- directed: the widest row, stretch extremes ----
        c = blank(); c.sf = 1; c.df = 1; c.sp = 8192; c.dp = 8192; c.sw = 4096; c.sh = 3; c.dw = 4096; c.dh = 3;
        c.s0 = 64'h0400_0000; c.d0 = 64'h0410_0000; run(c);
        c.dw = 1000; c.dh = 7; run(c);
        c.sw = 37; c.sh = 5; c.dw = 4096; c.dh = 11; c.mx = 1; run(c);
        c = blank(); c.op = 2; c.sf = 0; c.sp = 16384; c.sw = 4096; c.sh = 2; c.s0 = 64'h0420_0000; c.tf = 10; c.first = 100; c.npix = 8000;
        run(c);
        $display("ok: 4096-pixel rows, stretch 4096 -> 1000 and 37 -> 4096 (mirrored), READ across rows");
        // ---- random ----
        for (int n = 0; n < 500; n++) begin
            int W, H, sb, db, kind;
            c = blank();
            kind = urand(10);
            c.op = (kind < 5) ? 0 : (kind < 8) ? 1 : 2;
            if (urand(6) == 0) begin c.sf = 2; c.df = 2; end
            else begin c.sf = urand(2); c.df = urand(2); end
            if (c.op == 1) c.sf = c.df;
            sb = bpp(c.sf); db = bpp(c.df);
            W = 1 + urand((n < 60) ? 12 : 400); H = 1 + urand((n < 60) ? 6 : 30);
            c.sp = W * sb + sb * urand(4) * ((urand(2) == 0) ? 9 : 1);
            c.dp = W * db + db * urand(4) * 3;
            c.s0 = 64'h0800_0000 + 32 * urand(8192) + 4 * urand(8);
            c.d0 = 64'h0C00_0000 + 32 * urand(8192) + 4 * urand(8);
            c.sx = urand(W); c.sy = urand(H); c.sw = 1 + urand(W - c.sx); c.sh = 1 + urand(H - c.sy);
            if (c.op == 0) begin
                case (urand(4))
                    0: begin c.dw = c.sw; c.dh = c.sh; end                                           // 1:1
                    1: begin c.dw = 1 + urand(3 * c.sw); c.dh = 1 + urand(3 * c.sh); end             // up / down
                    2: begin c.dw = 1 + urand(c.sw); c.dh = 1 + urand(c.sh); end                     // down
                    default: begin c.dw = c.sw * (1 + urand(3)); c.dh = c.sh * (1 + urand(3)); end  // integer up
                endcase
                if (c.dw > 4096) c.dw = 4096;
                c.dx = urand(64); c.dy = urand(20);
                c.dp = (c.dx + c.dw) * db + db * urand(4) * 3;                  // the rectangle inside its rows
                c.mx = urand(3) == 0; c.my = urand(3) == 0;
                // overlapping 1:1 copies inside one surface, every direction
                if (urand(5) == 0) begin
                    c.df = c.sf; c.dp = c.sp; c.d0 = c.s0; c.dw = c.sw; c.dh = c.sh; c.mx = 0; c.my = 0;
                    c.dx = c.sx + int'(urand(9)) - 4; c.dy = c.sy + int'(urand(7)) - 3;
                    if (c.dx < 0) c.dx = 0;
                    if (c.dx + c.dw > W) c.dx = W - c.dw;
                    if (c.dy < 0) c.dy = 0;
                end
                if (urand(3) == 0) begin
                    bit [31:0] a, b;
                    a = {urand(65536), urand(65536)}; b = {urand(65536), urand(65536)};
                    c.key = 1;
                    for (int q = 0; q < 4; q++) begin
                        if (urand(3) == 0) b[q*8 +: 8] = a[q*8 +: 8];                               // single colour
                        if (a[q*8 +: 8] > b[q*8 +: 8]) begin bit [7:0] t; t = a[q*8 +: 8]; a[q*8 +: 8] = b[q*8 +: 8]; b[q*8 +: 8] = t; end
                    end
                    if (c.sf == 1) begin                                                             // 5 / 6 / 5 fields
                        int lo[3], hi[3], wd[3] = '{5, 6, 5};
                        for (int q = 0; q < 3; q++) begin
                            lo[q] = urand(1 << wd[q]); hi[q] = lo[q] + urand((1 << wd[q]) - lo[q]);
                        end
                        a = {16'h0, 5'(lo[0]), 6'(lo[1]), 5'(lo[2])}; b = {16'h0, 5'(hi[0]), 6'(hi[1]), 5'(hi[2])};
                    end
                    c.klo = a; c.khi = b;
                end
            end else if (c.op == 1) begin
                c.d0 = c.s0; c.dp = c.sp; c.dx = c.sx; c.dy = c.sy; c.dw = c.sw; c.dh = c.sh;
                c.mask = urand(16);
                if (urand(3) == 0) c.mask = 15;
                c.tf = (c.df == 2) ? 0 : fmt_tf(urand(7));
                c.value = {urand(65536), urand(65536)};
            end else begin
                c.tf = (c.sf == 2) ? 0 : fmt_tf(urand(7));
                c.first = urand(c.sw * c.sh); c.npix = 1 + urand(c.sw * c.sh - c.first);
                rdy_pct = (urand(2) == 0) ? 100 : 40;
            end
            run(c);
            rdy_pct = 100;
            if (n % 100 == 99) $display("ok: %0d random commands", n + 1);
        end
        // ---- directed: the widest burst-path rows (RGBA8 4088 pixels at the worst alignment, RGB565 4096), alpha = 255 ----
        c = blank(); c.sf = 0; c.df = 0; c.sp = 16384 + 64; c.dp = 16384 + 96; c.sw = 4088; c.sh = 3; c.dw = 4088; c.dh = 3;
        c.s0 = 64'h0500_0000 + 28; c.d0 = 64'h0510_0000 + 4; run(c);
        c.alpha1 = 1; c.s0 = 64'h0520_0000 + 4; c.d0 = 64'h0530_0000 + 28; run(c);
        c = blank(); c.sf = 1; c.df = 1; c.sp = 8192 + 34; c.dp = 8192 + 62; c.sw = 4096; c.sh = 3; c.dw = 4096; c.dh = 3;
        c.s0 = 64'h0540_0000 + 30; c.d0 = 64'h0550_0000 + 2; run(c);
        $display("ok: widest burst-path rows (RGBA8 4088, RGB565 4096, worst alignments, alpha = 255)");
        // ---- random burst-path copies: every 2-byte alignment, overlaps in one surface, mirror y, alpha = 255 ----
        for (int n = 0; n < 400; n++) begin
            int W, H, sb, al;
            c = blank();
            c.sf = urand(2); c.df = c.sf; sb = bpp(c.sf); al = (c.sf == 0) ? 4 : 2;
            W = 1 + urand((n < 100) ? 40 : (n < 380) ? 700 : 2048); H = 1 + urand((n < 100) ? 6 : 24);
            c.sp = W * sb + sb * urand(40); c.dp = W * sb + sb * urand(40);
            c.s0 = 64'h0800_0000 + 32 * urand(8192) + al * urand(32 / al);
            c.d0 = 64'h0C00_0000 + 32 * urand(8192) + al * urand(32 / al);
            c.sx = urand(W); c.sy = urand(H); c.sw = 1 + urand(W - c.sx); c.sh = 1 + urand(H - c.sy);
            c.dw = c.sw; c.dh = c.sh; c.dx = urand(W - c.sw + 1); c.dy = urand(20);
            c.my = urand(4) == 0; c.alpha1 = urand(3) == 0;
            if (urand(4) == 0) begin                                         // inside one surface, every direction
                c.d0 = c.s0; c.dp = c.sp; c.my = 0;
                c.dx = c.sx + int'(urand(81)) - 40; c.dy = c.sy + int'(urand(9)) - 4;
                if (c.dx < 0) c.dx = 0;
                if (c.dx + c.dw > W) c.dx = W - c.dw;
                if (c.dy < 0) c.dy = 0;
            end
            run(c);
            if (n % 100 == 99) $display("ok: %0d random burst-path copies", n + 1);
        end
        // ---- random swizzle / alpha = 255 / wrapped tiles (the pixel path), with stretch, mirrors and keys ----
        for (int n = 0; n < 300; n++) begin
            int W, H, sb, db;
            c = blank();
            c.sf = urand(2); c.df = urand(2); sb = bpp(c.sf); db = bpp(c.df);
            W = 1 + urand((n < 60) ? 16 : 300); H = 1 + urand((n < 60) ? 6 : 20);
            c.sp = W * sb + sb * urand(9); c.s0 = 64'h0800_0000 + 32 * urand(8192) + 4 * urand(8);
            c.d0 = 64'h0C00_0000 + 32 * urand(8192) + 4 * urand(8);
            c.sx = urand(W); c.sy = urand(H); c.sw = 1 + urand(W - c.sx); c.sh = 1 + urand(H - c.sy);
            c.swz = urand(2) == 0; c.sel = urand(256); c.alpha1 = urand(2) == 0;
            if (urand(2) == 0) begin                                         // a tile over a larger rectangle
                c.wrap = 1; c.dw = 1 + urand(4 * c.sw + 40); c.dh = 1 + urand(3 * c.sh + 8);
                if (c.dw > 4096) c.dw = 4096;
                c.px = urand(c.sw); c.py = urand(c.sh);
            end else begin
                c.dw = (urand(2) == 0) ? c.sw : 1 + urand(2 * c.sw); c.dh = (urand(2) == 0) ? c.sh : 1 + urand(2 * c.sh);
                c.mx = urand(3) == 0; c.my = urand(3) == 0;
                if (urand(3) == 0) begin c.key = 1; c.klo = 32'h0000_0000; c.khi = {urand(65536), urand(65536)}; end
            end
            c.dx = urand(64); c.dy = urand(20); c.dp = (c.dx + c.dw) * db + db * urand(4) * 3;
            run(c);
            if (n % 100 == 99) $display("ok: %0d random swizzle / alpha / tile copies", n + 1);
        end
        // ---- speed: an 800 x 100 RGBA8 1:1 copy on the burst path, then on the pixel path (an identity swizzle) ----
        begin
            longint t0, tb_, tp_;
            c = blank(); c.sf = 0; c.df = 0; c.sp = 3200; c.dp = 3200; c.sw = 800; c.sh = 100; c.dw = 800; c.dh = 100;
            c.s0 = 64'h0600_0000 + 8; c.d0 = 64'h0640_0000 + 20;
            t0 = $time; run(c); tb_ = ($time - t0) / 10;
            c.swz = 1; c.sel = 8'hE4; c.d0 = 64'h0680_0000 + 20;
            t0 = $time; run(c); tp_ = ($time - t0) / 10;
            $display("INFO_TB 800 x 100 RGBA8 copy: %0d cycles on the burst path (%.2f pixels a cycle), %0d on the pixel path (%.2f)",
                     tb_, 80000.0 / tb_, tp_, 80000.0 / tp_);
        end
        $display("INFO_TB %0d of the copies on the burst path", n_bm);
        if (n_bm < 400) report($sformatf("only %0d burst-path copies", n_bm));
        if (c_ops != n_cmd) report($sformatf("c_ops %0d, expected %0d", c_ops, n_cmd));
        if (c_pixels != n_px) report($sformatf("c_pixels %0d, expected %0d", c_pixels, n_px));
        if (mig_err != 0) report($sformatf("MIG model errors %0d", mig_err));
        $display("INFO_TB %0d commands, %0d pixels, %0d MIG writes, %0d reads", n_cmd, n_px, n_wr, n_rd);
        if (errors == 0) $display("TB_PASS tb_img_xfer");
        else $display("TB_FAIL tb_img_xfer errors=%0d", errors);
        $finish;
    end
    initial begin #(2_000_000_000.0); $display("TB_FAIL tb_img_xfer timeout"); $finish; end
endmodule
`default_nettype wire
