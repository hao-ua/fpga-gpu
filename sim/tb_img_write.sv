`timescale 1ns/1ps
// tb_img_write — unit bench of img_write (board B WRITE_IMAGE engine) behind mem_arb with the MIG model.
// Every case: background bytes are preset around the target, a command (random storage format, transfer
// format, pitch, level offset, rectangle, first pixel and pixel count) streams its payload with random gaps
// while the MIG model applies random back-pressure; afterwards every byte of the rows touched (plus one row
// above and below, and 64 bytes either side) is compared with an independent model: pixels converted with
// real-number rounding both ways (round(v * 31 / 255), round(c * 255 / 31): ES 2.0 exact), bytes
// outside the written pixels unchanged. Storage formats RGBA8, RGB565 and P8 (RAW only).
// Directed cases: all 65536 RGB565 values into RGBA8 storage, all 256 grey levels of every 8-bit transfer
// format into RGB565 storage, and RAW in all storage formats.
`default_nettype none
module tb_img_write;
    reg clk = 0, rst = 1;
    always #5.0 clk = ~clk;
    int errors = 0, nprint = 0;
    task automatic report(input string s);
        errors++;
        if (nprint < 30) begin $display("%t FAIL %s", $time, s); nprint++; end
    endtask

    // ---------------- DUT + arbiter + MIG model ----------------
    reg         cmd_valid = 0; wire cmd_ready;
    reg  [29:0] cmd_row; reg [15:0] cmd_pitch; reg [12:0] cmd_x, cmd_w, cmd_i0; reg [23:0] cmd_npix;
    reg  [7:0]  cmd_tfmt; reg [1:0] cmd_sf;
    reg         d_valid = 0; reg [31:0] d_data = 0; wire d_ready, busy; wire [31:0] c_pixels;
    wire        w_req, w_en, w_wren, w_wend, w_rdy, w_wdf_rdy; wire [28:0] w_addr; wire [255:0] w_data; wire [31:0] w_mask;
    img_write dut (.clk(clk), .rst(rst), .cmd_valid(cmd_valid), .cmd_ready(cmd_ready), .cmd_row(cmd_row),
        .cmd_pitch(cmd_pitch), .cmd_x(cmd_x), .cmd_w(cmd_w), .cmd_i0(cmd_i0), .cmd_npix(cmd_npix), .cmd_tfmt(cmd_tfmt),
        .cmd_sf(cmd_sf), .d_valid(d_valid), .d_data(d_data), .d_ready(d_ready), .busy(busy), .c_pixels(c_pixels),
        .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren), .w_wend(w_wend),
        .w_rdy(w_rdy), .w_wdf_rdy(w_wdf_rdy));
    wire        app_en, app_wdf_wren, app_wdf_end, app_rdy, app_wdf_rdy, app_rd_data_valid;
    wire [2:0]  app_cmd; wire [28:0] app_addr; wire [255:0] app_wdf_data, app_rd_data; wire [31:0] app_wdf_mask;
    wire        r_rdy, r_valid; wire [255:0] rd_data;
    mem_arb #(.NR(1), .NW(1)) u_arb (.clk(clk), .rst(rst), .r_en(1'b0), .r_addr(29'h0), .r_rdy(r_rdy), .r_valid(r_valid),
        .rd_data(rd_data), .w_req(w_req), .w_en(w_en), .w_addr(w_addr), .w_data(w_data), .w_mask(w_mask), .w_wren(w_wren),
        .w_wend(w_wend), .w_rdy(w_rdy), .w_wdf_rdy(w_wdf_rdy),
        .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data), .app_wdf_mask(app_wdf_mask),
        .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy), .app_wdf_rdy(app_wdf_rdy),
        .app_rd_data_valid(app_rd_data_valid), .app_rd_data(app_rd_data));
    int mig_err; longint n_wr, n_rd;
    mig_app_model #(.RD_LAT(20), .RDY_PCT(70), .WDF_PCT(75), .SEED(11)) u_mig (
        .ui_clk(clk), .rst(rst), .app_en(app_en), .app_cmd(app_cmd), .app_addr(app_addr), .app_wdf_data(app_wdf_data),
        .app_wdf_mask(app_wdf_mask), .app_wdf_wren(app_wdf_wren), .app_wdf_end(app_wdf_end), .app_rdy(app_rdy),
        .app_wdf_rdy(app_wdf_rdy), .app_rd_data(app_rd_data), .app_rd_data_valid(app_rd_data_valid),
        .errors(mig_err), .n_wr(n_wr), .n_rd(n_rd));

    // ---------------- reference model ----------------
    bit [7:0] ref_mem [longint];                     // expected bytes (only addresses the bench touched)
    function automatic int sbpp(input int sf); return (sf == 0) ? 4 : (sf == 1) ? 2 : 1; endfunction
    function automatic int tbpp_of(input [7:0] tf, input int sf);
        case (tf)
            8'h00: return sbpp(sf);
            8'h04: return 2;
            8'h0A, 8'h0B: return 3;
            default: return 4;
        endcase
    endfunction
    function automatic int rnd(input int v, input int bits);       // round(v * (2^bits - 1) / 255)
        real f; f = v * ((1 << bits) - 1) / 255.0;
        return $rtoi(f + 0.5);
    endfunction
    function automatic int up(input int c, input int bits);         // round(c * 255 / (2^bits - 1))
        real f; f = c * 255.0 / ((1 << bits) - 1);
        return $rtoi(f + 0.5);
    endfunction
    // pixel bytes (transfer format) -> storage bytes
    function automatic bit [31:0] convert(input [7:0] tf, input int sf, input bit [31:0] b);
        int r, g, bl, a; bit [15:0] v; bit s565; s565 = (sf == 1);
        if (tf == 8'h00) return (sf == 0) ? b : (sf == 1) ? {16'h0, b[15:0]} : {24'h0, b[7:0]};
        case (tf)
            8'h01: begin r = b[7:0];  g = b[15:8]; bl = b[23:16]; a = b[31:24]; end
            8'h02: begin bl = b[7:0]; g = b[15:8]; r = b[23:16];  a = b[31:24]; end
            8'h03: begin bl = b[7:0]; g = b[15:8]; r = b[23:16];  a = 255; end
            8'h0A: begin r = b[7:0];  g = b[15:8]; bl = b[23:16]; a = 255; end
            8'h0B: begin bl = b[7:0]; g = b[15:8]; r = b[23:16];  a = 255; end
            default: begin
                v = b[15:0];
                r = up(v[15:11], 5); g = up(v[10:5], 6); bl = up(v[4:0], 5);
                a = 255;
            end
        endcase
        if (s565) begin
            v = (rnd(r, 5) << 11) | (rnd(g, 6) << 5) | rnd(bl, 5);
            return {16'h0, v};
        end
        return {8'(r), 8'(g), 8'(bl), 8'(a)};      // little-endian word 0xRRGGBBAA: bytes A, B, G, R
    endfunction
    function automatic bit [7:0] mem_byte(input longint a);
        return u_mig.peek_byte(a);
    endfunction
    // preset a byte both in the MIG model and in the reference
    task automatic preset(input longint a, input bit [7:0] v);
        bit [255:0] bur; longint bi;
        bi = a >> 5;
        bur = u_mig.mem.exists(bi) ? u_mig.mem[bi] : 256'h0;
        bur[(a & 31) * 8 +: 8] = v;
        u_mig.mem[bi] = bur;
        ref_mem[a] = v;
    endtask

    // ---------------- one command ----------------
    int unsigned seed = 32'h1234_5678;
    function automatic int unsigned urand(input int unsigned n);   // 0 .. n-1
        seed = seed * 1664525 + 1013904223;
        return (n == 0) ? 0 : ((seed >> 8) % n);
    endfunction
    int gap_pct = 30;
    longint total_px = 0;

    // pix[k] = transfer bytes of pixel k (k = 0 .. npix-1)
    task automatic run_cmd(input longint level_base, input int pitch, input int x, input int y, input int w,
                           input int first, input int npix, input [7:0] tf, input int sf, ref bit [31:0] pix[$]);
        int tb, sb, nbytes, nwords, j0, i0, k, wi;
        bit [7:0] bytes[$];
        longint a, lo, hi;
        tb = tbpp_of(tf, sf); sb = sbpp(sf);
        j0 = first / w; i0 = first % w;
        // payload bytes
        for (k = 0; k < npix; k++) for (int q = 0; q < tb; q++) bytes.push_back(pix[k][q*8 +: 8]);
        nbytes = bytes.size(); nwords = (nbytes + 3) / 4;
        while (bytes.size() < nwords * 4) bytes.push_back(8'(urand(256)));     // padding: must be ignored
        // background around the touched rows
        lo = level_base + (y + j0) * pitch - pitch - 64;
        hi = level_base + (y + (first + npix - 1) / w) * pitch + pitch + 64;
        if (lo < 0) lo = 0;
        for (a = lo; a < hi; a++) preset(a, 8'(urand(256)));
        // expected pixels
        for (k = 0; k < npix; k++) begin
            bit [31:0] sv; int p, row, col;
            p = first + k; row = y + p / w; col = x + p % w;
            sv = convert(tf, sf, pix[k]);
            a = level_base + row * pitch + col * sb;
            for (int q = 0; q < sb; q++) ref_mem[a + q] = sv[q*8 +: 8];
        end
        // command
        @(posedge clk); while (!cmd_ready) @(posedge clk);
        cmd_valid <= 1; cmd_row <= 30'(level_base + (y + j0) * pitch); cmd_pitch <= 16'(pitch);
        cmd_x <= 13'(x); cmd_w <= 13'(w); cmd_i0 <= 13'(i0); cmd_npix <= 24'(npix); cmd_tfmt <= tf; cmd_sf <= 2'(sf);
        @(posedge clk); cmd_valid <= 0;
        // payload with random gaps
        wi = 0;
        while (wi < nwords) begin
            if (urand(100) < gap_pct) begin d_valid <= 0; @(posedge clk); end
            else begin
                d_valid <= 1; d_data <= {bytes[wi*4+3], bytes[wi*4+2], bytes[wi*4+1], bytes[wi*4]};
                @(posedge clk); while (!d_ready) @(posedge clk);
                wi++;
            end
        end
        d_valid <= 0;
        repeat (2) @(posedge clk);
        while (busy) @(posedge clk);
        if (d_ready) report("d_ready high while idle");
        total_px += npix;
        // compare
        for (a = lo; a < hi; a++)
            if (mem_byte(a) !== ref_mem[a]) begin
                report($sformatf("tf %02h sf %0d pitch %0d rect x %0d y %0d w %0d first %0d npix %0d: byte %0h = %02h expected %02h",
                                 tf, sf, pitch, x, y, w, first, npix, a, mem_byte(a), ref_mem[a]));
                break;
            end
    endtask

    initial begin
        bit [31:0] pix[$];
        bit [7:0] tfs [7] = '{8'h00, 8'h01, 8'h02, 8'h03, 8'h04, 8'h0A, 8'h0B};
        int n_cases;
        repeat (10) @(posedge clk); rst <= 0;
        repeat (10) @(posedge clk);
        // ---- directed: every RGB565 value into RGBA8 storage (expansion): a 4096 x 16 rectangle in 4 commands ----
        for (int part = 0; part < 4; part++) begin
            pix.delete();
            for (int k = 0; k < 16384; k++) pix.push_back({16'h0, 16'(part * 16384 + k)});
            run_cmd(64'h0100_0000, 4096 * 4, 0, 0, 4096, part * 16384, 16384, 8'h04, 0, pix);
        end
        $display("ok: all 65536 RGB565 values -> RGBA8 (exact round(c * 255 / 31))");
        // ---- directed: P8 (RAW bytes) ----
        pix.delete();
        for (int k = 0; k < 1000; k++) pix.push_back({24'h0, 8'(k * 37)});
        run_cmd(64'h0180_0000, 1003, 5, 2, 997, 3, 1000, 8'h00, 2, pix);
        $display("ok: P8 RAW");
        // ---- directed: every grey level of every 8-bit format into RGB565 (rounding), and RGB565 -> RGB565 ----
        foreach (tfs[t]) begin
            pix.delete();
            for (int v = 0; v < 256; v++) begin
                bit [7:0] c0, c1, c2; c0 = 8'(v); c1 = 8'(v ^ 8'h5A); c2 = 8'(255 - v);
                pix.push_back({c0, c2, c1, c0});
            end
            run_cmd(64'h0200_0000 + t * 4096, 512, 0, 0, 256, 0, 256, tfs[t], 1, pix);
        end
        $display("ok: 256 levels of every transfer format -> RGB565 (round to nearest)");
        // ---- random cases ----
        n_cases = 0;
        for (int c = 0; c < 400; c++) begin
            int sf; bit [7:0] tf; int sb, W, H, pitch, x, y, w, h, first, npix, tb, maxpix;
            longint lbase;
            sf = urand(5); if (sf > 2) sf = urand(2); sb = sbpp(sf);           // P8 one case in five
            tf = (sf == 2) ? 8'h00 : tfs[urand(7)]; tb = tbpp_of(tf, sf);
            W = 1 + urand((c < 50) ? 9 : 700); H = 1 + urand(40);
            pitch = W * sb + sb * urand(3) * ((urand(3) == 0) ? 7 : 1);
            lbase = 64'h0400_0000 + 32 * urand(4096) + 4 * urand(8);          // level offsets are 4-byte aligned
            x = urand(W); y = urand(H); w = 1 + urand(W - x); h = 1 + urand(H - y);
            first = urand(w * h);
            maxpix = 1440 / tb;
            if (urand(4) == 0) maxpix = 1 + urand(4);
            npix = 1 + urand((w * h - first < maxpix) ? w * h - first : maxpix);
            pix.delete();
            for (int k = 0; k < npix; k++) pix.push_back({16'(urand(65536)), 16'(urand(65536))});
            gap_pct = (c % 3 == 0) ? 0 : 40;
            run_cmd(lbase, pitch, x, y, w, first, npix, tf, sf, pix);
            n_cases++;
        end
        $display("ok: %0d random commands (%0d pixels in all)", n_cases, total_px);
        if (c_pixels != total_px) report($sformatf("pixel counter %0d, expected %0d", c_pixels, total_px));
        if (mig_err != 0) report($sformatf("MIG model errors %0d", mig_err));
        if (errors == 0) $display("TB_PASS tb_img_write");
        else $display("TB_FAIL tb_img_write errors=%0d", errors);
        $finish;
    end
    initial begin #(200_000_000.0); $display("TB_FAIL tb_img_write timeout"); $finish; end
endmodule
`default_nettype wire
