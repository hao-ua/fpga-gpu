// tb_img_geom — img_geom's level / layer geometry: 2D and cube layers as before, 3D images (depth halving with the level,
// slices inside each level, 16-bit levels rounded to 4 bytes as tmu24's level table does). Hand-computed expectations.
`default_nettype none
module tb_img_geom;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1, start = 0, vol = 0;
    reg [12:0] width = 0, height = 0; reg [15:0] pitch = 0; reg [3:0] levels = 0, level = 0; reg [7:0] layers = 0, layer = 0;
    reg [1:0] bpl = 0;
    wire done; wire [12:0] wl, hl; wire [7:0] dl; wire [15:0] pitchl; wire [31:0] off; wire [39:0] total;
    img_geom dut(.clk, .rst, .start, .width, .height, .pitch, .levels, .layers, .vol, .samples_log2(2'd0), .bpp_log2(bpl),
                 .level, .layer, .done, .wl, .hl, .dl, .pitchl, .off, .total);
    integer err = 0, n = 0;
    task automatic chk(input int w, h, p, lv, ly, v, b, l, z, input longint eoff, etot, input int ew, eh, ed, ep);
        @(posedge clk);
        width <= 13'(w); height <= 13'(h); pitch <= 16'(p); levels <= 4'(lv); layers <= 8'(ly); vol <= 1'(v); bpl <= 2'(b);
        level <= 4'(l); layer <= 8'(z); start <= 1'b1;
        @(posedge clk); start <= 1'b0;
        do @(posedge clk); while (!done);
        @(negedge clk);
        n++;
        if (off !== 32'(eoff) || total !== 40'(etot) || wl !== 13'(ew) || hl !== 13'(eh) || dl !== 8'(ed) || pitchl !== 16'(ep)) begin
            err++;
            $display("  case %0d (%0dx%0d layers %0d vol %0d, level %0d layer %0d): off %0d total %0d %0dx%0dx%0d pitch %0d, expected %0d %0d %0dx%0dx%0d %0d",
                     n, w, h, ly, v, l, z, off, total, wl, hl, dl, pitchl, eoff, etot, ew, eh, ed, ep);
        end
    endtask
    initial begin
        repeat (4) @(posedge clk); rst <= 0;
        // 2D and cube: levels 256 + 64 = stride 320
        chk(8, 8, 32, 2, 1, 0, 2, 0, 0,   0,  320, 8, 8, 1, 32);
        chk(8, 8, 32, 2, 6, 0, 2, 1, 2, 896, 1920, 4, 4, 1, 16);
        // 3D RGBA8 8 x 4 x 5, 4 levels: 640, 64, 8, 4 bytes = 716 -> 736; slices of 128, 32, 8, 4 bytes
        chk(8, 4, 32, 4, 5, 1, 2, 0, 0,   0, 736, 8, 4, 5, 32);
        chk(8, 4, 32, 4, 5, 1, 2, 0, 4, 512, 736, 8, 4, 5, 32);
        chk(8, 4, 32, 4, 5, 1, 2, 1, 1, 672, 736, 4, 2, 2, 16);
        chk(8, 4, 32, 4, 5, 1, 2, 2, 0, 704, 736, 2, 1, 1,  8);
        chk(8, 4, 32, 4, 5, 1, 2, 3, 0, 712, 736, 1, 1, 1,  4);
        // 3D 16-bit 3 x 3 x 3, 2 levels: 54 -> 56, 2 -> 4 = 60 -> 64; the slices unpadded (18 bytes)
        chk(3, 3, 6, 2, 3, 1, 1, 0, 2, 36, 64, 3, 3, 3, 6);
        chk(3, 3, 6, 2, 3, 1, 1, 1, 0, 56, 64, 1, 1, 1, 2);
        // the largest: 256 x 256 x 255 RGBA8, one level
        chk(256, 256, 1024, 1, 255, 1, 2, 0, 254, 254 * 262144, 255 * 262144, 256, 256, 255, 1024);
        $display("IMG_GEOM: %0d cases, errors = %0d", n, err);
        if (err == 0 && n == 10) $display("TB_PASS tb_img_geom"); else $display("TB_FAIL tb_img_geom");
        $finish;
    end
    initial begin #1ms; $display("TB_FAIL tb_img_geom: TIMEOUT"); $finish; end
endmodule
`default_nettype wire
