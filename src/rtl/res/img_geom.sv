// img_geom — image level / layer geometry of board B, clk_sys,
// sequential (two cycles per mip level). For a descriptor and a (level, layer) it computes:
//   wl, hl     level size: max(1, width >> level), max(1, height >> level)
//   pitchl     row pitch of the level: the descriptor pitch for level 0, wl * bpp above
//   off        byte offset of (level, layer) from the image base: layer * stride + sum of the sizes of the
//              levels below, where a level's size is pitch_k * h_k rounded up to 4 bytes and stride (the layer
//              size) is the size of the whole chain rounded up to 32 bytes
//   total      bytes the image occupies: layers * stride * samples (CREATE_IMAGE heap check)
// A 3D image (vol, one sample, pitch = width * bpp, width and height <= 256): layers is the depth, which halves with the
// level; a level's size is pitch_k * h_k * d_k, its slices follow each other unpadded, off = the sizes below + layer *
// pitch_l * h_l, total = the chain rounded up to 32 bytes. dl = max(1, depth >> level) (1 when not 3D).
// The caller has checked level < levels and layer < layers (the level's values are captured on the way); a 3D image's
// layer < dl afterwards.
`default_nettype none
module img_geom (
    input  wire         clk, rst,
    input  wire         start,
    input  wire [12:0]  width, height,
    input  wire [15:0]  pitch,
    input  wire [3:0]   levels,
    input  wire [7:0]   layers,
    input  wire         vol,
    input  wire [1:0]   samples_log2,
    input  wire [1:0]   bpp_log2,              // log2 bytes per pixel: 0 (P8), 1 (RGB565), 2 (RGBA8)
    input  wire [3:0]   level,
    input  wire [7:0]   layer,
    output reg          done,
    output reg  [12:0]  wl, hl,
    output reg  [7:0]   dl,
    output reg  [15:0]  pitchl,
    output reg  [31:0]  off,
    output reg  [39:0]  total
);
    typedef enum logic [2:0] {G_IDLE, G_MUL, G_VOL, G_ACC, G_STRIDE, G_LAYER} gst_t;
    gst_t       gs;
    reg  [3:0]  k, nlev, lev;
    reg  [7:0]  nlay, lay, dk;
    reg  [7:0]  nlm;                                // the layers the total counts: 1 for a 3D image (the slices are inside the
                                                    // stride), else layers — registered at start so G_LAYER's multiply has no mux
                                                    // in front of it (timing: the `v3 -> total` path)
    reg         v3;
    reg  [1:0]  slog;
    reg  [1:0]  bl;
    reg  [12:0] wk, hk;
    reg  [15:0] pk;
    reg  [29:0] sz;
    reg  [31:0] chain, off_l, stride, slice;
    wire [12:0] wk1 = (wk > 13'd1) ? (wk >> 1) : 13'd1;
    wire [12:0] hk1 = (hk > 13'd1) ? (hk >> 1) : 13'd1;
    wire [7:0]  dk1 = (dk > 8'd1) ? (dk >> 1) : 8'd1;
    always @(posedge clk) begin
        done <= 1'b0;
        if (rst) gs <= G_IDLE;
        else case (gs)
            G_IDLE: if (start) begin
                nlev <= levels; nlay <= layers; slog <= samples_log2; v3 <= vol; nlm <= vol ? 8'd1 : layers;
                bl <= bpp_log2; lev <= level; lay <= layer;
                k <= 0; wk <= width; hk <= height; pk <= pitch; chain <= 0; off_l <= 0;
                dk <= vol ? layers : 8'd1;
                wl <= width; hl <= height; pitchl <= pitch; dl <= 8'd1;
                gs <= G_MUL;
            end
            G_MUL: begin
                sz <= pk * hk;                          // <= 65535 * 4096
                if (k == lev) begin wl <= wk; hl <= hk; pitchl <= pk; off_l <= chain; dl <= dk; end
                gs <= v3 ? G_VOL : G_ACC;
            end
            G_VOL: begin                                // 3D: <= 1024 * 256 * 255
                sz <= sz[18:0] * dk;
                if (k == lev) slice <= 32'(sz);
                gs <= G_ACC;
            end
            G_ACC: begin
                chain <= chain + {(sz[29:0] + 30'd3) & ~30'd3};
                k <= k + 4'd1; wk <= wk1; hk <= hk1; dk <= dk1;
                pk <= {3'b000, wk1} << bl;
                gs <= (k + 4'd1 == nlev) ? G_STRIDE : G_MUL;
            end
            G_STRIDE: begin stride <= (chain + 32'd31) & ~32'd31; gs <= G_LAYER; end
            G_LAYER: begin
                off <= off_l + lay * (v3 ? slice : stride);
                total <= ({32'h0, nlm} * {8'h0, stride}) << slog;
                done <= 1'b1; gs <= G_IDLE;
            end
            default: gs <= G_IDLE;
        endcase
    end
endmodule
`default_nettype wire
