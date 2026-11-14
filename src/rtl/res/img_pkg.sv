// img_pkg — image descriptors and storage formats.
// One descriptor = 128 bits in img_table (board B), fields as img_desc_t; layers holds 1..255 (256 reserved).
`ifndef IMG_PKG_SV
`define IMG_PKG_SV
package img_pkg;
    // storage formats; built: RGBA8, RGB565, P8 (8-bit palette index)
    localparam [5:0] FMT_RGBA8 = 6'h00, FMT_RGB565 = 6'h04, FMT_RGBA4444 = 6'h05, FMT_RGBA5551 = 6'h06,
                     FMT_L8 = 6'h07, FMT_A8 = 6'h08, FMT_LA88 = 6'h09, FMT_P8 = 6'h0C,
                     FMT_D24S8 = 6'h10, FMT_D16 = 6'h11, FMT_S8 = 6'h12,
                     FMT_ETC1 = 6'h20, FMT_DXT1 = 6'h21, FMT_DXT5 = 6'h22;
    // usage bits
    localparam int USE_SAMPLED = 0, USE_RT = 1, USE_SCANOUT = 2, USE_TRANSFER = 3;

    typedef struct packed {
        logic [27:0] rsvd;      // [127:100]
        logic        vol;       // [99]    3D texture: layers = depth, halving with the level, slices inside each level
        logic [3:0]  usage;     // [98:95]
        logic        origin;    // [94]    0 = top-left (rows top first), 1 = bottom-left (GL)
        logic [1:0]  layout;    // [93:92] 0 = linear
        logic [1:0]  samples;   // [91:90] log2(samples): 0 = 1, 2 = 4
        logic [7:0]  layers;    // [89:82]
        logic [3:0]  levels;    // [81:78]
        logic [5:0]  format;    // [77:72]
        logic [12:0] height;    // [71:59] 1..4096
        logic [12:0] width;     // [58:46] 1..4096
        logic [15:0] pitch;     // [45:30] bytes per row of level 0
        logic [29:0] base;      // [29:0]  byte address in B's DDR, 32-byte aligned
    } img_desc_t;
endpackage
`endif
