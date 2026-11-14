// video_timing: parameterized H/V counter for runtime mode switching.
//
// All timing parameters are runtime register inputs (driven by resolution_ctrl)
// so the same module produces 640x480 / 800x600 / 1280x720 / 1280x800 timing
// depending on what's loaded.
//
// Counter widths: 12 bits is enough for the worst-case totals among our
// supported modes (largest H_total = 1650 for 1280x720, V_total = 823 for
// 1280x800; both fit in 11 bits, 12 leaves margin).
//
// Conventions:
//   - hsync_pol = 1 → HSYNC active HIGH during sync window; 0 → active LOW.
//   - vsync_pol = 1 → likewise.
//   - de        = active video region (0..h_active-1, 0..v_active-1).
//   - de, x, y and the sync windows are registered; hsync / vsync are the registered window flags in
//     the requested polarity. The flags reset to a constant (inactive), so the polarity inputs, which may
//     be run-time registers, never become an asynchronous load value (Vivado would build a latch for that).
//
// The register inputs (h_active, h_fp, ...) must be stable while rst_n=1.
// resolution_ctrl achieves this by holding rst_n=0 around the parameter
// update.
`timescale 1ns/1ps
module video_timing (
    input  wire        pixel_clk,
    input  wire        rst_n,

    // Runtime mode parameters (latched externally; should be stable while
    // not in reset). All values are in pixel-clock counts.
    input  wire [11:0] h_active,
    input  wire [11:0] h_fp,
    input  wire [11:0] h_sync_w,
    input  wire [11:0] h_bp,
    input  wire [11:0] v_active,
    input  wire [11:0] v_fp,
    input  wire [11:0] v_sync_w,
    input  wire [11:0] v_bp,
    input  wire        hsync_pol,           // 1 = positive
    input  wire        vsync_pol,

    output reg  [11:0] h_count,
    output reg  [11:0] v_count,
    output wire        hsync,
    output wire        vsync,
    output reg         de,
    output reg  [11:0] x,                   // active x (0 .. h_active-1)
    output reg  [11:0] y                    // active y (0 .. v_active-1)
);
    wire [11:0] h_total      = h_active + h_fp + h_sync_w + h_bp;
    wire [11:0] v_total      = v_active + v_fp + v_sync_w + v_bp;
    wire [11:0] h_sync_start = h_active + h_fp;
    wire [11:0] h_sync_end   = h_sync_start + h_sync_w;
    wire [11:0] v_sync_start = v_active + v_fp;
    wire [11:0] v_sync_end   = v_sync_start + v_sync_w;

    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            h_count <= 12'd0;
            v_count <= 12'd0;
        end else begin
            if (h_count == h_total - 1'b1) begin
                h_count <= 12'd0;
                v_count <= (v_count == v_total - 1'b1) ? 12'd0 : v_count + 1'b1;
            end else begin
                h_count <= h_count + 1'b1;
            end
        end
    end

    wire hs_window = (h_count >= h_sync_start) && (h_count < h_sync_end);
    wire vs_window = (v_count >= v_sync_start) && (v_count < v_sync_end);

    reg hs_act, vs_act;                     // sync window active (registered)
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin
            hs_act <= 1'b0;                 // idle = inactive level
            vs_act <= 1'b0;
            de    <= 1'b0;
            x     <= 12'd0;
            y     <= 12'd0;
        end else begin
            hs_act <= hs_window;
            vs_act <= vs_window;
            de    <= (h_count < h_active) && (v_count < v_active);
            x     <= (h_count < h_active) ? h_count : 12'd0;
            y     <= (v_count < v_active) ? v_count : 12'd0;
        end
    end
    assign hsync = hs_act ? hsync_pol : ~hsync_pol;
    assign vsync = vs_act ? vsync_pol : ~vsync_pol;
endmodule
