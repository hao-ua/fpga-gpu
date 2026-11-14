// disp_out_b — board B's display output: video timing, line-buffer read, frame-lock and underrun signals, TMDS
// encoders and serializers, with disp_post (palette lookup for P8 words, hardware cursor) inserted between the
// line-buffer read and the encoders; the sync qualifiers are delayed with the pixels (5 cycles after video_timing).
`timescale 1ns/1ps
module disp_out_b #(
    parameter int PREROLL_NONEMPTY_CYCLES = 1024
) (
    input  wire        pixel_clk,
    input  wire        serial_clk_5x,
    input  wire        rst_n,                  // pixel-domain reset, active-low
    input  wire [11:0] vt_h_active, vt_h_fp, vt_h_sync_w, vt_h_bp,
    input  wire [11:0] vt_v_active, vt_v_fp, vt_v_sync_w, vt_v_bp,
    input  wire        vt_hsync_pol, vt_vsync_pol,
    // line buffer read interface (rd_clk = pixel_clk)
    output wire        lb_rd_pixel_en,
    output wire [10:0] lb_rd_pixel_idx,
    input  wire [31:0] lb_rd_pixel,
    output reg         lb_rd_line_done,
    output reg         lb_rd_frame_sync,
    input  wire        lb_rd_buf_avail,
    output reg         frame_sync_toggle,
    // palette / cursor write side and cursor mailbox (wclk = clk_sys), see disp_post
    input  wire        wclk,
    input  wire        pal_we,
    input  wire [8:0]  pal_wa,
    input  wire [23:0] pal_wd,
    input  wire        cs_we,
    input  wire [12:0] cs_wa,
    input  wire [31:0] cs_wd,
    input  wire        mb_req,
    input  wire [47:0] mb_state,
    output wire        mb_ack,
    // HDMI
    output wire        hdmi_clk_p, hdmi_clk_n,
    output wire [2:0]  hdmi_data_p, hdmi_data_n,
    output wire        dbg_lb_underrun_toggle
);
    // ---------------- preroll: wait for at least one filled buffer ----------------
    localparam int CW = $clog2(PREROLL_NONEMPTY_CYCLES + 1);
    reg [CW-1:0] preroll_ctr;
    reg          preroll_done;
    always @(posedge pixel_clk or negedge rst_n) begin
        if (!rst_n) begin preroll_ctr <= '0; preroll_done <= 1'b0; end
        else if (!preroll_done) begin
            if (!lb_rd_buf_avail) preroll_ctr <= '0;
            else if (preroll_ctr == PREROLL_NONEMPTY_CYCLES[CW-1:0]) preroll_done <= 1'b1;
            else preroll_ctr <= preroll_ctr + 1'b1;
        end
    end
    wire vt_rst_n = rst_n & preroll_done;

    // ---------------- video timing ----------------
    wire        hsync, vsync, de;
    wire [11:0] x, y, h_count, v_count;
    video_timing u_vt (.pixel_clk(pixel_clk), .rst_n(vt_rst_n), .h_active(vt_h_active), .h_fp(vt_h_fp),
        .h_sync_w(vt_h_sync_w), .h_bp(vt_h_bp), .v_active(vt_v_active), .v_fp(vt_v_fp), .v_sync_w(vt_v_sync_w),
        .v_bp(vt_v_bp), .hsync_pol(vt_hsync_pol), .vsync_pol(vt_vsync_pol), .h_count(h_count), .v_count(v_count),
        .hsync(hsync), .vsync(vsync), .de(de), .x(x), .y(y));

    // ---------------- line buffer read (as display_out, no replication) ----------------
    assign lb_rd_pixel_en  = de;
    assign lb_rd_pixel_idx = x[10:0];
    reg [11:0] y_q;
    reg        de_q, underrun_toggle_q;
    wire de_fall = de_q & ~de;
    always @(posedge pixel_clk or negedge vt_rst_n) begin
        if (!vt_rst_n) begin de_q <= 1'b0; y_q <= 12'd0; lb_rd_line_done <= 1'b0; underrun_toggle_q <= 1'b0; end
        else begin
            de_q <= de; y_q <= y;
            lb_rd_line_done <= de_fall & lb_rd_buf_avail;
            if (de_fall & ~lb_rd_buf_avail) underrun_toggle_q <= ~underrun_toggle_q;
        end
    end

    // ---------------- frame-lock pulse (vsync rising) ----------------
    reg vsync_active_q;
    always @(posedge pixel_clk or negedge vt_rst_n) begin
        if (!vt_rst_n) begin vsync_active_q <= 1'b0; lb_rd_frame_sync <= 1'b0; frame_sync_toggle <= 1'b0; end
        else begin
            vsync_active_q   <= (vsync == vt_vsync_pol);
            lb_rd_frame_sync <= (vsync == vt_vsync_pol) & ~vsync_active_q;
            if ((vsync == vt_vsync_pol) & ~vsync_active_q) frame_sync_toggle <= ~frame_sync_toggle;
        end
    end
    assign dbg_lb_underrun_toggle = underrun_toggle_q;

    // ---------------- palette + cursor (5 cycles) ----------------
    wire        p_de, p_hs, p_vs; wire [23:0] p_rgb;
    disp_post u_post (.wclk(wclk), .pal_we(pal_we), .pal_wa(pal_wa), .pal_wd(pal_wd), .cs_we(cs_we), .cs_wa(cs_wa),
        .cs_wd(cs_wd), .mb_req(mb_req), .mb_state(mb_state), .mb_ack(mb_ack),
        .pclk(pixel_clk), .prst(~vt_rst_n), .vsync_start(lb_rd_frame_sync),
        .in_de(de), .in_hs(hsync == vt_hsync_pol), .in_vs(vsync == vt_vsync_pol), .in_x(x), .in_y(y),
        .in_px(lb_rd_pixel), .out_de(p_de), .out_hs(p_hs), .out_vs(p_vs), .out_rgb(p_rgb));
    // registered "active" flags reset to a constant; the polarity is applied after them (run-time polarities)
    reg de_d, hs_act_d, vs_act_d;
    reg [23:0] rgb_d;
    always @(posedge pixel_clk or negedge vt_rst_n) begin
        if (!vt_rst_n) begin de_d <= 1'b0; hs_act_d <= 1'b0; vs_act_d <= 1'b0; rgb_d <= 24'h0; end
        else begin de_d <= p_de; hs_act_d <= p_hs; vs_act_d <= p_vs; rgb_d <= p_de ? p_rgb : 24'h0; end
    end
    wire hsync_d = hs_act_d ? vt_hsync_pol : ~vt_hsync_pol;
    wire vsync_d = vs_act_d ? vt_vsync_pol : ~vt_vsync_pol;

    // ---------------- TMDS encoders / serializers ----------------
    wire [9:0] tmds_b, tmds_g, tmds_r;
    tmds_encoder u_enc_b (.clk(pixel_clk), .rst_n(vt_rst_n), .de(de_d), .ctrl({vsync_d, hsync_d}), .data(rgb_d[7:0]),   .tmds(tmds_b));
    tmds_encoder u_enc_g (.clk(pixel_clk), .rst_n(vt_rst_n), .de(de_d), .ctrl(2'b00),              .data(rgb_d[15:8]),  .tmds(tmds_g));
    tmds_encoder u_enc_r (.clk(pixel_clk), .rst_n(vt_rst_n), .de(de_d), .ctrl(2'b00),              .data(rgb_d[23:16]), .tmds(tmds_r));
    tmds_serializer u_ser_b (.pixel_clk(pixel_clk), .serial_clk_5x(serial_clk_5x), .rst_n(vt_rst_n), .data(tmds_b),
        .tmds_p(hdmi_data_p[0]), .tmds_n(hdmi_data_n[0]));
    tmds_serializer u_ser_g (.pixel_clk(pixel_clk), .serial_clk_5x(serial_clk_5x), .rst_n(vt_rst_n), .data(tmds_g),
        .tmds_p(hdmi_data_p[1]), .tmds_n(hdmi_data_n[1]));
    tmds_serializer u_ser_r (.pixel_clk(pixel_clk), .serial_clk_5x(serial_clk_5x), .rst_n(vt_rst_n), .data(tmds_r),
        .tmds_p(hdmi_data_p[2]), .tmds_n(hdmi_data_n[2]));
    tmds_serializer u_ser_clk (.pixel_clk(pixel_clk), .serial_clk_5x(serial_clk_5x), .rst_n(vt_rst_n),
        .data(10'b0000011111), .tmds_p(hdmi_clk_p), .tmds_n(hdmi_clk_n));
    wire _unused = &{1'b0, y_q, h_count, v_count};
endmodule
