// DVI/HDMI TMDS 8b/10b encoder — DVI 1.0 spec, section 3.2.
// Encodes one channel: video data during DE, control data (C0/C1) during blanking.
`timescale 1ns/1ps
module tmds_encoder (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       de,
    input  wire [1:0] ctrl,
    input  wire [7:0] data,
    output reg  [9:0] tmds
);
    // Stage 1: minimize transitions (xor vs xnor)
    function automatic [3:0] count_ones;
        input [7:0] v;
        begin
            count_ones = v[0] + v[1] + v[2] + v[3] + v[4] + v[5] + v[6] + v[7];
        end
    endfunction

    wire [3:0] n1_d = count_ones(data);
    wire use_xnor = (n1_d > 4'd4) || ((n1_d == 4'd4) && (data[0] == 1'b0));

    reg [8:0] q_m;
    integer i;
    always @(*) begin
        q_m[0] = data[0];
        for (i = 1; i < 8; i = i + 1) begin
            q_m[i] = use_xnor ? (q_m[i-1] ~^ data[i]) : (q_m[i-1] ^ data[i]);
        end
        q_m[8] = use_xnor ? 1'b0 : 1'b1;
    end

    // Stage 2: balance running disparity
    reg signed [4:0] cnt;
    wire [3:0] n1_qm = q_m[0]+q_m[1]+q_m[2]+q_m[3]+q_m[4]+q_m[5]+q_m[6]+q_m[7];
    wire [3:0] n0_qm = 4'd8 - n1_qm;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tmds <= 10'b0;
            cnt  <= 5'sd0;
        end else if (de) begin
            if (cnt == 0 || (n1_qm == n0_qm)) begin
                tmds[9]   <= ~q_m[8];
                tmds[8]   <=  q_m[8];
                tmds[7:0] <=  q_m[8] ? q_m[7:0] : ~q_m[7:0];
                if (q_m[8] == 1'b0)
                    cnt <= cnt + $signed({1'b0, n0_qm}) - $signed({1'b0, n1_qm});
                else
                    cnt <= cnt + $signed({1'b0, n1_qm}) - $signed({1'b0, n0_qm});
            end else if ((cnt > 0 && n1_qm > n0_qm) ||
                         (cnt < 0 && n0_qm > n1_qm)) begin
                tmds[9]   <= 1'b1;
                tmds[8]   <= q_m[8];
                tmds[7:0] <= ~q_m[7:0];
                cnt <= cnt + $signed({1'b0, q_m[8], 1'b0}) // +2*q_m[8]
                           + $signed({1'b0, n0_qm}) - $signed({1'b0, n1_qm});
            end else begin
                tmds[9]   <= 1'b0;
                tmds[8]   <= q_m[8];
                tmds[7:0] <= q_m[7:0];
                cnt <= cnt - $signed({1'b0, ~q_m[8], 1'b0}) // -2*(~q_m[8])
                           + $signed({1'b0, n1_qm}) - $signed({1'b0, n0_qm});
            end
        end else begin
            case (ctrl)
                2'b00: tmds <= 10'b1101010100;
                2'b01: tmds <= 10'b0010101011;
                2'b10: tmds <= 10'b0101010100;
                2'b11: tmds <= 10'b1010101011;
            endcase
            cnt <= 5'sd0;
        end
    end
endmodule
