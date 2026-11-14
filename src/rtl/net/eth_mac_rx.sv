// eth_mac_rx — receive MAC with FCS check and store-and-forward ring.
// Strips preamble/SFD, writes the frame bytes (destination MAC .. FCS) into a byte ring, runs CRC-32
// over them and commits the frame only if the FCS residue is correct, the length is 64..MAX_FRAME
// bytes, no RX_ER was seen, the ring did not overflow and the length FIFO has room. A rejected frame is
// rolled back (its bytes are overwritten by the next frame), so nothing downstream ever sees a bad frame.
// The committed length excludes the FCS. The parser reads the ring through rd_addr/rd_data (1-cycle
// latency), starting at f_start = the oldest unreleased frame, and releases it with f_release.
`default_nettype none
module eth_mac_rx #(
    parameter int RING_AW   = 15,             // 32 KB ring
    parameter int MAX_FRAME = 9022            // incl. FCS: 9000 payload + 14 header + 4 VLAN + 4 FCS
)(
    input  wire               clk, rst,
    input  wire               dv, stb, er,
    input  wire [7:0]         data,
    // committed frames
    output wire               f_avail,
    output wire [13:0]        f_len,          // bytes, FCS excluded
    output wire [RING_AW-1:0] f_start,
    input  wire [RING_AW-1:0] rd_addr,
    output reg  [7:0]         rd_data,
    input  wire               f_release,      // pulse: done with the frame at f_start
    // counters
    output reg  [31:0]        c_ok, c_fcs, c_len, c_ovf, c_er
);
    import net_pkg::*;
    localparam int RING = 1 << RING_AW;

    (* ram_style = "block" *) reg [7:0] ring [0:RING-1];
    reg [RING_AW:0] wp, wp_commit, rp;
    reg [7:0]       wdat;
    reg [RING_AW-1:0] wadr;
    reg             wen;
    always @(posedge clk) begin
        if (wen) ring[wadr] <= wdat;
        rd_data <= ring[rd_addr];
    end

    // length FIFO (LUTRAM)
    localparam int LF_AW = 5;
    reg [13:0]      lf [0:(1<<LF_AW)-1];
    reg [LF_AW:0]   lf_w, lf_r;
    wire [LF_AW:0]  lf_held  = lf_w - lf_r;     // sized differences: they must wrap at the pointer width
    wire            lf_full  = lf_held[LF_AW];
    assign f_avail = (lf_w != lf_r);
    assign f_len   = lf[lf_r[LF_AW-1:0]];
    assign f_start = rp[RING_AW-1:0];

    typedef enum logic [1:0] {S_IDLE, S_PRE, S_DATA, S_DROP} st_t;
    st_t        st;
    reg [31:0]  crc;
    reg [13:0]  len;
    reg         er_seen;
    wire [RING_AW:0] ring_held = wp - rp;
    wire        ring_full = ring_held[RING_AW];

    always @(posedge clk) begin
        wen <= 1'b0;
        if (rst) begin
            st <= S_IDLE; wp <= 0; wp_commit <= 0; rp <= 0; lf_w <= 0; lf_r <= 0;
            c_ok <= 0; c_fcs <= 0; c_len <= 0; c_ovf <= 0; c_er <= 0;
            crc <= 32'hFFFFFFFF; len <= 0; er_seen <= 1'b0;
        end else begin
            if (f_release && f_avail) begin rp <= rp + lf[lf_r[LF_AW-1:0]]; lf_r <= lf_r + 1'b1; end
            case (st)
                S_IDLE: if (dv && stb) begin
                            crc <= 32'hFFFFFFFF; len <= 0; er_seen <= er;
                            if (data == 8'h55)      st <= S_PRE;
                            else if (data == 8'hD5) st <= S_DATA;
                            else                    st <= S_DROP;
                        end
                S_PRE:  if (!dv) st <= S_IDLE;
                        else if (stb) begin
                            if (er) er_seen <= 1'b1;
                            if (data == 8'hD5)      st <= S_DATA;
                            else if (data != 8'h55) st <= S_DROP;
                        end
                S_DATA: if (!dv) begin
                            st <= S_IDLE;
                            if (er_seen) begin c_er <= c_er + 1; wp <= wp_commit; end
                            else if (len < 14'd64) begin c_len <= c_len + 1; wp <= wp_commit; end
                            else if (crc != CRC_RESIDUE) begin c_fcs <= c_fcs + 1; wp <= wp_commit; end
                            else if (lf_full) begin c_ovf <= c_ovf + 1; wp <= wp_commit; end
                            else begin
                                c_ok <= c_ok + 1;
                                lf[lf_w[LF_AW-1:0]] <= len - 14'd4;
                                lf_w <= lf_w + 1'b1;
                                wp <= wp_commit + (len - 14'd4);
                                wp_commit <= wp_commit + (len - 14'd4);
                            end
                        end else if (stb) begin
                            if (er) er_seen <= 1'b1;
                            if (ring_full) begin c_ovf <= c_ovf + 1; st <= S_DROP; wp <= wp_commit; end
                            else if (len == MAX_FRAME[13:0]) begin c_len <= c_len + 1; st <= S_DROP; wp <= wp_commit; end
                            else begin
                                wdat <= data; wadr <= wp[RING_AW-1:0]; wen <= 1'b1;
                                wp   <= wp + 1'b1;
                                len  <= len + 1'b1;
                                crc  <= crc32_byte(crc, data);
                            end
                        end
                S_DROP: if (!dv) begin st <= S_IDLE; wp <= wp_commit; end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
