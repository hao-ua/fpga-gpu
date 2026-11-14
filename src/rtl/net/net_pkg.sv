// net_pkg — shared Ethernet helpers for the board-A network stack.
//   * CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320). Register starts at 0xFFFFFFFF; the FCS sent on
//     the wire is ~crc, least significant byte first. Running the register over a frame *including* its
//     FCS leaves the residue 0xDEBB20E3. Check value: ~crc over "123456789" = 0xCBF43926.
//   * One's-complement 16-bit add used by the IPv4 header checksum.
//   * Word kinds of the 36-bit datagram FIFOs between the Ethernet clock domain and clk_sys:
//       {kind[1:0], nvalid[1:0], data[31:0]}  kind: 2 = descriptor d0, 3 = descriptor d1..d3,
//       0 = payload, 1 = last payload word; nvalid = valid bytes - 1 (meaningful on the last word).
//     Descriptor (both directions): d0 = {port[15:0], len[15:0]} (UDP payload bytes),
//       d1 = IPv4 address, d2 = MAC[31:0], d3 = {16'h0, MAC[47:32]}.
//     Payload bytes are packed little-endian: stream byte k of a word -> bits [8k+7:8k].
`ifndef NET_PKG_SV
`define NET_PKG_SV
package net_pkg;
    localparam [31:0] CRC_RESIDUE = 32'hDEBB20E3;
    localparam [1:0]  K_PAY = 2'd0, K_LAST = 2'd1, K_D0 = 2'd2, K_DN = 2'd3;

    function automatic [31:0] crc32_byte(input [31:0] crc, input [7:0] d);
        reg [31:0] c; integer i;
        begin
            c = crc;
            for (i = 0; i < 8; i = i + 1) c = (c[0] ^ d[i]) ? ((c >> 1) ^ 32'hEDB88320) : (c >> 1);
            crc32_byte = c;
        end
    endfunction

    function automatic [15:0] oc_add(input [15:0] a, input [15:0] b);
        reg [16:0] s;
        begin s = a + b; oc_add = s[15:0] + s[16]; end
    endfunction
endpackage
`endif
