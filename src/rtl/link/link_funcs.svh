// link_funcs.svh — shared constants and constant functions for the board link (include inside modules).

// Training words: sent back to back by a framer whose far receiver is not
// trained, alternating TRAIN0/TRAIN1. Each is {~L, L}: every data wire toggles between the two
// halves of a word; L0 (5AC3, 8 ones) has even and L1 (5AC2, 7 ones) odd parity — a 16-bit
// complement keeps the parity — so the PAR wire toggles between the words. A half-swapped capture
// ({L, ~L} or a mix) matches neither word, which is what the trainer needs.
localparam [31:0] LINK_TRAIN0 = {~16'h5AC3, 16'h5AC3};   // A53C_5AC3
localparam [31:0] LINK_TRAIN1 = {~16'h5AC2, 16'h5AC2};   // A53D_5AC2
// Forward-lane (64-bit beat) training words: rising half = lo word, falling half = ~lo, so again
// every one of the 32 data wires toggles and both parity wires toggle between the two beats.
localparam [63:0] LINK_FTRAIN0 = {~LINK_TRAIN0, LINK_TRAIN0};
localparam [63:0] LINK_FTRAIN1 = {~LINK_TRAIN1, LINK_TRAIN1};

// MMCM geometry: the lane clock is VCO / D with D integer; VCO = 1000 MHz except 150 MHz (900).
function automatic int link_div(input int mhz);
    link_div = (mhz == 150) ? 6 : (1000 / mhz);
endfunction

// CRC-32, polynomial 0x04C11DB7, 32 data bits per step, MSB first, no reflection.
// Packet CRC = ~crc after feeding header+payload starting from 32'hFFFFFFFF.
function automatic [31:0] crc32_word(input [31:0] crc, input [31:0] data);
    integer b; reg [31:0] c; reg fb;
    begin
        c = crc;
        for (b = 31; b >= 0; b = b - 1) begin
            fb = c[31] ^ data[b];
            c  = {c[30:0], 1'b0} ^ (fb ? 32'h04C11DB7 : 32'h0);
        end
        crc32_word = c;
    end
endfunction

// xorshift32: a full-period (2^32-1) bijection on non-zero states. The test payload
// stream is s, f(s), f(f(s)), ...  so a receiver re-syncs from any single good word.
function automatic [31:0] xorshift32(input [31:0] s);
    reg [31:0] x;
    begin
        x = s;
        x = x ^ (x << 13);
        x = x ^ (x >> 17);
        x = x ^ (x << 5);
        xorshift32 = x;
    end
endfunction

// Packet types reserved by the link layer (header[31:24]): ARQ ACK / NAK control packets (the seq
// byte carries the receiver's next expected sequence) and the boundary word sent once after every
// link-up (the seq byte carries the transmitter's next sequence number, which the receiver adopts —
// a restarted board starts numbering from 0 again).
localparam [7:0] LINK_T_ACK = 8'hFF, LINK_T_NAK = 8'hFE, LINK_T_BND = 8'hFD;
// Fill word: sent between packets while there is nothing to send, so the lane never
// idles. It stands at a header position and carries LAST (no real header does; the boundary word is 0xFD),
// {0xFC, 24 bits of the training stream}; the receiver drops it before its FIFO, never through ARQ.
localparam [7:0] LINK_T_FILL = 8'hFC;
// Payload scrambler: every payload and CRC word of a packet is XORed with a keystream that
// restarts at the packet's header (xorshift32 from LINK_SSEED, one step per word: header + 1 uses the seed itself).
// Headers, boundary and fill words stay as they are. The CRC is computed over the unscrambled words. Unscrambled,
// smooth image data holds single wires static for long stretches and makes the 3.3 V lanes fail; scrambled, every
// lane carries data with the statistics of the (random) training stream its eye was measured with.
localparam [31:0] LINK_SSEED = 32'h6C8E9CF5;

// Second generator for the high half of a 64-bit training beat: g(x) = ~f(~x). xorshift32 is linear
// over GF(2), so g(x) = f(x) ^ ~f(32'hFFFFFFFF) differs from f(x) for every x — a half-swapped
// capture of the training stream therefore never passes the continuity check (see link_rx_meas).
function automatic [31:0] xorshift32b(input [31:0] s);
    xorshift32b = ~xorshift32(~s);
endfunction

// Training streams: while the far UP is low the framer sends a known
// pseudo-random sequence — data-like, so the receiver trains on the eye real traffic will see
// (a fixed toggling pattern measured a square-wave eye with no inter-symbol interference).
localparam [31:0] LINK_TSEED_LO = 32'h2545F491;      // seeds (must be non-zero)
localparam [31:0] LINK_TSEED_HI = 32'h9E3779B9;

function automatic [7:0] hex_digit(input [3:0] n);
    hex_digit = (n < 10) ? (8'h30 + n) : (8'h41 + n - 10);
endfunction
