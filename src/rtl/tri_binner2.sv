// tri_binner2 — board A's two-bank binner.
// Given a setup primitive that has already been written to its record (the caller supplies its index) and the band range
// [band_lo, band_hi] it covers, the binner APPENDS the primitive to each covered band's linked bin list (so head -> next
// walk == GL SUBMISSION ORDER, required for ES2 alpha blending):
//   * BIN_HEADS — ON-CHIP table, one entry per band and bank = {count, head, tail}. head = oldest entry (front of the
//     list), tail = newest. Empty when count == 0.
//   * BINLIST   — DDR linked-list pool of 8-byte entries {flags[7:0], next[23:0]}, {draw id, tri idx}, 4 entries per 32-byte
//     line. flags = {first[7], 0, TRI bursts[3:0]}: the entry's band is the primitive's first, where the render
//     side sends its TRI item, and the item's burst count — so a_render's fetcher can issue a visit's reads without waiting
//     for its header. They are written with the entry; the link write (below) covers the three bytes of next alone.
//     CHUNKED: a band's entries are allocated in chunks of 4 consecutive entries (one line); the next entry of a
//     band goes to the slot after its tail while the chunk has room, so the new entry AND the old tail's next field are
//     written in ONE masked burst (they share the line); a full chunk links to a fresh one (two writes). The explicit next
//     pointers stay, so the render side's walk (a_render: head -> next, count entries) does not depend on the chunks.
// TWO BANKS: the heads table and the pool exist twice; the geometry side (clear, push) works on bank push_bank
// while the render side's query port reads bank q_bank, so the next frame's binning runs while the previous frame is
// walked. Each bank has its own bump allocator (chunks), rewound by its clear. Each slot / field is written exactly once
// -> RMW-free.
`default_nettype none
module tri_binner2 #(
    parameter int NBANDS  = 400,            // max bands (e.g. 800 lines / NLINES=2)
    parameter int IDX_W   = 20,             // record index width
    parameter int DID_W   = 12,             // per-draw state-bucket id width; requires IDX_W+DID_W==32 (BINLIST low word)
    parameter int PTR_W   = 23,             // BINLIST entry index width (8M @ 64MB/8B)
    parameter int COUNT_W = 16,             // per-band primitive count
    parameter [28:0] BINLIST_BASE = 29'h1200000,  // MM_BINLIST (app_addr units): bank 0's pool
    parameter [28:0] POOL_W = 29'h800000,   // words per pool (32 MB: 2^22 entries of 8 B); bank 1's pool follows bank 0's
    parameter int POOL_ENTRIES = 1 << 22,   // entries a pool holds (8 B each, a multiple of 4): pushes beyond it are dropped and flagged
    localparam int BANDW  = $clog2(NBANDS+1)   // +1: holds the COUNT, not just an index
)(
    input  wire        clk, rst,
    // control
    input  wire        clear,               // sweep BIN_HEADS of bank push_bank empty (frame start)
    output wire        clear_done,
    output wire        busy,
    output reg         ovf,                 // sticky until clear: the pool overflowed or a band's count saturated (entries lost)
    input  wire        push_bank,           // the bank the geometry side builds (clear, pushes)
    // primitive input (one set-up primitive, already in its record)
    input  wire        push_valid,
    output wire        push_ready,
    input  wire [IDX_W-1:0]  tri_idx,
    input  wire [DID_W-1:0]  draw_id,             // per-draw state-bucket id (stamped into the BINLIST entry)
    input  wire [BANDW-1:0]  band_lo, band_hi,   // inclusive covered-band range
    input  wire [3:0]        tri_nb,              // the record's TRI bursts (after the header burst): into the entries' flags
    // BIN_HEADS query port (render phase) — q_* valid 1 cycle after q_band
    input  wire               q_bank,            // the bank the render side walks
    input  wire [BANDW-1:0]   q_band,
    output wire [PTR_W-1:0]    q_head,
    output wire [COUNT_W-1:0]  q_count,
    output wire                q_valid_head,   // band non-empty
    // MIG write master (to BINLIST, muxed by mig_arbiter)
    input  wire        can_start,
    output reg         w_en, output reg w_busy,
    output reg  [28:0] w_addr,
    output reg  [255:0] w_data, output reg [31:0] w_mask,
    output reg         w_wren, w_wend,
    input  wire        w_rdy, w_wdf_rdy,
    output wire [2:0]  dbg_st                    // debug: binner FSM state for status readback
);
    localparam int TW = COUNT_W + 2*PTR_W;      // BIN_HEADS entry = {count, head, tail}
    localparam [23:0] NULL24    = 24'hFFFFFF;    // end-of-list sentinel in entry.next

    // ---- BIN_HEADS table, both banks (on-chip, simple dual-port: FSM r/w on the geometry bank + query read on the render bank) ----
    (* ram_style="block" *) reg [TW-1:0] heads [0:2*NBANDS-1];
    reg [BANDW-1:0] a_addr; reg [TW-1:0] a_wdata; reg a_we; reg [TW-1:0] a_rdata;
    reg bank_r;                                   // the geometry bank of the job in progress (latched with the push / clear)
    // bank 1's entries follow bank 0's NBANDS entries (BANDW counts 0..NBANDS, so {bank, band} would overshoot the table)
    wire [BANDW:0] a_idx = {1'b0, a_addr} + (bank_r ? (BANDW+1)'(NBANDS) : '0);
    wire [BANDW:0] q_idx = {1'b0, q_band} + (q_bank ? (BANDW+1)'(NBANDS) : '0);
    always @(posedge clk) begin
        if (a_we) heads[a_idx] <= a_wdata;
        a_rdata <= heads[a_idx];
    end
    reg [TW-1:0] b_rdata;
    always @(posedge clk) b_rdata <= heads[q_idx];
    assign q_head       = b_rdata[2*PTR_W-1:PTR_W];
    assign q_count      = b_rdata[TW-1:2*PTR_W];
    assign q_valid_head = (b_rdata[TW-1:2*PTR_W] != 0);

    // ---- chunk allocators (one per bank, in entries: a chunk = 4) + the latched push job ----
    reg [PTR_W-1:0]  next_free [0:1];
    reg [PTR_W-1:0]  ptr_r, old_head_r, old_tail_r;
    reg [COUNT_W-1:0] old_cnt_r;
    reg              link_far;                    // the old tail is in another line (a new chunk): its next needs its own write
    reg [IDX_W-1:0]  tri_r;
    reg [DID_W-1:0]  did_r;                       // per-draw state-bucket id, latched with the primitive
    reg [BANDW-1:0]  band_r, band_hi_r, band_lo_r;
    reg [3:0]        nb_r;                        // the TRI bursts, latched with the primitive
    wire [PTR_W-1:0] nf = next_free[bank_r];
    wire [28:0] pool_base = BINLIST_BASE + (bank_r ? POOL_W : 29'd0);

    // the new entry {flags, next = NULL, {draw id, tri idx}} at lane ptr_r[1:0] of its line; when the old tail shares the
    // line (ptr_r = tail + 1 inside the chunk) its next field (lane_t, the high word's three low bytes) is written in the
    // same burst
    wire [7:0]  flags_new = {band_r == band_lo_r, 3'd0, nb_r};
    wire [63:0] entry_new = {flags_new, NULL24, did_r, tri_r};   // requires IDX_W+DID_W==32
    wire [1:0]  lane_n    = ptr_r[1:0];
    wire [1:0]  lane_t    = old_tail_r[1:0];
    wire        merge     = (old_cnt_r != 0) && !link_far;
    wire [255:0] d_new  = ({192'd0, entry_new} << (lane_n*32'd64));
    wire [255:0] d_link = ({232'd0, {{(24-PTR_W){1'b0}}, ptr_r}} << (lane_t*32'd64 + 32'd32));
    wire [31:0]  m_new  = ~(32'h000000FF << (lane_n*32'd8));
    wire [31:0]  m_link = ~(32'h00000007 << (lane_t*32'd8 + 32'd4));   // next's three bytes: the flags byte stays

    // ---- the write engine: one burst job at a time (address / data / mask loaded by the FSM, which goes on with the next
    //      band's heads meanwhile); w_busy until its command and data are accepted
    reg cmd_ack, wdf_ack; reg ld_job; reg [28:0] ld_addr; reg [255:0] ld_data; reg [31:0] ld_mask;
    always @(posedge clk) begin
        if (rst) begin w_en<=0; w_busy<=0; w_wren<=0; w_wend<=0; w_mask<=32'hffffffff; cmd_ack<=0; wdf_ack<=0; end
        else if (ld_job) begin
            w_en<=1'b1; w_busy<=1'b1; w_addr<=ld_addr; w_data<=ld_data; w_mask<=ld_mask; w_wren<=1'b1; w_wend<=1'b1; cmd_ack<=0; wdf_ack<=0;
        end else if (w_busy) begin
            if (w_en   && w_rdy)     begin w_en<=0;   cmd_ack<=1; end
            if (w_wren && w_wdf_rdy) begin w_wren<=0; w_wend<=0; wdf_ack<=1; end
            if ((cmd_ack||(w_en&&w_rdy)) && (wdf_ack||(w_wren&&w_wdf_rdy))) begin w_busy<=0; cmd_ack<=0; wdf_ack<=0; end
        end
    end

    typedef enum logic [2:0] {B_IDLE,B_CLR,B_RD,B_RD2,B_WR_NEW,B_WR_LINK,B_UPD,B_UPD2} st_t; st_t st;
    reg [BANDW:0] csweep;                       // clear sweep counter (0..NBANDS)

    assign busy       = (st != B_IDLE) | w_busy | ld_job;
    assign dbg_st     = st;                      // debug tap for the network status readback
    assign push_ready = (st == B_IDLE) & ~clear;
    assign clear_done = (st == B_IDLE);

    always @(posedge clk) begin
        if (rst) begin
            st<=B_IDLE; a_we<=0; ld_job<=0;
            next_free[0]<=0; next_free[1]<=0; bank_r<=1'b0; ovf<=1'b0;
        end else begin
            a_we<=0; ld_job<=0;
            case (st)
                B_IDLE: begin
                    // Frame start: empty the bank's bins AND rewind its allocator (the pool is per frame: each frame's
                    // entries are consumed by the render before the bank's next clear)
                    if (clear) begin csweep<=0; next_free[push_bank]<=0; bank_r<=push_bank; ovf<=1'b0; st<=B_CLR; end
                    // a push with a full pool (the primitive needs up to band_hi - band_lo + 1 chunks; checked per band
                    // below as well) is taken and dropped: the handshake completes, the flag records the loss
                    else if (push_valid && next_free[push_bank] >= PTR_W'(POOL_ENTRIES)) ovf<=1'b1;
                    else if (push_valid) begin
                        tri_r<=tri_idx; did_r<=draw_id; band_r<=band_lo; band_hi_r<=band_hi; band_lo_r<=band_lo; nb_r<=tri_nb; bank_r<=push_bank;
                        a_addr<=band_lo; st<=B_RD;          // kick read of first band's entry
                    end
                end
                B_CLR: begin                                 // BIN_HEADS[*] <- {0,0,0} (empty)
                    a_we<=1'b1; a_wdata<={TW{1'b0}}; a_addr<=csweep[BANDW-1:0];
                    if (csweep==NBANDS) begin a_we<=0; st<=B_IDLE; end
                    else csweep<=csweep+1'b1;
                end
                B_RD:  st<=B_RD2;                            // BRAM read latency
                B_RD2: begin : RD2                           // a_rdata = {count, head, tail}: where the new entry goes
                    reg [PTR_W-1:0] tl; reg [COUNT_W-1:0] cn; reg room;
                    tl = a_rdata[PTR_W-1:0]; cn = a_rdata[TW-1:2*PTR_W];
                    room = (cn != 0) && (tl[1:0] != 2'b11);               // the band's chunk has a free slot after its tail
                    old_tail_r<=tl; old_head_r<=a_rdata[2*PTR_W-1:PTR_W]; old_cnt_r<=cn;
                    // no room for this band's entry (pool full for a new chunk, or the band's count would wrap): drop the rest
                    // of the primitive's bands and flag it (the render would walk count mod 2^COUNT_W entries otherwise)
                    if ((!room && nf >= PTR_W'(POOL_ENTRIES)) || (&cn)) begin ovf<=1'b1; st<=B_IDLE; end
                    else begin
                        if (room) begin ptr_r<=tl+1'b1; link_far<=1'b0; end
                        else begin ptr_r<=nf; next_free[bank_r]<=nf+PTR_W'(4); link_far<=(cn != 0); end
                        st<=B_WR_NEW;
                    end
                end
                B_WR_NEW: if (!w_busy && !ld_job && can_start) begin   // BINLIST[ptr_r] = {NULL, tri} (+ the old tail's next when it shares the line)
                    ld_job<=1'b1; ld_addr<=pool_base + ({6'd0, ptr_r[PTR_W-1:2]} << 3);
                    ld_data<=merge ? (d_new | d_link) : d_new; ld_mask<=merge ? (m_new & m_link) : m_new;
                    st<=link_far ? B_WR_LINK : B_UPD;            // the first entry needs no link; a shared line carried it
                end
                B_WR_LINK: if (!w_busy && !ld_job && can_start) begin   // old_tail.next <- ptr_r (4-byte masked) in the previous chunk's line
                    ld_job<=1'b1; ld_addr<=pool_base + ({6'd0, old_tail_r[PTR_W-1:2]} << 3); ld_data<=d_link; ld_mask<=m_link;
                    st<=B_UPD;
                end
                B_UPD: begin                                 // BIN_HEADS[band] <- {count+1, head, tail=ptr}
                    a_we<=1'b1; a_addr<=band_r;
                    a_wdata<={ old_cnt_r+1'b1, (old_cnt_r==0)?ptr_r:old_head_r, ptr_r };
                    st<=B_UPD2;
                end
                B_UPD2: begin                                // BIN_HEADS write committed
                    if (band_r==band_hi_r) st<=B_IDLE;
                    else begin band_r<=band_r+1'b1; a_addr<=band_r+1'b1; st<=B_RD; end
                end
                default: st<=B_IDLE;
            endcase
        end
    end
endmodule
`default_nettype wire
