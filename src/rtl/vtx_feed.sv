// vtx_feed — board A's vertex fetch and warp formation (a_vertex.sv). The gather is a pipeline with many DDR reads in
//   flight.
//   For each index of a draw (u8 / u16 from the index buffer at byte address dr_idx_base, or first + i for arrays; 32-bit
//   first / count) the vertex cache — 32 entries, direct-mapped by the index's low 5 bits, tag = the upper 27 bits,
//   flushed per draw — answers with the vertex's reference {result slot, lane} on a hit; on a miss the vertex takes the next
//   lane of the warp being formed and its 16 attributes are gathered and written into the vertex core's lane (va_*), and
//   the cache takes the entry. Every index produces one reference on the ref stream, in index order.
//   Three parts, decoupled by queues so that the DDR latency overlaps everything else:
//   * the INDEX PREFETCHER: two burst buffers; the burst holding the next index is fetched, then the one after it.
//   * the ISSUE side: per miss it walks the 16 attribute descriptors (one per cycle through a 2-stage address pipe:
//     byte address = base + offset + index * stride, mod 2^30), looks every burst an enabled attribute touches (one or
//     two: a component span is <= 16 bytes) up in the BURST CACHE (NCE fully associative entries of 32 bytes with a
//     reference lock), issues a DDR read for a missing one and allocates an entry for it (any unlocked, round robin),
//     and pushes a JOB {lane, attribute, format, byte offset, the entries} into the job FIFO; a disabled attribute pushes
//     a CONST job. Launch requests (16 lanes, the draw's end, or the assembler's FIFO >= REF_THR references while the warp
//     has lanes: the assembler waits for the forming warp's results, so without this the two would deadlock) are jobs too,
//     so they stay ordered behind the lanes' writes. The next result slot is ANY slot vtx_rbuf reports free (not ring
//     order: the assembler may hold a reference for a whole draw — a fan's first vertex — and waiting for that very slot
//     would deadlock); the cache entries that pointed at the slot's previous use are invalidated then.
//   * the BUILD side: pops jobs in order; an ATTR job waits for its bursts' data, selects the component bytes out of the
//     two-burst window (any byte offset), converts the components to FP24 (attr_conv: BYTE / UNSIGNED_BYTE / SHORT /
//     UNSIGNED_SHORT / FIXED / FLOAT / FP24, normalized or not — ES 2.0 §2.8) and writes the vec4 (components beyond the
//     attribute's size read (0, 0, 0, 1)) into the core's lane, one job per cycle; a CONST job writes the attribute's
//     constant (ES 2.0 §2.7: (0, 0, 0, 1) until the host sets one); a LAUNCH job waits until every earlier write has been
//     made and the core's warp slot is free, then launches (in_start). DDR reads return in issue order (mem_arb): a small
//     FIFO of destinations routes each burst to its cache entry or index buffer.
//   DRAW OVERLAP: a draw starts (dr_start) as soon as the issue side is idle, while the previous draw's
//   jobs, warps and primitives are still in flight. The attribute descriptors the host writes (ad_* / ac_*) are a SHADOW
//   copied into the draw's bank (dr_bank) at its start (16 cycles; the previous draw's jobs read their own bank), the
//   LAUNCH job carries the draw's bank (the core's uniform bank), varmask and point-size flag to the core and vtx_rbuf.
//   DDR addresses are 32-bit words (the MIG app_addr unit), a burst = 8 words = 32 bytes; buffers are byte addressed here.
`default_nettype none
module vtx_feed #(
    parameter int W = 24,
    parameter int NSLOT = 8,                     // result slots of vtx_rbuf
    parameter int REF_THR = 24,                  // launch the forming warp when the assembler's FIFO holds this many refs
    parameter int NCE = 16,                      // burst cache entries (<= 16: 4-bit ids in the job)
    parameter int JQ = 64                        // job FIFO depth (a power of 2)
)(
    input  wire        clk, rst,
    // draw (dr_start while issue_idle; draws overlap — the descriptors are copied into the draw's bank
    // first, the launch jobs carry the draw's bank / varmask / point-size flag to the core and vtx_rbuf)
    input  wire        dr_start,
    input  wire        dr_indexed, dr_u8,
    input  wire [29:0] dr_idx_base,              // byte address of index 0
    input  wire [31:0] dr_first, dr_count,
    input  wire        dr_bank,                  // the draw's uniform / descriptor bank (its parity)
    input  wire [7:0]  dr_varmask,               // the varyings its warps' results need (the fragment shader's read set)
    input  wire        dr_ps,                    // a POINTS draw: the gl_PointSize row is copied too
    output wire        busy,
    output wire        issue_idle,               // the next draw may start (its jobs queue behind this one's)
    output wire        dcopy_busy,               // the descriptor shadow is being copied: no VERTEX_ATTR write meanwhile
    // attribute descriptor table (the shadow: the host's current state; copied into the draw's bank at its start)
    input  wire        ad_we,
    input  wire [3:0]  ad_idx,
    input  wire        ad_en,
    input  wire [2:0]  ad_type,                  // attr_conv's type codes
    input  wire        ad_norm,
    input  wire [2:0]  ad_size,                  // components 1..4
    input  wire [29:0] ad_base, ad_stride, ad_off,   // bytes
    input  wire        ac_we,
    input  wire [3:0]  ac_idx,
    input  wire [4*W-1:0] ac_const,
    // DDR read master (mem_arb port)
    output wire        r_en,
    output wire [28:0] r_addr,
    input  wire        r_rdy, r_valid,
    input  wire [255:0] rd_data,
    // vertex core: attributes of the warp being filled, launch
    output reg         va_we,
    output reg  [3:0]  va_lane, va_idx,
    output reg  [4*W-1:0] va_data,
    input  wire        in_ready,
    output reg         in_start,
    output reg         in_bank,                  // the launching warp's uniform bank
    output reg  [15:0] in_active,
    // result slots (vtx_rbuf): the launch order, the allocation of a slot, which slots are free
    output reg         launch,                   // with in_start: slot launch_slot runs in the core next
    output reg  [$clog2(NSLOT)-1:0] launch_slot,
    output reg  [7:0]  launch_varmask,           // its draw's copy set
    output reg         launch_ps,
    output reg         alloc,                    // slot alloc_slot is taken for the next warp (its done flag clears)
    output reg  [$clog2(NSLOT)-1:0] alloc_slot,
    input  wire [NSLOT-1:0] slot_free,
    // reference stream (one per index, in index order) to the assembler
    output reg         ref_valid,
    output reg  [$clog2(NSLOT)-1:0] ref_slot,
    output reg  [3:0]  ref_lane,
    input  wire        ref_ready,
    input  wire [5:0]  ref_cnt                   // the assembler's FIFO occupancy
);
    localparam int SW = $clog2(NSLOT), CW = $clog2(NCE), JW = $clog2(JQ);
    localparam [W-1:0] ONE1 = ((1 << 6) - 1) << 16;         // FP24 1.0 = 0x3F0000 (W = 24)
    localparam [1:0] K_ATTR = 2'd0, K_CONST = 2'd1, K_LAUNCH = 2'd2;
    localparam [2:0] T_FP24 = 3'd6;
    // ================================================================ descriptor table: the shadow (the host's state) and two banks
    reg  [2:0]  s_ty [0:15]; reg s_nm [0:15]; reg [2:0] s_sz [0:15];
    reg  [29:0] s_bo [0:15], s_strd [0:15];      // base + offset (bytes), stride (bytes)
    reg  [4*W-1:0] s_const [0:15];
    reg  [15:0] en_mask, cv_mask;                // enabled; a constant was written (else the ES 2.0 default (0, 0, 0, 1))
    always @(posedge clk) begin
        if (ad_we) begin s_ty[ad_idx] <= ad_type; s_nm[ad_idx] <= ad_norm; s_sz[ad_idx] <= ad_size; s_bo[ad_idx] <= ad_base + ad_off; s_strd[ad_idx] <= ad_stride; end
        if (ac_we) s_const[ac_idx] <= ac_const;
    end
    always @(posedge clk) begin
        if (rst) begin en_mask <= 16'd0; cv_mask <= 16'd0; end
        else begin
            if (ad_we) en_mask[ad_idx] <= ad_en;
            if (ac_we) cv_mask[ac_idx] <= 1'b1;
        end
    end
    reg         d_en [0:31]; reg [2:0] d_ty [0:31]; reg d_nm [0:31]; reg [2:0] d_sz [0:31];   // {bank, attribute}
    reg  [29:0] d_bo [0:31], d_strd [0:31];
    (* ram_style = "block" *) reg [4*W-1:0] d_const [0:31];   // the constant resolved (the default when none was written);
                                                 // block RAM: its one read lands in p1_cval at the pop
    reg         dbank, dps; reg [7:0] dvm;       // the draw's bank, point-size flag, varmask
    reg  [3:0]  cp_i;                            // the copy's attribute
    // ================================================================ draw state, vertex cache
    reg         indexed, u8; reg [31:0] first, count, i;
    reg  [31:0] vidx;                            // the resolved vertex index
    reg  [SW-1:0] cs;                            // the forming warp's result slot
    reg  [4:0]  cl;                              // lanes filled (0..16)
    reg         end_pend;                        // after the launch: the draw is over (no next index)
    reg  [31:0] pv; reg [26:0] ptag [0:31]; reg [SW-1:0] pslot [0:31]; reg [3:0] plane [0:31];
    wire hit = pv[vidx[4:0]] & (ptag[vidx[4:0]] == vidx[31:5]);
    typedef enum logic [2:0] {I_IDLE, I_COPY, I_IDX, I_PTQ, I_GATH, I_LAUNCH, I_NEWSLOT} ist_t;
    ist_t ist;
    assign issue_idle = (ist == I_IDLE);
    assign dcopy_busy = (ist == I_COPY);
    // the next slot: ANY free one other than the forming slot
    reg [SW-1:0] ns; reg ns_ok;
    always @* begin
        ns_ok = 1'b0; ns = '0;
        for (int k = NSLOT-1; k >= 0; k--) if (slot_free[k] && k != int'(cs)) begin ns_ok = 1'b1; ns = SW'(k); end
    end
    // ================================================================ index prefetcher (two burst buffers)
    // the byte position of index i = ib + i (u8) / 2 i (u16), kept as a REGISTER stepped with i (timing at 120 MHz: no
    // adder in front of the two tag compares and the 64 : 1 byte select)
    reg  [29:0] ip;
    wire [24:0] ibu = ip[29:5], ibu1 = ip[29:5] + 25'd1;
    wire        straddle = ~u8 & (ip[4:0] == 5'd31);           // a u16 across the burst boundary
    reg  [1:0]  bs_v, bs_p; reg [24:0] bs_tag [0:1]; reg [255:0] bs_d [0:1];
    wire have0 = bs_v[ibu[0]] & (bs_tag[ibu[0]] == ibu);
    wire have1 = bs_v[ibu1[0]] & (bs_tag[ibu1[0]] == ibu1);
    wire [511:0] iwin = {bs_d[~ibu[0]], bs_d[ibu[0]]};
    wire [15:0] iraw = iwin[ip[4:0]*8 +: 16];
    wire [31:0] idx_val = !indexed ? first + i : u8 ? {24'd0, iraw[7:0]} : {16'd0, iraw};
    wire idx_ready = !indexed | (have0 & (~straddle | have1));
    wire ix_act = indexed & (ist != I_IDLE) & (i != count);
    wire ix_want0 = ix_act & ~have0 & ~bs_p[ibu[0]];
    wire ix_want1 = ix_act & ~have1 & ~bs_p[ibu1[0]] & (have0 | bs_p[ibu[0]]);
    wire ix_req_c = ix_want0 | ix_want1;
    wire [24:0] ix_tag_c = ix_want0 ? ibu : ibu1;
    // The request the arbiter sees is REGISTERED (it keeps the tag compare off r_en -> the arbiter's grant chain, which
    // reaches every port's ready). It is the decision of the previous cycle; its go marks the buffer pending at
    // the next edge, so the decision made in the go's cycle (which still sees the buffer free) is masked for that buffer.
    reg         ix_req; reg [24:0] ix_tag;
    // ================================================================ burst cache
    reg  [NCE-1:0] c_v, c_p;                     // data present / read pending
    reg  [NCE-1:0] c_cur;                        // allocated during the current draw: only these can be hit. A draw's start
                                                 // clears it (and bs_v): the host may have rewritten the buffers since the
                                                 // previous draw (the dispatcher lets a write into A's memory land only
                                                 // while no geometry runs), so nothing cached before it may be served again
                                                 // (else new buffer contents at a previous draw's addresses would be read
                                                 // from the cache). A stale entry a queued job still holds (lock)
                                                 // keeps its data until that job is built, then is reallocated like any other.
    reg  [24:0] c_tag [0:NCE-1];
    reg  [7:0]  c_lock [0:NCE-1];                // jobs referencing the entry, not built yet
    (* ram_style = "distributed" *) reg [255:0] c_d [0:NCE-1];
    reg  [CW-1:0] rr;                            // round-robin allocation pointer
    // ================================================================ the issue side's address pipe and lookup
    reg  [4:0]  ga;                              // the attribute entering the pipe (0..16)
    wire        g_run = (ist == I_GATH);
    reg         s0_v, s0_en, s0_nm; reg [3:0] s0_a; reg [2:0] s0_ty, s0_sz; reg [29:0] s0_prod, s0_bo;
    reg         s1_v, s1_en, s1_nm; reg [3:0] s1_a; reg [2:0] s1_ty, s1_sz; reg [29:0] s1_addr, s1_addr1;   // s1_addr1 = s1_addr + 32 (the next burst)
    wire [1:0]  s1_esz_l = (s1_ty[2:1] == 2'b00) ? 2'd0 : (s1_ty[2:1] == 2'b01) ? 2'd1 : 2'd2;   // log2 of the element size
    wire [5:0]  s1_span = 6'(s1_sz) << s1_esz_l;
    wire        s1_cross = (6'(s1_addr[4:0]) + s1_span) > 6'd32;
    reg         l_ph; reg [CW-1:0] l_e0;         // lookup phase (0: the first burst, 1: the second), the first's entry
    // both bursts' tags come registered from the pipe; the phase only selects (timing: no +l_ph adder in front of the
    // 16 comparators)
    wire [24:0] lk_tag = l_ph ? s1_addr1[29:5] : s1_addr[29:5];
    reg         li_v; reg [CW-1:0] li_e;         // the lookup's lock, counted the cycle after (off the arbiter's ready path)
    reg  [NCE-1:0] cm; reg cm_any; reg [CW-1:0] cm_id;           // tag match (present or pending)
    reg  [NCE-1:0] cf; reg cf_any; reg [CW-1:0] cf_id;           // allocatable: unlocked and not pending, from rr on
    always @* begin
        cm_any = 1'b0; cm_id = '0; cf_any = 1'b0; cf_id = '0;
        for (int k = 0; k < NCE; k++) begin
            cm[k] = c_cur[k] & (c_v[k] | c_p[k]) & (l_ph ? (c_tag[k] == s1_addr1[29:5]) : (c_tag[k] == s1_addr[29:5]));
            cf[k] = ~c_p[k] & (c_lock[k] == 8'd0) & ~(li_v & (li_e == CW'(k)));
            if (cm[k]) begin cm_any = 1'b1; cm_id = CW'(k); end
        end
        for (int k = NCE-1; k >= 0; k--) begin : RR            // the first allocatable entry at or after rr
            reg [CW-1:0] e; e = CW'(rr + CW'(k));
            if (cf[e]) begin cf_any = 1'b1; cf_id = e; end
        end
    end
    // job FIFO
    (* ram_style = "distributed" *) reg [31:0] jq [0:JQ-1];
    reg  [JW-1:0] jq_wp, jq_rp; reg [JW:0] jq_cnt;
    wire jq_space = (jq_cnt < (JW+1)'(JQ - 2));                // the push is registered: a decision may follow before the count sees it
    reg  jq_push; reg [31:0] jq_wd; wire jq_pop;
    function automatic [31:0] mkjob(input [1:0] kind, input [3:0] lane, input [3:0] a, input [2:0] ty, input nm, input [2:0] sz,
                                    input [4:0] o, input [3:0] e0, input [3:0] e1, input two);
        mkjob = {1'b0, two, e1, e0, o, sz, nm, ty, a, lane, kind};
    endfunction
    // the lookup: a hit (present or pending) or an allocation + read issue; it needs job space (the push may follow at once)
    wire lk_act  = s1_v & s1_en & jq_space;
    wire lk_req  = lk_act & ~cm_any & cf_any;                   // a read to issue
    // the request the arbiter sees is REGISTERED, like the prefetcher's: it keeps the 16 tag compares -> lk_req off the
    // arbiter's grant chain -> every port's ready (timing at 120 MHz). lk_req_r holds
    // last cycle's request while it was not granted; the go needs it AND this cycle's lk_act (the stage is held meanwhile,
    // so a request once raised stays valid: the tags only change at this unit's own go, locks and pending bits can only
    // free entries; job space is the one term that may close, hence lk_act). The miss's read goes out a cycle later.
    // EVERY READ THE ARBITER TAKES MUST BE COUNTED: r_en carries lk_req_r, so a standing request whose job space has
    // closed would still be granted and read by the MIG, but lk_go would not count it — no return-order entry (rq), its
    // burst would pop the next read's destination and every later return would land one place off (wrong bursts in the
    // cache, the last destination pending for ever: the feed waiting on an index burst, the assembler on its
    // references). So a request stands only if job space HOLDS next cycle: jq_cnt next cycle <=
    // jq_cnt + jq_push (the build side's pops only lower it), so (jq_cnt + jq_push) < JQ - 2 now guarantees jq_space then,
    // and with the stage held lk_act = lk_req_r; lk_go = r_en & r_rdy & ~ix_req. The term is a register-only AND
    // on lk_req_r's D, beside the tag compare, off the grant chain (a jq_space term in r_en would put it on that chain).
    wire jq_space_nx = ((jq_cnt + (JW+1)'(jq_push)) < (JW+1)'(JQ - 2));
    reg  lk_req_r;
    wire lk_go   = lk_req_r & lk_act & ~ix_req & r_rdy;
    wire lk_done = lk_act & (cm_any | lk_go);
    // the entry a miss allocates is last cycle's cf_id, REGISTERED with the request (timing at 120 MHz: it keeps the
    // lock compares -> the 16-way round-robin search off the entry decode into the tags' write enables).
    // Valid whenever lk_req_r is: a request stands only when no lookup completed last cycle (a go
    // clears lk_req_r, a hit means no request), so nothing has locked or taken that entry since; entries can only have
    // become free meanwhile (another one might be chosen now, any allocatable entry is correct).
    reg  [CW-1:0] cf_id_r;
    wire [CW-1:0] lk_e = cm_any ? cm_id : cf_id_r;
    wire take_en = lk_done & (l_ph | ~s1_cross);                // the attribute's job goes out this cycle
    wire take_cn = s1_v & ~s1_en & jq_space;
    wire take    = take_en | take_cn;
    wire adv     = ~s1_v | take;
    // DDR read port: the index prefetcher first, then the lookup
    assign r_en   = ix_req | lk_req_r;
    assign r_addr = {1'b0, ix_req ? ix_tag : lk_tag, 3'b000};
    wire ix_go = ix_req & r_rdy;
    // the return-order FIFO: {index buffer (1) / cache entry (0), id}
    reg  [4:0] rq [0:31]; reg [4:0] rq_wp, rq_rp;
    wire [4:0] rq_head = rq[rq_rp];
    // ================================================================ the build side
    wire jv = (jq_cnt != '0);
    wire [31:0] jh = jq[jq_rp];
    wire [1:0] j_kind = jh[1:0]; wire [3:0] j_lane = jh[5:2], j_a = jh[9:6]; wire [2:0] j_ty = jh[12:10]; wire j_nm = jh[13];
    wire [2:0] j_sz = jh[16:14]; wire [4:0] j_o = jh[21:17]; wire [3:0] j_e0 = jh[25:22], j_e1 = jh[29:26]; wire j_two = jh[30];
    reg  [7:0] inflight;                         // jobs popped and not yet written
    wire p0_attr   = jv & (j_kind == K_ATTR) & c_v[j_e0[CW-1:0]] & (~j_two | c_v[j_e1[CW-1:0]]) & in_ready & ~in_start;
    wire p0_const  = jv & (j_kind == K_CONST) & in_ready & ~in_start;
    wire p0_launch = jv & (j_kind == K_LAUNCH) & (inflight == 8'd0) & in_ready & ~in_start;
    assign jq_pop = p0_attr | p0_const | p0_launch;
    wire [511:0] cwin = {c_d[j_e1[CW-1:0]], c_d[j_e0[CW-1:0]]};
    wire [159:0] csel = cwin[j_o[4:2]*32 +: 160];
    reg  p1_v, p1_const, p1_nm; reg [3:0] p1_lane, p1_a; reg [2:0] p1_ty, p1_sz; reg [1:0] p1_o; reg [159:0] p1_sel; reg [4*W-1:0] p1_cval;
    // a CONST job's value (nm = the draw's bank): p1_cval is the block RAM's output register, loaded at the pop
    always @(posedge clk) if (p0_attr | p0_const) p1_cval <= d_const[{j_nm, j_a}];
    wire [127:0] w128 = p1_sel[p1_o*8 +: 128];
    wire [1:0] p1_esz_l = (p1_ty[2:1] == 2'b00) ? 2'd0 : (p1_ty[2:1] == 2'b01) ? 2'd1 : 2'd2;
    wire [31:0] craw [0:3]; wire [W-1:0] cres [0:3]; wire [3:0] cres_v;
    genvar gk;
    generate for (gk = 0; gk < 4; gk = gk + 1) begin : CV
        assign craw[gk] = p1_const ? {{(32-W){1'b0}}, p1_cval[gk*W +: W]} :
                          (p1_esz_l == 2'd0) ? {24'd0, w128[gk*8 +: 8]} : (p1_esz_l == 2'd1) ? {16'd0, w128[gk*16 +: 16]} : w128[gk*32 +: 32];
        attr_conv #(.EXP_W(7), .MAN_W(W-8)) u_cv (.clk, .rst, .in_valid(p1_v), .raw(craw[gk]), .ty(p1_const ? T_FP24 : p1_ty), .nm(p1_nm),
            .out_valid(cres_v[gk]), .result(cres[gk]));
    end endgenerate
    reg  [3:0] dl_lane [0:5], dl_a [0:5]; reg [2:0] dl_sz [0:5];     // the job's lane / attribute / size alongside the converters (latency 6)
    wire wr_now = cres_v[0];
    assign busy = (ist != I_IDLE) | jv | (inflight != 8'd0) | in_start | va_we | ref_valid | jq_push;
    // ================================================================ sequential
    always @(posedge clk) begin
        if (rst) begin
            ist <= I_IDLE; ref_valid <= 1'b0; in_start <= 1'b0; launch <= 1'b0; alloc <= 1'b0; va_we <= 1'b0;
            pv <= 32'd0; cs <= '0; cl <= 5'd0; end_pend <= 1'b0; i <= 32'd0; count <= 32'd0; indexed <= 1'b0; u8 <= 1'b0;
            bs_v <= 2'b00; bs_p <= 2'b00; c_v <= '0; c_p <= '0; c_cur <= '0; rr <= '0; for (int k = 0; k < NCE; k++) c_lock[k] <= 8'd0;
            ga <= 5'd16; s0_v <= 1'b0; s1_v <= 1'b0; l_ph <= 1'b0; li_v <= 1'b0; ix_req <= 1'b0; lk_req_r <= 1'b0;
            jq_wp <= '0; jq_rp <= '0; jq_cnt <= '0; jq_push <= 1'b0; rq_wp <= 5'd0; rq_rp <= 5'd0;
            inflight <= 8'd0; p1_v <= 1'b0; ip <= 30'd0;
        end else begin
            in_start <= 1'b0; launch <= 1'b0; alloc <= 1'b0; va_we <= 1'b0; jq_push <= 1'b0;
            if (ref_valid & ref_ready) ref_valid <= 1'b0;
            // ---------------- DDR: the prefetcher's and the lookup's requests (registered), issue bookkeeping and the returns
            ix_req <= ix_req_c & ~(ix_go & (ix_tag_c[0] == ix_tag[0])); ix_tag <= ix_tag_c;
            lk_req_r <= lk_req & ~lk_go & jq_space_nx; cf_id_r <= cf_id;   // (jq_space_nx: a standing request can always complete)
            if (ix_go) begin bs_p[ix_tag[0]] <= 1'b1; bs_v[ix_tag[0]] <= 1'b0; bs_tag[ix_tag[0]] <= ix_tag; end
            if (ix_go | lk_go) begin rq[rq_wp] <= ix_go ? {1'b1, 3'd0, ix_tag[0]} : {1'b0, 4'(lk_e)}; rq_wp <= rq_wp + 5'd1; end
            if (r_valid) begin
                rq_rp <= rq_rp + 5'd1;
                if (rq_head[4]) begin bs_d[rq_head[0]] <= rd_data; bs_v[rq_head[0]] <= 1'b1; bs_p[rq_head[0]] <= 1'b0; end
                else begin c_d[rq_head[CW-1:0]] <= rd_data; c_v[rq_head[CW-1:0]] <= 1'b1; c_p[rq_head[CW-1:0]] <= 1'b0; end
            end
            // ---------------- the lookup's allocation and locks (the build side's unlock may hit the same entry: net count);
            // the lock is counted the cycle after the lookup (li_*; cf excludes that entry meanwhile) and lands before the
            // job can be popped (the push is registered too, so the job is in the FIFO a cycle later still)
            li_v <= lk_done; li_e <= lk_e;
            for (int k = 0; k < NCE; k++) begin : LK
                reg [1:0] inc, dec;
                inc = 2'(li_v && li_e == CW'(k));
                dec = 2'(p0_attr && j_e0[CW-1:0] == CW'(k)) + 2'(p0_attr && j_two && j_e1[CW-1:0] == CW'(k));
                c_lock[k] <= c_lock[k] + 8'(inc) - 8'(dec);
            end
            // a go is always a miss (a hit stops the request), so the allocated entry is cf_id_r itself: the tag compares stay
            // off the tags' write enables (no s1_addr -> cm -> lk_e -> c_tag CE path)
            if (lk_go) begin c_p[cf_id_r] <= 1'b1; c_v[cf_id_r] <= 1'b0; c_cur[cf_id_r] <= 1'b1; c_tag[cf_id_r] <= lk_tag; rr <= cf_id_r + CW'(1); end
            if (lk_done) begin
                if (!l_ph && s1_cross) begin l_e0 <= lk_e; l_ph <= 1'b1; end
                else begin
                    l_ph <= 1'b0;
                    jq_push <= 1'b1; jq_wd <= mkjob(K_ATTR, cl[3:0], s1_a, s1_ty, s1_nm, s1_sz, s1_addr[4:0], l_ph ? 4'(l_e0) : 4'(lk_e), 4'(lk_e), l_ph);
                end
            end
            if (take_cn) begin jq_push <= 1'b1; jq_wd <= mkjob(K_CONST, cl[3:0], s1_a, T_FP24, dbank, 3'd4, 5'd0, 4'd0, 4'd0, 1'b0); end
            // ---------------- the address pipe (the draw's descriptor bank)
            if (adv) begin
                s1_v <= s0_v; s1_en <= s0_en; s1_nm <= s0_nm; s1_a <= s0_a; s1_ty <= s0_ty; s1_sz <= s0_sz;
                s1_addr <= s0_bo + s0_prod; s1_addr1 <= s0_bo + s0_prod + 30'd32;
                s0_v <= g_run & (ga != 5'd16); s0_a <= ga[3:0]; s0_en <= d_en[{dbank, ga[3:0]}]; s0_ty <= d_ty[{dbank, ga[3:0]}]; s0_nm <= d_nm[{dbank, ga[3:0]}]; s0_sz <= d_sz[{dbank, ga[3:0]}];
                s0_prod <= 30'(vidx[29:0] * d_strd[{dbank, ga[3:0]}]); s0_bo <= d_bo[{dbank, ga[3:0]}];
                if (g_run && ga != 5'd16) ga <= ga + 5'd1;
            end
            // ---------------- the issue side
            case (ist)
                I_IDLE: if (dr_start) begin
                    indexed <= dr_indexed; u8 <= dr_u8; ip <= dr_idx_base; first <= dr_first; count <= dr_count;
                    dbank <= dr_bank; dvm <= dr_varmask; dps <= dr_ps; cp_i <= 4'd0;
                    i <= 32'd0; pv <= 32'd0; end_pend <= 1'b0; ist <= I_COPY;
                    c_cur <= '0; bs_v <= 2'b00;                              // nothing fetched before this draw is served again
                end
                // the shadow descriptors into the draw's bank (16 cycles; the previous draw's jobs keep their own bank)
                I_COPY: begin
                    d_en[{dbank, cp_i}] <= en_mask[cp_i]; d_ty[{dbank, cp_i}] <= s_ty[cp_i]; d_nm[{dbank, cp_i}] <= s_nm[cp_i]; d_sz[{dbank, cp_i}] <= s_sz[cp_i];
                    d_bo[{dbank, cp_i}] <= s_bo[cp_i]; d_strd[{dbank, cp_i}] <= s_strd[cp_i];
                    d_const[{dbank, cp_i}] <= cv_mask[cp_i] ? s_const[cp_i] : {ONE1, {3*W{1'b0}}};
                    cp_i <= cp_i + 4'd1;
                    if (cp_i == 4'd15) ist <= I_IDX;
                end
                // the next index (or the draw's end); the FIFO-threshold launch comes first
                I_IDX: begin
                    if (ref_cnt >= 6'(REF_THR) && cl != 5'd0) ist <= I_LAUNCH;
                    else if (i == count) begin
                        if (cl != 5'd0) begin end_pend <= 1'b1; ist <= I_LAUNCH; end else ist <= I_IDLE;
                    end else if (idx_ready && !ref_valid) begin vidx <= idx_val; i <= i + 32'd1; ip <= ip + (u8 ? 30'd1 : 30'd2); ist <= I_PTQ; end
                end
                // cache lookup: the reference (hit: the cached one; miss: the forming warp's next lane, then the gather)
                I_PTQ: begin
                    ref_valid <= 1'b1; ref_slot <= hit ? pslot[vidx[4:0]] : cs; ref_lane <= hit ? plane[vidx[4:0]] : cl[3:0];
                    if (hit) ist <= I_IDX;
                    else begin
                        pv[vidx[4:0]] <= 1'b1; ptag[vidx[4:0]] <= vidx[31:5]; pslot[vidx[4:0]] <= cs; plane[vidx[4:0]] <= cl[3:0];
                        ga <= 5'd0; ist <= I_GATH;
                    end
                end
                // the 16 attributes through the pipe; done when it has drained
                I_GATH: if (ga == 5'd16 && !s0_v && !s1_v) begin
                    if (cl == 5'd15) begin cl <= 5'd16; ist <= I_LAUNCH; end else begin cl <= cl + 5'd1; ist <= I_IDX; end
                end
                // launch the forming warp (a job, behind its lanes' writes), then take the next slot
                I_LAUNCH: if (jq_space) begin                           // {varmask in lane / a, {bank, ps} in ty, the lanes in o, the slot in e0}
                    jq_push <= 1'b1; jq_wd <= mkjob(K_LAUNCH, dvm[3:0], dvm[7:4], {1'b0, dbank, dps}, 1'b0, 3'd0, cl, 4'(cs), 4'd0, 1'b0);
                    ist <= I_NEWSLOT;
                end
                I_NEWSLOT: if (ns_ok) begin
                    for (int k = 0; k < 32; k++) if (pslot[k] == ns) pv[k] <= 1'b0;
                    alloc <= 1'b1; alloc_slot <= ns; cs <= ns; cl <= 5'd0;
                    ist <= end_pend ? I_IDLE : I_IDX;
                end
                default: ist <= I_IDLE;
            endcase
            // ---------------- the job FIFO (the push is registered: it lands the cycle after the decision)
            if (jq_push) begin jq[jq_wp] <= jq_wd; jq_wp <= jq_wp + JW'(1); end
            if (jq_pop) jq_rp <= jq_rp + JW'(1);
            jq_cnt <= jq_cnt + (JW+1)'(jq_push) - (JW+1)'(jq_pop);
            // ---------------- the build side
            p1_v <= p0_attr | p0_const;
            if (p0_attr | p0_const) begin
                p1_const <= p0_const; p1_lane <= j_lane; p1_a <= j_a; p1_ty <= j_ty; p1_nm <= j_nm; p1_sz <= j_sz; p1_o <= j_o[1:0];
                p1_sel <= csel;
            end
            if (p0_launch) begin
                in_start <= 1'b1; in_active <= 16'((17'd1 << j_o) - 17'd1); in_bank <= j_ty[1];
                launch <= 1'b1; launch_slot <= j_e0[SW-1:0]; launch_varmask <= {j_a, j_lane}; launch_ps <= j_ty[0];
            end
            dl_lane[0] <= p1_lane; dl_a[0] <= p1_a; dl_sz[0] <= p1_sz;
            for (int k = 1; k < 6; k++) begin dl_lane[k] <= dl_lane[k-1]; dl_a[k] <= dl_a[k-1]; dl_sz[k] <= dl_sz[k-1]; end
            if (wr_now) begin
                va_we <= 1'b1; va_lane <= dl_lane[5]; va_idx <= dl_a[5];
                for (int k = 0; k < 4; k++) va_data[k*W +: W] <= (k < int'(dl_sz[5])) ? cres[k] : (k == 3) ? ONE1 : '0;
            end
            inflight <= inflight + 8'(p0_attr | p0_const) - 8'(wr_now);
        end
    end
    // synthesis translate_off
    // a read the arbiter takes that the feed does not count (no return-order entry: every later return lands one place off)
    always @(posedge clk) if (!rst && r_en && r_rdy && !(ix_go || lk_go))
        $fatal(1, "vtx_feed: phantom read - the arbiter took a read the feed did not count (lk_req_r %b lk_act %b jq %0d)", lk_req_r, lk_act, jq_cnt);
    // synthesis translate_on
    wire _unused = &{1'b0, cres_v[3:1], jh[31], s1_span};
endmodule
`default_nettype wire
