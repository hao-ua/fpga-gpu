// tex_cache — 4-way set-associative texture burst cache for the TMU, PIPELINED, HIT-UNDER-MISS, with a stream PREFETCH.
//   SETS x 4 ways x 256-bit lines (one DDR burst), raw-burst / format-agnostic. 512 sets = 64 KB (256-set data arrays
//   would use their 16 RAMB36 at half depth: 512 x 72 is one RAMB36 in SDP mode, so the 512 sets are free).
//   One lookup per cycle: stage A accepts the lookup and reads the set's {gen,tag} and
//   data BRAMs; stage B compares, and a HIT completes its queue slot at once while a MISS reserves the slot, picks
//   the PLRU victim and queues the DDR read (the miss queue issues them in order while further lookups continue, so
//   DDR latencies overlap). Results return IN ORDER through a CQ-deep completion queue; fills (in DDR return order)
//   complete the miss queue's head entry and write its {idx, victim, tag, gen}. One lu_done pulse per result.
//   Back-to-back lookups of a burst whose miss is still in flight miss again and fetch it again (the line then sits
//   in two ways with identical data): correct, rare (the TMU's burst buffer catches repeats).
//   HASH (OFF by default): the set index is the burst's low IXW bits XOR the next IXW bits (the tag stays
//   burst[25:IXW], so {idx, tag} still determines the burst). Measured on game frames: nothing on
//   three, +2 points of misses on the miss-heavy one — a texel row's bursts are consecutive, so the plain index already
//   maps 16 rows of a 256-texel texture onto distinct sets, and the XOR only folds distant rows back onto them.
//   PF: a demand miss (or merge) of burst X queues a PREFETCH of the next burst in the
//   stream's direction (X + 1, or X - 1 when the misses run downwards): it takes stage A in a cycle with no demand
//   lookup, and in stage B a hit or a pending match drops it, a miss allocates a victim and queues the DDR read with
//   no completion slot. In-flight prefetches sit in a PFN-entry pending table; a demand lookup that misses the tags
//   but matches a pending prefetch MERGES onto it (its slot completes when that fill lands) instead of fetching again.
//   Every miss-queue entry carries the generation it was queued under, so a fill after an invalidate (a prefetch can
//   be in flight when the TMU is idle) writes a stale generation = an invalid line, never fresh-looking stale data.
`default_nettype none
module tex_cache #(parameter int GENW=8,                  // GENW: generation-tag width (param so a TB can force a fast wrap)
                   parameter int SETS=512,                 // 256 = a 32 KB cache
                   parameter bit HASH=1'b0, PF=1'b1) (
    input  wire        clk, rst,
    input  wire        inval,               // pulse: invalidate all lines (render-to-texture / bind)
    input  wire        lu_valid,
    input  wire [25:0] lu_burst,            // burst address = app_addr >> 3
    output wire        lu_ready,            // can accept a lookup this cycle
    output reg         lu_done,             // one pulse per result, in issue order
    output reg [255:0] lu_data,
    output reg         mig_en,              // MIG read master (cache-owned)
    output reg [28:0]  mig_addr,
    input  wire        mig_rdy, mig_rd_valid,
    input  wire [255:0] mig_rd_data,
    output reg [31:0] n_hit, n_miss          // demand lookups only (a merge counts as a miss); n_pf / n_pfdrop / n_merge below
);
    localparam int WAYS=4, IXW=$clog2(SETS), TAGW=26-IXW, CQ=16, CQW=5;  // CQ: 16 results (misses) in flight
    localparam int MFD=32, MFW=6;           // miss queue: <= CQ demand misses + <= PFN prefetches (a prefetch is admitted only
                                            // while fewer than 16 entries are queued, so the sum never exceeds 32)
    localparam int PFN=8, PFW=3;            // prefetches in flight (pending table)
    localparam [GENW-1:0] GEN_MAX = {GENW{1'b1}};
    // The per-line valid bit + tag are a per-way BRAM word {gen[GENW], tag[TAGW]}; a line is VALID iff its stored gen
    // == cur_gen. inval = cur_gen++ (O(1)); a cur_gen wrap triggers a one-shot SETS-cycle sweep clearing every gen
    // (rare, at a draw boundary where the TMU is drained). Tag + gen + data are BRAM, read in stage A (registered).
    reg [GENW-1:0] cur_gen;
    reg            sweeping; reg [IXW-1:0] sweep_idx;
    (* ram_style="distributed" *) reg [2:0] plru [0:SETS-1];   // tree-PLRU per set (LUTRAM: 1R async + 1W @ b_idx)
    (* ram_style="block" *) reg [GENW+TAGW-1:0] ctag0 [0:SETS-1], ctag1 [0:SETS-1], ctag2 [0:SETS-1], ctag3 [0:SETS-1];
    (* ram_style="block" *) reg [255:0] cdata0 [0:SETS-1], cdata1 [0:SETS-1], cdata2 [0:SETS-1], cdata3 [0:SETS-1];
    reg [255:0] dq0,dq1,dq2,dq3;
    reg [GENW+TAGW-1:0] tq0,tq1,tq2,tq3;
    integer kp; initial for (kp=0;kp<SETS;kp=kp+1) begin plru[kp]=3'd0; ctag0[kp]=0; ctag1[kp]=0; ctag2[kp]=0; ctag3[kp]=0; end

    // ---- completion queue (in order) + miss queue (pending fills: slot / prefetch entry, burst, where to write) ----
    // result data in two LUTRAM banks, one write port each (a hit and a fill may complete in the same cycle):
    // hits -> cqh, fills -> cqf; cqm says which bank holds a slot's data
    (* ram_style="distributed" *) reg [255:0] cqh [0:CQ-1], cqf [0:CQ-1];
    reg           cqm [0:CQ-1];                 // slot completed by a fill
    reg           cqr [0:CQ-1];                 // ready
    reg [CQW-1:0] cq_h, cq_t;                   // drain head / enqueue tail
    reg [CQW-2:0] mfs [0:MFD-1];                // miss queue: the demand miss's cq slot
    reg [25:0]    mfb [0:MFD-1];                // the burst address (DDR command issue)
    reg [IXW-1:0] mfi [0:MFD-1];                // fill index
    reg [TAGW-1:0] mft[0:MFD-1];                // fill tag
    reg [1:0]     mfv [0:MFD-1];                // fill victim way
    reg [GENW-1:0] mfg[0:MFD-1];                // the generation the miss was queued under
    reg           mfp [0:MFD-1];                // a prefetch (no slot; completes a merged waiter, if any)
    reg [PFW-1:0] mfpe[0:MFD-1];                // its pending-table entry
    reg [MFW-1:0] mf_h, mf_i, mf_t;             // fill head / issue pointer / tail
    integer kq; initial begin
        for (kq=0;kq<CQ;kq=kq+1) begin cqh[kq]='0; cqf[kq]='0; cqm[kq]=1'b0; cqr[kq]=1'b0; end
        for (kq=0;kq<MFD;kq=kq+1) begin mfs[kq]='0; mfb[kq]='0; mfi[kq]='0; mft[kq]='0; mfv[kq]='0; mfg[kq]='0; mfp[kq]=1'b0; mfpe[kq]='0; end
    end
    // ---- the prefetch pending table: burst, waiter slot ----
    reg [PFN-1:0] pft_v, pft_wv; reg [25:0] pft_b [0:PFN-1]; reg [CQW-2:0] pft_ws [0:PFN-1];
    integer kf; initial for (kf=0;kf<PFN;kf=kf+1) begin pft_b[kf]='0; pft_ws[kf]='0; end
    // the prefetch request (one, the latest): set by a demand miss / merge, taken by stage A
    reg pf_v; reg [25:0] pf_b;
    reg [25:0] last_p1, last_m1;                // the previous miss's burst +- 1 (stream direction)
    reg [31:0] n_pf, n_pfdrop, n_merge;         // prefetches queued / dropped (hit or pending) / demand merges (bench-visible)

    // ---- stage A: accept + BRAM read; stage B: compare ----
    reg           b_v, b_pf; reg [IXW-1:0] b_idx; reg [TAGW-1:0] b_tag; reg [25:0] b_burst;
    reg           b_pm_v; reg [PFW-1:0] b_pm_e, b_pfe;
    wire [CQW-1:0] cq_cnt = cq_t - cq_h;
    wire [MFW-1:0] mf_cnt = mf_t - mf_h;
    assign lu_ready = ~rst & ~sweeping & ((cq_cnt + {{(CQW-1){1'b0}}, b_v}) < CQ[CQW-1:0]);   // slot for A + B
    wire          a_go_d = lu_valid & lu_ready;
    // the prefetch table entry a prefetch in A would take: the first free one, never the one a prefetch in B may take now
    wire [PFN-1:0] pft_busy = pft_v | ((b_v & b_pf) ? (PFN'(1) << b_pfe) : PFN'(0));
    reg [PFW-1:0] a_pfe; reg a_pf_free;
    always @* begin a_pf_free = 1'b0; a_pfe = '0; for (int e = PFN-1; e >= 0; e--) if (!pft_busy[e]) begin a_pf_free = 1'b1; a_pfe = PFW'(e); end end
    wire          a_go_p = PF & pf_v & ~a_go_d & ~rst & ~sweeping & a_pf_free & ((mf_cnt + {{(MFW-1){1'b0}}, b_v}) < MFW'(16));
    wire          a_go   = a_go_d | a_go_p;
    wire [25:0]   a_burst = a_go_d ? lu_burst : pf_b;
    wire          a_pf_stale;                   // the admitted prefetch is dropped (below, after the hit logic it needs)
    wire [IXW-1:0] a_idx = HASH ? (a_burst[IXW-1:0] ^ a_burst[2*IXW-1:IXW]) : a_burst[IXW-1:0];
    // pending match: the table, plus the prefetch in B (its entry is written at this edge)
    reg [PFN-1:0] a_pm; reg a_pm_any; reg [PFW-1:0] a_pm_e;
    wire a_pm_bp = b_v & b_pf & (b_burst == a_burst);
    always @* begin
        for (int e = 0; e < PFN; e++) a_pm[e] = pft_v[e] & (pft_b[e] == a_burst);
        a_pm_any = |a_pm | a_pm_bp; a_pm_e = b_pfe;
        if (!a_pm_bp) for (int e = PFN-1; e >= 0; e--) if (a_pm[e]) a_pm_e = PFW'(e);
    end

    // ---- fill: in-order DDR returns complete the miss queue's head entry ----
    wire          do_fill = mig_rd_valid & (mf_h != mf_t);
    wire [MFW-2:0] mh    = mf_h[MFW-2:0];
    wire [CQW-2:0] fslot = mfs[mh];
    wire [IXW-1:0] fidx  = mfi[mh]; wire [1:0] fvic = mfv[mh]; wire [TAGW-1:0] ftag = mft[mh]; wire [GENW-1:0] fgen = mfg[mh];
    wire          fpf    = mfp[mh]; wire [PFW-1:0] fpe = mfpe[mh];
    wire [CQW-2:0] fwslot = fpf ? pft_ws[fpe] : fslot;         // the slot this fill completes (a prefetch: its waiter, if any)
    wire          fw_v    = fpf ? pft_wv[fpe] : 1'b1;

    // data + {gen,tag} BRAMs: 1W (fill at fidx, or gen-sweep) / 1R (stage A at a_idx, registered). The tag write is ONE
    // statement with a muxed address and data: written as "if (sweeping) ... else if (fill) ...", Vivado 2026.1 calls the
    // block attribute infeasible and builds the four tag arrays as LUTRAM (at 256 sets: 288 RAM64M = 1 454 LUT).
    wire          tag_we = sweeping | do_fill;
    wire [IXW-1:0] tag_wa = sweeping ? sweep_idx : fidx;
    wire [GENW+TAGW-1:0] tag_wd = sweeping ? '0 : {fgen, ftag};
    always @(posedge clk) begin
        if (do_fill && fvic==2'd0) cdata0[fidx]<=mig_rd_data; if (tag_we && (sweeping || fvic==2'd0)) ctag0[tag_wa]<=tag_wd;
        if (a_go) begin dq0<=cdata0[a_idx]; tq0<=ctag0[a_idx]; end end
    always @(posedge clk) begin
        if (do_fill && fvic==2'd1) cdata1[fidx]<=mig_rd_data; if (tag_we && (sweeping || fvic==2'd1)) ctag1[tag_wa]<=tag_wd;
        if (a_go) begin dq1<=cdata1[a_idx]; tq1<=ctag1[a_idx]; end end
    always @(posedge clk) begin
        if (do_fill && fvic==2'd2) cdata2[fidx]<=mig_rd_data; if (tag_we && (sweeping || fvic==2'd2)) ctag2[tag_wa]<=tag_wd;
        if (a_go) begin dq2<=cdata2[a_idx]; tq2<=ctag2[a_idx]; end end
    always @(posedge clk) begin
        if (do_fill && fvic==2'd3) cdata3[fidx]<=mig_rd_data; if (tag_we && (sweeping || fvic==2'd3)) ctag3[tag_wa]<=tag_wd;
        if (a_go) begin dq3<=cdata3[a_idx]; tq3<=ctag3[a_idx]; end end
    // A block RAM read on port B in the cycle port A writes the same address returns undefined data on silicon (UG473
    // collision; the RTL would read the old value). A lookup of set a_idx in the cycle way fvic of that set is filled is
    // such a collision for that way's data and tag: that way is counted a miss in stage B (a refetch, correct in every case;
    // the line may then sit in two ways, as the header allows).
    reg [WAYS-1:0] coll_q;
    always @(posedge clk) coll_q <= (a_go && do_fill && (a_idx == fidx)) ? (WAYS'(1) << fvic) : '0;

    wire [WAYS-1:0] way_hit = { (tq3[GENW+TAGW-1 -: GENW]==cur_gen)&(tq3[TAGW-1:0]==b_tag), (tq2[GENW+TAGW-1 -: GENW]==cur_gen)&(tq2[TAGW-1:0]==b_tag),
                                (tq1[GENW+TAGW-1 -: GENW]==cur_gen)&(tq1[TAGW-1:0]==b_tag), (tq0[GENW+TAGW-1 -: GENW]==cur_gen)&(tq0[TAGW-1:0]==b_tag) }
                              & ~coll_q;
    wire hit_any = |way_hit;
    wire [1:0] hit_w = way_hit[0]?2'd0 : way_hit[1]?2'd1 : way_hit[2]?2'd2 : 2'd3;
    wire [255:0] dq_hit = (hit_w==2'd0)?dq0 : (hit_w==2'd1)?dq1 : (hit_w==2'd2)?dq2 : dq3;
    wire [2:0] pl = plru[b_idx];
    wire [1:0] victim = {pl[0], pl[0] ? pl[2] : pl[1]};
    wire b_dem = b_v & ~b_pf;
    // a demand miss merges onto a pending prefetch that is still pending (not the one filled this very cycle) and has no waiter yet
    wire b_pm_ok = PF & b_pm_v & pft_v[b_pm_e] & ~pft_wv[b_pm_e] & ~(do_fill & fpf & (fpe == b_pm_e));
    wire b_merge = b_dem & ~hit_any & b_pm_ok;
    wire b_miss  = b_dem & ~hit_any & ~b_pm_ok;
    wire b_pf_alloc = b_v & b_pf & ~hit_any & ~b_pm_v;              // a prefetch that is neither cached nor pending
    // a prefetch admitted in the cycle its own burst is decided a demand miss in B is stale (the demand lookups came back to
    // back, so the prefetch waited in pf_b until after the burst it predicted): it would allocate a duplicate line — dropped
    assign a_pf_stale = a_go_p & b_dem & ~hit_any & (b_burst == pf_b);
    wire [MFW-2:0] mt = mf_t[MFW-2:0];
    // the stream's direction: a miss at the previous miss's burst - 1 runs downwards
    wire b_down = (b_burst == last_m1);
    // completion data writes (one port per bank). A hit's data goes through a register (stage C): it keeps the path tag
    // block RAMs' latches -> the 4-way compare -> the 256-bit select out of the cqh LUTRAM, which is placed by its readers
    // (timing at 120 MHz). The hit's slot turns ready one cycle later (lu_done + 1 for hits;
    // the TMU takes results in queue order whenever they come); misses are not delayed.
    reg c_v, c_hit; reg [CQW-2:0] c_slot; reg [255:0] dq_sel;
    always @(posedge clk) begin c_v <= b_dem & ~rst; c_hit <= hit_any; c_slot <= cq_t[CQW-2:0]; dq_sel <= dq_hit; end
    always @(posedge clk) if (c_v & c_hit) cqh[c_slot] <= dq_sel;
    always @(posedge clk) if (do_fill & fw_v) cqf[fwslot] <= mig_rd_data;
    function automatic [2:0] plru_upd(input [2:0] cur, input [1:0] a);
        begin plru_upd = cur; plru_upd[0] = ~a[1];
              if (a[1]) plru_upd[2] = ~a[0]; else plru_upd[1] = ~a[0]; end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            b_v<=1'b0; b_pf<=1'b0; mig_en<=0; lu_done<=0; n_hit<=0; n_miss<=0; cq_h<=0; cq_t<=0; mf_h<=0; mf_i<=0; mf_t<=0;
            cur_gen<=1; sweeping<=1'b0; pft_v<='0; pft_wv<='0; pf_v<=1'b0; pf_b<='0; last_p1<='0; last_m1<='0;
            b_pm_v<=1'b0; b_pm_e<='0; b_pfe<='0; n_pf<=0; n_pfdrop<=0; n_merge<=0;
        end else begin
            lu_done<=1'b0;
            if (sweeping) begin
                if (sweep_idx==IXW'(SETS-1)) begin sweeping<=1'b0; cur_gen<=1; end else sweep_idx<=sweep_idx+1'b1;
            end else if (inval) begin
                if (cur_gen==GEN_MAX) begin sweeping<=1'b1; sweep_idx<=0; end else cur_gen<=cur_gen+1'b1;
            end
            // ===== stage A -> B =====
            b_v <= a_go & ~a_pf_stale; b_pf <= a_go_p;
            if (a_go) begin b_idx<=a_idx; b_tag<=a_burst[25:IXW]; b_burst<=a_burst; b_pm_v<=a_pm_any; b_pm_e<=a_pm_e; b_pfe<=a_pfe; end
            if (a_go_p) pf_v <= 1'b0;                     // taken (a miss in B this cycle sets a new one below)
            // ===== stage B, demand: hit -> ready slot; merge -> slot waits for the prefetch's fill; miss -> slot + miss queue =====
            if (b_dem) begin
                cqr[cq_t[CQW-2:0]]<=1'b0;                     // the slot is not ready yet, hit or miss (a hit's turns ready at stage C
                                                              // with its data: keeps the tag compare out of cqr's write)
                if (hit_any) begin
                    cqm[cq_t[CQW-2:0]]<=1'b0;
                    plru[b_idx]<=plru_upd(pl, hit_w);
                end else if (b_pm_ok) begin
                    cqm[cq_t[CQW-2:0]]<=1'b1;
                    pft_wv[b_pm_e]<=1'b1; pft_ws[b_pm_e]<=cq_t[CQW-2:0];
                    n_merge<=n_merge+1'b1;
                end else begin
                    cqm[cq_t[CQW-2:0]]<=1'b1;
                    mfs[mt]<=cq_t[CQW-2:0]; mfb[mt]<=b_burst; mfi[mt]<=b_idx; mft[mt]<=b_tag; mfv[mt]<=victim; mfg[mt]<=cur_gen; mfp[mt]<=1'b0;
                    mf_t<=mf_t+1'b1;
                    plru[b_idx]<=plru_upd(pl, victim);       // victim MRU now: the next miss in this set picks another way
                end
                cq_t<=cq_t+1'b1;
                if (PF && !hit_any) begin                     // the stream goes on: prefetch the next burst in its direction
                    pf_v<=1'b1; pf_b<=b_down ? b_burst-26'd1 : b_burst+26'd1;
                    last_p1<=b_burst+26'd1; last_m1<=b_burst-26'd1;
                end
            end
            // ===== stage B, prefetch: cached or pending -> dropped; else a victim + a miss-queue entry without a slot =====
            if (b_v & b_pf) begin
                if (b_pf_alloc) begin
                    mfb[mt]<=b_burst; mfi[mt]<=b_idx; mft[mt]<=b_tag; mfv[mt]<=victim; mfg[mt]<=cur_gen; mfp[mt]<=1'b1; mfpe[mt]<=b_pfe;
                    mf_t<=mf_t+1'b1;
                    plru[b_idx]<=plru_upd(pl, victim);
                    pft_v[b_pfe]<=1'b1; pft_wv[b_pfe]<=1'b0; pft_b[b_pfe]<=b_burst;
                    n_pf<=n_pf+1'b1;
                end else n_pfdrop<=n_pfdrop+1'b1;
            end
            // the hit / miss statistics counted a stage later, from the registered decision (keeps the tag compare off
            // the two 32-bit incrementers)
            if (c_v & c_hit) n_hit<=n_hit+1'b1;
            if (c_v & ~c_hit) n_miss<=n_miss+1'b1;
            // ===== DDR command issue, in miss order =====
            if (mig_en) begin if (mig_rdy) begin mig_en<=1'b0; mf_i<=mf_i+1'b1; end end
            else if (mf_i != mf_t) begin mig_en<=1'b1; mig_addr<={mfb[mf_i[MFW-2:0]], 3'b000}; end
            // ===== stage C: a hit's slot is ready once its data is in cqh =====
            if (c_v & c_hit) cqr[c_slot]<=1'b1;
            // ===== fill: complete the miss queue's head entry (a prefetch: its waiter's slot, and free the table entry) =====
            if (do_fill) begin
                if (fw_v) cqr[fwslot]<=1'b1;
                if (fpf) pft_v[fpe]<=1'b0;
                mf_h<=mf_h+1'b1;
            end
            // ===== drain: head ready -> emit in order =====
            if ((cq_t!=cq_h) && cqr[cq_h[CQW-2:0]]) begin
                lu_done<=1'b1; lu_data<=cqm[cq_h[CQW-2:0]] ? cqf[cq_h[CQW-2:0]] : cqh[cq_h[CQW-2:0]]; cq_h<=cq_h+1'b1; cqr[cq_h[CQW-2:0]]<=1'b0;
            end
        end
    end
endmodule
`default_nettype wire
