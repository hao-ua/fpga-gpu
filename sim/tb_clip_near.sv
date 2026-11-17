// tb_clip_near — the clipper against the w, near and far planes and the x / y guard band (Sutherland-Hodgman, per-attribute
// lerp from the inside vertex in the clip format, 0 .. 8 triangles out through the tri_valid / tri_ack handshake) against
// tools/a_setup_ref.py clip() (python3 tools/a_setup_ref.py --clipvec -> tools/gen/clipnear_vectors.hex: the guard
// constants of a 640 x 480 viewport, then the cases — small ones and vertices tens of thousands of times outside the
// view), bit for bit. +LINE: the line mode against clip_line() (clipnear_lines.hex: two ends in, none or two ends out).
`default_nettype none
module tb_clip_near;
    localparam int EXP_W=7, MAN_W=16, W=24, NVAR=8, NATTR=12, VB=NATTR*W, TOL=0;
    reg clk=0; always #5 clk=~clk; reg rst=1, start=0, tri_ack=0;
    reg [3*VB-1:0] verts_in; wire busy, tri_valid, done; wire [3*VB-1:0] tri_out; reg [4*W-1:0] gb = 0;
    reg z0 = 0;                                  // +Z0: the clip volume 0 <= z <= w (the _z0 vector files)
    reg line = 0;                                // +LINE: lines
    // +VMASK1 / +VMASK2: one varying vec4 masked out (the golden lerps every attribute, so only the triangle COUNT and the
    // enabled attributes are compared; a masked-out varying of an intersection must read 0)
    reg [NVAR/4-1:0] vmask = {NVAR/4{1'b1}};
    initial begin if ($test$plusargs("VMASK1")) vmask = 2'b01; if ($test$plusargs("VMASK2")) vmask = 2'b10; end
    function automatic aen(input int a); aen = (a < 4) ? 1'b1 : vmask[(a - 4) >> 2]; endfunction
    clip_near #(.EXP_W(EXP_W),.MAN_W(MAN_W),.NVAR(NVAR)) DUT(.clk,.rst,.start,.line,.verts_in,.vmask,.gb,.z0,.busy,.tri_valid,.tri_ack,.tri_out,.done);

    // collect output triangles: take each one and ask for the next
    reg [3*VB-1:0] got [0:7]; integer gn;
    always @(posedge clk) begin
        tri_ack <= 1'b0;
        if (tri_valid) begin got[gn]=DUT.pass ? verts_in : tri_out; gn=gn+1; tri_ack <= 1'b1; end   // pass: the caller keeps its copy
    end

    integer fd,code,nvr,nc,ci,i,a,nout,errors,t,v,k,d,nvin,clipped,cyc,cmax;
    reg [VB-1:0] V [0:2]; reg [VB-1:0] gtri [0:7][0:2]; reg [W-1:0] tmp,gg,ee;
    initial begin
        errors=0; start=0; clipped=0; cmax=0;
        if ($test$plusargs("Z0")) z0 = 1;
        if ($test$plusargs("LINE")) line = 1;
        nvin = line ? 2 : 3;
        fd=$fopen(line ? (z0 ? "tools/gen/clipnear_lines_z0.hex" : "tools/gen/clipnear_lines.hex")
                       : (z0 ? "tools/gen/clipnear_vectors_z0.hex" : "tools/gen/clipnear_vectors.hex"),"r");
        if (fd == 0) begin $display("FAIL: no vector file"); $finish; end
        code=$fscanf(fd,"%d",nvr); code=$fscanf(fd,"%d",nc);
        for (a=0;a<4;a=a+1) begin code=$fscanf(fd,"%h",tmp); gb[a*W+:W]=tmp; end     // gxlo gxhi gylo gyhi
        repeat(4) @(posedge clk); rst=0;
        for (ci=0; ci<nc; ci=ci+1) begin
            for (i=0;i<nvin;i=i+1) for (a=0;a<NATTR;a=a+1) begin code=$fscanf(fd,"%h",tmp); V[i][a*W+:W]=tmp; end
            if (line) V[2] = V[0];                                                    // a line's vertex 2 is its vertex 0
            code=$fscanf(fd,"%d",nout);
            for (t=0;t<nout;t=t+1) begin
                for (v=0;v<nvin;v=v+1) for (a=0;a<NATTR;a=a+1) begin code=$fscanf(fd,"%h",tmp); gtri[t][v][a*W+:W]=tmp; end
                if (line) gtri[t][2] = gtri[t][0];
            end
            // run clip
            gn=0; verts_in={V[2],V[1],V[0]}; cyc=0;
            @(posedge clk); start<=1; @(posedge clk); start<=0;
            while(!done) begin @(posedge clk); cyc=cyc+1; if (cyc > 20000) begin $display("FAIL: case%0d TIMEOUT", ci); $finish; end end
            repeat(2) @(posedge clk);
            if (cyc > cmax) cmax = cyc;
            if (gn!==nout) begin errors=errors+1; $display(" case%0d: got %0d, exp %0d",ci,gn,nout); end
            else for (t=0;t<nout;t=t+1) begin
                if (got[t] !== {V[2],V[1],V[0]}) clipped=clipped+1;
                for (v=0;v<3;v=v+1) for (k=0;k<NATTR;k=k+1) if (aen(k)) begin
                    gg=got[t][v*VB + k*W +: W]; ee=gtri[t][v][k*W +: W]; d=(gg>=ee)?gg-ee:ee-gg;
                    if (d>TOL || (^gg === 1'bx)) begin errors=errors+1;
                        if(errors<12) $display(" case%0d tri%0d v%0d a%0d got %h exp %h d=%0d",ci,t,v,k,gg,ee,d); end
                end
            end
        end
        $fclose(fd);
        $display("CLIP_NEAR%s%s: %0d cases (%0d results changed by the clip, the longest %0d cycles), errors = %0d (tol=%0d ULP)",
                 line ? " lines" : "", z0 ? " z0" : "", nc, clipped, cmax, errors, TOL);
        $display("%s", (errors==0 && nc > 0) ? "PASS":"FAIL"); $finish;
    end
endmodule
`default_nettype wire
