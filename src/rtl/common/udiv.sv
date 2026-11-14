// udiv — sequential unsigned divider, N-bit dividend by D-bit divisor, one quotient bit per clock (restoring).
// `start` loads the operands; `done` pulses with q = a / b and r = a % b (b = 0 gives q = all ones, r = a).
`default_nettype none
module udiv #(
    parameter int N = 32,
    parameter int D = 16
)(
    input  wire          clk, rst,
    input  wire          start,
    input  wire [N-1:0]  a,
    input  wire [D-1:0]  b,
    output reg           done,
    output reg  [N-1:0]  q,
    output reg  [D-1:0]  r
);
    reg          run;
    reg  [$clog2(N+1)-1:0] n;
    reg  [D:0]   rem;
    reg  [N-1:0] qa;
    reg  [D-1:0] bb;
    wire [D:0]   sh   = {rem[D-1:0], qa[N-1]};
    wire [D+1:0] diff = {1'b0, sh} - {2'b00, bb};
    always @(posedge clk) begin
        done <= 1'b0;
        if (rst) run <= 1'b0;
        else if (start) begin run <= 1'b1; n <= N; rem <= 0; qa <= a; bb <= b; end
        else if (run) begin
            if (diff[D+1]) begin rem <= sh; qa <= {qa[N-2:0], 1'b0}; end
            else begin rem <= diff[D:0]; qa <= {qa[N-2:0], 1'b1}; end
            n <= n - 1'b1;
            if (n == 1) begin
                run <= 1'b0; done <= 1'b1;
                q <= diff[D+1] ? {qa[N-2:0], 1'b0} : {qa[N-2:0], 1'b1};
                r <= diff[D+1] ? sh[D-1:0] : diff[D-1:0];
            end
        end
    end
endmodule
`default_nettype wire
