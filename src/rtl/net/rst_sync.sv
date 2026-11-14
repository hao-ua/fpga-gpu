// rst_sync — reset synchronizer: asynchronous assert, synchronous release in the destination clock.
// Works while the destination clock is stopped (the output asserts immediately).
`default_nettype none
module rst_sync #(parameter int STAGES = 3) (
    input  wire clk,
    input  wire arst,          // asynchronous, active high
    output wire rst
);
    (* ASYNC_REG = "TRUE" *) reg [STAGES-1:0] s = {STAGES{1'b1}};
    always @(posedge clk or posedge arst)
        if (arst) s <= {STAGES{1'b1}};
        else      s <= {s[STAGES-2:0], 1'b0};
    assign rst = s[STAGES-1];
endmodule
`default_nettype wire
