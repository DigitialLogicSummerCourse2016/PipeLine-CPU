module Adder(
    input[31:0] a,
    input[31:0] b,
    input ctrl,     //0 to add, 1 to sub
    input sign,     //1 to signed, 0 to unsigned
    output [31:0] Adder_result,
    output zero,
    output negative,
    output over
    );

wire[32:0] A = {1'b0, a};
wire[32:0] B = ctrl ? (~{1'b0, b} + 1'b1) : {1'b0, b} ;
wire[32:0] result = A + B;

assign  Adder_result = result[31:0],
        zero = (result==0'd0),
        negative = sign & result[31] ^ over |
                   (~sign & ctrl & over),
        over = sign & ((a[31] & B[31] & ~result[31]) |
               (~a[31] & ~B[31] & result[31])) |
               (~sign & result[32]);

endmodule
