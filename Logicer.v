module Logicer(
  input[31:0] a,
  input[31:0] b,
  input[3:0] ctrl,
  output reg[31:0] result
  );

  always@(*)
  begin
    case(ctrl)
    4'b1000:
      result<= a & b;
    4'b1110:
      result<= a | b;
    4'b0110:
      result<= a ^ b;
    4'b0001:
      result<= ~(a | b);
    4'b1010:
      result<= a;
    default:
      result<=0;
    endcase
  end

endmodule
