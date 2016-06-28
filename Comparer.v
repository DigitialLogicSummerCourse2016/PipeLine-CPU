module Comparer(
  input zero,
  input negative,
  input[2:0] ctrl,
  output reg[31:0] result
  );

  always @(*) begin
    case(ctrl)
      3'b001:     //EQ
        result <= {31'd0, zero};
      3'b000:     //NEQ
        result <= {31'd0, ~zero};
      3'b010:     //LT
        result <= {31'd0, negative};
      3'b110:     //LEZ
        result <= {31'd0, negative | zero};
      3'b100:     //LTZ
        result <= {31'd0, negative};
      3'b111:     //GTZ
        result <= {31'd0, ~ (negative | zero)};
    endcase
  end

endmodule
