module ALU(
    input[31:0] A,
    input[31:0] B,
    input[5:0] ALUFun,
    input Sign,
    output reg[31:0] Z,
    output zero
    );
  wire Zero, N, O;
  wire[31:0] Adder_result;
  Adder Ad(.a(A), .b(B), .ctrl(ALUFun[0]), .sign(1'b1), .Adder_result(Adder_result),
            .zero(Zero), .negative(N), .over(O));
  wire[31:0] Biter_result;
  Logicer Lo(.a(A),.b(B), .ctrl(ALUFun[3:0]), .result(Biter_result));
  wire[31:0] Shifter_result;
  Shifter Sh(.a(A), .b(B), .ctrl(ALUFun[1:0]), .result(Shifter_result));
  wire[31:0] Comparer_result;
  Comparer Co(.zero(Zero), .negative(N), .ctrl(ALUFun[3:1]), .result(Comparer_result));
  assign zero = Zero;
  always@(*)
  begin
    case(ALUFun[5:4])
    2'b00:
      Z <= Adder_result;
    2'b01:
      Z <= Biter_result;
    2'b10:
      Z <= Shifter_result;
    2'b11:
      Z <= Comparer_result;
    endcase
  end
endmodule
