module Shifter(
  input[31:0] a,
  input[31:0] b,
  input[1:0] ctrl,
  output[31:0] result
  );
wire[31:0] t1, t2, t4, t8;

Shifter_mod2 #(1) S1(.b(b), .ctrl(ctrl), .en(a[0]), .temp_result(t1));
Shifter_mod2 #(2) S2(.b(t1), .ctrl(ctrl), .en(a[1]), .temp_result(t2));
Shifter_mod2 #(4) S3(.b(t2), .ctrl(ctrl), .en(a[2]), .temp_result(t4));
Shifter_mod2 #(8) S4(.b(t4), .ctrl(ctrl), .en(a[3]), .temp_result(t8));
Shifter_mod2 #(16) S5(.b(t8), .ctrl(ctrl), .en(a[4]), .temp_result(result));

endmodule

module Shifter_mod2(
  input[31:0] b,
  input[1:0] ctrl,
  input en,
  output reg[31:0] temp_result
  );

  parameter level = 1;
  wire signed [31:0] signedb = b;
  always@(*)
  begin
    if(en)
      case(ctrl)
      2'b00:
        temp_result <= b << level;
      2'b01:
        temp_result <= b >> level;
      2'b11:
        temp_result <= signedb >>> level;
      default:
        temp_result <= b;
      endcase
    else begin
      temp_result <= b;
    end
  end

endmodule

