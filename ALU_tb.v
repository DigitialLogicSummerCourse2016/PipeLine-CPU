module ALU_tb();
  wire[31:0] A = 32'b0000_0000_0000_0101_1011_0001_0000_1111;
  wire[31:0] B = 32'b1000_0010_1001_1111_1000_0001_1010_0011;

  wire sign = 1;
  wire[31:0] result;
  reg[5:0] AF;

  initial begin
    #5 AF<=6'b000000;
    #5 AF<=6'b000001;
    #5 AF<=6'b011000;
    #5 AF<=6'b011110;
    #5 AF<=6'b010110;
    #5 AF<=6'b010001;
    #5 AF<=6'b011010;
    #5 AF<=6'b100000;
    #5 AF<=6'b100001;
    #5 AF<=6'b100011;
    #5 AF<=6'b110011;
    #5 AF<=6'b110001;
    #5 AF<=6'b110101;
    #5 AF<=6'b111101;
    #5 AF<=6'b111001;
    #5 AF<=6'b111111;
  end
  ALU al1(.A(A), .B(B), .ALUFun(AF), .Sign(sign), .Z(result));
endmodule