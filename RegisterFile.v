
module RegisterFile(reset, clk, WrC, AddrA, AddrB, AddrC, WriteDataC, ReadDataA, ReadDataB);
	input reset, clk;
	input WrC;
	input [4:0] AddrA, AddrB, AddrC;
	input [31:0] WriteDataC;
	output [31:0] ReadDataA;
	output [31:0] ReadDataB;

	reg [31:0] RF_data[31:1];

	assign ReadDataA = (AddrA == 5'b00000)? 32'h00000000: RF_data[AddrA];
	assign ReadDataB = (AddrB == 5'b00000)? 32'h00000000: RF_data[AddrB];

	integer i;
	always @(negedge reset or posedge clk)
		if (~reset)
			for (i = 1; i < 32; i = i + 1)
				RF_data[i] <= 32'h00000000;
		else if (WrC && (AddrC != 5'b00000))
			RF_data[AddrC] <= WriteDataC;

endmodule

