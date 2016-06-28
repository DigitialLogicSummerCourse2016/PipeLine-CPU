module IFID(
	input CLK,
	input reset,
	input IFID_flush,
	input IFID_Wrt,
	input [31:0] PC_in,
	input [31:0] Instruct_in,
	
	output reg [31:0] PC_out,
	output reg [31:0] Instruct_out
	);
	
	always@(posedge CLK,negedge reset)
	begin
		if (~reset)
		begin
			PC_out			<= 32'b0; 
			Instruct_out	<= 32'b0;
		end
		else if (IFID_flush)
		begin
			PC_out			<= 32'b0; 
			Instruct_out	<= 32'b0;
		end
		else if (IFID_Wrt)
		begin
			PC_out			<= PC_in;
			Instruct_out	<= Instruct_in;
		end
	end
endmodule
