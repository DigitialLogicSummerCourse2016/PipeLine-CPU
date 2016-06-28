module IDEX(
	input CLK,
	input reset,
	input IDEX_flush,
	input [31:0] PC_in,
	input [31:0] Instruction_in,
	input [20:0] CtrlSig_in,
	input [31:0] DataBusA_in,
	input [31:0] DataBusB_in,
	input [31:0] Im_in,
	input [4:0] shamt_in, 
	
	output reg [31:0] PC_out,
	output reg [31:0] Instruction_out,
	output reg [20:0] CtrlSig_out,
	output reg [31:0] DataBusA_out,
	output reg [31:0] DataBusB_out,
	output reg [31:0] Im_out,
	output reg [4:0] shamt_out
	);
	always@(posedge CLK,negedge reset)
	begin
		if (~reset)
		begin
			PC_out			<= 32'b0; 
			Instruction_out	<= 32'b0; 
			CtrlSig_out		<= 21'b0;
			DataBusA_out	<= 32'b0; 
			DataBusB_out	<= 32'b0; 
			Im_out			<= 32'b0; 
			shamt_out		<= 5'b0;
		end
		else if (IDEX_flush)
		begin
			PC_out			<= 32'b0; 
			Instruction_out	<= 32'b0; 
			CtrlSig_out		<= 21'b0;
			DataBusA_out	<= 32'b0; 
			DataBusB_out	<= 32'b0; 
			Im_out			<= 32'b0; 
			shamt_out		<= 5'b0;
		end
		else
		begin
			PC_out			<= PC_in;
			Instruction_out	<= Instruction_in;
			CtrlSig_out		<= CtrlSig_in;
			DataBusA_out	<= DataBusA_in;
			DataBusB_out	<= DataBusB_in;
			Im_out			<= Im_in;
			shamt_out		<= shamt_in;
		end
	end
endmodule