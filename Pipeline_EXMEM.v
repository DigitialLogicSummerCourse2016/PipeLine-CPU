module EXMEM(
	input CLK,
	input reset,
	input EXMEM_flush,
	input [31:0] PC_in,
	input [31:0] Instruction_in,
	input [20:0] CtrlSig_in,
	input [31:0] DataBusB_in,
	input [31:0] ALU_in,
	
	output reg [31:0] PC_out,
	output reg [31:0] Instruction_out,
	output reg [20:0] CtrlSig_out,
	output reg [31:0] DataBusB_out,
	output reg [31:0] ALU_out
	);
	always@(posedge CLK,negedge reset)
	begin
		if (~reset) 
		begin
			PC_out				<= 32'b0; 
			Instruction_out		<= 32'b0; 
			CtrlSig_out			<= 21'b0;
			DataBusB_out		<= 32'b0; 
			ALU_out				<= 32'b0;
		end
		else if (EXMEM_flush)
		begin
			PC_out				<= 32'b0; 
			Instruction_out		<= 32'b0; 
			CtrlSig_out			<= 21'b0;
			DataBusB_out		<= 32'b0; 
			ALU_out				<= 32'b0;
		end
		else 
		begin
			PC_out			<= PC_in;
			Instruction_out	<= Instruction_in;
			CtrlSig_out		<= CtrlSig_in;
			DataBusB_out	<= DataBusB_in;
			ALU_out			<= ALU_in;
		end
	end
endmodule