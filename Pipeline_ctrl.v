module P_control(
	input clk,
	input [20:0] IDEX_ctrl,
	input [20:0] EXMEM_ctrl,
	input [20:0] MEMWB_ctrl,
	input [31:0] IFID_Instruct,
	input [31:0] IDEX_Instruct,
	input [31:0] EXMEM_Instruct,
	input [31:0] MEMWB_Instruct,
	input IRQ,
	input ALUOut0,
	
	output [1:0] Fwd_A,
	output [1:0] Fwd_B,
	//output FFA,FFB; //解决ID WB冲突
	output [1:0] jrs,
	output PC_Wr,
	output IFID_Wr,
	output IFID_Flush,
	output IDEX_Flush,
	output EXMEM_Flush,
	output MEMWB_X,	//修改指令，将PC写入$ra
	output reg trq,
	output stall,
	output is_B,
	output is_J
	);
	
	wire MEMWB_RegWr, EXMEM_RegWr, IDEX_RegWr, IDEX_MemRd;
	wire [1:0] MEMWB_RegDst, EXMEM_RegDst, IDEX_RegDst;
	wire [4:0] 
		MEMWB_Rd, MEMWB_Rt, MEMWB_dt,
		EXMEM_Rd, EXMEM_Rt, EXMEM_dt,
		IDEX_Rd, IDEX_Rt, IDEX_Rs, IDEX_dt,
		IFID_Rs, IFID_Rt;
		
	assign MEMWB_RegWr = MEMWB_ctrl[15];	
	assign MEMWB_RegDst = MEMWB_ctrl[17:16];	
	assign MEMWB_Rd = MEMWB_Instruct[15:11];
	assign MEMWB_Rt = MEMWB_Instruct[20:16];
	assign MEMWB_dt=
		(MEMWB_RegDst==2'b00)?MEMWB_Rd:
		(MEMWB_RegDst==2'b01)?MEMWB_Rt:
		5'b0;
	assign EXMEM_RegWr = EXMEM_ctrl[15];
	assign EXMEM_RegDst = EXMEM_ctrl[17:16];
	assign EXMEM_Rd = EXMEM_Instruct[15:11];
	assign EXMEM_Rt = EXMEM_Instruct[20:16];
	assign EXMEM_dt=
		(EXMEM_RegDst==2'b00)?EXMEM_Rd:
		(EXMEM_RegDst==2'b01)?EXMEM_Rt:
		5'b0;
	assign IDEX_RegWr = IDEX_ctrl[15];
	assign IDEX_MemRd = IDEX_ctrl[4];
	assign IDEX_RegDst = IDEX_ctrl[17:16];
	assign IDEX_Rd = IDEX_Instruct[15:11];
	assign IDEX_Rt = IDEX_Instruct[20:16];
	assign IDEX_Rs = IDEX_Instruct[25:21];
	assign IDEX_dt=
		(IDEX_RegDst==2'b00)?IDEX_Rd:
		(IDEX_RegDst==2'b01)?IDEX_Rt:
		5'b0;
	assign IFID_Rs = IFID_Instruct[25:21];
	assign IFID_Rt = IFID_Instruct[20:16];
		
	/*&&(EXMEM_dt!=IDEX_Rs) &&(EXMEM_dt!=IDEX_Rt)*/
	assign Fwd_A=
		((EXMEM_RegWr)&(EXMEM_dt!=5'b0)&(EXMEM_dt==IDEX_Rs))?2'b10:
		((MEMWB_RegWr)&(MEMWB_dt!=5'b0)&(MEMWB_dt==IDEX_Rs))?2'b01:
		2'b0;
	assign Fwd_B=
		((EXMEM_RegWr)&(EXMEM_dt!=5'b0)&(EXMEM_dt==IDEX_Rt))?2'b10:
		((MEMWB_RegWr)&(MEMWB_dt!=5'b0)&(MEMWB_dt==IDEX_Rt))?2'b01:
		2'b0;
	//assign FFA=(MEMWB_RegWr)&&(MEMWB_dt==IFID_Rs)?1:0;
	//assign FFB=(MEMWB_RegWr)&&(MEMWB_dt==IFID_Rt)?1:0;
	assign stall=((IDEX_MemRd)&( (IDEX_Rt==IFID_Rs)|(IDEX_Rt==IFID_Rt) ))?1:0;	
	assign PC_Wr=~stall;
	assign IFID_Wr=~stall;
	
	assign jrs=	
		((IDEX_RegWr)&(IDEX_dt ==IFID_Rs))?2'b11:
		((EXMEM_RegWr)&(EXMEM_dt==IFID_Rs))?2'b10:
		((MEMWB_RegWr)&(MEMWB_dt==IFID_Rs))?2'b01:
		2'b00;	
	
	
	
	assign is_J=
		(
		 (IFID_Instruct[31:26]==6'b000010)
		|(IFID_Instruct[31:26]==6'b000011)
		|((IFID_Instruct[31:26]==6'b000000)&(IFID_Instruct[5:0]==6'b001000))
		|((IFID_Instruct[31:26]==6'b000000)&(IFID_Instruct[5:0]==6'b001001))
		)?1:0;
	assign is_B=
		(ALUOut0)&
		(
			(IDEX_Instruct[31:26]==6'b000100)
			|(IDEX_Instruct[31:26]==6'b000101)
			|(IDEX_Instruct[31:26]==6'b000110)
			|(IDEX_Instruct[31:26]==6'b000111)
			|(IDEX_Instruct[31:26]==6'b000001)
		) ?1:0;
	reg IQ;
	initial 
	begin
		trq=0;
	end
	always@(negedge clk)
	begin
		IQ <= IRQ;
	end
	always@(negedge clk)
	begin
		if((~IQ) & IRQ)
		begin
			trq <= 1;
		end
		else
		begin
			trq <= 0;
		end
	end
	assign IFID_Flush=is_J|is_B|trq;
	assign IDEX_Flush=stall|is_B|trq;
	assign EXMEM_Flush=trq;
	assign MEMWB_X=trq;

endmodule
