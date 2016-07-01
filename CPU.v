module CPU(
	input reset,
	input clk,
	input [7:0] switch,
	input uart_rx,

	output [7:0] led,
	output [11:0] digi,
	output uart_tx
	);
	reg[31:0] PC;
	wire[31:0] Instrument,
		IDEX_Instruction_out, EXMEM_Instruction_out, MEMWB_Instruction_out,
		IFID_PC_out, IDEX_PC_out, EXMEM_PC_out, MEMWB_PC_out;
	wire [20:0] IDEX_CtrlSig_out, EXMEM_CtrlSig_out, MEMWB_CtrlSig_out, CtrlSig;
	wire PCSupervisor;

	wire IRQ, EXTOp, LUOp, nALUSrc1,nALUSrc2, nSign, nMemWr, nMemRd, nRegWr;
	wire[2:0] PCSrc;
	wire[5:0] nALUFun;
	wire[1:0] nRegDst;
	wire[1:0] nMemToReg;
	wire[31:0] ALUOut;

	wire[1:0] Fwd_A, Fwd_B, jrs;
	wire PC_Wr, IFID_Wr, IFID_Flush, IDEX_Flush, EXMEM_Flush, MEMWB_X,
		trq,stall,is_B,is_J;

	wire [31:0] DataBusA, Im_out, Data_ra, EXMEM_ALUOut, DMData,MEMWB_DMData, MEMWB_ALUOut;

	wire [31:0] XADR, ILLOP, ConBA, PC_plus_4;
	wire [25:0] JT;

	wire[2:0] EX_PCSrc;
	wire [31:0] PC_next;

	wire[31:0] rom_instruct;

	wire[31:0] LUOpout,ExtImm;

	wire[31:0] DataBusB,DataBusC;
	reg [31:0] R_TGT;
	wire [1:0] RegDst,MemToReg;
	wire[4:0] AddrC,Xp,Ra;

	wire [31:0] IDEX_DataBusA, IDEX_DataBusB, DataBusA_MD, DataBusB_MD;
	wire[4:0] Shamt;

	wire[31:0] A,B,FA,FB,EXMEM_DataBusB;
	wire Zero;
	wire ALUSrc1,ALUSrc2,Sign;
	wire[5:0] ALUFun;

	wire MemWr1,MemWr2,MemRd,MemWr;
	wire[31:0] DMData1,DMData2;
	wire [11:0] DIGI;
/*	wire [7:0] leds;
	wire [14:0] led;
	assign ledtemp [14:0] =
		(~switch[7])?{7'b0, leds}:
		led;

	assign led[14] = PC[31];
	assign led[9:0] =
		(switch[0])?IFID_PC_out[9:0]:
		(switch[1])?IDEX_PC_out[9:0]:
		(switch[2])?EXMEM_PC_out[9:0]:
		(switch[3])?MEMWB_PC_out[9:0]:
		(switch[4])?Instrument[9:0]:
		PC[9:0];
	assign led[13]=trq;
	assign led[12]=stall;
	assign led[11]=is_B;
	assign led[10]=is_J;
	*/

//	assign signal = IRQ;
//	always@(posedge clk or negedge reset)
//    begin
//        if(~reset)
//            signal=0;
//        else
//            signal=~signal;
//    end

	//all control

	assign PCSupervisor= PC[31];

	Controller control1(.Instruction(Instrument),.IRQ(IRQ),.PCSupervisor(PCSupervisor)

		,.PCSrc(PCSrc),.CtrlSig(CtrlSig)
		,.EXTOp(EXTOp),.LUOp(LUOp)
		,.ALUFun(nALUFun),.ALUSrc1(nALUSrc1),.ALUSrc2(nALUSrc2),.Sign(nSign)
		,.MemWr(nMemWr),.MemRd(nMemRd)
		,.RegDst(nRegDst),.MemToReg(nMemToReg),.RegWr(nRegWr)
		);

	P_control _P_control(
		.clk(clk),
		.IDEX_ctrl(IDEX_CtrlSig_out),
		.EXMEM_ctrl(EXMEM_CtrlSig_out),
		.MEMWB_ctrl(MEMWB_CtrlSig_out),
		.IFID_Instruct(Instrument),
		.IDEX_Instruct(IDEX_Instruction_out),
		.EXMEM_Instruct(EXMEM_Instruction_out),
		.MEMWB_Instruct(MEMWB_Instruction_out),
		.IRQ(IRQ),
		.ALUOut0(ALUOut[0]),

		.Fwd_A(Fwd_A),.Fwd_B(Fwd_B),
		.jrs(jrs),.PC_Wr(PC_Wr),
		.IFID_Wr(IFID_Wr),.IFID_Flush(IFID_Flush),
		.IDEX_Flush(IDEX_Flush),
		.EXMEM_Flush(EXMEM_Flush),
		.MEMWB_X(MEMWB_X),//修改指令，将PC写入$ra)
		.trq(trq),.stall(stall),.is_B(is_B),.is_J(is_J)
	);

	//PC reg
	/*
	XADR, ILLOP		Const
	PC+4			IF
	JT				ID
	ConBA			EX
	*/

	assign XADR = 32'h80000008;
	assign ILLOP = 32'h80000004;
	assign PC_plus_4 = PC + 32'd4;
	assign ConBA = IDEX_PC_out+{Im_out[29:0],2'b00}+32'd4;
	assign JT = Instrument[25:0];
	assign Data_ra=
		(jrs==2'b00)?DataBusA:
		(jrs==2'b01)?
		(
			(MEMWB_CtrlSig_out[3:2]==2'b00)?MEMWB_ALUOut:
			(MEMWB_CtrlSig_out[3:2]==2'b01)?MEMWB_DMData:
			(MEMWB_CtrlSig_out[3:2]==2'b10)?MEMWB_PC_out+32'd4:
			32'd0
		):
		(jrs==2'b10)?
		(
			(EXMEM_CtrlSig_out[3:2]==2'b00)?EXMEM_ALUOut:
			(EXMEM_CtrlSig_out[3:2]==2'b01)?DMData:
			(EXMEM_CtrlSig_out[3:2]==2'b10)?EXMEM_PC_out+32'd4:
			32'd0
		):
		(jrs==2'b11)?
		(
			(IDEX_CtrlSig_out[3:2]==2'b00)?ALUOut:
			(IDEX_CtrlSig_out[3:2]==2'b10)?IDEX_PC_out+32'd4:
			32'd0
		):
		32'd0;

	assign EX_PCSrc = IDEX_CtrlSig_out[20:18];
	assign PC_next =
		(EX_PCSrc==3'b001)?(ALUOut[0]? ConBA:PC_plus_4):
		(PCSrc==3'b000)? PC_plus_4:
		(PCSrc==3'b010)? {IFID_PC_out[31],3'b0,JT,2'b0}:
		(PCSrc==3'b011)? Data_ra:
		(PCSrc==3'b100)? ILLOP:
		(PCSrc==3'b101)? XADR:
		PC_plus_4;

	always@(negedge reset or posedge clk)
	begin
		if(~reset)
			PC <= 32'h80000000;
		else if(PC_Wr)
		begin
			PC <= PC_next;
		end
	end

	ROM ROM1(.addr(PC[30:0]),.data(rom_instruct));

	//IFID
	IFID _IFID(
		.CLK(clk) ,.reset(reset),
		.IFID_flush(IFID_Flush),.IFID_Wrt(IFID_Wr),
		.PC_in(PC) ,.Instruct_in(rom_instruct),

		.PC_out(IFID_PC_out) ,.Instruct_out(Instrument)
	);

	assign LUOpout = LUOp? {Instrument[15:0],16'h0000} : ExtImm;
	assign ExtImm = {EXTOp?{16{Instrument[15]}}:16'h0000,Instrument[15:0]};

	//MEMWB
	MEMWB _MEMWB(
		.CLK(clk) ,.reset(reset) ,.MEMWB_X(MEMWB_X),
		.PC_in(EXMEM_PC_out) ,.Instruction_in(EXMEM_Instruction_out) ,.CtrlSig_in(EXMEM_CtrlSig_out),
		.ALU_in(EXMEM_ALUOut),
		.MEM_in(DMData),

		.PC_out(MEMWB_PC_out) ,.Instruction_out(MEMWB_Instruction_out) ,.CtrlSig_out(MEMWB_CtrlSig_out),
		.MEM_out(MEMWB_DMData),
		.ALU_out(MEMWB_ALUOut)
	);

	assign Xp = 5'd26;
	assign Ra = 5'd31;
	assign RegDst = MEMWB_CtrlSig_out[17:16];
	assign MemToReg = MEMWB_CtrlSig_out[3:2];
	assign AddrC = (RegDst==2'b00)? MEMWB_Instruction_out[15:11]://rd
	               (RegDst==2'b01)? MEMWB_Instruction_out[20:16]://rt
	               (RegDst==2'b10)? Ra:
	               (RegDst==2'b11)? Xp:
				   5'd0;
	assign DataBusC = (MemToReg==2'b00)? MEMWB_ALUOut:
					  (MemToReg==2'b01)? MEMWB_DMData:
					  (MemToReg==2'b10)? MEMWB_PC_out+32'd4:
					  R_TGT;

	always@(posedge trq)
	begin
		if      (EXMEM_PC_out!=32'd0)
			R_TGT <= EXMEM_PC_out;			//normal
		else if (MEMWB_PC_out!=32'd0)
			R_TGT <= MEMWB_PC_out;			//j lw
		else
			R_TGT <= MEMWB_PC_out-32'd4;		//b
	end

    RegisterFile Register(.reset(reset), .clk(clk),
		.WrC(MEMWB_CtrlSig_out[15]),
		.AddrA(Instrument[25:21]), .AddrB(Instrument[20:16]),
		.ReadDataA(DataBusA) ,.ReadDataB(DataBusB),
		.AddrC(AddrC), .WriteDataC(DataBusC));

	//IDEX

	assign DataBusA_MD=
		((MEMWB_CtrlSig_out[15])&(AddrC==Instrument[25:21]))?DataBusC:DataBusA;
	assign DataBusB_MD=
		((MEMWB_CtrlSig_out[15])&(AddrC==Instrument[20:16]))?DataBusC:DataBusB;
	IDEX _IDEX(
		.CLK(clk) ,.reset(reset),
		.IDEX_flush(IDEX_Flush),
		.PC_in(IFID_PC_out) ,.Instruction_in(Instrument) ,.CtrlSig_in(CtrlSig),
		.DataBusA_in(DataBusA_MD) ,.DataBusB_in(DataBusB_MD),
		.Im_in(LUOpout) ,.shamt_in(Instrument[10:6]),

		.PC_out(IDEX_PC_out) ,.Instruction_out(IDEX_Instruction_out) ,.CtrlSig_out(IDEX_CtrlSig_out),
		.DataBusA_out(IDEX_DataBusA) ,.DataBusB_out(IDEX_DataBusB),
		.Im_out(Im_out) ,.shamt_out(Shamt)
	);

	assign ALUSrc1 = IDEX_CtrlSig_out[14];
	assign ALUSrc2 = IDEX_CtrlSig_out[13];
	assign ALUFun = IDEX_CtrlSig_out[12:7];
	assign Sign = IDEX_CtrlSig_out[6];
	assign FA=
		(Fwd_A==2'b00)?IDEX_DataBusA:
		(Fwd_A==2'b01)?
			((MEMWB_CtrlSig_out[3:2]==2'b00)?MEMWB_ALUOut:MEMWB_DMData):
		(Fwd_A==2'b10)?EXMEM_ALUOut:32'd0;
	assign FB=
		(Fwd_B==2'b00)?IDEX_DataBusB:
		(Fwd_B==2'b01)?
			((MEMWB_CtrlSig_out[3:2]==2'b00)?MEMWB_ALUOut:MEMWB_DMData):
		(Fwd_B==2'b10)?EXMEM_ALUOut:32'd0;
	assign A =
		ALUSrc1? Shamt[4:0]:FA;
	assign B =
		ALUSrc2? Im_out:FB;


	ALU ALU1(.A(A),.B(B),.ALUFun(ALUFun),.Sign(Sign),.Z(ALUOut),.zero(Zero));

	//EXMEM

	EXMEM _EXMEM(
		.CLK(clk) ,.reset(reset) ,.EXMEM_flush(EXMEM_Flush),
		.PC_in(IDEX_PC_out) ,.Instruction_in(IDEX_Instruction_out) ,.CtrlSig_in(IDEX_CtrlSig_out),
		.DataBusB_in(FB),
		.ALU_in(ALUOut),

		.PC_out(EXMEM_PC_out) ,.Instruction_out(EXMEM_Instruction_out) ,.CtrlSig_out(EXMEM_CtrlSig_out),
		.DataBusB_out(EXMEM_DataBusB),
		.ALU_out(EXMEM_ALUOut)
	);


	assign MemRd=EXMEM_CtrlSig_out[4];
	assign MemWr=EXMEM_CtrlSig_out[5];
	assign MemWr1 = (EXMEM_ALUOut[31:28] == 4'b0100) ? 1'b0 : MemWr;
	assign MemWr2 = (EXMEM_ALUOut[31:28] == 4'b0100) ? MemWr : 1'b0;
	assign DMData = (EXMEM_ALUOut[31:28] == 4'b0100) ? DMData2 : DMData1;
	assign digi = ~DIGI;

	DataMemory DM1(.reset(reset), .clk(clk),
		.Address(EXMEM_ALUOut), .Write_data(EXMEM_DataBusB),
		.Read_data(DMData1), .MemRead(MemRd), .MemWrite(MemWr1));
	//UART and Peripheral
	Peripheral Perip1(.reset(reset),.clk(clk),
		.MemRead(MemRd),.MemWrite(MemWr2),
		.Address(EXMEM_ALUOut),.Write_data(EXMEM_DataBusB),.Read_data(DMData2),.led(led),
		.switch(switch),.digi(DIGI),.irqout(IRQ),.uart_tx(uart_tx),.uart_rx(uart_rx));
endmodule
