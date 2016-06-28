module Peripheral (reset,clk,MemRead,MemWrite,Address,Write_data,Read_data,led,
	switch,digi,irqout,uart_tx,uart_rx);
input reset,clk;
input MemRead,MemWrite;
input [31:0] Address;
input [31:0] Write_data;
output [31:0] Read_data;
reg [31:0] Read_data;

output [7:0] led;
reg [7:0] led;
input [7:0] switch;
output [11:0] digi;
reg [11:0] digi;
output irqout;

input uart_rx;
output uart_tx;

reg [31:0] TH,TL;
reg [2:0] TCON;
reg [4:0] UART_CON;
wire rx_status,tx_status;
reg RX_state;
reg TX_state;
reg TX_EN;
reg [7:0] UART_TXD,UART_RXD;
wire [7:0] RX_DATA;

assign irqout = TCON[2] | UART_CON[2] | UART_CON[3];

always@(*) begin
	if(MemRead) begin
		case(Address)
			32'h40000000: Read_data <= TH;
			32'h40000004: Read_data <= TL;
			32'h40000008: Read_data <= {29'b0,TCON};
			32'h4000000C: Read_data <= {24'b0,led};
			32'h40000010: Read_data <= {24'b0,switch};
			32'h40000014: Read_data <= {20'b0,digi};
			32'h40000018: Read_data <= {24'b0,UART_TXD};
			32'h4000001c: Read_data <= {24'b0,UART_RXD};
			32'h40000020: Read_data <= {27'b0,UART_CON};
			default: Read_data <= 32'b0;
		endcase
	end
	else
		Read_data <= 32'b0;
end

//UART
wire bot_clk,smp_clk;
Generator Gen1(.sys_clk(clk),.reset(reset),.bot_clk(bot_clk),.smp_clk(smp_clk));
Receiver Rec1(.uart_rx(uart_rx),.smp_clk(smp_clk),.reset(reset),.rx_status(rx_status),
	.rx_data(RX_DATA));
Sender Sen1(.smp_clk(smp_clk),.tx_en(TX_EN),.bot_clk(bot_clk),.tx_data(UART_TXD),.reset(reset),
	.uart_tx(uart_tx),.tx_status(tx_status));


always@(negedge reset or posedge clk) begin
	if(~reset) begin
		TH <= 32'b0;
		TL <= 32'b0;
		TCON <= 3'b0;
		UART_TXD <= 8'b0;
		UART_RXD <= 8'b0;
		UART_CON <= 5'b0;
		TX_EN <= 1'b0;
		led <= 8'd0;
		digi <= 12'd1;
	end
	else begin
		if(TCON[0]) begin	//timer is enabled
			if(TL==32'hffffffff) begin
				TL <= TH;
				if(TCON[1]) TCON[2] <= 1'b1;		//irq is enabled
			end
			else TL <= TL + 1;
		end

		if(MemWrite) begin
			case(Address)
				32'h40000000: TH <= Write_data;
				32'h40000004: TL <= Write_data;
				32'h40000008: TCON <= Write_data[2:0];
				32'h4000000C: led <= Write_data[7:0];
				32'h40000014: digi <= Write_data[11:0];
				32'h40000018: UART_TXD <= Write_data[7:0];
				32'h40000020: UART_CON <= Write_data[4:0];
				default: ;
			endcase
		end
		if(~TX_EN)
			TX_EN <= (MemWrite & (Address == 32'h40000018));
		else if(~tx_status)
			TX_EN <= 1'b0;

		if(rx_status)
			UART_RXD <=RX_DATA;

		TX_state <= tx_status;
		if(TX_state & ~tx_status & UART_CON[0])
			UART_CON[2] <= 1'b1;

		RX_state <= rx_status;
		if(~RX_state & rx_status & UART_CON[1])
			UART_CON[3] <= 1'b1;

		if(MemRead & Address == 32'h40000020)
			UART_CON[3:2] <= 2'b00;
	end
end
endmodule