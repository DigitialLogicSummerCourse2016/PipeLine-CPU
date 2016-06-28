`timescale 1ns/1ps
module CPU_tb();
	reg reset;
	reg clk;
	wire [7:0] switch;
	reg uart_rx;
	wire  [7:0] led;
	wire  [11:0] digi;
	wire  uart_tx;

	PP_CPU CPU1(.reset(reset),.clk(clk),.switch(switch),.uart_rx(uart_rx),.led(led),
		.digi(digi),.uart_tx(uart_tx));

	initial begin
	reset <= 1'b1;
	clk <= 1'b0;
	uart_rx <= 1'b1;
end

initial fork
	#5  reset <= 1'b0;
	#10 reset <= 1'b1;
	forever #1 clk <= ~clk;
	forever #20833 uart_rx <= ~uart_rx;
join
endmodule
