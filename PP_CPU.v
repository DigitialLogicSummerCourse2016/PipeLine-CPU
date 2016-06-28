module PP_CPU(
	input reset,
	input clk,
	input [7:0] switch,
	input uart_rx,
	output [7:0] led,
	output [11:0] digi,
	output uart_tx
	);
	reg half_clk;
    always@(posedge clk or negedge reset)
    begin
        if(~reset)
            half_clk<=0;
        else
            half_clk<=~half_clk;
    end
    CPU C1(.reset(reset),.clk(half_clk),.switch(switch),.uart_rx(uart_rx),.led(led),.digi(digi),
    	.uart_tx(uart_tx));
endmodule
