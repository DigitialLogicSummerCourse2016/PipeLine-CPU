module Sender(
  input smp_clk,
  input tx_en,
  input bot_clk,
  input[7:0] tx_data,
  input reset,
  output reg uart_tx,
  output reg tx_status
);  reg [7:0] sect_lim,sect_ctr;

  initial
  begin
    sect_lim=8'b10100000-1'b1;sect_ctr=8'b0;tx_status=1;
  end
  always@(posedge smp_clk or negedge reset)
  begin
  if(~reset)
  begin
    sect_ctr<=0;tx_status<=1;
  end
  else begin
    if((tx_status)&&(tx_en))
    begin
      tx_status<=0;sect_ctr<=8'b00000001;
    end
    else if(tx_status==0)
    begin
      if(sect_ctr==sect_lim)
      begin
        tx_status<=1;sect_ctr<=0;
      end
      else sect_ctr<=sect_ctr+1'b1;
    end
  end
  end
  always@(posedge smp_clk or negedge reset)
  begin
  if(~reset)
  begin
    uart_tx<=1;
  end
  else begin
    if (sect_ctr==8'b00000001) uart_tx<=0;
    if (sect_ctr==8'b00010001) uart_tx<=tx_data[0];
    if (sect_ctr==8'b00100001) uart_tx<=tx_data[1];
    if (sect_ctr==8'b00110001) uart_tx<=tx_data[2];
    if (sect_ctr==8'b01000001) uart_tx<=tx_data[3];
    if (sect_ctr==8'b01010001) uart_tx<=tx_data[4];
    if (sect_ctr==8'b01100001) uart_tx<=tx_data[5];
    if (sect_ctr==8'b01110001) uart_tx<=tx_data[6];
    if (sect_ctr==8'b10000001) uart_tx<=tx_data[7];
    if (sect_ctr==8'b10010001) uart_tx<=1;
  end
  end
endmodule
