module Receiver(
  input uart_rx,
  input smp_clk,
  input reset,
  output reg rx_status,
  output reg[7:0] rx_data
);
  reg copy_RX;
  reg section;
  reg [7:0] sect_lim,sect_ctr;
  reg rcv_smp;

  initial
  begin
    sect_lim=8'b10100000-1'b1;sect_ctr=8'b0;section=0;
  end

  always@(posedge smp_clk or negedge reset)
  begin
    if(~reset)
    begin
      section<=0;
      sect_ctr<=8'b0;
    end
    else
      begin
        if(section==0)
        begin
          if((copy_RX==1)&&(uart_rx==0))
          begin
            section<=1;
            sect_ctr<=8'b0+1'b1;
          end
        end
        else
        begin
          if(sect_ctr==sect_lim)
          begin
            section<=0;sect_ctr<=8'b0;
          end
          else
            sect_ctr<=sect_ctr+1'b1;
        end
        copy_RX<=uart_rx;
      end
  end

  always@(posedge smp_clk or negedge reset)
  begin
    if(reset)
    begin
    if(rcv_smp) rcv_smp<=0;
    if( (sect_ctr==8'b00011000)||
      (sect_ctr==8'b00101000)||
      (sect_ctr==8'b00111000)||
      (sect_ctr==8'b01001000)||
      (sect_ctr==8'b01011000)||
      (sect_ctr==8'b01101000)||
      (sect_ctr==8'b01111000)||
      (sect_ctr==8'b10001000) ) rcv_smp<=1;
    end
  end
  
  always@(posedge smp_clk or negedge reset)
  begin
    if(~reset)
    begin
        rx_status<=0;
    end
    else begin
        if(rx_status) rx_status<=0;
        if(sect_ctr==8'b10011000)
        if(uart_rx) rx_status<=1;
    end
  end
  
  always@(posedge rcv_smp or negedge section)
  begin
  if(~reset)
  begin
    rx_data<=0;
  end
  else begin
    if(~section) rx_data<=8'b0;
    else
      begin
        rx_data[6:0]<=rx_data[7:1];
        rx_data[7]<=uart_rx;
      end
  end
  end
endmodule
