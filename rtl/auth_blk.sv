module auth_blk(clk, rst_n, RX, rider_off, pwr_up);
    input clk, rst_n;
    input logic RX;
    input logic rider_off;
    output logic pwr_up;


logic [7:0] rx_data;
logic rx_rdy, clr_rx_rdy;

UART_rx iUART_rx(.clk(clk), .rst_n(rst_n), .RX(RX), .clr_rdy(clr_rx_rdy), .rdy(rx_rdy), .rx_data(rx_data));
auth_SM iauth_SM(.clk(clk), .rst_n(rst_n), .rx_data(rx_data), .rx_rdy(rx_rdy),
 .rider_off(rider_off), .clr_rx_rdy(clr_rx_rdy), .pwr_up(pwr_up));

endmodule