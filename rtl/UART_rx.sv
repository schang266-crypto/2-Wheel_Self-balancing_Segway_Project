module UART_rx(clk, rst_n, rx_data, RX, clr_rdy, rdy);
input clk, rst_n;
input RX;      //Receiving data line
input clr_rdy;  //Clear ready signal

output logic rdy;       //Data ready signal
output logic [7:0] rx_data;

logic start, shift, receiving;


//Double flop RX to avoid metastability
logic RX_sync_0, RX_sync_1;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin       //Preset to avoid false start bit detection (0)
        RX_sync_0 <= 1'b1;
        RX_sync_1 <= 1'b1;
    end
    else begin
        RX_sync_0 <= RX;
        RX_sync_1 <= RX_sync_0;
    end
end


// Shift register for reception
logic [8:0] rx_shift_reg; // 8 data bits + 1 stop bit
always_ff @(posedge clk ) begin
     if (!shift)
        rx_shift_reg <= rx_shift_reg;   // Hold current value
     else if (shift)
        rx_shift_reg <= {RX_sync_1, rx_shift_reg[8:1]};   // Shift right, fill MSB with received bit
end
assign rx_data = rx_shift_reg[7:0]; // Received data bits (ignore start and stop bits)


// 4 bits to count up to 10 (1 start + 8 data + 1 stop)
logic [3:0] bit_cnt;   
always_ff @(posedge clk) begin
    if ({start, shift} == 2'b00)
        bit_cnt <= bit_cnt;        // Hold current value
    else if ({start, shift} == 2'b01) 
        bit_cnt <= bit_cnt + 4'd1; // Increment count on shift
    else if (start) 
        bit_cnt <= 4'd0;           // Load count for 1 start + 8 data + 1 stop bits
end


// Baud counter
logic [12:0] baud_cnt; // Count up to 5208
always_ff @(posedge clk) begin
    if ({(start|shift), receiving} == 2'b00)
        baud_cnt <= baud_cnt; 
    else if ({(start|shift), receiving} == 2'b01)
            baud_cnt <= baud_cnt - 13'd1; 
    else if (start) 
        baud_cnt <= 13'd2604; // Half bit period to sample in middle
    else if (shift) 
        baud_cnt <= 13'd5208; // Full bit period
end
assign shift = (baud_cnt == 13'd0); 


//State machine to control loading and shifting
typedef enum logic [0:0] {IDLE, RECEIVE} state_t;
state_t current_state, next_state;
logic set_rdy;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;
end 

always_comb begin
    // Default values
    start = 0;
    receiving = 0;
    set_rdy = 0;
    next_state = current_state;  // Hold state by default

    case (current_state)
        IDLE: begin
            if (!RX_sync_1) begin            // Start bit detected (line goes low)
                start = 1;
                next_state = RECEIVE;
            end
        end

        RECEIVE: begin
            receiving = 1;
            if (bit_cnt == 4'd9 && shift) begin 
                set_rdy = 1; // Assert rdy when byte is received
                next_state = IDLE;
            end
        end
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rdy <= 0;
    else if (start || clr_rdy)
        rdy <= 0;
    else if (set_rdy) // comb logic would only generate 1-clk set_rdy signal, but rdy needs to be 1 until clr_rdy = 1, so save it in reg
        rdy <= 1;
end

endmodule