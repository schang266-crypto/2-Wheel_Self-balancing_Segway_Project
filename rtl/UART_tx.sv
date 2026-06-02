module UART_tx(clk, rst_n, tx_data, trmt, TX, tx_done);
input clk, rst_n;
input [7:0] tx_data;
input trmt;      //Initiating transmission

output logic TX;
output logic tx_done;

logic load, shift, transmitting;


// Shift register for transmission
logic [8:0] tx_shift_reg; // 8 data bits + 1 start bit
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        tx_shift_reg <= 9'h1ff; // preset to idle state (all 1s), so when start bit (0) is sent, it can be detected
    else if ({load, shift} == 2'b00)
        tx_shift_reg <= tx_shift_reg;   // Hold current value
    else if ({load, shift} == 2'b01) 
        tx_shift_reg <= {1'b1, tx_shift_reg[8:1]}; // Shift right, fill MSB with stop bit (1)
    else if (load) 
        tx_shift_reg <= {tx_data, 1'b0}; // Load data bits, and start bit (0)
end
assign TX = tx_shift_reg[0]; // Transmit LSB first


// 4 bits to count up to 10 (1 start + 8 data + 1 stop)
logic [3:0] bit_cnt;   
always_ff @(posedge clk) begin
    if ({load, shift} == 2'b00)
        bit_cnt <= bit_cnt; // Hold current value
    else if ({load, shift} == 2'b01) begin
        bit_cnt <= bit_cnt + 4'd1; // Increment count on shift
        if (bit_cnt == 4'd9 && shift)
            bit_cnt <= 4'd0; // Reset after sending 10 bits
    end
    else if (load) 
        bit_cnt <= 4'd0; // Load count for 1 start + 8 data + 1 stop bits
end

// Baud counter, used to generate shift signal 
logic [12:0] baud_cnt; // shift every 5208 clock cycles (50 MHz clk/ 9600 baud)
always_ff @(posedge clk) begin
    if ({(load|shift), transmitting} == 2'b00)
        baud_cnt <= baud_cnt; 
    else if ({(load|shift), transmitting} == 2'b01)
            baud_cnt <= baud_cnt + 13'd1; 
    else if ((load|shift) == 1'b1) 
        baud_cnt <= 13'd0; 
end
assign shift = (baud_cnt == 13'd5208);


//State machine to control loading and shifting
typedef enum logic [0:0] {IDLE, TRANSMIT} state_t;
state_t current_state, next_state;
logic set_done;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;
end 

always_comb begin
    // Default values
    load = 0;
    transmitting = 0;
    set_done = 0;
    next_state = current_state;  // Hold state by default

    case (current_state)
        IDLE: begin
            if (trmt) begin
                load = 1;
                next_state = TRANSMIT;
            end
        end

        TRANSMIT: begin
            transmitting = 1;
            if (bit_cnt == 4'd9 && shift) begin
                set_done = 1;
                next_state = IDLE;
            end
        end
    endcase
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tx_done <= 0;
    else if (load)
        tx_done <= 0;
    else if (set_done)
        tx_done <= 1;
end

endmodule