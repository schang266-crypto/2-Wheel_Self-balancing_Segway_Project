// LOGIC: A2D Interface outputs which channels to read via MOSI, and receives A2D data via MISO from SPI_mnrch
module A2D_intf( 
    input  logic nxt, // indicating the next channel to read
    input  logic MISO,

    output logic [11:0] lft_ld,
    output logic [11:0] rght_ld,
    output logic [11:0] steer_pot,
    output logic [11:0] batt,
    output logic SS_n,
    output logic SCLK,
    output logic MOSI
);
    logic [15:0] wrt_data; // data to write to SPI_mnrch, which is channel info
    logic [15:0] rd_data; // data read from SPI_mnrch, which is A2D data (lft/rght load, steer pot, batt)
    logic done_signal; // SPI_mnrch is done with transaction
    logic wrt; // Initialize the SPI_mnrch 
    
SPI_mnrch spi_mnrch_inst (
    .clk(clk),
    .rst_n(rst_n),
    .wrt(wrt),
    .wrt_data(wrt_data), 
    .MISO(MISO),
    .SS_n(SS_n),
    .SCLK(SCLK),
    .MOSI(MOSI),
    .done(done_signal),
    .rd_data(rd_data)
);


// Round Robin Counter Logic
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        round_robin <= 2'b00;
    else if (update) 
        round_robin <= round_robin + 2'b01;
end
// Update outputs logic based on the round robin counter
wire en_channel_0;
wire en_channel_4;
wire en_channel_5;
wire en_channel_6;
// Transaction order: Channel 0 -> Channel 4 -> Channel 5 -> Channel 6
assign en_channel_0 = (round_robin == 2'b00);
assign en_channel_4 = (round_robin == 2'b01);
assign en_channel_5 = (round_robin == 2'b10);
assign en_channel_6 = (round_robin == 2'b11);


//Channel_0 - Left Load
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        lft_ld <= 12'h000;
    else if (en_channel_0 && update)
        lft_ld <= rd_data[11:0]; // Only care about MISO data, not MOSI
end 
//Channel_4 - Right Load
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rght_ld <= 12'h000;
    else if (en_channel_4 && update)
        rght_ld <= rd_data[11:0];
end 
//Channel_5 - Steering Pot
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        steer_pot <= 12'h000;
    else if (en_channel_5 && update)
        steer_pot <= rd_data[11:0];
end 
//Channel_6 - Steering Pot
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        batt <= 12'h801; // battery not low
    else if (en_channel_6 && update)
        batt <= rd_data[11:0];
end 


// State machine states
typedef enum logic [1:0] {
    Idle, 
    Transaction_1, 
    Wait, 
    Transaction_2
} state_t;
    state_t state, next_state;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= Idle;
    else
        state <= next_state;
end

logic [1:0] round_robin;
logic update; 
always_comb begin 
    wrt = 1'b0;
    wrt_wdata = 16'h0000;
    update = 1'b0;
    next_state = state; 

    case (state)
        Idle: begin // wrt_data = {2’b00, channel, 11’b000}
            if(nxt && en_channel_0) begin
                wrt = 1'b1;
                wrt_data = 16'h0000; //Left Load, channel 000, SPI_mnrch sends wrt_data on MOSI, don't care MISO
                next_state = Transaction_1;
            end 
            else if(nxt && en_channel_4) begin
                wrt = 1'b1;
                wrt_data = 16'h2000; //Right Load, channel 100
                next_state = Transaction_1;
            end 
            else if(nxt && en_channel_5) begin
                wrt = 1'b1;
                wrt_data = 16'h2800; //Steering Pot, channel 101
                next_state = Transaction_1;
            end
            else if(nxt && en_channel_6) begin
                wrt = 1'b1;
                wrt_data = 16'h3000; //Battery Voltage, channel 110
                next_state = Transaction_1;
            end
        end

        Transaction_1: begin
            if (done_signal)  // done transaction from SPI_mnrch
                next_state = Wait; 
        end 

        Wait: begin // wait for 1 clk cycle 
                next_state = Transaction_2;
                wrt = 1'b1; // read MISO data
        end

        Transaction_2: begin // receive MISO data and update outputs
            if (done_signal) begin
                next_state = Idle;
                update = 1'b1;
            end
        end
    endcase
end 

endmodule