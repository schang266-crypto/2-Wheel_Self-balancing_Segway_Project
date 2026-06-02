module SPI_mnrch(clk, rst_n, SS_n, SCLK, MOSI, MISO, wrt, wrt_data, done, rd_data);
  input clk, rst_n;
  input MISO;
  input wrt;
  input [15:0] wrt_data;

  output logic SCLK; // serial clock to slave, for synchronizing data transfer
  output logic SS_n; // active low slave select
  output logic done;
  output logic [15:0] rd_data;
  output logic MOSI;


// state machine declaration
typedef enum logic [1:0] {
    IDLE,
    FRONT_PORCH,
    TRANSFER,
    BACK_PORCH
    } state_t;
state_t current_state, next_state;


// sclk divider = 1/16 of clk 
logic [3:0] SCLK_div;
logic ld_sclk;
logic smpl;
logic shft_im;
logic sclk_idle; // when high, SCLK is idle (high)

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        SCLK_div <= 4'b1000; // SLCK active high at reset
    else if (sclk_idle)
        SCLK_div <= 4'b1000; // idle SCLK = high 
    else if (ld_sclk)
        SCLK_div <= 4'b1011;
    else
        SCLK_div <= SCLK_div + 1;
end

assign SCLK = SCLK_div[3];
assign smpl = (SCLK_div == 4'b0111); // sample at the rising edge of SCLK
assign shft_im = ((SCLK_div == 4'b1111) && !(current_state == FRONT_PORCH));  // shift at the falling edge of SCLK & skip shifting on front porch


// bit counter
logic [3:0] bit_cntr;
logic init;
logic done15;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        bit_cntr <= 0;
    else if (init || done15) 
        bit_cntr <= 0;
    else if (shft_im) 
        bit_cntr <= bit_cntr + 1;
end
assign done15 = &bit_cntr;  // done when all 15 bits shifted


// shift registers
logic MISO_smpl;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        MISO_smpl <= 0;
    else if (smpl) 
        MISO_smpl <= MISO; // smaple MISO at rising edge of SCLK
end

logic [15:0] shft_reg;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        shft_reg <= 0;
    else if (init) 
        shft_reg <= wrt_data;
    else if (shft_im)  
        shft_reg <= {shft_reg[14:0], MISO_smpl};
end

assign MOSI = shft_reg[15];
assign rd_data = shft_reg;  // when done, rd_data has received the 16-bit data from MISO


// state machine
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        current_state <= IDLE;
    else 
        current_state <= next_state;
end

logic set_done;
always_comb begin
    // default values
    next_state = current_state;
    ld_sclk = 0;
    init = 0;
    set_done = 0;
    sclk_idle = 0;

    case (current_state)
        IDLE: begin
            sclk_idle = 1;
            if (wrt) begin
                init = 1;
                ld_sclk = 1;
                next_state = FRONT_PORCH;
            end
        end

        FRONT_PORCH: begin // If it shifts first, it will miss sampling the first bit
            if (smpl)
                next_state = TRANSFER;
        end

        TRANSFER: begin
            if (done15) // when done shifting 15 bits, go to back porch state to do one more shift
                next_state = BACK_PORCH;
        end

        BACK_PORCH: begin // Shift the last bit out
            if (shft_im) begin  // wait for one more SCLK cycle
                set_done = 1;   // finish all the 16 bits transfer
                sclk_idle = 1;
                next_state = IDLE;
            end
        end

    endcase
end

// Done flip-flop
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        done <= 0;
    else if (init)
        done <= 0;
    else if (set_done)
        done <= 1;

end

// SS_n flip-flop
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)
        SS_n <= 0;
    else if (init)
        SS_n <= 0;
    else if (set_done)
        SS_n <= 1; // when done, data transfer is complete, slave can use the data

end

endmodule
