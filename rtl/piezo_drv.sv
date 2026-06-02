module piezo_drv #(parameter fast_sim = 1)(
    input clk,
    input rst_n,
    input en_steer,
    input too_fast,
    input batt_low,
    output logic piezo,
    output logic piezo_n
);

// State Machine Definition
typedef enum logic [3:0] {
    IDLE,
    TONE_G6,
    TONE_C7,
    TONE_E7,
    TONE_G7,
    TONE_E7_2,
    TONE_G7_2
} state_t;
state_t current_state, next_state;

logic clear_timer; 
logic [14:0] desired_freq_timer; // Time period needed to generate desired frequency

// Fast simulation increment values
logic [25:0] duration_inc;
logic [27:0] repeat_inc;
logic [14:0] freq_inc;

generate
    if (fast_sim) begin : gen_fast
        assign duration_inc = 26'd640;  // Increment by 64 for fast simulation
        assign repeat_inc = 28'd640;    // Increment by 64 for fast simulation
        assign freq_inc = 15'd640;      // Increment by 64 for fast simulation
    end
    else begin : gen_normal
        assign duration_inc = 26'd1;   // Increment by 1 for normal operation
        assign repeat_inc = 28'd1;     // Increment by 1 for normal operation
        assign freq_inc = 15'd1;       // Increment by 1 for normal operation
    end
endgenerate


// Duration Timer, counts up to various timeouts for different tones
logic [25:0] duration_timer;
logic timeout_22;
logic timeout_23;
logic timeout_25;
logic timeout_23_22;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        duration_timer <= 26'd0;
    else if (clear_timer)
        duration_timer <= 26'd0;
    else 
        duration_timer <= duration_timer + duration_inc;
end

// Timeout values adjusted for fast simulation
assign timeout_22 =    (duration_timer >= 26'h400000);  // 2^22 
assign timeout_23 =    (duration_timer >= 26'h800000);  // 2^23 
assign timeout_23_22 = (duration_timer >= 26'hC00000);  // 2^23 + 2^22
assign timeout_25 =    (duration_timer >= 26'h2000000); // 2^25


// Repeat Timer (count to 3 seconds for 50 MHz clock), to space out en_steer & batt_low fanfare repeats
logic [27:0] repeat_timer;
logic repeat_timeout;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        repeat_timer <= 28'd0;
    else if (clear_timer) 
        repeat_timer <= 28'd0;
    else 
        repeat_timer <= repeat_timer + repeat_inc;
end
// 3 seconds at 50 MHz = 150,000,000 cycles
assign repeat_timeout = (repeat_timer >= 28'h5F5E100); 


// Frequency Timer, how fast to toggle piezo output
logic [14:0] freq_timer;
logic freq_timeout;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) 
        freq_timer <= 15'd0;
    else if (clear_timer)
        freq_timer <= 15'd0;
    else if (freq_timeout)
        freq_timer <= 15'd0;
    else 
        freq_timer <= freq_timer + freq_inc;
end
assign freq_timeout = (freq_timer >= desired_freq_timer);  

// Generating square wave for piezo
assign piezo = (!rst_n) ? 1'b0 : (freq_timer < (desired_freq_timer >> 1)) ? 1'b1 : 1'b0; // 50% duty cycle
assign piezo_n = ~piezo; 


// State Machine Logic
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        current_state <= IDLE;
    else
        current_state <= next_state;    
end

always_comb begin
    clear_timer = 0;
    next_state = current_state;
    desired_freq_timer = 15'd0; // No sound output from piezo

    // Priority: too_fast > batt_low > en_steer
    // en_steer sequence : G6 -> C7 -> E7 -> G7 -> E7_2 -> G7_2 -> IDLE
    // batt_low sequence : G7_2 -> E7_2 -> G7 -> E7 ->C7 -> G6 -> IDLE
    // too_fast sequence : G6 -> C7 -> E7
    case (current_state)
        IDLE: begin
            if (too_fast) begin                         // too fast has the highest priority without having to wait
                clear_timer = 1;
                next_state = TONE_G6;
            end
            else if (batt_low && repeat_timeout) begin  // wait for 3 seconds before starting batt_low fanfare
                clear_timer = 1;
                next_state = TONE_G7_2;
            end
            else if (en_steer && repeat_timeout) begin  // wait for 3 seconds before starting en_steer fanfare
                clear_timer = 1;
                next_state = TONE_G6;
            end
        end

        TONE_G6: begin   // too_fast fanfare sequence starts here
            desired_freq_timer = 15'd31888;  // 50MHz / 1568 Hz for G6

            if (timeout_23 && too_fast) begin
                clear_timer = 1;
                next_state = TONE_C7;
            end
            else if (timeout_23 && batt_low) begin
                clear_timer = 1;
                next_state = IDLE;
            end
            else if (timeout_23 && en_steer) begin
                clear_timer = 1;
                next_state = TONE_C7;
            end
        end

        TONE_C7: begin
            desired_freq_timer = 15'd23889;  // 50MHz / 2093 Hz for C7

            if (timeout_23 && too_fast) begin
                clear_timer = 1;
                next_state = TONE_E7;
            end
            else if (timeout_23 && batt_low) begin
                clear_timer = 1;
                next_state = TONE_G6;
            end
            else if (timeout_23 && en_steer) begin
                clear_timer = 1;
                next_state = TONE_E7;
            end
        end

        TONE_E7: begin   
            desired_freq_timer = 15'd18961;  // 50MHz / 2637 Hz for E7

            if (timeout_23 && too_fast) begin
                clear_timer = 1;
                next_state = TONE_G6;
            end
            else if (timeout_23 && batt_low) begin
                clear_timer = 1;
                next_state = TONE_C7;
            end
            else if (timeout_23 && en_steer) begin
                clear_timer = 1;
                next_state = TONE_G7;
            end
        end

        TONE_G7: begin
            desired_freq_timer = 15'd15944;  // 50MHz / 3136 Hz for G7

            if (timeout_23_22 && too_fast) begin // if too_fast asserted jump to its 3-note loop
                clear_timer = 1;
                next_state = TONE_G6;
            end
            else if (timeout_23_22 && batt_low) begin // if batt_low asserted mid-forward, switch to backward full start
                clear_timer = 1;
                next_state = TONE_E7;
            end
            else if (timeout_23_22 && en_steer) begin
                clear_timer = 1;
                next_state = TONE_E7_2;
            end
        end

        TONE_E7_2: begin
            desired_freq_timer = 15'd18961;  // 50MHz / 2637 Hz for E7

            if (timeout_22 && too_fast) begin
                clear_timer = 1;
                next_state = TONE_G6;
            end
            else if (timeout_22 && batt_low) begin
                clear_timer = 1;
                next_state = TONE_G7;
            end
            else if (timeout_22 && en_steer) begin
                clear_timer = 1;
                next_state = TONE_G7_2;
            end
        end

        TONE_G7_2: begin   // batt_low fanfare sequence starts here
            desired_freq_timer = 15'd15944;  // 50MHz / 3136 Hz for G7

            if (timeout_25 && too_fast) begin
                clear_timer = 1;
                next_state = TONE_G6;
            end
            else if (timeout_25 && batt_low) begin
                clear_timer = 1;
                next_state = TONE_E7_2; 
            end
            else if (timeout_25 && en_steer) begin
                clear_timer = 1;
                next_state = IDLE;
            end
            // else if (timeout_25) next_state = IDLE;
        end
        
        default: begin
            desired_freq_timer = 15'd0;
            next_state = IDLE;
        end
    endcase
end

endmodule