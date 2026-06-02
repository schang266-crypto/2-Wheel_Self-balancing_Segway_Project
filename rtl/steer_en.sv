module steer_en (clk, rst_n, lft_ld, rght_ld, en_steer, rider_off);

  input clk;				    // 50 MHz clock
  input rst_n;			    	// Active low asynch reset
  input [11:0] lft_ld;
  input [11:0] rght_ld;

  output logic en_steer;	// enables steering (goes to balance_cntrl)
  output logic rider_off;	// held high in intitial state when waiting for sum_gt_min

parameter FASTSIM = 1'b1;   // set to 1 for fastsim (only check bits [14:0] of clock timer)

logic sum_gt_min;	// asserted when left + right load cells > min rider weight
logic sum_lt_min;	// asserted when left + right load cells < min_rider_weight 
logic diff_gt_1_4;	// (rider not situated)
logic diff_gt_15_16;// (rider is stepping off)
logic clr_tmr;		// clears the 1.34sec timer
logic tmr_full;		// asserted when timer reaches 1.34sec

localparam MIN_RIDER_WT  = 12'h200;
localparam WT_HYSTERESIS = 8'h40;

logic        [12:0] sum;
logic signed [11:0] diff;
logic        [11:0] diff_abs;

assign sum  = lft_ld + rght_ld;
assign diff = lft_ld - rght_ld;

assign diff_abs = (diff < 0) ? -diff : diff; // absolute value

logic [12:0] sum_scale_15_16;
assign sum_scale_15_16 =  sum - (sum >> 4);  // sum * 15/16

logic [12:0] sum_scale_1_4;
assign sum_scale_1_4   =  sum >> 2;           // sum * 1/4


// Comparators
assign sum_lt_min    = (sum < (MIN_RIDER_WT - WT_HYSTERESIS));
assign sum_gt_min    = (sum > (MIN_RIDER_WT + WT_HYSTERESIS));
assign diff_gt_1_4   = (diff_abs > sum_scale_1_4);
assign diff_gt_15_16 = (diff_abs > sum_scale_15_16);


// Timer used to count 1.34 seconds of steady rider
// 1.34 second timer at 50 MHz clock
// 1.34 sec * 50,000,000 = 67,000,000 cycles
logic [25:0] counter; 
generate 
    if(FASTSIM) begin : fastsim_timer // Only check bits [14:0] for fastsim
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)
                counter <= 26'd0;
            else if (clr_tmr)
                counter <= 26'd0;
            else begin
                counter <= counter + 1;
                if (counter == 15'h7FFF)
                    tmr_full <= 1'b1;
            end
        end
    end
    else begin : normal_timer
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n)
                counter <= 26'd0;
            else if (clr_tmr)
                counter <= 26'd0;
            else begin
                counter <= counter + 1;
                if (counter == 26'd67000000)
                    tmr_full <= 1'b1;
            end
        end
    end
endgenerate


// State machine to control steering enable
typedef enum logic [1:0] {
    INITIAL       = 2'b00,
    STEADY        = 2'b01,
    STEER_ENABLED = 2'b10
  } state_t;

state_t current_state, next_state;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        current_state <= INITIAL;
    else
        current_state <= next_state;
end

always_comb begin
    rider_off = 0;
    en_steer  = 0;
    clr_tmr   = 1'b1;// Default to clearing timer
    next_state = current_state;  // Hold state by default
    
    case (current_state)
        INITIAL: begin // Waiting for rider to step on
            rider_off = 1'b1;
            if (sum_gt_min) // Rider steps on
                next_state = STEADY;
        end
        
        STEADY: begin // Rider is on board but not yet balanced
            if (sum_lt_min) begin
                rider_off = 1'b1;
                next_state = INITIAL; // Rider stepped off
            end
            else if (!diff_gt_1_4) begin // Rider is steady
                clr_tmr = 1'b0; // Start timer
                if (tmr_full) // wait 1.34 seconds
                    next_state = STEER_ENABLED; // Enable steering after timer if rider is steady
            end
        end
        
        STEER_ENABLED: begin
            en_steer = 1'b1; // Enable steering
            if (sum_lt_min) begin // Rider completely stepped off 
                rider_off = 1'b1;
                next_state = INITIAL; 
            end
            else if (diff_gt_15_16) // Rider is unstable, go back to steady state
                next_state = STEADY; 
        end
        default: begin
            next_state = INITIAL;
        end

    endcase
end
endmodule