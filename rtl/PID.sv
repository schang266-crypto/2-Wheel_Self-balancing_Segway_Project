module PID (input signed [15:0] ptch,
            input signed [15:0] ptch_rt,
            input clk,
            input rst_n,
            input pwr_up,
            input rider_off,
            input vld,

            output [7:0] ss_tmr,
            output logic signed [11:0]  PID_cntrl);

    parameter fast_sim = 1'b0;
    wire signed [9:0] ptch_pid_err_sat;
    assign ptch_pid_err_sat = ptch[15] == 1'b1 ? 
                            ((ptch[14:9] & 6'b111111) == 6'b111111 ? ptch[9:0] : 10'b1000000000) :
                            ((ptch[14:9] & 6'b111111) == 6'b000000 ? ptch[9:0] : 10'b0111111111);
    
    // localparam P_COEFF = 5'h09;
    // wire signed [14:0] P_term;
    // assign P_term = ptch_pid_err_sat * $signed(P_COEFF);

    wire signed [14:0] P_term;
    assign P_term = (ptch_pid_err_sat << 3) + ptch_pid_err_sat;

    // wire signed [14:0] I_term;
    // assign I_term = {{3{integrator[17]}},integrator[17:6]};

    //Integrator logic for PID Controller
    logic [17:0] integrator;

    wire signed [17:0] error_ext;
    assign error_ext = {{8{ptch_pid_err_sat[9]}},ptch_pid_err_sat[9:0]};

    wire signed [17:0] integrator_acc;
    assign integrator_acc = integrator + error_ext;

    wire ov;
    assign ov = ((integrator[17] == error_ext[17]) && (integrator_acc[17] != integrator[17])) ? 1'b1 : 1'b0;
    wire en_int;
    assign en_int = vld & ~ov;
    wire signed [17:0] integrator_chk;
    assign integrator_chk = en_int ? integrator_acc : integrator;

    wire signed [17:0] integrator_rst;
    assign integrator_rst = (rider_off) ? 18'h00000 : integrator_chk;

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            integrator <= 18'h00000;
        else
            integrator <= integrator_rst;
    end

    //I-term saturation logic. If in fast simulation mode, reduce precision to speed up sim time.
    
    wire signed [15:0] I_term_ext;

    generate
        if (fast_sim) begin
        // Fast-sim: coarse/low-precision I-term but keep widths consistent
        // Use equality '==' for comparisons, and make saturation values match 12-bit I_term
        // Here we emulate integrator[17:6] reduced precision by right-shifting further (example)
        // Adjust the right-shift amount (here: 4) if you intended a different scale.
        // Check integrator sign bit and top bits for saturation decision.
            wire signed [14:0] I_term;
            wire [1:0] top_bits;
            assign top_bits = integrator[16:15];

        // If integrator negative and top bits == 2'b11 => keep sliced value, else saturate negative
        // If integrator positive and top bits == 2'b00 => keep sliced value, else saturate positive
            assign I_term = (integrator[17] == 1'b1) ?
                            ((top_bits == 2'b11) ? integrator[15:1] : 15'b1000_0000_0000_000) :
                            ((top_bits == 2'b00) ? integrator[15:1] : 15'b0111_1111_1111_111);
            assign I_term_ext = {{1{I_term[14]}},I_term};

    end else begin
            wire signed [11:0] I_term;
        // Full-precision path (same as your original working code)
            assign I_term = integrator[17:6];               // 12-bit slice
            assign I_term_ext = {{4{I_term[11]}}, I_term};  // sign-extend to 16 bits
    end
endgenerate

    wire mux_en;
    wire [26:0] long_tmr_inc;
    reg [26:0] long_tmr;
    wire [26:0] long_tmr_chk;
    wire [26:0] long_tmr_rst;

    //Soft-start timer logic
    generate 
        if(fast_sim) begin
            assign mux_en = &long_tmr[26:19]; // Check if the upper 8 bits are all 1s
            assign long_tmr_inc = long_tmr + 27'h0000100; //Increment the timer
            assign long_tmr_chk = (mux_en) ? long_tmr : long_tmr_inc; // Mux to hold the timer value
            assign long_tmr_rst = (pwr_up) ? long_tmr_chk : 27'h0000000; // Reset the timer on power-up
        end
        else begin
            assign mux_en = &long_tmr[26:19]; // Check if the upper 7 bits are all 1s
            assign long_tmr_inc = long_tmr + 27'h0000001; //Increment the timer
            assign long_tmr_chk = (mux_en) ? long_tmr : long_tmr_inc; // Mux to hold the timer value
            assign long_tmr_rst = (pwr_up) ? long_tmr_chk : 27'h0000000; // Reset the timer on power-up
        end
    endgenerate

    always_ff @(posedge clk or negedge rst_n) begin
                if(!rst_n)
                    long_tmr <= 27'h0000000;
                else
                    long_tmr <= long_tmr_rst;
                end

    assign ss_tmr = long_tmr[26:19]; // Output the upper 7 bits as the soft-start timer

    wire signed [12:0] D_term;
    wire signed [12:0] ptch_term;
    assign ptch_term = {{3{ptch_rt[15]}},ptch_rt[15:6]};
    assign D_term = -ptch_term;

    wire signed [15:0] P_term_ext,D_term_ext;
    assign P_term_ext = {{1{P_term[14]}},P_term[14:0]};
    assign D_term_ext = {{3{D_term[12]}},D_term[12:0]};

    wire signed [15:0] PID_sum;
    assign PID_sum = P_term_ext + I_term_ext + D_term_ext;


    wire signed [11:0] PID_cntrl_ff;
    assign PID_cntrl_ff = PID_sum[15] == 1'b1 ? 
                      ((PID_sum[14:11] & 4'b1111) == 4'b1111 ? PID_sum[11:0] : 12'b100000000000) :
                      ((PID_sum[14:11] & 4'b1111) == 4'b0000 ? PID_sum[11:0] : 12'b011111111111);

    // logic signed [11:0] PID_cntrl;

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            PID_cntrl <= 16'h0000;
        else
            PID_cntrl <= PID_cntrl_ff;
    end

endmodule
