module SegwayMath(input clk,
                  input rst_n,
                  input signed [11:0] PID_cntrl,
                  input [7:0] ss_tmr,
                  input [11:0] steer_pot,
                  input en_steer,
                  input pwr_up,
                  
                  output signed [11:0] lft_spd,
                  output signed [11:0] rght_spd,
                  output too_fast);

    wire signed [19:0] PID_cntrl_ss;
    assign PID_cntrl_ss = PID_cntrl * $signed({1'b0,ss_tmr}); // Converting ss_tmr to signed since Multiply is not an agnostic operation
    wire signed [11:0] PID_ss;
    assign PID_ss = PID_cntrl_ss[19:8]; // Taking upper bits to divide by 256

    wire [11:0] manual_steer;
    //We are limiting the steer potentiometer input to be between 0x200 and 0xE00 to avoid extreme steering
    assign manual_steer = (steer_pot < 12'h200)? 12'h200 :
                          (steer_pot > 12'hE00)? 12'hE00 :
                           steer_pot;
    wire signed [11:0] steer_adj;
    assign steer_adj = manual_steer - 12'h7ff; // Centering the steer potentiometer value around 0 (0x7ff acts as center value)
    wire signed [11:0] prop_steer_adj;
    assign prop_steer_adj = $signed(steer_adj[11:3]) + $signed(steer_adj[11:4]); // Scaling down the steer adjustment by multiplying with 3/16 to avoid excessive steering torque

    wire signed [12:0] PID_ss_ext_ff;
    assign PID_ss_ext_ff = {PID_ss[11],PID_ss};  //// Extending PID_ss to 13 bits to avoid overflow during addition with steering adjustment
    
    logic signed [12:0] PID_ss_ext;
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n)
            PID_ss_ext <= 13'h0000;
        else
            PID_ss_ext <= PID_ss_ext_ff;
    end
    
    
    wire signed [12:0] lft_in,rght_in;
    assign lft_in = PID_ss_ext + prop_steer_adj;
    assign rght_in = PID_ss_ext - prop_steer_adj;

    // Steering and Smooth Start combined torque values if en_steer is high, we use steering values, else we use smooth start values only
    wire signed [12:0] lft_torque_ff,rght_torque_ff;
    assign lft_torque_ff = (en_steer)? lft_in : PID_ss_ext;
    assign rght_torque_ff = (en_steer)? rght_in : PID_ss_ext;

    logic signed [12:0] lft_torque,rght_torque;
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            lft_torque <= 13'h0000;
            rght_torque <= 13'h0000;
        end
        else begin
            lft_torque <= lft_torque_ff;
            rght_torque <= rght_torque_ff;
        end
    end


    // There is a low gain region in the motor where it does not respond to low voltages due to inertia
    //To overcome this, we add a Minimum duty cycle so that the voltage can be driven higher to overcome inertia

    localparam MIN_DUTY = 13'h0A8; // Minimum duty cycle to overcome motor inertia
    localparam GAIN_MULT = 4'h4; // Gain multiplier of 14 to scale up the torque values
    localparam LOW_TORQUE_BAND = 7'h2A; // Torque band below which we apply deadband compensation

    wire signed [12:0] lft_torque_low;
    assign lft_torque_low = lft_torque - MIN_DUTY;

    wire signed [12:0] lft_torque_high;
    assign lft_torque_high = lft_torque + MIN_DUTY;

    wire signed [12:0] lft_torque_deadband;
    wire signed [12:0] lft_torque_comp;
    //If torque is negative, we subtract minimum duty cycle, else we add minimum duty cycle. This is to ensure that the deadband compensation is in the right direction
    assign lft_torque_comp = (lft_torque[12]) ? (lft_torque_low): (lft_torque_high);
    assign lft_torque_deadband = lft_torque * $signed(GAIN_MULT);

    wire signed [12:0] lft_torque_int;
    wire signed [12:0] lft_torque_abs;

    assign lft_torque_abs = (lft_torque[12]) ? -lft_torque : lft_torque;
    assign lft_torque_int = (lft_torque_abs > LOW_TORQUE_BAND) ? lft_torque_comp[12:0] : lft_torque_deadband[12:0];

    wire signed [12:0] lft_shaped;
    assign lft_shaped = (pwr_up) ? lft_torque_int[12:0] : 13'h0000;

    wire signed [12:0] rght_torque_low;
    assign rght_torque_low = rght_torque - MIN_DUTY;

    wire signed [12:0] rght_torque_high;
    assign rght_torque_high = rght_torque + MIN_DUTY;

    wire signed [12:0] rght_torque_deadband;
    wire signed [12:0] rght_torque_comp;
    assign rght_torque_comp = (rght_torque[12]) ? (rght_torque_low): (rght_torque_high);
    assign rght_torque_deadband = rght_torque * $signed(GAIN_MULT);

    wire signed [12:0] rght_torque_int;
    wire signed [12:0] rght_torque_abs;
    assign rght_torque_abs = (rght_torque[12]) ? -rght_torque : rght_torque;
    assign rght_torque_int = (rght_torque_abs > LOW_TORQUE_BAND) ? rght_torque_comp[12:0] : rght_torque_deadband[12:0];
    
    wire signed [12:0] rght_shaped;
    assign rght_shaped = (pwr_up) ? rght_torque_int[12:0] : 13'h0000;

    wire signed [11:0] lft_shaped_sat;
    wire signed [11:0] rght_shaped_sat;

    assign lft_shaped_sat = (lft_shaped[12]) ?
                            ((lft_shaped[11]==1'b1)? lft_shaped[11:0] : 12'b100000000000) :
                            ((lft_shaped[11]==1'b0)? lft_shaped[11:0] : 12'b011111111111);
    assign rght_shaped_sat = (rght_shaped[12]) ?
                            ((rght_shaped[11]==1'b1)? rght_shaped[11:0] : 12'b100000000000) :
                            ((rght_shaped[11]==1'b0)? rght_shaped[11:0] : 12'b011111111111);
    
    assign too_fast = ((lft_shaped_sat > $signed(12'd1536)) || (rght_shaped_sat > $signed(12'd1536))) ? 1'b1 : 1'b0;
    assign lft_spd = lft_shaped_sat;
    assign rght_spd = rght_shaped_sat;
endmodule






// // module SegwayMath(
// //     input clk,                // Added clock for pipelining
// //     input rst_n,              // Added reset for pipelining  
// //     input signed [11:0] PID_cntrl,
// //     input [7:0] ss_tmr,
// //     input [11:0] steer_pot,
// //     input en_steer,
// //     input pwr_up,
// //     output signed [11:0] lft_spd,
// //     output signed [11:0] rght_spd,
// //     output too_fast
// // );

// //     // ------------------------------------------------------------------
// //     // Stage 1: Multiplications and Initial Processing (Combinational)
// //     // ------------------------------------------------------------------
// //     wire signed [19:0] PID_cntrl_ss;
// //     wire signed [11:0] PID_ss;
// //     wire [11:0] manual_steer;
// //     wire signed [11:0] steer_adj;
// //     wire signed [11:0] prop_steer_adj;
    
// //     // These operations can be parallelized
// //     assign PID_cntrl_ss = PID_cntrl * $signed({1'b0,ss_tmr});
// //     assign PID_ss = PID_cntrl_ss[19:8];  // Divide by 256
    
// //     assign manual_steer = (steer_pot < 12'h200) ? 12'h200 :
// //                          (steer_pot > 12'hE00) ? 12'hE00 : steer_pot;
    
// //     assign steer_adj = manual_steer - 12'h7ff;
    
// //     // prop_steer_adj = steer_adj × 3/16 = (steer_adj/8 + steer_adj/16)
// //     assign prop_steer_adj = (steer_adj >>> 3) + (steer_adj >>> 4);
    
// //     // ------------------------------------------------------------------
// //     // Stage 1 Pipeline Registers
// //     // ------------------------------------------------------------------
// //     logic signed [11:0] PID_ss_reg;
// //     logic signed [11:0] prop_steer_adj_reg;
// //     logic en_steer_reg;
// //     logic pwr_up_reg;
    
// //     always_ff @(posedge clk or negedge rst_n) begin
// //         if (!rst_n) begin
// //             PID_ss_reg <= 12'h0;
// //             prop_steer_adj_reg <= 12'h0;
// //             en_steer_reg <= 1'b0;
// //             pwr_up_reg <= 1'b0;
// //         end else begin
// //             PID_ss_reg <= PID_ss;
// //             prop_steer_adj_reg <= prop_steer_adj;
// //             en_steer_reg <= en_steer;
// //             pwr_up_reg <= pwr_up;
// //         end
// //     end
    
// //     // ------------------------------------------------------------------
// //     // Stage 2: Torque Calculations (Combinational)
// //     // ------------------------------------------------------------------
// //     wire signed [12:0] PID_ss_ext;
// //     wire signed [12:0] lft_in, rght_in;
// //     wire signed [12:0] lft_torque, rght_torque;
    
// //     assign PID_ss_ext = {PID_ss_reg[11], PID_ss_reg};
// //     assign lft_in = PID_ss_ext + prop_steer_adj_reg;
// //     assign rght_in = PID_ss_ext - prop_steer_adj_reg;
    
// //     assign lft_torque = en_steer_reg ? lft_in : PID_ss_ext;
// //     assign rght_torque = en_steer_reg ? rght_in : PID_ss_ext;
    
// //     // Deadband compensation logic
// //     localparam MIN_DUTY = 13'h0A8;
// //     localparam GAIN_MULT = 4'h4;
// //     localparam LOW_TORQUE_BAND = 7'h2A;
    
// //     // Process left motor
// //     wire signed [12:0] lft_torque_abs = lft_torque[12] ? -lft_torque : lft_torque;
// //     wire signed [12:0] lft_torque_comp = lft_torque[12] ? 
// //                                         (lft_torque - MIN_DUTY) : 
// //                                         (lft_torque + MIN_DUTY);
// //     wire signed [12:0] lft_torque_deadband = lft_torque * GAIN_MULT;
// //     wire signed [12:0] lft_torque_int = (lft_torque_abs > LOW_TORQUE_BAND) ? 
// //                                        lft_torque_comp : lft_torque_deadband;
// //     wire signed [12:0] lft_shaped = pwr_up_reg ? lft_torque_int : 13'h0000;
    
// //     // Process right motor  
// //     wire signed [12:0] rght_torque_abs = rght_torque[12] ? -rght_torque : rght_torque;
// //     wire signed [12:0] rght_torque_comp = rght_torque[12] ?
// //                                          (rght_torque - MIN_DUTY) :
// //                                          (rght_torque + MIN_DUTY);
// //     wire signed [12:0] rght_torque_deadband = rght_torque * GAIN_MULT;
// //     wire signed [12:0] rght_torque_int = (rght_torque_abs > LOW_TORQUE_BAND) ?
// //                                         rght_torque_comp : rght_torque_deadband;
// //     wire signed [12:0] rght_shaped = pwr_up_reg ? rght_torque_int : 13'h0000;
    
// //     // ------------------------------------------------------------------
// //     // Stage 2 Pipeline Registers
// //     // ------------------------------------------------------------------
// //     logic signed [12:0] lft_shaped_reg, rght_shaped_reg;
    
// //     always_ff @(posedge clk or negedge rst_n) begin
// //         if (!rst_n) begin
// //             lft_shaped_reg <= 13'h0000;
// //             rght_shaped_reg <= 13'h0000;
// //         end else begin
// //             lft_shaped_reg <= lft_shaped;
// //             rght_shaped_reg <= rght_shaped;
// //         end
// //     end
    
// //     // ------------------------------------------------------------------
// //     // Stage 3: Saturation and Output (Combinational)
// //     // ------------------------------------------------------------------
// //     // Saturation function: 13-bit signed to 12-bit signed
// //     // function automatic signed [11:0] saturate_13to12(input signed [12:0] val);
// //     //     if (val > $signed(13'd2047))      // > +2047
// //     //         return 12'h7FF;
// //     //     else if (val < $signed(13'h1000)) // < -2048
// //     //         return 12'h800;
// //     //     else
// //     //         return val[11:0];
// //     // endfunction
    
// //     wire signed [11:0] lft_sat;
// //     wire signed [11:0] rght_sat; 

// //     assign lft_sat = (lft_shaped_reg [12]) ?
// //                             ((lft_shaped_reg[11]==1'b1)? lft_shaped_reg[11:0] : 12'b100000000000) :
// //                             ((lft_shaped_reg[11]==1'b0)? lft_shaped_reg[11:0] : 12'b011111111111);
// //     assign rght_sat = (rght_shaped_reg[12]) ?
// //                             ((rght_shaped_reg[11]==1'b1)? rght_shaped_reg[11:0] : 12'b100000000000) :
// //                             ((rght_shaped_reg[11]==1'b0)? rght_shaped_reg[11:0] : 12'b011111111111);
    
// //     wire too_fast_wire = (lft_sat > $signed(12'd1536)) || 
// //                         (rght_sat > $signed(12'd1536));
    
// //     // ------------------------------------------------------------------
// //     // Output Registers (Optional - depends on timing)
// //     // ------------------------------------------------------------------
// //     logic signed [11:0] lft_spd_reg, rght_spd_reg;
// //     logic too_fast_reg;
    
// //     always_ff @(posedge clk or negedge rst_n) begin
// //         if (!rst_n) begin
// //             lft_spd_reg <= 12'h0;
// //             rght_spd_reg <= 12'h0;
// //             too_fast_reg <= 1'b0;
// //         end else begin
// //             lft_spd_reg <= lft_sat;
// //             rght_spd_reg <= rght_sat;
// //             too_fast_reg <= too_fast_wire;
// //         end
// //     end
    
// //     assign lft_spd = lft_spd_reg;
// //     assign rght_spd = rght_spd_reg;
// //     assign too_fast = too_fast_reg;

// // endmodule

// // =============================================================
// // Optimized SegwayMath.sv (NO multipliers, pipelined, compact)
// // =============================================================
// module SegwayMath(
//     input  clk,
//     input  rst_n,
//     input  signed [11:0] PID_cntrl,
//     input  [7:0] ss_tmr,
//     input  [11:0] steer_pot,
//     input  en_steer,
//     input  pwr_up,
//     output signed [11:0] lft_spd,
//     output signed [11:0] rght_spd,
//     output too_fast
// );

//     // ------------------------------  
//     // Stage 1: scaling and steering
//     // ------------------------------  

//     // Remove expensive multiply: PID_ss = floor(PID_cntrl * ss_tmr / 256)
//     wire signed [11:0] PID_ss =
//         (PID_cntrl >>> 4) + (PID_cntrl >>> 5);

//     wire [11:0] manual_steer =
//         (steer_pot < 12'h200) ? 12'h200 :
//         (steer_pot > 12'hE00) ? 12'hE00 : steer_pot;

//     wire signed [11:0] steer_adj = manual_steer - 12'h7ff;

//     wire signed [11:0] prop_steer_adj =
//         (steer_adj >>> 3) + (steer_adj >>> 4);

//     // Stage-1 registers
//     logic signed [11:0] PID_ss_reg, prop_adj_reg;
//     logic signed [11:0] steer_adj_reg, manual_steer_reg;
//     logic en_steer_reg, pwr_up_reg;

//     always_ff @(posedge clk or negedge rst_n)
//         if (!rst_n) begin
//             PID_ss_reg     <= 0;
//             prop_adj_reg   <= 0;
//             steer_adj_reg  <= 0;
//             manual_steer_reg <= 0;
//             en_steer_reg   <= 0;
//             pwr_up_reg     <= 0;
//         end else begin
//             PID_ss_reg     <= PID_ss;
//             prop_adj_reg   <= prop_steer_adj;
//             steer_adj_reg  <= steer_adj;
//             manual_steer_reg <= manual_steer;
//             en_steer_reg   <= en_steer;
//             pwr_up_reg     <= pwr_up;
//         end

//     // ------------------------------
//     // Stage 2 torque
//     // ------------------------------
//     wire signed [12:0] PID_ext = {PID_ss_reg[11], PID_ss_reg};

//     wire signed [12:0] lft_base  = PID_ext + prop_adj_reg;
//     wire signed [12:0] rght_base = PID_ext - prop_adj_reg;

//     wire signed [12:0] lft_torque  = en_steer_reg ? lft_base  : PID_ext;
//     wire signed [12:0] rght_torque = en_steer_reg ? rght_base : PID_ext;

//     // Deadband: remove multiplier *4
//     localparam MIN_DUTY = 13'h0A8;
//     localparam LOW_TORQUE_BAND = 7'h2A;

//     // Left
//     wire signed [12:0] lft_abs = lft_torque[12] ? -lft_torque : lft_torque;
//     wire signed [12:0] lft_comp =
//         lft_torque[12] ? (lft_torque - MIN_DUTY) :
//                          (lft_torque + MIN_DUTY);

//     wire signed [12:0] lft_dead = lft_torque <<< 2; // replace multiply

//     wire signed [12:0] lft_int =
//         (lft_abs > LOW_TORQUE_BAND) ? lft_comp : lft_dead;

//     wire signed [12:0] lft_shape = pwr_up_reg ? lft_int : 13'sd0;

//     // Right
//     wire signed [12:0] rght_abs = rght_torque[12] ? -rght_torque : rght_torque;
//     wire signed [12:0] rght_comp =
//         rght_torque[12] ? (rght_torque - MIN_DUTY) :
//                           (rght_torque + MIN_DUTY);

//     wire signed [12:0] rght_dead = rght_torque <<< 2;

//     wire signed [12:0] rght_int =
//         (rght_abs > LOW_TORQUE_BAND) ? rght_comp : rght_dead;

//     wire signed [12:0] rght_shape = pwr_up_reg ? rght_int : 13'sd0;

//     // Stage 2 registers
//     logic signed [12:0] lft_reg, rght_reg;

//     always_ff @(posedge clk or negedge rst_n)
//         if (!rst_n) begin
//             lft_reg  <= 0;
//             rght_reg <= 0;
//         end else begin
//             lft_reg  <= lft_shape;
//             rght_reg <= rght_shape;
//         end

//     // ------------------------------
//     // Stage 3: saturation
//     // ------------------------------
//     function automatic signed [11:0] sat13(
//         input signed [12:0] x
//     );
//         if (x > 13'sd2047)       sat13 = 12'sd2047;
//         else if (x < -13'sd2048) sat13 = -12'sd2048;
//         else                     sat13 = x[11:0];
//     endfunction

//     wire signed [11:0] lft_sat  = sat13(lft_reg);
//     wire signed [11:0] rght_sat = sat13(rght_reg);

//     // Final registers
//     logic signed [11:0] lft_out, rght_out;
//     logic too_fast_reg;

//     always_ff @(posedge clk or negedge rst_n)
//         if (!rst_n) begin
//             lft_out <= 0;
//             rght_out <= 0;
//             too_fast_reg <= 0;
//         end else begin
//             lft_out <= lft_sat;
//             rght_out <= rght_sat;
//             too_fast_reg <= (lft_sat > 12'sd1536) ||
//                             (rght_sat > 12'sd1536);
//         end

//     assign lft_spd  = lft_out;
//     assign rght_spd = rght_out;
//     assign too_fast = too_fast_reg;

// endmodule

// =============================================================
// Area-optimized SegwayMath.sv (Compact, pipelined)
// =============================================================
// module SegwayMath(
//     input  clk,
//     input  rst_n,
//     input  signed [11:0] PID_cntrl,
//     input  [7:0] ss_tmr,
//     input  [11:0] steer_pot,
//     input  en_steer,
//     input  pwr_up,
//     output signed [11:0] lft_spd,
//     output signed [11:0] rght_spd,
//     output too_fast
// );

//     // ------------------------------
//     // Stage 1: scaling and steering
//     // ------------------------------
//     // Remove expensive multiply: PID_ss ≈ PID_cntrl * ss_tmr / 256
//     wire signed [11:0] PID_ss = (PID_cntrl >>> 4); // simpler shift
//     wire [11:0] manual_steer =
//         (steer_pot < 12'h200) ? 12'h200 :
//         (steer_pot > 12'hE00) ? 12'hE00 : steer_pot;

//     wire signed [11:0] steer_adj = manual_steer - 12'h7FF;

//     // Reduced prop_steer_adj calculation to a single shift
//     wire signed [11:0] prop_steer_adj = (steer_adj >>> 3) + (steer_adj >>> 4);

//     // Stage-1 pipeline registers
//     logic signed [11:0] PID_ss_reg, prop_adj_reg;
//     logic en_steer_reg, pwr_up_reg;

//     always_ff @(posedge clk or negedge rst_n)
//         if (!rst_n) begin
//             PID_ss_reg   <= 0;
//             prop_adj_reg <= 0;
//             en_steer_reg <= 0;
//             pwr_up_reg   <= 0;
//         end else begin
//             PID_ss_reg   <= PID_ss;
//             prop_adj_reg <= prop_steer_adj;
//             en_steer_reg <= en_steer;
//             pwr_up_reg   <= pwr_up;
//         end

//     // ------------------------------
//     // Stage 2: torque
//     // ------------------------------
//     wire signed [12:0] PID_ext = {PID_ss_reg[11], PID_ss_reg};

//     wire signed [12:0] lft_base  = PID_ext + prop_adj_reg;
//     wire signed [12:0] rght_base = PID_ext - prop_adj_reg;

//     wire signed [12:0] lft_torque  = en_steer_reg ? lft_base  : PID_ext;
//     wire signed [12:0] rght_torque = en_steer_reg ? rght_base : PID_ext;

//     // Deadband
//     localparam MIN_DUTY = 13'h0A8;
//     localparam LOW_TORQUE_BAND = 7'h2A;

//     // Left
//     wire signed [12:0] lft_abs = lft_torque[12] ? -lft_torque : lft_torque;
//     wire signed [12:0] lft_comp = lft_torque[12] ? (lft_torque - MIN_DUTY) :
//                                                (lft_torque + MIN_DUTY);
//     wire signed [12:0] lft_dead = lft_torque <<< 2;
//     wire signed [12:0] lft_int = (lft_abs > LOW_TORQUE_BAND) ? lft_comp : lft_dead;
//     wire signed [12:0] lft_shape = pwr_up_reg ? lft_int : 13'sd0;

//     // Right
//     wire signed [12:0] rght_abs = rght_torque[12] ? -rght_torque : rght_torque;
//     wire signed [12:0] rght_comp = rght_torque[12] ? (rght_torque - MIN_DUTY) :
//                                                  (rght_torque + MIN_DUTY);
//     wire signed [12:0] rght_dead = rght_torque <<< 2;
//     wire signed [12:0] rght_int = (rght_abs > LOW_TORQUE_BAND) ? rght_comp : rght_dead;
//     wire signed [12:0] rght_shape = pwr_up_reg ? rght_int : 13'sd0;

//     // Stage 2 registers
//     logic signed [12:0] lft_reg, rght_reg;
//     always_ff @(posedge clk or negedge rst_n)
//         if (!rst_n) begin
//             lft_reg  <= 0;
//             rght_reg <= 0;
//         end else begin
//             lft_reg  <= lft_shape;
//             rght_reg <= rght_shape;
//         end

//     // ------------------------------
//     // Stage 3: saturation & output
//     // ------------------------------
//     logic signed [11:0] lft_out, rght_out;
//     logic too_fast_reg;

//     always_ff @(posedge clk or negedge rst_n)
//         if (!rst_n) begin
//             lft_out      <= 0;
//             rght_out     <= 0;
//             too_fast_reg <= 0;
//         end else begin
//             // Inline saturation
//             lft_out  <= (lft_reg > 13'sd2047) ? 12'sd2047 :
//                          (lft_reg < -13'sd2048) ? -12'sd2048 : lft_reg[11:0];
//             rght_out <= (rght_reg > 13'sd2047) ? 12'sd2047 :
//                          (rght_reg < -13'sd2048) ? -12'sd2048 : rght_reg[11:0];
//             too_fast_reg <= (lft_out > 12'sd1536) || (rght_out > 12'sd1536);
//         end

//     assign lft_spd  = lft_out;
//     assign rght_spd = rght_out;
//     assign too_fast = too_fast_reg;

// endmodule

// module SegwayMath(input clk, rst_n,
//                   input signed [11:0] PID_cntrl,
//                   input [7:0] ss_tmr,
//                   input [11:0] steer_pot,
//                   input en_steer,
//                   input pwr_up,
//                   output signed [11:0] lft_spd,
//                   output signed [11:0] rght_spd,
//                   output too_fast);

//     wire signed [19:0] PID_cntrl_ss;
//     assign PID_cntrl_ss = PID_cntrl * $signed({1'b0,ss_tmr}); // Converting ss_tmr to signed since Multiply is not an agnostic operation
//     wire signed [11:0] PID_ss;
//     assign PID_ss = PID_cntrl_ss[19:8]; // Taking upper bits to divide by 256

//     wire [11:0] manual_steer;
//     //We are limiting the steer potentiometer input to be between 0x200 and 0xE00 to avoid extreme steering
//     assign manual_steer = (steer_pot < 12'h200)? 12'h200 :
//                           (steer_pot > 12'hE00)? 12'hE00 :
//                            steer_pot;
//     wire signed [11:0] steer_adj;
//     assign steer_adj = manual_steer - 12'h7ff; // Centering the steer potentiometer value around 0 (0x7ff acts as center value)
//     wire signed [11:0] prop_steer_adj;
//     assign prop_steer_adj = $signed(steer_adj[11:3]) + $signed(steer_adj[11:4]); // Scaling down the steer adjustment by multiplying with 3/16 to avoid excessive steering torque

//     wire signed [12:0] PID_ss_ext;
//     assign PID_ss_ext = {PID_ss[11],PID_ss};  //// Extending PID_ss to 13 bits to avoid overflow during addition with steering adjustment
//     wire signed [12:0] lft_in,rght_in;
//     assign lft_in = PID_ss_ext + prop_steer_adj;
//     assign rght_in = PID_ss_ext - prop_steer_adj;

//     // Steering and Smooth Start combined torque values if en_steer is high, we use steering values, else we use smooth start values only
//     wire signed [12:0] lft_torque,rght_torque;
//     assign lft_torque = (en_steer)? lft_in : PID_ss_ext;
//     assign rght_torque = (en_steer)? rght_in : PID_ss_ext;

//     logic [12:0] lft_torque_reg, rght_torque_reg;

//     always_ff @(posedge clk or negedge rst_n) begin
//         if (!rst_n) begin
//             lft_torque_reg <= 1'b0;
//             rght_torque_reg <= 1'b0;
//         end else begin
//             lft_torque_reg <= lft_torque;
//             rght_torque_reg <= rght_torque;
//         end
//     end

//     // There is a low gain region in the motor where it does not respond to low voltages due to inertia
//     //To overcome this, we add a Minimum duty cycle so that the voltage can be driven higher to overcome inertia

//     localparam MIN_DUTY = 13'h0A8; // Minimum duty cycle to overcome motor inertia
//     localparam GAIN_MULT = 4'h4; // Gain multiplier of 14 to scale up the torque values
//     localparam LOW_TORQUE_BAND = 7'h2A; // Torque band below which we apply deadband compensation

//     wire signed [12:0] lft_torque_low;
//     assign lft_torque_low = lft_torque_reg - MIN_DUTY;

//     wire signed [12:0] lft_torque_high;
//     assign lft_torque_high = lft_torque_reg + MIN_DUTY;

//     wire signed [12:0] lft_torque_deadband;
//     wire signed [12:0] lft_torque_comp;
//     //If torque is negative, we subtract minimum duty cycle, else we add minimum duty cycle. This is to ensure that the deadband compensation is in the right direction
//     assign lft_torque_comp = (lft_torque_reg[12]) ? (lft_torque_low): (lft_torque_high);
//     assign lft_torque_deadband = lft_torque_reg * $signed(GAIN_MULT);

//     wire signed [12:0] lft_torque_int;
//     wire signed [12:0] lft_torque_abs;

//     assign lft_torque_abs = (lft_torque_reg[12]) ? -lft_torque_reg : lft_torque_reg;
//     assign lft_torque_int = (lft_torque_abs > LOW_TORQUE_BAND) ? lft_torque_comp[12:0] : lft_torque_deadband[12:0];

//     wire signed [12:0] lft_shaped;
//     assign lft_shaped = (pwr_up) ? lft_torque_int[12:0] : 13'h0000;

//     wire signed [12:0] rght_torque_low;
//     assign rght_torque_low = rght_torque_reg - MIN_DUTY;

//     wire signed [12:0] rght_torque_high;
//     assign rght_torque_high = rght_torque_reg + MIN_DUTY;

//     wire signed [12:0] rght_torque_deadband;
//     wire signed [12:0] rght_torque_comp;
//     assign rght_torque_comp = (rght_torque_reg[12]) ? (rght_torque_low): (rght_torque_high);
//     assign rght_torque_deadband = rght_torque_reg * $signed(GAIN_MULT);

//     wire signed [12:0] rght_torque_int;
//     wire signed [12:0] rght_torque_abs;
//     assign rght_torque_abs = (rght_torque_reg[12]) ? -rght_torque_reg : rght_torque_reg;
//     assign rght_torque_int = (rght_torque_abs > LOW_TORQUE_BAND) ? rght_torque_comp[12:0] : rght_torque_deadband[12:0];
    
//     wire signed [12:0] rght_shaped;
//     assign rght_shaped = (pwr_up) ? rght_torque_int[12:0] : 13'h0000;

//     wire signed [11:0] lft_shaped_sat;
//     wire signed [11:0] rght_shaped_sat;

//     // logic [12:0] lft_shaped_reg, rght_shaped_reg;
//     // always_ff @(posedge clk or negedge rst_n) begin
//     //     if (!rst_n) begin
//     //         lft_shaped_reg <= 1'b0;
//     //         rght_shaped_reg <= 1'b0;
//     //     end else begin
//     //         lft_shaped_reg <= lft_shaped;
//     //         rght_shaped_reg <= rght_shaped;
//     //     end
//     // end

//     assign lft_shaped_sat = (lft_shaped[12]) ?
//                             ((lft_shaped[11]==1'b1)? lft_shaped[11:0] : 12'b100000000000) :
//                             ((lft_shaped[11]==1'b0)? lft_shaped[11:0] : 12'b011111111111);
//     assign rght_shaped_sat = (rght_shaped[12]) ?
//                             ((rght_shaped[11]==1'b1)? rght_shaped[11:0] : 12'b100000000000) :
//                             ((rght_shaped[11]==1'b0)? rght_shaped[11:0] : 12'b011111111111);
//     wire too_fast_wire;
//     assign too_fast_wire = ((lft_shaped_sat > $signed(12'd1536)) || (rght_shaped_sat > $signed(12'd1536))) ? 1'b1 : 1'b0;

//     // logic [12:0] lft_spd_reg, rght_spd_reg;
//     // logic too_fast_reg;

//     // always_ff @(posedge clk or negedge rst_n) begin
//     //     if (!rst_n) begin
//     //         lft_spd_reg <= 1'b0;
//     //         rght_spd_reg <= 1'b0;
//     //         too_fast_reg <= 1'b0;
//     //     end else begin
//     //         lft_spd_reg <= lft_shaped;
//     //         rght_spd_reg <= rght_shaped;
//     //         too_fast_reg <= too_fast_wire;
//     //     end
//     // end

    
//     assign lft_spd = lft_shaped_sat;
//     assign rght_spd = rght_shaped_sat;
//     assign too_fast = too_fast_reg;
// endmodule


