module balance_cntrl
  #(parameter int fast_sim = 1)
  (clk, rst_n, vld, ptch, ptch_rt, pwr_up, rider_off, steer_pot, en_steer, 
          lft_spd, rght_spd, too_fast);

  input clk, rst_n;           // Clock and active low reset
  input logic vld;                  // Valid signal for pitch and pitch rate
  input logic signed [15:0] ptch;   // Pitch from segway math
  input logic signed [15:0] ptch_rt;// Pitch rate from segway math
  input logic pwr_up;               // Power up signal
  input logic rider_off;            // Rider off detection
  input logic [11:0] steer_pot;     // Steering enable value
  input logic en_steer;             // Enable steering
  
  output logic signed [11:0] lft_spd;      // Left motor speed
  output logic signed [11:0] rght_spd;     // Right motor speed
  output logic too_fast;                   // Excessive speed indication


  // Internal signals
  logic signed [11:0] PID_cntrl;           // Output from PID control
  logic [7:0] ss_tmr;                      // Internal timer signal

  // Instantiate PID controller (forward fast_sim)
  PID #(.fast_sim(fast_sim)) iPID(
      .clk(clk), .rst_n(rst_n), .vld(vld), .ptch(ptch), .ptch_rt(ptch_rt),
      .pwr_up(pwr_up), .rider_off(rider_off), .PID_cntrl(PID_cntrl), .ss_tmr(ss_tmr));

  // Connect PID outputs into segwaymath
  SegwayMath isegwaymath(
    .clk(clk), .rst_n(rst_n), .PID_cntrl(PID_cntrl), .ss_tmr(ss_tmr), .steer_pot(steer_pot), .en_steer(en_steer), .pwr_up(pwr_up),
    .lft_spd(lft_spd), .rght_spd(rght_spd), .too_fast(too_fast));
endmodule