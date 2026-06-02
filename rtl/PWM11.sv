module PWM11 (clk, rst_n, duty, PWM1, PWM2, PWM_synch, ovr_I_blank);

input clk;
input rst_n;
input [10:0] duty; // Width when PWM = 1

output logic PWM1, PWM2;  //ensure both MOSFETs in a stack are never on at same time
output logic PWM_synch;
output logic ovr_I_blank;

logic [10:0] cnt; // 11 bits to count to 2048
localparam [11:0] NONOVERLAP = 12'h040; // (128 clocks) dead time between PWM1 and PWM2 to avoid shoot through in MOSFET H-bridge

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        cnt <= 0;
    else
        cnt <= cnt + 1;
end

// when cnt is all zeros
assign PWM_synch = ~|cnt; // changes to duty should be synched w/ the PWM cycle so changes to duty do not occur in the middle of a PWM period
// ignore the over current detect if we are in the first 128 clock cycles of either PWM1 or PWM2 = 1
assign ovr_I_blank = ((cnt > NONOVERLAP) && (cnt < (NONOVERLAP + 11'd128))) ||
       ((cnt > (NONOVERLAP + duty)) && (cnt < (NONOVERLAP + duty + 11'd128)));


always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        PWM1 <= 1'b0;
    else if (cnt >= NONOVERLAP && cnt < duty) // PWM1 run from NONOVERLAP to duty
        PWM1 <= 1'b1;   // Set
    else if (cnt >= duty || cnt < NONOVERLAP)
        PWM1 <= 1'b0;   // Reset
end

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        PWM2 <= 1'b0;
    else if (cnt >= (NONOVERLAP+duty) && !(&cnt)) // PWM2 run from NONOVERLAP + duty to end of cycle
        PWM2 <= 1'b1;   // Set
    else if (cnt < (NONOVERLAP+duty) || (&cnt))
        PWM2 <= 1'b0;   // Reset
end

endmodule