// Team STAY on Segway
// Shao-Kai Chang
// Arthur Stanson

module inertial_integrator( // create a low noise, no drift version of pitch
    input clk, 
    input rst_n,
    input vld,                        // High for a single clock cycle when new inertial readings are valid
    
    input signed [15:0] ptch_rt,      // from gyro (angular rate in pitch axis), quiet but drifts over time
    input signed [15:0] AZ,           // from noisy accelerometer (acceleration in Z axis), used to correct drift in pitch angle
                                      // but long term it can give corrections to pitch angle
    output logic signed [15:0] ptch   // fully compensated and fused pitch angle
);

// Gyro
localparam PTCH_RT_OFFSET = 16'h0050; // a fusion correction term
logic signed [15:0] ptch_rt_comp;
assign ptch_rt_comp = ptch_rt - PTCH_RT_OFFSET;

// Accelerometer
localparam AZ_OFFSET = 16'h00A0;
logic signed [15:0] AZ_comp;
assign AZ_comp = AZ - AZ_OFFSET;

logic signed [15:0] ptch_acc;
logic signed [25:0] ptch_acc_product;
assign ptch_acc_product = AZ_comp * $signed(327);  // Convert accelerometer reading to angle (trial and error derived)
assign ptch_acc = {{3{ptch_acc_product[25]}}, ptch_acc_product[25:13]}; // Pitch angle calculated from accelerometer only

// Fusion offset, to be added to the integrator based on comparison between accel-derived pitch and gyro-derived pitch
logic signed [11:0] fusion_ptch_offset; // use accerelometer to correct drift in pitch angle
always_comb begin
if (ptch_acc > ptch) 
    fusion_ptch_offset = 12'sd1024;
else
    fusion_ptch_offset = -12'sd1024;
end


logic signed [26:0] ptch_int;  // Pitch integrating accumulator
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        ptch_int <= 0;
    else if (vld)   // On every vld pulse this unit will integrate, the minus sign for correct direction
        ptch_int <= ptch_int - {{11{ptch_rt_comp[15]}}, ptch_rt_comp} + {{15{fusion_ptch_offset[11]}}, fusion_ptch_offset};
end

assign ptch = ptch_int[26:11];  // Derived by trial and error with actual Segway data

endmodule