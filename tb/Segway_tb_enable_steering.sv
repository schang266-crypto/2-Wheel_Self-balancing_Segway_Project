`timescale 1ns/1ps
module Segway_tb_enable_steering();

    import tasks::*;  //Importing all the things from package "tasks" :)

    /*Interconnects to the Device_Under_Test. 
    These are modeled for now, but after synthesis,
    we will be able to use tangible sensor inputs on our DUT! :) */
    wire SS_n, SCLK, MISO, MOSI, INT;  //Inertial sensor connections to the Digital SegWay Model
    wire A2D_ss_n, A2D_SCLK, A2D_MISO, A2D_MOSI; //A2D Converter connections to the Digital SegWay Model
    wire RX_TX;
    wire PWM1_lft, PWM1_rght;
    wire PWM2_lft, PWM2_rght;
    wire piezo, piezo_n;
    logic cmd_sent;
    logic rst_n;
    wire [11:0] lft_ld, rght_ld;		// measurements from load cells
    logic en_steer;
    logic rider_off;						// from steer_en to auth_blk

    

    //Stimulus to the Device_Under_Test
    logic clk,RST_n;
    logic [7:0] cmd;
    logic send_cmd;
    logic signed [15:0] rider_lean;
    logic [11:0] ld_cell_lft, ld_cell_rght, steerPot, batt; //A2D values which A2D will convert to Digital values
    logic OVR_I_lft, OVR_I_rght;

    ////////////////////////////////////////////////////////////////
    // Instantiate Physical Model of Segway with Inertial sensor //
    //////////////////////////////////////////////////////////////	
    SegwayModel iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(SS_n),.SCLK(SCLK),
                      .MISO(MISO),.MOSI(MOSI),.INT(INT),.PWM1_lft(PWM1_lft),
				      .PWM2_lft(PWM2_lft),.PWM1_rght(PWM1_rght),
				      .PWM2_rght(PWM2_rght),.rider_lean(rider_lean));
    
    /////////////////////////////////////////////////////////
    // Instantiate Model of A2D for load cell and battery //
    ///////////////////////////////////////////////////////
    ADC128S_FC iA2D(.clk(clk),.rst_n(RST_n),.SS_n(A2D_SS_n),.SCLK(A2D_SCLK),
                    .MISO(A2D_MISO),.MOSI(A2D_MOSI),.ld_cell_lft(ld_cell_lft),.ld_cell_rght(ld_cell_rght),
			        .steerPot(steerPot),.batt(batt));

    ////// Instantiate DUT ////////
    Segway iDUT(.clk(clk),.RST_n(RST_n),.INERT_SS_n(SS_n),.INERT_MOSI(MOSI),
                .INERT_SCLK(SCLK),.INERT_MISO(MISO),.INERT_INT(INT),.A2D_SS_n(A2D_SS_n),
			    .A2D_MOSI(A2D_MOSI),.A2D_SCLK(A2D_SCLK),.A2D_MISO(A2D_MISO),
			    .PWM1_lft(PWM1_lft),.PWM2_lft(PWM2_lft),.PWM1_rght(PWM1_rght),
			    .PWM2_rght(PWM2_rght),.OVR_I_lft(OVR_I_lft),.OVR_I_rght(OVR_I_rght),
			    .piezo_n(piezo_n),.piezo(piezo),.RX(RX_TX));

    assign en_steer = iDUT.en_steer;
    assign rider_off = iDUT.rider_off;

    //// Instantiate UART_tx (mimics command from BLE module) //////
    UART_tx iTX(.clk(clk),.rst_n(rst_n),.TX(RX_TX),.trmt(send_cmd),.tx_data(cmd),.tx_done(cmd_sent));

    /////////////////////////////////////
    // Instantiate reset synchronizer //
    ///////////////////////////////////
    rst_synch iRST(.clk(clk),.RST_n(RST_n),.rst_n(rst_n));

    initial begin
        $display("Begin testing test case: Enable Steer State Machine");

	    Initialize(.clk(clk), .rst_n(RST_n), .send_cmd(send_cmd), .OVR_I_lft(OVR_I_lft), .OVR_I_rght(OVR_I_rght), .rider_lean(rider_lean), 
				.ld_cell_lft(ld_cell_lft), .ld_cell_rght(ld_cell_rght), .batt(batt), .steerPot(steerPot));
				
	    Check_Enable_Steering(.clk(clk),.ld_cell_lft(ld_cell_lft),.ld_cell_rght(ld_cell_rght),.send_cmd(send_cmd),.cmd(cmd),.cmd_sent(cmd_sent),.en_steer(en_steer),.rider_off(rider_off));

        $display("Finished testing test case: Enable Steer State Machine!");
        $stop();

    end

    always
        #10 clk = ~clk;

endmodule