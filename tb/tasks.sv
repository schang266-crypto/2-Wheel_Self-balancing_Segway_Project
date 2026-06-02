`timescale 1ns/1ps
package tasks;

//Basic initialization
task automatic Initialize(ref clk, ref rst_n, ref send_cmd, ref OVR_I_lft, ref OVR_I_rght, ref reg signed [15:0] rider_lean,
						ref reg[11:0] ld_cell_lft, ref reg[11:0] ld_cell_rght, ref reg[11:0] batt, ref reg[11:0] steerPot);
	begin
		clk = 0;
		rst_n = 0;
		send_cmd = 0;
		OVR_I_lft = 0;
		OVR_I_rght = 0;
		rider_lean = 0;
		ld_cell_lft = 0;
		ld_cell_rght = 0;
		batt = 12'h801;     // batt not low
		steerPot = 12'h7ff; // mid point 
		repeat(2) @(negedge clk);	
		rst_n = 1;
		repeat(500) @(posedge clk);
	end
endtask : Initialize

//Check if the command is sent successfully
task automatic Check_SendCmd(ref clk, ref send_cmd, input [7:0] cmd2send, ref reg[7:0] cmd, ref cmd_sent);
	begin
		@(negedge clk);
		cmd = cmd2send;
		send_cmd = 1;
		@(negedge clk);
		send_cmd = 0;
	
		fork
			begin: timeout;
				repeat(1000000) @(negedge clk);
				$display("Command not sent");
				$stop();
			end

			begin
				@(posedge cmd_sent);
				$display("Command sent successfully");
				disable timeout;
			end
		join
		
	end

endtask : Check_SendCmd


// Test theta_platform through applying rider_lean
task automatic Check_theta_platform(ref clk, ref reg [11:0]ld_cell_lft, ref reg [11:0]ld_cell_rght,
 ref send_cmd, ref reg signed[15:0] rider_lean, ref reg [7:0] cmd, ref cmd_sent);
	begin
		//Sum > 12'h240 && !diff_gt_1_4 => en_steer = 1 
		ld_cell_lft = 12'h200;
		ld_cell_rght = 12'h200;	

		// Power up the Segway with the power-up command 8'h47
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h47), .cmd(cmd), .cmd_sent(cmd_sent));
		
		//Wait for the ss_tmr to count 
		repeat(500000)@(negedge clk);
		
		//Positive rider_lean
		$display("Changing rider lean to positive");
		rider_lean = 16'h1fff;
		repeat(800000)@(negedge clk);

		//Step rider_lean back to zero
		rider_lean = 16'h0000;
		repeat(500000)@(negedge clk);

		//Negative rider_lean
		$display("Changing rider lean to negative");
		rider_lean = 16'hE000;
		repeat(800000)@(negedge clk);

		//Step rider_lean back to zero
		rider_lean = 16'h0000;
		repeat(500000)@(negedge clk);

	end
endtask : Check_theta_platform


// Test the function of the piezo and make sure all the notes play
task automatic Check_piezo(ref clk, ref reg [11:0]ld_cell_lft, ref reg [11:0]ld_cell_rght, ref reg signed[15:0] rider_lean, 
ref send_cmd, ref reg [7:0] cmd, ref cmd_sent, ref reg [11:0]steerPot, ref reg[11:0] batt);
	begin
		// Power up the Segway with the power-up command 8'h47
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h47), .cmd(cmd), .cmd_sent(cmd_sent));

		//Sum > 12'h240 && !diff_gt_1_4 => en_steer = 1, piezo should play en_steer fanfare
		$display("Start testing en_steer fanfare at time: %0t!", $time);
		ld_cell_lft = 12'h200;
		ld_cell_rght = 12'h200;
		batt = 12'h801; // battery not low
		repeat(500000) @(negedge clk);

		// By adding battery low, piezo should have the priority of batt_low fanfare 
		$display("Start testing batt_low fanfare,(should override en_steer fanfare) at time: %0t!", $time);
		batt = 12'h7ff;
		repeat(500000) @(negedge clk);
		
		// By triggering too_fast, piezo should have the priority of too_fast fanfare 
		$display("Start testing too_fast fanfare,(should override batt_low fanfare) at time: %0t!", $time);
		//Trigger too_fast
		steerPot = 12'hfff;
		repeat(4095) #5000 rider_lean = rider_lean + 4'd9;
		repeat(500000) @(negedge clk);
		
		// Disable too_fast, trigger batt_low
		$display("Disable too_fast, should trigger batt_low fanfare at time: %0t!", $time);
		steerPot = 12'h7ff;
		repeat(4095) #5000 rider_lean = rider_lean - 4'd9;
		// rider_lean = 16'h0000;
		repeat(500000) @(negedge clk);

		// Disable batt_low, trigger normal en_steer fanfare
		$display("Disable batt_low, should trigger en_steer fanfare at time: %0t!", $time);
		batt = 12'h801;
		repeat(500000) @(negedge clk);

		// Disable en_steer fanfare, which means rider off
		$display("Disable en_steer, should not be playing any fanfare at time: %0t!", $time);
		ld_cell_lft = 0;
		ld_cell_rght = 0;
		repeat(500000) @(negedge clk);
	end
endtask : Check_piezo


// Test the Segway model's reponse when the segway changes directions
task automatic Check_steering(ref clk, ref reg [11:0]ld_cell_lft, ref reg [11:0]ld_cell_rght, ref send_cmd, ref reg signed[15:0] rider_lean, ref reg [7:0] cmd, ref cmd_sent, ref reg [11:0]steerPot);
	begin
		//Sum > 12'h240 && !diff_gt_1_4 => en_steer = 1 
		ld_cell_lft = 12'h200;
		ld_cell_rght = 12'h200;	
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h47), .cmd(cmd), .cmd_sent(cmd_sent));

		//Center
		$display("Centering steering pot at time: %0t!", $time);
		steerPot = 12'h7ff;
		repeat(800000) @(negedge clk);

		//Steer to the right
		$display("Steering to the right at time: %0t!", $time);
		steerPot = 12'hfff;
		repeat(800000) @(negedge clk);

		//Steer to the Left
		$display("Steering to the left at time: %0t!", $time);
		steerPot = 12'h000;
		repeat(800000) @(negedge clk);	

		//Steer back to the right
		$display("Steering back to the right at time: %0t!", $time);
		steerPot = 12'hfff;
		repeat(800000) @(negedge clk);

		//Steer to the left again
		$display("Steering to the left again at time: %0t!", $time);
		steerPot = 12'h000;
		repeat(800000) @(negedge clk);

		//Center
		$display("Centering steering pot at time: %0t!", $time);
		steerPot = 12'h7ff;	
		repeat(800000) @(negedge clk);
	end
endtask : Check_steering


// Test the Segway model's reponse when the segway changes directions and rider lean is reversed
task automatic Check_Rev_steering(ref clk, ref reg [11:0]ld_cell_lft, ref reg [11:0]ld_cell_rght, ref send_cmd, ref reg signed[15:0] rider_lean, ref reg [7:0] cmd, ref cmd_sent, ref reg [11:0]steerPot);
	begin
		//Sum > 12'h240 && !diff_gt_1_4 => en_steer = 1 
		ld_cell_lft = 12'h200;
		ld_cell_rght = 12'h200;	
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h47), .cmd(cmd), .cmd_sent(cmd_sent));
		rider_lean = 16'h0000 - 16'h0200;

		//Center
		$display("Centering steering pot at time: %0t!", $time);
		steerPot = 12'h7ff;
		repeat(800000) @(negedge clk);

		//Steer to the right
		$display("Steering to the right at time: %0t!", $time);
		steerPot = 12'hfff;
		repeat(800000) @(negedge clk);

		//Steer to the Left
		$display("Steering to the left at time: %0t!", $time);
		steerPot = 12'h000;
		repeat(800000) @(negedge clk);

		//Steer back to the right
		$display("Steering back to the right at time: %0t!", $time);	
		steerPot = 12'hfff;
		repeat(800000) @(negedge clk);

		//Steer to the left again
		$display("Steering to the left again at time: %0t!", $time);
		steerPot = 12'h000;
		repeat(800000) @(negedge clk);

		//Center
		$display("Centering steering pot at time: %0t!", $time);
		steerPot = 12'h7ff;	
		repeat(800000) @(negedge clk);
	end
endtask : Check_Rev_steering


// Test the auth block power up and power down sequences
task automatic Check_auth_block(ref clk, ref reg [11:0]ld_cell_lft, ref reg [11:0]ld_cell_rght, ref send_cmd, ref reg [7:0] cmd, ref cmd_sent, ref pwr_up);
	begin
		// Sum > 12'h240 && !diff_gt_1_4 => en_steer = 1 
		$display("Starting testing power up and rider on at time: %0t!", $time);
		ld_cell_lft = 12'h200;
		ld_cell_rght = 12'h200;	
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h47), .cmd(cmd), .cmd_sent(cmd_sent));
		repeat(500000)@(negedge clk);

		fork
			begin: timeout2;
				repeat(500000) @(negedge clk);
				$display("Error! pwr_up should be asserted!"); 
				$stop();
			end

			begin
				if(pwr_up == 1'b1) 
					disable timeout2;	
			end
		join

		// Now disconnect it to see if it powers down
		$display("Now disconnecting to test if it still powers up at time: %0t!", $time);
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h53), .cmd(cmd), .cmd_sent(cmd_sent));
		repeat(500000)@(negedge clk);
		fork
			begin
				repeat(500000) @(negedge clk);
				disable check_pwr_up;
			end

			begin: check_pwr_up
				if(pwr_up == 1'b0) begin
					$display("Segway should not be powered down!"); 
					$stop();
				end				
			end
		join

		// Now rider off, segway should power down
		$display("Now rider off to test power down at time: %0t!", $time);
		ld_cell_lft = 12'h000;
		ld_cell_rght = 12'h000;	
		repeat(500000)@(negedge clk);
		fork
			begin: timer
				repeat(500000) @(negedge clk);
				$display("Segway should be powered down!");
				$stop();
			end

			begin
				if (pwr_up == 1'b0)
				disable timer;				
			end
		join

		// Now send connect command again to see if it stays powered down
		$display("Now sending connect command again to test if it stays powered down at time: %0t!", $time);
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h47), .cmd(cmd), .cmd_sent(cmd_sent));
		repeat(500000) @(negedge clk);	
		fork
			begin: timer1
				repeat(500000) @(negedge clk);
				disable check_pwr_down;
			end

			begin: check_pwr_down
				if(pwr_up == 1'b1) begin
					$display("Segway should power down!"); 
					$stop();
				end				
			end
		join

		// Now reconnect it and rider on to see if it powers up
		$display("Now rider is on to test power up at time: %0t!", $time);
		ld_cell_lft = 12'h200;
		ld_cell_rght = 12'h200;	
		repeat(500000)@(negedge clk);
		fork
			begin: timer2;
				repeat(500000) @(negedge clk);
				if(pwr_up == 1'b0) begin
					$display("Segway should be powered up!"); 
					$stop();
				end
			end

			begin
				if(pwr_up == 1'b1) 
					disable timer2;	
			end
		join

		// Now rider off again, segway should power down
		$display("Now rider off again to test power down at time: %0t!", $time);
		ld_cell_lft = 12'h000;
		ld_cell_rght = 12'h000;	
		repeat(500000)@(negedge clk);
		fork
			begin
				repeat(500000) @(negedge clk);
				disable timer3;
			end

			begin: timer3
				if(pwr_up == 1'b1) begin
					$display("Segway should power down!"); 
					$stop();
				end				
			end
		join

		// Finally, send disconnect command again to make sure it stays powered down
		$display("Finally, sending disconnect command again to test power down at time: %0t!", $time);
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h53), .cmd(cmd), .cmd_sent(cmd_sent));
		repeat(500000) @(negedge clk);
		if(pwr_up === 1'b1) begin
				$display("Segway should power down!"); 
		end	
	end
endtask : Check_auth_block


// Test the function of steer enable state machine
task automatic Check_Enable_Steering(ref clk, ref reg [11:0]ld_cell_lft, ref reg [11:0]ld_cell_rght, ref send_cmd, ref reg [7:0] cmd, ref cmd_sent, ref en_steer, ref rider_off);
begin	
	//Sum > 12'h240 && !diff_gt_1_4 => en_steer = 1, diff_gt_1_4 < 0 => en_steer = 1 
	ld_cell_lft = 12'h200;
	ld_cell_rght = 12'h200;
	repeat(500000)@(negedge clk);	
	Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h47), .cmd(cmd), .cmd_sent(cmd_sent));

	$display("Rider is on and balanced, segway should be steer enabled at time: %0t!", $time);
	repeat(500000)@(negedge clk); 
	if(en_steer == 1'b0)begin
		$display("Segway should enable steering!");
		$stop();
	end

	// Now trigger diff_gt_1_4 by unbalancing the load cells
	$display("Now rider is on but unbalanced, segway should be disable steering at time: %0t!", $time);
	ld_cell_lft = 12'h400;
	ld_cell_rght = 12'h001;
	repeat(500000)@(negedge clk); 
	if(en_steer == 1'b1 || rider_off == 1'b1)begin
		$display("Segway should disable steering, rider is not balanced!");
		$stop();
	end

	// Now rebalance the load cells to enable steering again
	$display("Now rider is rebalanced, segway should enable steering at time: %0t!", $time);
	ld_cell_lft = 12'h200;
	ld_cell_rght = 12'h200;
	repeat(500000)@(negedge clk); 
	if(en_steer == 1'b0 || rider_off == 1'b1)begin
		$display("Rider is balanced, Segway should enable steering again!");
		$stop();
	end

	// Now the rider gets off, en_steer should be 0, rider_off should be 1
	$display("Now rider gets off, segwayshould disable steering at time: %0t!", $time);
	ld_cell_lft = 12'h020;
	ld_cell_rght = 12'h020;
	repeat(500000)@(negedge clk); 
	if(en_steer == 1'b1 || rider_off == 1'b0)begin
		$display("Rider is off, Segway should disable steering!");
		$stop();
	end

	// Now rider gets back on segway but unbalanced, en_steer should be 0, rider_off should be 0
	$display("Now rider gets back on but unbalanced, segway should disable steering at time: %0t!", $time);
	ld_cell_lft = 12'h400;
	ld_cell_rght = 12'h001;
	repeat(500000)@(negedge clk); 
	if(en_steer == 1'b1 || rider_off == 1'b1)begin
		$display("Rider is back on but unbalanced, Segway should disable steering!");
		$stop();
	end

	// Finally, the rider is balanced, en_steer should be 1, rider_off should be 0
	$display("Now rider is balanced, segway should enable steering at time: %0t!", $time);
	ld_cell_lft = 12'h200;
	ld_cell_rght = 12'h200;
	repeat(500000)@(negedge clk); 
	if(en_steer == 1'b0 || rider_off == 1'b1) begin
		$display("Rider is back on, Segway should enable steering!");
		$stop();
	end
end
endtask : Check_Enable_Steering


// Test for oevrcurrent events
task automatic Check_overcurrent (ref clk, ref rst_n, ref reg OVR_I_lft, ref reg OVR_I_rght, ref reg [11:0]ld_cell_lft, ref reg [11:0]ld_cell_rght,
 ref send_cmd, ref reg signed[15:0] rider_lean, ref reg [11:0] steerPot, ref reg [7:0] cmd, ref cmd_sent, ref pwr_up);
	begin
		//Sum > 12'h240 && !diff_gt_1_4 => en_steer = 1 
		ld_cell_lft = 12'h200;
		ld_cell_rght = 12'h200;	
		Check_SendCmd(.clk(clk), .send_cmd(send_cmd), .cmd2send(8'h47), .cmd(cmd), .cmd_sent(cmd_sent));
		
		//Wait for the ss_tmr to count 
		repeat(100000)@(negedge clk);

		fork
			begin: timeout2;
				repeat(500000) @(negedge clk);
				$display("Error waiting for pwr_up!"); 
				$stop();
			end

			begin
				if(pwr_up == 1'b1) 
					disable timeout2;	
			end
		join

		// Test when applying rider lean
        $display(">>> TEST 1: Forward tilt");
        repeat(500000) @(posedge clk);
        rider_lean = 16'sd2000; // slight forward tilt

        $display(">>> TEST 2: Strong forward tilt (faster PWM)");
        repeat(500000) @(posedge clk);
        rider_lean = 16'h1fff; // strong forward tilt

        $display(">>> TEST 3: Reverse tilt");
        repeat(500000) @(posedge clk);
        rider_lean = -16'sd2000; // slight reverse tilt

        $display(">>> TEST 4: Steering LEFT");
        steerPot = 12'h200;
        repeat(500000) @(posedge clk);
        rider_lean = -16'sd1000; // slight reverse tilt

        $display(">>> TEST 5: Steering RIGHT");
        steerPot = 12'hE00;
        repeat(500000) @(posedge clk);
        rider_lean = 16'sd1000; // slight forward tilt

        $display(">>> TEST 6: Overcurrent Shutdown test for LEFT motor");
		repeat(500000) @(posedge clk);
        inject_overcurrent_left (45, .clk(clk), .OVR_I_lft(OVR_I_lft));
		repeat(500000) @(posedge clk);

		$display(">>> TEST 7: Overcurrent Shutdown test for RIGHT motor");
		rst_n = 0;
		repeat(10) @(posedge clk);
		rst_n = 1;
		repeat(500000) @(posedge clk);
		inject_overcurrent_right (45, .clk(clk), .OVR_I_rght(OVR_I_rght));
		repeat(500000) @(posedge clk);
	end
endtask : Check_overcurrent


 // Inject overcurrent pulses
    task automatic inject_overcurrent_left (int pulses, ref clk, ref reg OVR_I_lft);
        repeat(pulses) begin
            OVR_I_lft = 1;
            repeat(2000) @(posedge clk);
            OVR_I_lft = 0;
            repeat(2000) @(posedge clk);
        end
	endtask : inject_overcurrent_left

	task automatic inject_overcurrent_right (int pulses, ref clk, ref reg OVR_I_rght);
		repeat(pulses) begin
			OVR_I_rght = 1;
			repeat(2000) @(posedge clk);
			OVR_I_rght = 0;
			repeat(2000) @(posedge clk);
		end
	endtask : inject_overcurrent_right

endpackage
