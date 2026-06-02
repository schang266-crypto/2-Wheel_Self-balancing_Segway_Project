module auth_SM(
    input clk,              // 50MHz system clock
    input rst_n,            // Active low reset
    input [7:0] rx_data,    // Received UART data
    input rx_rdy,           // RX data ready signal
    input rider_off,        // Indicates rider is off the platform
    
    output reg pwr_up,      // Power enable signal to balance controller
    output reg clr_rx_rdy   // Clear RX ready flag
);

    // State encoding
    typedef enum logic [1:0] {
        IDLE        = 2'b00,
        CONNECTED   = 2'b01,
        DISCONNECTED= 2'b10
    } state_t;
    
    state_t current_state, next_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    // State Transition Logic
    always_comb begin
        next_state = current_state;
        pwr_up = 1'b0; // Power off only if command S received & rider off
        clr_rx_rdy = 1'b0;

        case (current_state)
            IDLE: begin
                // Wait for 'G' command to enable power
                if (rx_rdy && rx_data == 8'h47) begin  // 'G'
                    next_state = CONNECTED;
                    clr_rx_rdy = 1'b1;
                end
            end
            
            CONNECTED: begin
                pwr_up = 1'b1;
                if (rx_rdy && rx_data == 8'h53) begin // Check for 'S' disconnect command
                    clr_rx_rdy = 1'b1;
                    next_state = DISCONNECTED;
                end
            end
            
            DISCONNECTED: begin
                pwr_up = 1'b1; // Power stays on until rider gets off
                if (rider_off)
                    next_state = IDLE;
                else if (rx_rdy && rx_data == 8'h47) begin  // Reconnected
                    next_state = CONNECTED;
                    clr_rx_rdy = 1'b1;
                end
            end

            default:
                next_state = IDLE;
        endcase
    end

endmodule