module Auth_blk(
    input RX,
    input clk,
    input rst_n,
    input rider_off,
    output logic pwr_up
);
logic [7:0] rx_data;
logic rx_rdy, clr_rx_rdy;
// Instantiate the UART Receiver
UART_rx uart_receiver (
    .clk(clk),          // Connect to system clock
    .rst_n(rst_n),       // Connect to system reset (active low)
    .RX(rx),        // UART RX input
    .clr_rdy(clr_rx_rdy), // Clear ready signal when rider is off
    .rdy(rx_rdy),   // Power up signal when byte received
    .rx_data(rx_data)      // Received data byte that goes into our state machine
);

// PWR_UP is asserted when we receive the command 0x47 ('G')
// When we disconnect due to app disconnection or range of module, it sends a 0x53 ('S').
// The segway then shuts down (if weight of platform no longer exceeds MIN_RIDER_WEIGHT), indicated by rider_off


typedef enum logic [1:0] { WAIT_ON_CMD, SEGWAY_ON, SEGWAY_OFF} state_t;
state_t current_state, next_state;
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        current_state <= WAIT_ON_CMD;
    end else begin
        current_state <= next_state;
    end
end

always_comb begin
    next_state = current_state;
    clr_rx_rdy = 1'b0;
    pwr_up = 0;
    case (current_state)
        WAIT_ON_CMD: begin
            if (rx_rdy) begin
                clr_rx_rdy = 1'b1;
                // If we receive 'G' (0x47), move to SEGWAY_ON, else stay in WAIT_ON_CMD (No power up)
                if (rx_data == 8'h47) begin
                    next_state <= SEGWAY_ON;
                    pwr_up = 1'b1;
                end
            end
        end
        SEGWAY_ON: begin
            pwr_up = 1'b1;
            // Only leave this state when we receive 'S' (0x53)
            if (rx_rdy) begin
                clr_rx_rdy = 1'b1;
                if (rx_data == 8'h53) begin
                    next_state <= SEGWAY_OFF;
                end
            end
        end
        SEGWAY_OFF: begin
            pwr_up = 1'b1;
            // PWR_UP deasserted when rider_off signal is high and last reception was 0x53 ('S')
            if (rider_off) begin
                next_state <= WAIT_ON_CMD;
                clr_rx_rdy = 1'b1;
            end
            else if (rx_rdy) begin
                clr_rx_rdy = 1'b1;
                if (rx_data == 8'h47) begin
                    next_state <= SEGWAY_ON;
                end
            end
        end
        // If an error occurs and somehow enter this state, go to the same state as reset
        default: begin
            next_state = WAIT_ON_CMD;
        end
    endcase
end


endmodule
