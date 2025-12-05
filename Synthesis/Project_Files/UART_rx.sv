module UART_rx (
    input logic clk,
    input logic rst_n,
    input logic RX,
    input logic clr_rdy,
    output logic rdy, // Asserted high when byte received. Stays high until startSignal but of next byte starts or clr_rdy asserted
    output logic [7:0]rx_data
);

logic shift, startSignal, receiving, set_rdy;


// Double flop RX For meta-stability
logic RX_sync_0, RX_sync_1;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        RX_sync_0 <= 1'b1;
        RX_sync_1 <= 1'b1;
    end else begin
        RX_sync_0 <= RX;
        RX_sync_1 <= RX_sync_0;
    end
end

// Baud Counter
logic [12:0] baud_cnt;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        baud_cnt <= 13'h0000;
    end else if (startSignal || shift) begin
        baud_cnt <= (startSignal) ? 13'd2603 : 13'd5207;
    end else if (receiving) begin
        baud_cnt <= baud_cnt - 1;
    end
end
assign shift = (baud_cnt == 13'd0);


// Data
logic [8:0] rx_shift_reg;
always_ff @(posedge clk) begin
    if (shift) begin
        rx_shift_reg <= {RX_sync_1, rx_shift_reg[8:1]}; // Shift in new bit
    end
end
assign rx_data = rx_shift_reg[7:0]; // Only read when rdy is high in testbench

// Bit Counter
logic [3:0] bit_cnt;
always_ff @(posedge clk) begin
    if (startSignal) begin
        bit_cnt <= 0;
    end else if (shift) begin
        bit_cnt <= bit_cnt + 1;
    end
end

// State Machine
typedef enum logic [0:0] {IDLE, RECEIVE} state_t;
state_t state, nxt_state;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
    end else begin
        state <= nxt_state;
    end
end
always_comb begin
    // Default inputs/outputs
    startSignal = 0;
    receiving = 0;
    set_rdy = 0;
case(state)
    RECEIVE: if (bit_cnt < 10) begin
        receiving = 1;
        nxt_state = RECEIVE;
    end else begin
        nxt_state = IDLE;
        set_rdy = 1;
    end
    default: if (!RX_sync_1) begin // This is our IDLE State // Default State
        startSignal = 1;
        nxt_state = RECEIVE;
    end else begin
        nxt_state = IDLE;
    end
endcase
end


always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rdy <= 0;
    end else if (clr_rdy || startSignal) begin
        rdy <= 0;
    end else if (set_rdy) begin
        rdy <= 1;
    end
end
endmodule
