module UART_rx (
    input clk,
    input rst_n,
    input RX,
    input clr_rdy,
    output [7:0] rx_data,
    output logic rdy
);

    typedef enum {RESET, RECEIVE} state_t;

    // internal signals
    logic [8:0] rx_shift_reg;
    logic [3:0] bit_cnt;
    logic set_rdy;
    logic start;
    logic shift;
    logic rxing;
    logic [12:0] baud_cnt;
    state_t state;
    state_t next_state;
    logic s_m1, s_m2, rx_meta_bit;

    // continuous output assignment
    assign rx_data = rx_shift_reg[7:0];

    // shift assignment logic, assign shift to 1 when the counter reaches 5208
    assign shift = baud_cnt[12] & baud_cnt[10] & baud_cnt[6] & baud_cnt[4] & baud_cnt[3];

    // meta stability protection
    always @ (posedge clk) begin
        s_m1 <= RX;
        rx_meta_bit <= s_m1;
    end

    // shift register
    always_ff @ (posedge clk) begin
        if (shift) begin
            rx_shift_reg <= {rx_meta_bit, rx_shift_reg[8:1]};
        end
    end

    // bit counter, reset only on start
    always_ff @ (posedge clk) begin
        if (start) // load command given, reset the counter
            bit_cnt <= 4'h0;
        else if (shift) // shift command, increment count
            bit_cnt <= bit_cnt + 1;
    end

    // TODO
    // rx_done flop
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            rdy <= 1'b0;
        else
            rdy <= set_rdy & ~start & ~clr_rdy; // hold until next recieve
    end

    // baud counter, incrementing shift every time the cnt fills up
    always_ff @ (posedge clk) begin
        if (start | shift) // reset every time data shifts
            baud_cnt <= 13'h0000;
        else if (rxing) // hold high to keep recieving
            baud_cnt <= baud_cnt + 1;
    end

    // state transition
    always @ (posedge clk) begin
        state <= next_state;
    end
    initial begin
        state = RESET;
        next_state = RESET;
    end

    // state machine
    always @(*) begin
        case (state)
            RESET : begin
                if (rx_meta_bit) begin
                    start = 0;
                    rxing = 0;
                    next_state = RESET;
                end else begin
                    start = 1;
                    rxing = 1;
                    set_rdy = 0;
                    next_state = RECEIVE;
                end
            end
            RECEIVE : begin
                if (bit_cnt < 10) begin
                    start = 0;
                    rxing = 1;
                    next_state = RECEIVE;
                end else begin
                    start = 0;
                    rxing = 0;
                    set_rdy = 1;
                    next_state = RESET;
                end
            end
        endcase
    end




endmodule