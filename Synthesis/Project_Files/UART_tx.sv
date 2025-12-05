module UART_tx (
    input clk,
    input rst_n,
    input trmt, // asserted for one clock cycle to initiate a transmission
    input [7:0] tx_data,
    output TX,
    output logic tx_done
);

    typedef enum {RESET, TRANSMIT} state_t;

    // internal signals
    logic [8:0] tx_shift_reg;
    logic [3:0] bit_cnt;
    logic set_done;
    logic load;
    logic shift;
    logic txing;
    logic [12:0] baud_cnt;
    state_t state;
    state_t next_state;

    // continuous output assignment
    assign TX = tx_shift_reg[0];

    // shift assignment logic, assign shift to 1 when the counter reaches 5208
    assign shift = baud_cnt[12] & baud_cnt[10] & baud_cnt[6] & baud_cnt[4] & baud_cnt[3];

    // shift register
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift_reg <= 9'h000;
        end else if (load) begin // load command
            tx_shift_reg <= {tx_data, 1'b0};
        end else if (shift) begin // no load, but shift
            tx_shift_reg <= {1'b1, tx_shift_reg[8:1]};
        end
    end

    // bit counter, reset only on load
    always_ff @ (posedge clk) begin
        if (load) // load command given, reset the counter
            bit_cnt <= 4'h0;
        else if (shift) // shift command, increment count
            bit_cnt <= bit_cnt + 1;
    end

    // tx_done flop
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            tx_done <= 1'b0;
        else
            tx_done <= set_done & ~load; // hold until next transmission
    end

    // baud counter, incrementing shift every time the cnt fills up
    always_ff @ (posedge clk) begin
        if (load | shift) // reset every time data shifts
            baud_cnt <= 13'h0000;
        else if (txing) // hold high to keep transmitting
            baud_cnt <= baud_cnt + 1;
    end

    // state transition
    always @ (posedge clk) begin
        state <= next_state;
    end

    // state machine
    always @(*) begin
        case (state)
            RESET : begin
                if (trmt) begin
                    next_state = TRANSMIT;
                    load = 1;
                    set_done = 0;
                end else begin
                    next_state = RESET;
                end
            end
            TRANSMIT : begin
                load = 0;
                if (bit_cnt < 4'd10) begin
                    next_state = TRANSMIT;
                    txing = 1;
                end else begin
                    next_state = RESET;
                    txing = 0;
                    set_done = 1;
                end
            end
        endcase
    end




endmodule