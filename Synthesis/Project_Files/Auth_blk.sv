module Auth_blk (
    input clk,
    input rst_n,
    input RX,
    input rider_off,
    output logic pwr_up
);

    // internal signals
    logic [7:0] rx_data;
    logic rdy, clr_rdy, pow_rec, s_rec, pow_down;

    // internal uart_rx module
    UART_rx rx_mod (.clk(clk), .rst_n(rst_n), .RX(RX), .rdy(rdy), .clr_rdy(clr_rdy), .rx_data(rx_data));

    // internal states
    typedef enum {INIT, RDY, G, S} state_t;
    state_t state;
    state_t next_state;

    // state transitions
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= INIT;
        else
            state <= next_state;
    end

    // state machine
    always_comb begin
        pow_rec = 0;
        s_rec = 0;
        clr_rdy = 0;
        next_state = state;
        case (state)
            INIT : begin
                if (rdy)
                    next_state = RDY;
            end
            RDY : begin
                if (rx_data == 8'h47) begin // case of recieving a 'G'
                    clr_rdy = 1;
                    next_state = G;
                end else if (rx_data == 8'h53) begin // case of recieving an 'S'
                    clr_rdy = 1;
                    next_state = S;
                end else begin
                    clr_rdy = 1;
                    next_state = INIT;
                end
            end
            G : begin
                pow_rec = 1;
                next_state = INIT;
            end
            S : begin
                s_rec = 1;
                next_state = INIT;
            end
        endcase
    end

    // latch power down so that it waits for rider_off to affect pwr_up
    always @ (*) begin
        if (!rst_n)
            pow_down <= 1'b0;
        else if (s_rec)
            pow_down <= 1'b1;
        else if (pow_rec)
            pow_down <= 1'b0;
    end

    // flop pwr_up based on state machine assertions
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            pwr_up <= 1'b0;
        else if (pow_down & rider_off) // power down priority
            pwr_up <= 1'b0;
        else if (pow_rec)
            pwr_up <= 1'b1;
    end

endmodule