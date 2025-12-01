module A2D_intf (
    input wire clk,
    input wire rst_n,
    input wire nxt,
    output logic [11:0] lft_ld,
    output logic [11:0] rght_ld,
    output logic [11:0] steer_pot,
    output logic [11:0] batt,
    // SPI Interface
    input wire MISO,
    output logic MOSI,
    output logic SS_n,
    output logic SCLK
);

// SPI Interface for reading external A2D converter
logic [15:0] SPI_wt_data, SPI_rd_data;
logic SPI_wrt, SPI_done;
SPI_mnrch u_SPI (.clk(clk), .rst_n(rst_n), .wrt(SPI_wrt), .wt_data(SPI_wt_data), 
        .MISO(MISO), .MOSI(MOSI), .SS_n(SS_n), .SCLK(SCLK),
        .rd_data(SPI_rd_data), .done(SPI_done));

// assignment of outputs at clock edge with associated enables
logic [1:0] update_readout_enable; // 00 lft, 01 rght, 10 steer, 11 batt
logic update; // sm output for updating flops
always_ff @ (posedge clk or negedge rst_n)
    if (!rst_n) begin
        lft_ld <= 11'd0;
        rght_ld <= 11'd0;
        steer_pot <= 11'd0;
        batt <= 11'd0;
    end else if (update_readout_enable == 2'b00 && update)
        lft_ld <= SPI_rd_data[11:0];
    else if (update_readout_enable == 2'b01 && update)
        rght_ld <= SPI_rd_data[11:0];
    else if (update_readout_enable == 2'b10 && update)
        steer_pot <= SPI_rd_data[11:0];
    else if (update_readout_enable == 2'b11 && update)
        batt <= SPI_rd_data[11:0];

// round robin counter for changing which device is read
always_ff @ (posedge clk or negedge rst_n)
    if (!rst_n)
        update_readout_enable <= 2'b00;
    else if (update)
        update_readout_enable <= update_readout_enable + 1;

// assignment of wt_data based on update_readout_enable
// Both transactions will use the same data since the second write_data and first read_data are don't care
// Format of packet to A2D is {2'h0, channel[2:0], 11'h000}
assign SPI_wt_data =    (update_readout_enable == 2'b00) ?  {2'd0, 3'd0, 11'd0} : // lft on channel 0
                        (update_readout_enable == 2'b01) ?  {2'd0, 3'd4, 11'd0} : // rght on channel 4
                        (update_readout_enable == 2'b10) ?  {2'd0, 3'd5, 11'd0} : // steer_pot on channel 5
                        (update_readout_enable == 2'b11) ?  {2'd0, 3'd6, 11'd0} : // batt on channel 6
                                                            16'd0; // default to zeros, never reached


typedef enum logic [2:0] {IDLE, CONVERT, INCREMENT, BUFF} state_t;
state_t state, next_state;
always_ff @(posedge clk, negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
    end else begin
        state <= next_state;
    end
end

always_comb begin
    // Inputs: nxt, clk, rst_n, done (from SPI_mnrch)
    // Outputs: update, wrt
    next_state = state;
    update = 0;
    SPI_wrt = 0;
    case (state)
        IDLE: begin
            if (nxt) begin
                SPI_wrt = 1;
                next_state = CONVERT;
            end
        end
        // Then, we kick off two SPI transactions via SPI_mnrch. First determines which channel to convert, second reads that channel.
        // Then, the round-robin counter is incremented and on a NXT request it will convert the next channel in the sequence.
        CONVERT: begin // Determines which channel to convert
            if (SPI_done) begin
                next_state = BUFF;
            end
        end 
        BUFF : begin // running into timing issue, so wait an extra clock cycle
            SPI_wrt = 1; // Kick off read transaction
            next_state = INCREMENT;
        end
        INCREMENT: begin // Update round-robin counter to next channel and wait until NXT is asserted again.
            if (SPI_done) begin
                update = 1;
                next_state = IDLE;
            end
        end 


        
        
    endcase
            
end

endmodule

