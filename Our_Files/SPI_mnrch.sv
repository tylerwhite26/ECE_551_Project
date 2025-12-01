module SPI_mnrch(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        wrt,
    input  logic [15:0] wt_data,
    input  logic        MISO,
    output logic        done,
    output logic [15:0] rd_data,
    output logic        SS_n,
    output logic        SCLK,
    output logic        MOSI
);
// MSB is SCLK. Divides clk by 16 to create SCLK
logic shft_im, smpl, init, set_done, ld_SCLK, shft, smpl_im;
logic [3:0] SCLK_div;
always_ff @(posedge clk) begin
    if (ld_SCLK) 
        SCLK_div <= 4'b1011;
    else
        SCLK_div <= SCLK_div + 1;
end

assign SCLK = SCLK_div[3];
// Our shifter imminent signals for our SM
assign smpl_im = (SCLK_div == 4'b0111);
assign shft_im = (SCLK_div == 4'b1111);

logic MISO_smpl;
always_ff @(posedge clk) begin
    if (smpl)
        MISO_smpl <= MISO;
end

// Our shift register that shifts in MISO from NEMO and shifts out MOSI
logic [15:0] shft_reg;
always_ff @(posedge clk) begin
    if (init) begin
        shft_reg <= wt_data;
    end
    else if (shft) begin
        shft_reg <= {shft_reg[14:0], MISO_smpl};
    end
end
assign MOSI = shft_reg[15];

assign rd_data = shft_reg;

// Checks how many shifts we have done, when == 15, go to the backporch state
logic [3:0] bit_cntr;
logic done15;
always_ff @(posedge clk) begin
    if (init)
        bit_cntr <= 4'b0000;
    else if (shft)
        bit_cntr <= bit_cntr + 1;
    else
        bit_cntr <= bit_cntr;
end

assign done15 = &bit_cntr; // When 4'b1111, assert done15 which transitions to the final state

// Signals that we have completed and that rd_data contains the valid data if we are reading or that write has completed.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        done <= 1'b0;
    else if (init)
        done <= 1'b0;
    else if (set_done)
        done <= 1'b1;
end

// This is our SS_n control logic, which indicates that we are done and is where the last shift occurs but where SCLK stays high.
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        SS_n <= 1'b1;
    else if (init)
        SS_n <= 1'b0;
    else if (set_done)
        SS_n <= 1'b1;
end

// State machine to control the SPI transaction
typedef enum logic [1:0] {IDLE, FRONTPORCH, CONTINUE, BACKPORCH} state_t;
state_t state, next_state;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= IDLE;
    else
        state <= next_state;
end

always_comb begin
    // defaults
    next_state = state;
    init = 1'b0;
    set_done = 1'b0;
    ld_SCLK = 1'b0;
    shft = 1'b0;
    smpl = 1'b0;
    next_state = state;
    case (state)
        IDLE: begin
            if (wrt) begin
                next_state = FRONTPORCH;
                init = 1'b1;
                ld_SCLK = 1'b1;
            end
        end
        FRONTPORCH: begin
            if (smpl_im) begin
                // Front porch where SCLK is high, but we still sample
                smpl = 1'b1;
                next_state = CONTINUE;
            end          
        end
        CONTINUE: begin

            if (shft_im) begin
                shft = 1'b1;
            end
            if (smpl_im) begin
                smpl = 1'b1;
            end
            if (done15) begin
                next_state = BACKPORCH;
            end
            end
        BACKPORCH: begin
            // First, we sample then shift after being in the backporch
            if (smpl_im) begin
                smpl = 1'b1;
            end
            if (shft_im) begin
                shft = 1'b1;
                set_done = 1'b1;
                // Assert ld_SCLK so that it stays high!
                ld_SCLK = 1'b1;
                next_state = IDLE;
            end
        end
    endcase
end
endmodule