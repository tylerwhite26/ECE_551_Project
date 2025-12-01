module PWM11(
    input clk,
    input rst_n,
    input [10:0] duty,
    output logic PWM1,
    output logic PWM2,
    output PWM_synch,
    output ovr_I_blank
);

    localparam NONOVERLAP = 11'h040;
    localparam OVER_I_ADD = 11'd128;

    // signal declarations
    logic [10:0] cnt;
    logic over_i_lower;
    logic over_i_upper;

    // cnt incrementation
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            cnt = 11'h000;
        else
            cnt = cnt + 1;
    end

    // PWM_synch assignment
    assign PWM_synch = ~|cnt;

    // PWM1 assignment (Lower portion of cycle)
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            PWM1 = 1'b0;
        else if (cnt >= duty)
            PWM1 = 1'b0;
        else if (cnt >= NONOVERLAP)
            PWM1 = 1'b1;
    end

    // PWM2 assignment (Upper portion of cycle)
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            PWM2 = 1'b0;
        else if (&cnt)
            PWM2 = 1'b0;
        else if (cnt>= (duty + NONOVERLAP))
            PWM2 = 1'b1;
    end

    // Overcurrent blankout
    assign over_i_lower = (cnt > NONOVERLAP) && (cnt < (NONOVERLAP + OVER_I_ADD));
    assign over_i_upper = (cnt > (NONOVERLAP + duty)) && (cnt < (NONOVERLAP + duty + OVER_I_ADD));
    assign ovr_I_blank = over_i_lower | over_i_upper;

endmodule