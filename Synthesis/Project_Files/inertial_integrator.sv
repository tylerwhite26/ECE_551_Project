// Teammates: Paras, Tyler, Raymond, Parker

module inertial_integrator (
input logic clk, 
input logic rst_n,
input logic vld, // High for a single clock cycle when new inertial readings are valid.
input logic signed [15:0] ptch_rt,// 16-bit signed raw pitch rate from inertial sensor
input logic signed [15:0] AZ, // Will be used for sensor fusion (acceleration in Z direction)
output logic  signed [15:0] ptch // Fully compensated and “fused” 16-bit signed pitch.
);
    localparam PTCH_RT_OFFSET = 16'h0050;
    localparam AZ_OFFSET = 16'h00A0;

    wire signed [15:0] ptch_rt_comp = ptch_rt - PTCH_RT_OFFSET;
    wire signed [15:0] AZ_comp = AZ - AZ_OFFSET;

    wire signed [25:0] ptch_acc_product = AZ_comp * $signed(327); // 327 is fudge factor
    wire signed [15:0] ptch_acc = {{3{ptch_acc_product[25]}}, ptch_acc_product[25:13]}; // pitch angle calculated
    wire signed [26:0] fusion_ptch_offset = ptch_acc > ptch_rt_comp ? 27'd1024 : ptch_acc < ptch_rt_comp ? -27'd1024 : 27'd0;

    reg [26:0] ptch_int;
    always_ff @ (posedge clk or negedge rst_n)
        if (!rst_n)
            ptch_int <= 27'h0000000;
        else if (vld)
            ptch_int <= ptch_int - {{11{ptch_rt_comp[15]}},ptch_rt_comp} + fusion_ptch_offset;

    assign ptch = ptch_int[26:11];


endmodule