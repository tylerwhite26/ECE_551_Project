// Teammates: Paras, Tyler, Raymond, Parker

module inertial_integrator (
input logic clk, 
input logic rst_n,
input logic vld, // High for a single clock cycle when new inertial readings are valid.
input logic signed [15:0] ptch_rt,// 16-bit signed raw pitch rate from inertial sensor
input logic signed [15:0] AZ, // Will be used for sensor fusion (acceleration in Z direction)
output logic  signed [15:0] ptch // Fully compensated and “fused” 16-bit signed pitch.
);


logic signed [26:0] ptch_int; // Pitch integrating accumulator. This is the register ptch_rt is summed into
logic signed [15:0] ptch_rt_comp; // Compensated pitch rate after removing offset
logic signed [15:0] AZ_comp;// Compensated Z acceleration after removing offset
logic signed [15:0] ptch_acc; // Pitch acceleration derived from Z acceleration
logic signed [25:0] ptch_acc_product; // Intermediate product for pitch acceleration calculation
logic signed [26:0] fusion_ptch_offset; // Offset to be added to pitch integrator for sensor fusion
localparam PTCH_RT_OFFSET = 16'h0050;
localparam AZ_OFFSET = 16'h00A0;

 // Compensate pitch rate and Z acceleration by removing their respective offsets
assign ptch_rt_comp = ptch_rt - PTCH_RT_OFFSET;
assign AZ_comp = AZ - AZ_OFFSET;

// Calculate pitch acceleration from Z acceleration
assign ptch_acc_product = AZ_comp * $signed(327);
assign ptch_acc = {{3{ptch_acc_product[25]}},ptch_acc_product[25:13]};

// Determine fusion offset based on pitch acceleration direction
assign fusion_ptch_offset = (ptch_acc > ptch) ? 12'h400 : ((ptch_acc < ptch) ? -12'h400 : 27'h0); 

// Integrate pitch rate and apply sensor fusion when valid signal is high
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ptch_int <= 0;
        ptch <= 0;
    end else if (vld) begin
        ptch_int <= ptch_int - {{11{ptch_rt_comp[15]}}, ptch_rt_comp} + fusion_ptch_offset;
        ptch <= ptch_int[26:11];
    end
end



endmodule