module inertial_integrator(
    input logic clk,
    input logic rst_n,
    input logic vld,
    input logic signed [15:0] ptch_rt,
    input logic [15:0] AZ,
    output logic signed [15:0] ptch
);


logic signed [26:0] ptch_int;
logic signed [26:0] fusion_ptch_offset;
logic signed [15:0] ptch_rt_comp;
logic signed [15:0] AZ_comp;
logic signed [15:0] ptch_acc;
logic signed [25:0] ptch_acc_product;

assign AZ_comp = AZ - $signed(16'h00A0);
assign ptch_rt_comp = ptch_rt - $signed(16'h0050);


// Shift adder
assign ptch_acc_product = AZ_comp * $signed(327); // 327 is fudge factor
assign ptch_acc = {{3{ptch_acc_product[25]}}, ptch_acc_product[25:13]};

assign fusion_ptch_offset = (ptch_acc > ptch) ? 27'sd1024 : -27'sd1024;



always_ff @(posedge clk, negedge rst_n) begin 
    if (!rst_n) begin
        ptch_int <= 0;
    end else if (vld) begin
        ptch_int <= ptch_int - {{11{ptch_rt_comp[15]}}, ptch_rt_comp} + fusion_ptch_offset;
    end
end

assign ptch = ptch_int[26:11];
endmodule