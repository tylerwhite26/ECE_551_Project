module balance_cntrl (
    input clk,
    input rst_n,
    input vld,
    input [15:0] ptch,
    input [15:0] ptch_rt,
    input pwr_up,
    input rider_off,
    input [11:0] steer_pot,
    input en_steer,
    output [11:0] lft_spd,
    output [11:0] rght_spd,
    output logic too_fast
);
    parameter FAST_SIM = 1;

    // internal signals
    logic [11:0] PID_cntrl;
    logic [7:0] ss_tmr;

    // module connections
    PID #(.FAST_SIM(FAST_SIM)) pid_unit (.clk(clk), .rst_n(rst_n), .vld(vld), .ptch(ptch), .ptch_rt(ptch_rt), .pwr_up(pwr_up), .rider_off(rider_off),
            .PID_cntrl(PID_cntrl), .ss_tmr(ss_tmr));
    SegwayMath math_unit(.clk(clk), .PID_cntrl(PID_cntrl), .ss_tmr(ss_tmr), .steer_pot(steer_pot), .en_steer(en_steer),
            .pwr_up(pwr_up), .lft_spd(lft_spd), .rght_spd(rght_spd), .too_fast(too_fast));

endmodule


module SegwayMath(
    input logic clk,
    input logic signed [11:0] PID_cntrl,
    input logic [7:0] ss_tmr,
    input logic [11:0] steer_pot,
    input en_steer,
    input pwr_up,
    output logic signed [11:0] lft_spd,
    output logic signed [11:0] rght_spd,
    output logic too_fast
);

    // Pipeline registers for signals from PID module
    logic signed [11:0] PID_cntrl_piped;
    logic [7:0] ss_tmr_piped;
    always_ff @ (posedge clk) begin
        PID_cntrl_piped <= PID_cntrl;
        ss_tmr_piped <= ss_tmr;
    end

    // For pipelining outputs
    logic signed [11:0] lft_spd_internal;
    logic signed [11:0] rght_spd_internal;
    logic too_fast_internal;
    always_ff @ (posedge clk) begin
        lft_spd <= lft_spd_internal;
        rght_spd <= rght_spd_internal;
        too_fast <= too_fast_internal;
    end
    
    // Local Parameters
    localparam [12:0] MIN_DUTY = 13'h0A8;
    localparam [6:0] LOW_TORQUE_BAND = 7'h2A;
    localparam [3:0] GAIN_MULT = 4'h4;
    localparam [11:0] UPPER_SPEED = 12'd1536;

    // Signal declarations
    logic signed [19:0] PID_mult;
    logic signed [11:0] PID_ss;
    logic [11:0] steer_pot_lim;
    logic signed [12:0] steer_pot_scale;
    logic signed [12:0] lft_torque;
    logic signed [12:0] rght_torque;
    logic signed [12:0] lft_torque_comp;
    logic signed [12:0] lft_shaped_always;
    logic signed [12:0] lft_shaped;
    logic signed [12:0] rght_torque_comp;
    logic signed [12:0] rght_shaped_always;
    logic signed [12:0] rght_shaped;
    logic signed [12:0] lft_torque_abs;
    logic signed [12:0] rght_torque_abs;

    // pipe the multiply
    logic signed [19:0] PID_mult_i;
    always_ff @ (posedge clk)
        PID_mult <= PID_mult_i;

    // Scale PID_cntrl_piped to ensure a smooth start
    assign PID_mult_i = PID_cntrl_piped * $signed({1'h0, ss_tmr_piped});
    assign PID_ss = PID_mult[19:8];

    // pipe this multiply as well
    logic signed [12:0] steer_pot_scale_i;
    always_ff @ (posedge clk)
        steer_pot_scale <= steer_pot_scale_i;

    // Limit steer pot signal and scale by 3/16
    assign steer_pot_lim =  steer_pot < 12'h200 ? 12'h200 : 
                            steer_pot > 12'hE00 ? 12'hE00 :
                            steer_pot;
    assign steer_pot_scale_i = $signed(3) * $signed(steer_pot_lim - 12'h800) / $signed(16);

    // Torques should be equal if steer not enabled and scaled steer values if steer is enabled
    assign lft_torque = en_steer ? $signed({PID_ss[11], PID_ss}) + steer_pot_scale : $signed({PID_ss[11], PID_ss});
    assign rght_torque = en_steer ? $signed({PID_ss[11], PID_ss}) - steer_pot_scale : $signed({PID_ss[11], PID_ss});

    // pipe the shaped values so propogation is slower
    logic signed [12:0] lft_shaped_i;
    logic signed [12:0] rght_shaped_i;
    always_ff @ (posedge clk) begin
        lft_shaped <= lft_shaped_i;
        rght_shaped <= rght_shaped_i;
    end

    // Value of the torque must exceed |min_duty| for the motors to deliver enough power to drive
    assign lft_torque_comp = lft_torque[12] ? lft_torque - $signed(MIN_DUTY) : lft_torque + $signed(MIN_DUTY);
    // Ensure the torque exceeds torque band, adding gain if necessary
    assign lft_torque_abs = lft_torque[12] ? -lft_torque : lft_torque; 
    assign lft_shaped_always = lft_torque_abs > LOW_TORQUE_BAND ? lft_torque_comp : lft_torque * $signed(GAIN_MULT);
    // Account for power enable, if not enabled should be 0
    assign lft_shaped_i = pwr_up ? lft_shaped_always : 13'h0000;

    // repeat last three steps for the right side
    assign rght_torque_comp = rght_torque[12] ? rght_torque - $signed(MIN_DUTY) : rght_torque + $signed(MIN_DUTY);
    assign rght_torque_abs = rght_torque[12] ? -rght_torque : rght_torque; 
    assign rght_shaped_always = rght_torque_abs > LOW_TORQUE_BAND ? rght_torque_comp : rght_torque * $signed(GAIN_MULT);
    assign rght_shaped_i = pwr_up ? rght_shaped_always : 13'h0000;

    // Sature the shaped torques to speeds and ensure they are not too fast
    assign lft_spd_internal =    lft_shaped[12] & ~lft_shaped[11] ? 12'h800 :
                        ~lft_shaped[12] & lft_shaped[11] ? 12'h7FF :
                        lft_shaped[11:0];
    assign rght_spd_internal =   rght_shaped[12] & ~rght_shaped[11] ? 12'h800 :
                        ~rght_shaped[12] & rght_shaped[11] ? 12'h7FF :
                        rght_shaped[11:0];
    assign too_fast_internal = rght_spd_internal > $signed(UPPER_SPEED) | lft_spd_internal > $signed(UPPER_SPEED);

endmodule


module PID (
    input rst_n,
    input clk,
    input vld,
    input pwr_up,
    input rider_off,
    input logic signed [15:0] ptch,
    input logic signed [15:0] ptch_rt,
    output logic signed [11:0] PID_cntrl,
    output logic [7:0] ss_tmr
);

    parameter FAST_SIM = 1;

    localparam signed P_COEFF = 5'h09;

    logic signed [9:0] ptch_err_sat;
    logic signed [14:0] P_term;
    logic signed [9:0] neg_div_ptch;
    logic signed [9:0] neg_div_ptch_a;
    logic signed [12:0] D_term;
    logic signed [14:0] I_term;
    logic signed [15:0] sum;
    logic [26:0] long_tmr; 
    logic signed [17:0] I_extend;
    logic signed [17:0] I_new;
    logic signed [17:0] I_add;
    logic signed [17:0] integrator;
    logic ov;

    // Saturate ptch and multiply by P_COEFF
    assign ptch_err_sat = ptch[15] && ~&ptch[14:9] ? 10'h200 :
                            ~ptch[15] && |ptch[14:9] ? 10'h1ff :
                            ptch[9:0];
    assign P_term = ptch_err_sat * P_COEFF;

    // Get D_term by div 64, flip sign, and then sign extend
    assign neg_div_ptch = ~ptch_rt[15:6] + 10'h001;
    assign D_term = {{3{neg_div_ptch[9]}}, neg_div_ptch};

    // Get I_term by accumulating old terms as an add and have reset available
    assign I_extend = $signed({{8{ptch_err_sat[9]}}, ptch_err_sat});
    assign I_add = I_extend + integrator;
    // detect overflow using sign bits of operands and result (use integrator MSB, not I_term)
    assign ov = (I_extend[17] & ~I_add[17] & integrator[17]) | (~I_extend[17] & I_add[17] & ~integrator[17]);
    // parenthesize the condition for clarity
    assign I_new = (vld & ~ov) ? I_add : integrator;
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            integrator <= 18'h00000;
        else if (rider_off)
            integrator <= 18'h00000;
        else
            integrator <= I_new;
    end

    generate
        if (FAST_SIM == 1)
            assign I_term = integrator[17] & ~&integrator[16:15] ? 15'h4000 :
                            ~integrator[17] & |integrator[16:15] ? 15'h3fff :
            
                            integrator[15:1];
        else
            assign I_term = {integrator[17], integrator[17], integrator[17], integrator[17:6]};
    endgenerate

    // Take the sum of all terms and saturate to twelve bits
    assign sum = {P_term[14], P_term} + {{3{D_term[12]}}, D_term} + {I_term[14], I_term};
    assign PID_cntrl = sum[15] && ~&sum[14:11] ? 12'h800 :
                        ~sum[15] && |sum[14:11] ? 12'h7FF :
                        sum[11:0];

    // Slow start timer logic. Count on each clock and take upper bits as accumulated time
    generate
        if (FAST_SIM ==1) begin
            assign ss_tmr = long_tmr[26:19];
            always_ff @ (posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    long_tmr <= 27'h0000000;
                end else if (!pwr_up) begin
                    long_tmr <= 27'h0000000;
                end else begin
                    long_tmr <= &long_tmr[26:19] ? long_tmr : long_tmr + 256;
                end
            end
        end else begin
            assign ss_tmr = long_tmr[26:19];
            always_ff @ (posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    long_tmr <= 27'h0000000;
                end else if (!pwr_up) begin
                    long_tmr <= 27'h0000000;
                end else begin
                    long_tmr <= &long_tmr[26:19] ? long_tmr : long_tmr + 1;
                end
            end
        end
    endgenerate

endmodule
