module balance_cntrl (
    input logic clk,
    input logic rst_n,
    input logic vld, // New inertial sensor reading is valid
    input logic pwr_up, // when segway balance control is powered up
    input logic rider_off, // asserted when no rider detected
    input logic signed [15:0] ptch, // Signed 16-bit pitch signal from inertial_interface
    input logic signed [15:0] ptch_rt, // Signed 16-bit pitch rate from inertial_interface. Used for D_term.
    input logic [11:0] steer_pot,
    input logic en_steer,
    output logic signed [11:0] lft_spd, 
    output logic signed [11:0] rght_spd,
    output logic too_fast
);

logic [7:0] ss_tmr; // soft start timer for SegwayMath
logic signed [11:0] PID_cntrl; // 12-bit signed result of PID control
parameter FAST_SIM = 1'b1;

// Instantiate PID

PID #(.fast_sim(FAST_SIM)) pid_inst (
    .clk(clk),
    .rst_n(rst_n),
    .vld(vld),
    .pwr_up(pwr_up),
    .rider_off(rider_off),
    .ptch(ptch),
    .ptch_rt(ptch_rt),
    .ss_tmr(ss_tmr),
    .PID_cntrl(PID_cntrl)
);

// Instantiate SegwayMath

SegwayMath segway_math_inst (
    .PID_cntrl(PID_cntrl),
    .ss_tmr(ss_tmr),
    .steer_pot(steer_pot),
    .en_steer(en_steer),
    .pwr_up(pwr_up),
    .lft_spd(lft_spd),
    .rght_spd(rght_spd),
    .too_fast(too_fast)
);

endmodule

module PID (
    input logic clk,
    input logic rst_n,
    input logic vld, // New inertial sensor reading is valid
    input logic pwr_up, // when segway balance control is powered up
    input logic rider_off, // asserted when no rider detected
    input logic signed [15:0] ptch, // Signed 16-bit pitch signal from inertial_interface
    input logic signed [15:0] ptch_rt, // Signed 16-bit pitch rate from inertial_interface. Used for D_term.
    output logic [7:0] ss_tmr, // soft start timer for SegwayMath
    output logic signed [11:0] PID_cntrl // 12-bit signed result of PID control
);

parameter fast_sim = 1'b1;

logic signed [17:0] integrator; // 18-bit signed accumulator for I_term
wire signed [9:0] ptch_err_sat; // 10-bit saturated pitch error
wire signed [15:0] PID_sum; // 16-bit sum of P, I and D terms before saturation
wire pid_sum_all_ones_signed = &PID_sum[14:11]; // for checking if PID sum is too negative
wire pid_sum_not_all_zeroes_signed = |PID_sum[14:11]; // for checking if PID sum is too positive
localparam P_COEFF = 5'h09;
wire signed [14:0] P_term;
wire signed [14:0] I_term;
wire signed [12:0] D_term;

// Saturate the ptch
assign ptch_err_sat = (~&ptch[14:9] && ptch[15]) ? 10'b1000000000 : // Negative saturation
                        (|ptch[14:9] && !ptch[15]) ? 10'b0111111111 : // Positive saturation
                        ptch[9:0]; // Error within range

// P_term calculation
assign P_term = ptch_err_sat * $signed(P_COEFF);

// I_term calculation
wire ov;
wire signed [17:0] sum1;
wire signed [17:0] ptch_err_sat_extended;
logic signed [17:0] integrator_next;

// Overflow logic
assign ov = (integrator[17] == ptch_err_sat_extended[17]) && (sum1[17] != integrator[17]);

// Integrator accumulator
assign ptch_err_sat_extended = {{8{ptch_err_sat[9]}}, ptch_err_sat}; // sign extend to 18 bits
assign sum1 = ptch_err_sat_extended + integrator;
assign integrator_next = (rider_off) ? 18'h00000 : (vld & ~ov) ? sum1 : integrator;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) integrator <= 0;
    else integrator <= integrator_next;
end

generate
    if (fast_sim) begin
        assign I_term = (integrator[17] && ~&integrator[16:15]) ? 15'b100000000000000 :
                        (!integrator[17] && |integrator[16:15]) ? 15'b011111111111111 :
                        integrator[15:1];
    end
    else begin
        assign I_term = {{3{integrator[17]}}, integrator[17:6]};
    end
endgenerate

// D_term calculation
assign D_term = ~{{3{ptch_rt[15]}}, ptch_rt[15:6]} + 1'b1;

// PID control signal calculation
// sign extend to 16 bits P, I and D terms
assign PID_sum = ({{1{P_term[14]}}, P_term} + {{1{I_term[14]}}, I_term} + {{3{D_term[12]}}, D_term});
// Saturate the PID control signal to 12 bits
assign PID_cntrl = (!pid_sum_all_ones_signed && PID_sum[15]) ? 12'b100000000000 : // Negative saturation
                        (pid_sum_not_all_zeroes_signed && !PID_sum[15]) ? 12'b011111111111 : // Positive saturation
                        PID_sum[11:0]; // Error within range

// Soft start timer logic
logic [26:0] long_tmr;
logic [26:0] long_tmr_next;

logic [8:0] increment_by;

generate
    if (fast_sim) begin
        assign increment_by = 9'b100000000; // 256
    end
    else begin
        assign increment_by = 9'b000000001;
    end
endgenerate

assign long_tmr_next = (pwr_up) ? ((&long_tmr[26:19]) ? long_tmr : long_tmr + increment_by) : 27'b0;

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        long_tmr <= 27'b0;
    else
        long_tmr <= long_tmr_next;
end

assign ss_tmr = long_tmr[26:19];


endmodule

module SegwayMath (
    input logic signed [11:0] PID_cntrl,
    input logic [7:0] ss_tmr,
    input logic [11:0] steer_pot,
    input logic en_steer,
    input logic pwr_up,
    output logic signed [11:0] lft_spd, 
    output logic signed [11:0] rght_spd,
    output logic too_fast
);

// local parameters
localparam MIN_DUTY = 13'h0A8;
localparam LOW_TORQUE_BAND = 7'h2A;
localparam GAIN_MULT = 4'h4;
localparam MAX_SPEED = 12'd1536;

// Internal signals
// for steering
logic signed [19:0] cntrl_times_ss;
logic signed [11:0] PID_ss;
logic unsigned [11:0] steer_pot_lmt;
logic signed [12:0] steer_diff;
logic signed [12:0] steer_rght_shft_3; // steer_diff * 2/16
logic signed [12:0] steer_rght_shft_4; // steer_diff * 1/16
logic signed [12:0] steer_inpt_fraction; // fraction of steer input to be added/subtracted to error
logic signed [12:0] non_zero_lft_torque;
logic signed [12:0] non_zero_rght_torque;
logic signed [12:0] lft_torque;
logic signed [12:0] rght_torque;
// for deadzone shaping
logic signed [12:0] lft_torque_comp_high_gain;
logic signed [12:0] lft_torque_comp_low_gain;
logic signed [12:0] lft_torque_comp;
logic signed [16:0] lft_torque_deadzone;
logic signed [12:0] lft_torque_pwrd_up;
logic signed [12:0] lft_shaped;
logic signed [12:0] rght_torque_comp_high_gain;
logic signed [12:0] rght_torque_comp_low_gain;
logic signed [12:0] rght_torque_comp;
logic signed [12:0] rght_torque_deadzone;
logic signed [12:0] rght_torque_pwrd_up;
logic signed [12:0] rght_shaped;
logic signed [12:0] lft_torque_abs_val;
logic signed [12:0] rght_torque_abs_val;

// Scaling with soft start
assign cntrl_times_ss = PID_cntrl * $signed({1'b0, ss_tmr});
assign PID_ss = cntrl_times_ss[19:8]; // Divide by 256

// Calculate steering input
// limit steer_pot
assign steer_pot_lmt = (steer_pot < 12'h200) ? 12'h200 :
                       (steer_pot > 12'hE00) ? 12'hE00 :
                       steer_pot;
assign steer_diff = $signed({1'b0, steer_pot_lmt}) - $signed({1'b0, 12'h7ff}); // Center at 0
assign steer_rght_shft_3 = $signed(steer_diff) >>> 3; // multiply by 2/16
assign steer_rght_shft_4 = $signed(steer_diff) >>> 4; // multiply by 1/16
assign steer_inpt_fraction = steer_rght_shft_3 + steer_rght_shft_4; // multiply by 3/16

// Calculate left and right torque
assign non_zero_lft_torque = {PID_ss[11], PID_ss} + steer_inpt_fraction;
assign non_zero_rght_torque = {PID_ss[11], PID_ss} - steer_inpt_fraction;
assign lft_torque = (en_steer) ? non_zero_lft_torque : {PID_ss[11], PID_ss};
assign rght_torque = (en_steer) ? non_zero_rght_torque : {PID_ss[11], PID_ss};

// Deadzone shaping for left dc motor torque
assign lft_torque_comp_high_gain = lft_torque - $signed(MIN_DUTY);
assign lft_torque_comp_low_gain = lft_torque + $signed(MIN_DUTY);
assign lft_torque_comp = (lft_torque[12]) ? lft_torque_comp_high_gain : lft_torque_comp_low_gain;
assign lft_torque_deadzone = lft_torque * $signed(GAIN_MULT);
assign lft_torque_abs_val = (lft_torque < 0) ? -lft_torque : lft_torque;
assign lft_torque_pwrd_up = (lft_torque_abs_val < LOW_TORQUE_BAND) ? lft_torque_deadzone : lft_torque_comp;
assign lft_shaped = (pwr_up) ? lft_torque_pwrd_up : $signed(13'h0000);

// Deadzone shaping for right dc motor torque
assign rght_torque_comp_high_gain = rght_torque - $signed(MIN_DUTY);
assign rght_torque_comp_low_gain = rght_torque + $signed(MIN_DUTY);
assign rght_torque_comp = (rght_torque[12]) ? rght_torque_comp_high_gain : rght_torque_comp_low_gain;
assign rght_torque_deadzone = rght_torque * $signed(GAIN_MULT);
assign rght_torque_abs_val = (rght_torque < 0) ? -rght_torque : rght_torque;
assign rght_torque_pwrd_up = (rght_torque_abs_val < LOW_TORQUE_BAND) ? rght_torque_deadzone : rght_torque_comp;
assign rght_shaped = (pwr_up) ? rght_torque_pwrd_up : $signed(13'h0000);

// Final saturation and over speed detect
assign lft_spd = (lft_shaped[12] && !lft_shaped[11]) ? $signed(12'b100000000000) : // Negative saturation
                 (!lft_shaped[12] && lft_shaped[11]) ? $signed(12'b011111111111) : // Positive saturation
                 lft_shaped[11:0]; // lft_shaped within range
assign rght_spd = (rght_shaped[12] && !rght_shaped[11]) ? $signed(12'b100000000000) : // Negative saturation
                  (!rght_shaped[12] && rght_shaped[11]) ? $signed(12'b011111111111) : // Positive saturation
                  rght_shaped[11:0]; // lft_shaped within range
assign too_fast = (lft_spd > $signed(MAX_SPEED)) || (rght_spd > $signed(MAX_SPEED));

endmodule
