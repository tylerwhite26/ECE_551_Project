module balance_cntrl #(parameter fast_sim = 1) (
  input  logic signed [15:0] ptch,       // Pitch signal from inertial interface
  input  logic        [11:0] steer_pot,   // Unsigned 12-bit steering potentiometer
  input  logic               en_steer,    // Steering enable
  input  logic clk, // 50MHz clock
  input  logic pwr_up, // power up signal for ss_tmr
  input  logic rider_off, // when high, integrator is held at zero
  input  logic rst_n, // active low reset
  input  logic vld, // new pitch data valid signal
  input  logic signed [15:0] ptch_rt,    // Pitch rate for D-term
  output logic signed [11:0] lft_spd,     // Signed 12-bit left motor speed
  output logic signed [11:0] rght_spd,    // Signed 12-bit right motor speed
  output logic               too_fast     // Indicates overspeed
);

// Internal signals to connect submodules
logic signed [11:0] PID_cntrl;
logic [7:0] ss_tmr;

PID #(.fast_sim(fast_sim)) u_PID (
  .ptch(ptch),
  .clk(clk),
  .pwr_up(pwr_up),
  .rider_off(rider_off),
  .rst_n(rst_n),
  .vld(vld),
  .ptch_rt(ptch_rt),
  .PID_cntrl(PID_cntrl),
  .ss_tmr(ss_tmr)
);

SegwayMath u_SegwayMath (
    .PID_cntrl(PID_cntrl),
    .ss_tmr(ss_tmr),
    .clk(clk),
    .rst_n(rst_n),
  .steer_pot(steer_pot),
  .en_steer(en_steer),
  .pwr_up(pwr_up),
  .lft_spd(lft_spd),
  .rght_spd(rght_spd),
  .too_fast(too_fast)
);


endmodule

module PID
#(parameter fast_sim = 1)

(
    input  signed [15:0] ptch,
    input  signed [15:0] ptch_rt, // Used for D_Term
    input pwr_up, // Asserted when segway balance control is powered up. Used to keep ss_tmr at zero until then.
    input rider_off, // Asserted when no rider detected
    input vld,
    input clk,
    input rst_n,
    output signed [11:0] PID_cntrl,
    output logic [7:0] ss_tmr
);
// This is the error and P-Term 
logic signed [9:0]  ptch_err_sat;
logic signed [14:0] P_term;
localparam [4:0] P_COEFF = 5'h09;


assign ptch_err_sat = (ptch[15] == 1'b1 && !(&ptch[14:9])) ? 10'sh200 : 
                        (ptch[15] == 1'b0 && |ptch[14:9]) ? 10'sh1FF : 
                        $signed(ptch[9:0]);
assign P_term = $signed(P_COEFF) * ptch_err_sat;


// This is for the I term
logic signed [17:0] ptch_err_sat_extended;
logic signed [17:0] integrator;
assign ptch_err_sat_extended = { {8{ptch_err_sat[9]}}, ptch_err_sat };
logic signed [18:0] sum;
logic ov;

assign sum = integrator + ptch_err_sat_extended;
assign ov = (~integrator[17] & ~ptch_err_sat_extended[17] & sum[17]) | 
            (integrator[17] & ptch_err_sat_extended[17] & ~sum[17]);

logic signed [17:0] intermediate_signal;
assign intermediate_signal = ((!ov) && vld) ? sum : integrator;



always_ff@(posedge clk or negedge rst_n)
    if (!rst_n)
        integrator <= 18'h00000;
    else if (rider_off)
        integrator <= 18'h00000;
    else 
        integrator <= intermediate_signal;

// This is for the D_Term
logic signed [12:0] D_term;
assign D_term = -$signed(ptch_rt >>> 6);


logic [26:0] long_tmr;
// Generate statement based on if fast_sim is enabled
generate
    if (fast_sim) begin : fast_timer
        always_ff@(posedge clk or negedge rst_n)
            if (!rst_n)
                long_tmr <= 27'h0000000;
            else if (!pwr_up)
                long_tmr <= 27'h0000000;
            else if (!(&long_tmr[26:19]))
                long_tmr <= long_tmr + 256;
    end else begin : normal_timer
        always_ff@(posedge clk or negedge rst_n)
            if (!rst_n)
                long_tmr <= 27'h0000000;
            else if (!pwr_up)
                long_tmr <= 27'h0000000;
            else if (!(&long_tmr[26:19]))
                long_tmr <= long_tmr + 1;
    end
endgenerate

assign ss_tmr = long_tmr[26:19];


// This is for the final term
wire signed [15:0] P_term_ext = {P_term[14], P_term};
wire signed [15:0] D_term_ext = {D_term[12], D_term[12], D_term[12], D_term};

wire signed [14:0] I_term_ext;
generate
    if (fast_sim) begin : fast_I_term
        // Fast sim: tap bits [15:1] with saturation check on bits [17:15]
        assign I_term_ext = (integrator[17] == 1'b1 && !(&integrator[16:14])) ? 15'sh4000 :
                           (integrator[17] == 1'b0 && |integrator[16:14]) ? 15'sh3FFF :
                           integrator[15:1];
    end else begin : normal_I_term
        // Normal: tap bits [17:6] with sign extension
        assign I_term_ext = {{3{integrator[17]}}, integrator[17:6]};
    end
endgenerate

wire signed [16:0] addedSum;
assign addedSum = (P_term_ext) + (D_term_ext) + (I_term_ext);

assign PID_cntrl = (addedSum[16] == 1 && !(&addedSum[15:11])) ? 12'sh800 : 
                        (addedSum[16] == 0 && |addedSum[15:11]) ? 12'sh7FF : addedSum[11:0];

endmodule

module SegwayMath (
    input  logic               clk,
    input  logic               rst_n,
    input  logic signed [11:0] PID_cntrl,   // Signed 12-bit control from PID
    input  logic        [7:0]  ss_tmr,      // Unsigned 8-bit scaling quantity
    input  logic        [11:0] steer_pot,   // Unsigned 12-bit steering potentiometer
    input  logic               en_steer,    // Steering enable
    input  logic               pwr_up,      // Power-up signal

    output logic signed [11:0] lft_spd,     // Signed 12-bit left motor speed
    output logic signed [11:0] rght_spd,    // Signed 12-bit right motor speed
    output logic               too_fast     // Indicates overspeed
);
  

logic signed [19:0] PID_ext; // Signed 20-bit scaled PID
logic signed [11:0] PID_ss;
logic signed [12:0] PID_ss_ext;
logic signed [11:0] steer_clip; // Signed 12-bit steering adjustment
logic signed [11:0] steer_pot_sat;
logic signed [11:0] steer_scaled;    // 3/16 scaled version
logic signed [12:0] lft_torque;   // Signed 13-bit left motor temp
logic signed [12:0] rght_torque;  // Signed 13-bit right motor
logic [12:0] lft_abs; // Signed 13-bit left motor abs
logic [12:0] rght_abs;    // Signed 13-bit right motor abs
logic signed [12:0] lft_torque_comp;
logic signed [12:0] rght_torque_comp;
logic signed [12:0] lft_shaped;
logic signed [12:0] rght_shaped;

localparam pot_ctr = 12'h7ff; // center position of pot
localparam pot_low = 12'h200; // low position of pot
localparam pot_hgh = 12'hE00; // high position of pot
localparam signed [12:0] MIN_DUTY = 13'h0A8; // minimum duty cycle to overcome motor deadband
localparam signed [6:0] LOW_TORQUE_BAND = 7'h2A; // torque band for min duty cycle
localparam signed [3:0] GAIN_MULT = 4'h4; // gain multiplier for speed calculation
localparam signed [11:0] MAX_SPEED = 12'h600; // maximum speed limit




// Pipelining: register PID inputs and soft-start multiply to increase flop count
logic signed [11:0] PID_cntrl_piped;
logic [7:0] ss_tmr_piped;
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        PID_cntrl_piped <= '0;
        ss_tmr_piped <= '0;
    end else begin
        PID_cntrl_piped <= PID_cntrl;
        ss_tmr_piped <= ss_tmr;
    end
end

logic signed [19:0] PID_mult;
// produce the multiply result combinationally, then register the 12-bit slice
assign PID_mult = ($signed({1'b0, ss_tmr_piped})) * PID_cntrl_piped; // 20-bit signed
always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        PID_ss <= '0;
    else
        PID_ss <= PID_mult[19:8]; // registered 12-bit result
end
assign PID_ss_ext = {PID_ss[11], PID_ss};  // 12-bit → 13-bit signed


// motor torque calculations
assign steer_clip = steer_pot < pot_low ? pot_low : (steer_pot > pot_hgh ? pot_hgh : steer_pot);
assign steer_pot_sat = steer_clip - pot_ctr;
assign steer_scaled = (steer_pot_sat * 3) >>> 4; // 12-bit signed (3/16)

assign lft_torque = en_steer ? PID_ss_ext + steer_scaled : PID_ss_ext; // 13-bit signed
assign rght_torque = en_steer ? PID_ss_ext - steer_scaled : PID_ss_ext; // 13-bit signed

// left torque shaping
assign lft_torque_comp = lft_torque[12] ? (lft_torque - MIN_DUTY) : (lft_torque + MIN_DUTY); 
assign lft_abs = (lft_torque[12]) ? -lft_torque : lft_torque; // 13-bit unsigned
assign lft_shaped = pwr_up ? ((lft_abs > LOW_TORQUE_BAND) ? lft_torque_comp : lft_torque*GAIN_MULT) : 13'h0000;

// right torque shaping
assign rght_torque_comp = rght_torque[12] ? (rght_torque - MIN_DUTY) : (rght_torque + MIN_DUTY);
assign rght_abs = (rght_torque[12]) ? -rght_torque : rght_torque; // 13-bit unsigned
assign rght_shaped = pwr_up ? ((rght_abs > LOW_TORQUE_BAND) ? rght_torque_comp : rght_torque*GAIN_MULT) : 13'h0000;


// final speed assignments
// Register outputs to increase pipeline depth and better match the other implementation
logic signed [11:0] lft_spd_internal;
logic signed [11:0] rght_spd_internal;
logic too_fast_internal;

assign lft_spd_internal = lft_shaped[12] ? (lft_shaped[11] ? lft_shaped[11:0] : 12'h800) : (lft_shaped[11] ? 12'h7ff : lft_shaped[11:0]);
assign rght_spd_internal = rght_shaped[12] ? (rght_shaped[11] ? rght_shaped[11:0] : 12'h800) : (rght_shaped[11] ? 12'h7ff : rght_shaped[11:0]);
assign too_fast_internal = (lft_spd_internal > $signed(MAX_SPEED)) | (rght_spd_internal > $signed(MAX_SPEED));

always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lft_spd <= '0;
        rght_spd <= '0;
        too_fast <= 1'b0;
    end else begin
        lft_spd <= lft_spd_internal;
        rght_spd <= rght_spd_internal;
        too_fast <= too_fast_internal;
    end
end



endmodule