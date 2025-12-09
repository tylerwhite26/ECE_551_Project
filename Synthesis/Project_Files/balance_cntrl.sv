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

module PID #(parameter fast_sim = 1) (
  input  logic signed [15:0] ptch,       // Pitch signal from inertial interface
  input  logic clk, // 50MHz clock
  input  logic pwr_up, // power up signal for ss_tmr
  input  logic rider_off, // when high, integrator is held at zero
  input  logic rst_n, // active low reset
  input  logic vld, // new pitch data valid signal
  input  logic signed [15:0] ptch_rt,    // Pitch rate for D-term
  output logic signed [11:0] PID_cntrl,   // Final PID control output
  output logic [7:0] ss_tmr
);

logic signed [9:0] ptch_err_sat;
logic signed [17:0] integrator;
logic signed [14:0] P_term;
logic signed [14:0] I_term;
logic signed [12:0] D_term;
logic signed [15:0] P_ext, I_ext, D_ext;
logic signed [15:0] PID_ext;
logic signed [17:0] overflow_check;
logic [8:0] ss_tmr_speed;
logic [26:0] ss_tmr_ext;

    

localparam signed P_COEFF = 5'h09;



    // saturate the incoming signed 16-bit ptch to a signed 10-bit ptch_err_sat term.
    assign ptch_err_sat = (ptch[15]) ? (&ptch[14:9] ? ptch[9:0] : 10'h200) : 
    (|ptch[14:9] ? 10'h1ff : ptch[9:0]);

    assign P_term = ptch_err_sat * P_COEFF; // P-term calculation
    
    // Integrator logic 
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            integrator <= 18'h0; // reset integrator to zero
         else if (rider_off) 
            integrator <= 18'h0; // hold integrator at zero when rider is off
         else if (vld) begin
         // stop rolling over check MSB of two numbers being added if they match yet do not match of the addition freeze integrator
          overflow_check = integrator + ptch_err_sat;
         if ((integrator[17] != ptch_err_sat[9]) || (integrator[17] == overflow_check[17]))
            integrator <= integrator + ptch_err_sat; // accumulate error into integrator
         end
    end

generate 
    if (fast_sim)  begin
        assign I_term = integrator[17] ? (&integrator[16:15] ? integrator[15:1] : 15'h4000) : 
        (|integrator[16:15] ? 15'h3fff : integrator[15:1]); // fast sim version with coarser resolution saturated
        assign ss_tmr_speed = 9'h100; // faster soft start timer for fast sim
    end  else begin
    assign I_term = { {3{integrator[17]}}, integrator[17:6] };
    assign ss_tmr_speed = 9'h001; // normal soft start timer speed
    end
endgenerate

    // Divide ptch_rt by 64 for D-term and then negate
    assign D_term = -(ptch_rt >>> 6);

    // pipeline P, I, and D terms
    logic signed [14:0] P_term_piped;
    logic signed [14:0] I_term_piped;
    logic signed [12:0] D_term_piped;
    always_ff @(posedge clk) begin
        P_term_piped <= P_term;
        I_term_piped <= I_term;
        D_term_piped <= D_term;
    end

    // Extend P, I, and D terms to 16 bits for summation
    assign P_ext = {P_term_piped[14], P_term_piped}; 
    // Sign-extend I_term correctly from its MSB (bit 14). The previous code
    // used bit 11 as the sign which produced incorrect I_ext values and
    // caused improper PID saturation behavior in fast-sim mode.
    assign I_ext = {{I_term_piped[14]}, I_term_piped};
    assign D_ext = {{3{D_term_piped[12]}}, D_term_piped};

    // Get extended PID output before saturation to 12 bits
    assign PID_ext = (P_ext + I_ext + D_ext); 

    // Saturate to 12 bits for final output
    assign PID_cntrl = PID_ext[15] ? (&PID_ext[14:11] ? PID_ext[11:0] : 12'h800) : 
    (|PID_ext[14:11] ? 12'h7ff : PID_ext[11:0]);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
            ss_tmr_ext <= 27'h0; // reset timer to zero (match 27-bit width)
        else if (!pwr_up) 
            ss_tmr_ext <= 27'h0; // hold timer at zero when power is down
        else if (!(&ss_tmr_ext[26:19])) 
         ss_tmr_ext <= ss_tmr_ext + ss_tmr_speed; // increment timer   
    end

    assign ss_tmr = ss_tmr_ext[26:19]; // output the top 8 bits of the timer
    



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
assign steer_scaled_unshift = (steer_pot_sat * 3); // 12-bit signed (3/16)

// Pipeline steer_scaled
logic signed [11:0] steer_scaled_unshift_piped;
always_ff @(posedge clk) begin
    steer_scaled_unshift_piped <= steer_scaled_unshift;
end

assign steer_scaled = steer_scaled_unshift_piped >>> 4;
assign lft_torque = en_steer ? PID_ss_ext + steer_scaled : PID_ss_ext; // 13-bit signed
assign rght_torque = en_steer ? PID_ss_ext - steer_scaled : PID_ss_ext; // 13-bit signed

// Pipeline lft and rght torque
logic signed [12:0] lft_torque_piped;
logic signed [12:0] rght_torque_piped; 
always_ff @(posedge clk) begin
    lft_torque_piped <= lft_torque;
    rght_torque_piped <= rght_torque;
end

// left torque shaping
assign lft_torque_comp = lft_torque_piped[12] ? (lft_torque_piped - MIN_DUTY) : (lft_torque_piped + MIN_DUTY); 
assign lft_abs = (lft_torque_piped[12]) ? -lft_torque_piped : lft_torque_piped; // 13-bit unsigned
assign lft_shaped = pwr_up ? ((lft_abs > LOW_TORQUE_BAND) ? lft_torque_comp : lft_torque_piped*GAIN_MULT) : 13'h0000;

// right torque shaping
assign rght_torque_comp = rght_torque_piped[12] ? (rght_torque_piped - MIN_DUTY) : (rght_torque_piped + MIN_DUTY);
assign rght_abs = (rght_torque_piped[12]) ? -rght_torque_piped : rght_torque_piped; // 13-bit unsigned
assign rght_shaped = pwr_up ? ((rght_abs > LOW_TORQUE_BAND) ? rght_torque_comp : rght_torque_piped*GAIN_MULT) : 13'h0000;

// Pipeline lft and rght abs comp
logic signed [12:0] lft_shaped_piped;
logic signed [12:0] rght_shaped_piped;
always_ff @(posedge clk) begin
    lft_shaped_piped <= lft_shaped;
    rght_shaped_piped <= rght_shaped;
end

// final speed assignments
// Register outputs to increase pipeline depth and better match the other implementation
logic signed [11:0] lft_spd_internal;
logic signed [11:0] rght_spd_internal;
logic too_fast_internal;

assign lft_spd_internal = lft_shaped_piped[12] ? (lft_shaped_piped[11] ? lft_shaped_piped[11:0] : 12'h800) : (lft_shaped_piped[11] ? 12'h7ff : lft_shaped_piped[11:0]);
assign rght_spd_internal = rght_shaped_piped[12] ? (rght_shaped_piped[11] ? rght_shaped_piped[11:0] : 12'h800) : (rght_shaped_piped[11] ? 12'h7ff : rght_shaped_piped[11:0]);
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
