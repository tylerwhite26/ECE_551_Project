module steer_en 
#(parameter FAST_SIM = 1)
(
    input clk, 
    input rst_n,
    input [11:0] lft_ld,
    input [11:0] rght_ld,
    output logic en_steer,
    output logic rider_off
);
    localparam [11:0] MIN_RIDER_WT = 12'h200;
    localparam [7:0] WT_HYSTERESIS = 8'h40;

    // internal signals connecting to the state machine
    logic tmr_full, sum_gt_min, sum_lt_min, diff_gt_1_4, diff_gt_15_16, clr_tmr;
    logic [10:0] lft_rght_sum_quarter;
    logic [12:0] lft_rght_sum_15_16;
    logic [11:0] lft_rght_diff_abs;
    // sum and difference of inputs
    wire [12:0] sum = lft_ld + rght_ld;
    wire [12:0] diff = lft_ld - rght_ld;

    // internal state machine
    steer_en_SM sm(.clk(clk), .rst_n(rst_n), .tmr_full(tmr_full), .sum_gt_min(sum_gt_min), .sum_lt_min(sum_lt_min),
            .diff_gt_1_4(diff_gt_1_4), .diff_gt_15_16(diff_gt_15_16), .clr_tmr(clr_tmr), .en_steer(en_steer), .rider_off(rider_off));

    // summation assignment
    assign sum_lt_min = sum < MIN_RIDER_WT - WT_HYSTERESIS;
    assign sum_gt_min = sum > MIN_RIDER_WT + WT_HYSTERESIS;

    assign lft_rght_diff_abs = (lft_ld > rght_ld) ? (lft_ld - rght_ld) : (rght_ld - lft_ld); // Absolute value already
    assign lft_rght_sum_quarter = sum >> 2; // divide by 4
    assign lft_rght_sum_15_16 = (sum >> 1) + (sum >> 2) + (sum >> 3) + (sum >> 4); // 1/2 + 1/4 + 1/8 + 1/16
    assign diff_gt_1_4 = lft_rght_diff_abs > lft_rght_sum_quarter;
    assign diff_gt_15_16 = lft_rght_diff_abs > lft_rght_sum_15_16;

    // timer logic
    logic [25:0] timer;
    generate
        if (FAST_SIM)
            assign tmr_full = &timer[14:0];
        else
            assign tmr_full = &timer[25:17]; // appoximate to 1.337 seconds
    endgenerate
    always_ff @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            timer <= '0;
        else if (clr_tmr)
            timer <= '0;
        else
            timer <= timer + 1;
    end

endmodule



module steer_en_SM(clk,rst_n,tmr_full,sum_gt_min,sum_lt_min,diff_gt_1_4,
                   diff_gt_15_16,clr_tmr,en_steer,rider_off);

  input clk;				// 50MHz clock
  input rst_n;				// Active low asynch reset
  input tmr_full;			// asserted when timer reaches 1.3 sec
  input sum_gt_min;			// asserted when left and right load cells together exceed min rider weight
  input sum_lt_min;			// asserted when left_and right load cells are less than min_rider_weight

  /////////////////////////////////////////////////////////////////////////////
  // HEY HOFFMAN...you are a moron.  sum_gt_min would simply be ~sum_lt_min. 
  // Why have both signals coming to this unit??  ANSWER: What if we had a rider
  // (a child) who's weigth was right at the threshold of MIN_RIDER_WEIGHT?
  // We would enable steering and then disable steering then enable it again,
  // ...  We would make that child crash(children are light and flexible and 
  // resilient so we don't care about them, but it might damage our Segway).
  // We can solve this issue by adding hysteresis.  So sum_gt_min is asserted
  // when the sum of the load cells exceeds MIN_RIDER_WEIGHT + HYSTERESIS and
  // sum_lt_min is asserted when the sum of the load cells is less than
  // MIN_RIDER_WEIGHT - HYSTERESIS.  Now we have noise rejection for a rider
  // who's weight is right at the threshold.  This hysteresis trick is as old
  // as the hills, but very handy...remember it.
  //////////////////////////////////////////////////////////////////////////// 

  input diff_gt_1_4;		// asserted if load cell difference exceeds 1/4 sum (rider not situated)
  input diff_gt_15_16;		// asserted if load cell difference is great (rider stepping off)
  output logic clr_tmr;		// clears the 1.3sec timer
  output logic en_steer;	// enables steering (goes to balance_cntrl)
  output logic rider_off;	// held high in intitial state when waiting for sum_gt_min
  
  // You fill out the rest...use good SM coding practices ///
    typedef enum logic [1:0] {INITIAL, WAIT, STEER} state_t;
    state_t state;
    state_t next_state;

    // change states on clock
    always @ (posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= INITIAL;
        else
            state <= next_state;
    end

    always_comb begin
        next_state = state;
        rider_off = 0;
        clr_tmr = 0;
        en_steer = 0;
        case(state)
            INITIAL : begin
                if (sum_gt_min) begin
                    clr_tmr = 1;
                    next_state = WAIT;
                end else begin
                    rider_off = 1;
                end
            end
            WAIT : begin
                if (sum_lt_min) begin
                    rider_off = 1;
                    next_state = INITIAL;
                end else if (tmr_full) begin
                    en_steer = 1;
                    next_state = STEER;
                end else if (diff_gt_1_4) begin
                    clr_tmr = 1;
                end 
            end
            STEER : begin
                if (sum_lt_min) begin
                    rider_off = 1;
                    next_state = INITIAL;
                end else if (diff_gt_15_16) begin
                    next_state = WAIT;
                end else begin
                    en_steer = 1;
                end
            end
        endcase
    end
  
  
endmodule