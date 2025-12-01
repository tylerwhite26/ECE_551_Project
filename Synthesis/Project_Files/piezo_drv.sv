module piezo_drv

#(parameter fast_sim = 1)
(
    input en_steer,
    input too_fast,
    input batt_low,
    input clk,
    input rst_n,
    output logic piezo,
    output logic piezo_n
);

// Duration and frequency constants (use 32-bit widths to avoid sizing issues)
localparam [31:0] DURATION_2_23 = 32'd8388608;      // 2^23 clocks
localparam [31:0] DURATION_2_22 = 32'd4194304;      // 2^22 clocks
localparam [31:0] DURATION_2_25 = 32'd33554432;     // 2^25 clocks
localparam [31:0] DURATION_COMBINED = 32'd12582912; // 2^23 + 2^22 clocks
 
localparam [27:0] THREE_SEC = 28'd150000000;        // 3 seconds at 50MHz

// Frequency periods (clock cycles for full period)
localparam [14:0] G6_PERIOD = 15'd31888;   // 1568 Hz (50MHz / 1568)
localparam [14:0] C7_PERIOD = 15'd23893;   // 2093 Hz (50MHz / 2093)
localparam [14:0] E7_PERIOD = 15'd18962;   // 2637 Hz (50MHz / 2637)
localparam [14:0] G7_PERIOD = 15'd15943;   // 3136 Hz (50MHz / 3136)

// State machine declaration
typedef enum logic [2:0] {IDLE, G6, C7, E7, G7, E7S, G7L} state_t;
state_t state, next_state;

// Control signals
logic duration_done, period_done, three_sec_up;
logic [14:0] current_period;
logic [25:0] target_duration;

// Timer declarations  
logic [28:0] counter;
logic [25:0] durationTimer; 
logic [15:0] frequencyTimer;

// Timer comparisons
assign duration_done = (durationTimer >= target_duration);
assign period_done = (frequencyTimer >= current_period);
assign three_sec_up = (counter >= THREE_SEC);

generate 
    if (fast_sim) begin : fast_timer
    always_ff @(posedge clk, negedge rst_n) begin 
    if (!rst_n) 
        counter <= 0;
    else if (three_sec_up || (state == IDLE && next_state != IDLE))
        counter <= 0;  // Reset when 3 seconds up or starting new sequence
    else 
        counter <= counter + 64;
    end
    end else begin : normal_timer
        always_ff @(posedge clk, negedge rst_n)  
            if (!rst_n) 
                counter <= 0;
            else if (three_sec_up || (state == IDLE && next_state != IDLE))
                counter <= 0;  // Reset when 3 seconds up or starting new sequence
            else 
                counter <= counter + 1;
    end
    endgenerate

generate 
    if (fast_sim) begin : fast_timer_one
    always_ff @(posedge clk, negedge rst_n) begin 
    if (!rst_n) 
        durationTimer <= 0;
    else if (duration_done)
        durationTimer <= 0;  // Reset when duration completes
    else 
        durationTimer <= durationTimer + 64;
    end
    end else begin : normal_timer_one
        always_ff @(posedge clk, negedge rst_n) begin
            if (!rst_n)
                durationTimer <= 0;
            else if (duration_done)
                durationTimer <= 0;  // Reset when duration completes
            else 
                durationTimer <= durationTimer + 1;
        end
    end
endgenerate

generate 
    if (fast_sim) begin : fast_timer_two
    always_ff @(posedge clk, negedge rst_n) begin 
    if (!rst_n) 
        frequencyTimer <= 0;
    else if (period_done)
        frequencyTimer <= 0;  // Reset when period completes
    else 
        frequencyTimer <= frequencyTimer + 64;
    end
    end else begin : normal_timer_two
        always_ff @(posedge clk, negedge rst_n)  
            if (!rst_n) 
                frequencyTimer <= 0;
            else if (period_done)
                frequencyTimer <= 0;  // Reset when period completes
            else 
                frequencyTimer <= frequencyTimer + 1;
    end
    endgenerate

// State machine
always_ff@(posedge clk, negedge rst_n) begin
    if (!rst_n)
        state <= IDLE;
    else 
        state <= next_state;
end


always_comb begin
    next_state = state;
    piezo = 0;
    piezo_n = 1;
    current_period = G6_PERIOD;  // Default values
    target_duration = DURATION_2_23;
    case(state) 
        IDLE: begin
            // Priority: too_fast > batt_low > en_steer
            if (too_fast) begin // Play first three notes continuously (highest priority)
                next_state = G6;
            end
            else if (batt_low && three_sec_up) begin // Play backwards every 3 seconds
                next_state = G7L;
            end
            else if (en_steer && three_sec_up) begin // Play charge fanfare every 3 seconds
                next_state = G6;
            end
        end
        
        G6: begin // 1568 Hz 2^23 clocks
            current_period = G6_PERIOD;
            target_duration = DURATION_2_23;
            
            // Generate square wave: toggle when half period reached
            if (frequencyTimer >= (current_period >> 1)) begin
                piezo = 1;
                piezo_n = 0;
            end else begin
                piezo = 0;
                piezo_n = 1;
            end
            if (duration_done) begin
                if (too_fast)
                    next_state = C7; // Loop back for continuous play of first 3 notes
                else if (batt_low)
                    next_state = IDLE; // Start backwards sequence if battery low
                else
                next_state = C7;
            end
        end
        
        C7: begin // 2093 Hz 2^23 clocks
            current_period = C7_PERIOD;
            target_duration = DURATION_2_23;
            // Generate square wave: toggle when half period reached
            if (frequencyTimer >= (current_period >> 1)) begin
                piezo = 1;
                piezo_n = 0;
            end else begin
                piezo = 0;
                piezo_n = 1;
            end
            if (duration_done) begin
                next_state = E7;
                if (too_fast)
                    next_state = E7; // Loop back for continuous play of first 3 notes
                else if (batt_low)
                    next_state = G6; // Start backwards sequence if battery low
                else
                    next_state = E7; // Continue to complete charge fanfare
            end
        end 
        
        E7: begin // 2637 Hz 2^23 clocks
            current_period = E7_PERIOD;
            target_duration = DURATION_2_23;
            
            // Generate square wave: toggle when half period reached
            if (frequencyTimer >= (current_period >> 1)) begin
                piezo = 1;
                piezo_n = 0;
            end else begin
                piezo = 0;
                piezo_n = 1;
            end
            
            if (duration_done) begin
                if (too_fast)
                    next_state = G6; // Loop back for continuous play of first 3 notes
                else if (batt_low)
                    next_state = C7; // Start backwards sequence if battery low
                else if (en_steer)
                    next_state = G7; // Continue to complete charge fanfare
                else 
                    next_state = IDLE; // Then none are high, so continue on to idle.
            end
        end
        
        G7: begin // 3136 Hz 2^23 + 2^22 clocks
            current_period = G7_PERIOD;
            target_duration = DURATION_COMBINED;
            
            // Generate square wave: toggle when half period reached
            if (frequencyTimer >= (current_period >> 1)) begin
                piezo = 1;
                piezo_n = 0;
            end else begin
                piezo = 0;
                piezo_n = 1;
            end
            
            if (duration_done) begin
                if (batt_low)
                    next_state = E7; // Start backwards sequence if battery low
                else
                next_state = E7S;
            end
        end
        
        E7S: begin // 2637 Hz 2^22 clocks
            current_period = E7_PERIOD;
            target_duration = DURATION_2_22;
            
            // Generate square wave: toggle when half period reached
            if (frequencyTimer >= (current_period >> 1)) begin
                piezo = 1;
                piezo_n = 0;
            end else begin
                piezo = 0;
                piezo_n = 1;
            end
            
            if (duration_done) begin
                if (batt_low)
                    next_state = G7; 
                else
                    next_state = G7L; 
            end
        end
        G7L: begin // 3136 Hz longer duration 2^25 clocks
            current_period = G7_PERIOD;
            target_duration = DURATION_2_25;
            
            // Generate square wave: toggle when half period reached
            if (frequencyTimer >= (current_period >> 1)) begin
                piezo = 1;
                piezo_n = 0;
            end else begin
                piezo = 0;
                piezo_n = 1;
            end
            
            if (duration_done) begin
                if (batt_low)
                    next_state = E7S; 
                else
                    next_state = IDLE; 
            end
        end
    endcase
end



endmodule
