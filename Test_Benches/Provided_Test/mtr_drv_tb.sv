// Blue team, Tyler, Raymond, Parker, Paras.

module mtr_drv_tb;

logic      clk;
logic      rst_n;
logic [11:0] lft_spd;
logic [11:0] rght_spd;
logic      OVR_I_lft;
logic      OVR_I_rght;
logic      PWM1_lft;
logic      PWM2_lft;
logic      PWM1_rght;
logic      PWM2_rght;
logic      OVR_I_shtdwn;

// Instantiate the DUT
mtr_drv iDUT (
    .clk(clk),
    .rst_n(rst_n),
    .lft_spd(lft_spd),
    .rght_spd(rght_spd),
    .OVR_I_lft(OVR_I_lft),
    .OVR_I_rght(OVR_I_rght),
    .PWM1_lft(PWM1_lft),
    .PWM2_lft(PWM2_lft),
    .PWM1_rght(PWM1_rght),
    .PWM2_rght(PWM2_rght),
    .OVR_I_shtdwn(OVR_I_shtdwn)
);

int i = 0;

initial begin
    clk = 0;
    rst_n = 0;
    lft_spd = 12'h800;   // Neutral
    rght_spd = 12'h800;  // Neutral
    OVR_I_lft = 0;
    OVR_I_rght = 0;

    @(posedge clk);
    @(posedge clk);
    rst_n = 1;

    // Test 40 instances of overcurrent inside blanking period
    repeat (40) begin
        @(posedge clk);
      wait(PWM1_lft == 1'b1);
        OVR_I_lft = 1'b1;
        @(posedge clk);
        OVR_I_lft = 1'b0;
        wait(PWM1_lft == 1'b0);
    end
    if (OVR_I_shtdwn !== 1'b0) begin
         $error("Test failed: OVR_I_shtdwn should be 0 for 40 overcurrent events inside blanking period");
         $stop;
    end
    else $display("Test passed: OVR_I_shtdwn stays low");
    // Test 40 outside blanking period

    repeat (10) @(posedge clk); // wait a few cycles before next test

    for (i = 0; i<40; i++) begin
        wait(PWM1_lft == 1'b1);
        @(posedge clk);
        repeat( 130) @(posedge clk); // wait beyond blanking period
            OVR_I_lft = 1'b1;
            @(posedge clk);
            OVR_I_lft = 1'b0;
            wait(OVR_I_shtdwn == 1'b0);

            repeat(5) @(posedge clk); // wait a few cycles before next iteration
            if (OVR_I_shtdwn === 1'b1) begin
                break;
            end
        end

    if (OVR_I_shtdwn !== 1'b1) begin
        $error("Test failed: OVR_I_shtdwn should be 1 after 40 overcurrent events outside blanking period");
        $stop;
    end else if (i < 30) begin
        $error("Test failed: OVR_I_shtdwn asserted too early at %0d events", i);
        $stop;
    end else
    $display("Test passed: OVR_I_shtdwn asserted after %0d events", i);

    $display("All tests passed.");

    $stop;
   

end






always #10 clk = ~clk; // 50MHz clock  

endmodule