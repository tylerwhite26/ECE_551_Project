// run at the start of testing to initialize all values to 0
// and hold reset to internally reset all phases
task initialize_DUT();

    begin
        rst_n = 0;
        clk = 0;
        cmd = 8'h00;
        send_cmd = 0;
        rider_lean = 16'h0000;
        ld_cell_lft = 12'h000;
        ld_cell_rght = 12'h000;
        steerPot = 12'h000;
        batt = 12'h000;
        OVR_I_lft = 0;
        OVR_I_rght = 0;
        repeat(10) @ (posedge clk);
        rst_n = 1;
    end

endtask


// send a command over UART and wait for the tx to confirm transmission
task block_send_command(input [7:0] cmd_to_send);

    begin
        cmd = cmd_to_send;
        send_cmd = 1;
        @ (posedge clk);
        send_cmd = 0;
        @ (cmd_sent);
    end

endtask


// send a command over UART. Returns control before transaction necessarily ends
task send_command(input [7:0] cmd_to_send);

    begin
        cmd = cmd_to_send;
        send_cmd = 1;
        @ (posedge clk);
        send_cmd = 0;
    end

endtask



