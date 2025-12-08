package tb_tasks_pkg;

  // Initialize DUT: set signals to defaults and pulse release reset
  task automatic initialize_DUT(
      ref logic RST_n,
      input  logic clk,
      ref logic [7:0] cmd,
      ref logic send_cmd,
      ref logic signed [15:0] rider_lean,
      ref logic [11:0] ld_cell_lft,
      ref logic [11:0] ld_cell_rght,
      ref logic [11:0] steerPot,
      ref logic [11:0] batt,
      ref logic OVR_I_lft,
      ref logic OVR_I_rght
  );
    begin
      RST_n = 0;
      // Leave `clk` driven by the testbench clock generator; do not assign to it here
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
      RST_n = 1;
    end
  endtask

  // Send a command over UART and wait for TX to confirm transmission
  task automatic block_send_command(
      input  logic [7:0] cmd_to_send,
      ref    logic [7:0] cmd,
      ref    logic send_cmd,
      ref  logic clk,
      ref  logic cmd_sent
  );
    begin
      cmd = cmd_to_send;
      send_cmd = 1;
      @ (posedge cmd_sent);
      $display("Testing");
      send_cmd = 0;
    end
  endtask

  // Send a command but return control before transaction necessarily ends
  task automatic send_command(
      input logic [7:0] cmd_to_send,
      ref   logic [7:0] cmd,
      ref   logic send_cmd,
      ref logic clk
  );
    begin
      cmd = cmd_to_send;
      send_cmd = 1;
      @ (posedge clk);
      send_cmd = 0;
    end
  endtask

  // Creates a tilt to test theta_platform waveform
  task automatic check_theta_platform(
      ref logic signed [15:0] rider_lean_var,
      ref logic clk
  );
    begin
      // 1. Wait long enough for the physics to react to the lean
      // 200,000 clocks @ 10ns = 2ms. You might need even more (e.g., 1,000,000) 
      // to see the platform converge.
      repeat(350000) @ (posedge clk);
      
      // Abruptly set rider_lean to 0 and observe response
      rider_lean_var = 16'h0000;
      
      // Wait for it to settle back to 0
      repeat(350000) @ (posedge clk);
    end
  endtask

endpackage : tb_tasks_pkg
