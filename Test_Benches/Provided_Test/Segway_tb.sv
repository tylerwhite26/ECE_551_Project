module Segway_tb();
			
//// Interconnects to DUT/support defined as type wire /////
wire SS_n,SCLK,MOSI,MISO,INT;				// to inertial sensor
wire A2D_SS_n,A2D_SCLK,A2D_MOSI,A2D_MISO;	// to A2D converter
wire RX_TX;
wire PWM1_rght, PWM2_rght, PWM1_lft, PWM2_lft;
wire piezo,piezo_n;
logic cmd_sent;
wire rst_n;					// synchronized global reset

////// Stimulus is declared as type reg ///////
reg clk, RST_n;
reg [7:0] cmd;				// command host is sending to DUT
reg send_cmd;				// asserted to initiate sending of command
reg signed [15:0] rider_lean;
reg [11:0] ld_cell_lft, ld_cell_rght,steerPot,batt;	// A2D values
reg OVR_I_lft, OVR_I_rght;

///// Internal registers for testing purposes??? /////////


////////////////////////////////////////////////////////////////
// Instantiate Physical Model of Segway with Inertial sensor //
//////////////////////////////////////////////////////////////	
// Import tasks from package-based testbench helper
import tb_tasks_pkg::*;

SegwayModel iPHYS(.clk(clk),.RST_n(RST_n),.SS_n(SS_n),.SCLK(SCLK),
                  .MISO(MISO),.MOSI(MOSI),.INT(INT),.PWM1_lft(PWM1_lft),
				  .PWM2_lft(PWM2_lft),.PWM1_rght(PWM1_rght),
				  .PWM2_rght(PWM2_rght),.rider_lean(rider_lean));				  

/////////////////////////////////////////////////////////
// Instantiate Model of A2D for load cell and battery //
///////////////////////////////////////////////////////
ADC128S_FC iA2D(.clk(clk),.rst_n(RST_n),.SS_n(A2D_SS_n),.SCLK(A2D_SCLK),
             .MISO(A2D_MISO),.MOSI(A2D_MOSI),.ld_cell_lft(ld_cell_lft),.ld_cell_rght(ld_cell_rght),
			 .steerPot(steerPot),.batt(batt));			
	 
////// Instantiate DUT ////////
Segway iDUT(.clk(clk),.RST_n(RST_n),.INERT_SS_n(SS_n),.INERT_MOSI(MOSI),
            .INERT_SCLK(SCLK),.INERT_MISO(MISO),.INERT_INT(INT),.A2D_SS_n(A2D_SS_n),
			.A2D_MOSI(A2D_MOSI),.A2D_SCLK(A2D_SCLK),.A2D_MISO(A2D_MISO),
			.PWM1_lft(PWM1_lft),.PWM2_lft(PWM2_lft),.PWM1_rght(PWM1_rght),
			.PWM2_rght(PWM2_rght),.OVR_I_lft(OVR_I_lft),.OVR_I_rght(OVR_I_rght),
			.piezo_n(piezo_n),.piezo(piezo),.RX(RX_TX));

//// Instantiate UART_tx (mimics command from BLE module) //////
UART_tx iTX(.clk(clk),.rst_n(rst_n),.TX(RX_TX),.trmt(send_cmd),.tx_data(cmd),.tx_done(cmd_sent));

/////////////////////////////////////
// Instantiate reset synchronizer //
///////////////////////////////////
rst_synch iRST(.clk(clk),.RST_n(RST_n),.rst_n(rst_n));

initial begin
  clk = 0;
  RST_n = 0;
  batt = 12'hFFF;       // High battery
  ld_cell_lft = 12'h400;// Rider on left
  ld_cell_rght = 12'h400;// Rider on right
  rider_lean = 16'h0000;
  steerPot = 12'h800; // Centered
  RST_n = 1;      // Start HIGH
  repeat(100) @(posedge clk);
  RST_n = 0;     
  repeat(1000) @(posedge clk);
  RST_n = 1;      // Release Reset
  repeat(100000) @(posedge clk);
  // Send 'G' to power up segway
  // call package task, passing references and clk/signal used by the task
  block_send_command(8'h47, cmd, send_cmd, clk, cmd_sent);
  // Wait for a few thousand clock cycles to let the segway stabilize
  repeat(1000000) @(posedge clk);
  rider_lean = 16'h0FFF; // Invoke check_theta_platform task. Make sure theta_platform goes high then low. When rider_lean = 0, theta_platform should go negative and converge to 0
  @(posedge clk);
  check_theta_platform(rider_lean, clk);
  // Test a high negative rider_lean value function for 1 million clk cycles.Theta_platform should go low, then high, then converge to 0
  rider_lean = 16'h1FFF;
  $display("Starting second test...");
  @(posedge clk);
  check_theta_platform(rider_lean, clk); // Theta_platform goes low then high and converges to 0. When rider_lean = 0, theta_platform should go positive and converge to 0         
  $display("Second test complete.");
  $stop();
end

always
  #10 clk = ~clk; // 100MHz clock

endmodule	
