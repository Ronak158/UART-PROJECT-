`timescale 1ns/1ps

module uart_fifo_tb;

  // Parameters for the FIFO (must match the instantiated module)
  localparam DATA_WIDTH = 8;
  localparam DEPTH_BITS = 4; // Corresponds to 16 entries (2^4)
  localparam FIFO_DEPTH = (1 << DEPTH_BITS); // 16

  // Testbench signals
  reg clk;
  reg rstn; // Active low reset
  reg push;
  reg pop;
  reg [DATA_WIDTH-1:0] data_in;

  wire [DATA_WIDTH-1:0] data_out;
  wire full;
  wire empty;
  wire [DEPTH_BITS:0] count; // 0 to 16

  // Loop counter declaration (moved outside for loop for Verilog-2001 compatibility)
  integer i;

  // Instantiate the FIFO module
  uart_fifo #(
    .DATA_WIDTH(DATA_WIDTH),
    .DEPTH_BITS(DEPTH_BITS)
  ) fifo_inst (
    .clk(clk),
    .rstn(rstn),
    .push(push),
    .pop(pop),
    .data_in(data_in),
    .data_out(data_out),
    .full(full),
    .empty(empty),
    .count(count)
  );

  // Clock generation
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk; // 100 MHz clock (10ns period)
  end

  // Test sequence
  initial begin
    // Initialize inputs
    rstn = 1'b0; // Assert reset
    push = 1'b0;
    pop = 1'b0;
    data_in = 'b0;

    $display("Time %0t: Initializing and asserting reset.", $time);
    #10; // Wait for a bit during reset

    rstn = 1'b1; // Release reset
    $display("Time %0t: Reset released. FIFO should be empty.", $time);
    #10;

    // Test 1: Push data into FIFO
    $display("\nTime %0t: --- Starting Push Test ---", $time);
    for (i = 0; i < FIFO_DEPTH; i = i + 1) begin // Changed 'int i' to 'i'
      @(posedge clk);
      push = 1'b1;
      data_in = i; // Push 0, 1, 2, ...
      $display("Time %0t: Pushing data 0x%h. Count: %0d, Full: %b, Empty: %b", $time, data_in, count, full, empty);
      #1; // Small delay to ensure push is registered before next cycle
      push = 1'b0; // De-assert push after one cycle
    end

    @(posedge clk);
    $display("Time %0t: Finished pushing. Current Count: %0d, Full: %b, Empty: %b", $time, count, full, empty);
    #10;

    // Test 2: Try to push when full
    $display("\nTime %0t: --- Testing Push when Full ---", $time);
    @(posedge clk);
    push = 1'b1;
    data_in = 8'hFF; // Try to push new data
    $display("Time %0t: Attempting to push 0x%h when full. Count: %0d, Full: %b, Empty: %b", $time, data_in, count, full, empty);
    #10;
    push = 1'b0;
    $display("Time %0t: After attempted push. Count: %0d, Full: %b, Empty: %b", $time, count, full, empty);
    // Data should not have changed, and full should still be 1.

    // Test 3: Pop data from FIFO
    $display("\nTime %0t: --- Starting Pop Test ---", $time);
    for (i = 0; i < FIFO_DEPTH; i = i + 1) begin // Changed 'int i' to 'i'
      @(posedge clk);
      pop = 1'b1;
      $display("Time %0t: Popping data. Data Out: 0x%h, Count: %0d, Full: %b, Empty: %b", $time, data_out, count, full, empty);
      #1; // Small delay to ensure pop is registered
      pop = 1'b0; // De-assert pop after one cycle
    end

    @(posedge clk);
    $display("Time %0t: Finished popping. Current Count: %0d, Full: %b, Empty: %b", $time, count, full, empty);
    #10;

    // Test 4: Try to pop when empty
    $display("\nTime %0t: --- Testing Pop when Empty ---", $time);
    @(posedge clk);
    pop = 1'b1;
    $display("Time %0t: Attempting to pop when empty. Count: %0d, Full: %b, Empty: %b", $time, count, full, empty);
    #10;
    pop = 1'b0;
    $display("Time %0t: After attempted pop. Count: %0d, Full: %b, Empty: %b", $time, count, full, empty);
    // Data out should be the last valid data, and empty should still be 1.

    // Test 5: Simultaneous push and pop
    $display("\nTime %0t: --- Testing Simultaneous Push/Pop ---", $time);
    // Push some data first
    for (i = 0; i < FIFO_DEPTH / 2; i = i + 1) begin // Changed 'int i' to 'i'
      @(posedge clk);
      push = 1'b1;
      data_in = i + 10;
      $display("Time %0t: Pushing data 0x%h. Count: %0d", $time, data_in, count);
      #1;
      push = 1'b0;
    end
    $display("Time %0t: FIFO partially filled. Count: %0d", $time, count);
    #10;

    @(posedge clk);
    push = 1'b1;
    pop = 1'b1;
    data_in = 8'hAA;
    $display("Time %0t: Simultaneous Push (0x%h) and Pop. Data Out: 0x%h, Count: %0d", $time, data_in, data_out, count);
    #1;
    push = 1'b0;
    pop = 1'b0;
    $display("Time %0t: After simultaneous. Count should be same: %0d", $time, count);
    #10;

    $display("\nTime %0t: Simulation finished.", $time);
    $finish; // End simulation
  end

endmodule
