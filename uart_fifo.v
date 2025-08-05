`timescale 1ns/1ps

module uart_fifo #(
  parameter DATA_WIDTH = 8, // Parameter definition
  parameter DEPTH_BITS = 4 // Parameter definition
) (
  input wire clk,
  input wire rstn, // Active low reset
  input wire push, // Write control
  input wire pop,  // Read control
  input wire [7:0] data_in,
  output wire [7:0] data_out,
  output wire full,
  output wire empty,
  output wire [4:0] count // Number of elements in FIFO (0 to 16)
);

  // Microarchitecture components
  reg [7:0] mem [0:15]; // 16 x 8-bit memory
  reg [3:0] write_pointer; // 4-bit pointer for 16 entries (0-15)
  reg [3:0] read_pointer;  // 4-bit pointer for 16 entries (0-15)
  reg [4:0] current_count; // 5-bit count register (0 to 16)

  // Output assignments
  assign empty = (current_count == 5'd0);
  assign full = (current_count == 5'd16);
  assign data_out = mem[read_pointer]; // Data out is always from the read pointer

  // Synchronous operation
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin // Asynchronous reset (active low)
      write_pointer <= 4'd0;
      read_pointer <= 4'd0;
      current_count <= 5'd0;
      // Memory contents are undefined after reset, but pointers and count are reset.
    end else begin
      // Default assignments to prevent latches
      // No default assignments needed here as logic covers all cases.

      // Handle write operation
      if (push && !full) begin
        mem[write_pointer] <= data_in;
        write_pointer <= write_pointer + 1;
        current_count <= current_count + 1;
      end

      // Handle read operation
      if (pop && !empty) begin
        read_pointer <= read_pointer + 1;
        current_count <= current_count - 1;
      end

      // Handle simultaneous push and pop
      // If both push and pop occur in the same cycle, the count remains unchanged.
      // The pointer updates ensure correct data flow.
      if (push && !full && pop && !empty) begin
        current_count <= current_count; // Count remains the same
      end
    end
  end

endmodule
