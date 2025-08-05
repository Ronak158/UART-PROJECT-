`timescale 1ns/1ps

module uart_receiver_tb;

  // Parameters for the UART Receiver (must match the instantiated module)
  localparam DATA_WIDTH = 8;
  localparam CLK_FREQ = 100_000_000; // 100 MHz
  localparam BAUD_RATE = 9600;

  // Calculated timing for testbench
  localparam CLK_PERIOD_NS = 1_000_000_000 / CLK_FREQ; // 10ns for 100MHz
  localparam BIT_PERIOD_NS = 1_000_000_000 / BAUD_RATE; // Approx 104166 ns for 9600 baud
  localparam HALF_BIT_PERIOD_NS = BIT_PERIOD_NS / 2;

  // Testbench signals
  reg clk;
  reg rstn; // Active low reset
  reg rx; // Serial input line
  reg parity_en_tb; // Renamed to avoid conflict with module input
  reg two_stop_bits_tb; // Renamed
  reg [1:0] word_length_tb; // Renamed

  wire [DATA_WIDTH-1:0] data_out;
  wire data_valid;
  wire parity_error;
  wire frame_error;

  // Loop counter declaration (moved outside for loop for Verilog-2001 compatibility)
  integer i;

  // Instantiate the UART Receiver module
  uart_receiver #(
    .DATA_WIDTH(DATA_WIDTH),
    .CLK_FREQ(CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) uart_rx_inst (
    .clk(clk),
    .rstn(rstn),
    .rx(rx),
    .parity_en(parity_en_tb),
    .two_stop_bits(two_stop_bits_tb),
    .word_length(word_length_tb),
    .data_out(data_out),
    .data_valid(data_valid),
    .parity_error(parity_error),
    .frame_error(frame_error)
  );

  // Clock generation
  initial begin
    clk = 1'b0;
    forever #(CLK_PERIOD_NS / 2) clk = ~clk; // 100 MHz clock (10ns period)
  end

  // Task to send a serial byte
  task send_serial_byte;
    input [DATA_WIDTH-1:0] data_to_send;
    input parity_enable;
    input two_stop_bits_enable;
    input [1:0] w_len;
    input force_parity_error; // 1 to force parity error
    input force_frame_error;  // 1 to force frame error (stop bit low)
    reg [DATA_WIDTH-1:0] temp_data;
    reg calculated_parity;
    reg [2:0] actual_data_bits_val;

    begin
      // Set receiver configuration
      parity_en_tb = parity_enable;
      two_stop_bits_tb = two_stop_bits_enable;
      word_length_tb = w_len;

      // Determine actual data bits for transmission
      case (w_len)
        2'b00: actual_data_bits_val = 3'd5; // 5-bit
        2'b01: actual_data_bits_val = 3'd6; // 6-bit
        2'b10: actual_data_bits_val = 3'd7; // 7-bit
        default: actual_data_bits_val = 3'd8; // 8-bit
      endcase

      // Calculate parity for the data to be sent
      temp_data = data_to_send;
      calculated_parity = 1'b0;
      for (i = 0; i < actual_data_bits_val; i = i + 1) begin
        calculated_parity = calculated_parity ^ temp_data[i];
      end

      $display("Time %0t: send_serial_byte: Starting transmission of 0x%h (Parity: %b, 2 Stop: %b, WordLen: %b, Calc Parity: %b)",
               $time, data_to_send, parity_enable, two_stop_bits_enable, w_len, calculated_parity);

      // Start bit
      rx = 1'b0; // Start bit (0)
      $display("Time %0t: send_serial_byte: Sending Start Bit (0).", $time);
      #(BIT_PERIOD_NS);

      // Data bits (LSB first)
      temp_data = data_to_send;
      for (i = 0; i < actual_data_bits_val; i = i + 1) begin
        rx = temp_data[0];
        $display("Time %0t: send_serial_byte: Sending Data Bit %0d (%b).", $time, i, rx);
        temp_data = temp_data >> 1;
        #(BIT_PERIOD_NS);
      end

      // Parity bit (if enabled)
      if (parity_enable) begin
        if (force_parity_error) begin
          rx = ~calculated_parity; // Force incorrect parity
          $display("Time %0t: send_serial_byte: Forcing parity error. Sending %b (expected %b)", $time, rx, calculated_parity);
        end else begin
          rx = calculated_parity;
          $display("Time %0t: send_serial_byte: Sending Parity Bit (%b).", $time, rx);
        end
        #(BIT_PERIOD_NS);
      end

      // Stop bit(s)
      if (force_frame_error) begin
        rx = 1'b0; // Force framing error (stop bit low)
        $display("Time %0t: send_serial_byte: Forcing framing error. Sending 0 for stop bit.", $time);
      end else begin
        rx = 1'b1; // First stop bit (1)
        $display("Time %0t: send_serial_byte: Sending Stop1 Bit (1).", $time);
      end
      #(BIT_PERIOD_NS);

      if (two_stop_bits_enable) begin
        rx = 1'b1; // Second stop bit (1)
        $display("Time %0t: send_serial_byte: Sending Stop2 Bit (1).", $time);
        #(BIT_PERIOD_NS);
      end

      rx = 1'b1; // Return to idle high
      $display("Time %0t: send_serial_byte: Finished sending byte. Returning to idle.", $time);
      #50; // Small delay after transmission to allow receiver to settle
    end
  endtask

  // Test sequence
  initial begin
    // Initialize inputs
    rstn = 1'b0; // Assert reset
    rx = 1'b1; // Idle high
    parity_en_tb = 1'b0;
    two_stop_bits_tb = 1'b0;
    word_length_tb = 2'b11; // Default to 8-bit

    $display("Time %0t: Initializing and asserting reset.", $time);
    #20; // Wait for a bit during reset

    rstn = 1'b1; // Release reset
    $display("Time %0t: Reset released. Receiver should be idle.", $time);
    #20;

    // Test 1: Basic 8-bit reception, no parity, 1 stop bit (0x55)
    $display("\n--- Test 1: 8-bit, No Parity, 1 Stop Bit (0x55) ---");
    send_serial_byte(8'h55, 1'b0, 1'b0, 2'b11, 1'b0, 1'b0);
    $display("Time %0t: Test 1: Waiting for data_valid...", $time);
    @(posedge data_valid);
    $display("Time %0t: Test 1: Received data: 0x%h. Data Valid: %b, Parity Error: %b, Frame Error: %b",
             $time, data_out, data_valid, parity_error, frame_error);
    if (data_out == 8'h55 && data_valid && !parity_error && !frame_error) $display("Test 1 PASSED."); else $display("Test 1 FAILED.");
    #50;

    // Test 2: 8-bit reception, Even Parity, 1 Stop Bit (0x41 'A') - Correct Parity
    $display("\n--- Test 2: 8-bit, Even Parity, 1 Stop Bit (0x41 'A') - Correct Parity ---");
    send_serial_byte(8'h41, 1'b1, 1'b0, 2'b11, 1'b0, 1'b0);
    $display("Time %0t: Test 2: Waiting for data_valid...", $time);
    @(posedge data_valid);
    $display("Time %0t: Test 2: Received data: 0x%h. Data Valid: %b, Parity Error: %b, Frame Error: %b",
             $time, data_out, data_valid, parity_error, frame_error);
    if (data_out == 8'h41 && data_valid && !parity_error && !frame_error) $display("Test 2 PASSED."); else $display("Test 2 FAILED.");
    #50;

    // Test 3: 8-bit reception, Even Parity, 1 Stop Bit (0x43 'C') - Incorrect Parity
    $display("\n--- Test 3: 8-bit, Even Parity, 1 Stop Bit (0x43 'C') - Incorrect Parity ---");
    send_serial_byte(8'h43, 1'b1, 1'b0, 2'b11, 1'b1, 1'b0); // Force parity error
    $display("Time %0t: Test 3: Waiting for data_valid...", $time);
    @(posedge data_valid);
    $display("Time %0t: Test 3: Received data: 0x%h. Data Valid: %b, Parity Error: %b, Frame Error: %b",
             $time, data_out, data_valid, parity_error, frame_error);
    if (data_out == 8'h43 && data_valid && parity_error && !frame_error) $display("Test 3 PASSED."); else $display("Test 3 FAILED.");
    #50;

    // Test 4: 8-bit reception, No Parity, 2 Stop Bits (0xAA)
    $display("\n--- Test 4: 8-bit, No Parity, 2 Stop Bits (0xAA) ---");
    send_serial_byte(8'hAA, 1'b0, 1'b1, 2'b11, 1'b0, 1'b0);
    $display("Time %0t: Test 4: Waiting for data_valid...", $time);
    @(posedge data_valid);
    $display("Time %0t: Test 4: Received data: 0x%h. Data Valid: %b, Parity Error: %b, Frame Error: %b",
             $time, data_out, data_valid, parity_error, frame_error);
    if (data_out == 8'hAA && data_valid && !parity_error && !frame_error) $display("Test 4 PASSED."); else $display("Test 4 FAILED.");
    #50;

    // Test 5: 5-bit word length, no parity, 1 stop bit (0x0F)
    $display("\n--- Test 5: 5-bit, No Parity, 1 Stop Bit (0x0F) ---");
    send_serial_byte(8'h0F, 1'b0, 1'b0, 2'b00, 1'b0, 1'b0);
    $display("Time %0t: Test 5: Waiting for data_valid...", $time);
    @(posedge data_valid);
    $display("Time %0t: Test 5: Received data: 0x%h. Data Valid: %b, Parity Error: %b, Frame Error: %b",
             $time, data_out, data_valid, parity_error, frame_error);
    // For 5-bit, data_out will be 8'h0F, but only lower 5 bits are relevant.
    if ((data_out & 8'h1F) == 8'h0F && data_valid && !parity_error && !frame_error) $display("Test 5 PASSED."); else $display("Test 5 FAILED.");
    #50;

    // Test 6: Framing Error (Stop bit is 0 instead of 1)
    $display("\n--- Test 6: Framing Error (Stop bit is 0) ---");
    send_serial_byte(8'h12, 1'b0, 1'b0, 2'b11, 1'b0, 1'b1); // Force frame error
    $display("Time %0t: Test 6: Waiting for expected error flags...", $time);
    // No data_valid expected, but frame_error should be asserted.
    # (BIT_PERIOD_NS * 12); // Wait for the expected end of transmission + some margin
    $display("Time %0t: Test 6: After framing error test. Data Valid: %b, Parity Error: %b, Frame Error: %b",
             $time, data_valid, parity_error, frame_error);
    if (!data_valid && !parity_error && frame_error) $display("Test 6 PASSED."); else $display("Test 6 FAILED.");
    #50;

    $display("\nTime %0t: Simulation finished.", $time);
    $finish; // End simulation
  end

endmodule
