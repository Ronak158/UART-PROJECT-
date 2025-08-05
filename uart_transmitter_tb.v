
`timescale 1ns/1ps

module uart_transmitter_tb;

  // Parameters for the UART Transmitter (must match the instantiated module)
  localparam DATA_WIDTH = 8;
  localparam CLK_FREQ = 100_000_000; // 100 MHz
  localparam BAUD_RATE = 9600;

  // Calculated baud period in clock cycles
  localparam BAUD_PERIOD_CYCLES = (CLK_FREQ / BAUD_RATE); // Approx 10416 for 9600 baud
  localparam BIT_TIME_NS = (1_000_000_000 / BAUD_RATE); // Time for one bit in ns (approx 104166 ns)

  // Testbench signals
  reg clk;
  reg rstn; // Active low reset
  reg [DATA_WIDTH-1:0] tx_data_in;
  reg tx_start; // Pulse to start transmission
  reg parity_en;
  reg two_stop_bits;
  reg [1:0] word_length;

  wire tx_out; // Serial data output
  wire tx_busy; // 1 when transmitting

  // Instantiate the UART Transmitter module
  uart_transmitter #(
    .DATA_WIDTH(DATA_WIDTH),
    .CLK_FREQ(CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
  ) uart_tx_inst (
    .clk(clk),
    .rstn(rstn),
    .data_in(tx_data_in),
    .start(tx_start),
    .parity_en(parity_en),
    .two_stop_bits(two_stop_bits),
    .word_length(word_length),
    .tx(tx_out),
    .tx_busy(tx_busy)
  );

  // Clock generation
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk; // 100 MHz clock (10ns period)
  end

  // Task to send a byte
  task send_byte;
    input [DATA_WIDTH-1:0] data;
    input p_en;
    input ts_bits;
    input [1:0] w_len;
    begin
      // Set configuration
      parity_en = p_en;
      two_stop_bits = ts_bits;
      word_length = w_len;
      tx_data_in = data;

      @(posedge clk);
      tx_start = 1'b1; // Assert start pulse
      $display("Time %0t: Starting transmission of 0x%h (Parity: %b, 2 Stop: %b, WordLen: %b)",
               $time, data, parity_en, two_stop_bits, word_length);
      @(posedge clk);
      tx_start = 1'b0; // De-assert start pulse

      // Wait for transmission to complete
      wait(!tx_busy);
      $display("Time %0t: Transmission of 0x%h completed.", $time, data);
      #100; // Small delay after transmission
    end
  endtask

  // Test sequence
  initial begin
    // Initialize inputs
    rstn = 1'b0; // Assert reset
    tx_start = 1'b0;
    tx_data_in = 'b0;
    parity_en = 1'b0;
    two_stop_bits = 1'b0;
    word_length = 2'b11; // Default to 8-bit

    $display("Time %0t: Initializing and asserting reset.", $time);
    #20; // Wait for a bit during reset

    rstn = 1'b1; // Release reset
    $display("Time %0t: Reset released. Transmitter should be idle.", $time);
    #20;

    // Test 1: Basic 8-bit transmission, no parity, 1 stop bit (0x55)
    $display("\n--- Test 1: 8-bit, No Parity, 1 Stop Bit (0x55) ---");
    send_byte(8'h55, 1'b0, 1'b0, 2'b11); // Data: 0101_0101
    // Expected sequence on tx_out: Start (0), 0, 1, 0, 1, 0, 1, 0, 1, Stop (1)

    // Test 2: 8-bit transmission, Even Parity, 1 Stop Bit (0x41 'A')
    // Data 0x41 (0100_0001). Number of set bits = 2 (even). Parity bit should be 0.
    $display("\n--- Test 2: 8-bit, Even Parity, 1 Stop Bit (0x41 'A') ---");
    send_byte(8'h41, 1'b1, 1'b0, 2'b11);
    // Expected sequence on tx_out: Start (0), 1, 0, 0, 0, 0, 1, 0, 0, Parity (0), Stop (1)

    // Test 3: 8-bit transmission, Even Parity, 1 Stop Bit (0x42 'B')
    // Data 0x42 (0100_0010). Number of set bits = 2 (even). Parity bit should be 0.
    $display("\n--- Test 3: 8-bit, Even Parity, 1 Stop Bit (0x42 'B') ---");
    send_byte(8'h42, 1'b1, 1'b0, 2'b11);
    // Expected sequence on tx_out: Start (0), 0, 1, 0, 0, 0, 0, 1, 0, Parity (0), Stop (1)

    // Test 4: 8-bit transmission, Even Parity, 1 Stop Bit (0x43 'C')
    // Data 0x43 (0100_0011). Number of set bits = 3 (odd). Parity bit should be 1.
    $display("\n--- Test 4: 8-bit, Even Parity, 1 Stop Bit (0x43 'C') ---");
    send_byte(8'h43, 1'b1, 1'b0, 2'b11);
    // Expected sequence on tx_out: Start (0), 1, 1, 0, 0, 0, 0, 1, 0, Parity (1), Stop (1)

    // Test 5: 8-bit transmission, no parity, 2 stop bits (0xAA)
    $display("\n--- Test 5: 8-bit, No Parity, 2 Stop Bits (0xAA) ---");
    send_byte(8'hAA, 1'b0, 1'b1, 2'b11);
    // Expected sequence on tx_out: Start (0), 0, 1, 0, 1, 0, 1, 0, 1, Stop1 (1), Stop2 (1)

    // Test 6: 5-bit word length, no parity, 1 stop bit (0x0F)
    $display("\n--- Test 6: 5-bit, No Parity, 1 Stop Bit (0x0F) ---");
    send_byte(8'h0F, 1'b0, 1'b0, 2'b00); // Data: 0_1111 (only lower 5 bits used)
    // Expected sequence on tx_out: Start (0), 1, 1, 1, 1, 0, Stop (1)

    // Test 7: 7-bit word length, no parity, 1 stop bit (0x7F)
    $display("\n--- Test 7: 7-bit, No Parity, 1 Stop Bit (0x7F) ---");
    send_byte(8'h7F, 1'b0, 1'b0, 2'b10); // Data: 111_1111 (only lower 7 bits used)
    // Expected sequence on tx_out: Start (0), 1, 1, 1, 1, 1, 1, 1, Stop (1)

    $display("\nTime %0t: Simulation finished.", $time);
    $finish; // End simulation
  end

endmodule
