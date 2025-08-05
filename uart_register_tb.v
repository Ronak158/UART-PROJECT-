
`timescale 1ns/1ps

module uart_registers_tb;

  // Parameters for the UART Registers (must match the instantiated module)
  localparam ADDR_WIDTH = 32;
  localparam DATA_WIDTH = 32;

  // Testbench signals
  reg clk;
  reg rstn; // Active low reset

  // APB Interface
  reg [ADDR_WIDTH-1:0] paddr;
  reg [DATA_WIDTH-1:0] pwdata;
  reg pwrite;
  reg psel;
  reg penable;
  wire [DATA_WIDTH-1:0] prdata;
  wire pready;
  wire pslverr;

  // Interface to TX FIFO (inputs to registers module)
  reg tx_fifo_full;
  reg tx_fifo_empty;

  // Interface to RX FIFO (inputs to registers module)
  reg [7:0] rx_fifo_data;
  reg rx_data_valid_from_fifo;

  // Interface to UART TX/RX modules for status (inputs to registers module)
  reg tx_busy_from_tx;
  reg rx_frame_error_from_rx;
  reg rx_parity_error_from_rx;

  // Outputs from registers module
  wire [7:0] tx_data_to_fifo;
  wire tx_fifo_push_en;
  wire rx_fifo_pop_en;
  wire parity_en_out;
  wire two_stop_bits_out;
  wire [1:0] word_length_out;
  wire irq;

  // Internal variables for checking
  reg [DATA_WIDTH-1:0] read_data_check;

  // Instantiate the UART Registers module
  uart_registers #(
    .ADDR_WIDTH(ADDR_WIDTH),
    .DATA_WIDTH(DATA_WIDTH)
  ) uart_registers_inst (
    .clk(clk),
    .rstn(rstn),
    .paddr(paddr),
    .pwdata(pwdata),
    .pwrite(pwrite),
    .psel(psel),
    .penable(penable),
    .prdata(prdata),
    .pready(pready),
    .pslverr(pslverr),

    .tx_data_to_fifo(tx_data_to_fifo),
    .tx_fifo_push_en(tx_fifo_push_en),
    .tx_fifo_full(tx_fifo_full),
    .tx_fifo_empty(tx_fifo_empty),

    .rx_fifo_data(rx_fifo_data),
    .rx_data_valid_from_fifo(rx_data_valid_from_fifo),
    .rx_fifo_pop_en(rx_fifo_pop_en),

    .tx_busy_from_tx(tx_busy_from_tx),
    .rx_frame_error_from_rx(rx_frame_error_from_rx),
    .rx_parity_error_from_rx(rx_parity_error_from_rx),

    .parity_en_out(parity_en_out),
    .two_stop_bits_out(two_stop_bits_out),
    .word_length_out(word_length_out),

    .irq(irq)
  );

  // Clock generation
  initial begin
    clk = 1'b0;
    forever #5 clk = ~clk; // 100 MHz clock (10ns period)
  end

  // APB Write Task
  task apb_write;
    input [ADDR_WIDTH-1:0] addr;
    input [DATA_WIDTH-1:0] data;
    begin
      @(negedge clk);
      psel = 1'b1;
      pwrite = 1'b1;
      paddr = addr;
      pwdata = data;
      penable = 1'b1;
      wait(pready); // Wait for pready to go high
      @(negedge clk); // Wait for the end of the transfer
      psel = 1'b0;
      pwrite = 1'b0;
      penable = 1'b0;
      paddr = 'b0;
      pwdata = 'b0;
      #10; // Small delay to ensure signals settle
    end
  endtask

  // APB Read Task
  task apb_read;
    input [ADDR_WIDTH-1:0] addr;
    output [DATA_WIDTH-1:0] read_data;
    begin
      @(negedge clk);
      psel = 1'b1;
      pwrite = 1'b0;
      paddr = addr;
      penable = 1'b1;
      wait(pready); // Wait for pready to go high
      @(negedge clk); // Wait for the end of the transfer
      read_data = prdata;
      psel = 1'b0;
      pwrite = 1'b0;
      penable = 1'b0;
      paddr = 'b0;
      #10; // Small delay to ensure signals settle
    end
  endtask

  // Test sequence
  initial begin
    // Initialize inputs
    rstn = 1'b0; // Assert reset
    paddr = 'b0;
    pwdata = 'b0;
    pwrite = 1'b0;
    psel = 1'b0;
    penable = 1'b0;

    tx_fifo_full = 1'b0;
    tx_fifo_empty = 1'b1; // Initially empty
    rx_fifo_data = 'b0;
    rx_data_valid_from_fifo = 1'b0;
    tx_busy_from_tx = 1'b0;
    rx_frame_error_from_rx = 1'b0;
    rx_parity_error_from_rx = 1'b0;

    $display("Time %0t: Initializing and asserting reset.", $time);
    #20; // Wait for a bit during reset

    rstn = 1'b1; // Release reset
    $display("Time %0t: Reset released. Registers should be in default state.", $time);
    #20;

    // Test 1: Write to LCR (Line Control Register) - Address 0x0C
    // LCR value: 8-bit word (11), Parity Enable (1), 2 Stop Bits (1) -> 00001111 (0xF)
    $display("\n--- Test 1: Write LCR (0x0C) with 0x0F (8-bit, Parity, 2 Stop) ---");
    apb_write(32'h0C, 32'h0F);
    $display("Time %0t: LCR written. parity_en_out: %b, two_stop_bits_out: %b, word_length_out: %b",
             $time, parity_en_out, two_stop_bits_out, word_length_out);
    if (parity_en_out == 1'b1 && two_stop_bits_out == 1'b1 && word_length_out == 2'b11) $display("Test 1 PASSED."); else $display("Test 1 FAILED.");
    #20;

    // Test 2: Read LCR (0x0C)
    $display("\n--- Test 2: Read LCR (0x0C) ---");
    apb_read(32'h0C, read_data_check);
    $display("Time %0t: Read LCR: 0x%h (Expected 0x0F)", $time, read_data_check);
    if (read_data_check[7:0] == 8'h0F) $display("Test 2 PASSED."); else $display("Test 2 FAILED.");
    #20;

    // Test 3: Write to THR (Transmitter Holding Register) - Address 0x00
    // Simulate TX FIFO being empty and not full
    tx_fifo_empty = 1'b1;
    tx_fifo_full = 1'b0;
    $display("\n--- Test 3: Write THR (0x00) with 0xAA (TX FIFO empty) ---");
    apb_write(32'h00, 32'hAA);
    $display("Time %0t: THR written. tx_data_to_fifo: 0x%h, tx_fifo_push_en: %b",
             $time, tx_data_to_fifo, tx_fifo_push_en);
    if (tx_data_to_fifo == 8'hAA && tx_fifo_push_en == 1'b1 && !pslverr) $display("Test 3 PASSED."); else $display("Test 3 FAILED.");
    #20;

    // Test 4: Write to THR when TX FIFO is full (should cause pslverr)
    tx_fifo_full = 1'b1;
    tx_fifo_empty = 1'b0; // Assume not empty if full
    $display("\n--- Test 4: Write THR (0x00) with 0xBB (TX FIFO full) ---");
    apb_write(32'h00, 32'hBB);
    $display("Time %0t: THR written. pslverr: %b", $time, pslverr);
    if (pslverr == 1'b1) $display("Test 4 PASSED."); else $display("Test 4 FAILED.");
    tx_fifo_full = 1'b0; // Reset for next tests
    #20;

    // Test 5: Simulate RX data arrival and read RBR (Receiver Buffer Register) - Address 0x00
    // Simulate data 0xCC arriving from RX FIFO
    rx_fifo_data = 8'hCC;
    rx_data_valid_from_fifo = 1'b1;
    $display("\n--- Test 5: Simulate RX data 0xCC arrival ---");
    @(posedge clk); // Allow data to be latched into RBR
    rx_data_valid_from_fifo = 1'b0; // Pulse the valid signal
    $display("Time %0t: After RX data arrival. IRQ: %b", $time, irq);
    #20;

    $display("\n--- Test 5 (cont.): Read RBR (0x00) ---");
    apb_read(32'h00, read_data_check);
    $display("Time %0t: Read RBR: 0x%h. rx_fifo_pop_en: %b, IRQ: %b",
             $time, read_data_check, rx_fifo_pop_en, irq);
    if (read_data_check[7:0] == 8'hCC && rx_fifo_pop_en == 1'b1 && !irq) $display("Test 5 PASSED."); else $display("Test 5 FAILED.");
    #20;

    // Test 6: Check LSR (Line Status Register) - Address 0x14
    // Simulate TX busy, RX errors
    tx_fifo_empty = 1'b0; // Not empty
    tx_busy_from_tx = 1'b1; // TX busy
    rx_frame_error_from_rx = 1'b1; // Frame error
    rx_parity_error_from_rx = 1'b0; // No parity error
    $display("\n--- Test 6: Read LSR (0x14) with TX busy, Frame Error ---");
    apb_read(32'h14, read_data_check);
    $display("Time %0t: Read LSR: 0x%h", $time, read_data_check);
    // Expected: Bit 6 (TX Empty) = 0, Bit 5 (THR Empty) = 0, Bit 4 (Frame Error) = 1, Bit 3 (Parity Error) = 0, Bit 0 (Data Ready) = 0
    if (read_data_check[6] == 1'b0 && read_data_check[5] == 1'b0 && read_data_check[4] == 1'b1 && read_data_check[3] == 1'b0 && read_data_check[0] == 1'b0) $display("Test 6 PASSED."); else $display("Test 6 FAILED.");
    #20;

    // Test 7: Check LSR after read (error flags should clear)
    $display("\n--- Test 7: Read LSR (0x14) again (error flags should clear) ---");
    apb_read(32'h14, read_data_check);
    $display("Time %0t: Read LSR: 0x%h", $time, read_data_check);
    // Expected: Bit 4 (Frame Error) = 0
    if (read_data_check[4] == 1'b0) $display("Test 7 PASSED."); else $display("Test 7 FAILED.");
    #20;

    // Test 8: Simulate TX FIFO empty and check IRQ
    tx_fifo_empty = 1'b1;
    tx_busy_from_tx = 1'b0; // TX not busy
    rx_frame_error_from_rx = 1'b0;
    rx_parity_error_from_rx = 1'b0;
    rx_data_valid_from_fifo = 1'b0;
    $display("\n--- Test 8: Simulate TX FIFO empty and check IRQ ---");
    @(posedge clk);
    $display("Time %0t: IRQ: %b", $time, irq);
    if (irq == 1'b1) $display("Test 8 PASSED."); else $display("Test 8 FAILED.");
    #20;

    $display("\nTime %0t: Simulation finished.", $time);
    $finish; // End simulation
  end

endmodule
