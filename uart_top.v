`timescale 1ns/1ps

module uart_top #(
  parameter TOP_DATA_WIDTH = 8, // Renamed parameter
  parameter TOP_FIFO_DEPTH_BITS = 4, // Renamed parameter
  parameter TOP_CLK_FREQ = 100_000_000, // Renamed parameter
  parameter TOP_BAUD_RATE = 9600 // Renamed parameter
) (
  // APB Interface (from CPU)
  input wire pclk,
  input wire presetn, // Active low reset
  input wire [31:0] paddr,
  input wire [31:0] pwdata,
  input wire pwrite,
  input wire penable,
  input wire psel,
  output wire [31:0] prdata,
  output wire pready,
  output wire pslverr,

  // UART Serial Interface
  input wire rx_input, // Incoming serial data
  output wire tx_output, // UART Transmitter Output

  // Interrupt Output
  output wire irq,

  // Debug/Testbench Output (optional)
  output wire baud_o // Simplified baud rate output (e.g., pclk for observation)
);

  // Internal Wires for connections between sub-modules

  // --- Connections to/from Registers Module ---
  wire [7:0] tx_data_to_registers; // Data from registers (THR) to TX FIFO
  wire tx_fifo_push_en_from_registers; // Push enable from registers (on THR write)
  wire [7:0] rx_fifo_data_to_registers; // Data from RX FIFO to registers (RBR)
  wire rx_fifo_pop_en_from_registers; // Pop enable from registers (on RBR read)

  wire parity_en_from_lcr;       // Parity enable from LCR (to TX/RX FSMs)
  wire two_stop_bits_from_lcr;   // Two stop bits from LCR (to TX/RX FSMs)
  wire [1:0] word_length_from_lcr; // Word length from LCR (to TX/RX FSMs)

  // --- Connections to/from TX FIFO ---
  wire [TOP_DATA_WIDTH-1:0] tx_fifo_data_out; // Data from TX FIFO to Transmitter FSM
  wire tx_fifo_full; // TX FIFO full status
  wire tx_fifo_empty; // TX FIFO empty status

  // --- Connections to/from RX FIFO ---
  wire [TOP_DATA_WIDTH-1:0] rx_fifo_data_in; // Data from Receiver FSM to RX FIFO
  wire rx_fifo_full; // RX FIFO full status
  wire rx_fifo_empty; // RX FIFO empty status

  // --- Connections to/from Transmitter FSM ---
  wire tx_start_to_tx_module; // Start signal to Transmitter FSM
  wire tx_busy_from_tx_module; // Busy status from Transmitter FSM

  // --- Connections to/from Receiver FSM ---
  wire rx_data_valid_from_rx_module; // Data valid pulse from Receiver FSM
  wire rx_frame_error_from_rx_module; // Frame error from Receiver FSM
  wire rx_parity_error_from_rx_module; // Parity error from Receiver FSM

  // --- Internal Control Signals ---
  wire tx_fifo_write_en; // Actual write enable for TX FIFO
  wire tx_fifo_read_en;  // Actual read enable for TX FIFO
  wire rx_fifo_write_en; // Actual write enable for RX FIFO
  wire rx_fifo_read_en;  // Actual read enable for RX FIFO

  // Instantiate UART Registers Block
  uart_registers #(
    .ADDR_WIDTH(32),
    .DATA_WIDTH(32)
  ) uart_registers_inst (
    .clk(pclk),
    .rstn(presetn),
    .paddr(paddr),
    .pwdata(pwdata),
    .pwrite(pwrite),
    .psel(psel),
    .penable(penable),
    .prdata(prdata),
    .pready(pready),
    .pslverr(pslverr),

    // Interface to TX FIFO
    .tx_data_to_fifo(tx_data_to_registers), // Data from THR write
    .tx_fifo_push_en(tx_fifo_push_en_from_registers), // Push request from THR write
    .tx_fifo_full(tx_fifo_full), // TX FIFO status to registers
    .tx_fifo_empty(tx_fifo_empty), // TX FIFO status to registers

    // Interface to RX FIFO
    .rx_fifo_data(rx_fifo_data_to_registers), // Data from RX FIFO to RBR
    .rx_data_valid_from_fifo(rx_data_valid_from_rx_module), // Valid signal from RX FSM
    .rx_fifo_pop_en(rx_fifo_pop_en_from_registers), // Pop request from RBR read

    // Interface to UART TX/RX modules for status
    .tx_busy_from_tx(tx_busy_from_tx_module), // TX FSM busy status
    .rx_frame_error_from_rx(rx_frame_error_from_rx_module), // RX FSM frame error
    .rx_parity_error_from_rx(rx_parity_error_from_rx_module), // RX FSM parity error

    // Output for LCR configuration to TX/RX modules
    .parity_en_out(parity_en_from_lcr),
    .two_stop_bits_out(two_stop_bits_from_lcr),
    .word_length_out(word_length_from_lcr),

    // Interrupt output
    .irq(irq)
  );

  // Instantiate TX FIFO
  uart_fifo #(
    .DATA_WIDTH(TOP_DATA_WIDTH), // Pass top-level parameter
    .DEPTH_BITS(TOP_FIFO_DEPTH_BITS) // Pass top-level parameter
  ) tx_fifo_inst (
    .clk(pclk),
    .rstn(presetn),
    .push(tx_fifo_write_en), // Controlled by registers and FIFO full status
    .pop(tx_fifo_read_en),   // Controlled by TX FSM readiness and FIFO empty status
    .data_in(tx_data_to_registers), // Data from registers (THR)
    .data_out(tx_fifo_data_out), // Data to TX FSM
    .full(tx_fifo_full),
    .empty(tx_fifo_empty),
    .count() // Not used in this top module
  );

  // Instantiate RX FIFO
  uart_fifo #(
    .DATA_WIDTH(TOP_DATA_WIDTH), // Pass top-level parameter
    .DEPTH_BITS(TOP_FIFO_DEPTH_BITS) // Pass top-level parameter
  ) rx_fifo_inst (
    .clk(pclk),
    .rstn(presetn),
    .push(rx_fifo_write_en), // Controlled by RX FSM data valid and FIFO full status
    .pop(rx_fifo_read_en),   // Controlled by registers (RBR read) and FIFO empty status
    .data_in(rx_fifo_data_in), // Data from RX FSM
    .data_out(rx_fifo_data_to_registers), // Data to registers (RBR)
    .full(rx_fifo_full),
    .empty(rx_fifo_empty),
    .count() // Not used in this top module
  );

  // Instantiate UART Transmitter FSM
  uart_transmitter #(
    .DATA_WIDTH(TOP_DATA_WIDTH), // Pass top-level parameter
    .CLK_FREQ(TOP_CLK_FREQ),     // Pass top-level parameter
    .BAUD_RATE(TOP_BAUD_RATE)    // Pass top-level parameter
  ) uart_tx_inst (
    .clk(pclk),
    .rstn(presetn),
    .data_in(tx_fifo_data_out), // Data from TX FIFO
    .start(tx_start_to_tx_module), // Start signal to TX FSM
    .parity_en(parity_en_from_lcr),
    .two_stop_bits(two_stop_bits_from_lcr),
    .word_length(word_length_from_lcr),
    .tx(tx_output), // Serial data output
    .tx_busy(tx_busy_from_tx_module) // TX FSM busy status
  );

  // Instantiate UART Receiver FSM
  uart_receiver #(
    .DATA_WIDTH(TOP_DATA_WIDTH), // Pass top-level parameter
    .CLK_FREQ(TOP_CLK_FREQ),     // Pass top-level parameter
    .BAUD_RATE(TOP_BAUD_RATE)    // Pass top-level parameter
  ) uart_rx_inst (
    .clk(pclk),
    .rstn(presetn),
    .rx(rx_input), // Serial data input
    .parity_en(parity_en_from_lcr),
    .two_stop_bits(two_stop_bits_from_lcr),
    .word_length(word_length_from_lcr),
    .data_out(rx_fifo_data_in), // Data from RX FSM to RX FIFO
    .data_valid(rx_data_valid_from_rx_module), // Data valid pulse from RX FSM
    .parity_error(rx_parity_error_from_rx_module), // Parity error from RX FSM
    .frame_error(rx_frame_error_from_rx_module) // Frame error from RX FSM
  );

  // Control Logic for FIFOs and TX FSM
  // TX FIFO Write Enable: Registers request push AND TX FIFO is not full
  assign tx_fifo_write_en = tx_fifo_push_en_from_registers && !tx_fifo_full;

  // TX FSM Start Signal: Transmitter is idle AND TX FIFO is not empty
  // This signal also triggers the pop from TX FIFO.
  assign tx_start_to_tx_module = !tx_busy_from_tx_module && !tx_fifo_empty;

  // TX FIFO Read Enable: Asserted when TX FSM is ready for new data (i.e., tx_start_to_tx_module is high)
  assign tx_fifo_read_en = tx_start_to_tx_module;

  // RX FIFO Write Enable: RX FSM has valid data AND RX FIFO is not full
  assign rx_fifo_write_en = rx_data_valid_from_rx_module && !rx_fifo_full;

  // RX FIFO Read Enable: Registers request pop (RBR read) AND RX FIFO is not empty
  assign rx_fifo_read_en = rx_fifo_pop_en_from_registers && !rx_fifo_empty;

  // Simplified baud_o output (for testbench observation)
  assign baud_o = pclk; // Placeholder, a real baud_o would be derived from a baud rate generator

endmodule
