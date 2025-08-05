`timescale 1ns/1ps

module uart_registers #(
  parameter ADDR_WIDTH = 32, // Parameter definition
  parameter DATA_WIDTH = 32 // Parameter definition
) (
  input wire clk,
  input wire rstn, // Active low reset

  // APB Interface
  input wire [ADDR_WIDTH-1:0] paddr,
  input wire [DATA_WIDTH-1:0] pwdata,
  input wire pwrite,
  input wire psel,
  input wire penable,
  output reg [DATA_WIDTH-1:0] prdata,
  output reg pready,
  output reg pslverr,

  // Interface to TX FIFO
  output reg [7:0] tx_data_to_fifo, // Data to TX FIFO
  output reg tx_fifo_push_en, // Enable push to TX FIFO
  input wire tx_fifo_full, // From TX FIFO
  input wire tx_fifo_empty, // From TX FIFO

  // Interface to RX FIFO
  input wire [7:0] rx_fifo_data, // Data from RX FIFO
  input wire rx_data_valid_from_fifo, // Valid signal from RX FIFO
  output reg rx_fifo_pop_en, // Enable pop from RX FIFO

  // Interface to UART TX/RX modules for status
  input wire tx_busy_from_tx, // From TX module
  input wire rx_frame_error_from_rx, // From RX module
  input wire rx_parity_error_from_rx, // From RX module

  // Output for LCR configuration to TX/RX modules
  output wire parity_en_out,
  output wire two_stop_bits_out,
  output wire [1:0] word_length_out,

  // Interrupt output (simplified)
  output reg irq
);

  // Internal Registers
  reg [7:0] thr; // Transmitter Holding Register (Write-only)
  reg [7:0] rbr; // Receiver Buffer Register (Read-only)
  reg [7:0] lcr; // Line Control Register
  reg [7:0] lsr; // Line Status Register (Read-only)

  // Internal flags for LSR bits
  reg lsr_data_ready;
  reg lsr_overrun_error;
  reg lsr_parity_error;
  reg lsr_frame_error;
  reg lsr_thr_empty; // This is tx_fifo_empty
  reg lsr_tx_empty; // This is tx_fifo_empty && !tx_busy_from_tx

  // LCR outputs based on the new specification
  assign word_length_out = lcr[1:0]; // Bits 1:0 for word length
  assign parity_en_out = lcr[2];     // Bit 2 for parity enable
  assign two_stop_bits_out = lcr[3]; // Bit 3 for two stop bits

  // Simplified Interrupt Logic (can be expanded with IER/IIR)
  reg rx_data_irq_pending;
  reg tx_empty_irq_pending;

  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      pready <= 1'b0;
      pslverr <= 1'b0;
      prdata <= 'b0;
      thr <= 'b0;
      rbr <= 'b0;
      lcr <= 'b0; // Reset value for LCR
      lsr <= 'b0;
      tx_data_to_fifo <= 'b0;
      tx_fifo_push_en <= 1'b0;
      rx_fifo_pop_en <= 1'b0;
      lsr_data_ready <= 1'b0;
      lsr_overrun_error <= 1'b0;
      lsr_parity_error <= 1'b0;
      lsr_frame_error <= 1'b0;
      lsr_thr_empty <= 1'b1; // Initially empty
      lsr_tx_empty <= 1'b1; // Initially empty
      irq <= 1'b0;
      rx_data_irq_pending <= 1'b0;
      tx_empty_irq_pending <= 1'b0;
    end else begin
      // Default assignments for outputs
      pready <= 1'b0;
      pslverr <= 1'b0;
      tx_fifo_push_en <= 1'b0;
      rx_fifo_pop_en <= 1'b0;
      irq <= 1'b0; // Default to no interrupt

      // APB Write Cycle
      if (psel && penable && pwrite) begin
        pready <= 1'b1;
        case (paddr)
          32'h00: begin // THR (Transmitter Holding Register)
            if (!tx_fifo_full) begin // Only write if TX FIFO is not full
              thr <= pwdata[7:0];
              tx_data_to_fifo <= pwdata[7:0];
              tx_fifo_push_en <= 1'b1; // Request push to TX FIFO
            end else begin
              pslverr <= 1'b1; // Indicate error if FIFO is full
            end
          end
          32'h0C: lcr <= pwdata[7:0]; // Line Control Register
          // Add other writeable registers here (e.g., IER, FCR if needed)
          default: pslverr <= 1'b1; // Invalid address
        endcase
      end
      // APB Read Cycle
      else if (psel && penable && !pwrite) begin
        pready <= 1'b1;
        case (paddr)
          32'h00: begin // RBR (Receiver Buffer Register)
            prdata <= {24'b0, rbr};
            rx_fifo_pop_en <= 1'b1; // Request pop from RX FIFO
            lsr_data_ready <= 1'b0; // Clear Data Ready on read
            rx_data_irq_pending <= 1'b0; // Clear RX Data IRQ pending
          end
          32'h0C: prdata <= {24'b0, lcr}; // Line Control Register
          32'h14: begin // LSR (Line Status Register)
            prdata <= {24'b0, lsr};
            // Clear error flags on read
            lsr_overrun_error <= 1'b0;
            lsr_parity_error <= 1'b0;
            lsr_frame_error <= 1'b0;
            tx_empty_irq_pending <= 1'b0; // Clear TX Empty IRQ pending
          end
          // Add other readable registers here (e.g., IIR, MSR)
          default: pslverr <= 1'b1; // Invalid address
        endcase
      end

      // Update RBR from RX FIFO
      if (rx_data_valid_from_fifo) begin
        rbr <= rx_fifo_data;
        lsr_data_ready <= 1'b1; // Set Data Ready flag
        rx_data_irq_pending <= 1'b1; // Set RX Data IRQ pending
      end

      // Update LSR (Line Status Register) bits
      lsr_thr_empty <= tx_fifo_empty; // Bit 5: Transmitter Holding Register Empty
      lsr_tx_empty <= tx_fifo_empty && !tx_busy_from_tx; // Bit 6: Transmitter Empty

      // Error flags from RX module
      if (rx_frame_error_from_rx) lsr_frame_error <= 1'b1;
      if (rx_parity_error_from_rx) lsr_parity_error <= 1'b1;

      // Assemble LSR
      lsr = {
        1'b0, // Bit 7: FIFO Error (simplified, can be more complex)
        lsr_tx_empty, // Bit 6: Transmitter Empty
        lsr_thr_empty, // Bit 5: Transmitter Holding Register Empty
        lsr_frame_error, // Bit 4: Framing Error
        lsr_parity_error, // Bit 3: Parity Error
        lsr_overrun_error, // Bit 2: Overrun Error
        lsr_data_ready // Bit 0: Data Ready
      };

      // Simplified IRQ output (can be expanded with IER/IIR)
      // For example, if IER[0] enables RX Data Ready interrupt
      if (lsr_data_ready) irq <= 1'b1; // Example: IRQ on data ready
      else if (lsr_thr_empty) irq <= 1'b1; // Example: IRQ on THR empty
      else irq <= 1'b0; // No interrupt

    end
  end

endmodule
