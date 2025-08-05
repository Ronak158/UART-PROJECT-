`timescale 1ns/1ps

module uart_transmitter #(
  parameter DATA_WIDTH = 8,
  parameter CLK_FREQ = 100_000_000,
  parameter BAUD_RATE = 9600
) (
  input wire clk,
  input wire rstn,
  input wire [7:0] data_in,
  input wire start,
  input wire parity_en,
  input wire two_stop_bits,
  input wire [1:0] word_length,
  output reg tx,
  output reg tx_busy
);

  localparam IDLE        = 3'b000;
  localparam START_BIT   = 3'b001;
  localparam DATA_BITS   = 3'b010;
  localparam PARITY_BIT  = 3'b011;
  localparam STOP1_BIT   = 3'b100;
  localparam STOP2_BIT   = 3'b101;

  reg [2:0] tx_state, next_tx_state;

  reg [7:0] tx_shift_reg;
  reg [2:0] bit_counter;
  reg parity_bit;
  reg [31:0] baud_rate_counter;

  localparam CLK_FREQ_INTERNAL = 100_000_000;
  localparam BAUD_RATE_INTERNAL = 9600;
  localparam BAUD_PERIOD_CYCLES = (CLK_FREQ_INTERNAL / BAUD_RATE_INTERNAL);

  wire [2:0] actual_data_bits;
  assign actual_data_bits = (word_length == 2'b00) ? 3'd4 :
                            (word_length == 2'b01) ? 3'd5 :
                            (word_length == 2'b10) ? 3'd6 :
                            3'd7;

  wire [7:0] data_for_parity;
  assign data_for_parity = (word_length == 2'b00) ? {3'b0, data_in[4:0]} :
                           (word_length == 2'b01) ? {2'b0, data_in[5:0]} :
                           (word_length == 2'b10) ? {1'b0, data_in[6:0]} :
                           data_in;

  always @(*) begin
    parity_bit = ^data_for_parity;
  end

  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      tx_state <= IDLE;
      tx_busy <= 1'b0;
      tx <= 1'b1;
      baud_rate_counter <= 0;
      bit_counter <= 0;
      tx_shift_reg <= 0;
    end else begin
      if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1)
        baud_rate_counter <= 0;
      else
        baud_rate_counter <= baud_rate_counter + 1;

      case (tx_state)
        IDLE: begin
          tx_busy <= 1'b0;
          tx <= 1'b1;
          if (start) begin
            next_tx_state = START_BIT;
            tx_shift_reg = data_in;
            bit_counter = 0;
            baud_rate_counter <= 0;
          end else begin
            next_tx_state = IDLE;
          end
        end
        START_BIT: begin
          tx_busy <= 1'b1;
          tx <= 1'b0;
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1)
            next_tx_state = DATA_BITS;
          else
            next_tx_state = START_BIT;
        end
        DATA_BITS: begin
          tx_busy <= 1'b1;
          tx <= tx_shift_reg[0];
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1) begin
            tx_shift_reg <= tx_shift_reg >> 1;
            if (bit_counter == actual_data_bits) begin
              if (parity_en)
                next_tx_state = PARITY_BIT;
              else
                next_tx_state = STOP1_BIT;
            end else begin
              bit_counter <= bit_counter + 1;
              next_tx_state = DATA_BITS;
            end
          end else begin
            next_tx_state = DATA_BITS;
          end
        end
        PARITY_BIT: begin
          tx_busy <= 1'b1;
          tx <= parity_bit;
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1)
            next_tx_state = STOP1_BIT;
          else
            next_tx_state = PARITY_BIT;
        end
        STOP1_BIT: begin
          tx_busy <= 1'b1;
          tx <= 1'b1;
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1) begin
            if (two_stop_bits)
              next_tx_state = STOP2_BIT;
            else
              next_tx_state = IDLE;
          end else
            next_tx_state = STOP1_BIT;
        end
        STOP2_BIT: begin
          tx_busy <= 1'b1;
          tx <= 1'b1;
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1)
            next_tx_state = IDLE;
          else
            next_tx_state = STOP2_BIT;
        end
        default: next_tx_state = IDLE;
      endcase

      tx_state <= next_tx_state;
    end
  end

endmodule
