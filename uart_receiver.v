`timescale 1ns/1ps

module uart_receiver #(
  parameter DATA_WIDTH = 8,
  parameter CLK_FREQ = 100_000_000,
  parameter BAUD_RATE = 9600
) (
  input wire clk,
  input wire rstn,
  input wire rx,
  input wire parity_en,
  input wire two_stop_bits,
  input wire [1:0] word_length,
  output reg [7:0] data_out,
  output reg data_valid,
  output reg parity_error,
  output reg frame_error
);

  localparam IDLE        = 3'b000;
  localparam START_BIT   = 3'b001;
  localparam DATA_BITS   = 3'b010;
  localparam PARITY_BIT  = 3'b011;
  localparam STOP1_BIT   = 3'b100;
  localparam STOP2_BIT   = 3'b101;
  localparam VALID_STATE = 3'b110;

  reg [2:0] rx_state, next_rx_state;
  reg [7:0] rx_shift_reg;
  reg [2:0] bit_counter;
  reg [31:0] baud_rate_counter;
  reg rx_sync;
  reg rx_prev;

  localparam CLK_FREQ_INTERNAL = 100_000_000;
  localparam BAUD_RATE_INTERNAL = 9600;
  localparam BAUD_PERIOD_CYCLES = (CLK_FREQ_INTERNAL / BAUD_RATE_INTERNAL);
  localparam HALF_BAUD_PERIOD_CYCLES = BAUD_PERIOD_CYCLES / 2;

  wire [2:0] actual_data_bits;
  assign actual_data_bits = (word_length == 2'b00) ? 3'd4 :
                            (word_length == 2'b01) ? 3'd5 :
                            (word_length == 2'b10) ? 3'd6 :
                            3'd7;

  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      rx_sync <= 1'b1;
      rx_prev <= 1'b1;
    end else begin
      rx_sync <= rx;
      rx_prev <= rx_sync;
    end
  end

  reg expected_parity;
  wire [7:0] data_for_parity_check;

  assign data_for_parity_check = (word_length == 2'b00) ? {3'b0, rx_shift_reg[4:0]} :
                                 (word_length == 2'b01) ? {2'b0, rx_shift_reg[5:0]} :
                                 (word_length == 2'b10) ? {1'b0, rx_shift_reg[6:0]} :
                                 rx_shift_reg;

  always @(*) begin
    expected_parity = ^data_for_parity_check;
  end

  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      rx_state <= IDLE;
      data_valid <= 1'b0;
      parity_error <= 1'b0;
      frame_error <= 1'b0;
      baud_rate_counter <= 0;
      bit_counter <= 0;
      rx_shift_reg <= 0;
      data_out <= 0;
    end else begin
      data_valid <= 1'b0;
      parity_error <= 1'b0;
      frame_error <= 1'b0;

      if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1)
        baud_rate_counter <= 0;
      else
        baud_rate_counter <= baud_rate_counter + 1;

      case (rx_state)
        IDLE: begin
          if (!rx_sync && rx_prev) begin
            next_rx_state = START_BIT;
            baud_rate_counter <= 0;
          end else begin
            next_rx_state = IDLE;
          end
        end
        START_BIT: begin
          if (baud_rate_counter == HALF_BAUD_PERIOD_CYCLES - 1) begin
            if (rx_sync == 1'b0) begin
              next_rx_state = DATA_BITS;
              baud_rate_counter <= 0;
              bit_counter <= 0;
            end else begin
              next_rx_state = IDLE;
              frame_error <= 1'b1;
            end
          end else begin
            next_rx_state = START_BIT;
          end
        end
        DATA_BITS: begin
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1) begin
            baud_rate_counter <= 0;
            rx_shift_reg <= {rx_sync, rx_shift_reg[7:1]};
            if (bit_counter == actual_data_bits) begin
              if (parity_en)
                next_rx_state = PARITY_BIT;
              else
                next_rx_state = STOP1_BIT;
            end else begin
              bit_counter <= bit_counter + 1;
              next_rx_state = DATA_BITS;
            end
          end else begin
            next_rx_state = DATA_BITS;
          end
        end
        PARITY_BIT: begin
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1) begin
            baud_rate_counter <= 0;
            if (rx_sync != expected_parity)
              parity_error <= 1'b1;
            next_rx_state = STOP1_BIT;
          end else begin
            next_rx_state = PARITY_BIT;
          end
        end
        STOP1_BIT: begin
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1) begin
            baud_rate_counter <= 0;
            if (rx_sync == 1'b1) begin
              if (two_stop_bits)
                next_rx_state = STOP2_BIT;
              else
                next_rx_state = VALID_STATE;
            end else begin
              next_rx_state = IDLE;
              frame_error <= 1'b1;
            end
          end else begin
            next_rx_state = STOP1_BIT;
          end
        end
        STOP2_BIT: begin
          if (baud_rate_counter == BAUD_PERIOD_CYCLES - 1) begin
            baud_rate_counter <= 0;
            if (rx_sync == 1'b1)
              next_rx_state = VALID_STATE;
            else begin
              frame_error <= 1'b1;
              next_rx_state = IDLE;
            end
          end else begin
            next_rx_state = STOP2_BIT;
          end
        end
        VALID_STATE: begin
          data_out <= rx_shift_reg;
          data_valid <= 1'b1;
          next_rx_state = IDLE;
        end
        default: next_rx_state = IDLE;
      endcase

      rx_state <= next_rx_state;
    end
  end

endmodule
