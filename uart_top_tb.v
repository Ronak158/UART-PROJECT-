`timescale 1ns/1ps
// `include "using_pulse_MS.v" // Removed: Cannot open include file

module uart_top_tb;

  // Signals for HOST A
  reg pclk_A;
  reg presetn_A;
  reg [31:0] paddr_A;
  reg [31:0] pwdata_A;
  reg pwrite_A;
  reg penable_A;
  reg psel_A;
  reg rxd_input_A;
  wire [31:0] prdata_A;
  wire pready_A;
  wire pslverr_A;
  wire irq_A;
  wire txd_A;
  wire baud_o_A;

  // Signals for HOST B
  reg pclk_B;
  reg presetn_B;
  reg [31:0] paddr_B;
  reg [31:0] pwdata_B;
  reg pwrite_B;
  reg penable_B;
  reg psel_B;
  reg rxd_input_B;
  wire [31:0] prdata_B;
  wire pready_B;
  wire pslverr_B;
  wire irq_B;
  wire txd_B;
  wire baud_o_B;

  // Declare variables as reg for procedural assignment
  reg [31:0] lsr_val_A;
  reg [31:0] lsr_val_B;
  reg [31:0] rbr_val_B;

  // host a and host b different signals and different clock
  // define UART 1 and UART 2 as rx and tx
  // from 1 to 2 and 2 to 1
  // UART 1 is Transmitter UART 2 is receiver

  uart_top #( // <--- UPDATED PARAMETER OVERRIDES HERE
    .TOP_DATA_WIDTH(8),
    .TOP_FIFO_DEPTH_BITS(4),
    .TOP_CLK_FREQ(100_000_000),
    .TOP_BAUD_RATE(9600)
  ) UART_1 (
    .pclk(pclk_A),
    .presetn(presetn_A),
    .paddr(paddr_A),
    .pwdata(pwdata_A),
    .pwrite(pwrite_A),
    .penable(penable_A),
    .psel(psel_A),
    .rx_input(txd_B), // UART1 TX connects to UART2 RX
    .prdata(prdata_A),
    .pready(pready_A),
    .pslverr(pslverr_A),
    .irq(irq_A),
    .tx_output(txd_A),
    .baud_o(baud_o_A)
  );

  uart_top #( // <--- UPDATED PARAMETER OVERRIDES HERE
    .TOP_DATA_WIDTH(8),
    .TOP_FIFO_DEPTH_BITS(4),
    .TOP_CLK_FREQ(100_000_000),
    .TOP_BAUD_RATE(9600)
  ) UART_2 (
    .pclk(pclk_B),
    .presetn(presetn_B),
    .paddr(paddr_B),
    .pwdata(pwdata_B),
    .pwrite(pwrite_B),
    .penable(penable_B),
    .psel(psel_B),
    .rx_input(txd_A), // UART2 TX connects to UART1 RX
    .prdata(prdata_B),
    .pready(pready_B),
    .pslverr(pslverr_B),
    .irq(irq_B),
    .tx_output(txd_B),
    .baud_o(baud_o_B)
  );

  // Clock generation
  initial begin
    pclk_A = 1'b0;
    forever #10 pclk_A = ~pclk_A; // 100MHz clock (20ns period)
  end

  initial begin
    pclk_B = 1'b0;
    forever #10 pclk_B = ~pclk_B; // 100MHz clock (20ns period)
  end

  // APB Write Task (simplified)
  task apb_write;
    input [31:0] addr;
    input [31:0] data;
    input host_select; // 0 for Host A, 1 for Host B
    begin
      @(negedge (host_select ? pclk_B : pclk_A));
      if (host_select == 0) begin // Host A
        psel_A = 1'b1;
        pwrite_A = 1'b1;
        paddr_A = addr;
        pwdata_A = data;
        penable_A = 1'b1;
        wait(pready_A);
        @(negedge pclk_A);
        psel_A = 1'b0;
        pwrite_A = 1'b0;
        penable_A = 1'b0;
      end else begin // Host B
        psel_B = 1'b1;
        pwrite_B = 1'b1;
        paddr_B = addr;
        pwdata_B = data;
        penable_B = 1'b1;
        wait(pready_B);
        @(negedge pclk_B);
        psel_B = 1'b0;
        pwrite_B = 1'b0;
        penable_B = 1'b0;
      end
    end
  endtask

  // APB Read Task (simplified)
  task apb_read;
    input [31:0] addr;
    input host_select; // 0 for Host A, 1 for Host B
    output [31:0] read_data;
    begin
      @(negedge (host_select ? pclk_B : pclk_A));
      if (host_select == 0) begin // Host A
        psel_A = 1'b1;
        pwrite_A = 1'b0;
        paddr_A = addr;
        penable_A = 1'b1;
        wait(pready_A);
        @(negedge pclk_A);
        read_data = prdata_A;
        psel_A = 1'b0;
        pwrite_A = 1'b0;
        penable_A = 1'b0;
      end else begin // Host B
        psel_B = 1'b1;
        pwrite_B = 1'b0;
        paddr_B = addr;
        penable_B = 1'b1;
        wait(pready_B);
        @(negedge pclk_B);
        read_data = prdata_B;
        psel_B = 1'b0;
        pwrite_B = 1'b0;
        penable_B = 1'b0;
      end
    end
  endtask

  // Test Sequence
  initial begin
    // 0ns: Reset Active
    presetn_A = 1'b0;
    presetn_B = 1'b0;
    psel_A = 1'b0; pwrite_A = 1'b0; penable_A = 1'b0; paddr_A = 'b0; pwdata_A = 'b0;
    psel_B = 1'b0; pwrite_B = 1'b0; penable_B = 1'b0; paddr_B = 'b0; pwdata_B = 'b0;
    rxd_input_A = 1'b1; // Idle high
    rxd_input_B = 1'b1; // Idle high

    #10; // Wait for 10ns (half clock cycle)

    // 10ns: System starts (Reset inactive)
    presetn_A = 1'b1;
    presetn_B = 1'b1;
    $display("Time %0t: Reset released.", $time);

    // Configure UART_1 (Host A) - LCR: 8-bit, no parity, 1 stop bit (00000011)
    // LCR address is 0x0C
    #20; // Wait for a full clock cycle
    apb_write(32'h0C, 32'h03, 0); // Host A, LCR = 0x03 (8-bit, 1 stop, no parity)
    $display("Time %0t: Host A LCR configured to 0x03.", $time);

    // Configure UART_2 (Host B) - LCR: 8-bit, no parity, 1 stop bit (00000011)
    apb_write(32'h0C, 32'h03, 1); // Host B, LCR = 0x03
    $display("Time %0t: Host B LCR configured to 0x03.", $time);

    // 20ns: Data pushed into FIFO (example data 0x55)
    // THR address is 0x00
    #20; // Wait for a full clock cycle
    apb_write(32'h00, 32'h55, 0); // Host A, THR = 0x55
    $display("Time %0t: Host A THR written with 0x55. Data pushed to FIFO.", $time);

    // Wait for transmission to complete (approx. 10 bits * BAUD_PERIOD_CYCLES * 20ns/cycle)
    // For 9600 baud, BAUD_PERIOD_CYCLES = 100M/9600 = 10416 cycles.
    // 10 bits * 10416 cycles * 20ns/cycle = 2.08ms.
    // We'll wait a bit more to ensure reception.
    #2_500_000; // Wait for 2.5ms (adjust as needed for actual baud rate)
    $display("Time %0t: Waiting for transmission to complete.", $time);

    // Check LSR of Host A (Transmitter)
    // LSR address is 0x14
    apb_read(32'h14, 0, lsr_val_A); // Read LSR of Host A
    $display("Time %0t: Host A LSR value: %h. (Expected TX Empty)", $time, lsr_val_A);
    if (lsr_val_A[6] == 1'b1) begin
      $display("Time %0t: Host A TX is empty as expected.", $time);
    end else begin
      $display("Time %0t: ERROR: Host A TX is NOT empty.", $time);
    end

    // Check LSR of Host B (Receiver)
    apb_read(32'h14, 1, lsr_val_B); // Read LSR of Host B
    $display("Time %0t: Host B LSR value: %h. (Expected Data Ready)", $time, lsr_val_B);
    if (lsr_val_B[0] == 1'b1) begin
      $display("Time %0t: Host B has data ready as expected.", $time);
    end else begin
      $display("Time %0t: ERROR: Host B has NO data ready.", $time);
    end

    // Read RBR of Host B (Receiver)
    // RBR address is 0x00
    apb_read(32'h00, 1, rbr_val_B); // Read RBR of Host B
    $display("Time %0t: Host B RBR value: %h. (Expected 0x55)", $time, rbr_val_B);
    if (rbr_val_B[7:0] == 8'h55) begin
      $display("Time %0t: Host B received data 0x55 correctly.", $time);
    end else begin
      $display("Time %0t: ERROR: Host B received incorrect data. Expected 0x55, got %h.", $time, rbr_val_B[7:0]);
    end

    // Final check on Host B LSR after reading RBR
    apb_read(32'h14, 1, lsr_val_B); // Read LSR of Host B again
    $display("Time %0t: Host B LSR value after RBR read: %h. (Expected Data Not Ready)", $time, lsr_val_B);
    if (lsr_val_B[0] == 1'b0) begin
      $display("Time %0t: Host B Data Ready flag cleared as expected.", $time);
    end else begin
      $display("Time %0t: ERROR: Host B Data Ready flag NOT cleared.", $time);
    end

    #100; // Small delay before finishing
    $finish; // End simulation
  end

endmodule
