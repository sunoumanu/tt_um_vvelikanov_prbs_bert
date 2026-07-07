/*
 * tt_um_vvelikanov_prbs_bert -- PRBS generator + BER tester (1x1 tile)
 *
 * Copyright (c) 2026 Vladimir Velikanov
 * SPDX-License-Identifier: Apache-2.0
 *
 * TX: Fibonacci LFSR generating PRBS-7 / PRBS-15 / PRBS-31, serial out.
 * RX: self-synchronizing checker -- the receive shift register is always
 *     loaded with the *received* bits, so the predicted bit is a pure
 *     function of channel history and the checker locks onto any phase
 *     of the sequence automatically (no seed exchange needed).
 * BER: 16-bit saturating error counter, gated by sync.
 * SYNC: acquires after 64 consecutive error-free bits; declares loss of
 *     sync (sticky flag) if >= 16 errors are seen inside a 128-bit window.
 * INJECT: single-shot (button) or periodic (every 512 / 8192 bits) error
 *     injection into the *transmitted* bit only -- TX LFSR state is not
 *     corrupted, exactly like a real channel error.
 *
 * Note: with a self-synchronizing checker, ONE channel error produces
 * THREE error pulses (once as the incoming bit, once per polynomial tap
 * as it moves through the register). Real BERTs behave the same way;
 * divide the count by 3 for the raw channel-error count.
 *
 * Pinout
 * ------
 *  ui_in[1:0]  prbs_sel     00=PRBS-7, 01=PRBS-15, 1x=PRBS-31
 *  ui_in[2]    inj_single   rising edge injects one error
 *  ui_in[3]    inj_periodic enable periodic injection
 *  ui_in[4]    inj_rate     0: every 512 bits, 1: every 8192 bits
 *  ui_in[5]    clear        sync clear of error counter + sticky flag
 *  ui_in[6]    byte_sel     0: uio = errors[7:0], 1: uio = errors[15:8]
 *  ui_in[7]    rx_in        serial input (loop from uo_out[0] externally)
 *
 *  uo_out[0]   tx_out       PRBS serial output (one bit per clk)
 *  uo_out[1]   sync         checker is locked
 *  uo_out[2]   err_pulse    one clk per counted error (gated by sync)
 *  uo_out[3]   sync_lost    sticky: sync was lost since last clear
 *  uo_out[4]   inj_pulse    one clk per injected error
 *  uo_out[5]   rx_echo      registered copy of rx_in (scope/debug)
 *  uo_out[7:6] 0
 *
 *  uio[7:0]    outputs: selected byte of the 16-bit error counter
 */

`default_nettype none

module tt_um_vvelikanov_prbs_bert (
    input  wire [7:0] ui_in,    // dedicated inputs
    output wire [7:0] uo_out,   // dedicated outputs
    input  wire [7:0] uio_in,   // bidir: input path (unused)
    output wire [7:0] uio_out,  // bidir: output path
    output wire [7:0] uio_oe,   // bidir: enable (1 = output)
    input  wire       ena,      // always 1 when powered (unused)
    input  wire       clk,
    input  wire       rst_n     // active-low reset
);

  // ------------------------------------------------------------------
  // Pin map
  // ------------------------------------------------------------------
  wire [1:0] prbs_sel = ui_in[1:0];
  wire       inj_btn  = ui_in[2];
  wire       inj_peri = ui_in[3];
  wire       inj_rate = ui_in[4];
  wire       clr      = ui_in[5];
  wire       byte_sel = ui_in[6];
  wire       rx_in    = ui_in[7];

  // ------------------------------------------------------------------
  // TX: PRBS LFSR (Fibonacci, shift left, new bit enters at [0])
  //   PRBS-7 : x^7  + x^6  + 1  -> taps 6,5
  //   PRBS-15: x^15 + x^14 + 1  -> taps 14,13
  //   PRBS-31: x^31 + x^28 + 1  -> taps 30,27
  // ------------------------------------------------------------------
  reg [30:0] tx_sr;

  reg tx_taps;
  always @(*) begin
    case (prbs_sel)
      2'b00:   tx_taps = tx_sr[6]  ^ tx_sr[5];
      2'b01:   tx_taps = tx_sr[14] ^ tx_sr[13];
      default: tx_taps = tx_sr[30] ^ tx_sr[27];
    endcase
  end

  // Lock-up protection: if the active window is all zero (possible right
  // after switching to a shorter polynomial), force a 1 into the stream.
  reg tx_zero;
  always @(*) begin
    case (prbs_sel)
      2'b00:   tx_zero = ~|tx_sr[6:0];
      2'b01:   tx_zero = ~|tx_sr[14:0];
      default: tx_zero = ~|tx_sr;
    endcase
  end

  wire tx_next = tx_taps | tx_zero;

  // ------------------------------------------------------------------
  // Error injector
  // ------------------------------------------------------------------
  // 2FF synchronizer + edge detect for the button-like single input
  reg [2:0] inj_sync;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) inj_sync <= 3'b000;
    else        inj_sync <= {inj_sync[1:0], inj_btn};
  end
  wire inj_single = inj_sync[1] & ~inj_sync[2];

  // Periodic divider: fires every 512 or 8192 bits
  reg [12:0] inj_div;
  wire inj_wrap = inj_rate ? (inj_div == 13'd8191)
                           : (inj_div[8:0] == 9'd511);
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)       inj_div <= 13'd0;
    else if (inj_wrap) inj_div <= 13'd0;
    else               inj_div <= inj_div + 13'd1;
  end

  wire inj_fire = inj_single | (inj_peri & inj_wrap);

  // ------------------------------------------------------------------
  // TX state + registered (possibly corrupted) output bit
  // ------------------------------------------------------------------
  reg tx_out;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_sr  <= 31'h7FFF_FFFF;       // any non-zero seed
      tx_out <= 1'b0;
    end else begin
      tx_sr  <= {tx_sr[29:0], tx_next};
      tx_out <= tx_next ^ inj_fire;  // corrupt the wire, not the LFSR
    end
  end

  // ------------------------------------------------------------------
  // RX: self-synchronizing checker
  // ------------------------------------------------------------------
  reg        rx_d;                    // input register (timing/debug)
  reg [30:0] rx_sr;                   // history of RECEIVED bits

  reg rx_pred;                        // predicted next bit from history
  always @(*) begin
    case (prbs_sel)
      2'b00:   rx_pred = rx_sr[6]  ^ rx_sr[5];
      2'b01:   rx_pred = rx_sr[14] ^ rx_sr[13];
      default: rx_pred = rx_sr[30] ^ rx_sr[27];
    endcase
  end

  wire rx_err = rx_d ^ rx_pred;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_d  <= 1'b0;
      rx_sr <= 31'd0;
    end else begin
      rx_d  <= rx_in;
      rx_sr <= {rx_sr[29:0], rx_d};
    end
  end

  // ------------------------------------------------------------------
  // Sync acquisition / loss detection
  //   acquire: 64 consecutive error-free bits
  //   lose:    >= 16 errors within a 128-bit window
  // ------------------------------------------------------------------
  reg       sync;
  reg       lost;                     // sticky
  reg [5:0] acq_cnt;
  reg [6:0] win_cnt;
  reg [4:0] win_err;

  wire acq_done = (acq_cnt == 6'd63) & ~rx_err;
  wire win_done = (win_cnt == 7'd127);
  wire win_bad  = win_err >= 5'd16;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      sync    <= 1'b0;
      lost    <= 1'b0;
      acq_cnt <= 6'd0;
      win_cnt <= 7'd0;
      win_err <= 5'd0;
    end else begin
      if (!sync) begin
        // acquisition: count consecutive clean bits
        acq_cnt <= rx_err ? 6'd0 : (acq_cnt + 6'd1);
        if (acq_done) begin
          sync    <= 1'b1;
          win_cnt <= 7'd0;
          win_err <= 5'd0;
        end
      end else begin
        // tracking: windowed error-rate monitor
        win_cnt <= win_cnt + 7'd1;
        if (rx_err && !(&win_err))
          win_err <= win_err + 5'd1;
        if (win_done) begin
          win_err <= 5'd0;
          if (win_bad) begin
            sync    <= 1'b0;
            lost    <= 1'b1;
            acq_cnt <= 6'd0;
          end
        end
      end
      if (clr) lost <= 1'b0;
    end
  end

  // ------------------------------------------------------------------
  // 16-bit saturating BER counter (counts only while locked)
  // ------------------------------------------------------------------
  reg [15:0] err_cnt;
  wire count_en = sync & rx_err;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)               err_cnt <= 16'd0;
    else if (clr)             err_cnt <= 16'd0;
    else if (count_en && !(&err_cnt))
                              err_cnt <= err_cnt + 16'd1;
  end

  // ------------------------------------------------------------------
  // Outputs
  // ------------------------------------------------------------------
  assign uo_out[0] = tx_out;
  assign uo_out[1] = sync;
  assign uo_out[2] = count_en;   // error pulse, gated by sync
  assign uo_out[3] = lost;
  assign uo_out[4] = inj_fire;
  assign uo_out[5] = rx_d;
  assign uo_out[7:6] = 2'b00;

  assign uio_out = byte_sel ? err_cnt[15:8] : err_cnt[7:0];
  assign uio_oe  = 8'hFF;

  // silence unused-signal lint warnings
  wire _unused = &{ena, uio_in, 1'b0};

endmodule
