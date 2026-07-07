## How it works

The project is a serial bit-error-rate tester (BERT): a PRBS generator (TX) and a self-synchronizing checker (RX) that are completely independent inside the chip and are connected only externally, by looping the TX output pin back into the RX input pin.

**TX** is a 31-bit Fibonacci LFSR that generates PRBS-7 (x⁷+x⁶+1), PRBS-15 (x¹⁵+x¹⁴+1) or PRBS-31 (x³¹+x²⁸+1), selected with `ui[1:0]`, one bit per clock on `uo[0]`. The register has all-zero lock-up protection for the active window, so switching between polynomials at runtime is safe. An error injector can corrupt the *transmitted bit only* — the LFSR state is never touched, exactly like a real channel error. Injection is either single-shot (rising edge on `ui[2]`) or periodic (`ui[3]`, every 512 or 8192 bits per `ui[4]`).

**RX** is self-synchronizing: its shift register is loaded with the *received* bits, so the predicted next bit is a pure function of channel history and the checker locks onto any phase of the sequence without seed exchange. Because of this, one channel error is counted **three times** — once as the erroneous bit itself, then once per polynomial tap as it shifts through the register. Real BERTs behave the same way; divide the count by 3 for the raw channel-error count.

**Sync FSM**: the checker declares lock (`uo[1]`) after 64 consecutive error-free bits, and declares loss of sync — a sticky flag on `uo[3]` — when 16 or more errors are seen inside a 128-bit window. A 16-bit saturating error counter counts only while locked; its low or high byte (selected by `ui[6]`) is presented on the `uio[7:0]` pins, which are all configured as outputs. `ui[5]` clears the counter and the sticky flag.

## How to test

1. Connect the loopback: wire `uo[0]` (TX out) to `ui[7]` (RX in). A single jumper wire is enough.
2. Set all inputs low, select a polynomial on `ui[1:0]` (e.g. `10` for PRBS-31), apply a clock (any frequency; 10 MHz nominal) and release reset.
3. Within ~100 clock cycles `uo[1]` (sync) goes high and stays high. The error counter on `uio[7:0]` stays at zero.
4. Pulse `ui[2]` high for a few clocks to inject a single error: `uo[2]` pulses and the counter increments by exactly 3 (see the ×3 note above). Sync stays high.
5. Set `ui[3]` high for continuous periodic injection — every 512 bits (`ui[4]`=0) or every 8192 bits (`ui[4]`=1). The counter accumulates 3 counts per hit; sparse errors do not drop sync.
6. Break the loopback wire (or inject errors rapidly): sync drops and the sticky sync-loss flag `uo[3]` goes high. Restore the loop — sync re-acquires automatically, while `uo[3]` stays latched until cleared with `ui[5]`.
7. Read the full 16-bit count by sampling `uio[7:0]` with `ui[6]`=0 (low byte) and `ui[6]`=1 (high byte).

The repository also contains a self-checking cocotb testbench (`test/test.py`) that runs the same scenarios in simulation with the loopback modeled in software.

## External hardware

None required, apart from a single jumper wire looping output `uo[0]` back to input `ui[7]`. Optionally, a scope/logic analyzer on `uo[0]` (PRBS stream), `uo[2]` (error pulses) and `uo[4]` (injection pulses) is handy for demonstration.
