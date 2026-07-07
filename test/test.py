# SPDX-FileCopyrightText: © 2026 Vladimir Velikanov
# SPDX-License-Identifier: Apache-2.0

"""Cocotb tests for tt_um_vvelikanov_prbs_bert.

Migrated from the original self-checking Verilog testbench. The external
loopback (uo_out[0] -> ui_in[7]) is modeled by a background task that
re-drives ui_in on every falling clock edge, so at each rising edge the
DUT samples the tx bit registered one cycle earlier -- exactly as with a
physical loopback wire.

Reminder: the RX checker is self-synchronizing, so ONE channel error is
counted THREE times (the erroneous bit itself, then once per polynomial
tap as it shifts through the register).
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, Timer

# ui_in control bits (ui_in[7] is rx_in, driven by the loopback task)
PRBS_7 = 0b00
PRBS_15 = 0b01
PRBS_31 = 0b10
INJ_SINGLE = 1 << 2  # rising edge injects one error
INJ_PERIODIC = 1 << 3  # periodic injection enable
INJ_RATE = 1 << 4  # 0: every 512 bits, 1: every 8192 bits
CLEAR = 1 << 5  # clears error counter + sticky sync-loss flag
BYTE_SEL = 1 << 6  # selects error-counter byte on uio_out

# uo_out bit positions
SYNC = 1
ERR_PULSE = 2
LOST = 3


class BertBench:
    """Drives ui_in = {loopback bit, ctrl[6:0]} and reads DUT outputs."""

    def __init__(self, dut):
        self.dut = dut
        self.ctrl = 0

    async def _loopback(self):
        while True:
            await FallingEdge(self.dut.clk)
            try:
                tx = self.dut.uo_out.value.to_unsigned() & 1
            except ValueError:  # uo_out still 'x' at time zero
                tx = 0
            self.dut.ui_in.value = (tx << 7) | (self.ctrl & 0x7F)

    async def cycles(self, n):
        """Wait n rising edges, then 1 ns for outputs to settle."""
        await ClockCycles(self.dut.clk, n)
        await Timer(1, unit="ns")

    def uo_bit(self, bit):
        return (self.dut.uo_out.value.to_unsigned() >> bit) & 1

    async def pulse(self, bits, n):
        """Assert ctrl bits for n clocks, then deassert."""
        self.ctrl |= bits
        await self.cycles(n)
        self.ctrl &= ~bits

    async def clear(self):
        """Clear the error counter and the sticky sync-loss flag."""
        await self.pulse(CLEAR, 2)

    async def read_counter(self):
        """Read the 16-bit error counter byte by byte via uio_out."""
        self.ctrl &= ~BYTE_SEL
        await self.cycles(2)
        lo = self.dut.uio_out.value.to_unsigned()
        self.ctrl |= BYTE_SEL
        await self.cycles(2)
        hi = self.dut.uio_out.value.to_unsigned()
        self.ctrl &= ~BYTE_SEL
        await self.cycles(2)
        return (hi << 8) | lo


async def start(dut, ctrl):
    """Start clock + loopback and reset the DUT with the given ctrl bits."""
    bench = BertBench(dut)
    bench.ctrl = ctrl
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    dut.ena.value = 1
    dut.uio_in.value = 0
    dut.ui_in.value = ctrl & 0x7F
    dut.rst_n.value = 0
    cocotb.start_soon(bench._loopback())
    await bench.cycles(5)
    dut.rst_n.value = 1
    return bench


async def start_synced(dut, ctrl):
    """Reset, then wait for the checker to lock onto the loopback."""
    bench = await start(dut, ctrl)
    await bench.cycles(200)
    assert bench.uo_bit(SYNC) == 1, "sync not acquired during setup"
    return bench


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def test_sync_acquisition(dut):
    """Sync acquires on PRBS-31 and a clean loopback stays error-free."""
    bench = await start(dut, PRBS_31)

    await bench.cycles(200)
    assert bench.uo_bit(SYNC) == 1, "sync acquired on PRBS-31"
    assert bench.uo_bit(LOST) == 0, "no sync loss on clean loopback"

    await bench.cycles(2000)
    cnt = await bench.read_counter()
    assert cnt == 0, f"error counter stays 0 on clean channel (got {cnt})"


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def test_single_injection_and_clear(dut):
    """One injected error is counted exactly 3x; clear resets the counter."""
    bench = await start_synced(dut, PRBS_31)

    await bench.pulse(INJ_SINGLE, 4)
    await bench.cycles(100)  # let the error traverse the taps
    cnt = await bench.read_counter()
    assert cnt == 3, f"single injected error counted 3x (got {cnt})"
    assert bench.uo_bit(SYNC) == 1, "still in sync after single error"

    await bench.clear()
    cnt = await bench.read_counter()
    assert cnt == 0, f"counter clears (got {cnt})"


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def test_periodic_injection(dut):
    """Periodic injection at 1/512 keeps sync and counts ~3 per hit."""
    bench = await start_synced(dut, PRBS_31)

    bench.ctrl |= INJ_PERIODIC  # rate bit low -> every 512 bits
    await bench.cycles(512 * 4 + 100)
    bench.ctrl &= ~INJ_PERIODIC
    await bench.cycles(100)
    cnt = await bench.read_counter()
    assert 9 <= cnt <= 15, f"periodic errors counted ~3 per hit (got {cnt})"
    assert bench.uo_bit(SYNC) == 1, "sparse errors do not drop sync"


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def test_polynomial_switch(dut):
    """Switching PRBS-31 -> PRBS-7 re-syncs automatically and runs clean."""
    bench = await start_synced(dut, PRBS_31)

    bench.ctrl = (bench.ctrl & ~0b11) | PRBS_7
    await bench.cycles(300)
    assert bench.uo_bit(SYNC) == 1, "re-synced after switching to PRBS-7"

    await bench.clear()  # drop the errors counted during the switch
    await bench.cycles(1000)
    cnt = await bench.read_counter()
    assert cnt == 0, f"clean run on PRBS-7 after resync (got {cnt})"


@cocotb.test(timeout_time=100, timeout_unit="ms")
async def test_sync_loss_storm(dut):
    """An error storm sets the sticky loss flag; sync re-acquires after."""
    bench = await start_synced(dut, PRBS_7)

    # Toggle the single-injection input every clock: one rising edge (and
    # thus one injected error) every 2 clocks for 200 clocks.
    for _ in range(200):
        bench.ctrl ^= INJ_SINGLE
        await ClockCycles(dut.clk, 1)
    bench.ctrl &= ~INJ_SINGLE

    await bench.cycles(600)
    assert bench.uo_bit(LOST) == 1, "sticky sync-loss flag set after error storm"
    assert bench.uo_bit(SYNC) == 1, "re-acquired sync after storm ends"
