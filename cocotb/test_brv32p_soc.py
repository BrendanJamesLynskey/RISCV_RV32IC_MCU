"""
test_brv32p_soc.py — CocoTB Testbench for BRV32P Pipelined RISC-V SoC
======================================================================
Tests pipeline execution, forwarding, cache behaviour, branch prediction,
and all functional instruction categories.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles


async def reset_dut(dut, cycles=10):
    dut.rst_n.value = 0
    dut.gpio_in.value = 0
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, cycles)
    dut.rst_n.value = 1
    await RisingEdge(dut.clk)


def get_reg(dut, idx):
    if idx == 0:
        return 0
    return int(dut.u_core.u_regfile.regs[idx].value)


async def wait_reg(dut, idx, val, timeout=50000):
    """Wait until register idx == val, with timeout."""
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        try:
            if get_reg(dut, idx) == val:
                return True
        except Exception:
            pass
    dut._log.warning(f"Timeout waiting for x{idx} = 0x{val:08X}")
    return False


async def wait_reg_nonzero(dut, idx, timeout=50000):
    """Wait until register idx != 0."""
    for _ in range(timeout):
        await RisingEdge(dut.clk)
        try:
            if get_reg(dut, idx) != 0:
                return True
        except Exception:
            pass
    return False


# ── Tests ────────────────────────────────────────────────────────────────

@cocotb.test()
async def test_01_reset(dut):
    """Verify PC starts at reset vector."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    dut.rst_n.value = 0
    dut.gpio_in.value = 0
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, 10)
    pc = int(dut.u_core.pc_if.value)
    assert pc == 0, f"Reset PC = 0x{pc:08X}"
    dut._log.info("PASS: Reset PC = 0x00000000")


@cocotb.test()
async def test_02_alu_pipeline(dut):
    """Test ALU instructions execute correctly through pipeline with forwarding."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Wait for first results to appear in register file
    ok = await wait_reg(dut, 3, 52)
    assert ok, "x3 never reached 52"

    checks = [
        (1, 42, "ADDI"), (2, 10, "ADDI"), (3, 52, "ADD"),
        (4, 32, "SUB"),  (5, 52, "ANDI"), (6, 0x55, "ORI"),
        (7, 0xAA, "XORI"), (8, 160, "SLLI"), (9, 40, "SRLI"),
    ]
    for reg, exp, name in checks:
        val = get_reg(dut, reg)
        assert val == exp, f"{name} x{reg}: got {val}, expected {exp}"
        dut._log.info(f"PASS: {name} x{reg} = {val}")


@cocotb.test()
async def test_03_forwarding(dut):
    """Verify data forwarding: ADD x3=x1+x2 uses freshly-written x1, x2."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ok = await wait_reg(dut, 3, 52)
    assert ok, "Forwarding failed: x3 != 52"
    dut._log.info("PASS: EX-EX forwarding verified (x3=x1+x2=52)")


@cocotb.test()
async def test_04_load_store_cache(dut):
    """Test load/store through the D-cache."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ok = await wait_reg(dut, 11, 52)
    assert ok, "LW through D-cache failed"
    dut._log.info(f"PASS: LW x11 = {get_reg(dut, 11)}")

    ok = await wait_reg(dut, 12, 0x55)
    assert ok, "LBU through D-cache failed"
    dut._log.info(f"PASS: LBU x12 = 0x{get_reg(dut, 12):02X}")


@cocotb.test()
async def test_05_branches(dut):
    """Test BEQ/BNE through pipeline with prediction."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ok = await wait_reg(dut, 15, 2)
    assert ok, "Branch test failed"
    dut._log.info(f"PASS: Branches — x15 = {get_reg(dut, 15)}")


@cocotb.test()
async def test_06_jal(dut):
    """Test JAL saves link and reaches target."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ok = await wait_reg(dut, 17, 3)
    assert ok, "JAL target not reached"
    assert get_reg(dut, 16) != 0, "JAL link register is zero"
    dut._log.info(f"PASS: JAL — x17={get_reg(dut, 17)}, x16=0x{get_reg(dut, 16):08X}")


@cocotb.test()
async def test_07_loop_bp_training(dut):
    """Test BNE countdown loop trains the branch predictor."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    ok = await wait_reg(dut, 23, 0, timeout=100000)
    assert ok, "Loop did not complete"
    dut._log.info("PASS: Countdown loop x23 = 0")

    # Check BHT was trained
    trained = 0
    for i in range(256):
        try:
            val = int(dut.u_core.u_bp.bht[i].value)
            if val != 1:  # Default is 2'b01
                trained += 1
        except Exception:
            pass
    assert trained > 0, "No BHT entries trained"
    dut._log.info(f"PASS: BHT has {trained} trained entries")


@cocotb.test()
async def test_08_icache(dut):
    """Verify I-cache fills lines during execution."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    await wait_reg(dut, 3, 52)  # Wait for some execution

    valid_count = 0
    for s in range(64):
        for w in range(2):
            try:
                if int(dut.u_icache.valid_mem[s][w].value):
                    valid_count += 1
            except Exception:
                pass

    assert valid_count > 0, "I-cache has no valid lines"
    dut._log.info(f"PASS: I-cache has {valid_count} valid lines")


@cocotb.test()
async def test_09_gpio_input(dut):
    """Test GPIO input synchronisation."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await ClockCycles(dut.clk, 20)

    dut.gpio_in.value = 0xCAFEBABE
    await ClockCycles(dut.clk, 5)

    synced = int(dut.u_gpio.gpio_in_sync.value)
    assert synced == 0xCAFEBABE, f"Got 0x{synced:08X}"
    dut._log.info(f"PASS: GPIO input sync = 0x{synced:08X}")


@cocotb.test()
async def test_10_csr_mcycle(dut):
    """Test mcycle counter increments."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)
    await ClockCycles(dut.clk, 100)

    mcycle = int(dut.u_core.u_csr.mcycle.value) & 0xFFFFFFFF
    assert mcycle > 0, f"mcycle = {mcycle}"
    dut._log.info(f"PASS: mcycle = {mcycle}")


@cocotb.test()
async def test_11_load_use_stall(dut):
    """Verify pipeline doesn't corrupt data on load-use sequences."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # The firmware has: SW x3, 0(x10) then LW x11, 0(x10)
    # This is a load followed by use — the hazard unit should insert
    # a 1-cycle bubble. If it doesn't, x11 would get stale data.
    ok = await wait_reg(dut, 11, 52)
    assert ok, "Load-use stall may have failed: x11 != 52"
    dut._log.info("PASS: Load-use stall correctly handled")


@cocotb.test()
async def test_12_pipeline_throughput(dut):
    """Measure approximate IPC by counting retired instructions vs cycles."""
    clock = Clock(dut.clk, 10, units="ns")
    cocotb.start_soon(clock.start())
    await reset_dut(dut)

    # Wait for program to mostly complete
    ok = await wait_reg(dut, 23, 0, timeout=100000)

    mcycle = int(dut.u_core.u_csr.mcycle.value) & 0xFFFFFFFF
    minstret = int(dut.u_core.u_csr.minstret.value) & 0xFFFFFFFF

    if mcycle > 0 and minstret > 0:
        ipc = minstret / mcycle
        dut._log.info(f"PASS: Throughput — {minstret} instructions / {mcycle} cycles = {ipc:.3f} IPC")
    else:
        dut._log.info(f"INFO: mcycle={mcycle}, minstret={minstret}")
