"""Run the linked firmware with deterministic inputs on Unicorn Cortex-M0."""

from math import atan2, cos, isqrt, pi, sin, sqrt
from pathlib import Path
from random import Random
import json
import re
import struct
import tempfile
import time
from types import SimpleNamespace

from elftools.elf.elffile import ELFFile
from unicorn import UC_ARCH_ARM, UC_HOOK_CODE, UC_HOOK_MEM_WRITE, UC_MODE_THUMB, Uc
from unicorn.arm_const import (
    UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0, UC_ARM_REG_R1,
    UC_ARM_REG_R2, UC_ARM_REG_R3, UC_ARM_REG_SP, UC_CPU_ARM_CORTEX_M0,
)

from host.foc import FrameDecoder, Link, TRACE_FORMAT

ARGS = (UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3)
RAM = 0x20000000
SCRATCH = RAM + 0xA00
RETURN = 0xF000


class Emulator:
    def __init__(self, elf_path, reset=False):
        self.cpu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
        self.cpu.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M0)
        assert self.cpu.ctl_get_cpu_model() == UC_CPU_ARM_CORTEX_M0, "unexpected CPU model"
        self.cpu.mem_map(0, 0x10000)
        self.cpu.mem_map(RAM, 0x1000)
        if reset:
            self.cpu.mem_write(RAM, bytes([0xA5]) * 0x1000)
            self.cpu.mem_map(0x00100000, 0x1000)
            self.cpu.mem_map(0x40004000, 0x1000)
            self.cpu.mem_write(0x001007C0, struct.pack("<HH", 0x123, 0x45))

        with elf_path.open("rb") as stream:
            elf = ELFFile(stream)
            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_LOAD" and segment["p_filesz"]:
                    address = segment["p_paddr" if reset else "p_vaddr"]
                    self.cpu.mem_write(address, segment.data())
            self.symbols = {
                sym.name: (sym["st_value"], sym["st_size"])
                for sym in elf.get_section_by_name(".symtab").iter_symbols()
            }
        self.cpu.reg_write(UC_ARM_REG_SP, RAM + 0x1000)

    def address(self, name):
        return self.symbols[name][0] & ~1

    def read32(self, address):
        return struct.unpack("<I", self.cpu.mem_read(address, 4))[0]

    def write32(self, address, value):
        self.cpu.mem_write(address, struct.pack("<I", value & 0xFFFFFFFF))

    def args(self, count):
        return tuple(self.cpu.reg_read(reg) for reg in ARGS[:count])

    def result(self, value):
        self.cpu.reg_write(UC_ARM_REG_R0, value)

    def stub(self, name, function):
        def dispatch(machine, _address, _size, _data):
            function()
            machine.reg_write(UC_ARM_REG_PC, machine.reg_read(UC_ARM_REG_LR))

        address = self.address(name)
        self.cpu.hook_add(UC_HOOK_CODE, dispatch, begin=address, end=address)

    def call(self, name, *values):
        stack = RAM + 0xF00
        self.cpu.reg_write(UC_ARM_REG_SP, stack)
        self.cpu.reg_write(UC_ARM_REG_LR, RETURN | 1)
        for reg, value in zip(ARGS, values):
            self.cpu.reg_write(reg, value & 0xFFFFFFFF)
        for index, value in enumerate(values[4:]):
            self.write32(stack + 4 * index, value)
        self.cpu.emu_start(self.address(name) | 1, RETURN, count=100000)
        assert self.cpu.reg_read(UC_ARM_REG_PC) == RETURN, f"{name} did not return"
        return self.cpu.reg_read(UC_ARM_REG_R0)

    def count_call(self, name, *values):
        count = 0

        def count_instruction(_machine, _address, _size, _data):
            nonlocal count
            count += 1

        hook = self.cpu.hook_add(UC_HOOK_CODE, count_instruction)
        try:
            self.call(name, *values)
        finally:
            self.cpu.hook_del(hook)
        return count


def check_startup_and_modes(elf_path):
    emu = Emulator(elf_path, reset=True)
    cpu = emu.cpu
    rtt = emu.symbols["_SEGGER_RTT"][0]
    up, trace, down = rtt + 24, rtt + 48, rtt + 72
    commands = {
        1: b"@Q;", 2: b"@E;", 3: b"@C,600;",
        640: b"@V,200;", 641: b"@E;",
        650: b"@I,-120;", 660: b"@Q;", 670: b"@S,3000;", 690: b"@P,3500;",
        710: b"@Z,3200,500,20;", 730: b"@T,4000,6000,12000;",
        1500: b"@G,1,600,32000;", 1600: b"@X;", 1610: b"@P,1571;",
        1620: b"@G,1,600,32000;", 1630: b"@E;", 1642: b"@Q;", 1650: b"@D,4,2;",
        1660: b"@H,0,100,8;", 1661: b"@H,24,100,8;", 1662: b"@E;", 1663: b"@Q;",
        1740: b"@Q;", 1820: b"@Q;", 1900: b"@Q;", 1980: b"@Q;", 2060: b"@Q;", 2140: b"@Q;",
        2150: b"@X;", 2160: b"@H,120,180,0;", 2161: b"@E;", 2162: b"@Q;",
        2240: b"@Q;", 2320: b"@Q;", 2400: b"@Q;", 2480: b"@Q;", 2481: b"@H,120,501,0;",
        2482: b"@H,120,500,0;", 2491: b"@Q;", 2560: b"@Q;", 2570: b"@X;", 2575: b"@I,701;",
        2580: b"@L,700,3500;", 2585: b"@F,200,400,1000;", 2590: b"@I,650;", 2595: b"@V,3200;",
        2605: b"@Y,1500;", 2606: b"@E;", 3900: b"@Q;", 3901: b"@W,1200,950,1000;", 3902: b"@B,9000;", 3960: b"@X;",
        3961: b"@F,200,400,2000;", 3962: b"@F,200,400,2001;",
        3963: b"@L,2000,3000;", 3964: b"@I,2000;", 3965: b"@I,2001;",
        3966: b"@I,-2000;", 3967: b"@L,2001,3000;", 3968: b"@Y,1500;",
        3969: b"@Y,1000000;", 3970: b"@Y,-1000000;", 3971: b"@Y,1000001;",
        3972: b"@L,2000,6928;", 3973: b"@L,2000,6929;", 3974: b"@A,100000;",
        3975: b"@A,4999;", 3976: b"@A,1000000;", 3977: b"@A,1000001;",
        3978: b"@W,2000,950,1000;", 3979: b"@W,2001,950,1000;",
        3980: b"@W,1200,699,1000;", 3981: b"@W,1200,950,99;", 3982: b"@W,0,950,1000;",
        3983: b"@B,9000;", 3984: b"@B,18000;", 3985: b"@L,2000,0;",
        3986: b"@B,18000;", 3987: b"@V,10000;", 3988: b"@B,6000;",
        3989: b"@V,4000;", 3990: b"@B,24001;", 3991: b"@B,5999;",
        3992: b"@B,12000;", 3993: b"@B,0;",
    }
    expected = {
        1: (0, 0, 0), 2: (0, 0, 2), 3: (0, 1, 0), 640: (0, 3, 0),
        641: (0, 4, 0), 650: (1, 4, 0), 670: (2, 4, 0), 690: (3, 4, 0),
        710: (4, 4, 0), 730: (5, 4, 0), 1500: (5, 4, 2), 1600: (5, 3, 0),
        1610: (3, 3, 0), 1620: (3, 3, 0), 1630: (3, 4, 0),
        1660: (3, 3, 1), 1661: (6, 3, 0), 1662: (6, 4, 0), 2150: (6, 3, 0),
        2160: (6, 3, 0), 2161: (6, 4, 0), 2481: (6, 4, 1), 2482: (6, 4, 0),
        2570: (6, 3, 0), 2575: (6, 3, 1), 2580: (6, 3, 0), 2585: (6, 3, 0),
        2590: (1, 3, 0), 2595: (0, 3, 0),
        2605: (7, 3, 0), 2606: (7, 1, 0), 3900: (7, 4, 0), 3901: (7, 4, 0), 3960: (7, 3, 0),
        3961: (7, 3, 0), 3962: (7, 3, 1), 3963: (7, 3, 0), 3964: (1, 3, 0),
        3965: (1, 3, 1), 3966: (1, 3, 0), 3967: (1, 3, 1), 3968: (7, 3, 0),
        3969: (7, 3, 0), 3970: (7, 3, 0), 3971: (7, 3, 1),
        3972: (7, 3, 0), 3973: (7, 3, 1), 3974: (7, 3, 0),
        3975: (7, 3, 1), 3976: (7, 3, 0), 3977: (7, 3, 1),
        3978: (7, 3, 0), 3979: (7, 3, 1), 3980: (7, 3, 1), 3981: (7, 3, 1), 3982: (7, 3, 0),
    }
    decoder = FrameDecoder()
    tick = sequence = clock = 0
    frames, enables = [], []
    trace_bytes = bytearray()

    def check_ram():
        assert cpu.mem_read(rtt, 10) == b"SEGGER RTT", "RTT .data initialization"
        assert emu.read32(rtt + 16) == 2, "RTT up channels"
        for symbol, length in (("up_data", 128), ("trace_data", 512), ("down_data", 32)):
            address = emu.symbols[symbol][0]
            assert cpu.mem_read(address, length) == bytes(length), f"{symbol} was not zeroed"

    def snapshot():
        nonlocal tick, sequence, clock
        write, read = emu.read32(up + 12), emu.read32(up + 16)
        size, buffer = emu.read32(up + 8), emu.read32(up + 4)
        if write != read:
            data = bytes(cpu.mem_read(buffer + (read + i) % size, 1)[0]
                         for i in range((write - read) % size))
            frames.extend((tick, status) for status in decoder.feed(data))
            emu.write32(up + 16, write)
        write, read = emu.read32(trace + 12), emu.read32(trace + 16)
        size, buffer = emu.read32(trace + 8), emu.read32(trace + 4)
        trace_bytes.extend(cpu.mem_read(buffer + (read + i) % size, 1)[0]
                           for i in range((write - read) % size))
        emu.write32(trace + 16, write)
        tick += 1
        if tick == 4000:
            cpu.emu_stop()
            return
        if tick in commands:
            offset = emu.read32(down + 12)
            size, buffer = emu.read32(down + 8), emu.read32(down + 4)
            for byte in commands[tick]:
                cpu.mem_write(buffer + offset, bytes([byte]))
                offset = (offset + 1) % size
            emu.write32(down + 12, offset)
        increment = 3 if tick == 900 else 1
        sequence += increment
        clock += 1000 * increment
        b, c, timestamp = emu.args(3)
        cpu.mem_write(b, struct.pack("<H", 2048))
        cpu.mem_write(c, struct.pack("<H", 2050))
        emu.write32(timestamp, clock)
        emu.result(sequence)

    def encoder():
        valid = tick < 2607 and not 800 <= tick < 805
        if valid:
            displacement = 0
            for start, counts in ((1670, 261), (1750, 391), (1830, 352), (1910, 261),
                                  (1990, -261), (2070, -391), (2170, -365), (2250, -320),
                                  (2330, -309), (2410, -391), (2490, -320)):
                if tick < start:
                    break
                displacement = counts
            cpu.mem_write(emu.args(1)[0], struct.pack("<H", 8384 - displacement))
        emu.result(int(valid))

    def duty():
        assert all(0 <= value <= 1199 for value in emu.args(3)), f"invalid PWM at sample {tick}"

    def hfi_frame():
        values = (0, 33, 0, -33, -22, 41, 22, -41)
        cpu.mem_write(emu.args(1)[0], struct.pack("<I8h2H", tick, *values, 2048, 2050))
        emu.result(1)

    emu.stub("BSP_Init", check_ram)
    emu.stub("BoardTime_NowUs", lambda: emu.result(clock))
    emu.stub("BSP_StopRequested", lambda: emu.result(int(tick == 1640)))
    emu.stub("BSP_StopPressed", lambda: emu.result(0))
    emu.stub("CurrentAdc_Next", snapshot)
    emu.stub("MT6701_ReadAngle", encoder)
    emu.stub("DRV8313_SetEnabled", lambda: enables.append((tick, emu.args(1)[0])))
    emu.stub("DRV8313_SetDuty", duty)
    emu.stub("CurrentAdc_StartHfi", lambda: None)
    emu.stub("CurrentAdc_SetHfiDuty", lambda: None)
    emu.stub("CurrentAdc_PulseActive", lambda: emu.result(1))
    emu.stub("CurrentAdc_ReadHfi", hfi_frame)
    stack, entry = struct.unpack("<II", cpu.mem_read(0, 8))
    assert stack == RAM + 0x1000 and entry == emu.address("Reset_Handler") | 1, "reset vector"
    cpu.reg_write(UC_ARM_REG_SP, stack)
    cpu.emu_start(entry, RETURN, count=80_000_000)

    assert tick == 4000, f"replay stopped at sample {tick}"
    assert any(t == 1642 and s.position_mrad == 3067 for t, s in frames), "encoder direction"
    for sample, values in expected.items():
        tolerance = 32 if sample >= 3900 else 2
        nearby = [(t, s.mode, s.state, s.result, s.command) for t, s in frames
                  if sample <= t <= sample + tolerance]
        assert any(sample <= t <= sample + tolerance and (s.mode, s.state, s.result) == values
                   for t, s in frames), (
                       f"unexpected command reply at sample {sample}: {values}; "
                       f"nearby={nearby}")
    assert any(s.mode == 1 and s.iq_ref_ma == -120 and s.vq_mv < 0
               for _, s in frames), "negative current command"
    assert any(1300 <= t < 1500 and s.profile_mrad == 4000 and s.encoder_errors == 5
               and s.missed_samples == 2 and s.state == 4 for t, s in frames), "trajectory or error recovery"
    assert not any(641 < t < 1600 and not enabled for t, enabled in enables), "unsolicited stop"
    assert any(t == 1642 and s.state == 3 and s.event == 5 for t, s in frames), "board key stop"
    assert any(3900 <= t < 3960 and s.mode == 7 and s.state == 4 and s.vd_mv > 0 and s.encoder_errors > 100
               for t, s in frames), "HFI rotating field must work with no encoder readings"
    for sample, center, low, high in ((1663, 3067, 0, 0), (1740, 3067, -100, -60),
                                       (1820, 3328, 60, 100), (1900, 3328, 60, 100),
                                       (1980, 3067, -100, -60), (2060, 3067, 60, 100),
                                       (2140, 2806, -100, -60)):
        assert any(sample <= t <= sample + 2 and s.command == ord("Q") and s.mode == 6 and
                   s.profile_mrad == center and low <= s.iq_ref_ma <= high for t, s in frames), (
                       f"ratchet center or restoring current at sample {sample}")
    fine_origin = next(s.profile_mrad for t, s in frames if 2162 <= t <= 2164 and s.command == ord("Q"))
    for sample, shift, low, high in ((2162, 0, 0, 0), (2240, 0, -18, -11), (2320, 0, -180, -175),
                                      (2400, 52, 85, 115), (2480, 0, -2, 2), (2491, 0, -250, -150),
                                      (2560, 0, -500, -490)):
        assert any(sample <= t <= sample + 2 and s.command == ord("Q") and s.mode == 6 and
                   s.profile_mrad == fine_origin + shift and low <= s.iq_ref_ma <= high for t, s in frames), (
                       f"fine ratchet center or torque shape at sample {sample}")
    assert any(3902 <= t <= 3934 and s.command == ord("B") and s.result == 2 and s.bus_mv == 12000 for t, s in frames), "bus edit while running"
    for tick_start, bus, limit in ((3983, 9000, 5196), (3984, 18000, 6928), (3986, 18000, 10392),
                                   (3988, 6000, 3464), (3992, 12000, 6928)):
        assert any(tick_start <= t <= tick_start + 1 and s.command == ord("B") and s.result == 0
                   and s.bus_mv == bus and s.voltage_limit_mv == limit for t, s in frames), "bus command/limits"
    for tick_start in (3989, 3990, 3991):
        assert any(t == tick_start and s.result == 1 for t, s in frames), "bus voltage bounds"
    assert any(t >= 3993 and s.bus_source == 0 and s.bus_mv == 12000 for t, s in frames), "stub supply source"
    records = list(TRACE_FORMAT.iter_unpack(trace_bytes))
    assert [r[16] for r in records] == [0, 1, 2, 3], "trace record count and indices"
    assert all(r[4:9] == (8384, 0, 0xFFFF, 2048, 2050) for r in records), "trace sensor values"
    assert all(r[21] == 1000 for r in records), "SMO trace sample interval"
    assert all(b[0] - a[0] == 2 and b[1] - a[1] == 2000
               for a, b in zip(records, records[1:])), "trace sample timing"
    payload = bytearray()
    expected_speeds = []
    for index, (velocity_q8, sample_us) in enumerate(((701, 1000), (-701, 1000),
                                                    (701, 1600), (-701, 1600))):
        values = list(records[index])
        values[16], values[17], values[21] = index, velocity_q8, sample_us
        payload.extend(TRACE_FORMAT.pack(*values))
        expected_speeds.append(int(velocity_q8 * 1000000 / (4673 * sample_us)))
    chunks = iter((b"", payload))
    link = Link(None, None, 1, SimpleNamespace(read=lambda: next(chunks, b"")))
    link.command = lambda *_: None
    link.read = lambda: None
    decoded = list(link.trace(4, 2))
    assert [s["smo_speed_mrad_s"] for s in decoded] == expected_speeds, "host SMO speed conversion"
    return {"samples": tick - 1, "frames": len(frames), "trace_records": len(records)}


def check_adc_and_pwm(elf_path):
    emu = Emulator(elf_path)
    for address, size in ((0x40000000, 0x2000), (0x40004000, 0x1000),
                          (0x48000000, 0x1000), (0xE000E000, 0x1000)):
        emu.cpu.mem_map(address, size)
    emu.call("CurrentAdc_Init")
    emu.call("DRV8313_Init")
    assert emu.read32(0x40000018) == 2, "ADC trigger route"
    assert (emu.read32(0x40001404) >> 20) & 31 == 0x14, "ATIM TRGO2 source"
    assert emu.read32(0x40001448) == 96, "ADC compare point"
    assert emu.read32(0x40000074) & 2 and emu.read32(0xE000E100) & (1 << 12), "ADC IRQ enable"
    for sample in range(1, 41):
        emu.write32(0x4000007C, 2)
        emu.write32(0x40000040, 2048 + sample)
        emu.write32(0x40000044, 2050 - sample)
        emu.call("ADC_IRQHandler")
        count = emu.call("CurrentAdc_Next", SCRATCH, SCRATCH + 2, SCRATCH + 4)
        assert count == sample // 20, f"wrong ADC decimation at sample {sample}"
        if sample % 20 == 0:
            actual = struct.unpack("<HH", emu.cpu.mem_read(SCRATCH, 4))
            assert actual == (2048 + sample, 2050 - sample), "wrong ADC sample order"

    duties, enabled = [], []
    assert emu.call("CurrentAdc_Next", SCRATCH, SCRATCH + 2, SCRATCH + 4) == 2
    for sample in range(200):
        emu.call("ADC_IRQHandler")
    assert emu.call("CurrentAdc_Next", SCRATCH, SCRATCH + 2, SCRATCH + 4) == 5, "queue overflow sequence"
    for expected_sequence in (6, 7, 8, 9, 10, 11, 12, 12):
        assert emu.call("CurrentAdc_Next", SCRATCH, SCRATCH + 2, SCRATCH + 4) == expected_sequence
    emu.stub("DRV8313_SetDuty", lambda: duties.append(emu.args(3)))
    emu.stub("DRV8313_SetEnabled", lambda: enabled.append(emu.args(1)[0]))
    def timer_duty(_machine, _access, _address, _size, value, _context):
        duties.append((emu.read32(0x40001434), emu.read32(0x40001438), value))
    emu.cpu.hook_add(UC_HOOK_MEM_WRITE, timer_duty, begin=0x40001440, end=0x40001440)
    emu.cpu.mem_write(SCRATCH, struct.pack("<15H", *range(100, 115)))
    emu.call("CurrentAdc_StartPulse", SCRATCH, 2, 2, 2048, 2050)
    for sample in range(168):
        vector, point = (sample // 11) % 4, sample % 11 - 8
        delta = (vector + 1) * (point + 1) if point >= 0 else 0
        emu.write32(0x40000040, 2048 + delta)
        emu.write32(0x40000044, 2050 - delta)
        emu.call("ADC_IRQHandler")
    assert not emu.call("CurrentAdc_PulseActive") and emu.call("CurrentAdc_PulseFault") == 0
    assert enabled == [1, 0], "pulse run must disable the drive"
    zero = (112, 113, 114)
    expected = [zero]
    for vector in range(8):
        start = 100 + 3 * (vector % 4)
        expected.extend(((start, start + 1, start + 2), zero))
    assert duties == expected + [zero], "pulse PWM schedule"
    results = emu.call("CurrentAdc_PulseResponses")
    for vector in range(4):
        row = struct.unpack("<I10i10I", emu.cpu.mem_read(results + 84 * vector, 84))
        assert row[0] == 2 and row[1:7] == tuple(
            factor * (vector + 1) for factor in (2, -2, 4, -4, 6, -6)), "pulse sample sums"
        assert row[11:17] == tuple(
            factor * (vector + 1) ** 2 for factor in (2, 2, 8, 8, 18, 18)), "pulse sample variance sums"
    emu.call("CurrentAdc_StartPulse", SCRATCH, 1, 1, 2048, 2050)
    emu.write32(0x40000040, 2210)
    emu.call("ADC_IRQHandler")
    assert not emu.call("CurrentAdc_PulseActive") and emu.call("CurrentAdc_PulseFault") == 1
    assert enabled[-2:] == [1, 0], "pulse current limit must disable drive"
    emu.cpu.mem_write(SCRATCH, struct.pack("<15H", *(round(value * 256 / 5) for value in range(100, 115))))
    emu.cpu.mem_write(SCRATCH, struct.pack("<15H", *(value * 51 for value in range(100, 115))))
    emu.call("CurrentAdc_StartHfi", SCRATCH, 2048, 2050)
    assert emu.read32(0x40001448) == 24, "100 kHz ADC compare point"
    multiplications = []
    def check_hot_multiply(machine, address, _size, _context):
        opcode = struct.unpack("<H", machine.mem_read(address, 2))[0]
        if opcode & 0xFFC0 == 0x4340:
            multiplications.append(address)
    multiply_hook = emu.cpu.hook_add(UC_HOOK_CODE, check_hot_multiply)
    for sample in range(168):
        delta = (1, 3, 2, 4)[(sample - 8) % 4] if sample >= 8 else 0
        emu.write32(0x40000040, 2048 + delta)
        emu.write32(0x40000044, 2050 - delta)
        emu.call("ADC_IRQHandler")
    emu.cpu.hook_del(multiply_hook)
    assert not multiplications, "32-cycle multiply returned to the 100 kHz ADC path"
    frame = SCRATCH + 64
    emu.cpu.mem_write(frame, bytes(24))
    assert emu.call("CurrentAdc_ReadHfi", frame) == 1
    assert struct.unpack("<I8h2H", emu.cpu.mem_read(frame, 24)) == (
        1, 8, -8, 16, -16, 24, -24, 32, -32, 2050, 2048), "continuous HFI frame"
    assert emu.call("CurrentAdc_ReadHfi", frame) == 0
    alpha_mv, beta_mv = struct.unpack("<ii", emu.cpu.mem_read(frame + 24, 8))
    assert abs(alpha_mv + 10) <= 1 and abs(beta_mv + 6) <= 1, (
        f"averaged PWM voltage ({alpha_mv}, {beta_mv})")
    for sample in range(160):
        delta = (1, 3, 2, 4)[sample % 4]
        emu.write32(0x40000040, 2048 + delta)
        emu.write32(0x40000044, 2050 - delta)
        emu.call("ADC_IRQHandler")
    assert emu.call("CurrentAdc_ReadHfi", frame) == 1
    assert struct.unpack("<I8h2H", emu.cpu.mem_read(frame, 24)) == (
        2, 8, -8, 16, -16, 24, -24, 32, -32, 2050, 2048), "phase-continuous HFI frame"

    # A new current-loop target must settle across carrier cycles instead of
    # jumping to its final timer counts at the control-subframe boundary.
    emu.cpu.mem_write(SCRATCH, struct.pack("<15H", *(round(value * 256 / 5) for value in range(400, 415))))
    emu.call("CurrentAdc_SetHfiDuty", SCRATCH)
    ramp_start = len(duties) + 3
    for _ in range(1600):
        emu.call("ADC_IRQHandler")
    ramp = duties[ramp_start::4]
    assert 20 < ramp[0][0] < 80, "HFI target must begin with a partial step"
    assert all(all(a <= b + 1 for a, b in zip(before, after))
               for before, after in zip(ramp, ramp[1:])), "HFI ramp must remain monotonic"

    emu.cpu.mem_write(SCRATCH, struct.pack("<15H", *(round(value * 256 / 5) for value in range(100, 115))))
    emu.call("CurrentAdc_SetHfiDuty", SCRATCH)
    ramp_start = len(duties) + 3
    for _ in range(1600):
        emu.call("ADC_IRQHandler")
    ramp = duties[ramp_start::4]
    assert 20 < ramp[0][0] < 80, "HFI falling target must also begin with a partial step"
    assert all(all(a + 1 >= b for a, b in zip(before, after))
               for before, after in zip(ramp, ramp[1:])), "HFI falling ramp must remain monotonic"
    emu.call("CurrentAdc_CancelPulse")
    assert enabled[-2:] == [1, 0], "continuous HFI stop must disable the drive"
    assert emu.read32(0x40001448) == 96, "restore 20 kHz ADC compare point"
    # Publishing a target preserves Q8 timer resolution.
    hfi = emu.address("measurement")
    for value in range(1200):
        value_q8 = value * 51
        emu.cpu.mem_write(SCRATCH, struct.pack("<15H", *([value_q8] * 15)))
        emu.call("CurrentAdc_SetHfiDuty", SCRATCH)
        active_targets = emu.read32(hfi + 60)
        scaled = struct.unpack("<15H", emu.cpu.mem_read(active_targets, 30))
        assert scaled[:12] == (0,) * 12, "zero HFI excitation"
        assert scaled[12:] == (value_q8,) * 3, "HFI Q8 target"
    # Returning to finite calibration replaces the shared HFI state completely.
    emu.cpu.mem_write(SCRATCH, struct.pack("<15H", *range(100, 115)))
    emu.call("CurrentAdc_StartPulse", SCRATCH, 1, 1, 2048, 2050)
    emu.write32(0x40000040, 2048)
    emu.write32(0x40000044, 2050)
    for _ in range(40):
        emu.call("ADC_IRQHandler")
    assert not emu.call("CurrentAdc_PulseActive")
    assert emu.call("CurrentAdc_PulseFault") == 0
    results = emu.call("CurrentAdc_PulseResponses")
    for vector in range(4):
        row = struct.unpack("<I10i10I", emu.cpu.mem_read(results + 84 * vector, 84))
        assert row == (1,) + (0,) * 20, "shared calibration buffer was not reinitialized"
    emu.call("CurrentAdc_SetHfiCurrentLimit", 2000)
    emu.call("CurrentAdc_StartHfi", SCRATCH, 2048, 2050)
    emu.cpu.mem_write(frame, bytes(32))
    for index, current_counts in enumerate((1241, -1241, 1241), start=1):
        emu.write32(0x40000040, 2048 + current_counts)
        emu.write32(0x40000044, 2050 - current_counts)
        for _ in range(168 if index == 1 else 160):
            emu.call("ADC_IRQHandler")
        assert emu.call("CurrentAdc_PulseActive"), "2 A must remain within the HFI limit"
        assert emu.call("CurrentAdc_ReadHfi", frame), "missing high-current HFI frame"
        actual = struct.unpack("<I8h2H", emu.cpu.mem_read(frame, 24))
        expected = (index, *((8 * current_counts, -8 * current_counts) * 4),
                    2048 + current_counts, 2050 - current_counts)
        assert actual == expected, f"32-bit HFI accumulation overflow or stale data: {actual}"
    emu.call("CurrentAdc_CancelPulse")
    for b, c in ((1242, 0), (-1242, 0), (700, 700), (-700, -700)):
        emu.call("CurrentAdc_StartHfi", SCRATCH, 2048, 2050)
        emu.write32(0x40000040, 2048 + b)
        emu.write32(0x40000044, 2050 + c)
        emu.call("ADC_IRQHandler")
        assert not emu.call("CurrentAdc_PulseActive"), "phase current above 2 A must trip"
        assert emu.call("CurrentAdc_PulseFault") == 1
    return {"conversions": 40, "control_samples": 2, "pulse_bursts": 8,
            "hfi_frames": 2, "hfi_ramp_directions": 2, "hfi_compare_values": 1200,
            "hfi_2a_frames": 3, "hfi_2a_overcurrent_cases": 4,
            "hfi_hot_path_multiplies": len(multiplications)}


def check_sensorless_sampler(elf_path):
    emu = Emulator(elf_path)
    for address, size in ((0x40000000, 0x2000), (0x40004000, 0x1000),
                          (0x48000000, 0x1000), (0xE000E000, 0x1000)):
        emu.cpu.mem_map(address, size)
    emu.call("CurrentAdc_Init")
    emu.call("DRV8313_Init")
    enabled = []
    emu.stub("DRV8313_SetEnabled", lambda: enabled.append(emu.args(1)[0]))
    duties, frame = SCRATCH, SCRATCH + 64

    def target(values):
        emu.cpu.mem_write(duties, struct.pack("<15H", *(tuple(v * 256 for v in values) * 5)))

    def pwm():
        return tuple(emu.read32(address) for address in (0x40001434, 0x40001438, 0x40001440))

    def conversions(count):
        for _ in range(count):
            emu.write32(0x4000007C, 2)
            emu.write32(0x40000040, 2168)
            emu.write32(0x40000044, 1970)
            emu.call("ADC_IRQHandler")

    target((100, 125, 80))
    emu.call("CurrentAdc_StartHfi", duties, 2048, 2050)
    assert pwm() == (100, 125, 80)
    emu.call("CurrentAdc_SetSmoMode", 1, duties)
    assert enabled == [1], "changing rate must retain bridge enable"
    assert pwm() == (500, 625, 400), "PWM voltage changed at the fast-mode boundary"
    assert emu.read32(0x4000142C) == 1200 and emu.read32(0x40001448) == 96
    emu.cpu.mem_write(frame, bytes(36))
    conversions(3)
    assert not emu.call("CurrentAdc_ReadHfi", frame), "fast frame published too early"
    conversions(1)
    assert emu.call("CurrentAdc_ReadHfi", frame)
    sequence, *rest = struct.unpack("<I8h2H", emu.cpu.mem_read(frame, 24))
    assert sequence == 1 and rest == [0] * 8 + [2168, 1970], "fast averaged currents"
    alpha, beta, period = struct.unpack("<iiH", emu.cpu.mem_read(frame + 24, 10))
    assert (alpha, beta, period) == (-83, 1299, 200), "fast averaged voltage or period"
    emu.call("CurrentAdc_SetSmoDuty", 600, 500, 550)
    conversions(4)
    assert emu.call("CurrentAdc_ReadHfi", frame)
    assert struct.unpack("<iiH", emu.cpu.mem_read(frame + 24, 10)) == (500, -289, 200)
    conversions(32)
    assert emu.call("CurrentAdc_Next", SCRATCH + 128, SCRATCH + 130, SCRATCH + 132) == 1, (
        "fast control must retain 500 Hz diagnostics")
    target((120, 100, 110))
    emu.call("CurrentAdc_SetSmoMode", 0, duties)
    assert enabled == [1], "returning to HFI must retain bridge enable"
    assert pwm() == (120, 100, 110), "PWM voltage changed on return to HFI"
    assert emu.read32(0x4000142C) == 240 and emu.read32(0x40001448) == 24
    conversions(160)
    assert emu.call("CurrentAdc_ReadHfi", frame)
    actual = struct.unpack("<I8h2H", emu.cpu.mem_read(frame, 24))
    assert actual == (11, *((960, -640) * 4), 2168, 1970), "HFI buffer after fast operation"
    alpha, beta, period = struct.unpack("<iiH", emu.cpu.mem_read(frame + 24, 10))
    assert abs(alpha - 500) <= 1 and beta == -289 and period == 1600
    for bus in (6000, 9000, 12000, 18000, 24000):
        emu.call("CurrentAdc_SetBusVoltage", bus)
        conversions(160)
        assert emu.call("CurrentAdc_ReadHfi", frame)
        alpha, beta = struct.unpack("<ii", emu.cpu.mem_read(frame + 24, 8))
        assert abs(alpha - 500 * bus / 12000) <= 2
        assert abs(beta + 289 * bus / 12000) <= 3
    emu.call("CurrentAdc_SetSmoMode", 1, duties)
    emu.call("CurrentAdc_SetSmoDuty", 600, 500, 550)
    for bus in (6000, 9000, 12000, 18000, 24000):
        emu.call("CurrentAdc_SetBusVoltage", bus)
        conversions(4)
        assert emu.call("CurrentAdc_ReadHfi", frame)
        alpha, beta = struct.unpack("<ii", emu.cpu.mem_read(frame + 24, 8))
        assert abs(alpha - 500 * bus / 12000) <= 2
        assert abs(beta + 289 * bus / 12000) <= 3
    emu.call("CurrentAdc_CancelPulse")
    assert enabled == [1, 0] and emu.read32(0x4000142C) == 1200
    return {"fast_control_hz": 5000, "diagnostics_hz": 500, "voltage_preserving_changes": 2}


def check_hfi_observer(elf_path):
    emu = Emulator(elf_path)
    observer, currents = SCRATCH, SCRATCH + 64
    emu.cpu.mem_write(observer, bytes(32))

    def inject(phase, locking):
        theta = phase * 2 * pi / 16384
        c, s = cos(2 * theta), sin(2 * theta)
        x, y = -44 - 64 * c - 15 * s, -87 + 20 * c - 58 * s
        determinant = 64 - (37 / 8) ** 2
        ab = round((8 * x + 37 * y / 8) / determinant)
        bc = round((37 * x / 8 + 8 * y) / determinant)
        ab, bc = round(ab / 2), round(bc / 2)
        values = (ab, 0, -ab, 0, 0, bc, 0, -bc)
        emu.cpu.mem_write(currents, struct.pack("<8h", *values))
        return emu.call("Hfi_Update", observer, currents, locking)

    for _ in range(120):
        locked = inject(0, True)
    assert locked and abs((emu.call("Hfi_Angle", observer) + 8192) % 16384 - 8192) < 900, "HFI lock"
    phase = 0
    for direction in (1, -1):
        for _ in range(250):
            phase += direction * 30
            assert inject(phase, False), "HFI tracking lost"
        estimate = emu.call("Hfi_Angle", observer)
        speed = struct.unpack("<i", struct.pack("<I", emu.call("Hfi_Speed", observer)))[0]
        assert abs((estimate - phase + 8192) % 16384 - 8192) < 1200, "HFI angle tracking"
        assert 650 < direction * speed < 1350, f"HFI speed {speed}"
    return {"tracking_directions": 2}


def check_smo_observer(elf_path):
    image = elf_path.with_name("smo_validation.elf")
    cases = []
    operating_points = ((50, 8), (1000, 8), (1600, 8),
                        (200, 30), (200, 150), (200, 300), (1600, 160),
                        (200, 600), (200, 1000))
    for sample_us, mechanical_speed in operating_points:
        for direction in (1, -1):
            emu = Emulator(image)
            observer, config, inputs = SCRATCH, SCRATCH + 64, SCRATCH + 128
            emu.call("Smo_Init", observer)
            emu.call("Smo_ConfigDefault", config)
            values = list(struct.unpack("<9i", emu.cpu.mem_read(config, 36)))
            values[2] = sample_us
            values[5] = 3 if sample_us == 50 else 1
            emu.cpu.mem_write(config, struct.pack("<9i", *values))
            emu.cpu.mem_write(config + 48, struct.pack("<3i", 130, 120, 10000))
            emu.call("Smo_Prepare", config)
            omega = direction * mechanical_speed * 7
            advance = omega * sample_us / 1e6
            speed_q8 = round(advance * 16384 * 256 / (2 * pi))
            emu.call("Smo_Seed", observer, 0, speed_q8)
            errors, speeds = [], []
            for index in range(1600):
                theta = index * advance
                # Backward-Euler plant consistent with the observer's input
                # timing. The plant uses floating point, the ARM observer does not.
                ia, ib = 200 * cos(theta), 200 * sin(theta)
                pa, pb = 200 * cos(theta - advance), 200 * sin(theta - advance)
                # Higher-speed cases check sampling with reduced flux. Their
                # voltage does not imply this 12 V motor can reach that speed.
                emf = min(mechanical_speed * 19, 5700)
                ea = -direction * emf * sin(theta)
                eb = direction * emf * cos(theta)
                if sample_us == 200:
                    va = 3.7 * ia - .001 * omega * ib + ea
                    vb = 3.7 * ib + .001 * omega * ia + eb
                else:
                    va = 3.7 * ia + 1000 * (ia - pa) / sample_us + ea
                    vb = 3.7 * ib + 1000 * (ib - pb) / sample_us + eb
                emu.cpu.mem_write(inputs, struct.pack("<4i", round(va), round(vb),
                                                     round(ia), round(ib)))
                valid = emu.call("Smo_Update", observer, config, inputs)
                if index >= 1000:
                    phase = emu.call("Smo_Angle", observer)
                    error = (phase - theta * 16384 / (2 * pi) + 8192) % 16384 - 8192
                    errors.append(abs(error) * 360 / 16384)
                    raw = emu.call("Smo_ElectricalSpeedQ8", observer)
                    speeds.append(struct.unpack("<i", struct.pack("<I", raw))[0])
                    assert valid, f"SMO lost lock at {sample_us} us, direction {direction}"
            # At 1000 rad/s, 5 kHz gives 4.49 samples per electrical turn.
            # Bound the continuous-plant discretization error separately.
            limit_deg = 8 if mechanical_speed == 1000 else 6
            assert max(errors) < limit_deg, (
                f"SMO angle error {max(errors):.2f} degrees at {mechanical_speed} rad/s")
            mean_speed = sum(speeds) / len(speeds)
            assert abs(mean_speed - speed_q8) < max(64, abs(speed_q8) * 0.06), "SMO speed"
            assert emu.call("Smo_CanTakeOver", observer), "SMO EMF qualification"
            # Remove voltage and current. The observer must revoke validity.
            emu.cpu.mem_write(inputs, bytes(16))
            for _ in range(160):
                valid = emu.call("Smo_Update", observer, config, inputs)
            assert not valid, "SMO retained lock after loss of back EMF"
            assert not emu.call("Smo_CanTakeOver", observer), "SMO fallback after EMF loss"
            emu.call("Smo_Reset", observer)
            assert not emu.call("Smo_IsValid", observer, config), "SMO reset validity"
            cases.append({"sample_us": sample_us, "speed_rad_s": mechanical_speed,
                          "direction": direction,
                          "emf_mv": min(mechanical_speed * 19, 5700),
                          "max_angle_error_deg": round(max(errors), 3)})
    return {"cases": cases, "signal_loss_cases": len(cases)}


def check_sensorless_handover(elf_path):
    for direction in (1, -1):
        emu = Emulator(elf_path)
        observer, config, control = SCRATCH, SCRATCH + 64, SCRATCH + 160
        emu.call("Smo_Reset", observer)
        emu.call("Smo_ConfigDefault", config)
        emu.write32(config + 8, 1600)
        phase, speed_q8 = 16300, direction * 50000
        advance = int(speed_q8 / 256)
        emu.call("Sensorless_Start", control, phase)

        def step(emf=300, ready=True, enabled=True, mismatch=0, phase_offset=400,
                 phase_error=0, valid_frames=32):
            nonlocal phase
            phase = (phase + advance) & 16383
            emu.write32(observer + 16, ((phase - advance + phase_offset) & 16383) * 256)
            emu.write32(observer + 20, speed_q8 + mismatch)
            emu.cpu.mem_write(observer + 32, struct.pack("<HH", emf, valid_frames))
            emu.write32(observer + 28, phase_error)
            emu.cpu.mem_write(observer + 40, bytes([ready]))
            selected = emu.call("Sensorless_Step", control, observer, config,
                                phase, speed_q8, int(enabled))
            return selected, emu.read32(control + 20), struct.unpack(
                "<H", emu.cpu.mem_read(control + 18, 2))[0]

        for _ in range(80):
            assert step(ready=False)[1:] == (2, 0), "unqualified observer takeover"
        for _ in range(80):
            assert step(mismatch=100000)[1:] == (2, 0), "inconsistent observer speed takeover"
        for _ in range(80):
            assert step(phase_offset=3000)[1:] == (2, 0), "inconsistent phase takeover"
        for _ in range(80):
            assert step(phase_offset=1100)[1:] == (2, 0), "handover phase window"
        for _ in range(62):
            assert step()[1:] == (2, 0), "takeover qualification duration"
        selected, source, blend = step()
        assert source == 3 and blend == 4, "EMF-qualified takeover did not start"
        assert abs((selected - phase + 8192) % 16384 - 8192) <= 16, "takeover angle step"
        for _ in range(20):
            selected, source, blend = step()
        assert blend == 84, "SMO blend timing"
        for _ in range(43):
            selected, source, blend = step()
        assert source == 4 and blend == 256, "SMO takeover did not complete"
        for _ in range(40):
            selected, source, blend = step(ready=False, phase_error=1000, valid_frames=0)
            assert source == 4 and blend == 256, "brief PLL loss must remain on SMO"
        # The foreground seeds the fallback field from the last selected angle.
        phase = selected
        for _ in range(24):
            selected, source, blend = step(emf=150)
            assert source == 4 and blend == 256, (
                f"short EMF dip must not cause fallback: source={source}, blend={blend}, "
                f"phase={phase}, selected={selected}"
            )
        previous_phase = selected
        selected, source, blend = step(emf=150)
        assert source == 5 and blend == 252, "sustained EMF loss did not start fallback"
        assert selected == (previous_phase + advance) & 16383, "fallback must continue the actual phase"
        phase = selected
        for _ in range(63):
            selected, source, blend = step(emf=150)
        assert source == 2 and blend == 0, "HFI fallback did not complete"
        for _ in range(300):
            assert step()[1:] == (2, 0), "fallback retry delay"
        for _ in range(160):
            selected, source, blend = step()
        assert source == 4, "SMO re-entry after fallback"
        phase = selected
        previous_phase = selected
        selected, source, blend = step(phase_offset=6000)
        assert source == 5 and blend == 252, "SMO angle discontinuity must fall back immediately"
        assert selected == (previous_phase + advance) & 16383, "fallback must retain actual direction"
        phase = selected
        for _ in range(62):
            _, source, blend = step()
            assert source == 5, "angle discontinuity fallback ramp"
        _, source, blend = step()
        assert source == 2 and blend == 0, "angle discontinuity fallback completion"
        for _ in range(100):
            assert step()[1:] == (2, 0), "angle discontinuity retry delay"
        for _ in range(400):
            selected, source, blend = step()
        assert source == 4, "SMO re-entry after angle discontinuity"
        for _ in range(80):
            _, source, _ = step(mismatch=100000)
            assert source == 4, "tracking a slower speed reference must retain SMO"
        emu.call("Sensorless_Start", control, phase)
        for frame in range(240):
            _, source, blend = step(phase_offset=800 if frame & 1 else 1200)
        assert source == 4 and blend == 256, "phase jitter prevented qualified takeover"
    return {"directions": 2, "takeovers": 8, "fallbacks": 4}


def check_hfi_current_handover(elf_path):
    for direction in (1, -1):
        emu = Emulator(elf_path)
        motion = SCRATCH
        emu.call("Motion_Init", motion)
        emu.write32(motion, 7)
        emu.write32(motion + 4, direction * 15000)
        emu.write32(motion + 72, 400)
        emu.write32(motion + 100, direction * 15000)
        emu.call("Motion_SetSensorless", motion, 0, 0)

        previous_d = 400
        for blend in range(0, 257, 4):
            emu.call("Motion_SetSensorless", motion, 0, blend)
            emu.call("Motion_Step", SCRATCH + 256, motion, 0, direction * 15000, 0, 0)
            d_current = struct.unpack("<i", emu.cpu.mem_read(motion + 72, 4))[0]
            q_current = struct.unpack("<i", emu.cpu.mem_read(motion + 88, 4))[0]
            assert 0 <= d_current <= 400, f"D-axis target out of range during {direction} handover"
            assert abs(q_current) <= 500, f"Q-axis current exceeds limit during {direction} handover"
            assert struct.unpack("<i", emu.cpu.mem_read(motion + 76, 4))[0] == 0, (
                "HFI field current must not become a torque command")
            assert d_current <= previous_d + 4, "D-axis current reference must not jump upward"
            previous_d = d_current
        assert previous_d == 399, f"HFI field must remain during angle handover: {previous_d}"
        assert emu.read32(motion + 80) == 500, "handover must preserve motor current limit"
        for _ in range(399):
            emu.call("Motion_Step", SCRATCH + 256, motion, 0, direction * 15000, 0, 0)
        assert emu.read32(motion + 72) == 0, "startup field did not fade after SMO takeover"

        previous_d = 0
        for blend in range(256, -1, -4):
            emu.call("Motion_SetSensorless", motion, 0, blend)
            emu.call("Motion_Step", SCRATCH + 256, motion, 0, direction * 15000, 0, 0)
            d_current = struct.unpack("<i", emu.cpu.mem_read(motion + 72, 4))[0]
            q_current = struct.unpack("<i", emu.cpu.mem_read(motion + 88, 4))[0]
            assert d_current >= previous_d - 4, "fallback D-axis current must ramp back"
            assert struct.unpack("<i", emu.cpu.mem_read(motion + 76, 4))[0] == 0, (
                "fallback must not create a torque-current command")
            assert abs(q_current) <= 500, "fallback Q-axis current exceeds limit"
            previous_d = d_current
        assert 0 < previous_d <= 400, "HFI fallback field reference must ramp within limits"

        for _ in range(100):
            emu.call("Motion_Step", SCRATCH + 256, motion, 0, direction * 15000, 0, 0)
        assert struct.unpack("<i", emu.cpu.mem_read(motion + 72, 4))[0] == 400
        assert struct.unpack("<i", emu.cpu.mem_read(motion + 88, 4))[0] == 0

    emu = Emulator(elf_path)
    motion = SCRATCH
    emu.call("Motion_Init", motion)
    emu.write32(motion, 7)
    emu.write32(motion + 4, 1000)
    emu.write32(motion + 100, 1000)
    emu.call("Motion_SetSensorless", motion, 1, 256)
    for _ in range(4):
        emu.call("Motion_Step", SCRATCH + 256, motion, 0, 0, 0, 0)
        assert emu.read32(motion + 132) == 0, "speed PI updated before its 8 ms interval"
    emu.call("Motion_Step", SCRATCH + 256, motion, 0, 0, 0, 0)
    assert emu.read32(motion + 132) == 160, "sensorless speed PI must integrate over five 1.6 ms frames"
    return {"directions": 2, "current_limit_ma": 500,
            "handover_steps": 65, "fallback_steps": 65, "speed_pi_interval_ms": 8}


def check_control_instruction_counts(elf_path):
    emu = Emulator(elf_path)
    hfi, currents = SCRATCH, SCRATCH + 64
    emu.cpu.mem_write(hfi, bytes(32))
    emu.cpu.mem_write(currents, struct.pack("<8h", 40, -25, -40, 25, -30, 20, 30, -20))
    hfi_count = emu.count_call("Hfi_Update", hfi, currents, 0)

    smo, config, inputs = SCRATCH + 128, SCRATCH + 192, SCRATCH + 256
    emu.call("Smo_Reset", smo)
    emu.call("Smo_ConfigDefault", config)
    emu.write32(config + 8, 1600)
    emu.call("Smo_Prepare", config)
    emu.cpu.mem_write(inputs, struct.pack("<4i", 800, -300, 120, -80))
    smo_count = emu.count_call("Smo_Update", smo, config, inputs)
    # Exercise the two phase calculations used after EMF becomes visible.
    emu.cpu.mem_write(smo + 8, struct.pack("<4i", -300, 500, 1200 * 256, 18000))
    emu.cpu.mem_write(smo + 32, struct.pack("<HH", 500, 32))
    smo_visible_count = emu.count_call("Smo_Update", smo, config, inputs)

    motion, output = SCRATCH + 320, SCRATCH + 512
    emu.call("Motion_Init", motion)
    emu.write32(motion, 7)
    emu.write32(motion + 4, 15000)
    emu.write32(motion + 100, 15000)
    emu.call("Motion_SetSensorless", motion, 1, 256)
    motion_count = emu.count_call("Motion_Step", output, motion, 0, 15000, 100, -20)
    angle_counts = [emu.count_call("FOC_Angle", output, phase)
                    for phase in (0, 1000, 4095, 8193, 15000)]
    assert min(hfi_count, smo_count, motion_count) > 0
    return {
        "hfi_update_instructions": hfi_count,
        "smo_update_instructions": smo_count,
        "smo_visible_update_instructions": smo_visible_count,
        "foc_angle_max_instructions": max(angle_counts),
        "motion_step_instructions": motion_count,
        "interpretation": "dynamic Cortex-M0 instruction counts, not target cycles",
    }


def check_sensorless_fast_control(elf_path):
    emu = Emulator(elf_path)
    observer, config, control = SCRATCH, SCRATCH + 64, SCRATCH + 160
    for direction in (1, -1):
        emu.call("Smo_Reset", observer)
        emu.call("Smo_ConfigDefault", config)
        emu.write32(config + 8, 1600)
        speed = direction * 224304
        emu.call("Smo_Seed", observer, 15000, speed)
        selected = (emu.call("Smo_Angle", observer) + int(speed / 256)) & 16383
        emu.call("Smo_SetSamplePeriod", observer, config, 200, selected)
        rescaled = struct.unpack("<i", emu.cpu.mem_read(observer + 20, 4))[0]
        assert rescaled == int(speed / 8)
        predicted = (emu.call("Smo_Angle", observer) + int(rescaled / 256)) & 16383
        assert abs((predicted - selected + 8192) % 16384 - 8192) <= 1, "rate-change angle jump"
        emu.call("Smo_SetSamplePeriod", observer, config, 1600, selected)
        assert emu.read32(config + 8) == 1600

    emu.call("Smo_Reset", observer)
    emu.write32(config + 8, 200)
    phase, speed = 15000, 280380
    emu.call("Sensorless_Start", control, phase)
    emu.cpu.mem_write(control + 18, struct.pack("<H", 256))
    emu.write32(control + 20, 4)
    emu.write32(control + 24, speed)

    def tracked_frame(bad_phase=False):
        nonlocal phase
        emu.write32(observer + 16, ((phase + (8192 if bad_phase else 0)) & 16383) * 256)
        emu.write32(observer + 20, speed)
        emu.write32(observer + 28, 0)
        emu.cpu.mem_write(observer + 32, struct.pack("<HH", 5700, 32))
        # The requested field is already reversing while the rotor is positive.
        selected = emu.call("Sensorless_Step", control, observer, config,
                            (phase - speed // 256) & 16383, -speed, 1)
        source = emu.read32(control + 20)
        if source != 6:
            assert selected == (phase + speed // 256) & 16383
        phase = selected
        return source

    for _ in range(100):
        assert tracked_frame() == 4, "speed-reference reversal withdrew SMO"
    assert tracked_frame(True) == 4, "one bad high-speed phase must be rejected"
    assert tracked_frame() == 4 and emu.read32(control + 8) == 0
    for _ in range(199):
        assert tracked_frame(True) == 4, "brief high-speed observer loss"
    assert tracked_frame(True) == 6, "high-speed loss must not start blind HFI"

    for direction, phase_shift, settling in ((1, 1200, False), (-1, 1200, False),
                                            (1, 3000, True), (-1, 3000, True)):
        emu.call("Sensorless_Start", control, 15000)
        emu.cpu.mem_write(control + 18, struct.pack("<H", 256))
        emu.write32(control + 20, 4)
        velocity = direction * 56076
        emu.write32(control + 24, velocity)
        emu.write32(control + 28, 40000 if settling else 0)
        physical_phase = selected = 15000
        for _ in range(220):
            emu.write32(observer + 16, ((physical_phase + direction * phase_shift) & 16383) * 256)
            emu.write32(observer + 20, velocity)
            emu.write32(observer + 28, 0)
            emu.cpu.mem_write(observer + 32, struct.pack("<HH", 1000, 0 if settling else 32))
            previous = selected
            selected = emu.call("Sensorless_Step", control, observer, config,
                                physical_phase, velocity, 1)
            physical_phase = (physical_phase + int(velocity / 256)) & 16383
            correction = (selected - previous - int(velocity / 256) + 8192) % 16384 - 8192
            assert abs(correction) <= 16, "resampling correction jumped the control angle"
            assert emu.read32(control + 20) == 4 and emu.read32(control + 8) == 0, (
                "qualified sampling-delay change latched observer loss")
        assert emu.read32(control) == 0, "resampling phase offset did not converge"

    motion, output = SCRATCH, SCRATCH + 256
    emu.call("Motion_Init", motion)
    emu.write32(motion, 7)
    emu.write32(motion + 4, 1000000)
    for _ in range(1000):
        emu.call("Motion_HfiAdvance", motion)
    assert emu.read32(motion + 100) == 100000, "startup must await fast sampling"
    assert emu.read32(motion + 148) == 100000, "default acceleration"
    emu.write32(motion + 100, 1000000)
    emu.call("Motion_HfiAdvance", motion)
    assert emu.read32(motion + 136) > 2097152, "full signed speed must survive phase wrap"
    emu.call("Motion_SetSamplePeriod", motion, 200)
    emu.write32(motion + 100, 1000000)
    emu.call("Motion_HfiAdvance", motion)
    assert emu.read32(motion + 136) == 934600, "high-speed phase multiplication overflow"
    emu.write32(motion + 4, -1000000)
    for _ in range(100):
        emu.call("Motion_HfiAdvance", motion)
    assert emu.read32(motion + 100) == 998000, "reverse command must retain deceleration ramp"
    emu.write32(motion + 148, 1000000)
    for _ in range(100):
        emu.call("Motion_HfiAdvance", motion)
    assert emu.read32(motion + 100) == 978000, "runtime acceleration change"
    for sample_us, frames in ((200, 5000), (1600, 625)):
        for acceleration in (7500, 100000):
            emu.call("Motion_Init", motion)
            emu.write32(motion + 4, 1000000)
            emu.write32(motion + 148, acceleration)
            emu.call("Motion_SetSamplePeriod", motion, sample_us)
            for _ in range(frames):
                emu.call("Motion_HfiAdvance", motion)
            assert emu.read32(motion + 100) == acceleration, "acceleration depends on sampling rate"
    emu.call("Motion_Reset", motion, 0, 0)
    assert emu.read32(motion + 148) == 100000, "stop erased the ramp setting"

    for direction in (1, -1):
        emu.call("Motion_Init", motion)
        emu.write32(motion, 1)
        emu.write32(motion + 4, direction * 2000)
        emu.call("Motion_SetLimits", motion, 2000, 6928)
        emu.write32(motion + 16, 0)
        emu.write32(motion + 20, 0)
        emu.write32(motion + 24, 3000000)
        emu.call("Motion_Step", output, motion, 0, 0, 0, 0)
        d, q = struct.unpack("<2i", emu.cpu.mem_read(output, 8))
        assert d == 3000 and direction * q == isqrt(6928**2 - d**2), "circular voltage limit"
        assert d*d + q*q <= 6928**2
        emu.cpu.mem_write(motion + 146, struct.pack("<H", 500))
        emu.call("Motion_Step", output, motion, 0, 0, 0, 0)
        d, q = struct.unpack("<2i", emu.cpu.mem_read(output, 8))
        assert d*d + q*q <= 6428**2, "HFI injection voltage headroom"
        assert d + abs(q) > 6428, "old L1 limiter still wastes voltage"

    emu.call("Motion_Init", motion)
    emu.write32(motion, 7)
    emu.write32(motion + 100, 1000)
    emu.call("Motion_SetSamplePeriod", motion, 200)
    emu.call("Motion_SetSensorless", motion, 1, 256)
    for _ in range(39):
        emu.call("Motion_Step", output, motion, 0, 0, 0, 0)
        assert emu.read32(motion + 132) == 0
    emu.call("Motion_Step", output, motion, 0, 0, 0, 0)
    assert emu.read32(motion + 132) == 160, "fast SMO speed PI must retain its 8 ms period"
    emu.write32(motion + 100, 100000)
    emu.write32(motion + 132, 123000)
    for _ in range(80):
        emu.write32(motion + 140, 1)
        emu.call("Motion_Step", output, motion, 0, 0, 0, 0)
    assert emu.read32(motion + 132) == 123000, "voltage saturation wound up the speed integral"
    emu.write32(motion + 100, -100000)
    for _ in range(40):
        emu.write32(motion + 140, 1)
        emu.call("Motion_Step", output, motion, 0, 0, 0, 0)
    assert emu.read32(motion + 132) == 118200, "braking must unwind the speed integral"
    return {"resample_directions": 2, "high_speed_rad_s": 1000,
            "acceleration_rad_s2": 100, "voltage_limit_mv": 6928,
            "fast_control_us": 200, "speed_pi_us": 8000, "high_speed_loss_frames": 200}


def check_smo_continuous_weak_field(elf_path):
    results = []
    for speed, id_ma in ((300, 0), (400, -600), (450, -1200)):
        for direction in (1, -1):
            emu = Emulator(elf_path)
            observer, config, inputs = SCRATCH, SCRATCH + 64, SCRATCH + 128
            emu.call("Smo_Reset", observer)
            emu.call("Smo_ConfigDefault", config)
            emu.write32(config + 8, 200)
            emu.write32(config + 20, 1)
            emu.call("Smo_Prepare", config)
            omega = direction * speed * 7
            advance = omega * .0002
            emu.call("Smo_Seed", observer, 0, round(advance * 16384 * 256 / (2*pi)))
            errors = []
            for index in range(1600):
                angle = index * advance
                iq = direction * 100
                ia = id_ma * cos(angle) - iq * sin(angle)
                ib = id_ma * sin(angle) + iq * cos(angle)
                emf = direction * speed * 19
                # Analytic continuous R/L plant. Its derivative does not
                # reuse the observer's discretization.
                va = 3.7 * ia - .001 * omega * ib - emf * sin(angle)
                vb = 3.7 * ib + .001 * omega * ia + emf * cos(angle)
                emu.cpu.mem_write(inputs, struct.pack("<4i", round(va), round(vb), round(ia), round(ib)))
                valid = emu.call("Smo_Update", observer, config, inputs)
                if index >= 1000:
                    phase = emu.call("Smo_Angle", observer)
                    error = (phase - angle * 16384 / (2*pi) + 8192) % 16384 - 8192
                    errors.append(abs(error) * 360 / 16384)
                    assert valid, "continuous weak-field observer lost lock"
            assert max(errors) < 6, f"continuous weak-field phase error {max(errors):.2f} deg"
            results.append({"speed_rad_s": direction * speed, "id_ma": id_ma,
                            "maximum_phase_error_deg": round(max(errors), 3)})
    return {"cases": results}


def check_field_weakening(elf_path):
    for sample_us in (200, 1600):
        emu = Emulator(elf_path)
        motion, output = SCRATCH, SCRATCH + 256
        emu.call("Motion_Init", motion)
        emu.write32(motion, 7)
        emu.write32(motion + 72, 400)
        emu.call("Motion_SetSamplePeriod", motion, sample_us)
        emu.call("Motion_SetSensorless", motion, 1, 256)
        for _ in range(640000 // sample_us - 1):
            emu.call("Motion_Step", output, motion, 0, 15000, 400, 0)
        assert emu.read32(motion + 72) == 1, "sampling-rate-dependent startup-field fade"
        emu.call("Motion_Step", output, motion, 0, 15000, 400, 0)
        assert emu.read32(motion + 72) == 0
    for direction in (1, -1):
        emu = Emulator(elf_path)
        motion, output = SCRATCH, SCRATCH + 256
        budget_roots = 0

        def count_budget_roots(*_):
            nonlocal budget_roots
            if emu.args(1)[0] <= 2000**2:
                budget_roots += 1

        sqrt_address = emu.address("FOC_SqrtLowerBound")
        hook = emu.cpu.hook_add(UC_HOOK_CODE, count_budget_roots, begin=sqrt_address, end=sqrt_address)
        emu.call("Motion_Init", motion)
        emu.write32(motion, 7)
        emu.write32(motion + 4, direction * 400000)
        emu.write32(motion + 100, direction * 400000)
        emu.call("Motion_SetLimits", motion, 2000, 6928)
        emu.call("Motion_SetSamplePeriod", motion, 200)
        emu.call("Motion_SetSensorless", motion, 1, 256)

        def signed(address):
            return struct.unpack("<i", emu.cpu.mem_read(address, 4))[0]

        def step(speed, saturated=True):
            # Supply a controlled voltage-utilization observation. This tests
            # the actual ARM regulator, independently of the motor plant.
            emu.cpu.mem_write(motion + 164, struct.pack("<hh", 0, 6928 if saturated else 4000))
            old_id = signed(motion + 72)
            iq = signed(motion + 88)
            emu.call("Motion_Step", output, motion, 0, speed, old_id, iq)
            id_ref, iq_ref = signed(motion + 72), signed(motion + 88)
            assert id_ref**2 + iq_ref**2 <= 2000**2, "field weakening exceeded total current"
            iq_limit = struct.unpack("<H", emu.cpu.mem_read(motion + 162, 2))[0]
            assert id_ref**2 + iq_limit**2 <= 2000**2, "cached current budget exceeds circle"
            assert -1 <= id_ref - old_id <= 4, "field weakening changed Id discontinuously"
            d, q = struct.unpack("<ii", emu.cpu.mem_read(output, 8))
            assert d*d + q*q <= 6928**2, "field weakening exceeded voltage circle"
            return id_ref

        budget_roots = 0
        try:
            for _ in range(2000):
                last_id = step(direction * 360000)
        finally:
            emu.cpu.hook_del(hook)
        assert 0 < budget_roots <= 100, (
            f"negative-Id budget used {budget_roots} roots in 2000 frames, expected at most 100")
        assert -1200 <= last_id < -100, "high voltage did not produce negative Id"
        emu.call("Motion_SetWeakening", motion, 0, 950, 1000)
        for _ in range(2000):
            last_id = step(direction * 360000)
        assert last_id == 0 and signed(motion + 152) == 0, "runtime disable did not release weak field"
        emu.call("Motion_SetWeakening", motion, 1200, 950, 1000)
        emu.write32(motion + 152, -1200 * 64)
        for _ in range(1600):
            step(direction * 360000)
        assert signed(motion + 72) == -1200, "negative Id maximum"
        assert 1596 <= struct.unpack("<H", emu.cpu.mem_read(motion + 162, 2))[0] <= 1600
        for _ in range(4000):
            last_id = step(direction * 100000, False)
        assert last_id == 0, "deceleration did not restore zero Id"
        emu.call("Motion_SetWeakening", motion, 2000, 700, 10000)
        for _ in range(3000):
            last_id = step(direction * 360000)
        assert last_id == -2000, "full configured Id range"
        emu.call("Motion_SetWeakening", motion, 0, 950, 1000)
        for _ in range(5200):
            last_id = step(direction * 360000)
        assert last_id == 0, "full-current release did not restore zero Id"
        emu.call("Motion_SetWeakening", motion, 1200, 950, 1000)
        emu.call("Motion_Reset", motion, 0, 0)
        assert signed(motion + 152) == 0
        assert struct.unpack("<3H", emu.cpu.mem_read(motion + 156, 6)) == (1200, 950, 1000)
    return {"directions": 2, "total_current_ma": 2000, "maximum_negative_id_ma": 2000,
            "budget_update_hz": 250,
            "runtime_disable": True, "low_speed_release": True}


def check_sensorless_runtime(elf_path, startup_speed=120000):
    """Replay commands through main with ideal current/observer input.

    The actual motor plant and ADC paths have separate checks. This replay
    catches application-level gating, rate changes and command sequencing.
    """
    emu = Emulator(elf_path, reset=True)
    cpu = emu.cpu
    rtt = emu.address("_SEGGER_RTT")
    up, down = rtt + 24, rtt + 72
    clock = sequence = conversion_time = frame_sequence = 0
    next_diagnostic = 1000
    diagnostic_pause_until = 0
    next_frame = 0
    period = 1600
    active = False
    motion = control = None
    decoder = FrameDecoder()
    frames, rate_changes, enables, fast_entry_fields = [], [], [], []
    commands = [(200, f"@Y,{startup_speed};".encode()), (400, b"@E;"),
                (4_000_000, b"@A,100000;"),
                (5_000_000, b"@Y,-30000;"), (12_000_000, b"@X;")]

    def signed(address):
        return struct.unpack("<i", cpu.mem_read(address, 4))[0]

    def capture_pointer(name, setter):
        address = emu.address(name)
        cpu.hook_add(UC_HOOK_CODE, lambda *_: setter(emu.args(1)[0]), begin=address, end=address)

    def set_motion(address):
        nonlocal motion
        motion = address

    def set_control(address):
        nonlocal control
        control = address

    capture_pointer("Motion_Init", set_motion)
    capture_pointer("Sensorless_Start", set_control)

    def raw_current():
        if motion is None or control is None:
            return 2048, 2050
        phase = struct.unpack("<H", cpu.mem_read(control + 16, 2))[0] * 2 * pi / 16384
        d, q = signed(motion + 72), signed(motion + 88)
        ia, beta = d * cos(phase) - q * sin(phase), d * sin(phase) + q * cos(phase)
        ib = (-ia + sqrt(3) * beta) / 2
        ic = -ia - ib
        return 2048 - round(ib * 4096 / 6602), 2050 - round(ic * 4096 / 6602)

    def diagnostic():
        nonlocal clock, sequence, conversion_time, next_diagnostic
        clock += 200
        write, read = emu.read32(up + 12), emu.read32(up + 16)
        size, buffer = emu.read32(up + 8), emu.read32(up + 4)
        data = bytes(cpu.mem_read(buffer + (read + i) % size, 1)[0]
                     for i in range((write - read) % size))
        frames.extend((clock, value) for value in decoder.feed(data))
        emu.write32(up + 16, write)
        if commands and clock >= commands[0][0]:
            _, command = commands.pop(0)
            offset = emu.read32(down + 12)
            size, buffer = emu.read32(down + 8), emu.read32(down + 4)
            for byte in command:
                cpu.mem_write(buffer + offset, bytes([byte]))
                offset = (offset + 1) % size
            emu.write32(down + 12, offset)
        if clock >= next_diagnostic and clock >= diagnostic_pause_until:
            sequence += 1
            conversion_time = clock
            next_diagnostic = clock + (2000 if active else 1000)
        b, c, timestamp = emu.args(3)
        rb, rc = raw_current()
        cpu.mem_write(b, struct.pack("<H", rb))
        cpu.mem_write(c, struct.pack("<H", rc))
        emu.write32(timestamp, conversion_time)
        emu.result(sequence)
        if clock >= 12_020_000:
            cpu.emu_stop()

    def start_hfi():
        nonlocal active, period, next_frame, frame_sequence
        active, period, frame_sequence = True, 1600, 0
        next_frame = clock + period

    def cancel_hfi():
        nonlocal active
        active = False

    def change_rate():
        nonlocal period, next_frame, diagnostic_pause_until
        fast = bool(emu.args(1)[0])
        if fast:
            fast_entry_fields.append(signed(motion + 72))
        period = 200 if fast else 1600
        next_frame = clock + period
        if not fast:
            diagnostic_pause_until = clock + 10000
        rate_changes.append((clock, fast))

    def current_frame():
        nonlocal frame_sequence, next_frame
        if clock < next_frame:
            emu.result(0)
            return
        next_frame += period
        frame_sequence += 1
        rb, rc = raw_current()
        pointer = emu.args(1)[0]
        cpu.mem_write(pointer, struct.pack("<I8h2HiiH", frame_sequence, *([0] * 8),
                                           rb, rc, 0, 0, period))
        emu.result(1)

    def observer():
        pointer, config, _ = emu.args(3)
        sample_us = emu.read32(config + 8)
        velocity = signed(motion + 100)
        speed_q8 = round(velocity * sample_us * 4673 / 1_000_000)
        phase = struct.unpack("<H", cpu.mem_read(control + 16, 2))[0]
        emf = abs(velocity) * 19 // 1000
        emu.write32(pointer + 16, phase * 256)
        emu.write32(pointer + 20, speed_q8)
        emu.write32(pointer + 28, 0)
        cpu.mem_write(pointer + 32, struct.pack("<HH", emf, 32 if emf >= 120 else 0))
        cpu.mem_write(pointer + 40, bytes([emf >= 400]))
        emu.result(int(emf >= 120))

    emu.stub("BSP_Init", lambda: None)
    emu.stub("BoardTime_NowUs", lambda: emu.result(clock))
    emu.stub("BSP_StopRequested", lambda: emu.result(0))
    emu.stub("BSP_StopPressed", lambda: emu.result(0))
    emu.stub("CurrentAdc_Next", diagnostic)
    emu.stub("MT6701_ReadAngle", lambda: (
        cpu.mem_write(emu.args(1)[0], struct.pack("<H", 8384)), emu.result(1)))
    emu.stub("MT6701_BeginRead", lambda: None)
    emu.stub("MT6701_PollAngle", lambda: (
        cpu.mem_write(emu.args(1)[0], struct.pack("<H", 8384)), emu.result(1)))
    emu.stub("DRV8313_SetEnabled", lambda: enables.append((clock, emu.args(1)[0])))
    emu.stub("DRV8313_SetDuty", lambda: None)
    emu.stub("CurrentAdc_StartHfi", start_hfi)
    emu.stub("CurrentAdc_CancelPulse", cancel_hfi)
    emu.stub("CurrentAdc_SetHfiDuty", lambda: None)
    emu.stub("CurrentAdc_SetSmoDuty", lambda: None)
    emu.stub("CurrentAdc_SetSmoMode", change_rate)
    emu.stub("CurrentAdc_PulseActive", lambda: emu.result(int(active)))
    emu.stub("CurrentAdc_ReadHfi", current_frame)
    emu.stub("Hfi_Update", lambda: emu.result(1))
    emu.stub("Smo_Update", observer)
    stack, entry = struct.unpack("<II", cpu.mem_read(0, 8))
    cpu.reg_write(UC_ARM_REG_SP, stack)
    cpu.reg_write(UC_ARM_REG_LR, RETURN | 1)
    cpu.emu_start(entry, RETURN, count=300_000_000)
    assert clock == 12_020_000, f"runtime replay stopped at {clock} us"
    reverse = [status for timestamp, status in frames if 5_050_000 <= timestamp <= 5_200_000]
    assert reverse and all(s.target == -30000 and s.control_source == 4 for s in reverse), (
        "reverse command immediately withdrew SMO")
    assert all(s.speed_mrad_s > 0 for s in reverse), "requested reversal changed speed before deceleration"
    assert [fast for _, fast in rate_changes] == [True, False], f"unexpected rate transitions {rate_changes}"
    assert fast_entry_fields and (fast_entry_fields[0] > 0 if startup_speed >= 100000
                                  else fast_entry_fields[0] == 0), "startup-field fast-mode gate"
    assert not any(1_000_000 < timestamp < 12_000_000 and not value for timestamp, value in enables), (
        "mode handover disabled the drive")
    assert any(timestamp > 11_000_000 and s.control_source == 4 and s.speed_mrad_s < -29000
               for timestamp, s in frames), "reverse rotation did not reacquire SMO"
    assert frames[-1][1].state == 3 and not active, "runtime replay did not stop"
    assert all(s.missed_samples == 0 and s.result == 0 for _, s in frames)
    assert any(s.state == 4 and s.event == 3 for _, s in frames), (
        "diagnostic timeout must be reported while control remains running")
    assert frames[-1][1].event == 3, "sample-gap warning overwrote the diagnostic timeout"
    return {"simulated_us": clock, "rate_changes": len(rate_changes),
            "reverse_under_smo": True, "reverse_smo_reacquired": True,
            "diagnostic_pause_us": 10000, "diagnostic_timeout_keeps_control_running": True,
            "input_model": "ideal current and observer, actual application/control code"}


def check_fixed_point(elf_path):
    emu = Emulator(elf_path)
    quotients = (0, 1, 999, 1000, 1001, 31999999, 32000000, 32000001, 2147483647,
                 *range(0, 32000001, 7919))
    for absolute in quotients:
        for value in (absolute, -absolute):
            raw = emu.call("per_thousand", value)
            actual = struct.unpack("<i", struct.pack("<I", raw))[0]
            assert actual == int(value / 1000), f"PI quotient at {value}"
    max_error = 0
    for phase in range(0, 16384, 73):
        emu.call("FOC_Angle", SCRATCH, phase)
        s, c = struct.unpack("<ii", emu.cpu.mem_read(SCRATCH, 8))
        theta = phase * 2 * pi / 16384
        assert abs(s / 16384 - sin(theta)) < 0.0002, f"sine at phase {phase}"
        assert abs(c / 16384 - cos(theta)) < 0.0002, f"cosine at phase {phase}"
        d, q = 700, -1100
        emu.call("FOC_Modulate", SCRATCH, d, q, s, c, 12000, 1199)
        actual = struct.unpack("<HHH", emu.cpu.mem_read(SCRATCH, 6))
        alpha = d * cos(theta) - q * sin(theta)
        beta = d * sin(theta) + q * cos(theta)
        abc = (alpha, -alpha / 2 + sqrt(3) * beta / 2, -alpha / 2 - sqrt(3) * beta / 2)
        common = (12000 - min(abc) - max(abc)) / 2
        expected = [round((value + common) * 1199 / 12000) for value in abc]
        max_error = max(max_error, *(abs(a - b) for a, b in zip(actual, expected)))
    assert max_error <= 1, f"SVPWM error {max_error} compare counts"
    exact_cases = 0
    def truncate(value, divisor):
        return (abs(value) // divisor) * (-1 if value < 0 else 1)
    for bus in (7000, 12000, 20000):
        for pwm in (239, 1199, 2047, 61440):
            for alpha in range(-7000, 7001, 83):
                beta = (alpha * 53) % 14001 - 7000
                abc = (alpha, -truncate(alpha, 2) + truncate(beta * 14189, 16384),
                       -truncate(alpha, 2) - truncate(beta * 14189, 16384))
                common = truncate(bus - min(abc) - max(abc), 2)
                expected = tuple((max(0, min(bus, value + common)) * pwm + bus // 2) // bus
                                 for value in abc)
                emu.call("FOC_ModulateStationary", SCRATCH, alpha, beta, bus, pwm)
                actual = struct.unpack("<3H", emu.cpu.mem_read(SCRATCH, 6))
                assert actual == expected, f"SVPWM integer rounding at {alpha}, {beta}, {bus}, {pwm}"
                exact_cases += 1
    return {"max_compare_error": max_error, "exact_integer_cases": exact_cases}


def check_math_tables(elf_path):
    emu = Emulator(elf_path)
    for name in ("sine_quarter", "atan_octant", "atan_reciprocal", "sqrt_seed"):
        address, size = emu.symbols[name]
        assert 0 <= address < address + size <= 65536, f"{name} must stay in Flash"
    sine_error = 0
    for phase in range(16384):
        emu.call("FOC_Angle", SCRATCH, phase)
        actual = struct.unpack("<ii", emu.cpu.mem_read(SCRATCH, 8))
        theta = phase * 2 * pi / 16384
        expected = (round(16384 * sin(theta)), round(16384 * cos(theta)))
        sine_error = max(sine_error, *(abs(a - b) for a, b in zip(actual, expected)))
    assert sine_error == 0, f"direct Q14 sine/cosine lookup error {sine_error}"

    vectors = [(0, 0)]
    for scale in (1, 128, 2_000_000, 2**31 - 1):
        for phase in range(0, 16384, 31):
            theta = phase * 2 * pi / 16384
            vectors.append((round(scale * sin(theta)), round(scale * cos(theta))))
    for value in (-2**31, -1, 0, 1, 2**31 - 1):
        vectors.extend(((value, 0), (0, value), (value, value), (value, 2**31 - 1)))
    for index in range(512, 1024):
        for offset in (0, 63):
            x = index * 64 + offset
            vectors.extend((y, x) for y in (1, x // 4, x // 2, x - 1, x))
    angle_error = 0
    for y, x in vectors:
        actual = emu.call("FOC_Atan2", y, x)
        expected = round(atan2(y, x) * 8192 / pi)
        error = abs((actual - expected + 8192) % 16384 - 8192)
        angle_error = max(angle_error, error)
    assert angle_error <= 4, f"atan2 error {angle_error} electrical counts"

    roots = set(range(1025))
    for root in range(0, 65536, 127):
        roots.update((root * root - 1, root * root, root * root + 1))
    for power in range(33):
        roots.update((2**power - 1, 2**power, 2**power + 1))
    random = Random(32)
    roots.update(random.getrandbits(32) for _ in range(1024))
    roots = sorted(value for value in roots if 0 <= value < 2**32)
    for value in roots:
        actual = emu.call("FOC_Sqrt", value)
        assert actual == isqrt(value), f"integer sqrt at {value}: {actual}"
    return {"sine_phases": 16384, "sine_max_q14_error": sine_error,
            "atan_vectors": len(vectors), "atan_max_count_error": angle_error,
            "sqrt_values": len(roots), "sqrt_max_integer_error": 0}


def check_i2c(elf_path):
    cases = ("success", "immediate_start", "nack", "timeout", "async_success", "async_timeout")
    for case in cases:
        outcome = case.removeprefix("async_")
        emu = Emulator(elf_path)
        control, data, state = 0x40, 0, 0xF8
        pending = stopped = False
        index = now = 0
        states = (0x08, 0x18, 0x28, 0x10, 0x40, 0x50, 0x58)
        sent = []
        irq = emu.address("I2C_GetIrq")
        irq_end = irq + emu.symbols["I2C_GetIrq"][1]
        cr, dr, stat = 0x808, 0x80C, 0x814
        si, sta, sto, aa = 8, 32, 16, 4

        def read(machine, offset, _width, _context):
            nonlocal control, data, state, pending, index
            if offset == cr:
                if pending and irq <= machine.reg_read(UC_ARM_REG_PC) < irq_end and outcome != "timeout":
                    pending = False
                    state = 0x20 if outcome == "nack" and index == 1 else states[index]
                    if index == 5:
                        assert control & aa, "first byte should be ACKed"
                        data = 0xAB
                    elif index == 6:
                        assert not control & aa, "last byte should be NACKed"
                        data = 0xCD
                    control |= si
                    index += 1
                return control
            if offset == dr:
                return data
            assert offset == stat, f"unexpected I2C read {offset:#x}"
            return state

        def write(_machine, offset, _width, value, _context):
            nonlocal control, data, pending, stopped, index, state
            if offset == dr:
                sent.append(value)
                data = value
                return
            assert offset == cr, f"unexpected I2C write {offset:#x}"
            if value & sto:
                stopped = True
            elif outcome == "immediate_start" and index == 0 and value & sta:
                state = states[index]
                index += 1
                value |= si
            elif (not value & si and control & si) or (value & sta and not control & (sta | si)):
                pending = True
            control = value

        def clock():
            nonlocal now
            now += 50
            emu.result(now)

        emu.cpu.mmio_map(0x40005000, 0x1000, read, None, write, None)
        emu.stub("BoardTime_NowUs", clock)
        emu.cpu.mem_write(SCRATCH, b"\xff\xff")
        if case.startswith("async_"):
            emu.call("MT6701_BeginRead")
            for _ in range(20):
                result = emu.call("MT6701_PollAngle", SCRATCH)
                if result != 0:
                    break
            assert result != 0, "nonblocking I2C did not complete or time out"
            result = int(result == 1)
        else:
            result = emu.call("MT6701_ReadAngle", SCRATCH)
        assert stopped, f"I2C did not issue STOP after {outcome}"
        if outcome in ("success", "immediate_start"):
            assert sent == [0x0C, 3, 0x0D] and index == len(states), f"I2C sequence {sent}"
            assert result == 1 and emu.cpu.mem_read(SCRATCH, 2) == struct.pack("<H", 0xAB << 6 | 0xCD >> 2)
        else:
            assert result == 0 and emu.cpu.mem_read(SCRATCH, 2) == b"\xff\xff", outcome
    return {"cases": list(cases)}


def check_rtt_packets(elf_path):
    emu = Emulator(elf_path)
    cases = 0
    for channel, size in ((0, 128), (1, 512)):
        up = emu.address("_SEGGER_RTT") + 24 * (channel + 1)
        buffer = emu.read32(up + 4)
        for offset in (0, 4, size - 4, size - 1):
            for alignment in (0, 1):
                for length in (0, 1, 3, 4, 68, 72, 80, 96, size - 1):
                    payload = bytes((index * 19 + 7) & 255 for index in range(length))
                    emu.cpu.mem_write(SCRATCH + alignment, payload or b"\0")
                    emu.cpu.mem_write(buffer, b"\xa5" * size)
                    emu.write32(up + 12, offset)
                    emu.write32(up + 16, (offset + length + 1) % size)
                    emu.call("Rtt_Write", channel, SCRATCH + alignment, length)
                    expected = bytearray(b"\xa5" * size)
                    for index, value in enumerate(payload):
                        expected[(offset + index) % size] = value
                    assert emu.cpu.mem_read(buffer, size) == expected, "RTT packet or wrap copy"
                    assert emu.read32(up + 12) == (offset + length) % size, "RTT publication"
                    emu.write32(up + 16, (offset + length + 1) % size)
                    emu.call("Rtt_Write", channel, SCRATCH, 1)
                    assert emu.cpu.mem_read(buffer, size) == expected, "RTT full-buffer drop"
                    assert emu.read32(up + 12) == (offset + length) % size
                    cases += 1
    return {"packet_copy_and_full_buffer_cases": cases}


def check_trace_speed_analysis(_elf_path):
    from debug import trace_metrics
    from host.foc import TRACE_FIELDS
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "trace.jsonl"
        for speed in (15, -15, 300, -300):
            samples = []
            for index in range(100):
                sample = dict.fromkeys(TRACE_FIELDS, 0)
                sample.update(sequence=index * 2, time_us=index * 4000,
                              speed_mrad_s=speed * 1000,
                              raw_angle=round(-speed * index * .004 * 16384 / (2*pi)) & 16383,
                              electrical_angle=0, hfi_angle=65535, smo_angle=65535)
                samples.append(sample)
            path.write_text("\n".join(json.dumps(item) for item in samples), encoding="utf-8")
            result = trace_metrics(path, 2)
            assert abs(result["raw_speed_mean_rad_s"] - speed) < .1, "encoder window aliasing"
    return {"speed_rad_s": [15, -15, 300, -300]}


def check_bus_voltage(elf_path):
    emu = Emulator(elf_path)
    output = SCRATCH
    cases = 0
    for bus in (6000, 7001, 9000, 12000, 18000, 23999, 24000):
        emu.call("FOC_SetBusVoltage", bus)
        assert emu.call("FOC_GetBusVoltage") == bus
        assert emu.call("FOC_GetVoltageLimit") == int(bus / sqrt(3))
        for pwm_max in (1199, 61440):
            for fraction in range(-90, 91, 3):
                alpha = bus * fraction // 200
                beta = bus * (90 - abs(fraction)) // 300
                a = alpha
                b = int(-alpha / 2) + int(beta * 14189 / 16384)
                c = int(-alpha / 2) - int(beta * 14189 / 16384)
                common = int((bus - min(a, b, c) - max(a, b, c)) / 2)
                expected = [max(0, min(pwm_max, (v + common) * pwm_max / bus)) for v in (a, b, c)]
                emu.call("FOC_ModulateStationary", output, alpha, beta, bus, pwm_max)
                actual = struct.unpack("<3H", emu.cpu.mem_read(output, 6))
                assert all(abs(x - y) <= 1.1 for x, y in zip(actual, expected)), "dynamic bus PWM scaling"
                cases += 1
    for value in (0, 1, 2, 3, 4, 0xFFFFFFFF, *range(0, 4000001, 7919)):
        lower = emu.call("FOC_SqrtLowerBound", value)
        exact = isqrt(value)
        assert 0 <= exact - lower <= exact // 512 + 1, "current-budget root bound"
    return {"bus_range_mv": [6000, 24000], "modulation_cases": cases}


def check_low_speed_start(elf_path):
    return check_sensorless_runtime(elf_path, 60000)


def check_bus_sense(elf_path):
    emu = Emulator(elf_path)
    header = Path(__file__).resolve().parents[2] / "firmware/BSP/Inc/bus_sense.h"
    config = {name: int(value) for name, value in re.findall(
        r"#define BUS_SENSE_(\w+) (\d+)u", header.read_text(encoding="utf-8"))}
    default_mv = config["DEFAULT_MV"]
    full_mv = config["ADC_REFERENCE_MV"] * (config["TOP_OHM"] + config["BOTTOM_OHM"]) / config["BOTTOM_OHM"]
    normal_raw = round(default_mv * 4095 / full_mv)
    mid_raw = round((config["KEY_PRESS_PIN_MV"] + config["KEY_RELEASE_PIN_MV"]) * 4095 /
                    (2 * config["ADC_REFERENCE_MV"]))
    emu.call("BusSense_Update", 0, 0)
    assert emu.call("BusSense_Voltage") == default_mv
    assert not emu.call("BusSense_KeyPressed")
    for _ in range(80):
        emu.call("BusSense_Update", normal_raw, 1)
    before = emu.call("BusSense_Voltage")
    assert abs(before - normal_raw * full_mv / 4095) <= 2
    emu.call("BusSense_Update", 0, 1)
    assert not emu.call("BusSense_KeyPressed"), "KEY debounce"
    emu.call("BusSense_Update", 0, 1)
    assert emu.call("BusSense_KeyPressed")
    assert emu.call("BusSense_TakeKeyPress") and not emu.call("BusSense_TakeKeyPress")
    assert emu.call("BusSense_Voltage") == before, "KEY altered the supply estimate"
    emu.call("BusSense_Update", mid_raw, 1)
    assert emu.call("BusSense_KeyPressed"), "KEY hysteresis"
    emu.call("BusSense_Update", normal_raw, 1)
    assert not emu.call("BusSense_KeyPressed")
    emu.call("BusSense_Update", 0, 1)
    emu.call("BusSense_Update", normal_raw, 1)
    assert not emu.call("BusSense_TakeKeyPress"), "single near-zero glitch became KEY"
    for _ in range(80):
        emu.call("BusSense_Update", 4095, 1)
    assert abs(emu.call("BusSense_Voltage") - full_mv) <= 2
    emu.call("BusSense_Update", 0, 0)
    assert emu.call("BusSense_Voltage") == default_mv and not emu.call("BusSense_KeyPressed")
    return {"stub_mv": default_mv, "divider_ohm": [config["TOP_OHM"], config["BOTTOM_OHM"]], "key_debounce_samples": 2}


def run_checks(elf_path):
    checks = (check_startup_and_modes, check_adc_and_pwm, check_sensorless_sampler,
              check_fixed_point, check_math_tables, check_i2c,
              check_hfi_observer, check_smo_observer, check_sensorless_handover,
              check_hfi_current_handover, check_sensorless_fast_control,
              check_sensorless_runtime, check_control_instruction_counts,
              check_rtt_packets, check_trace_speed_analysis, check_field_weakening,
              check_smo_continuous_weak_field, check_bus_voltage, check_low_speed_start, check_bus_sense)
    results = []
    for check in checks:
        start = time.monotonic()
        result = {"name": check.__name__.removeprefix("check_")}
        try:
            result.update(ok=True, metrics=check(elf_path))
        except Exception as error:
            result.update(ok=False, error=f"{type(error).__name__}: {error}")
        result["seconds"] = round(time.monotonic() - start, 3)
        results.append(result)
    return results
