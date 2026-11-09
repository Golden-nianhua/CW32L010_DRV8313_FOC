# /// script
# requires-python = ">=3.12"
# dependencies = ["pyocd==0.46.0", "imageio-ffmpeg>=0.6", "scipy>=1.14"]
# ///

"""CW32L010 FOC console over SWD RTT."""

import argparse
from contextlib import contextmanager
import csv
from dataclasses import asdict, astuple, dataclass, fields
import json
from math import tau
import shlex
import struct
import sys
import time
from pathlib import Path

MAGIC = b"FO"
STATUS_FORMAT = struct.Struct("<HBB6BH9i3I3H2B2I")
TRACE_FORMAT = struct.Struct("<II2i5H5h2BHi4H2I2H4hhHhH")
TRACE_FIELDS = ("sequence", "time_us", "speed_mrad_s", "hfi_speed_mrad_s", "raw_angle",
                "electrical_angle", "hfi_angle", "adc_b", "adc_c", "id_ma", "iq_ma", "iq_ref_ma",
                "vd_mv", "vq_mv", "observer_flags", "control_source", "index", "smo_speed_q8",
                "smo_angle", "smo_emf_mv", "smo_error_counts", "smo_sample_us",
                "hfi_sequence", "hfi_missed_frames", "control_angle", "injection_mv",
                "observer_voltage_alpha_mv", "observer_voltage_beta_mv",
                "observer_current_alpha_ma", "observer_current_beta_ma",
                "id_ref_ma", "iq_limit_ma", "weakening_target_ma", "voltage_magnitude_mv")
CONTROL_SOURCES = ("none", "encoder", "hfi", "to_smo", "smo", "to_hfi", "observer_lost")
MODES = ("voltage", "current", "speed", "position", "impedance", "trajectory", "ratchet", "hfi")
STATES = ("unaligned", "calibrating", "aligning", "ready", "running", "probing", "hfi locking")
EVENTS = ("none", "encoder error", "sample gap", "sample timeout", "high current", "key stop", "HFI signal lost", "SMO signal lost")
RESULTS = ("ok", "invalid argument", "invalid state", "unknown command")
LOOPS = ("d", "q", "speed", "position", "sensorless-speed")
SMO_PARAMETERS = ("enable", "resistance-mohm", "inductance-uh", "gain-mv",
                  "boundary-ma", "filter-shift", "pll-kp", "pll-ki", "min-emf-mv",
                  "takeover-mv", "fallback-mv", "qualify-ms")
PYOCD_PROJECT = Path(__file__).resolve().parents[1] / "pyocd"


@dataclass
class Status:
    mode: int
    state: int
    event: int
    command: int
    result: int
    control_source: int
    ack: int
    target: int
    position_mrad: int
    speed_mrad_s: int
    id_ma: int
    iq_ma: int
    iq_ref_ma: int
    vd_mv: int
    vq_mv: int
    profile_mrad: int
    missed_samples: int
    encoder_errors: int
    sample_us: int
    bus_mv: int
    voltage_limit_mv: int
    supply_raw: int
    bus_source: int
    key_pressed: int
    audio_frames: int
    audio_underruns: int

    def text(self):
        mode = label(MODES, self.mode)
        unit = label(("mV", "mA", "mrad/s", "mrad", "mrad", "mrad", "teeth/rev", "mrad/s"), self.mode)
        reference = {"ratchet": "tooth", "hfi": "control_angle"}.get(mode, "profile")
        speed_name = "sensorless_speed" if mode == "hfi" else "speed"
        return (
            f"{label(STATES, self.state)} {mode} source={label(CONTROL_SOURCES, self.control_source)} "
            f"target={self.target} {unit} "
            f"pos={self.position_mrad / 1000:.3f} rad {speed_name}={self.speed_mrad_s / 1000:.3f} rad/s "
            f"({self.speed_mrad_s * 60 / (1000 * tau):.0f} rpm) "
            f"Id/Iq={self.id_ma}/{self.iq_ma} mA Iq_ref={self.iq_ref_ma} mA "
            f"Vd/Vq={self.vd_mv}/{self.vq_mv} mV {reference}={self.profile_mrad / 1000:.3f} rad "
            f"bus={self.bus_mv / 1000:.3f} V bus_source={label(('stub', 'manual', 'adc'), self.bus_source)} "
            f"KEY={self.key_pressed} audio={self.audio_frames}/{self.audio_underruns} Vlimit={self.voltage_limit_mv / 1000:.3f} V "
            f"sample={self.sample_us} us missed={self.missed_samples} "
            f"encoder_errors={self.encoder_errors} event={label(EVENTS, self.event)}"
        )


def label(names, index):
    return names[index] if index < len(names) else f"unknown({index})"


class FrameDecoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while len(self.buffer) >= 4:
            start = self.buffer.find(MAGIC)
            if start < 0:
                del self.buffer[:len(self.buffer) - 1]
                return
            del self.buffer[:start]
            if len(self.buffer) < 4:
                return
            version, size = self.buffer[2:4]
            if version != 4 or size != STATUS_FORMAT.size:
                del self.buffer[0]
                continue
            if len(self.buffer) < size:
                return
            values = STATUS_FORMAT.unpack_from(self.buffer)
            yield Status(*values[3:])
            del self.buffer[:size]


class Link:
    def __init__(self, up, down, timeout, trace_up=None, audio_down=None, directory=None):
        self.up = up
        self.down = down
        self.timeout = timeout
        self.trace_up = trace_up
        self.audio_down = audio_down
        self.directory = Path.cwd() if directory is None else directory
        self.decoder = FrameDecoder()
        self.latest = None

    def read(self):
        frames = list(self.decoder.feed(self.up.read()))
        if frames:
            self.latest = frames[-1]
        return frames

    def command(self, code, *values):
        self.read()
        baseline = self.latest.ack if self.latest else None
        payload = ("@" + ",".join((code, *map(str, values))) + ";").encode("ascii")
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if payload:
                payload = payload[self.down.write(payload):]
            for status in self.read():
                if not payload and status.command == ord(code) and status.ack != baseline:
                    if status.result != 0:
                        raise RuntimeError(f"{code}: {label(RESULTS, status.result)} ({status.text()})")
                    return status
            time.sleep(0.01)
        raise TimeoutError(f"no acknowledgement for {code}; check that the firmware is running")

    def wait_for(self, predicate, description):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            for status in self.read():
                if predicate(status):
                    return status
            time.sleep(0.01)
        raise TimeoutError(description)

    def collect(self, seconds=None):
        end = None if seconds is None else time.monotonic() + seconds
        last_frame = time.monotonic()
        received = False
        while end is None or time.monotonic() < end:
            frames = self.read()
            if frames:
                received = True
                last_frame = time.monotonic()
                yield from frames
            elif time.monotonic() - last_frame >= self.timeout:
                raise TimeoutError("RTT telemetry stopped")
            time.sleep(0.01)
        if not received:
            raise TimeoutError("no RTT telemetry received")

    def trace(self, count, divider, sample_rate=1000):
        if self.trace_up is None:
            raise RuntimeError("firmware RTT trace channel is unavailable")
        self.command("D", 0)
        self.trace_up.read()
        try:
            self.command("D", count, divider)
            deadline = time.monotonic() + count * divider / sample_rate + self.timeout
            next_status = 0.0
            data = bytearray()
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_status:
                    self.read()
                    next_status = now + 0.025
                data.extend(self.trace_up.read())
                complete = len(data) // TRACE_FORMAT.size * TRACE_FORMAT.size
                for values in TRACE_FORMAT.iter_unpack(data[:complete]):
                    sample = dict(zip(TRACE_FIELDS, values))
                    sample["smo_valid"] = sample["observer_flags"] & 1
                    sample["smo_ready"] = (sample["observer_flags"] >> 1) & 1
                    numerator = sample["smo_speed_q8"] * 1000000
                    speed = abs(numerator) // (4673 * sample["smo_sample_us"])
                    sample["smo_speed_mrad_s"] = -speed if numerator < 0 else speed
                    yield sample
                    if sample["index"] == count - 1:
                        return
                del data[:complete]
                time.sleep(0.001)
            raise TimeoutError("trace did not receive the final sample")
        finally:
            self.command("D", 0)


@contextmanager
def open_link(probe=None, frequency=None, timeout=2.0, target_state=None):
    from pyocd.core.helpers import ConnectHelper
    from pyocd.debug.rtt import RTTControlBlock

    directory = Path.cwd()
    options = {} if frequency is None else {"frequency": frequency}
    session = ConnectHelper.session_with_chosen_probe(
        project_dir=str(PYOCD_PROJECT), unique_id=probe, connect_mode="attach",
        blocking=False, **options,
    )
    if session is None:
        raise RuntimeError("no SWD probe found")
    with session:
        target = session.board.target
        if target_state is not None:
            target_state["state"] = target.get_state().name
            if target_state["state"] == "HALTED":
                target_state["registers"] = {
                    name: target.read_core_register(name) for name in ("pc", "sp", "lr")
                }
        rtt = RTTControlBlock.from_target(target, address=0x20000000, size=0x1000)
        rtt.start()
        if not rtt.up_channels or not rtt.down_channels:
            raise RuntimeError("firmware RTT channel 0 is unavailable")
        trace_up = rtt.up_channels[1] if len(rtt.up_channels) > 1 else None
        yield Link(rtt.up_channels[0], rtt.down_channels[0], timeout, trace_up,
                   rtt.down_channels[1] if len(rtt.down_channels) > 1 else None, directory)


def probe_inventory(selector=None):
    from pyocd.core.helpers import ConnectHelper

    return [
        {"id": probe.unique_id, "description": probe.description}
        for probe in ConnectHelper.get_all_connected_probes(blocking=False, unique_id=selector)
    ]


def select_probe(selector=None):
    probes = probe_inventory(selector)
    for probe in probes:
        if probe["id"] == selector:
            return selector
    if not probes:
        raise RuntimeError("no matching SWD probe found" if selector else "no SWD probe found")
    if len(probes) > 1:
        found = ", ".join(probe["id"] for probe in probes)
        raise RuntimeError(f"multiple SWD probes found: {found}; select with --probe")
    return probes[0]["id"]


def positive(text):
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("expected a positive finite number") from error
    if not 0 < value < float("inf"):
        raise argparse.ArgumentTypeError("expected a positive finite number")
    return value


def milli(text):
    try:
        value = round(float(text) * 1000)
    except (ValueError, OverflowError) as error:
        raise argparse.ArgumentTypeError("expected a finite number") from error
    if not -(2**31) <= value < 2**31:
        raise argparse.ArgumentTypeError("value outside protocol range")
    return value


def commands(parser, shell=False):
    subs = parser.add_subparsers(dest="action", required=True)
    for action in ("status", "enable", "stop", "clear"):
        subs.add_parser(action)
    if not shell:
        subs.add_parser("shell", help="keep one RTT session open for interactive commands")
    play = subs.add_parser("play", help="play an audio file through the motor at 48 kHz")
    play.add_argument("file", type=Path)
    play.add_argument("--seconds", type=positive, default=None, help="duration in seconds; omit to play to the end")
    play.add_argument("--start", type=float, default=0.0, help="start position in seconds")
    play.add_argument("--speed", type=milli, default=0, metavar="RAD_S", help="encoder speed, -30 to 30 rad/s; 0 holds still")
    play.add_argument("--amplitude", type=milli, default=5500, metavar="V", help="peak audio voltage, default 5.5 V, up to 6 V")
    play.add_argument("--transport", choices=("mulaw", "pcm16"), default="mulaw", help="RTT sample encoding, default mulaw")
    play.add_argument("--preemphasis-us", type=float, default=270.0, help="R/L high-frequency compensation, default 270 us")
    play.add_argument("--bass-db", type=float, default=12.0, help="300 Hz low-shelf gain, -12 to 18 dB, default 12 dB")
    watch = subs.add_parser("watch", help="stream status until Ctrl+C")
    watch.add_argument("--csv", action="store_true")
    watch.add_argument("--seconds", type=positive, help="stop reading after this duration")
    cal = subs.add_parser("calibrate", help="measure current zero and align the encoder")
    cal.add_argument("--voltage", type=milli, default=600, metavar="V")
    hfi_config = subs.add_parser("hfi-config", help="set runtime HFI excitation and current limits")
    hfi_config.add_argument("injection_mv", type=int, metavar="MV")
    hfi_config.add_argument("field_current_ma", type=int, metavar="MA")
    hfi_config.add_argument("peak_current_ma", type=int, metavar="MA")
    ramp = subs.add_parser("hfi-ramp", help="set sensorless acceleration during operation, default 100 rad/s^2")
    ramp.add_argument("acceleration", type=milli, metavar="RAD_S2")
    weakening = subs.add_parser("fw-config", help="set runtime field weakening, maximum current 0 disables it")
    weakening.add_argument("maximum_ma", type=int, metavar="MA")
    weakening.add_argument("utilization_permille", type=int, metavar="PERMILLE")
    weakening.add_argument("slew_ma_s", type=int, metavar="MA_S")
    bus = subs.add_parser("bus-voltage", help="set supply voltage while stopped; 0 selects sensing (12 V stub until PA3 is wired)")
    bus.add_argument("voltage", type=milli, metavar="V")
    limits = subs.add_parser("limits", help="set runtime current and voltage limits")
    limits.add_argument("current_ma", type=int, metavar="MA")
    limits.add_argument("voltage_mv", type=int, metavar="MV")
    observer = subs.add_parser("smo-config", help="set an SMO observer parameter while stopped")
    observer.add_argument("parameter", choices=SMO_PARAMETERS)
    observer.add_argument("value", type=int)

    for action, unit in (("voltage", "V"), ("current", "A"), ("speed", "rad/s"), ("position", "rad"), ("hfi", "rad/s")):
        subs.add_parser(action).add_argument("target", type=milli, metavar=unit)
    imp = subs.add_parser("impedance")
    imp.add_argument("target", type=milli, metavar="RAD")
    imp.add_argument("stiffness", type=milli, metavar="A_PER_RAD")
    imp.add_argument("damping", type=milli, metavar="A_PER_RAD_S")
    traj = subs.add_parser("trajectory")
    traj.add_argument("target", type=milli, metavar="RAD")
    traj.add_argument("speed", type=milli, metavar="RAD_S")
    traj.add_argument("acceleration", type=milli, metavar="RAD_S2")
    ratchet = subs.add_parser("ratchet", help="bidirectional tactile detents")
    ratchet.add_argument("teeth", type=int, nargs="?", default=24, metavar="TEETH_PER_REV")
    ratchet.add_argument("current", type=milli, nargs="?", default=100, metavar="A")
    ratchet.add_argument("--damping", type=milli, default=8, metavar="A_PER_RAD_S", help="additional damping")
    gain = subs.add_parser("gain")
    gain.add_argument("loop", choices=LOOPS)
    gain.add_argument("kp", type=milli)
    gain.add_argument("ki", type=milli)


def dispatch(link, args, machine=False):
    action = args.action
    settings = None
    if action == "play":
        if __package__:
            from .audio import play_file
        else:
            from audio import play_file
        result = play_file(link, args)
        if machine:
            print(json.dumps(result))
        else:
            print(f"played {result['file']} at {result['sample_rate']} Hz, {result['seconds']:.3f} s, "
                  f"speed={result['speed_rad_s']} rad/s, underruns={result['status']['audio_underruns']}")
            for warning in result["warnings"]:
                print(f"warning {warning}", file=sys.stderr)
        return result
    if action == "watch":
        writer = csv.writer(sys.stdout) if args.csv else None
        if writer:
            writer.writerow(field.name for field in fields(Status))
        for status in link.collect(args.seconds):
            if writer:
                writer.writerow(astuple(status))
                sys.stdout.flush()
            else:
                print(json.dumps(asdict(status)) if machine else status.text(), flush=True)
        return

    simple = {"status": "Q", "enable": "E", "stop": "X", "clear": "R"}
    target = {"voltage": "V", "current": "I", "speed": "S", "position": "P", "hfi": "Y"}
    if action in simple:
        status = link.command(simple[action])
    elif action in target:
        status = link.command(target[action], args.target)
    elif action == "calibrate":
        link.command("C", args.voltage)
        status = link.wait_for(lambda item: item.state in (0, 3), "calibration did not finish")
        if status.state != 3:
            raise RuntimeError(f"calibration incomplete ({status.text()})")
    elif action == "hfi-config":
        status = link.command("F", args.injection_mv, args.field_current_ma, args.peak_current_ma)
        settings = {
            "injection_mv": args.injection_mv,
            "field_current_ma": args.field_current_ma,
            "peak_current_ma": args.peak_current_ma,
        }
    elif action == "bus-voltage":
        status = link.command("B", args.voltage)
        settings = {"bus_mv": status.bus_mv, "voltage_limit_mv": status.voltage_limit_mv}
    elif action == "limits":
        status = link.command("L", args.current_ma, args.voltage_mv)
        settings = {"current_ma": args.current_ma, "voltage_mv": args.voltage_mv}
    elif action == "hfi-ramp":
        status = link.command("A", args.acceleration)
        settings = {"acceleration_rad_s2": args.acceleration / 1000}
    elif action == "fw-config":
        status = link.command("W", args.maximum_ma, args.utilization_permille, args.slew_ma_s)
        settings = {"maximum_ma": args.maximum_ma, "utilization_permille": args.utilization_permille,
                    "slew_ma_s": args.slew_ma_s}
    elif action == "smo-config":
        status = link.command("O", SMO_PARAMETERS.index(args.parameter), args.value)
        settings = {args.parameter: args.value}
    elif action == "impedance":
        status = link.command("Z", args.target, args.stiffness, args.damping)
    elif action == "trajectory":
        status = link.command("T", args.target, args.speed, args.acceleration)
    elif action == "ratchet":
        status = link.command("H", args.teeth, args.current, args.damping)
    else:
        status = link.command("G", LOOPS.index(args.loop), args.kp, args.ki)
    if machine:
        result = {"status": asdict(status)}
        if settings is not None:
            result["settings"] = settings
        print(json.dumps(result))
    else:
        print(status.text())
        if settings is not None:
            print("runtime settings " + json.dumps(settings))


def interactive(link):
    parser = argparse.ArgumentParser(prog="foc")
    commands(parser, shell=True)
    print("Type --help for commands or exit to close the session.")
    while True:
        try:
            line = input("foc> ").strip()
            if line in ("exit", "quit"):
                return
            if line:
                dispatch(link, parser.parse_args(shlex.split(line)))
        except EOFError:
            return
        except (KeyboardInterrupt, SystemExit):
            print()
        except (ValueError, RuntimeError, TimeoutError) as error:
            print(error, file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", help="probe unique ID")
    parser.add_argument("--frequency", type=int, help="SWD clock in Hz")
    parser.add_argument("--timeout", type=positive, default=2.0, help="response timeout in seconds")
    parser.add_argument("--json", action="store_true", help="JSON status or JSON Lines for watch")
    commands(parser)
    args = parser.parse_args()
    if args.json and (args.action == "shell" or getattr(args, "csv", False)):
        parser.error("--json requires a non-interactive command and cannot be used with --csv")

    from pyocd.core.exceptions import Error as PyOcdError
    try:
        probe = select_probe(args.probe) if args.json else args.probe
        frequency = args.frequency if args.frequency is not None else (8000000 if args.action == "play" else None)
        with open_link(probe, frequency, args.timeout) as link:
            if args.action == "shell":
                interactive(link)
            else:
                outcome = dispatch(link, args, args.json)
                if outcome is not None:
                    return int(not outcome.get("ok", True))
        return 0
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, TimeoutError, OSError, PyOcdError) as error:
        if args.json:
            print(json.dumps({"ok": False, "error": str(error)}), file=sys.stderr)
        else:
            print(f"foc: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
