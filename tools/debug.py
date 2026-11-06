# /// script
# requires-python = ">=3.12"
# dependencies = ["pyocd==0.46.0", "unicorn==2.1.4", "pyelftools==0.33"]
# ///

"""Build, replay and diagnose CW32L010 firmware with machine-readable results."""

import argparse
from contextlib import closing
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
from importlib.metadata import version
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import time

from host.foc import CONTROL_SOURCES, EVENTS, PYOCD_PROJECT, label, milli, open_link, positive, probe_inventory, select_probe
from host.hfi import analyze as analyze_hfi, capture_burst

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "tools" / "build" / "debug"
FIRMWARE = ROOT / "firmware"
IMAGE = "CW32L010_DRV8313_FOC"


def json_line(value):
    return json.dumps(value, ensure_ascii=False)


def run_logged(args, log, timeout):
    with log.open("wb") as stream:
        return subprocess.run(args, cwd=ROOT, stdin=subprocess.DEVNULL, stdout=stream,
                              stderr=subprocess.STDOUT, check=False, timeout=timeout).returncode


def artifact(profile):
    build = FIRMWARE / "build" / profile
    paths = {kind: build / f"{IMAGE}.{kind}" for kind in ("elf", "hex", "bin", "map")}
    entries = {}
    for kind, path in paths.items():
        data = path.read_bytes()
        if not data:
            raise RuntimeError(f"empty build artifact: {path}")
        entries[kind] = {"path": str(path), "bytes": len(data), "sha256": hashlib.sha256(data).hexdigest()}
    return entries


def verify(args, report, folder):
    from checks.simulate import run_checks

    report["profiles"] = []
    configurations = ("Debug", "Release") if args.configuration == "both" else (args.configuration,)
    for profile in configurations:
        result = {"configuration": profile, "ok": False, "stages": []}
        report["profiles"].append(result)
        try:
            if not args.no_build:
                configure = ["cmake", "-S", str(FIRMWARE), "--preset", f"CW32L010-{profile}"]
                if args.sdk_root:
                    configure.append(f"-DCW32_SPL_ROOT={args.sdk_root.resolve()}")
                if args.toolchain_root:
                    configure.append(f"-DARM_GNU_TOOLCHAIN_ROOT={args.toolchain_root.resolve()}")
                build = ["cmake", "--build", str(FIRMWARE / "build" / profile), "--parallel"]
                for name, command in (("configure", configure), ("build", build)):
                    log = folder / f"{profile}-{name}.log"
                    stage = {"name": name, "log": str(log)}
                    result["stages"].append(stage)
                    stage["exit_code"] = run_logged(command, log, timeout=180)
                    if stage["exit_code"]:
                        raise RuntimeError(f"{profile} {name} failed; see {log}")
            result["artifacts"] = artifact(profile)
            elf = Path(result["artifacts"]["elf"]["path"])
            result["checks"] = run_checks(elf)
            result["ok"] = all(item["ok"] for item in result["checks"])
        except (OSError, subprocess.TimeoutExpired, RuntimeError) as error:
            result["error"] = str(error)
    report["ok"] = all(item["ok"] for item in report["profiles"])


def flash(args, report, folder):
    report["artifacts"] = artifact(args.configuration)
    report["probe"] = select_probe(args.probe)
    report["configuration"] = args.configuration
    elf = report["artifacts"]["elf"]
    image = Path(elf["path"]).read_bytes()
    if hashlib.sha256(image).hexdigest() != elf["sha256"]:
        raise RuntimeError("ELF changed while preparing flash; rerun with a completed build")
    snapshot = folder / f"{IMAGE}.elf"
    snapshot.write_bytes(image)
    report["flash_image"] = str(snapshot)
    log = folder / "flash.log"
    report["log"] = str(log)
    report["exit_code"] = run_logged(
        [sys.executable, "-m", "pyocd", "load", "--project", str(PYOCD_PROJECT),
         "--uid", report["probe"], "--no-wait", "--erase=sector", str(snapshot)],
        log, timeout=120,
    )
    report["ok"] = report["exit_code"] == 0


class Recorder:
    def __init__(self, stream):
        self.stream = stream
        self.start = time.monotonic()

    def record(self, phase, status, kind="telemetry"):
        self.stream.write(json_line({
            "elapsed_s": round(time.monotonic() - self.start, 6),
            "phase": phase, "kind": kind, **asdict(status),
        }) + "\n")
        self.stream.flush()

    def command(self, link, phase, code, *values):
        status = link.command(code, *values)
        self.record(phase, status, "ack")
        return status

    def collect(self, link, phase, seconds, result, baseline):
        last = None
        iq_total = 0
        events = set()
        states = set()
        hfi_samples = [] if baseline.mode == 7 and "target" in result else None
        for status in link.collect(seconds):
            self.record(phase, status)
            last = status
            result["frames"] += 1
            iq_total += status.iq_ma
            if status.event:
                events.add(status.event)
            states.add(status.state)
            result.update(last=asdict(last), states=sorted(states),
                          events=[label(EVENTS, event) for event in sorted(events)])
            if hfi_samples is not None:
                now = time.monotonic()
                position = status.position_mrad / 1000
                hfi_samples.append((now, position))
                if status.state != 4:
                    raise RuntimeError(status.text())
                if len(hfi_samples) >= 4:
                    previous, origin = hfi_samples[-4]
                    speed_limit = max(5.5, abs(result["amplitude_milli"]) / 1000 * 1.5 + 1.0)
                    if now - previous >= 0.10 and abs(position - origin) > speed_limit * (now - previous):
                        raise RuntimeError(f"encoder exceeded {speed_limit:.1f} rad/s during {phase}")
        if last is None:
            raise TimeoutError(f"no telemetry received during {phase}")
        result.update(
            mean_iq_ma=round(iq_total / result["frames"], 2),
            missed_samples_delta=(last.missed_samples - baseline.missed_samples) & 0xFFFFFFFF,
            encoder_errors_delta=(last.encoder_errors - baseline.encoder_errors) & 0xFFFFFFFF,
        )
        if hfi_samples is not None:
            steady = hfi_samples[len(hfi_samples) // 2:]
            start, origin = steady[0]
            end, position = steady[-1]
            if end > start:
                result["encoder_mean_rad_s"] = (position - origin) / (end - start)
            speeds = []
            for index, (start, origin) in enumerate(steady):
                for other in range(index + 1, len(steady)):
                    end, position = steady[other]
                    if end - start >= 0.18:
                        speeds.append((position - origin) / (end - start))
                        break
            if speeds:
                result.update(encoder_window_speed_min_rad_s=min(speeds),
                              encoder_window_speed_max_rad_s=max(speeds),
                              encoder_window_speed_stddev_rad_s=statistics.pstdev(speeds),
                              encoder_window_stalls=sum(abs(speed) < 0.2 for speed in speeds)
                              if result["target"] else None)


def capture(args, report, link, recorder):
    if report["target"]["state"] == "HALTED":
        raise RuntimeError("target is halted; PC, SP and LR were recorded")
    link.read()
    initial = link.wait_for(lambda _: True, "no RTT status received; check that the firmware is running")
    recorder.record("capture", initial, "initial")
    report["initial"] = asdict(initial)
    result = report["capture"] = {"frames": 0}
    recorder.collect(link, "capture", args.seconds, result, initial)
    report["ok"] = True


def bench(args, report, link, recorder):
    code = {"voltage": "V", "current": "I", "hfi": "Y"}[args.mode]
    report["mode"] = args.mode
    report["amplitude_milli"] = args.amplitude
    report["phases"] = []
    stop_error = None
    try:
        recorder.command(link, "prepare", "X")
        if code != "Y":
            recorder.command(link, "calibrate", "C", args.align_voltage)
            ready = link.wait_for(lambda status: status.state in (0, 3), "calibration did not finish")
            recorder.record("calibrate", ready)
            if ready.state != 3:
                raise RuntimeError(f"calibration incomplete: {ready.text()}")
        recorder.command(link, "prepare", code, 0)
        recorder.command(link, "enable", "E")
        if code == "Y":
            ready = link.wait_for(lambda status: status.state in (0, 3, 4), "HFI did not lock")
            recorder.record("lock", ready)
            if ready.state != 4:
                raise RuntimeError(ready.text())
        for phase, target in (("zero", 0), ("positive", args.amplitude), ("zero_after_positive", 0),
                              ("negative", -args.amplitude), ("zero_after_negative", 0)):
            baseline = recorder.command(link, phase, code, target)
            result = {"phase": phase, "target": target, "amplitude_milli": args.amplitude, "frames": 0}
            report["phases"].append(result)
            recorder.collect(link, phase, args.seconds, result, baseline)
            if result["states"] != [4]:
                raise RuntimeError(f"drive left running state during {phase}")
    finally:
        try:
            stopped = recorder.command(link, "stop", "X")
            report["final_stop"] = {"ok": stopped.state in (0, 3), "status": asdict(stopped)}
        except Exception as error:
            stop_error = str(error)
            report["final_stop"] = {"ok": False, "error": stop_error}
    if not report["final_stop"]["ok"]:
        raise RuntimeError(f"final stop could not be confirmed: {stop_error or 'unexpected state'}")
    report["ok"] = True


def trace_metrics(path, divider):
    with path.open(encoding="utf-8") as stream:
        samples = [json.loads(line) for line in stream]
    speeds = [sample["speed_mrad_s"] / 1000 for sample in samples]
    iq_errors = [sample["iq_ma"] - sample["iq_ref_ma"] for sample in samples]
    half_window = max(1, round(4 / divider))
    raw_speeds = []
    angle_samples = []
    for i in range(half_window, len(samples) - half_window):
        start, sample, end = samples[i - half_window], samples[i], samples[i + half_window]
        gap = (end["sequence"] - start["sequence"]) & 0xFFFFFFFF
        elapsed = (end["time_us"] - start["time_us"]) & 0xFFFFFFFF
        window = samples[i - half_window:i + half_window + 1]
        if (gap != 2 * half_window * divider or elapsed == 0 or
                any(item["raw_angle"] >= 16384 for item in window)):
            continue
        # Unwrap adjacent records before averaging. The full smoothing window
        # can cover more than half a turn at ordinary sensorless speeds.
        delta = sum((left["raw_angle"] - right["raw_angle"] + 8192) % 16384 - 8192
                    for left, right in zip(window, window[1:]))
        speed = delta * math.tau * 1e6 / (16384 * elapsed)
        raw_speeds.append(speed)
        if sample["electrical_angle"] < 16384:
            angle_samples.append((speed, sample["electrical_angle"] * math.tau / 16384))
    result = {
        "speed_mean_rad_s": statistics.mean(speeds),
        "speed_stddev_rad_s": statistics.pstdev(speeds),
        "iq_rmse_ma": math.sqrt(statistics.mean(error * error for error in iq_errors)),
        "control_sources": {label(CONTROL_SOURCES, source): sum(
            sample["control_source"] == source for sample in samples)
            for source in sorted({sample["control_source"] for sample in samples})},
        "injection_min_mv": min(sample["injection_mv"] for sample in samples),
    }
    result["control_transitions"] = [
        {"time_us": sample["time_us"], "source": label(CONTROL_SOURCES, sample["control_source"]),
         "emf_mv": sample["smo_emf_mv"], "speed_mrad_s": sample["speed_mrad_s"],
         "injection_mv": sample["injection_mv"]}
        for index, sample in enumerate(samples)
        if index == 0 or sample["control_source"] != samples[index - 1]["control_source"]
    ]
    smo_start = None
    longest_smo_us = 0
    final_smo_us = 0
    for sample in samples:
        if sample["control_source"] == 4:
            if smo_start is None:
                smo_start = sample["time_us"]
            final_smo_us = (sample["time_us"] - smo_start) & 0xFFFFFFFF
            longest_smo_us = max(longest_smo_us, final_smo_us)
        else:
            smo_start = None
            final_smo_us = 0
    result["smo_longest_hold_ms"] = longest_smo_us / 1000
    result["smo_final_hold_ms"] = final_smo_us / 1000
    hfi_speeds = [sample["hfi_speed_mrad_s"] / 1000 for sample in samples
                  if sample["hfi_angle"] < 16384]
    smo_samples = [sample for sample in samples
                   if sample["smo_angle"] < 16384 and sample["electrical_angle"] < 16384]
    if smo_samples:
        errors = [((sample["smo_angle"] - sample["electrical_angle"] + 8192) % 16384 - 8192)
                  * 360 / 16384 for sample in smo_samples]
        ordered = sorted(abs(error) for error in errors)
        result.update(
            smo_angle_mean_error_deg=statistics.mean(errors),
            smo_angle_stddev_deg=statistics.pstdev(errors),
            smo_angle_p95_abs_error_deg=ordered[int(0.95 * (len(ordered) - 1))],
            smo_emf_mean_mv=statistics.mean(sample["smo_emf_mv"] for sample in smo_samples),
            smo_speed_mean_rad_s=statistics.mean(sample["smo_speed_mrad_s"] / 1000 for sample in smo_samples),
            smo_valid_fraction=statistics.mean(sample["smo_valid"] for sample in smo_samples),
            smo_ready_fraction=statistics.mean(sample["smo_ready"] for sample in smo_samples),
        )
    hfi_angles = [sample["hfi_angle"] for sample in samples if sample["hfi_angle"] < 16384]
    if hfi_speeds:
        result.update(hfi_speed_mean_rad_s=statistics.mean(hfi_speeds),
                      hfi_speed_stddev_rad_s=statistics.pstdev(hfi_speeds),
                      hfi_missed_frames_total=samples[-1]["hfi_missed_frames"],
                      hfi_missed_frames_delta=(samples[-1]["hfi_missed_frames"] -
                                               samples[0]["hfi_missed_frames"]) & 0xFFFFFFFF)
        elapsed = (samples[-1]["time_us"] - samples[0]["time_us"]) & 0xFFFFFFFF
        if elapsed:
            result["hfi_processed_frames_hz"] = (
                ((samples[-1]["hfi_sequence"] - samples[0]["hfi_sequence"]) & 0xFFFFFFFF)
                - result["hfi_missed_frames_delta"]) * 1e6 / elapsed
    if len(hfi_angles) > 1:
        hfi_steps = [((end - start + 8192) % 16384) - 8192
                     for start, end in zip(hfi_angles, hfi_angles[1:])]
        result["hfi_angle_reverse_samples"] = sum(step < -100 for step in hfi_steps)
        result["hfi_angle_large_steps"] = sum(abs(step) > 1000 for step in hfi_steps)
        result["hfi_angle_max_step_counts"] = max(abs(step) for step in hfi_steps)
    if raw_speeds:
        mean = statistics.mean(raw_speeds)
        result.update(raw_speed_mean_rad_s=mean, raw_speed_stddev_rad_s=statistics.pstdev(raw_speeds))
        if angle_samples:
            sine = statistics.mean((speed - mean) * math.sin(2 * phase) for speed, phase in angle_samples)
            cosine = statistics.mean((speed - mean) * math.cos(2 * phase) for speed, phase in angle_samples)
            result["raw_speed_electrical_second_rad_s"] = 2 * math.hypot(sine, cosine)
    return result


def trace(args, report, link, recorder, folder):
    control_rate = 500 if args.mode == "hfi" and args.speed is not None else 1000
    count = round(args.seconds * control_rate / args.divider)
    path = folder / "trace.jsonl"
    result = report["trace"] = {"path": str(path), "divider": args.divider,
                                 "expected": count, "frames": 0, "invalid_angles": 0}
    report["speed_mrad_s"] = args.speed
    try:
        if args.speed is not None:
            recorder.command(link, "prepare", "X")
            recorder.command(link, "observer", "O", 0, int(args.smo == "on"))
            if args.mode == "speed":
                recorder.command(link, "calibrate", "C", 1000)
                ready = link.wait_for(lambda status: status.state in (0, 3), "calibration did not finish")
                recorder.record("calibrate", ready)
                if ready.state != 3:
                    raise RuntimeError(f"calibration incomplete: {ready.text()}")
            recorder.command(link, "prepare", "Y" if args.mode == "hfi" else "S", args.speed)
            enabled = recorder.command(link, "enable", "E")
            if args.mode == "hfi":
                enabled = link.wait_for(lambda status: status.state == 4, "HFI did not lock")
            recorder.collect(link, "settle", args.settle, {"frames": 0}, enabled)
        before = recorder.command(link, "before", "Q")
        change_at = time.monotonic() + args.change_after
        speed_changed = False
        with path.open("w", encoding="utf-8") as stream, closing(
                link.trace(count, args.divider, control_rate)) as samples:
            for sample in samples:
                stream.write(json_line(sample) + "\n")
                result["frames"] += 1
                result["invalid_angles"] += int(sample["raw_angle"] == 0xFFFF)
                if args.next_speed is not None and not speed_changed and time.monotonic() >= change_at:
                    recorder.command(link, "speed_change", "Y" if args.mode == "hfi" else "S", args.next_speed)
                    speed_changed = True
                speed_out_of_range = args.mode != "hfi" and (
                    abs(sample["speed_mrad_s"]) > max(24000, 2 * abs(args.speed or 0)))
                if args.speed is not None and (
                    link.latest.state != 4 or link.latest.event == 4 or speed_out_of_range):
                    raise RuntimeError(f"speed trace interrupted: {link.latest.text()}")
        after = recorder.command(link, "after", "Q")
        result["missed_samples_delta"] = (after.missed_samples - before.missed_samples) & 0xFFFFFFFF
        result["encoder_errors_delta"] = (after.encoder_errors - before.encoder_errors) & 0xFFFFFFFF
    finally:
        result["rtt_dropped"] = count - result["frames"]
        if args.speed is not None:
            try:
                stopped = recorder.command(link, "stop", "X")
                report["final_stop"] = {"ok": stopped.state in (0, 3), "status": asdict(stopped)}
            except Exception as error:
                report["final_stop"] = {"ok": False, "error": str(error)}
    if args.speed is not None and not report["final_stop"]["ok"]:
        raise RuntimeError("final stop could not be confirmed")
    result["metrics"] = trace_metrics(path, args.divider)
    if result["rtt_dropped"]:
        raise RuntimeError(f"trace lost {result['rtt_dropped']} samples; increase SWD speed or divider")
    if args.mode == "hfi" and args.speed is not None:
        if result["missed_samples_delta"] or result["metrics"].get("hfi_missed_frames_total", 0):
            raise RuntimeError("HFI test missed foreground samples or observer frames; see trace metrics")
        if args.require_smo and result["metrics"]["smo_final_hold_ms"] < 1000:
            raise RuntimeError("SMO did not remain in control for the final 1 s")
    report["ok"] = True


def hfi_still(link):
    position = None
    since = 0.0

    def stable(status):
        nonlocal position, since
        now = time.monotonic()
        if status.state != 3 or abs(status.speed_mrad_s) > 200:
            position = None
            return False
        if position is None or abs(status.position_mrad - position) > 2:
            position, since = status.position_mrad, now
        return now - since >= 0.10

    return link.wait_for(stable, "rotor did not settle before injection")


def hfi(args, report, link, recorder, folder):
    path = folder / "hfi.jsonl"
    report.update(data=str(path), amplitude_mv=args.amplitude, periods=args.periods,
                  repeats=args.repeats, requested_points=args.points, step_mrad=args.step, reverse=args.reverse)
    points = []
    direction = -1 if args.reverse else 1
    try:
        state = recorder.command(link, "prepare", "X")
        if state.state != 3:
            recorder.command(link, "calibrate", "C", 800)
            state = link.wait_for(lambda s: s.state in (0, 3), "calibration did not finish")
            recorder.record("calibrate", state)
            if state.state != 3:
                raise RuntimeError(state.text())
        with path.open("w", encoding="utf-8") as stream:
            for index in range(args.points):
                if index:
                    origin = state.position_mrad
                    recorder.command(link, "index", "S", direction * 5000)
                    recorder.command(link, "index", "E")
                    try:
                        link.wait_for(lambda s: direction * (s.position_mrad - origin) >= args.step,
                                      "rotor did not move to the next probe point")
                    finally:
                        recorder.command(link, "index", "X")
                state = hfi_still(link)
                baseline = capture_burst(link, 0, args.periods, args.repeats) if args.amplitude else []
                if baseline:
                    hfi_still(link)
                injection = capture_burst(link, args.amplitude, args.periods, args.repeats)
                after = recorder.command(link, "pulse", "Q")
                expected = args.repeats * 40 if args.periods == 0 else args.repeats
                fault = any(row["fault"] or row["repeats"] != expected for row in baseline + injection)
                motion = max(row["motion_mrad"] for row in baseline + injection)
                shift = ((injection[0]["angle_start"] - baseline[0]["angle_start"] + 8192) % 16384 - 8192
                         if baseline else 0)
                point = {"index": index, "position_mrad": state.position_mrad, "baseline": baseline,
                         "injection": injection, "motion_mrad": motion, "baseline_shift_counts": shift,
                         "valid": not fault and motion <= 4 and abs(shift) <= 73 and
                         after.encoder_errors == state.encoder_errors and after.missed_samples == state.missed_samples}
                points.append(point)
                stream.write(json_line(point) + "\n")
                stream.flush()
                if fault or after.state != 3:
                    raise RuntimeError(f"injection burst incomplete at point {index}: {after.text()}")
                state = after
    finally:
        try:
            stopped = recorder.command(link, "stop", "X")
            report["final_stop"] = {"ok": stopped.state in (0, 3), "status": asdict(stopped)}
        except Exception as error:
            report["final_stop"] = {"ok": False, "error": str(error)}
        report["points"] = len(points)
        if points:
            positions = [point["position_mrad"] for point in points]
            report["mechanical_span_mrad"] = max(positions) - min(positions)
            report["analysis"] = analyze_hfi(points, args.periods)
    if not report["final_stop"]["ok"]:
        raise RuntimeError("final stop could not be confirmed")
    report["ok"] = report["analysis"]["valid_points"] > 0


def with_target(args, report, folder):
    report["probe"] = select_probe(args.probe)
    report["target"] = {}
    telemetry = folder / "telemetry.jsonl"
    report["telemetry"] = str(telemetry)
    with telemetry.open("w", encoding="utf-8") as stream:
        with open_link(report["probe"], args.frequency, args.timeout, report["target"]) as link:
            recorder = Recorder(stream)
            if args.action == "trace":
                trace(args, report, link, recorder, folder)
            elif args.action == "hfi":
                hfi(args, report, link, recorder, folder)
            else:
                handler = capture if args.action == "capture" else bench
                handler(args, report, link, recorder)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="parent directory for run reports")
    actions = parser.add_subparsers(dest="action", required=True)

    check = actions.add_parser("verify", help="build and run offline regression checks")
    check.add_argument("--configuration", choices=("both", "Debug", "Release"), default="both")
    check.add_argument("--no-build", action="store_true", help="check existing artifacts")
    check.add_argument("--sdk-root", type=Path)
    check.add_argument("--toolchain-root", type=Path)

    actions.add_parser("probe", help="list SWD probes")
    program = actions.add_parser("flash", help="flash an existing build and record its hashes")
    program.add_argument("--configuration", choices=("Debug", "Release"), default="Release")
    program.add_argument("--probe", help="probe ID")

    log = actions.add_parser("capture", help="attach and capture telemetry without commanding motion")
    exercise = actions.add_parser("bench", help="run a finite signed step test and issue stop at the end")
    scope = actions.add_parser("trace", help="capture raw encoder and current samples over RTT")
    pulses = actions.add_parser("hfi", help="measure stationary pulse responses, optionally indexing the rotor")
    for action, seconds in ((log, 3.0), (exercise, 0.4), (scope, 5.0)):
        action.add_argument("--probe", help="probe ID")
        action.add_argument("--frequency", type=int, help="SWD clock in Hz")
        action.add_argument("--timeout", type=positive, default=2.0)
        action.add_argument("--seconds", type=positive, default=seconds,
                            help="capture duration or duration per bench phase")
    exercise.add_argument("--mode", choices=("voltage", "current", "hfi"), required=True)
    exercise.add_argument("--amplitude", type=milli, required=True, help="positive amplitude in V, A or rad/s")
    exercise.add_argument("--align-voltage", type=milli, default=600, help="alignment voltage in V")
    scope.set_defaults(frequency=2000000)
    scope.add_argument("--divider", type=int, choices=range(1, 11), default=2, help="control samples per trace sample")
    scope.add_argument("--speed", type=milli, metavar="RAD_S", help="run at this speed during capture, then stop")
    scope.add_argument("--mode", choices=("speed", "hfi"), default="speed",
                       help="control mode used with --speed")
    scope.add_argument("--smo", choices=("on", "off"), default="on", help="SMO shadow observer")
    scope.add_argument("--next-speed", type=milli, help="change the speed once during the capture")
    scope.add_argument("--change-after", type=positive, default=5.0, help="seconds before --next-speed")
    scope.add_argument("--require-smo", action="store_true",
                       help="require continuous SMO control for the final 1 s")
    scope.add_argument("--settle", type=positive, default=2.0, help="seconds before sampling a commanded speed")
    pulses.add_argument("--probe", help="probe ID")
    pulses.add_argument("--frequency", type=int, default=2000000, help="SWD clock in Hz")
    pulses.add_argument("--timeout", type=positive, default=2.0)
    pulses.add_argument("--amplitude", type=milli, default=200, metavar="V")
    pulses.add_argument("--periods", type=int, choices=range(5), default=2, help="0 selects the rotating carrier")
    pulses.add_argument("--repeats", type=int, default=64)
    pulses.add_argument("--points", type=int, default=1, help="move between this many stationary probes")
    pulses.add_argument("--step", type=milli, default=80, metavar="RAD", help="minimum travel between points")
    pulses.add_argument("--reverse", action="store_true", help="index in the negative direction")
    args = parser.parse_args()
    if args.action in ("capture", "bench", "trace") and args.seconds < 0.1:
        parser.error("--seconds must be at least 0.1 for RTT telemetry")
    if args.action in ("capture", "bench", "trace", "hfi") and args.frequency is not None and args.frequency <= 0:
        parser.error("--frequency must be positive")
    if args.action == "bench":
        if not 0 < args.amplitude <= (2000 if args.mode == "current" else 1000000 if args.mode == "hfi" else 6928):
            parser.error("amplitude must be positive and within 6.928 V, 2 A or 1000 rad/s")
        if not 0 < args.align_voltage <= 3000:
            parser.error("alignment voltage must be within (0, 3] V")
    if args.action == "trace":
        if args.seconds > 20 * args.divider:
            parser.error("trace accepts up to 20000 samples")
        speed_limit = 1000000 if args.mode == "hfi" else 30000
        if args.speed is not None and not 0 < abs(args.speed) <= speed_limit:
            parser.error(f"speed must be nonzero and within ±{speed_limit // 1000} rad/s")
        if args.next_speed is not None and (args.speed is None or abs(args.next_speed) > speed_limit
                                             or args.change_after >= args.seconds):
            parser.error(f"next-speed requires --speed, ±{speed_limit // 1000} rad/s and a change before capture ends")
        if args.require_smo and (args.mode != "hfi" or args.speed is None or args.smo != "on"):
            parser.error("require-smo needs --mode hfi --speed and --smo on")
    if args.action == "hfi":
        if not 0 <= args.amplitude <= (2000 if args.periods == 0 else 500) or not 1 <= args.repeats <= 64:
            parser.error("HFI requires 1–64 repeats and up to 0.5 V for pulses or 0.8 V for the carrier")
        if not 1 <= args.points <= 100 or not 40 <= args.step <= 300:
            parser.error("HFI requires 1–100 points and a step of 0.04–0.30 rad")
    return args


def main():
    args = arguments()
    started = datetime.now(timezone.utc)
    args.output.mkdir(parents=True, exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix=f"{started:%Y%m%dT%H%M%S}-{args.action}-", dir=args.output.resolve()))
    report_path = folder / "report.json"
    report = {
        "schema": 1, "action": args.action, "ok": False, "started_utc": started.isoformat(),
        "report": str(report_path), "versions": {name: version(name) for name in ("pyocd", "unicorn", "pyelftools")},
    }
    start = time.monotonic()
    interrupted = False
    try:
        if args.action == "probe":
            report.update(ok=True, probes=probe_inventory())
        elif args.action == "verify":
            verify(args, report, folder)
        elif args.action == "flash":
            flash(args, report, folder)
        else:
            with_target(args, report, folder)
    except KeyboardInterrupt:
        report["error"] = "interrupted"
        interrupted = True
    except Exception as error:
        report["error"] = f"{type(error).__name__}: {error}"
    report["seconds"] = round(time.monotonic() - start, 3)
    output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    report_path.write_text(output, encoding="utf-8")
    print(output, end="")
    return 130 if interrupted else int(not report["ok"])


if __name__ == "__main__":
    raise SystemExit(main())
