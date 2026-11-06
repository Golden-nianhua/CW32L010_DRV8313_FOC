"""Stationary pulse capture and encoder-referenced saliency fitting."""

import math
import statistics
import struct
import time

PACKET = struct.Struct("<HBB4HI10i10I")
VECTORS = ("+alpha", "-alpha", "+beta", "-beta")
MA_PER_COUNT = 6602 / 4096


def capture_burst(link, amplitude, periods, repeats):
    if link.trace_up is None:
        raise RuntimeError("firmware pulse channel is unavailable")
    link.command("D", 0)
    link.trace_up.read()
    link.command("J", amplitude, periods, repeats)
    data = bytearray()
    deadline = time.monotonic() + link.timeout
    while len(data) < len(VECTORS) * PACKET.size and time.monotonic() < deadline:
        data.extend(link.trace_up.read())
        time.sleep(0.002)
    if len(data) != len(VECTORS) * PACKET.size:
        raise TimeoutError(f"pulse burst returned {len(data)} bytes")
    rows = []
    for index, values in enumerate(PACKET.iter_unpack(data)):
        magic, vector, width, fault, start, end, motion, count = values[:8]
        if (magic, vector, width) != (0x4A46, index, periods):
            raise RuntimeError("invalid pulse packet")
        rows.append({"vector": VECTORS[index], "fault": fault, "angle_start": start,
                     "angle_end": end, "motion_mrad": motion, "repeats": count,
                     "sum_counts": list(zip(values[8:18:2], values[9:18:2])),
                     "sum_square_counts": list(zip(values[18:28:2], values[19:28:2]))})
    return rows


def feature(rows, sample):
    def response(positive, negative):
        b, c = (-(p / positive["repeats"] - n / negative["repeats"]) * MA_PER_COUNT / 2
                for p, n in zip(positive["sum_counts"][sample], negative["sum_counts"][sample]))
        return -b - c, (b - c) / math.sqrt(3)

    aa, ba = response(*rows[:2])
    ab, bb = response(*rows[2:])
    return (aa - bb) / 2, (ab + ba) / 2, (aa + bb) / 2


def fit_angle(data):
    # Fit a constant stator-frame bias plus the two rotor-synchronous components.
    train, test = data[::2], data[1::2]
    c = [math.cos(phase) for phase, *_ in train]
    s = [math.sin(phase) for phase, *_ in train]
    cm, sm = statistics.mean(c), statistics.mean(s)
    c, s = [v - cm for v in c], [v - sm for v in s]
    cc = sum(v * v for v in c)
    ss = sum(v * v for v in s)
    cs = sum(a * b for a, b in zip(c, s))
    det = cc * ss - cs * cs
    if det <= len(train) ** 2 / 100:
        return None

    def coefficients(values):
        mean = statistics.mean(values)
        vc = sum((v - mean) * a for v, a in zip(values, c))
        vs = sum((v - mean) * a for v, a in zip(values, s))
        kcos = (vc * ss - vs * cs) / det
        ksin = (vs * cc - vc * cs) / det
        return mean - kcos * cm - ksin * sm, kcos, ksin

    x0, xc, xs = coefficients([x for _, x, _ in train])
    y0, yc, ys = coefficients([y for _, _, y in train])
    scale = math.sqrt((xc * xc + xs * xs + yc * yc + ys * ys) / 2)
    residual = math.sqrt(statistics.mean(
        (x - x0 - xc * math.cos(p) - xs * math.sin(p)) ** 2 +
        (y - y0 - yc * math.cos(p) - ys * math.sin(p)) ** 2 for p, x, y in test))
    result = {"saliency_ma": scale, "residual_ma": residual,
              "bias_ma": [x0, y0], "matrix_ma": [[xc, xs], [yc, ys]], "validation_points": len(test)}
    response_det = xc * ys - xs * yc
    if abs(response_det) > 1e-9:
        errors = []
        for phase, x, y in test:
            cos = (ys * (x - x0) - xs * (y - y0)) / response_det
            sin = (-yc * (x - x0) + xc * (y - y0)) / response_det
            errors.append(math.degrees(math.remainder(math.atan2(sin, cos) - phase, math.tau)) / 2)
        result["axis_error_rms_deg"] = math.sqrt(statistics.mean(v * v for v in errors))
        result["axis_error_max_deg"] = max(abs(v) for v in errors)
    return result


def analyze(points, periods):
    valid = [point for point in points if point["valid"]]
    metrics = {"valid_points": len(valid), "samples": []}
    for sample in range(periods + 1):
        data = []
        current = []
        for point in valid:
            phase = point["injection"][0]["angle_start"] * math.tau / 16384
            x, y, parallel = feature(point["injection"], sample)
            if point["baseline"]:
                bx, by, bp = feature(point["baseline"], sample)
                x, y = x - bx, y - by
                parallel -= bp
            data.append((2 * phase, x, y))
            current.append(parallel)
        metrics["samples"].append({"sample_after_pwm_write": sample + 1,
                                   "median_parallel_current_ma": statistics.median(current) if current else None,
                                   "fit": fit_angle(data) if len(data) >= 12 else None})
    return metrics
