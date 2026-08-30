#!/usr/bin/env -S uv run
# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "matplotlib>=3.9",
#   "numpy>=2.0",
#   "pyserial>=3.5",
# ]
# ///
"""THROWAWAY probe: desk-plane accelerometer dead reckoning with ZUPTs."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

# Deliberately exposed prototype tuning knobs.
REST_WINDOW_S = 0.12
MIN_REST_S = 0.15
GYRO_MAG_VAR_MAX_DPS2 = 0.80
ACCEL_NORM_MEAN_TOL_G = 0.010
# This can be generous because the independent vibration gate rejects sliding.
ACCEL_VECTOR_VAR_MAX_G2 = 1.0e-3
VIBRATION_LOWPASS_WINDOW_S = 0.080
VIBRATION_ENERGY_WINDOW_S = 0.080
VIBRATION_ENERGY_MAX_G = 0.0040
VIBRATION_MOTION_MIN_S = 0.15
GRAVITY_LEAK_TAU_S = 0.35
GYRO_BIAS_LEAK_TAU_S = 0.35
ENFORCE_TERMINAL_ZUPT = True

CURSOR_REPORT_INTERVAL_S = 0.100
CURSOR_GAIN_MIN_PX_PER_MM = 0.8
CURSOR_GAIN_MAX_PX_PER_MM = 4.0
CURSOR_GAIN_REFERENCE_MM_S = 150.0
CURSOR_GAIN_EXPONENT = 1.5

GRAVITY_M_S2 = 9.80665
DEFAULT_PORT = "/dev/cu.usbmodem101"


@dataclass
class Stroke:
    start_s: float
    duration_s: float
    peak_accel_m_s2: float
    displacement_m: np.ndarray
    path_m: np.ndarray
    velocity_m_s: np.ndarray
    sample_times_s: np.ndarray


def load_imu(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load protocol IMU rows, ignoring CFG, comments, and boot noise."""
    rows: list[list[float]] = []
    with path.open(errors="replace") as source:
        for line in source:
            fields = line.strip().split(",")
            if len(fields) != 8 or fields[0] != "IMU":
                continue
            try:
                rows.append([float(value) for value in fields[1:]])
            except ValueError:
                continue
    if len(rows) < 3:
        raise SystemExit(f"{path}: need at least three valid IMU rows")

    values = np.asarray(rows, dtype=float)
    t_s = (values[:, 0] - values[0, 0]) * 1e-6
    monotonic = np.r_[True, np.diff(t_s) > 0]
    if not np.all(monotonic):
        print(f"warning: discarded {np.count_nonzero(~monotonic)} non-monotonic rows")
        t_s, values = t_s[monotonic], values[monotonic]
    return t_s, values[:, 1:4], values[:, 4:7]


def centered_windows(values: np.ndarray, width: int) -> np.ndarray:
    left = width // 2
    right = width - 1 - left
    padded = np.pad(values, ((left, right), (0, 0)), mode="edge")
    return np.lib.stride_tricks.sliding_window_view(padded, width, axis=0)


def contiguous_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    starts = np.flatnonzero(edges == 1)
    ends = np.flatnonzero(edges == -1)
    return list(zip(starts.tolist(), ends.tolist()))


def vibration_energy(t_s: np.ndarray, accel_g: np.ndarray) -> np.ndarray:
    """Rolling RMS of accel-magnitude content above the local low-pass."""
    dt = float(np.median(np.diff(t_s)))
    lowpass_width = max(3, int(round(VIBRATION_LOWPASS_WINDOW_S / dt)))
    energy_width = max(3, int(round(VIBRATION_ENERGY_WINDOW_S / dt)))
    accel_mag = np.linalg.norm(accel_g, axis=1)
    magnitude_windows = centered_windows(accel_mag[:, None], lowpass_width)[:, 0, :]
    highpass = accel_mag - np.mean(magnitude_windows, axis=1)
    energy_windows = centered_windows((highpass * highpass)[:, None], energy_width)[:, 0, :]
    return np.sqrt(np.mean(energy_windows, axis=1))


def detect_rest(
    t_s: np.ndarray, gyro_dps: np.ndarray, accel_g: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    dt = float(np.median(np.diff(t_s)))
    width = max(3, int(round(REST_WINDOW_S / dt)))
    gyro_mag = np.linalg.norm(gyro_dps, axis=1)[:, None]
    accel_mag = np.linalg.norm(accel_g, axis=1)

    gyro_windows = centered_windows(gyro_mag, width)[:, 0, :]
    accel_windows = centered_windows(accel_g, width)
    gyro_var = np.var(gyro_windows, axis=1)
    accel_vector_var = np.var(accel_windows, axis=2).sum(axis=1)
    accel_norm_mean = np.mean(np.linalg.norm(accel_windows, axis=1), axis=1)
    vibration_g = vibration_energy(t_s, accel_g)
    vibrating = np.zeros(len(t_s), dtype=bool)
    minimum_vibration = max(2, int(round(VIBRATION_MOTION_MIN_S / dt)))
    for start, end in contiguous_runs(vibration_g >= VIBRATION_ENERGY_MAX_G):
        if end - start >= minimum_vibration:
            vibrating[start:end] = True

    # The real capture reads ~1.023 g at rest. Treat its robust median magnitude
    # as the sensor's measured 1 g rather than rejecting every stationary row.
    measured_one_g = float(np.median(accel_mag))
    candidate = (
        (gyro_var < GYRO_MAG_VAR_MAX_DPS2)
        & (np.abs(accel_norm_mean / measured_one_g - 1.0) < ACCEL_NORM_MEAN_TOL_G)
        & (accel_vector_var < ACCEL_VECTOR_VAR_MAX_G2)
        & ~vibrating
    )

    minimum = max(2, int(round(MIN_REST_S / dt)))
    rest = np.zeros(len(t_s), dtype=bool)
    for start, end in contiguous_runs(candidate):
        if end - start >= minimum:
            rest[start:end] = True
    return rest, vibration_g


def desk_basis(gravity_g: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    normal = -gravity_g / np.linalg.norm(gravity_g)
    x_axis = np.array([1.0, 0.0, 0.0])
    x_axis -= normal * np.dot(x_axis, normal)
    if np.linalg.norm(x_axis) < 0.1:
        x_axis = np.array([0.0, 1.0, 0.0])
        x_axis -= normal * np.dot(x_axis, normal)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(normal, x_axis)
    return x_axis, y_axis, normal


def integrate_interval(
    t_s: np.ndarray,
    gyro_dps: np.ndarray,
    accel_g: np.ndarray,
    gravity_g: np.ndarray,
    gyro_bias_dps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Integrate one rest-to-rest interval; velocity starts at zero."""
    # Freeze the rest-estimated desk normal for the whole stroke. Projecting
    # the residual before integration makes vertical velocity identically zero.
    x_axis, y_axis, normal = desk_basis(gravity_g)
    residual_m_s2 = (accel_g - gravity_g) * GRAVITY_M_S2
    residual_m_s2 -= np.outer(residual_m_s2 @ normal, normal)
    body_plane = np.column_stack((residual_m_s2 @ x_axis, residual_m_s2 @ y_axis))

    dt = np.diff(t_s, prepend=t_s[0])
    yaw_rate = np.deg2rad((gyro_dps - gyro_bias_dps) @ normal)
    yaw = np.cumsum(yaw_rate * dt)
    cosine, sine = np.cos(yaw), np.sin(yaw)
    desk_accel = np.column_stack(
        (
            cosine * body_plane[:, 0] - sine * body_plane[:, 1],
            sine * body_plane[:, 0] + cosine * body_plane[:, 1],
        )
    )

    velocity = np.zeros_like(desk_accel)
    for index in range(1, len(t_s)):
        step = t_s[index] - t_s[index - 1]
        velocity[index] = velocity[index - 1] + 0.5 * (
            desk_accel[index - 1] + desk_accel[index]
        ) * step

    # This is an offline preview, so use the following rest sample as a hard
    # terminal ZUPT and spread the accumulated velocity error over the stroke.
    if ENFORCE_TERMINAL_ZUPT and t_s[-1] > t_s[0]:
        progress = (t_s - t_s[0]) / (t_s[-1] - t_s[0])
        velocity -= progress[:, None] * velocity[-1]

    path = np.zeros_like(desk_accel)
    for index in range(1, len(t_s)):
        step = t_s[index] - t_s[index - 1]
        path[index] = path[index - 1] + 0.5 * (
            velocity[index - 1] + velocity[index]
        ) * step
    return path, velocity, desk_accel


def reconstruct(
    t_s: np.ndarray, gyro_dps: np.ndarray, accel_g: np.ndarray
) -> tuple[list[Stroke], np.ndarray, np.ndarray]:
    rest, vibration_g = detect_rest(t_s, gyro_dps, accel_g)
    rest_runs = contiguous_runs(rest)
    strokes: list[Stroke] = []
    gravity_g: np.ndarray | None = None
    gyro_bias_dps: np.ndarray | None = None
    sample_dt = float(np.median(np.diff(t_s)))

    for left, right in zip(rest_runs, rest_runs[1:]):
        rest_duration_s = max(sample_dt, t_s[left[1] - 1] - t_s[left[0]] + sample_dt)
        measured_gravity_g = np.mean(accel_g[left[0] : left[1]], axis=0)
        measured_gyro_bias_dps = np.mean(gyro_dps[left[0] : left[1]], axis=0)
        if gravity_g is None:
            gravity_g = measured_gravity_g
            gyro_bias_dps = measured_gyro_bias_dps
        else:
            gravity_alpha = 1.0 - np.exp(-rest_duration_s / GRAVITY_LEAK_TAU_S)
            gyro_alpha = 1.0 - np.exp(-rest_duration_s / GYRO_BIAS_LEAK_TAU_S)
            gravity_g += gravity_alpha * (measured_gravity_g - gravity_g)
            gyro_bias_dps += gyro_alpha * (measured_gyro_bias_dps - gyro_bias_dps)

        stroke_start, stroke_end = left[1] - 1, right[0]
        if stroke_end <= stroke_start + 1:
            continue

        segment = slice(stroke_start, stroke_end + 1)
        path, velocity, desk_accel = integrate_interval(
            t_s[segment], gyro_dps[segment], accel_g[segment], gravity_g, gyro_bias_dps
        )
        strokes.append(
            Stroke(
                start_s=float(t_s[stroke_start]),
                duration_s=float(t_s[stroke_end] - t_s[stroke_start]),
                peak_accel_m_s2=float(np.max(np.linalg.norm(desk_accel, axis=1))),
                displacement_m=path[-1],
                path_m=path,
                velocity_m_s=velocity,
                sample_times_s=t_s[segment].copy(),
            )
        )
    return strokes, rest, vibration_g


def save_plot(path: Path, strokes: list[Stroke]) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output = path.with_name(f"{path.stem}_table_zupt.png")
    figure, axis = plt.subplots(figsize=(7, 6))
    if strokes:
        origin = np.zeros(2)
        for number, stroke in enumerate(strokes, 1):
            points_mm = (stroke.path_m + origin) * 1000.0
            axis.plot(points_mm[:, 0], points_mm[:, 1], label=f"stroke {number}")
            axis.scatter(points_mm[-1, 0], points_mm[-1, 1], s=20)
            origin += stroke.displacement_m
        axis.legend()
    else:
        axis.scatter([0], [0], label="no strokes detected")
        axis.legend()
    axis.set_title("Table ZUPT reconstructed path")
    axis.set_xlabel("desk x (mm)")
    axis.set_ylabel("desk y (mm)")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(output, dpi=150)
    plt.close(figure)
    return output


def cursor_path(stroke: Stroke) -> np.ndarray:
    speed_mm_s = np.linalg.norm(stroke.velocity_m_s, axis=1) * 1000.0
    speed_ratio = (speed_mm_s / CURSOR_GAIN_REFERENCE_MM_S) ** CURSOR_GAIN_EXPONENT
    gain_px_per_mm = CURSOR_GAIN_MIN_PX_PER_MM + (
        CURSOR_GAIN_MAX_PX_PER_MM - CURSOR_GAIN_MIN_PX_PER_MM
    ) * speed_ratio / (1.0 + speed_ratio)
    cursor_velocity_px_s = stroke.velocity_m_s * 1000.0 * gain_px_per_mm[:, None]
    path_px = np.zeros_like(cursor_velocity_px_s)
    for index in range(1, len(path_px)):
        step = stroke.sample_times_s[index] - stroke.sample_times_s[index - 1]
        path_px[index] = path_px[index - 1] + 0.5 * (
            cursor_velocity_px_s[index - 1] + cursor_velocity_px_s[index]
        ) * step
    return path_px


def save_cursor_plot(path: Path, strokes: list[Stroke]) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output = path.with_name(f"{path.stem}_cursor_sim.png")
    figure, axis = plt.subplots(figsize=(7, 6))
    origin = np.zeros(2)
    for number, stroke in enumerate(strokes, 1):
        points_px = cursor_path(stroke) + origin
        axis.plot(points_px[:, 0], points_px[:, 1], label=f"stroke {number}")
        origin = points_px[-1]
    if strokes:
        axis.legend()
    axis.set_title("Nonlinear-gain simulated cursor path")
    axis.set_xlabel("cursor x (px)")
    axis.set_ylabel("cursor y (px)")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    figure.tight_layout()
    figure.savefig(output, dpi=150)
    plt.close(figure)
    return output


def simulate_cursor(path: Path) -> None:
    t_s, gyro_dps, accel_g = load_imu(path)
    strokes, _rest, _vibration_g = reconstruct(t_s, gyro_dps, accel_g)
    print(
        f"cursor gain: {CURSOR_GAIN_MIN_PX_PER_MM:.1f}-"
        f"{CURSOR_GAIN_MAX_PX_PER_MM:.1f} px/mm, "
        f"reference={CURSOR_GAIN_REFERENCE_MM_S:.0f} mm/s, "
        f"exponent={CURSOR_GAIN_EXPONENT:.1f}"
    )
    total = np.zeros(2)
    for number, stroke in enumerate(strokes, 1):
        points_px = cursor_path(stroke)
        edges = np.arange(
            stroke.sample_times_s[0],
            stroke.sample_times_s[-1] + CURSOR_REPORT_INTERVAL_S,
            CURSOR_REPORT_INTERVAL_S,
        )
        edges[-1] = min(edges[-1], stroke.sample_times_s[-1])
        edges = np.unique(edges)
        sampled = np.column_stack(
            (
                np.interp(edges, stroke.sample_times_s, points_px[:, 0]),
                np.interp(edges, stroke.sample_times_s, points_px[:, 1]),
            )
        )
        for start, end, delta in zip(edges, edges[1:], np.diff(sampled, axis=0)):
            print(
                f"cursor stroke {number} {start:.3f}-{end:.3f}s: "
                f"delta=({delta[0]:+.2f}, {delta[1]:+.2f}) px"
            )
        total += points_px[-1]
    print(f"final simulated cursor=({total[0]:+.2f}, {total[1]:+.2f}) px")
    output = save_cursor_plot(path, strokes)
    print(f"cursor plot: {output}")


def report_analysis(path: Path) -> tuple[list[Stroke], np.ndarray]:
    t_s, gyro_dps, accel_g = load_imu(path)
    strokes, rest, vibration_g = reconstruct(t_s, gyro_dps, accel_g)
    sample_rate = (len(t_s) - 1) / (t_s[-1] - t_s[0])
    print(
        f"loaded {len(t_s)} samples over {t_s[-1]:.3f} s "
        f"({sample_rate:.1f} Hz); rest={100.0 * np.mean(rest):.1f}%"
    )
    for number, stroke in enumerate(strokes, 1):
        x_mm, y_mm = stroke.displacement_m * 1000.0
        print(
            f"stroke {number}: duration={stroke.duration_s:.3f} s, "
            f"peak_accel={stroke.peak_accel_m_s2:.3f} m/s^2, "
            f"displacement=({x_mm:+.2f}, {y_mm:+.2f}) mm, "
            f"magnitude={np.linalg.norm(stroke.displacement_m) * 1000.0:.2f} mm"
        )
    total_phantom_mm = sum(np.linalg.norm(item.displacement_m) for item in strokes) * 1000.0
    print(f"total endpoint displacement across detected strokes: {total_phantom_mm:.3f} mm")
    vibration_stats: dict[str, np.ndarray] = {}
    for label, mask in (("rest", rest), ("stroke", ~rest)):
        energy_mg = vibration_g[mask] * 1000.0
        percentiles = np.percentile(energy_mg, [10, 25, 50, 75, 90, 95])
        vibration_stats[label] = percentiles
        print(
            f"vibration energy {label} (mg RMS): "
            f"p10={percentiles[0]:.2f}, p25={percentiles[1]:.2f}, "
            f"p50={percentiles[2]:.2f}, p75={percentiles[3]:.2f}, "
            f"p90={percentiles[4]:.2f}, p95={percentiles[5]:.2f}"
        )
    print(
        "vibration separation (mg RMS): "
        f"median gap={vibration_stats['stroke'][2] - vibration_stats['rest'][2]:+.2f}, "
        f"stroke p25 - rest p95={vibration_stats['stroke'][1] - vibration_stats['rest'][5]:+.2f}"
    )
    short_rests = [
        (t_s[start], t_s[end - 1] - t_s[start])
        for start, end in contiguous_runs(rest)[1:-1]
        if t_s[end - 1] - t_s[start] < 0.5
    ]
    print(
        f"micro-rests (qualified interior rest runs <0.5 s): {len(short_rests)}"
        + (
            "; start/duration="
            + ", ".join(f"{start:.3f}s/{duration:.3f}s" for start, duration in short_rests)
            if short_rests
            else ""
        )
    )
    output = save_plot(path, strokes)
    print(f"plot: {output}")
    return strokes, rest


def capture(port: str, output: Path, baud: int, seconds: float) -> None:
    import serial

    print(f"capturing raw serial lines from {port} at {baud} baud to {output}")
    print("press Ctrl-C to stop")
    started = time.monotonic()
    count = 0
    try:
        with serial.Serial(port, baudrate=baud, timeout=1) as device, output.open("w") as sink:
            while seconds <= 0 or time.monotonic() - started < seconds:
                raw = device.readline()
                if not raw:
                    continue
                sink.write(raw.decode("utf-8", errors="replace").rstrip("\r\n") + "\n")
                count += 1
                if count % 100 == 0:
                    sink.flush()
    except KeyboardInterrupt:
        pass
    print(f"captured {count} lines to {output}")


def synthetic_capture() -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    sample_path = Path(__file__).resolve().parents[1] / "sample_imu.csv"
    _, source_gyro, source_accel = load_imu(sample_path)
    rng = np.random.default_rng(20260830)
    rate_hz = 219.0
    t_s = np.arange(0.0, 1.7, 1.0 / rate_hz)
    indices = rng.integers(0, len(source_gyro), size=len(t_s))
    gyro = np.mean(source_gyro, axis=0) + (
        source_gyro[indices] - np.mean(source_gyro, axis=0)
    )
    accel = np.mean(source_accel, axis=0) + (
        source_accel[indices] - np.mean(source_accel, axis=0)
    )

    stroke_start, stroke_duration, distance_m = 0.6, 0.5, 0.100
    phase = (t_s - stroke_start) / stroke_duration
    active = (phase >= 0.0) & (phase <= 1.0)
    accel_m_s2 = np.zeros(len(t_s))
    accel_m_s2[active] = (
        2.0 * np.pi * distance_m / stroke_duration**2 * np.sin(2.0 * np.pi * phase[active])
    )
    accel[:, 0] += accel_m_s2 / GRAVITY_M_S2
    accel[:, 0] += 0.001  # requested constant 1 mg accelerometer bias
    return t_s, gyro, accel, distance_m


def verify() -> None:
    t_s, gyro, accel, truth_m = synthetic_capture()
    strokes, rest, _vibration_g = reconstruct(t_s, gyro, accel)
    if not strokes:
        raise SystemExit("synthetic verification failed: no stroke detected")
    stroke = max(strokes, key=lambda item: item.duration_s)
    recovered_mm = np.linalg.norm(stroke.displacement_m) * 1000.0
    error_percent = (recovered_mm / (truth_m * 1000.0) - 1.0) * 100.0
    _, source_gyro, source_accel = load_imu(
        Path(__file__).resolve().parents[1] / "sample_imu.csv"
    )
    accel_noise_mg = np.std(source_accel, axis=0) * 1000.0
    gyro_noise_dps = np.std(source_gyro, axis=0)
    print("synthetic 100 mm / 0.5 s sinusoidal-acceleration stroke")
    print(
        "  source 1-sigma noise: "
        f"accel=({accel_noise_mg[0]:.2f}, {accel_noise_mg[1]:.2f}, "
        f"{accel_noise_mg[2]:.2f}) mg; "
        f"gyro=({gyro_noise_dps[0]:.3f}, {gyro_noise_dps[1]:.3f}, "
        f"{gyro_noise_dps[2]:.3f}) dps"
    )
    print("  noise rows resampled from proto/sample_imu.csv; bias: +1 mg on accel x")
    print(f"  rest detector: {100.0 * np.mean(rest):.1f}% rest")
    print(
        f"  recovered=({stroke.displacement_m[0] * 1000.0:+.2f}, "
        f"{stroke.displacement_m[1] * 1000.0:+.2f}) mm, magnitude={recovered_mm:.2f} mm "
        f"vs truth=100.00 mm ({error_percent:+.2f}% error)"
    )

    if abs(error_percent) > 5.0:
        raise SystemExit("synthetic verification failed: displacement error exceeds 5%")

    duration_s = 0.5
    test_t = np.linspace(0.0, duration_s, 111)
    zero_gyro = np.zeros((len(test_t), 3))
    gravity = np.array([0.0, 0.0, -1.0])
    vertical_only = np.tile(gravity, (len(test_t), 1))
    vertical_only[:, 2] += 0.1 * np.sin(2.0 * np.pi * test_t / duration_s)
    path, velocity, projected_accel = integrate_interval(
        test_t, zero_gyro, vertical_only, gravity, np.zeros(3)
    )
    rejected_mm = np.max(np.linalg.norm(path, axis=1)) * 1000.0
    print("synthetic out-of-plane acceleration pulse")
    print(
        f"  max planar accel={np.max(np.linalg.norm(projected_accel, axis=1)):.6f} m/s^2; "
        f"max planar speed={np.max(np.linalg.norm(velocity, axis=1)):.6f} m/s; "
        f"max path={rejected_mm:.6f} mm"
    )
    if rejected_mm > 1e-6:
        raise SystemExit("synthetic verification failed: vertical acceleration entered planar path")

    gate_t = np.arange(0.0, 2.0, 1.0 / 219.0)
    gate_gyro = np.zeros((len(gate_t), 3))
    gate_accel = np.tile(gravity, (len(gate_t), 1))
    slow_drag = (gate_t >= 0.6) & (gate_t <= 1.4)
    gate_accel[slow_drag, 0] += 0.006
    gate_accel[slow_drag, 2] += 0.010 * np.sin(2.0 * np.pi * 55.0 * gate_t[slow_drag])
    gate_rest, gate_energy = detect_rest(gate_t, gate_gyro, gate_accel)
    motion_detection = 100.0 * np.mean(~gate_rest[slow_drag])
    quiet_rest = 100.0 * np.mean(gate_rest[~slow_drag])
    print("synthetic smooth low-variance drag with 55 Hz friction vibration")
    print(
        f"  median active vibration={np.median(gate_energy[slow_drag]) * 1000.0:.2f} mg RMS; "
        f"active detected motion={motion_detection:.1f}%; quiet detected rest={quiet_rest:.1f}%"
    )
    if motion_detection < 75.0:
        raise SystemExit("synthetic verification failed: vibration gate missed smooth drag")


def main() -> None:
    # Line-buffer stdout so output appears live even when piped through tee.
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--analyze", metavar="FILE", type=Path)
    mode.add_argument("--cursor-sim", metavar="FILE", type=Path)
    mode.add_argument("--capture", metavar="FILE", type=Path)
    mode.add_argument("--verify", action="store_true")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=0.0, help="capture duration; 0 means until Ctrl-C")
    args = parser.parse_args()

    if args.analyze:
        report_analysis(args.analyze)
    elif args.cursor_sim:
        simulate_cursor(args.cursor_sim)
    elif args.capture:
        capture(args.port, args.capture, args.baud, args.seconds)
    else:
        verify()


if __name__ == "__main__":
    main()
