# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "opencv-python",
#   "numpy",
#   "imufusion",
#   "pyserial",
#   "pyobjc-framework-Quartz",
# ]
# ///

"""THROWAWAY Mac-side absolute-pointing prototype. See ../PROTO_SPEC.md."""

import argparse
import math
import select
import sys
import termios
import time
import tty
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import cv2
import imufusion
import numpy as np
import serial
from Quartz import (
    CGDisplayBounds,
    CGDisplayScreenSize,
    CGMainDisplayID,
    CGWarpMouseCursorPosition,
)

# Verify with a ruler before trusting pose distance. Active ArUco symbol is
# 240 px at ~0.0788 mm/px (see tools/gen_marker.py; asset embeds quiet zones).
MARKER_SIDE_MM = 18.9
HORIZONTAL_FOV_DEG = 65.0
CAMERA_Y_OFFSET_MM = 0.0  # Camera above the physical display edge, if applicable.
AXIS_REMAP = np.eye(3)  # Unknown board mounting; determine with --probe.
DEFAULT_PORT = "/dev/cu.usbmodem101"
BIAS_SECONDS = 2.0
DESK_BIAS_SECONDS = 3.0
REST_GYRO_DPS = 1.0
REST_ACCEL_G_TOLERANCE = 0.08
# Cursor feel knobs for the final pixel-coordinate one-euro filter.
MIN_CUTOFF = 1.0  # Hz: lower is steadier but adds lag.
BETA = 0.007  # Higher follows fast motion more closely.
D_CUTOFF = 1.0  # Hz: derivative smoothing.
SERIAL_RECONNECT = object()


@dataclass
class Calibration:
    rotation: np.ndarray  # device -> screen, screen +z points into the display
    position_mm: np.ndarray


@dataclass
class Display:
    display_id: int
    origin_x: float
    origin_y: float
    width_px: float
    height_px: float
    width_mm: float
    height_mm: float

    @property
    def center_px(self):
        return np.array(
            [self.origin_x + self.width_px / 2, self.origin_y + self.height_px / 2]
        )


def display_info() -> Display:
    display_id = CGMainDisplayID()
    bounds = CGDisplayBounds(display_id)
    physical = CGDisplayScreenSize(display_id)
    width_mm = float(physical.width) or 344.0
    height_mm = float(physical.height) or 223.0
    return Display(
        display_id,
        float(bounds.origin.x),
        float(bounds.origin.y),
        float(bounds.size.width),
        float(bounds.size.height),
        width_mm,
        height_mm,
    )


def matrix_to_quaternion(rotation: np.ndarray) -> np.ndarray:
    """Return a normalized w,x,y,z quaternion."""
    trace = np.trace(rotation)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.array(
            [s / 4, (rotation[2, 1] - rotation[1, 2]) / s,
             (rotation[0, 2] - rotation[2, 0]) / s,
             (rotation[1, 0] - rotation[0, 1]) / s]
        )
    else:
        i = int(np.argmax(np.diag(rotation)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + rotation[i, i] - rotation[j, j] - rotation[k, k]) * 2
        q = np.empty(4)
        q[0] = (rotation[k, j] - rotation[j, k]) / s
        q[i + 1] = s / 4
        q[j + 1] = (rotation[j, i] + rotation[i, j]) / s
        q[k + 1] = (rotation[k, i] + rotation[i, k]) / s
    return q / np.linalg.norm(q)


def fake_calibration() -> Calibration:
    # Device marker faces the camera, centered 0.5 m in front of the display.
    return Calibration(np.eye(3), np.array([0.0, 0.0, -500.0]))


def camera_calibration(fov_deg: float, display: Display) -> Calibration:
    capture = cv2.VideoCapture(0)
    if not capture.isOpened():
        raise RuntimeError(
            "Could not open webcam (check macOS Camera permission), or use --fake-calib."
        )

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    half = MARKER_SIDE_MM / 2
    # ArUco corners are TL, TR, BR, BL; local +y is marker/device up.
    object_points = np.array(
        [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]],
        dtype=np.float32,
    )
    camera_to_screen = np.diag([1.0, -1.0, -1.0])
    # Screen +y is up from center, so a top-edge camera is at +height/2.
    camera_position = np.array(
        [0.0, display.height_mm / 2 + CAMERA_Y_OFFSET_MM, 0.0]
    )
    window = "Wand calibration — show marker id 0; SPACE capture; Q cancel"

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError(
                    "Webcam stopped producing frames; retry or use --fake-calib."
                )
            height, width = frame.shape[:2]
            focal = width / (2 * math.tan(math.radians(fov_deg) / 2))
            intrinsics = np.array(
                [[focal, 0, width / 2], [0, focal, height / 2], [0, 0, 1]],
                dtype=np.float64,
            )
            corners, ids, _ = detector.detectMarkers(frame)
            selected = None
            rvec = tvec = None
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                matches = np.flatnonzero(ids.flatten() == 0)
                if len(matches):
                    selected = corners[int(matches[0])].reshape(4, 2).astype(np.float32)
                    ok, rvec, tvec = cv2.solvePnP(
                        object_points,
                        selected,
                        intrinsics,
                        np.zeros(5),
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if ok:
                        cv2.drawFrameAxes(
                            frame, intrinsics, np.zeros(5), rvec, tvec, MARKER_SIDE_MM / 2
                        )
            status = "SPACE: capture id 0" if rvec is not None else "Looking for ArUco id 0"
            cv2.putText(frame, status, (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if rvec is not None else (0, 180, 255), 2)
            cv2.imshow(window, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                raise RuntimeError("Calibration canceled.")
            if key == ord(" ") and rvec is not None:
                rotation_camera, _ = cv2.Rodrigues(rvec)
                rotation_screen = camera_to_screen @ rotation_camera
                position_screen = camera_position + camera_to_screen @ tvec.reshape(3)
                q = matrix_to_quaternion(rotation_screen)
                print(
                    "CALIBRATION"
                    f" position_mm={np.round(position_screen, 1).tolist()}"
                    f" quaternion_wxyz={np.round(q, 5).tolist()}"
                )
                return Calibration(rotation_screen, position_screen)
    finally:
        capture.release()
        cv2.destroyAllWindows()


def parse_line(line: str):
    line = line.strip()
    if line.startswith("IMU,"):
        fields = line.split(",")
        if len(fields) != 8:
            return None
        try:
            return (
                "IMU",
                int(fields[1]),
                np.array([float(value) for value in fields[2:5]]),
                np.array([float(value) for value in fields[5:8]]),
            )
        except ValueError:
            return None
    if line.startswith("CFG,"):
        values = {}
        try:
            for field in line.split(",")[1:]:
                key, value = field.split("=", 1)
                values[key] = float(value)
            return "CFG", values
        except (ValueError, KeyError):
            return None
    return None


def replay_lines(path: Path):
    with path.open("r", encoding="utf-8") as replay:
        yield from replay


def serial_lines(port: str, baud: int):
    connected_once = False
    while True:
        print(f"Opening {port} at {baud} baud", file=sys.stderr)
        try:
            with serial.Serial(port, baudrate=baud, timeout=1.0) as connection:
                if connected_once:
                    yield SERIAL_RECONNECT
                connected_once = True
                while True:
                    line = connection.readline()
                    if line:
                        yield line.decode("utf-8", errors="replace")
        except (OSError, serial.SerialException):
            print("# serial lost, reconnecting...", file=sys.stderr, flush=True)
            time.sleep(0.5)


def clean_interrupts(lines):
    try:
        yield from lines
    except KeyboardInterrupt:
        return


def collect_gyro_bias(lines, seconds: float):
    samples = []
    start_us = None
    for line in clean_interrupts(lines):
        if line is SERIAL_RECONNECT:
            continue
        parsed = parse_line(line)
        if not parsed or parsed[0] != "IMU":
            continue
        _, t_us, gyro_raw, _ = parsed
        if start_us is None or t_us < start_us:
            samples.clear()
            start_us = t_us
        samples.append(AXIS_REMAP @ gyro_raw)
        if t_us - start_us >= seconds * 1e6:
            values = np.asarray(samples)
            return np.mean(values, axis=0), np.std(values, axis=0)
    raise RuntimeError("IMU stream ended before desk bias calibration completed.")


def probe(lines):
    last_print = -math.inf
    sample_count = 0
    first_sample_us = final_sample_us = None
    for line in clean_interrupts(lines):
        if line is SERIAL_RECONNECT:
            continue
        parsed = parse_line(line)
        if not parsed or parsed[0] != "IMU":
            continue
        _, t_us, gyro_raw, accel_raw = parsed
        gyro = AXIS_REMAP @ gyro_raw
        accel = AXIS_REMAP @ accel_raw
        sample_count += 1
        first_sample_us = t_us if first_sample_us is None else first_sample_us
        final_sample_us = t_us
        if t_us / 1e6 - last_print >= 0.1:
            last_print = t_us / 1e6
            print(
                f"t={t_us / 1e6:9.3f}s  "
                f"gyro dps  x={gyro[0]:+8.2f} y={gyro[1]:+8.2f} z={gyro[2]:+8.2f}  "
                f"accel g  x={accel[0]:+7.3f} y={accel[1]:+7.3f} z={accel[2]:+7.3f}"
            )
    duration = ((final_sample_us - first_sample_us) / 1e6) if sample_count > 1 else 0
    print(f"SUMMARY samples={sample_count} duration={duration:.3f}s")


def intersect_screen(
    position_mm: np.ndarray, rotation: np.ndarray, display: Display
):
    ray = rotation[:, 1]  # Device/marker up is the pointing direction.
    if abs(ray[2]) < 1e-4:
        return None, ray, None
    distance = (0.0 - position_mm[2]) / ray[2]
    if distance <= 0:
        return None, ray, None
    hit_mm = position_mm + distance * ray
    # Screen y is up from its center; Quartz y is down from the desktop's top.
    target = np.array(
        [display.center_px[0] + hit_mm[0] * display.width_px / display.width_mm,
         display.center_px[1] - hit_mm[1] * display.height_px / display.height_mm]
    )
    target[0] = np.clip(target[0], display.origin_x, display.origin_x + display.width_px - 1)
    target[1] = np.clip(target[1], display.origin_y, display.origin_y + display.height_px - 1)
    return target, ray, hit_mm


class OneEuroFilter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous_time = None
        self.previous_value = None
        self.previous_filtered = None
        self.previous_derivative = np.zeros(2)

    @staticmethod
    def alpha(cutoff, dt):
        return 1.0 / (1.0 + 1.0 / (2.0 * math.pi * cutoff * dt))

    def filter(self, value: np.ndarray, sample_time: float):
        if self.previous_time is None:
            self.previous_time = sample_time
            self.previous_value = value.copy()
            self.previous_filtered = value.copy()
            return value.copy()
        dt = sample_time - self.previous_time
        if dt <= 0:
            self.reset()
            return self.filter(value, sample_time)
        derivative = (value - self.previous_value) / dt
        derivative_alpha = self.alpha(D_CUTOFF, dt)
        filtered_derivative = (
            derivative_alpha * derivative
            + (1.0 - derivative_alpha) * self.previous_derivative
        )
        cutoff = MIN_CUTOFF + BETA * np.abs(filtered_derivative)
        value_alpha = self.alpha(cutoff, dt)
        filtered = value_alpha * value + (1.0 - value_alpha) * self.previous_filtered
        self.previous_time = sample_time
        self.previous_value = value.copy()
        self.previous_filtered = filtered
        self.previous_derivative = filtered_derivative
        return filtered.copy()


class FractionalCursor:
    def __init__(self, warp: bool):
        self.warp = warp
        self.reset()

    def reset(self):
        self.continuous = None
        self.integer = None
        self.carry = np.zeros(2)

    def move(self, target: np.ndarray):
        if self.continuous is None:
            self.continuous = target.copy()
            self.integer = np.rint(target).astype(int)
        else:
            delta = target - self.continuous + self.carry
            step = np.trunc(delta).astype(int)
            self.carry = delta - step
            self.integer += step
            self.continuous = target.copy()
        if self.warp:
            CGWarpMouseCursorPosition(tuple(self.integer.astype(float)))
        return self.integer.copy()


class TerminalKeys:
    def __enter__(self):
        self.fd = None
        self.previous_settings = None
        if sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.previous_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def read(self):
        if self.fd is not None and select.select([self.fd], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return None

    def __exit__(self, *_):
        if self.previous_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.previous_settings)


def lines_with_keys(lines):
    with TerminalKeys() as keys:
        for line in clean_interrupts(lines):
            yield keys.read(), line


def yaw_rotation(angle: float):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array([[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]])


def track(
    lines, calibration: Calibration, display: Display, warp: bool, debug: bool,
    initial_bias=None,
):
    ahrs = imufusion.Ahrs()
    ahrs.set_settings(
        imufusion.AhrsSettings(
            sample_rate=200,
            convention=imufusion.CONVENTION_NWU,
            gain=0.5,
            gyroscope_range=2048,
            acceleration_rejection=10,
            magnetic_rejection=0,
            rejection_timeout=5,
        )
    )

    bias_samples = []
    buffered = []
    bias_start_us = None
    bias = None if initial_bias is None else np.asarray(initial_bias)
    alignment = None
    last_t_us = None
    gyro_range = 2048.0
    peak_gyro = 0.0
    clip_count = 0
    clip_times = deque()
    cursor_filter = OneEuroFilter()
    cursor = FractionalCursor(warp)
    # Translation is not tracked after calibration; retaining the webcam-height
    # y origin makes a level post-flip ray target the top edge forever.
    ray_origin_mm = calibration.position_mm.copy()
    ray_origin_mm[1] = 0.0
    displayed_target = None
    raw_hit_mm = None
    ray = np.array([0.0, 1.0, 0.0])
    rate_times = deque()
    drift_samples = deque()
    rest_elapsed = 0.0
    integrated_yaw = 0.0
    drift = 0.0
    last_hud_us = -10**18
    sample_count = 0
    final_sample_us = None
    duration = 0.0
    target_min = np.array([math.inf, math.inf])
    target_max = np.array([-math.inf, -math.inf])
    yaw_reference = np.eye(3)
    recenter_requested = False

    for key, line in lines_with_keys(lines):
        if key == "q":
            break
        if key == "r":
            recenter_requested = True
        if line is SERIAL_RECONNECT:
            cursor_filter.reset()
            cursor.reset()
            displayed_target = None
            continue
        parsed = parse_line(line)
        if not parsed:
            continue
        if parsed[0] == "CFG":
            gyro_range = parsed[1].get("gyro_range_dps", gyro_range)
            continue

        _, t_us, gyro_raw, accel_raw = parsed
        gyro = AXIS_REMAP @ gyro_raw
        accel = AXIS_REMAP @ accel_raw
        sample_count += 1
        if final_sample_us is not None and 0 < t_us - final_sample_us <= 100_000:
            duration += (t_us - final_sample_us) / 1e6
        final_sample_us = t_us
        peak_gyro = max(peak_gyro, float(np.max(np.abs(gyro))))
        if last_t_us is not None and t_us <= last_t_us:
            rate_times.clear()
            clip_times.clear()
            last_hud_us = -10**18
        if np.any(np.abs(gyro) >= 2040.0):
            # Clipping loses angular motion, so orientation is untrustworthy after a clip.
            clip_count += 1
            clip_times.append(t_us)
        rate_times.append(t_us)
        while rate_times and t_us - rate_times[0] > 1_000_000:
            rate_times.popleft()
        while clip_times and t_us - clip_times[0] > 2_000_000:
            clip_times.popleft()

        if bias is None:
            if bias_start_us is None:
                bias_start_us = t_us
            bias_samples.append(gyro)
            buffered.append((t_us, gyro, accel))
            if t_us - bias_start_us < BIAS_SECONDS * 1e6:
                if t_us - last_hud_us >= 500_000:
                    print(
                        f"BIAS {max(0, BIAS_SECONDS - (t_us - bias_start_us) / 1e6):.1f}s"
                        f"  samples={len(bias_samples)}",
                        flush=True,
                    )
                    last_hud_us = t_us
                continue
            bias = np.mean(bias_samples, axis=0)
            print(f"GYRO BIAS dps={np.round(bias, 5).tolist()}")
            for buffered_t, buffered_gyro, buffered_accel in buffered:
                if last_t_us is not None:
                    dt = max(0.0001, min(0.05, (buffered_t - last_t_us) / 1e6))
                    ahrs.set_sample_period(dt)
                    ahrs.update_no_magnetometer(buffered_gyro - bias, buffered_accel)
                last_t_us = buffered_t
            # AHRS owns gravity correction; this rigid alignment gives it the
            # webcam-observed device->screen yaw and position at calibration.
            alignment = calibration.rotation @ imufusion.quaternion_to_matrix(
                ahrs.get_quaternion()
            ).T
            buffered.clear()
            continue

        if alignment is None:
            # Desk bias is already known. Converge tilt from this first gravity
            # sample immediately, then preserve the webcam-observed yaw.
            ahrs.set_sample_period(1.0 / 200.0)
            for _ in range(601):  # Complete Fusion's 3 s startup instantaneously.
                ahrs.update_no_magnetometer(np.zeros(3), accel)
            alignment = calibration.rotation @ imufusion.quaternion_to_matrix(
                ahrs.get_quaternion()
            ).T
            last_t_us = t_us
            continue

        dt = (t_us - last_t_us) / 1e6
        last_t_us = t_us
        if dt <= 0 or dt > 0.1:
            continue
        ahrs.set_sample_period(dt)
        corrected_gyro = gyro - bias
        ahrs.update_no_magnetometer(corrected_gyro, accel)
        base_rotation = alignment @ imufusion.quaternion_to_matrix(ahrs.get_quaternion())
        rotation = yaw_reference @ base_rotation
        if recenter_requested:
            azimuth = math.atan2(rotation[0, 1], rotation[2, 1])
            yaw_reference = yaw_rotation(-azimuth) @ yaw_reference
            rotation = yaw_reference @ base_rotation
            recenter_requested = False
            cursor_filter.reset()
            cursor.reset()
            print("# recentered", flush=True)
        target, ray, raw_hit_mm = intersect_screen(ray_origin_mm, rotation, display)
        if target is not None:
            filtered_target = cursor_filter.filter(target, t_us / 1e6)
            displayed_target = cursor.move(filtered_target)
            target_min = np.minimum(target_min, filtered_target)
            target_max = np.maximum(target_max, filtered_target)
        else:
            cursor_filter.reset()
            cursor.reset()
            displayed_target = None

        rest = (
            np.linalg.norm(corrected_gyro) < REST_GYRO_DPS
            and abs(np.linalg.norm(accel) - 1.0) < REST_ACCEL_G_TOLERANCE
        )
        if rest:
            gravity_axis = -accel / np.linalg.norm(accel)
            yaw_rate = float(np.dot(corrected_gyro, gravity_axis))
            rest_elapsed += dt
            integrated_yaw += yaw_rate * dt
            drift_samples.append((rest_elapsed, integrated_yaw))
            while drift_samples and rest_elapsed - drift_samples[0][0] > 60:
                drift_samples.popleft()
            if len(drift_samples) >= 2 and drift_samples[-1][0] - drift_samples[0][0] >= 0.5:
                times = np.array([item[0] for item in drift_samples])
                angles = np.array([item[1] for item in drift_samples])
                drift = float(np.polyfit(times - times[0], angles, 1)[0] * 60)

        if t_us - last_hud_us >= 500_000:
            sample_rate = max(0, len(rate_times) - 1)
            saturation = " SATURATION!" if peak_gyro > 0.9 * gyro_range else ""
            clip_warning = " CLIP!" if clip_times else ""
            target_text = (
                f"{displayed_target[0]:.0f},{displayed_target[1]:.0f}"
                if displayed_target is not None else "no intersection"
            )
            debug_text = ""
            if debug:
                azimuth = math.degrees(math.atan2(ray[0], ray[2]))
                elevation = math.degrees(
                    math.atan2(ray[1], math.hypot(ray[0], ray[2]))
                )
                hit_text = (
                    f"{raw_hit_mm[0]:+.1f},{raw_hit_mm[1]:+.1f}"
                    if raw_hit_mm is not None else "none"
                )
                debug_text = (
                    f"  ray_az/el={azimuth:+.1f}/{elevation:+.1f}deg"
                    f" hit_mm={hit_text}"
                )
            print(
                f"TRACK {('REST' if rest else 'MOVING'):6s}  rate={sample_rate:3d} Hz"
                f"  drift={drift:+7.2f} deg/min  peak={peak_gyro:7.1f}/{gyro_range:.0f} dps"
                f"{saturation}{clip_warning}  cursor={target_text}{debug_text}",
                flush=True,
            )
            last_hud_us = t_us

    if sample_count:
        span = target_max - target_min
        span_text = (
            f"[{span[0]:.2f}, {span[1]:.2f}]"
            if np.all(np.isfinite(span)) else "no intersections"
        )
        print(
            f"SUMMARY samples={sample_count} duration={duration:.3f}s"
            f" peak_gyro={peak_gyro:.2f}dps clips={clip_count}"
            f" drift={drift:+.3f}deg/min target_span_px={span_text}"
        )
    elif bias_samples:
        print("SUMMARY replay ended before bias calibration completed")
    else:
        print("SUMMARY no IMU samples found")


def main():
    # Line-buffer stdout so output appears live even when piped through tee.
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--replay", type=Path, metavar="FILE")
    parser.add_argument("--fake-calib", action="store_true")
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--warp", action="store_true", help="warp cursor during replay")
    parser.add_argument("--debug", action="store_true", help="show ray and raw intersection")
    parser.add_argument("--fov", type=float, default=HORIZONTAL_FOV_DEG,
                        help="approximate webcam horizontal FOV in degrees")
    args = parser.parse_args()

    if args.probe:
        lines = replay_lines(args.replay) if args.replay else serial_lines(args.port, args.baud)
        probe(lines)
        return

    display = display_info()
    print(
        f"DISPLAY origin=({display.origin_x:.0f},{display.origin_y:.0f})"
        f" px={display.width_px:.0f}x{display.height_px:.0f}"
        f" physical_mm={display.width_mm:.1f}x{display.height_mm:.1f}"
    )
    desk_bias = None
    use_fake_calibration = args.fake_calib or args.replay is not None
    if not use_fake_calibration:
        input("place device flat on desk, press Enter")
        desk_lines = serial_lines(args.port, args.baud)
        try:
            desk_bias, desk_std = collect_gyro_bias(desk_lines, DESK_BIAS_SECONDS)
        finally:
            desk_lines.close()
        print(
            f"GYRO BIAS dps={np.round(desk_bias, 5).tolist()}"
            f" std_dps={np.round(desk_std, 5).tolist()}"
        )
        if np.any(desk_std > 1.0):
            print("WARNING gyro std > 1 dps: device was moving during desk bias")

    calibration = fake_calibration() if use_fake_calibration else camera_calibration(args.fov, display)
    if use_fake_calibration:
        print("CALIBRATION fake position_mm=[0.0, 0.0, -500.0] quaternion_wxyz=[1,0,0,0]")
    lines = replay_lines(args.replay) if args.replay else serial_lines(args.port, args.baud)
    track(
        lines, calibration, display, warp=args.warp or args.replay is None,
        debug=args.debug, initial_bias=desk_bias,
    )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError, serial.SerialException) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
