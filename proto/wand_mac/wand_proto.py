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
import sys
import time
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
REST_GYRO_DPS = 1.0
REST_ACCEL_G_TOLERANCE = 0.08


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
    print(f"Opening {port} at {baud} baud", file=sys.stderr)
    with serial.Serial(port, baudrate=baud, timeout=1.0) as connection:
        while True:
            line = connection.readline()
            if line:
                yield line.decode("utf-8", errors="replace")


def probe(lines):
    last_print = -math.inf
    for line in lines:
        parsed = parse_line(line)
        if not parsed or parsed[0] != "IMU":
            continue
        _, t_us, gyro_raw, accel_raw = parsed
        gyro = AXIS_REMAP @ gyro_raw
        accel = AXIS_REMAP @ accel_raw
        if t_us / 1e6 - last_print >= 0.1:
            last_print = t_us / 1e6
            print(
                f"t={t_us / 1e6:9.3f}s  "
                f"gyro dps  x={gyro[0]:+8.2f} y={gyro[1]:+8.2f} z={gyro[2]:+8.2f}  "
                f"accel g  x={accel[0]:+7.3f} y={accel[1]:+7.3f} z={accel[2]:+7.3f}"
            )


def intersect_screen(
    position_mm: np.ndarray, rotation: np.ndarray, display: Display
):
    ray = rotation[:, 1]  # Device/marker up is the pointing direction.
    if abs(ray[2]) < 1e-4:
        return None, ray
    distance = -position_mm[2] / ray[2]
    if distance <= 0:
        return None, ray
    hit_mm = position_mm + distance * ray
    target = display.center_px + np.array(
        [hit_mm[0] * display.width_px / display.width_mm,
         -hit_mm[1] * display.height_px / display.height_mm]
    )
    target[0] = np.clip(target[0], display.origin_x, display.origin_x + display.width_px - 1)
    target[1] = np.clip(target[1], display.origin_y, display.origin_y + display.height_px - 1)
    return target, ray


class FractionalCursor:
    def __init__(self, warp: bool):
        self.warp = warp
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


def heading_deg(rotation: np.ndarray, ray: np.ndarray) -> float:
    # In pointing posture, this is the actual horizontal ray angle.
    if abs(ray[2]) > 0.2:
        return math.degrees(math.atan2(ray[0], ray[2]))
    normal = rotation[:, 2]
    return math.degrees(math.atan2(normal[0], normal[2]))


def track(lines, calibration: Calibration, display: Display, warp: bool):
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
    bias = None
    alignment = None
    last_t_us = None
    gyro_range = 2048.0
    peak_gyro = 0.0
    cursor = FractionalCursor(warp)
    displayed_target = None
    rate_times = deque()
    drift_samples = deque()
    unwrapped_heading = None
    previous_heading = None
    drift = 0.0
    previous_rest = False
    last_hud_us = -10**18
    sample_count = 0
    first_sample_us = final_sample_us = None
    target_min = np.array([math.inf, math.inf])
    target_max = np.array([-math.inf, -math.inf])

    for line in lines:
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
        first_sample_us = t_us if first_sample_us is None else first_sample_us
        final_sample_us = t_us
        peak_gyro = max(peak_gyro, float(np.max(np.abs(gyro))))
        rate_times.append(t_us)
        while rate_times and t_us - rate_times[0] > 1_000_000:
            rate_times.popleft()

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

        dt = (t_us - last_t_us) / 1e6
        last_t_us = t_us
        if dt <= 0 or dt > 0.1:
            continue
        ahrs.set_sample_period(dt)
        corrected_gyro = gyro - bias
        ahrs.update_no_magnetometer(corrected_gyro, accel)
        rotation = alignment @ imufusion.quaternion_to_matrix(ahrs.get_quaternion())
        target, ray = intersect_screen(calibration.position_mm, rotation, display)
        if target is not None:
            displayed_target = cursor.move(target)
            target_min = np.minimum(target_min, target)
            target_max = np.maximum(target_max, target)

        rest = (
            np.linalg.norm(corrected_gyro) < REST_GYRO_DPS
            and abs(np.linalg.norm(accel) - 1.0) < REST_ACCEL_G_TOLERANCE
        )
        heading = heading_deg(rotation, ray)
        if previous_heading is None:
            unwrapped_heading = heading
        else:
            delta_heading = (heading - previous_heading + 180) % 360 - 180
            unwrapped_heading += delta_heading
        previous_heading = heading
        if rest:
            if not previous_rest:
                drift_samples.clear()
            drift_samples.append((t_us / 1e6, unwrapped_heading))
            while drift_samples and t_us / 1e6 - drift_samples[0][0] > 60:
                drift_samples.popleft()
            if len(drift_samples) >= 2 and drift_samples[-1][0] - drift_samples[0][0] >= 0.5:
                times = np.array([item[0] for item in drift_samples])
                angles = np.array([item[1] for item in drift_samples])
                drift = float(np.polyfit(times - times[0], angles, 1)[0] * 60)
        else:
            drift_samples.clear()
        previous_rest = rest

        if t_us - last_hud_us >= 500_000:
            sample_rate = max(0, len(rate_times) - 1)
            saturation = " SATURATION!" if peak_gyro > 0.9 * gyro_range else ""
            target_text = (
                f"{displayed_target[0]:.0f},{displayed_target[1]:.0f}"
                if displayed_target is not None else "no intersection"
            )
            print(
                f"TRACK {('REST' if rest else 'MOVING'):6s}  rate={sample_rate:3d} Hz"
                f"  drift={drift:+7.2f} deg/min  peak={peak_gyro:7.1f}/{gyro_range:.0f} dps"
                f"{saturation}  cursor={target_text}",
                flush=True,
            )
            last_hud_us = t_us

    if sample_count:
        duration = ((final_sample_us - first_sample_us) / 1e6) if sample_count > 1 else 0
        span = target_max - target_min
        span_text = (
            f"[{span[0]:.2f}, {span[1]:.2f}]"
            if np.all(np.isfinite(span)) else "no intersections"
        )
        print(
            f"SUMMARY samples={sample_count} duration={duration:.3f}s"
            f" peak_gyro={peak_gyro:.2f}dps drift={drift:+.3f}deg/min"
            f" target_span_px={span_text}"
        )
    elif bias_samples:
        print("SUMMARY replay ended before bias calibration completed")
    else:
        print("SUMMARY no IMU samples found")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--replay", type=Path, metavar="FILE")
    parser.add_argument("--fake-calib", action="store_true")
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--warp", action="store_true", help="warp cursor during replay")
    parser.add_argument("--fov", type=float, default=HORIZONTAL_FOV_DEG,
                        help="approximate webcam horizontal FOV in degrees")
    args = parser.parse_args()

    lines = replay_lines(args.replay) if args.replay else serial_lines(args.port, args.baud)
    if args.probe:
        probe(lines)
        return

    display = display_info()
    print(
        f"DISPLAY origin=({display.origin_x:.0f},{display.origin_y:.0f})"
        f" px={display.width_px:.0f}x{display.height_px:.0f}"
        f" physical_mm={display.width_mm:.1f}x{display.height_mm:.1f}"
    )
    calibration = fake_calibration() if args.fake_calib else camera_calibration(args.fov, display)
    if args.fake_calib:
        print("CALIBRATION fake position_mm=[0.0, 0.0, -500.0] quaternion_wxyz=[1,0,0,0]")
    track(lines, calibration, display, warp=args.warp or args.replay is None)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, serial.SerialException) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
