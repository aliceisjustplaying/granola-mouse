# /// script
# requires-python = ">=3.11"
# dependencies = [
#   "opencv-python",
#   "numpy",
#   "imufusion==1.3.3",
#   "pyserial",
#   "pyobjc-framework-Quartz",
# ]
# ///

"""THROWAWAY Mac-side absolute-pointing prototype. See ../PROTO_SPEC.md.

Default: desk bias, flip flat, aim at screen center, press r, then point.
Use ``--distance-mm MM`` for assumed device distance and ``--sens FACTOR`` for cursor gain.
Use ``--camera`` only to run the legacy webcam ArUco calibration diagnostic.
Mount roll defaults to 0 degrees without a camera and -90 degrees with one;
``--mount-roll DEG`` overrides either default.
"""

import argparse
import math
import select
import subprocess
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
    CGEventCreate,
    CGEventCreateMouseEvent,
    CGEventGetLocation,
    CGEventPost,
    CGMainDisplayID,
    CGWarpMouseCursorPosition,
    kCGEventLeftMouseDown,
    kCGEventLeftMouseDragged,
    kCGEventLeftMouseUp,
    kCGHIDEventTap,
    kCGMouseButtonLeft,
)

# Verify with a ruler before trusting pose distance. Active ArUco symbol is
# 240 px at ~0.0788 mm/px (see tools/gen_marker.py; asset embeds quiet zones).
MARKER_SIDE_MM = 18.9
HORIZONTAL_FOV_DEG = 65.0
CAMERA_Y_OFFSET_MM = 0.0  # Camera above the physical display edge, if applicable.
AXIS_REMAP = np.eye(3)  # Unknown board mounting; determine with --probe.
# The on-device marker is rendered about 90 degrees from its assumed orientation,
# baking roll into webcam calibration. Assumed calibration has no marker roll.
ASSUMED_MOUNT_ROLL_DEG = 0.0
CAMERA_MOUNT_ROLL_DEG = -90.0
# With body +x pointing forward, positive body +z yaw (physical left) already
# moves toward negative screen x; no additional horizontal mirror is needed.
SCREEN_X_SIGN = 1.0
DEFAULT_PORT = "/dev/cu.usbmodem101"
BIAS_SECONDS = 2.0
DESK_BIAS_SECONDS = 3.0
ALIGNMENT_SECONDS = 0.5
AHRS_STARTUP_SECONDS = 3.0
AHRS_SAMPLE_RATE_HZ = 200
AHRS_REJECTION_TIMEOUT_SECONDS = 5
REST_WINDOW_SECONDS = 0.5
REST_WINDOW_MIN_SPAN_FRACTION = 0.9
# air4 settled desk gyro deviation is ~0.83 dps median, while YAW_SWEEP's
# minimum is 2.05 dps. Acceleration deviation on the settled desk is <0.01 g.
REST_GYRO_STD_DPS = 1.5
REST_ACCEL_STD_G = 0.02
REST_BIAS_DELAY_SECONDS = 1.5
REST_BIAS_UPDATE_SECONDS = 1.0
REST_BIAS_LEAK = 0.1
GYRO_PEAK_WINDOW_SECONDS = 2.0
# Cursor feel knobs for the final pixel-coordinate one-euro filter.
MIN_CUTOFF = 1.0  # Hz: lower is steadier but adds lag.
BETA = 0.007  # Higher follows fast motion more closely.
D_CUTOFF = 1.0  # Hz: derivative smoothing.
SERIAL_RECONNECT = object()

# AIR round 6. A duration of None waits for a keypress; SPACE always advances,
# while the two recenter steps also advance when their requested R key is pressed.
PROTOCOL = [
    ("FLIP_AND_CENTER", "Flip device flat like a remote, aim at screen center, press r", None),
    ("HOLD_STILL", "Hold still", 10),
    ("YAW_SWEEP", "Slow yaw sweep left-right", 15),
    ("PITCH_SWEEP", "Slow pitch sweep up-down", 15),
    ("CORNER_TL", "Aim at the top-left corner", 4),
    ("CORNER_TR", "Aim at the top-right corner", 4),
    ("CORNER_BR", "Aim at the bottom-right corner", 4),
    ("CORNER_BL", "Aim at the bottom-left corner", 4),
    ("DESK_REST", "Place device flat on desk, hands off", 60),
    ("PICK_UP_AND_CENTER", "Pick up, aim at center, press r", None),
    ("FREE_WAVE", "Free wave", 15),
    ("DONE", "Done — press q", None),
]


def speak(instruction: str, enabled: bool = True):
    """Ask macOS to speak without blocking, and tolerate a missing `say`."""
    if not enabled:
        return
    try:
        subprocess.Popen(
            ["say", instruction],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError:
        pass


class StepEngine:
    def __init__(self, protocol, marker, speaker=speak, announce=print):
        self.protocol = protocol
        self.marker = marker
        self.speaker = speaker
        self.announce = announce
        self.index = -1
        self.started_at = None
        self.warning_spoken = False
        self.finished = False

    @property
    def current(self):
        if self.finished or self.index < 0:
            return None
        return self.protocol[self.index]

    def start(self, now: float, last_t_us=None):
        if not self.protocol:
            self.finished = True
            return
        self.index = 0
        self._start_current(now, last_t_us)

    def _start_current(self, now: float, last_t_us):
        name, instruction, _ = self.protocol[self.index]
        self.started_at = now
        self.warning_spoken = False
        self.marker(self.index + 1, name, "START", last_t_us)
        self.announce(
            "\n" + "=" * 72 + "\n"
            f"GUIDED STEP {self.index + 1}/{len(self.protocol)} — {name}\n"
            f"{instruction}\n"
            + "=" * 72
        )
        self.speaker(instruction)

    def advance(self, now: float, last_t_us=None):
        if self.current is None:
            return
        name, _, _ = self.current
        self.marker(self.index + 1, name, "END", last_t_us)
        self.index += 1
        if self.index >= len(self.protocol):
            self.finished = True
            return
        self._start_current(now, last_t_us)

    def advance_for_key(self, key: str, now: float, last_t_us=None):
        if self.current is None:
            return
        _, instruction, duration = self.current
        if key == " " or (
            key == "r" and duration is None and "press r" in instruction.lower()
        ):
            self.advance(now, last_t_us)

    def update(self, now: float, last_t_us=None):
        while self.current is not None:
            _, _, duration = self.current
            if duration is None:
                return
            elapsed = now - self.started_at
            remaining = duration - elapsed
            if duration >= 10 and remaining <= 3 and not self.warning_spoken:
                self.speaker("3 seconds left")
                self.warning_spoken = True
            if remaining > 0:
                return
            # Anchor the following step to the scheduled boundary so sparse or
            # replayed samples can advance through more than one timed step.
            next_start = self.started_at + duration
            self.advance(next_start, last_t_us)

    def remaining_text(self, now: float):
        if self.current is None:
            return "complete"
        duration = self.current[2]
        if duration is None:
            return "waiting for key"
        return f"{max(0, math.ceil(duration - (now - self.started_at)))}s remaining"

    def finish(self, last_t_us=None):
        if self.current is not None:
            name = self.current[0]
            self.marker(self.index + 1, name, "END", last_t_us)
            self.finished = True


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


def assumed_calibration(distance_mm: float) -> Calibration:
    """Place an identity-oriented device in front of the screen center."""
    return Calibration(np.eye(3), np.array([0.0, 0.0, -distance_mm]))


def select_calibration(
    use_camera: bool, fov_deg: float, display: Display, distance_mm: float
) -> Calibration:
    if use_camera:
        return camera_calibration(fov_deg, display)
    return assumed_calibration(distance_mm)


def select_mount_roll(use_camera: bool, override_deg: float | None) -> float:
    if override_deg is not None:
        return override_deg
    return CAMERA_MOUNT_ROLL_DEG if use_camera else ASSUMED_MOUNT_ROLL_DEG


def camera_calibration(fov_deg: float, display: Display) -> Calibration:
    capture = cv2.VideoCapture(0)
    if not capture.isOpened():
        raise RuntimeError(
            "Could not open webcam (check macOS Camera permission)."
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
                    "Webcam stopped producing frames; retry calibration."
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
    if line.startswith("BTN,"):
        fields = line.split(",")
        if len(fields) != 3 or fields[2] not in ("0", "1"):
            return None
        try:
            return "BTN", int(fields[1]), fields[2] == "1"
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


def recording_lines(lines, recording):
    for line in lines:
        if line is not SERIAL_RECONNECT:
            recording.write(line)
            recording.flush()
        yield line


def recorded_lines(lines, path: Path):
    with path.open("a", encoding="utf-8") as recording:
        yield from recording_lines(lines, recording)


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


def screen_pointing_ray(rotation: np.ndarray) -> np.ndarray:
    """Return the body +x pointing ray in screen coordinates."""
    ray = rotation[:, 0].copy()
    ray[0] *= SCREEN_X_SIGN
    return ray


def intersect_screen_ray(
    position_mm: np.ndarray, ray: np.ndarray, display: Display
):
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


def intersect_screen(
    position_mm: np.ndarray, rotation: np.ndarray, display: Display
):
    return intersect_screen_ray(position_mm, screen_pointing_ray(rotation), display)


def scale_target_about_center(
    target_px: np.ndarray, center_px: np.ndarray, sensitivity: float
) -> np.ndarray:
    return center_px + (target_px - center_px) * sensitivity


class RollingRestDetector:
    """Classify rest from bias-independent raw IMU variation."""

    def __init__(self):
        self.samples = deque()
        self.gyro_mean = None
        self.gyro_deviation_dps = math.inf
        self.accel_deviation_g = math.inf

    def reset(self):
        self.samples.clear()
        self.gyro_mean = None
        self.gyro_deviation_dps = math.inf
        self.accel_deviation_g = math.inf

    def update(self, t_us: int, gyro: np.ndarray, accel: np.ndarray) -> bool:
        if self.samples and t_us <= self.samples[-1][0]:
            self.reset()
        self.samples.append((t_us, gyro.copy(), accel.copy()))
        while (
            self.samples
            and t_us - self.samples[0][0] > REST_WINDOW_SECONDS * 1e6
        ):
            self.samples.popleft()

        gyro_values = np.asarray([sample[1] for sample in self.samples])
        accel_values = np.asarray([sample[2] for sample in self.samples])
        self.gyro_mean = np.mean(gyro_values, axis=0)
        gyro_delta = gyro_values - self.gyro_mean
        accel_delta = accel_values - np.mean(accel_values, axis=0)
        self.gyro_deviation_dps = float(
            np.sqrt(np.mean(np.sum(gyro_delta**2, axis=1)))
        )
        self.accel_deviation_g = float(
            np.sqrt(np.mean(np.sum(accel_delta**2, axis=1)))
        )
        window_span = t_us - self.samples[0][0]
        window_ready = (
            window_span
            >= REST_WINDOW_SECONDS * REST_WINDOW_MIN_SPAN_FRACTION * 1e6
        )
        return (
            window_ready
            and self.gyro_deviation_dps < REST_GYRO_STD_DPS
            and self.accel_deviation_g < REST_ACCEL_STD_G
        )


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


def current_cursor_position():
    point = CGEventGetLocation(CGEventCreate(None))
    return np.array([float(point.x), float(point.y)])


def post_left_mouse_event(event_type, position):
    event = CGEventCreateMouseEvent(
        None, event_type, tuple(np.asarray(position, dtype=float)), kCGMouseButtonLeft
    )
    CGEventPost(kCGHIDEventTap, event)


class WiredButton:
    def __init__(self, post_events: bool):
        self.post_events = post_events
        self.held = False
        self.clicks = 0

    def update(self, pressed: bool, position=None):
        if pressed == self.held:
            return
        if self.post_events:
            if position is None:
                position = current_cursor_position()
            event_type = kCGEventLeftMouseDown if pressed else kCGEventLeftMouseUp
            post_left_mouse_event(event_type, position)
        self.held = pressed
        if pressed:
            self.clicks += 1

    def release(self, position=None):
        if self.held:
            self.update(False, position)


class FractionalCursor:
    def __init__(self, warp: bool):
        self.warp = warp
        self.reset()

    def reset(self):
        self.continuous = None
        self.integer = None
        self.carry = np.zeros(2)

    def move(self, target: np.ndarray, dragging=False):
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
            if dragging:
                post_left_mouse_event(kCGEventLeftMouseDragged, self.integer)
            else:
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


def pointing_axis_roll(angle: float):
    """Roll around device +x, its pointing axis."""
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]]
    )


def shortest_arc_rotation(source: np.ndarray, target: np.ndarray):
    """Return the minimal rotation that maps source onto target."""
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    source = source / np.linalg.norm(source)
    target = target / np.linalg.norm(target)
    cosine = float(np.clip(np.dot(source, target), -1.0, 1.0))
    cross = np.cross(source, target)
    sine = np.linalg.norm(cross)
    if sine < 1e-12:
        if cosine > 0.0:
            return np.eye(3)
        # Any axis perpendicular to source gives the same 180-degree mapping.
        basis = np.zeros(3)
        basis[int(np.argmin(np.abs(source)))] = 1.0
        axis = np.cross(source, basis)
        axis /= np.linalg.norm(axis)
        return 2.0 * np.outer(axis, axis) - np.eye(3)
    cross_matrix = np.array(
        [
            [0.0, -cross[2], cross[1]],
            [cross[2], 0.0, -cross[0]],
            [-cross[1], cross[0], 0.0],
        ]
    )
    return np.eye(3) + cross_matrix + cross_matrix @ cross_matrix * (
        (1.0 - cosine) / sine**2
    )


def gravity_aligned_screen_transform(screen_from_earth: np.ndarray):
    """Keep visual heading while mapping NWU earth-up to screen-up exactly."""
    screen_up = np.array([0.0, 1.0, 0.0])
    screen_north = screen_from_earth[:, 0].copy()
    screen_north -= np.dot(screen_north, screen_up) * screen_up
    if np.linalg.norm(screen_north) < 1e-6:
        screen_west = screen_from_earth[:, 1].copy()
        screen_west -= np.dot(screen_west, screen_up) * screen_up
        screen_west /= np.linalg.norm(screen_west)
        screen_north = np.cross(screen_west, screen_up)
    else:
        screen_north /= np.linalg.norm(screen_north)
    screen_west = np.cross(screen_up, screen_north)
    return np.column_stack((screen_north, screen_west, screen_up))


def aligned_screen_transform(
    calibration_rotation: np.ndarray,
    mount_correction: np.ndarray,
    ahrs_rotation: np.ndarray,
):
    visual_alignment = calibration_rotation @ mount_correction @ ahrs_rotation.T
    return gravity_aligned_screen_transform(visual_alignment)


def make_ahrs():
    ahrs = imufusion.Ahrs()
    ahrs.set_settings(
        imufusion.AhrsSettings(
            sample_rate=AHRS_SAMPLE_RATE_HZ,
            convention=imufusion.CONVENTION_NWU,
            gain=0.5,
            gyroscope_range=2048,
            acceleration_rejection=10,
            magnetic_rejection=0,
            # imufusion 1.3.3 defines this public setting in seconds and
            # converts it to samples internally using sample_rate.
            rejection_timeout=AHRS_REJECTION_TIMEOUT_SECONDS,
        )
    )
    return ahrs


def track(
    lines, calibration: Calibration, display: Display, warp: bool, debug: bool,
    initial_bias=None, mount_roll_deg=ASSUMED_MOUNT_ROLL_DEG, guided=False,
    recording=None, voice=True, sensitivity=1.0,
):
    ahrs = make_ahrs()

    bias_samples = []
    buffered = []
    bias_start_us = None
    bias = None if initial_bias is None else np.asarray(initial_bias)
    alignment = None
    alignment_start_us = None
    alignment_accel_samples = []
    last_t_us = None
    gyro_range = 2048.0
    peak_gyro = 0.0
    gyro_peaks = deque()
    clip_count = 0
    clip_times = deque()
    cursor_filter = OneEuroFilter()
    cursor = FractionalCursor(warp)
    button = WiredButton(post_events=warp)
    rest_detector = RollingRestDetector()
    # Translation is not tracked after calibration; retaining the webcam-height
    # y origin makes a level post-flip ray target the top edge forever.
    ray_origin_mm = calibration.position_mm.copy()
    ray_origin_mm[1] = 0.0
    displayed_target = None
    raw_hit_mm = None
    ray = np.array([1.0, 0.0, 0.0])
    rate_times = deque()
    drift_samples = deque()
    rest_elapsed = 0.0
    integrated_yaw = 0.0
    rest_start_us = None
    last_bias_update_us = None
    bias_update_printed = False
    drift = 0.0
    last_hud_us = -10**18
    sample_count = 0
    final_sample_us = None
    duration = 0.0
    target_min = np.array([math.inf, math.inf])
    target_max = np.array([-math.inf, -math.inf])
    pointing_reference = np.eye(3)
    recenter_requested = False
    recenter_count = 0
    mount_correction = pointing_axis_roll(math.radians(mount_roll_deg))
    protocol_elapsed = 0.0
    protocol_last_t_us = None

    def marker(step, name, event, t_us):
        device_time = "-" if t_us is None else str(t_us)
        marker_line = (
            f"# STEP {step} {name} {event} host={time.time():.3f}"
            f" t_us={device_time}"
        )
        print(marker_line, flush=True)
        if recording is not None:
            recording.write(marker_line + "\n")
            recording.flush()

    steps = StepEngine(
        PROTOCOL,
        marker,
        speaker=lambda instruction: speak(instruction, enabled=voice),
    ) if guided else None
    if steps is not None:
        steps.start(protocol_elapsed)

    for key, line in lines_with_keys(lines):
        if key == "q":
            if steps is not None:
                steps.finish(final_sample_us)
            break
        if key == "r":
            recenter_requested = True
        if steps is not None and key is not None:
            steps.advance_for_key(key, protocol_elapsed, final_sample_us)
        if line is SERIAL_RECONNECT:
            button.release(displayed_target)
            cursor_filter.reset()
            cursor.reset()
            rest_detector.reset()
            displayed_target = None
            continue
        parsed = parse_line(line)
        if not parsed:
            continue
        if parsed[0] == "CFG":
            gyro_range = parsed[1].get("gyro_range_dps", gyro_range)
            continue
        if parsed[0] == "BTN":
            button.update(parsed[2], displayed_target)
            continue

        _, t_us, gyro_raw, accel_raw = parsed
        if protocol_last_t_us is not None and 0 < t_us - protocol_last_t_us <= 100_000:
            protocol_elapsed += (t_us - protocol_last_t_us) / 1e6
        protocol_last_t_us = t_us
        if steps is not None:
            steps.update(protocol_elapsed, t_us)
        gyro = AXIS_REMAP @ gyro_raw
        accel = AXIS_REMAP @ accel_raw
        sample_count += 1
        if final_sample_us is not None and 0 < t_us - final_sample_us <= 100_000:
            duration += (t_us - final_sample_us) / 1e6
        final_sample_us = t_us
        sample_peak = float(np.max(np.abs(gyro)))
        peak_gyro = max(peak_gyro, sample_peak)
        if last_t_us is not None and t_us <= last_t_us:
            rate_times.clear()
            gyro_peaks.clear()
            clip_times.clear()
            last_hud_us = -10**18
        gyro_peaks.append((t_us, sample_peak))
        while gyro_peaks and t_us - gyro_peaks[0][0] > GYRO_PEAK_WINDOW_SECONDS * 1e6:
            gyro_peaks.popleft()
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
                hud_interval_us = 2_000_000 if guided else 500_000
                if t_us - last_hud_us >= hud_interval_us:
                    bias_text = (
                        f"BIAS {max(0, BIAS_SECONDS - (t_us - bias_start_us) / 1e6):.1f}s"
                        f"  samples={len(bias_samples)}"
                    )
                    if steps is not None:
                        bias_text = (
                            f"GUIDED {steps.index + 1}/{len(PROTOCOL)}"
                            f" {steps.remaining_text(protocol_elapsed)} | {bias_text}"
                        )
                    print(bias_text, flush=True)
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
            # AHRS owns gravity correction. Preserve the webcam heading while
            # forcing NWU earth-up to remain screen-up so yaw cannot leak into
            # cursor elevation.
            alignment = aligned_screen_transform(
                calibration.rotation,
                mount_correction,
                imufusion.quaternion_to_matrix(ahrs.get_quaternion()),
            )
            buffered.clear()
            continue

        if alignment is None:
            # Desk bias is already known. Use the median of a real convergence
            # window so one handheld acceleration sample cannot tilt the
            # permanent earth-to-screen alignment and couple yaw drift into
            # elevation. The user must hold the calibration pose for this
            # short window.
            if alignment_start_us is None or t_us < alignment_start_us:
                alignment_start_us = t_us
                alignment_accel_samples.clear()
            alignment_accel_samples.append(accel.copy())
            last_t_us = t_us
            if t_us - alignment_start_us < ALIGNMENT_SECONDS * 1e6:
                continue
            representative_accel = np.median(alignment_accel_samples, axis=0)
            ahrs = make_ahrs()
            ahrs.set_sample_period(1.0 / AHRS_SAMPLE_RATE_HZ)
            for _ in range(round(AHRS_STARTUP_SECONDS * AHRS_SAMPLE_RATE_HZ) + 1):
                ahrs.update_no_magnetometer(np.zeros(3), representative_accel)
            alignment = aligned_screen_transform(
                calibration.rotation,
                mount_correction,
                imufusion.quaternion_to_matrix(ahrs.get_quaternion()),
            )
            continue

        dt = (t_us - last_t_us) / 1e6
        last_t_us = t_us
        if dt <= 0 or dt > 0.1:
            rest_detector.reset()
            continue
        ahrs.set_sample_period(dt)
        corrected_gyro = gyro - bias
        ahrs.update_no_magnetometer(corrected_gyro, accel)
        base_rotation = alignment @ imufusion.quaternion_to_matrix(ahrs.get_quaternion())
        base_ray = screen_pointing_ray(base_rotation)
        ray = pointing_reference @ base_ray
        if recenter_requested:
            center_direction = -ray_origin_mm / np.linalg.norm(ray_origin_mm)
            correction = shortest_arc_rotation(ray, center_direction)
            pointing_reference = correction @ pointing_reference
            ray = pointing_reference @ base_ray
            recenter_requested = False
            recenter_count += 1
            cursor_filter.reset()
            cursor.reset()
            print(f"# !!! RECENTERED !!! recenters={recenter_count}", flush=True)
        target, ray, raw_hit_mm = intersect_screen_ray(ray_origin_mm, ray, display)
        if target is not None:
            target = scale_target_about_center(target, display.center_px, sensitivity)
            filtered_target = cursor_filter.filter(target, t_us / 1e6)
            displayed_target = cursor.move(filtered_target, dragging=button.held)
            target_min = np.minimum(target_min, filtered_target)
            target_max = np.maximum(target_max, filtered_target)
        else:
            cursor_filter.reset()
            cursor.reset()
            displayed_target = None

        rest = rest_detector.update(t_us, gyro, accel)
        if rest:
            if rest_start_us is None:
                rest_start_us = t_us
                last_bias_update_us = None
                bias_update_printed = False
            if (
                t_us - rest_start_us >= REST_BIAS_DELAY_SECONDS * 1e6
                and (
                    last_bias_update_us is None
                    or t_us - last_bias_update_us >= REST_BIAS_UPDATE_SECONDS * 1e6
                )
            ):
                rest_mean = rest_detector.gyro_mean
                bias = (1.0 - REST_BIAS_LEAK) * bias + REST_BIAS_LEAK * rest_mean
                last_bias_update_us = t_us
                if not bias_update_printed:
                    print(f"# bias updated {np.round(bias, 5).tolist()}", flush=True)
                    bias_update_printed = True

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
        else:
            rest_start_us = None
            last_bias_update_us = None
            bias_update_printed = False

        hud_interval_us = 2_000_000 if guided else 500_000
        if t_us - last_hud_us >= hud_interval_us:
            sample_rate = max(0, len(rate_times) - 1)
            rolling_peak = max((sample for _, sample in gyro_peaks), default=0.0)
            saturation = " SATURATION!" if rolling_peak > 0.9 * gyro_range else ""
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
            hud_text = (
                f"TRACK {('REST' if rest else 'MOVING'):6s}  rate={sample_rate:3d} Hz"
                f"  drift={drift:+7.2f} deg/min  peak={rolling_peak:7.1f}/{gyro_range:.0f} dps"
                f"{saturation}{clip_warning}  cursor={target_text}{debug_text}"
            )
            if steps is not None:
                hud_text = (
                    f"GUIDED {steps.index + 1}/{len(PROTOCOL)}"
                    f" {steps.remaining_text(protocol_elapsed)} | {hud_text}"
                )
            print(hud_text, flush=True)
            last_hud_us = t_us

    button.release(displayed_target)
    if steps is not None:
        steps.finish(final_sample_us)

    if sample_count:
        span = target_max - target_min
        span_text = (
            f"[{span[0]:.2f}, {span[1]:.2f}]"
            if np.all(np.isfinite(span)) else "no intersections"
        )
        print(
            f"SUMMARY samples={sample_count} duration={duration:.3f}s"
            f" peak_gyro={peak_gyro:.2f}dps clips={clip_count} clicks={button.clicks}"
            f" recenters={recenter_count} drift={drift:+.3f}deg/min"
            f" target_span_px={span_text}"
        )
    elif bias_samples:
        print(
            "SUMMARY replay ended before bias calibration completed"
            f" clicks={button.clicks}"
        )
    else:
        print(f"SUMMARY no IMU samples found clicks={button.clicks}")


def main():
    # Line-buffer stdout so output appears live even when piped through tee.
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--replay", type=Path, metavar="FILE")
    parser.add_argument(
        "--record", type=Path, metavar="FILE",
        help="append every raw serial line during live tracking",
    )
    parser.add_argument(
        "--camera", action="store_true",
        help="run the legacy webcam ArUco calibration diagnostic",
    )
    parser.add_argument("--fake-calib", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--distance-mm", type=float, default=500.0, metavar="MM",
        help="assumed device distance in front of screen center (default: 500)",
    )
    parser.add_argument(
        "--sens", type=float, default=1.0, metavar="FACTOR",
        help="cursor displacement gain about screen center (default: 1.0)",
    )
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--warp", action="store_true", help="warp cursor during replay")
    parser.add_argument("--debug", action="store_true", help="show ray and raw intersection")
    parser.add_argument(
        "--guided", action="store_true",
        help="coach the AIR round 6 protocol with instructions, timers, and markers",
    )
    parser.add_argument(
        "--voice", action="store_true",
        help="speak guided instructions via macOS say (default: silent)",
    )
    parser.add_argument("--fov", type=float, default=HORIZONTAL_FOV_DEG,
                        help="approximate webcam horizontal FOV in degrees")
    parser.add_argument(
        "--mount-roll", type=float, default=None, metavar="DEG",
        help=(
            "fixed calibration roll around the pointing axis "
            "(default: 0; -90 with --camera)"
        ),
    )
    args = parser.parse_args()
    if args.record is not None and args.replay is not None and not args.guided:
        parser.error("--record is only available during live tracking")
    if args.distance_mm <= 0:
        parser.error("--distance-mm must be greater than zero")

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
    if args.replay is None:
        if args.guided:
            desk_instruction = "Place device flat on desk, hands off, press Enter"
            print(f"\nGUIDED PRE-TRACK — DESK BIAS\n{desk_instruction}\n")
            speak(desk_instruction)
            input()
        else:
            input("place device flat on desk, press Enter")
    desk_lines = (
        replay_lines(args.replay)
        if args.replay is not None
        else serial_lines(args.port, args.baud)
    )
    try:
        desk_bias, desk_std = collect_gyro_bias(desk_lines, DESK_BIAS_SECONDS)
    finally:
        close_desk_lines = getattr(desk_lines, "close", None)
        if close_desk_lines is not None:
            close_desk_lines()
    print(
        f"GYRO BIAS dps={np.round(desk_bias, 5).tolist()}"
        f" std_dps={np.round(desk_std, 5).tolist()}"
    )
    if np.any(desk_std > 1.0):
        print("WARNING gyro std > 1 dps: device was moving during desk bias")

    use_camera = args.camera and not args.fake_calib
    calibration = select_calibration(
        use_camera,
        args.fov,
        display,
        args.distance_mm,
    )
    if not use_camera:
        q = matrix_to_quaternion(calibration.rotation)
        print(
            "CALIBRATION assumed"
            f" position_mm={np.round(calibration.position_mm, 1).tolist()}"
            f" quaternion_wxyz={np.round(q, 5).tolist()}"
        )
    print("press r while aiming at screen center to calibrate pointing")
    lines = replay_lines(args.replay) if args.replay else serial_lines(args.port, args.baud)
    track_options = dict(
        warp=args.warp or args.replay is None,
        debug=args.debug,
        initial_bias=desk_bias,
        mount_roll_deg=select_mount_roll(use_camera, args.mount_roll),
        guided=args.guided,
        voice=args.voice and args.replay is None,
        sensitivity=args.sens,
    )
    if args.record is not None and args.guided:
        with args.record.open("a", encoding="utf-8") as recording:
            track(
                recording_lines(lines, recording), calibration, display,
                recording=recording, **track_options,
            )
    elif args.record is not None:
        track(recorded_lines(lines, args.record), calibration, display, **track_options)
    else:
        track(lines, calibration, display, **track_options)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, OSError, serial.SerialException) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
