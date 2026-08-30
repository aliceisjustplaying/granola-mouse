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

"""Synthetic gravity and frame checks for wand_proto.py."""

import contextlib
import io
import math
import re
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import imufusion
import numpy as np

import wand_proto


RATE_HZ = 200
STATIONARY_ACCEL = np.array([0.06, 0.08, -1.02])
DISPLAY = wand_proto.Display(0, 0, 0, 1920, 1080, 344, 223)
# Body +x (the ray) points into the screen and body -z points screen-up.
SCREEN_FROM_FLAT_BODY = np.array(
    [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
)
CALIBRATION = wand_proto.Calibration(
    SCREEN_FROM_FLAT_BODY, np.array([0.0, 0.0, -500.0])
)


def rotation_x(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]]
    )


def rotation_z(angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
    )


def rotation_vector(vector):
    angle = np.linalg.norm(vector)
    if angle == 0.0:
        return np.eye(3)
    axis = vector / angle
    cross_matrix = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return (
        np.eye(3)
        + math.sin(angle) * cross_matrix
        + (1.0 - math.cos(angle)) * cross_matrix @ cross_matrix
    )


def offset_pointing_rotation(azimuth_deg, elevation_deg):
    centered = SCREEN_FROM_FLAT_BODY @ wand_proto.pointing_axis_roll(
        math.radians(wand_proto.CAMERA_MOUNT_ROLL_DEG)
    )
    return (
        wand_proto.yaw_rotation(math.radians(azimuth_deg))
        @ rotation_x(math.radians(-elevation_deg))
        @ centered
    )


def replay(gyro, first_accel=None, duration=40.0):
    first_accel = STATIONARY_ACCEL if first_accel is None else first_accel
    for index in range(round(RATE_HZ * duration) + 1):
        accel = first_accel if index == 0 else STATIONARY_ACCEL
        yield (
            f"IMU,{round(index * 1e6 / RATE_HZ)},"
            f"{gyro[0]},{gyro[1]},{gyro[2]},"
            f"{accel[0]},{accel[1]},{accel[2]}\n"
        )


def elevation_curve(lines):
    output = io.StringIO()
    with contextlib.redirect_stdout(output):
        wand_proto.track(
            lines,
            CALIBRATION,
            DISPLAY,
            warp=False,
            debug=True,
            initial_bias=np.zeros(3),
            mount_roll_deg=0.0,
        )
    return [
        float(match.group(1))
        for match in re.finditer(r"ray_az/el=[^/]+/([+-][0-9.]+)deg", output.getvalue())
    ]


class WiredButtonTests(unittest.TestCase):
    def test_btn_replay_parses_pairs_and_counts_one_click(self):
        self.assertEqual(wand_proto.parse_line("BTN,123,1\n"), ("BTN", 123, True))
        self.assertEqual(wand_proto.parse_line("BTN,456,0\n"), ("BTN", 456, False))
        self.assertIsNone(wand_proto.parse_line("BTN,456,2\n"))
        lines = [
            "BTN,100,1\n",
            "BTN,110,1\n",
            "BTN,200,0\n",
            "BTN,210,0\n",
            "IMU,300,0,0,0,0,0,-1\n",
        ]
        posted = []
        output = io.StringIO()
        with (
            mock.patch.object(
                wand_proto,
                "current_cursor_position",
                return_value=np.array([40.0, 50.0]),
            ),
            mock.patch.object(
                wand_proto,
                "post_left_mouse_event",
                side_effect=lambda event_type, position: posted.append(event_type),
            ),
            contextlib.redirect_stdout(output),
        ):
            wand_proto.track(
                lines,
                CALIBRATION,
                DISPLAY,
                warp=True,
                debug=False,
                initial_bias=np.zeros(3),
                mount_roll_deg=0.0,
            )

        self.assertEqual(
            posted,
            [wand_proto.kCGEventLeftMouseDown, wand_proto.kCGEventLeftMouseUp],
        )
        self.assertIn("clicks=1", output.getvalue())

    def test_cursor_moves_use_drag_events_while_button_is_held(self):
        cursor = wand_proto.FractionalCursor(warp=True)
        with (
            mock.patch.object(wand_proto, "post_left_mouse_event") as post_event,
            mock.patch.object(wand_proto, "CGWarpMouseCursorPosition") as warp_cursor,
        ):
            cursor.move(np.array([100.0, 200.0]), dragging=True)

        post_event.assert_called_once()
        self.assertEqual(
            post_event.call_args.args[0], wand_proto.kCGEventLeftMouseDragged
        )
        warp_cursor.assert_not_called()

    def test_replay_without_warp_never_posts_mouse_events(self):
        lines = ["BTN,100,1\n", "BTN,200,0\n"]
        output = io.StringIO()
        with (
            mock.patch.object(wand_proto, "post_left_mouse_event") as post_event,
            contextlib.redirect_stdout(output),
        ):
            wand_proto.track(
                lines,
                CALIBRATION,
                DISPLAY,
                warp=False,
                debug=False,
                initial_bias=np.zeros(3),
                mount_roll_deg=0.0,
            )

        post_event.assert_not_called()
        self.assertIn("clicks=1", output.getvalue())


class CalibrationAndSensitivityTests(unittest.TestCase):
    def test_default_calibration_uses_assumed_pose_without_camera_api(self):
        with mock.patch.object(
            wand_proto.cv2, "VideoCapture", side_effect=AssertionError("camera opened")
        ):
            calibration = wand_proto.select_calibration(
                False, wand_proto.HORIZONTAL_FOV_DEG, DISPLAY, 725.0
            )

        np.testing.assert_array_equal(calibration.rotation, np.eye(3))
        np.testing.assert_array_equal(
            calibration.position_mm, np.array([0.0, 0.0, -725.0])
        )

    def test_mount_roll_defaults_follow_calibration_flow_and_allow_override(self):
        self.assertEqual(wand_proto.select_mount_roll(False, None), 0.0)
        self.assertEqual(wand_proto.select_mount_roll(True, None), -90.0)
        self.assertEqual(wand_proto.select_mount_roll(False, 17.5), 17.5)
        self.assertEqual(wand_proto.select_mount_roll(True, 17.5), 17.5)

    def test_sensitivity_scales_cursor_displacement_about_center_exactly(self):
        raw_target = np.array([1170.25, 413.5])
        unit = wand_proto.scale_target_about_center(
            raw_target, DISPLAY.center_px, 1.0
        )
        doubled = wand_proto.scale_target_about_center(
            raw_target, DISPLAY.center_px, 2.0
        )

        np.testing.assert_array_equal(unit, raw_target)
        np.testing.assert_array_equal(
            doubled - DISPLAY.center_px, 2.0 * (unit - DISPLAY.center_px)
        )


class GuidedStepEngineTests(unittest.TestCase):
    def test_timed_step_advances_and_emits_end_then_start_markers(self):
        markers = []
        announcements = []
        engine = wand_proto.StepEngine(
            [("FIRST", "First motion", 2), ("SECOND", "Second motion", None)],
            lambda step, name, event, t_us: markers.append(
                (step, name, event, t_us)
            ),
            speaker=lambda _instruction: None,
            announce=announcements.append,
        )

        engine.start(10.0)
        engine.update(11.9, 1900)
        self.assertEqual(markers, [(1, "FIRST", "START", None)])

        engine.update(12.0, 2000)
        self.assertEqual(
            markers,
            [
                (1, "FIRST", "START", None),
                (1, "FIRST", "END", 2000),
                (2, "SECOND", "START", 2000),
            ],
        )
        self.assertIn("GUIDED STEP 2/2", announcements[-1])


class RecenterTests(unittest.TestCase):
    def test_full_recenter_maps_azimuth_and_elevation_offset_to_center(self):
        rotation = offset_pointing_rotation(20.0, 25.0)
        ray_origin = CALIBRATION.position_mm.copy()
        ray_origin[1] = 0.0
        center_direction = -ray_origin / np.linalg.norm(ray_origin)

        correction = wand_proto.shortest_arc_rotation(
            wand_proto.screen_pointing_ray(rotation), center_direction
        )
        centered_rotation = correction @ rotation
        target, ray, hit = wand_proto.intersect_screen(
            ray_origin, centered_rotation, DISPLAY
        )

        np.testing.assert_allclose(ray, center_direction, atol=1e-12)
        np.testing.assert_allclose(hit, np.zeros(3), atol=1e-12)
        np.testing.assert_allclose(target, DISPLAY.center_px, atol=1e-12)

    def test_repeated_recenter_composition_remains_exact(self):
        ray_origin = CALIBRATION.position_mm.copy()
        ray_origin[1] = 0.0
        center_direction = -ray_origin / np.linalg.norm(ray_origin)
        reference = np.eye(3)

        for azimuth, elevation in [(20, 25), (-35, 12), (8, -30), (42, 18)] * 10:
            base_rotation = offset_pointing_rotation(azimuth, elevation)
            current_rotation = reference @ base_rotation
            correction = wand_proto.shortest_arc_rotation(
                wand_proto.screen_pointing_ray(current_rotation), center_direction
            )
            reference = correction @ reference
            centered_rotation = reference @ base_rotation
            np.testing.assert_allclose(
                wand_proto.screen_pointing_ray(centered_rotation),
                center_direction,
                atol=2e-12,
            )

    def test_default_flow_physical_pitch_and_yaw_are_exact_after_recenter(self):
        calibration = wand_proto.assumed_calibration(500.0)
        # Body gravity is -z while NWU earth-up is +z.
        flat_body_to_earth = rotation_x(math.pi)
        mount_correction = wand_proto.pointing_axis_roll(
            math.radians(wand_proto.select_mount_roll(False, None))
        )
        alignment = wand_proto.aligned_screen_transform(
            calibration.rotation, mount_correction, flat_body_to_earth
        )
        centered = alignment @ flat_body_to_earth
        center_direction = np.array([0.0, 0.0, 1.0])
        recenter = wand_proto.shortest_arc_rotation(
            wand_proto.screen_pointing_ray(centered), center_direction
        )
        np.testing.assert_allclose(
            recenter @ wand_proto.screen_pointing_ray(centered),
            center_direction,
            atol=1e-12,
        )

        motion_deg = 10.0
        pitched = alignment @ (
            flat_body_to_earth @ wand_proto.yaw_rotation(math.radians(motion_deg))
        )
        yawed_left = alignment @ (
            flat_body_to_earth @ rotation_z(math.radians(motion_deg))
        )
        pitch_ray = recenter @ wand_proto.screen_pointing_ray(pitched)
        yaw_ray = recenter @ wand_proto.screen_pointing_ray(yawed_left)
        pitch_azimuth_deg = math.degrees(math.atan2(pitch_ray[0], pitch_ray[2]))
        pitch_elevation_deg = math.degrees(
            math.atan2(pitch_ray[1], math.hypot(pitch_ray[0], pitch_ray[2]))
        )
        yaw_azimuth_deg = math.degrees(math.atan2(yaw_ray[0], yaw_ray[2]))
        yaw_elevation_deg = math.degrees(
            math.atan2(yaw_ray[1], math.hypot(yaw_ray[0], yaw_ray[2]))
        )

        self.assertAlmostEqual(pitch_elevation_deg, motion_deg, places=12)
        self.assertAlmostEqual(pitch_azimuth_deg, 0.0, places=12)
        self.assertAlmostEqual(yaw_azimuth_deg, -motion_deg, places=12)
        self.assertAlmostEqual(yaw_elevation_deg, 0.0, places=12)

    def test_minus_90_mount_roll_has_expected_motion_signs_after_recenter(self):
        self.assertEqual(wand_proto.CAMERA_MOUNT_ROLL_DEG, -90.0)
        rotation = offset_pointing_rotation(20.0, 25.0)
        center_direction = np.array([0.0, 0.0, 1.0])
        centered = (
            wand_proto.shortest_arc_rotation(
                wand_proto.screen_pointing_ray(rotation), center_direction
            )
            @ rotation
        )
        motion_deg = 5.0

        # Negative screen-y angular velocity moves the ray left. Negative
        # screen-x angular velocity pitches it up. Convert each physical
        # velocity into raw body-frame gyro.
        yaw_body_gyro = centered.T @ np.array([0.0, -motion_deg, 0.0])
        pitch_body_gyro = centered.T @ np.array([-motion_deg, 0.0, 0.0])
        yaw_rotation = centered @ rotation_vector(np.radians(yaw_body_gyro))
        pitch_rotation = centered @ rotation_vector(np.radians(pitch_body_gyro))
        yaw_target, _, _ = wand_proto.intersect_screen(
            CALIBRATION.position_mm, yaw_rotation, DISPLAY
        )
        pitch_target, _, _ = wand_proto.intersect_screen(
            CALIBRATION.position_mm, pitch_rotation, DISPLAY
        )

        self.assertLess(yaw_target[0] - DISPLAY.center_px[0], 0.0)
        self.assertAlmostEqual(yaw_target[1], DISPLAY.center_px[1], places=9)
        self.assertAlmostEqual(pitch_target[0], DISPLAY.center_px[0], places=9)
        self.assertLess(pitch_target[1] - DISPLAY.center_px[1], 0.0)

    def test_positive_body_z_rotation_moves_cursor_right(self):
        # Hardware-determined (live test 2026-08-30): with the pixel-stage
        # horizontal mirror, positive body +z rotation moves the cursor RIGHT;
        # the user's physical-left is negative body z on this device.
        centered = SCREEN_FROM_FLAT_BODY.copy()
        physical_left = centered.copy()
        for _ in range(10):
            physical_left = physical_left @ rotation_z(math.radians(0.5))

        target, _, _ = wand_proto.intersect_screen(
            CALIBRATION.position_mm, physical_left, DISPLAY
        )

        self.assertGreater(target[0] - DISPLAY.center_px[0], 0.0)
        self.assertAlmostEqual(target[1], DISPLAY.center_px[1], places=9)


class RestDetectorTests(unittest.TestCase):
    def test_stale_bias_does_not_prevent_rest_detection_or_window_mean(self):
        detector = wand_proto.RollingRestDetector()
        stale_raw_mean = np.array([5.5, -3.25, 2.0])
        rest = False
        for index in range(RATE_HZ):
            phase = 2.0 * math.pi * index / 17.0
            gyro = stale_raw_mean + 0.1 * np.array(
                [math.sin(phase), math.cos(phase), math.sin(2.0 * phase)]
            )
            accel = STATIONARY_ACCEL + 0.001 * np.array(
                [math.cos(phase), math.sin(phase), math.cos(2.0 * phase)]
            )
            rest = detector.update(round(index * 1e6 / RATE_HZ), gyro, accel)

        self.assertTrue(rest)
        np.testing.assert_allclose(detector.gyro_mean, stale_raw_mean, atol=0.01)
        self.assertLess(
            detector.gyro_deviation_dps, wand_proto.REST_GYRO_STD_DPS
        )
        self.assertLess(
            detector.accel_deviation_g, wand_proto.REST_ACCEL_STD_G
        )

    def test_raw_gyro_variance_marks_motion_independent_of_bias(self):
        detector = wand_proto.RollingRestDetector()
        rest = False
        for index in range(RATE_HZ):
            gyro = np.array([5.5, -3.25, 2.0 + 4.0 * (-1) ** index])
            rest = detector.update(
                round(index * 1e6 / RATE_HZ), gyro, STATIONARY_ACCEL
            )

        self.assertFalse(rest)
        self.assertGreater(
            detector.gyro_deviation_dps, wand_proto.REST_GYRO_STD_DPS
        )

    def test_accel_variance_marks_motion_when_gyro_is_quiet(self):
        detector = wand_proto.RollingRestDetector()
        rest = False
        for index in range(RATE_HZ):
            accel = STATIONARY_ACCEL + np.array(
                [0.05 * (-1) ** index, 0.0, 0.0]
            )
            rest = detector.update(
                round(index * 1e6 / RATE_HZ), np.array([5.5, -3.25, 2.0]), accel
            )

        self.assertFalse(rest)
        self.assertGreater(
            detector.accel_deviation_g, wand_proto.REST_ACCEL_STD_G
        )

    def test_tracking_relearns_a_stale_bias_from_rest_window(self):
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            wand_proto.track(
                replay(np.array([5.5, -3.25, 2.0]), duration=4.0),
                CALIBRATION,
                DISPLAY,
                warp=False,
                debug=False,
                initial_bias=np.zeros(3),
                mount_roll_deg=0.0,
            )

        self.assertIn("# bias updated", output.getvalue())


class GravityCorrectionTests(unittest.TestCase):
    def test_pitch_bias_settles_instead_of_integrating(self):
        elevation = elevation_curve(replay(np.array([1.0, 0.0, 0.0])))

        self.assertLess(abs(elevation[-1]), 5.0)
        self.assertLess(np.ptp(elevation[-20:]), 0.5)

    def test_transient_first_sample_does_not_turn_yaw_drift_into_elevation_drift(self):
        elevation = elevation_curve(
            replay(
                np.array([0.0, 0.0, 1.0]),
                first_accel=np.array([0.8, 0.08, -0.6]),
            )
        )

        self.assertLess(np.ptp(elevation[-20:]), 1.0)
        self.assertLess(abs(elevation[-1]), 5.0)

    def test_nwu_stationary_acceleration_maps_to_earth_up(self):
        ahrs = wand_proto.make_ahrs()
        for _ in range(4 * RATE_HZ):
            ahrs.update_no_magnetometer(np.zeros(3), STATIONARY_ACCEL)

        rotation = imufusion.quaternion_to_matrix(ahrs.get_quaternion())
        measured_up = STATIONARY_ACCEL / np.linalg.norm(STATIONARY_ACCEL)
        np.testing.assert_allclose(rotation @ measured_up, [0.0, 0.0, 1.0], atol=1e-5)

    def test_alignment_composition_matches_known_rotation_sequence(self):
        body_to_earth_at_calibration = rotation_z(0.4) @ rotation_x(math.pi)
        screen_from_earth = rotation_x(-math.pi / 2) @ rotation_z(-0.2)
        mount_correction = wand_proto.pointing_axis_roll(0.3)
        calibration = (
            screen_from_earth
            @ body_to_earth_at_calibration
            @ mount_correction.T
        )
        alignment = wand_proto.aligned_screen_transform(
            calibration, mount_correction, body_to_earth_at_calibration
        )

        body_to_earth_later = rotation_z(-0.7) @ rotation_x(2.8)
        actual = alignment @ body_to_earth_later
        expected = screen_from_earth @ body_to_earth_later
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        actual_ray = wand_proto.screen_pointing_ray(actual)
        expected_elevation = math.atan2(
            expected[1, 0], math.hypot(expected[0, 0], expected[2, 0])
        )
        actual_elevation = math.atan2(
            actual_ray[1], math.hypot(actual_ray[0], actual_ray[2])
        )
        self.assertAlmostEqual(actual_elevation, expected_elevation, places=12)

    def test_recording_appends_imu_cfg_and_unknown_lines_verbatim(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "track.csv"
            path.write_text("existing\n", encoding="utf-8")
            lines = ["IMU,1,2,3,4,5,6,7\n", "CFG,odr_hz=200\n", "# boot noise\n"]

            self.assertEqual(list(wand_proto.recorded_lines(iter(lines), path)), lines)
            self.assertEqual(
                path.read_text(encoding="utf-8"), "existing\n" + "".join(lines)
            )


if __name__ == "__main__":
    unittest.main()
