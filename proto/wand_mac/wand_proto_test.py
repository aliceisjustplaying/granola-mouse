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

import imufusion
import numpy as np

import wand_proto


RATE_HZ = 200
STATIONARY_ACCEL = np.array([0.06, 0.08, -1.02])
DISPLAY = wand_proto.Display(0, 0, 0, 1920, 1080, 344, 223)
# Body +y (the ray) points into the screen and body -z points screen-up.
SCREEN_FROM_FLAT_BODY = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
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
        actual_ray = actual[:, 1]
        expected_elevation = math.atan2(
            expected[1, 1], math.hypot(expected[0, 1], expected[2, 1])
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
