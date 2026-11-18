import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from camera_calibrate import face_on_error  # noqa: E402
from camera_guide import median_direction  # noqa: E402
from robot_control.vision.guide import WRIST_SCALE, jaw_angle  # noqa: E402

LENS = (-150.0, -130.0, 183.6)


def test_square_to_the_line_of_sight_is_face_on_either_way_round():
    # the camera due -x of the tip: face-on jaws close along y
    assert face_on_error(90.0, (0.0, -130.0), LENS) == pytest.approx(0.0)
    assert face_on_error(93.0, (0.0, -130.0), LENS) == pytest.approx(3.0)
    assert face_on_error(178.0 - 90.0, (0.0, -130.0), LENS) == pytest.approx(-2.0)


def test_turning_the_wrist_by_the_error_over_the_scale_makes_it_face_on():
    base, elbow, wrist = -48.0, 112.0, -10.0
    error = face_on_error(jaw_angle(base, elbow, wrist) % 180.0, (0.0, -130.0), LENS)
    fixed = wrist + error / WRIST_SCALE
    assert face_on_error(jaw_angle(base, elbow, fixed) % 180.0, (0.0, -130.0), LENS) == pytest.approx(0.0, abs=1e-9)


def test_a_lines_median_direction_wraps_at_180():
    assert median_direction([179.0, 1.0, 0.5]) == pytest.approx(0.5)
    assert median_direction([90.0, 91.0, 92.0]) == pytest.approx(91.0)
    assert np.isclose(median_direction([0.2, 179.6, 179.9]), 179.9)
