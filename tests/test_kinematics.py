import math

import pytest

from robot_control.config import ArmSettings
from robot_control.kinematics import Kinematics


@pytest.fixture
def kin():
    return Kinematics(ArmSettings.from_config())


def test_fk(kin):
    assert kin.fk(0, 90, 0) == pytest.approx((0.100, 0.125, 90.0))


def test_ik(kin):
    r = kin.ik(0.100, 0.125, 90.0, 0.12)
    assert r.ok
    assert (r.joints.j0, r.joints.j1, r.joints.j2) == pytest.approx((0.0, 90.0, 0.0), abs=1e-6)


def test_fk_of_ik_round_trips(kin):
    for x in [i / 100 for i in range(-20, 21, 2)]:
        for y in [i / 100 for i in range(-5, 23, 2)]:
            for angle in (-150.0, 0.0, 75.0):
                r = kin.ik(x, y, angle, 0.12)
                if r.ok:
                    fx, fy, fa = kin.fk(r.joints.j0, r.joints.j1, r.joints.j2)
                    assert (fx, fy) == pytest.approx((x, y), abs=1e-9)
                    assert (fa - angle) % 360 == pytest.approx(0, abs=1e-6) or (fa - angle) % 360 == pytest.approx(360, abs=1e-6)


def test_plate_centre_matches_the_cad_pose(kin):
    # arm_and_rig_scene: posed over the plate centre (scene (0, -130) mm) at base -46.12, elbow 110.42
    r = kin.ik(0.0, 0.130, 0.0, 0.12)
    assert (r.joints.j0, r.joints.j1) == pytest.approx((-46.12, 110.42), abs=0.01)


@pytest.mark.parametrize("x, y, words", [
    (0.30, 0.00, "out of reach"),
    (0.05, 0.05, "elbow"),       # theta2 about 146 deg > 120
    (0.00, 0.00, "too close"),
    (0.00, -0.20, "base"),       # behind the arm
])
def test_refusals(kin, x, y, words):
    r = kin.ik(x, y, 0.0, 0.12)
    assert not r.ok and words in r.reason


def test_slide_out_of_range_is_refused(kin):
    r = kin.ik(0.0, 0.2, 0.0, 0.05)
    assert not r.ok and "slide" in r.reason


def test_wrist_is_wrapped_not_refused(kin):
    # theta1 + theta2 = -30: tool angle 170 would need theta3 = 200, so the same angle as -190 is sent (theta3 -160)
    x, y, _ = kin.fk(-60.0, 30.0)
    r = kin.ik(x, y, 170.0, 0.12)
    assert r.ok
    assert r.joints.j2 == pytest.approx(-160.0, abs=1e-6)
    assert r.tool_angle == pytest.approx(170.0 - 360.0)


def test_symmetric_tool_takes_the_smaller_wrist_turn(kin):
    x, y, _ = kin.fk(0.0, 90.0)
    r = kin.ik(x, y, 260.0, 0.12, symmetric=True)
    assert r.joints.j2 == pytest.approx(-10.0, abs=1e-6)       # 260 - 180 = 80 = 90 + (-10)


def test_reach(kin):
    r_min, r_max = kin.reach()
    assert r_min == pytest.approx(0.1146, abs=1e-4)
    assert r_max == pytest.approx(0.225)
    assert not kin.ik(0.0, r_min - 0.001, 0.0, 0.12).ok
    assert kin.ik(0.0, r_min + 0.001, 0.0, 0.12).ok
    assert math.isclose(r_max, 0.225)
