import numpy as np
import pytest

from robot_control.config import ArmSettings, RigSettings
from robot_control.kinematics.arm import Kinematics
from robot_control.vision.arm_fit import JointCorrection
from robot_control.vision.guide import (TIP_OFFSET_MM, axis_mm, jaw_angle, plan_jaws, plan_point, step_joints,
                                        tip_offset_mm)
from robot_control.vision.rig import robot_to_scene

KIN = Kinematics(ArmSettings.from_config())
RIG = RigSettings(180.0, (0.0, 0.0))
POSE = (-45.0, 110.0)


def moved_mm(joints):
    """How far, and which way, the pose moved, in scene mm."""
    a, b = robot_to_scene([KIN.fk(*POSE)[:2], KIN.fk(*joints)[:2]], RIG)
    return b - a


def test_a_step_is_a_fraction_of_the_error_toward_the_target():
    tip = np.array([0.0, -130.0])
    assert moved_mm(step_joints(tip, tip + (2.0, -1.0), *POSE, KIN, RIG)) == pytest.approx((1.4, -0.7), abs=1e-6)


def test_big_errors_are_capped():
    tip = np.array([0.0, -130.0])
    step = moved_mm(step_joints(tip, tip + (30.0, 40.0), *POSE, KIN, RIG))
    assert step == pytest.approx((3.0, 4.0), abs=1e-6)


def test_the_arm_is_trusted_for_direction_only():
    """Whatever the arm thinks its error is, only the camera's difference sets the step."""
    a = step_joints(np.array([10.0, -100.0]), np.array([11.0, -100.0]), *POSE, KIN, RIG)
    b = step_joints(np.array([-20.0, -160.0]), np.array([-19.0, -160.0]), *POSE, KIN, RIG)
    assert a == pytest.approx(b)


LENS = (-143.4, -144.1, 185.6)


def test_a_planned_pose_puts_the_tip_on_the_target_face_on():
    for target in ((0.0, -130.0), (40.0, -170.0), (-40.0, -130.0), (0.0, -112.0)):
        base, elbow, wrist = plan_point(target, LENS, KIN, RIG)
        axis = robot_to_scene([KIN.fk(base, elbow)[:2]], RIG)[0]
        offset = tip_offset_mm(base, elbow, wrist)
        assert axis + offset == pytest.approx(target, abs=1e-6)
        toward = (np.array(LENS[:2]) - target) / np.linalg.norm(np.array(LENS[:2]) - target)
        assert offset == pytest.approx(TIP_OFFSET_MM * toward, abs=1e-6)


def test_the_crosshair_plans_near_where_it_was_measured_face_on():
    assert plan_point((-0.48, -130.37), LENS, KIN, RIG)[2] == pytest.approx(74.0, abs=5.0)


def test_the_axis_comes_back_from_the_tip():
    assert axis_mm((3.0, -120.0), -45.0, 110.0, 70.0) == pytest.approx(np.array((3.0, -120.0)) - tip_offset_mm(-45.0, 110.0, 70.0))


def test_the_jaws_close_across_the_body_with_the_tip_kept_in_place():
    for body_deg in (0.0, 30.0, 65.0, 90.0, 120.0, 170.0):
        base, elbow, wrist = plan_jaws((17.0, -148.0), body_deg, (-90.0, 120.0), KIN, RIG)
        assert -90.0 <= wrist <= 120.0
        across = (jaw_angle(base, elbow, wrist) - body_deg) % 180.0
        assert across == pytest.approx(90.0, abs=0.1)
        tip = robot_to_scene([KIN.fk(base, elbow)[:2]], RIG)[0] + tip_offset_mm(base, elbow, wrist)
        assert tip == pytest.approx((17.0, -148.0), abs=0.05)


def test_the_jaws_centre_goes_on_the_point_at_any_jaw_angle():
    for body_deg in (0.0, 61.0, 135.0):
        base, elbow, wrist = plan_jaws((17.0, -148.0), body_deg, (-90.0, 120.0), KIN, RIG, at_axis=True)
        assert robot_to_scene([KIN.fk(base, elbow)[:2]], RIG)[0] == pytest.approx((17.0, -148.0), abs=1e-6)
        assert (jaw_angle(base, elbow, wrist) - body_deg) % 180.0 == pytest.approx(90.0, abs=0.1)


def test_plans_undo_the_joint_correction():
    rig, correction = RigSettings(183.2, (0.0005, -0.0007)), JointCorrection(1.044, -0.38)
    base, elbow, wrist = plan_jaws((17.0, -148.0), 61.0, (-90.0, 120.0), KIN, rig, 90.0, correction)
    true = correction.apply([(base, elbow)])[0]
    tip = robot_to_scene([KIN.fk(*true)[:2]], rig)[0] + tip_offset_mm(base, elbow, wrist)
    assert tip == pytest.approx((17.0, -148.0), abs=0.05)
    assert (jaw_angle(base, elbow, wrist) - 61.0) % 180.0 == pytest.approx(90.0, abs=0.1)


def test_no_wrist_angle_in_range_is_refused():
    with pytest.raises(ValueError):
        plan_jaws((17.0, -148.0), 65.0, (-1.0, 1.0), KIN, RIG)


def test_a_target_out_of_reach_is_refused():
    with pytest.raises(ValueError):
        step_joints(np.zeros(2), np.array([0.0, 400.0]), 0.0, 1.0, KIN, RIG, gain=1.0, max_step_mm=1000.0)
