import numpy as np
import pytest

from robot_control.config import ArmSettings, RigSettings
from robot_control.kinematics.arm import Kinematics
from robot_control.vision.arm_fit import (ArmFit, JointCorrection, arm_xy, fit_circle, fit_joints, fit_rigid,
                                          load_calibration, save_calibration)
from robot_control.vision.rig import robot_to_scene

KIN = Kinematics(ArmSettings.from_config())
JOINTS = np.array([[-46.0, 70.7], [-16.9, 70.6], [-44.8, 110.8], [-79.5, 131.8], [-20.9, 131.7], [-60.0, 95.0], [-30.0, 100.0]])


def pairs(rig, correction=JointCorrection()):
    """Tips as a perfect camera would see them, with the arm's real angles differing from the pots'."""
    return robot_to_scene(arm_xy(JOINTS, KIN, correction), rig)


def test_rigid_fit_finds_the_transform():
    rig = RigSettings(178.8, (0.0034, -0.0012))
    fit = fit_rigid(pairs(rig), JOINTS, KIN)
    assert fit.rig.rotation_deg % 360 == pytest.approx(178.8)
    assert fit.rig.translation_m == pytest.approx(rig.translation_m)
    assert fit.rms_mm == pytest.approx(0.0, abs=1e-6)


def test_rigid_fit_leaves_a_joint_error_in_the_residuals():
    fit = fit_rigid(pairs(RigSettings(180.0, (0.0, 0.0)), JointCorrection(1.03, 1.5)), JOINTS, KIN)
    assert fit.rms_mm > 0.5


def test_joint_fit_finds_the_correction():
    truth = JointCorrection(1.03, 1.5)
    rig = RigSettings(178.8, (0.0034, -0.0012))
    fit = fit_joints(pairs(rig, truth), JOINTS, KIN)
    assert fit.joints.base_scale == pytest.approx(1.03, abs=1e-5)
    assert fit.joints.elbow_offset_deg == pytest.approx(1.5, abs=1e-4)
    assert fit.rig.rotation_deg % 360 == pytest.approx(178.8, abs=1e-3)
    assert fit.rms_mm == pytest.approx(0.0, abs=1e-4)


def test_a_base_offset_is_taken_up_by_the_rotation():
    rig = RigSettings(180.0, (0.0, 0.0))
    turned = arm_xy(JOINTS + (2.0, 0.0), KIN)
    fit = fit_rigid(robot_to_scene(turned, rig), JOINTS, KIN)
    assert fit.rms_mm == pytest.approx(0.0, abs=1e-6)
    assert fit.rig.rotation_deg % 360 == pytest.approx(182.0)


def test_circle_from_part_of_an_arc():
    a = np.radians(np.arange(100.0, 170.0, 10.0))
    points = np.column_stack([25.0 + 8.0 * np.cos(a), -131.0 + 8.0 * np.sin(a)])
    centre, radius, residuals = fit_circle(points)
    assert centre == pytest.approx((25.0, -131.0))
    assert radius == pytest.approx(8.0)
    assert residuals == pytest.approx(0.0, abs=1e-9)
    with pytest.raises(ValueError):
        fit_circle(points[:2])


def test_a_saved_calibration_comes_back(tmp_path):
    rig, joints = RigSettings(183.2, (0.0005, -0.0007)), JointCorrection(1.044, -0.38)
    save_calibration(ArmFit(rig, joints, np.zeros((1, 2))), {"rms_mm": 1.44}, tmp_path / "c.json")
    assert load_calibration(tmp_path / "c.json") == (rig, joints)


def test_too_few_pairs():
    with pytest.raises(ValueError):
        fit_rigid(np.zeros((1, 2)), np.zeros((1, 2)), KIN)
    with pytest.raises(ValueError):
        fit_joints(np.zeros((2, 2)), np.zeros((2, 2)), KIN)
