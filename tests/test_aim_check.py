"""The aim check: a steady shift between the arm and the plate, measured and taken out."""

import json

import numpy as np
import pytest

from robot_control.config import ArmSettings, RigSettings
from robot_control.kinematics.arm import Kinematics
from robot_control.task import aim_check
from robot_control.task.aim_check import CHECK_POINTS, AimCheck, describe, to_scene
from robot_control.vision.arm_fit import JointCorrection, errors_mm, fit_shift, load_calibration
from robot_control.vision.guide import pots_for

KIN = Kinematics(ArmSettings.from_config())
SAVED = RigSettings(-177.3, (0.0014, -0.0007))
CORRECTION = JointCorrection(1.0465, -0.457)
SHIFT_SCENE_MM = np.array([2.67, -0.86])        # what the 2026-11-09 lab check found


def carried(shift_scene_mm):
    """The rig after carrying: the arm's base moved shift_scene_mm against the plate. Seen from the arm,
    every scene point is then that much the other way."""
    moved = to_scene_inverse(-np.asarray(shift_scene_mm), SAVED)
    return RigSettings(SAVED.rotation_deg, tuple(t + m / 1000.0 for t, m in zip(SAVED.translation_m, moved)))


def to_scene_inverse(vector_scene_mm, rig):
    a = np.radians(rig.rotation_deg)
    c, s = np.cos(a), np.sin(a)
    return np.asarray(vector_scene_mm, float) @ np.array([[c, s], [-s, c]])


def the_check(true_rig, points=CHECK_POINTS, noise_mm=0.0, seed=0):
    """(axes seen, pot joints) for the arm aiming at points by SAVED while the rig is really true_rig."""
    rng = np.random.default_rng(seed)
    joints = np.array([pots_for(p, KIN, SAVED, CORRECTION) for p in points])           # aimed by the saved one
    # Where those joints really put the wrist axis, in scene mm: the true rig's inverse of the arm's position.
    from robot_control.vision.arm_fit import arm_xy
    from robot_control.vision.rig import robot_to_scene
    seen = robot_to_scene(arm_xy(joints, KIN, CORRECTION), true_rig)
    return seen + rng.normal(0, noise_mm, seen.shape), joints


def test_a_carried_rig_shows_as_one_steady_shift_and_fit_shift_takes_it_out():
    axes, joints = the_check(carried(SHIFT_SCENE_MM))
    before = to_scene(errors_mm(axes, joints, KIN, SAVED, CORRECTION), SAVED)
    assert before.mean(0) == pytest.approx(SHIFT_SCENE_MM, abs=0.05)       # the arm lands that far off
    fit = fit_shift(axes, joints, KIN, SAVED, CORRECTION)
    assert fit.rms_mm == pytest.approx(0.0, abs=1e-6)
    assert fit.rig.rotation_deg == SAVED.rotation_deg and fit.joints == CORRECTION
    # Aiming by the shifted calibration puts the arm where it means to be.
    target = np.array([16.9, -120.7])                                        # the 680R that was missed
    axes2, joints2 = the_check(carried(SHIFT_SCENE_MM), points=[target])
    replanned = np.array([pots_for(target, KIN, fit.rig, CORRECTION)])
    from robot_control.vision.arm_fit import arm_xy
    from robot_control.vision.rig import robot_to_scene
    landed = robot_to_scene(arm_xy(replanned, KIN, CORRECTION), carried(SHIFT_SCENE_MM))[0]
    assert np.hypot(*(landed - target)) < 0.05


def test_noise_is_left_once_the_shift_is_out():
    axes, joints = the_check(carried(SHIFT_SCENE_MM), noise_mm=1.0, seed=3)
    clean, _ = the_check(carried(SHIFT_SCENE_MM))
    noise = axes - clean
    fit = fit_shift(axes, joints, KIN, SAVED, CORRECTION)
    left = np.sqrt(((noise - noise.mean(0)) ** 2).sum(1).mean())          # the noise, its own mean taken out too
    assert fit.rms_mm == pytest.approx(left, abs=0.05)
    assert fit.rms_mm < np.sqrt((errors_mm(axes, joints, KIN, SAVED, CORRECTION) ** 2).sum(1).mean())


class Nav:
    def __init__(self):
        self.kinematics, self.rig, self.correction = KIN, SAVED, CORRECTION


def judge(tmp_path, shift, noise=0.0, found=None):
    calibration = tmp_path / "camera_calibration.json"
    calibration.write_text(json.dumps({"rotation_deg": SAVED.rotation_deg, "translation_m": list(SAVED.translation_m),
                                       "base_scale": CORRECTION.base_scale, "elbow_offset_deg": CORRECTION.elbow_offset_deg,
                                       "date": "2026-11-06", "pairs": "x.csv", "rms_mm": 1.26, "worst_mm": 2.12,
                                       "left_out_rms_mm": 1.54}))
    axes, joints = the_check(carried(shift), noise_mm=noise)
    n = len(axes) if found is None else found
    nav = Nav()
    check = AimCheck(None, None, path=calibration, logs=tmp_path)
    tips = [tuple(a) for a in axes[:n]] + [None] * (len(axes) - n)
    report = check.judge(CHECK_POINTS, tips, axes[:n], joints[:n], nav, tmp_path / "pairs.csv", say=lambda t: None)
    return report, nav, calibration


def test_a_shift_worth_taking_out_is_saved_and_aimed_by_at_once(tmp_path):
    report, nav, calibration = judge(tmp_path, SHIFT_SCENE_MM)
    assert report.saved and "taken out and saved" in report.verdict
    assert report.shift_mm == pytest.approx(SHIFT_SCENE_MM, abs=0.05)
    assert nav.rig != SAVED                                            # picks aim by it from now on
    rig, correction = load_calibration(calibration)
    assert rig == nav.rig and correction == CORRECTION
    saved = json.loads(calibration.read_text())
    assert saved["shifted"]["fitted_on"] == "2026-11-06" and saved["rms_mm"] == 1.26
    assert list(tmp_path.glob("camera_calibration_replaced_*.json"))  # the old one kept
    assert describe(calibration).startswith("aim: checked")


def test_a_small_shift_is_left_alone(tmp_path):
    report, nav, calibration = judge(tmp_path, (0.2, 0.1))
    assert not report.saved and nav.rig == SAVED and "aim good" in report.verdict
    assert describe(calibration).startswith("aim: fitted 2026-11-06, not checked")


def test_more_than_a_shift_says_to_recalibrate(tmp_path, monkeypatch):
    monkeypatch.setattr(aim_check, "LEFT_OK_MM", 0.5)
    report, _, _ = judge(tmp_path, SHIFT_SCENE_MM, noise=1.5)
    assert "redo the full calibration" in report.verdict


def test_too_few_tips_give_no_verdict(tmp_path):
    report, nav, _ = judge(tmp_path, SHIFT_SCENE_MM, found=2)
    assert not report.saved and nav.rig == SAVED and "no verdict" in report.verdict


# The arm's part: the tip down at a point, as camera_calibrate's check puts it

class Bench:
    SWING_S, OVER_PLATE_SLIDE = 3.0, 63.0

    def __init__(self):
        self.done = []

    def approach(self, scara, base, elbow, wrist, seconds):
        self.done.append(("approach", round(base, 1), round(elbow, 1)))

    def land(self, scara, targets, wrist, slide):
        self.done.append(("land", slide))

    def read_joints(self, scara, n=30):
        return -46.0, 110.5

    def load_state(self):
        return {"wrist": 70.0, "slide": 63.0}


class Jaws:
    def __init__(self, done):
        self.done = done

    def close(self):
        self.done.append("close")


class Navigator:
    def __init__(self, done):
        self.kinematics, self.rig, self.correction = KIN, RigSettings.from_config(), JointCorrection()
        self.gripper, self.scara, self.parked, self.done = Jaws(done), None, True, done

    def lift(self):
        self.done.append("lift")

    def drive(self, joint, target, seconds):
        self.done.append(("drive", joint, target))


def test_touching_a_point_closes_the_jaws_comes_over_it_lowers_and_lands():
    from robot_control.motion.arm import Arm
    bench = Bench()
    nav = Navigator(bench.done)
    got = Arm(nav, {}, bench).touch((0.0, -130.0), (-145.0, -145.0, 183.0))
    assert [d if isinstance(d, str) else d[0] for d in bench.done] == ["close", "lift", "approach", "drive", "land"]
    assert ("drive", "slide", 63.0) in bench.done and not nav.parked
    assert got == (-46.0, 110.5, 70.0)
