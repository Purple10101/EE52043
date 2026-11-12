import sys
from pathlib import Path

import pytest

pytest.importorskip("serial")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import slide_wrist  # noqa: E402
from slide_wrist import (HOVER_SLIDE, LAND_MAX_DEG, LAND_TOLERANCE_DEG, PARK, PARK_SLIDE, arm_refusal,  # noqa: E402
                         bin_refusal, land, over_bin, over_plate, plate_refusal, refusal, slide_steps, swing_refusal)

from robot_control.config import ArmSettings, KeepOutMap, RigSettings  # noqa: E402
from robot_control.kinematics.arm import Kinematics  # noqa: E402
from robot_control.kinematics.keep_out import KeepOut  # noqa: E402

ARM = ArmSettings.from_config()
ZONES = [KeepOut(m) for m in KeepOutMap.from_config()]
KIN = Kinematics(ARM)
RIG = RigSettings.from_config()
BESIDE = (2.0, 110.0)       # after the elbow's fold: beside the plate
OVER = (-45.0, 110.0)       # the crosshair
PARKED = (-77.0, 110.0)


def plate(joint, angle, pose, slide):
    return plate_refusal(joint, angle, *pose, slide, KIN, RIG)


def test_where_the_plate_is():
    assert over_plate(*OVER, KIN, RIG)
    assert not over_plate(*BESIDE, KIN, RIG)


def test_the_base_only_crosses_the_plate_with_the_slide_up():
    assert plate("base", -77.0, BESIDE, HOVER_SLIDE) is None
    assert plate("base", -77.0, BESIDE, 0.0) is None
    assert "over the plate" in plate("base", -77.0, BESIDE, 30.0)
    assert "over the plate" in plate("base", -77.0, BESIDE, None)


def test_only_small_nudges_are_allowed_low_over_the_plate():
    assert plate("base", -43.0, OVER, 30.0) is None
    assert plate("elbow", 112.0, OVER, 30.0) is None
    assert "over the plate" in plate("base", -30.0, OVER, 30.0)
    assert "over the plate" in plate("base", -43.0, OVER, None)


def test_folding_the_elbow_after_calibrating_is_clear_of_the_plate():
    assert plate("elbow", 110.0, (2.14, 90.46), None) is None


def test_the_slide_keeps_off_the_plate():
    assert "first move" in plate("slide", 0.0, OVER, None)
    assert plate("slide", 0.0, BESIDE, None) is None
    assert "stays above" in plate("slide", 64.0, OVER, 60.0)
    assert plate("slide", -15.0, OVER, -20.0) is None
    assert plate("slide", 10.0, BESIDE, 5.0) is None


class ShortArm:
    """An arm whose joints stop `short` of what they are sent."""

    def __init__(self, base, elbow, short):
        self.at, self.short, self.sent = {0: base, 1: elbow}, short, []

    def SCARA_READ_ANGLE(self, axis):
        return self.at[axis]

    def SCARA_MOVE_JOINTS(self, base, elbow, wrist, slide, seconds):
        self.sent.append((base, elbow, wrist, slide))
        for axis, angle in enumerate((base, elbow)):
            self.at[axis] = angle - self.short * (1 if angle > self.at[axis] else -1)      # short of it, either way


def test_landing_makes_up_a_joints_shortfall():
    arm = ShortArm(-51.0, 96.0, 0.8)
    base, elbow = land(arm, {"base": -52.0, "elbow": 97.0}, 60.0, 45.0)
    assert abs(base + 52.0) <= LAND_TOLERANCE_DEG and abs(elbow - 97.0) <= LAND_TOLERANCE_DEG


def test_landing_leaves_a_joint_on_target_alone():
    arm = ShortArm(-52.1, 97.1, 0.0)
    land(arm, {"base": -52.0, "elbow": 97.0}, 60.0, 45.0)
    assert arm.sent == []


def test_landing_holds_the_wrist_and_slide_where_they_are():
    arm = ShortArm(-51.0, 96.0, 0.4)
    land(arm, {"base": -52.0, "elbow": 97.0}, 60.0, 45.0)
    assert arm.sent and all(s[2:] == (60.0, 45.0) for s in arm.sent)


def test_landing_never_asks_for_much_more_than_the_target():
    arm = ShortArm(-40.0, 97.0, 30.0)
    land(arm, {"base": -52.0, "elbow": 97.0}, 60.0, 45.0)
    assert all(abs(s[0] + 52.0) <= LAND_MAX_DEG for s in arm.sent)


def swing(base, elbow, wrist, start, slide):
    return swing_refusal(base, elbow, wrist, start, slide, ARM, ZONES, KIN, RIG)


def test_a_swing_across_the_plate_needs_the_slide_up():
    assert swing(-50.0, 97.0, 90.0, BESIDE, HOVER_SLIDE) is None
    assert "over the plate" in swing(-50.0, 97.0, 90.0, BESIDE, 45.0)


def test_a_glide_may_cross_the_plate_at_its_own_height_but_no_lower():
    assert swing_refusal(-50.0, 97.0, 90.0, BESIDE, 45.0, ARM, ZONES, KIN, RIG, clear=45.0) is None
    assert "over the plate" in swing_refusal(-50.0, 97.0, 90.0, BESIDE, 46.0, ARM, ZONES, KIN, RIG, clear=45.0)


def test_the_park_is_over_the_non_resistor_bin_and_the_plate_isnt():
    assert over_bin(*PARKED, KIN, RIG)
    assert not over_bin(*OVER, KIN, RIG)
    assert not over_bin(*BESIDE, KIN, RIG)


def test_leaving_or_reaching_the_park_needs_the_slide_fully_up():
    assert swing(-50.0, 97.0, 90.0, PARKED, PARK_SLIDE) is None
    assert "non-resistor bin" in swing(-50.0, 97.0, 90.0, PARKED, HOVER_SLIDE)
    assert "non-resistor bin" in swing(*PARKED, 90.0, OVER, HOVER_SLIDE)
    assert bin_refusal("base", -77.0, *OVER, PARK_SLIDE, KIN, RIG) is None
    assert "non-resistor bin" in bin_refusal("base", -77.0, *OVER, HOVER_SLIDE, KIN, RIG)
    assert "non-resistor bin" in bin_refusal("base", -45.0, *PARKED, HOVER_SLIDE, KIN, RIG)


def test_over_the_non_resistor_bin_the_slide_only_goes_up():
    assert bin_refusal("slide", PARK_SLIDE, *PARKED, HOVER_SLIDE, KIN, RIG) is None
    assert bin_refusal("slide", -10.0, *PARKED, HOVER_SLIDE, KIN, RIG) is None      # on its way up
    assert "only goes up" in bin_refusal("slide", HOVER_SLIDE, *PARKED, PARK_SLIDE, KIN, RIG)
    assert "first move" in bin_refusal("slide", 0.0, *PARKED, None, KIN, RIG)
    assert bin_refusal("slide", 63.0, *OVER, HOVER_SLIDE, KIN, RIG) is None         # over the plate, as before


def run_park(monkeypatch, pose, slide):
    """park with the arm faked at pose (base, elbow) and the slide at slide: the moves it made, in order."""
    made, at, state = [], dict(zip(("base", "elbow"), pose)), {"wrist": 0.0, "slide": slide}

    def move(scara, joint, angle, seconds):
        made.append((joint, angle))
        if joint in at:
            at[joint] = angle
        else:
            state[joint] = angle

    monkeypatch.setattr(slide_wrist, "move", move)
    monkeypatch.setattr(slide_wrist, "load_state", lambda: state)
    monkeypatch.setattr(slide_wrist, "read_joints", lambda *a: (at["base"], at["elbow"]))
    def swing(scara, base, elbow, wrist, seconds):
        made.append(("swing", base, elbow))
        at.update(base=base, elbow=elbow)

    monkeypatch.setattr(slide_wrist, "swing", swing)
    monkeypatch.setattr(slide_wrist, "print", lambda *a: None, raising=False)
    slide_wrist.park(None)
    return made


def test_parking_raises_the_slide_fully_then_swings_in_one_move(monkeypatch):
    made = run_park(monkeypatch, (-45.0, 120.0), 63.0)
    assert made == [("slide", PARK_SLIDE), ("swing", PARK["base"], PARK["elbow"])]


def test_parking_moves_the_elbow_then_the_base_when_one_swing_isnt_allowed(monkeypatch):
    monkeypatch.setattr(slide_wrist, "swing_refusal", lambda *a, **k: "crosses the camera_tower keep-out")
    made = run_park(monkeypatch, (-45.0, 120.0), 63.0)
    assert made == [("slide", PARK_SLIDE), ("elbow", PARK["elbow"]), ("base", PARK["base"])]


def test_parking_after_a_start_folds_the_elbow_before_the_slides_first_move(monkeypatch):
    made = run_park(monkeypatch, (2.14, 90.46), None)
    assert made == [("elbow", PARK["elbow"]), ("slide", 0.0), ("slide", PARK_SLIDE), ("base", PARK["base"])]
    # (after a start the wrist hasn't moved, so a swing, which sends all four joints, isn't allowed yet)


def test_a_swing_keeps_to_the_limits_and_clear_of_the_tower():
    assert "elbow" in swing(-50.0, 139.0, 90.0, PARKED, HOVER_SLIDE)
    assert "base" in swing(69.0, 110.0, 90.0, PARKED, HOVER_SLIDE)
    assert "wrist" in swing(-50.0, 97.0, 130.0, PARKED, HOVER_SLIDE)
    assert "keep-out" in swing(40.0, 60.0, 0.0, (0.0, 60.0), HOVER_SLIDE)


def test_slide_steps_to_the_hover():
    assert slide_steps(None, -40.0) == [0.0, -40.0]               # after a RESET: to 0 first, then up at once
    assert slide_steps(-40.0, -40.0) == []
    assert slide_steps(63.0, -50.0) == [-50.0]                    # up from the plate to fully up: one move
    assert slide_steps(-42.0, -40.0) == [-40.0]
    assert slide_steps(-50.0, 20.0) == [20.0]                     # park to the travel height: one move
    assert slide_steps(-50.0, 63.0) == [25.0, 63.0]               # down: at most 75 deg a move
    assert slide_steps(0.0, -40.0, 30.0, up_in_one=False) == [-30.0, -40.0]    # the wrist: steps both ways


def test_the_first_move_after_a_reset_must_be_to_zero():
    assert refusal("slide", 0.0, None) is None
    assert "first move" in refusal("slide", 5.0, None)
    assert "first move" in refusal("wrist", -10.0, None)


def test_later_moves_are_small_and_inside_the_safe_range():
    assert refusal("slide", 60.0, -15.0) is None
    assert "step" in refusal("slide", 60.0, -20.0)                # 80 deg down at once
    assert refusal("slide", -50.0, 63.0) is None                  # up: any distance
    assert "step" in refusal("wrist", -80.0, 110.0)                # the wrist keeps its step both ways
    assert refusal("wrist", 100.0, -60.0) is None
    assert "step" in refusal("wrist", 110.0, -80.0)
    assert "safe range" in refusal("slide", 64.0, 62.0)


def test_arm_moves_stay_in_range_and_clear_of_the_tower():
    assert arm_refusal("base", -45.0, 0.0, 110.0, ARM, ZONES) is None
    assert "range" in arm_refusal("elbow", 139.0, -45.0, 110.0, ARM, ZONES)
    assert "range" in arm_refusal("base", 69.0, 0.0, 110.0, ARM, ZONES)
    assert "keep-out" in arm_refusal("base", 40.0, 0.0, 60.0, ARM, ZONES)


def test_a_pose_in_the_keep_out_can_only_leave_it():
    assert arm_refusal("elbow", 110.0, 2.14, 90.46, ARM, ZONES) is None         # where calibrating ends
    assert "keep-out" in arm_refusal("elbow", 92.0, 2.14, 90.46, ARM, ZONES)    # still inside
    assert "keep-out" in arm_refusal("base", 40.0, 2.14, 90.46, ARM, ZONES)     # further in
