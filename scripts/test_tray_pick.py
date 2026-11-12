"""The user's test of picking with the arm starting from the trays (2026-11-11): two resistors on the plate.

    python scripts/test_tray_pick.py
    python scripts/test_tray_pick.py --from-park     # the same, but parked before each look and pick

1. The arm over the A1 drop point, as after a drop there.
2. Look: segmentation only, no band reading (values don't matter here).
3. Pick the first resistor found and drop it in A1.
4. Look again, the arm where it dropped.
5. Pick the other and drop it in A2.
6. Park.

Every step prints what it aimed at and where the joints went (the moves are also in logs/slide_wrist_<date>.csv),
and each look is saved to logs/sort_runs/<time>/ (picture, averaged frame, readings) for the record.

Before: the arm started and parked (CALIBRATE in the sorting window, then close that window: this needs the
camera and the Nucleo's port), the gripper board plugged in, exactly two resistors on the plate, and a hand on
the servo supply's plug.
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav  # noqa: E402
import slide_wrist as bench  # noqa: E402
import sort_resistors  # noqa: E402

from robot_control.config import RigSettings  # noqa: E402
from robot_control.planning.planner import Plan  # noqa: E402
from robot_control.task.run_log import RunLog  # noqa: E402
from robot_control.vision.arm_fit import JointCorrection  # noqa: E402
from robot_control.vision.eyes import Eyes  # noqa: E402
from robot_control.vision.guide import plan_jaws, pots_for  # noqa: E402


def say(text):
    print(text, flush=True)


def look(eyes, log, label):
    view = eyes.look(sharp=True)
    log.look(0, view, Plan(None, {r.number: label for r in view.resistors}, 0))
    say(f"look ({label}): {len(view.resistors)} resistor(s): " +
        ", ".join(f"#{r.number} at ({r.centre_mm[0]:.1f}, {r.centre_mm[1]:.1f}) mm, long axis {r.direction_deg:.0f} deg"
                  for r in view.resistors))
    return view


def hover_photo(arm, eyes, resistor, path):
    """The open jaws 9 mm over the resistor (Navigator.go), and a picture of them from the camera."""
    import cv2
    arm.navigator.go(resistor.centre_mm, resistor.direction_deg)
    for _ in range(6):                  # let the exposure settle on the arm in view
        eyes.cap.read()
    ok, frame = eyes.cap.read()
    if ok:
        cv2.imwrite(str(path), frame)
        say(f"  hover picture: {path}")


def pick_and_drop(arm, resistor, bin_id, eyes=None, photo=None):
    nav = arm.navigator
    base, elbow, wrist = plan_jaws(resistor.centre_mm, resistor.direction_deg, bench.SAFE["wrist"], nav.kinematics, nav.rig,
                                   resistor_nav.WRIST_CENTRE, nav.correction, at_axis=nav.aim_axis)
    say(f"picking #{resistor.number} at ({resistor.centre_mm[0]:.1f}, {resistor.centre_mm[1]:.1f}): "
        f"planned base {base:.2f}, elbow {elbow:.2f}, wrist {wrist:.1f}")
    if photo is not None:
        hover_photo(arm, eyes, resistor, photo)
    got = arm.pick(resistor)
    landed = arm.joints()
    say(f"  {'jaws closed at the bottom' if got else 'NOT picked: ' + arm.message}; joints now base {landed[0]:.2f}, "
        f"elbow {landed[1]:.2f} (off {landed[0] - base:+.2f}, {landed[1] - elbow:+.2f})")
    if arm.holding:
        say(f"  to {bin_id}: {'dropped' if arm.drop(bin_id) else 'NOT dropped: ' + arm.message}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--from-park", action="store_true", help="park before each look and pick, instead of the trays")
    p.add_argument("--nominal", action="store_true",
                   help="aim without the camera calibration: the rig from workspace.toml, the pots' angles as they are, "
                        "the gripper a line straight down from the wrist mount")
    p.add_argument("--aim-jaws", action="store_true",
                   help="aim the jaws' centre (6.8 mm off the shaft axis along the jaw faces), not the shaft axis")
    p.add_argument("--hover-photo", action="store_true", help="stop 9 mm over each resistor and save a camera picture")
    args = p.parse_args()
    if bench.load_state() is None:
        sys.exit("Not started: CALIBRATE in the sorting window first (then close it).")
    slots = resistor_nav.load_bins()
    arm, _ = sort_resistors.build_arm(None, slots, camera_calibration=not args.nominal)
    if args.nominal:
        arm.navigator.rig, arm.navigator.correction = RigSettings.from_config(), JointCorrection()
        arm.navigator.jaw_offset = None
        say(f"NOMINAL: no camera calibration ({arm.navigator.rig}, {arm.navigator.correction})")
    if args.aim_jaws:
        arm.navigator.aim_axis = False
        say("AIMING THE JAWS' CENTRE, not the shaft axis")
    eyes = Eyes(closeups=False, say=say)
    log = RunLog()
    say(f"Logging to {log.folder}")
    try:
        a1 = slots["A1"]
        base, elbow = pots_for(resistor_nav.drop_point(a1), arm.navigator.kinematics, RigSettings.from_config(),
                               JointCorrection())
        if args.from_park:
            say("1. parked (--from-park)")
            arm.park()
        else:
            say(f"1. over A1's drop point: base {base:.2f}, elbow {elbow:.2f}")
            arm.move_to(base, elbow)

        first_look = look(eyes, log, "first look, arm over A1")
        if len(first_look.resistors) != 2:
            say(f"expected 2 resistors, found {len(first_look.resistors)}: stopping (the arm stays where it is)")
            return
        first = first_look.resistors[0]
        say("3. the first resistor to A1")
        pick_and_drop(arm, first, "A1", eyes, log.folder / "hover_1.jpg" if args.hover_photo else None)

        if args.from_park:
            say("4. parked again before the second look (--from-park)")
            arm.park()
        second_look = look(eyes, log, "second look, " + ("parked" if args.from_park else "arm after the A1 drop"))
        others = [r for r in second_look.resistors if r.distance_to(first.centre_mm) > 4.0]
        if not others:
            say("the second resistor wasn't found apart from the first's spot: stopping")
            return
        say("5. the second resistor to A2")
        pick_and_drop(arm, others[0], "A2", eyes, log.folder / "hover_2.jpg" if args.hover_photo else None)

        say("6. parking")
        arm.park()
        look(eyes, log, "last look, parked")
    finally:
        eyes.close()


if __name__ == "__main__":
    main()
