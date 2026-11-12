"""Does the slide come straight down onto a resistor? The arm's geometry alone.

    python scripts/test_shaft_over.py                 # the CAD geometry: no camera calibration
    python scripts/test_shaft_over.py --calibrated    # the same with camera_calibration.json, to compare
    python scripts/test_shaft_over.py --hold 30       # seconds at the bottom over each (default 20)

The user's framing (2026-11-11): take the gripper as a line straight down from the slide's shaft; the job is the
shaft's axis lowered directly over the resistor's centre. For each resistor the camera finds (parked,
segmentation only), the shaft's axis is put over its centre (resistor_nav's go, aiming the axis), the open jaws
lowered to the bottom (63) without closing, and held there: where the body sits between the jaws, along and
across, is how far the arm is off. Then lifted; parks at the end.

Before: the arm started and parked, the sorting window closed, the gripper board plugged in, resistors spread
over the plate, a hand on the servo supply's plug.
"""

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav  # noqa: E402
import slide_wrist as bench  # noqa: E402
import sort_resistors  # noqa: E402

from robot_control.config import RigSettings  # noqa: E402
from robot_control.vision.arm_fit import JointCorrection  # noqa: E402
from robot_control.vision.eyes import Eyes  # noqa: E402
from robot_control.task.supervisor import LOOK_POSE  # noqa: E402
from robot_control.vision.guide import plan_jaws  # noqa: E402


def say(text):
    print(text, flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calibrated", action="store_true", help="aim with camera_calibration.json (default: the CAD geometry)")
    p.add_argument("--from-bin", action="store_true",
                   help="before each, over tray slot A2 (as after a drop), so every approach comes from the trays' side")
    p.add_argument("--hold", type=float, default=20.0, help="seconds at the bottom over each resistor")
    args = p.parse_args()
    if bench.load_state() is None:
        sys.exit("Not started: CALIBRATE in the sorting window first (then close it).")
    arm, _ = sort_resistors.build_arm(None, resistor_nav.load_bins(), camera_calibration=args.calibrated)
    nav = arm.navigator
    if not args.calibrated:
        nav.rig, nav.correction, nav.jaw_offset = RigSettings.from_config(), JointCorrection(), None
    nav.aim_axis = True
    say(f"aiming the shaft's axis by {'the camera calibration' if args.calibrated else 'the CAD geometry'}: "
        f"{nav.rig}, {nav.correction}")
    eyes = Eyes(closeups=False, say=say)
    try:
        arm.park()
        view = eyes.look(sharp=True)
        if not view.resistors:
            say("no resistors found on the plate")
            return
        for r in view.resistors:
            x, y = r.centre_mm
            if args.from_bin:
                say(f"\nover A2 first (--from-bin): base {LOOK_POSE[0]}, elbow {LOOK_POSE[1]}")
                arm.move_to(*LOOK_POSE)
            base, elbow, wrist = plan_jaws(r.centre_mm, r.direction_deg, bench.SAFE["wrist"], nav.kinematics, nav.rig,
                                           resistor_nav.WRIST_CENTRE, nav.correction, at_axis=True)
            say(f"\n#{r.number} at ({x:.1f}, {y:.1f}) mm, long axis {r.direction_deg:.0f} deg: "
                f"base {base:.2f}, elbow {elbow:.2f}")
            if nav.go(r.centre_mm, r.direction_deg) is None:
                say(f"  #{r.number} not reached: {nav.status}")
                continue
            nav.drive("slide", bench.OVER_PLATE_SLIDE, resistor_nav.SLIDE_MOVE_S)
            got = arm.joints()
            say(f"  DOWN over #{r.number}, jaws open: where does the body sit between them? ({args.hold:g} s) "
                f"Joints base {got[0]:.2f}, elbow {got[1]:.2f} (off {got[0] - base:+.2f}, {got[1] - elbow:+.2f})")
            time.sleep(args.hold)
            nav.lift()
        say("\nparking")
        arm.park()
    finally:
        eyes.close()


if __name__ == "__main__":
    main()
