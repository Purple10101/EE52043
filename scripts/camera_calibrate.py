"""Calibrates the arm against the camera: visits points on the plate and fits what it finds.

    python scripts/camera_calibrate.py run --calibration <saved plate fit>
    python scripts/camera_calibrate.py fit logs/camera_calibration_<time>.csv [--save]
    python scripts/camera_calibrate.py check --calibration <saved plate fit>

run needs scripts/tip_view.py --snapshot --lock running, which holds the camera, and the arm started and
parked (scripts/slide_wrist.py). At each point it lifts to the travel height, goes to the point with the
jaws face-on to the camera, comes down to the pick height, and reads the tip there beside the base and
elbow potentiometers. At the pick height the tip is all but on the plate, so what the camera sees needs
almost no correction for height. The arm needn't land on the point: a pair only needs its two halves
read together. A point where the camera can't read the tip is skipped. Every pair is written as it
comes, so a run stopped part way still leaves its results.

fit takes the tip's offset off each pair to get the wrist's axis, then fits the axes to the
potentiometers (robot_control/vision/arm_fit.py) and reports each point, and how well each is
predicted when left out of the fit. --save keeps the joint fit for everything that plans a move.

check goes to CHECK_POINTS, which the fit never saw, on the saved calibration alone: one move each, the
camera only watching, and reports how far the tip landed from each at the pick height.
"""

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import camera_guide  # noqa: E402
import slide_wrist as bench  # noqa: E402
from robot_offline.pre import rectify  # noqa: E402

from robot_control.config import ArmSettings, RigSettings  # noqa: E402
from robot_control.kinematics.arm import Kinematics  # noqa: E402
from robot_control.vision.arm_fit import (JointCorrection, errors_mm, fit_joints, fit_rigid, load_calibration,  # noqa: E402
                                          save_calibration)
from robot_control.vision.detector import camera_mm  # noqa: E402
from robot_control.vision.guide import axis_mm, plan_point  # noqa: E402
from robot_control.vision.rig import correct_height  # noqa: E402

# Scene mm: round the plate, kept to its white top and to where the elbow stays under bench.ELBOW_MAX.
POINTS = [(-0.48, -130.37), (40.0, -130.0), (40.0, -170.0), (0.0, -170.0), (-40.0, -170.0), (-40.0, -130.0),
          (-35.0, -112.0), (0.0, -114.0), (35.0, -112.0), (35.0, -100.0), (0.0, -102.0), (-35.0, -100.0)]
CHECK_POINTS = [(20.0, -150.0), (-20.0, -150.0), (20.0, -122.0), (-20.0, -122.0), (0.0, -142.0)]
PICK_SLIDE = bench.OVER_PLATE_SLIDE
PICK_HEIGHT_MM = 1.0        # the tip above the plate there, by eye
SLIDE_MOVE_S = 1.0
SETTLED_S = 1.0
FIELDS = ["target_x", "target_y", "tip_x", "tip_y", "base", "elbow", "wrist", "slide", "height"]


def drive(scara, joint, target, seconds):
    """Moves the wrist or slide to target in steps the bench allows."""
    last = bench.load_state()[joint]
    for angle in bench.slide_steps(last, target, bench.MAX_STEP[joint]):
        bench.move(scara, joint, round(angle, 1), seconds)


def visit(scara, point, lens, surface, kinematics, rig, correction, landed, writer):
    base, elbow, wrist = plan_point(point, lens, kinematics, rig, correction)
    print(f"\nPoint ({point[0]:g}, {point[1]:g}): base {base:.1f}, elbow {elbow:.1f}, wrist {wrist:.1f}", flush=True)
    drive(scara, "slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
    bench.approach(scara, base, elbow, wrist, bench.SWING_S)
    (camera_guide.LIVE.parent / "near.txt").write_text(f"{point[0]} {point[1]}")
    drive(scara, "slide", PICK_SLIDE, SLIDE_MOVE_S)
    if landed:      # only now: the descent can shift the base
        bench.land(scara, {"base": base, "elbow": elbow}, wrist, PICK_SLIDE)
    time.sleep(SETTLED_S)
    try:
        seen = camera_guide.read_seen()
    except SystemExit as stopped:
        print(f"  skipped: {stopped}", flush=True)
        return
    tip = correct_height([seen], lens, surface, PICK_HEIGHT_MM)[0]
    base, elbow = bench.read_joints(scara)
    state = bench.load_state()
    writer(point[0], point[1], round(tip[0], 2), round(tip[1], 2), round(base, 2), round(elbow, 2),
           state["wrist"], state["slide"], PICK_HEIGHT_MM)
    print(f"  tip ({tip[0]:.2f}, {tip[1]:.2f}), {np.hypot(*(tip - point)):.2f} mm from the point; "
          f"base {base:.2f}, elbow {elbow:.2f}", flush=True)


def tour(args, points, name, rig, correction, landed=False):
    """Visits points, writing each pair to logs/<name>_<time>.csv, and parks."""
    cal = rectify.Calibration.load(args.calibration)
    lens, surface = camera_mm(cal), cal.settings.top_z_mm
    kinematics = Kinematics(ArmSettings.from_config())
    scara = bench.smc_api.scara_motion_controller(port=args.port or bench.find_port(), timeout=120)
    path = ROOT / "logs" / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(FIELDS)

        def writer(*row):
            w.writerow(row)
            f.flush()

        for point in points:
            visit(scara, point, lens, surface, kinematics, rig, correction, landed, writer)
    drive(scara, "slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
    bench.park(scara)
    print(f"\nSaved {path.relative_to(ROOT)}")
    return path


def run(args):
    tour(args, POINTS, "camera_calibration", RigSettings.from_config(), JointCorrection())


def check(args):
    path = tour(args, CHECK_POINTS, "camera_check", *load_calibration(), landed=True)
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    misses = np.array([np.hypot(float(r["tip_x"]) - float(r["target_x"]), float(r["tip_y"]) - float(r["target_y"]))
                       for r in rows])
    if len(misses):
        print(f"{len(misses)} of {len(CHECK_POINTS)} points: {np.sqrt((misses ** 2).mean()):.2f} mm rms, "
              f"worst {misses.max():.2f}: " + " ".join(f"{d:.1f}" for d in misses))


def report(name, fit):
    print(f"{name}: {fit.rms_mm:.2f} mm rms, worst {fit.worst_mm:.2f}; rotation {fit.rig.rotation_deg % 360:.2f} deg, "
          f"translation ({1000 * fit.rig.translation_m[0]:.2f}, {1000 * fit.rig.translation_m[1]:.2f}) mm, "
          f"base scale {fit.joints.base_scale:.4f}, elbow offset {fit.joints.elbow_offset_deg:.2f} deg")
    print("  each point, mm: " + " ".join(f"{d:.1f}" for d in fit.distances_mm))


def left_out(fitter, scene, joints, kinematics):
    """Each pair's error when the fit is made without it, mm."""
    out = []
    for i in range(len(scene)):
        keep = np.arange(len(scene)) != i
        fit = fitter(scene[keep], joints[keep], kinematics)
        out.append(float(np.hypot(*errors_mm(scene[i:i + 1], joints[i:i + 1], kinematics, fit.rig, fit.joints)[0])))
    return np.array(out)


def fit(args):
    with open(args.pairs, newline="") as f:
        rows = list(csv.DictReader(f))
    tips = np.array([(float(r["tip_x"]), float(r["tip_y"])) for r in rows])
    joints = np.array([(float(r["base"]), float(r["elbow"])) for r in rows])
    axes = np.array([axis_mm(t, b, e, float(r["wrist"])) for t, (b, e), r in zip(tips, joints, rows)])
    kinematics, rig = Kinematics(ArmSettings.from_config()), RigSettings.from_config()
    print(f"{len(rows)} pairs")
    nominal = np.hypot(*errors_mm(axes, joints, kinematics, rig).T)
    print(f"as configured: {np.sqrt((nominal ** 2).mean()):.2f} mm rms, worst {nominal.max():.2f}")
    print("  each point, mm: " + " ".join(f"{d:.1f}" for d in nominal))
    for name, fitter in (("rigid", fit_rigid), ("joints", fit_joints)):
        report(name, fitter(axes, joints, kinematics))
        missing = left_out(fitter, axes, joints, kinematics)
        print(f"  left out: {np.sqrt((missing ** 2).mean()):.2f} mm rms, worst {missing.max():.2f}: "
              + " ".join(f"{d:.1f}" for d in missing))
    if args.save:
        best = fit_joints(axes, joints, kinematics)
        missing = left_out(fit_joints, axes, joints, kinematics)
        save_calibration(best, {"date": time.strftime("%Y-%m-%d"), "pairs": Path(args.pairs).name, "rms_mm": round(best.rms_mm, 2),
                                "worst_mm": round(best.worst_mm, 2),
                                "left_out_rms_mm": round(float(np.sqrt((missing ** 2).mean())), 2)})
        print("Saved the joint fit.")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    commands = p.add_subparsers(dest="command", required=True)
    r = commands.add_parser("run")
    r.add_argument("--calibration", required=True, help="the saved plate fit tip_view started from")
    r.set_defaults(go=run)
    c = commands.add_parser("check")
    c.add_argument("--calibration", required=True, help="the saved plate fit tip_view started from")
    c.set_defaults(go=check)
    f = commands.add_parser("fit")
    f.add_argument("pairs")
    f.add_argument("--save", action="store_true", help="keep the joint fit")
    f.set_defaults(go=fit)
    args = p.parse_args()
    args.go(args)


if __name__ == "__main__":
    main()
