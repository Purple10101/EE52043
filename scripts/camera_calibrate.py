"""Calibrates the arm against the camera: visits points on the plate and fits what it finds.

    python scripts/camera_calibrate.py plate
    python scripts/camera_calibrate.py run [--calibration <saved plate fit>]
    python scripts/camera_calibrate.py fit logs/camera_calibration_<time>.csv [--save]
    python scripts/camera_calibrate.py check [--calibration <saved plate fit>]

The README's "Calibrating the system" is the order to use them in.

plate finds the plate in a live frame (robot_offline's plate_search.find_now, as every tool finds it) and
saves the fit to logs/tips/plate_<time>_rectify.json, which run, check and tip_view.py use. Nothing else may be
using the camera, and the arm must be parked.

run and check (--calibration defaults to the newest plate_*_rectify.json) need scripts/tip_view.py
--snapshot --lock running, which holds the camera, and the arm started and parked (scripts/slide_wrist.py).
They close the jaws first if the gripper board is connected: the camera finds the tip by the closed jaws'
V. At each point it lifts to the travel height, goes to the point with the
jaws face-on to the camera, comes down to the pick height, and reads the tip there beside the base and
elbow potentiometers. At the pick height the tip is all but on the plate, so what the camera sees needs
almost no correction for height. The arm needn't land on the point: a pair only needs its two halves
read together. A point where the camera can't read the tip is skipped. Every pair is written as it
comes, so a run stopped part way still leaves its results.

fit takes the tip's offset off each pair to get the wrist's axis, then fits the axes to the
potentiometers (robot_control/vision/arm_fit.py) and reports each point, and how well each is
predicted when left out of the fit. --save keeps the joint fit for everything that plans a move.

check goes to CHECK_POINTS, which the fit never saw, on the saved calibration alone: one move each, the
camera only watching, and reports how far the tip landed from each at the pick height. It also reports
the calibration on its own: the tip seen against where the potentiometers say the arm is, which leaves
out how well the joints landed. A low figure there with large misses means landing, not calibration.
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
import cv2  # noqa: E402
import slide_wrist as bench  # noqa: E402
from robot_offline import config as offline_config  # noqa: E402
from robot_offline.pre import plate_search, rectify  # noqa: E402
from robot_offline.pre.camera import open_camera  # noqa: E402

from robot_control.config import ArmSettings, RigSettings  # noqa: E402
from robot_control.gripper import Gripper, GripperError  # noqa: E402
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
PLATE_FITS = ROOT / "logs" / "tips"     # plate's saved fits: records the arm calibration is taken against


def newest_plate_fit():
    """The newest plate fit saved by plate (not tip_view's tip_* saves), or None."""
    found = sorted(PLATE_FITS.glob("plate_*_rectify.json"), key=lambda p: p.stat().st_mtime) if PLATE_FITS.is_dir() else []
    return str(found[-1]) if found else None


def plate(args):
    """Finds the plate in a live frame, as every tool does (plate_search.find_now), and saves the fit."""
    cap = open_camera(offline_config.load("camera"))
    try:
        try:
            cal = plate_search.find_now(cap)
        except rectify.PlateNotFound as why:
            sys.exit(f"The plate wasn't found: {why}.\nKeep the arm parked, hands off the plate and the crosshair "
                     "clear: on a pale bench the crosshair stands in for the edge the bench hides. If it still "
                     "isn't found, lay a matte black strip along that edge, touching it.")
        ok, frame = cap.read()
        exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
    finally:
        cap.release()
    PLATE_FITS.mkdir(parents=True, exist_ok=True)
    path = PLATE_FITS / f"plate_{time.strftime('%Y%m%d_%H%M%S')}_rectify.json"
    cal.save(path)
    cv2.imwrite(str(path.with_name(path.name.replace("_rectify.json", "_raw.png"))), frame)
    s = cal.fit
    print(f"Found at exposure {exposure}: edges (share, rms px) {s['edges']}, near side by {s['near_side_by']}, "
          f"camera at {s['camera_mm']} mm, focal {s['focal_px']} px")
    print(f"Saved {path.relative_to(ROOT)}. Next, leave this running in its own terminal:\n"
          f"  .venv\\Scripts\\python scripts\\tip_view.py --snapshot --lock --calibration {path.relative_to(ROOT)}")


def close_jaws():
    """Closes the jaws if the gripper board is connected: the camera finds the tip by the closed V."""
    try:
        gripper = Gripper()
    except (GripperError, OSError) as why:
        print(f"The gripper board isn't connected ({why}): the jaws must already be closed.")
        return
    try:
        gripper.close()
        print("Jaws closed.")
    finally:
        gripper.disconnect()


def drive(scara, joint, target, seconds):
    """Moves the wrist or slide to target in steps the bench allows."""
    last = bench.load_state()[joint]
    for angle in bench.slide_steps(last, target, bench.MAX_STEP[joint], joint == "slide"):
        bench.move(scara, joint, round(angle, 1), seconds)


def visit(scara, point, lens, surface, kinematics, rig, correction, landed, writer):
    base, elbow, wrist = plan_point(point, lens, kinematics, rig, correction)
    print(f"\nPoint ({point[0]:g}, {point[1]:g}): base {base:.1f}, elbow {elbow:.1f}, wrist {wrist:.1f}", flush=True)
    if bench.load_state()["slide"] > bench.HOVER_SLIDE:     # up to the hover; from the park it is higher already
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
    if args.calibration is None:
        sys.exit("No saved plate fit: run plate first.")
    print(f"Plate fit: {args.calibration}")
    cal = rectify.Calibration.load(args.calibration)
    close_jaws()
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


def pairs(rows):
    """(wrist axes scene mm, potentiometer joints deg), each N × 2, from rows of a run or check."""
    tips = np.array([(float(r["tip_x"]), float(r["tip_y"])) for r in rows])
    joints = np.array([(float(r["base"]), float(r["elbow"])) for r in rows])
    axes = np.array([axis_mm(t, b, e, float(r["wrist"])) for t, (b, e), r in zip(tips, joints, rows)])
    return axes, joints


def check(args):
    rig, correction = load_calibration()
    path = tour(args, CHECK_POINTS, "camera_check", rig, correction, landed=True)
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print("No point was read: is the tip in view, with the jaws closed?")
        return
    misses = np.array([np.hypot(float(r["tip_x"]) - float(r["target_x"]), float(r["tip_y"]) - float(r["target_y"]))
                       for r in rows])
    print(f"{len(misses)} of {len(CHECK_POINTS)} points: {np.sqrt((misses ** 2).mean()):.2f} mm rms, "
          f"worst {misses.max():.2f}: " + " ".join(f"{d:.1f}" for d in misses))
    axes, joints = pairs(rows)
    alone = np.hypot(*errors_mm(axes, joints, Kinematics(ArmSettings.from_config()), rig, correction).T)
    print(f"calibration alone: {np.sqrt((alone ** 2).mean()):.2f} mm rms, worst {alone.max():.2f}: "
          + " ".join(f"{d:.1f}" for d in alone))


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
    axes, joints = pairs(rows)
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
    commands.add_parser("plate").set_defaults(go=plate)
    fit_help = "the saved plate fit tip_view started from (default: the newest from plate)"
    r = commands.add_parser("run")
    r.add_argument("--calibration", default=newest_plate_fit(), help=fit_help)
    r.set_defaults(go=run)
    c = commands.add_parser("check")
    c.add_argument("--calibration", default=newest_plate_fit(), help=fit_help)
    c.set_defaults(go=check)
    f = commands.add_parser("fit")
    f.add_argument("pairs")
    f.add_argument("--save", action="store_true", help="keep the joint fit")
    f.set_defaults(go=fit)
    args = p.parse_args()
    args.go(args)


if __name__ == "__main__":
    main()
