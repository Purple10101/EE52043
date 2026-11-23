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
They close the jaws first if the gripper board is connected: the camera finds the tip by the closed jaws
(v4: their black V; v5: the jaw bodies' corners and the two points, robot_control/vision/jaws.py). At each point
it lifts to the travel height, goes to the point with the jaws square to the plate (closing along its y edges,
SQUARE_JAW_DEG; --face-camera: face-on to the camera instead), comes down to the pick height, and reads the tip
there beside the base and elbow potentiometers. With v5 the camera also reads the jaw line: if it is more than
JAW_TOL_DEG from the aim, the wrist is turned by the difference (lifted, from the wrist's start angle, as every
turn) and the tip read again, up to JAW_TRIES times. At the pick height the tip is all but on the plate, so what the camera sees needs
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
from robot_control.vision.guide import CALIBRATION_JAW_DEG, WRIST_SCALE, axis_mm, plan_point, plan_square  # noqa: E402
from robot_control.vision.rig import correct_height  # noqa: E402

# Scene mm: round the plate, kept to its white top and to where the elbow stays under bench.ELBOW_MAX.
# Kept to where the camera sees the closed jaws against the plate (jaws.py): at y -170 one corner is 16 mm
# along y, past the plate's -180 edge, and at x 40 the bodies stand beyond the +x edge in the camera's view.
POINTS = [(-0.48, -130.37), (35.0, -130.0), (35.0, -158.0), (0.0, -158.0), (-40.0, -158.0), (-40.0, -130.0),
          (-35.0, -112.0), (0.0, -114.0), (35.0, -112.0), (35.0, -100.0), (0.0, -102.0), (-35.0, -100.0)]
CHECK_POINTS = [(20.0, -150.0), (-20.0, -150.0), (20.0, -122.0), (-20.0, -122.0), (0.0, -142.0)]
PICK_SLIDE = bench.OVER_PLATE_SLIDE
PICK_HEIGHT_MM = 1.0        # the tip above the plate there, by eye
SLIDE_MOVE_S = 1.0
SETTLED_S = 1.0
SQUARE_JAW_DEG = CALIBRATION_JAW_DEG      # the jaws' aim: closing along the plate's y edges (guide.py)
JAW_TOL_DEG = 3.0           # v5: the jaw line, as the camera reads it, this close to the aim
JAW_TRIES = 2
FIELDS = ["target_x", "target_y", "tip_x", "tip_y", "base", "elbow", "wrist", "slide", "height", "centre_x", "centre_y"]
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


def visit(scara, point, lens, surface, kinematics, rig, correction, landed, writer, face_camera=False):
    if face_camera:
        base, elbow, wrist = plan_point(point, lens, kinematics, rig, correction)
    else:
        base, elbow, wrist = plan_square(point, SQUARE_JAW_DEG, lens, kinematics, rig, correction)
    print(f"\nPoint ({point[0]:g}, {point[1]:g}): base {base:.1f}, elbow {elbow:.1f}, wrist {wrist:.1f}", flush=True)
    if bench.load_state()["slide"] > bench.HOVER_SLIDE:     # up to the hover; from the park it is higher already
        drive(scara, "slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
    bench.approach(scara, base, elbow, wrist, bench.SWING_S)
    (camera_guide.LIVE.parent / "near.txt").write_text(f"{point[0]} {point[1]}")
    drive(scara, "slide", PICK_SLIDE, SLIDE_MOVE_S)
    if landed:      # only now: the descent can shift the base
        bench.land(scara, {"base": base, "elbow": elbow}, wrist, PICK_SLIDE)
    time.sleep(SETTLED_S)
    for attempt in range(JAW_TRIES + 1):
        # tip_view keeps the raw frame (logs/tips/tip_<time>_<label>.png), to look at what the finder saw
        (camera_guide.LIVE.parent / "save.txt").write_text(f"cal_x{point[0]:+.0f}_y{point[1]:.0f}_try{attempt}")
        try:
            seen, jaw, centre = camera_guide.read_seen_and_jaw()
        except SystemExit as stopped:
            print(f"  skipped: {stopped}", flush=True)
            return
        if jaw is None:
            break
        error = face_on_error(jaw, seen, lens) if face_camera else line_error(jaw, SQUARE_JAW_DEG)
        aim = "face-on" if face_camera else f"{SQUARE_JAW_DEG:g} deg"
        if abs(error) <= JAW_TOL_DEG:
            break
        if attempt == JAW_TRIES:
            print(f"  skipped: the jaws are still {error:+.1f} deg from {aim}", flush=True)
            return
        # The jaw line turns by -WRIST_SCALE per wrist degree (guide.jaw_angle).
        wrist = round(bench.load_state()["wrist"] + error / WRIST_SCALE, 1)
        print(f"  jaws {error:+.1f} deg from {aim}: wrist to {wrist:g}", flush=True)
        drive(scara, "slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
        bench.move(scara, "wrist", bench.WRIST_START, bench.WRIST_TURN_S)
        bench.move(scara, "wrist", wrist, bench.WRIST_TURN_S)
        drive(scara, "slide", PICK_SLIDE, SLIDE_MOVE_S)
        if landed:
            bench.land(scara, {"base": base, "elbow": elbow}, wrist, PICK_SLIDE)
        time.sleep(SETTLED_S)
    tip = correct_height([seen], lens, surface, PICK_HEIGHT_MM)[0]
    axis = ("", "") if centre is None else tuple(round(v, 2) for v in correct_height([centre], lens, surface, PICK_HEIGHT_MM)[0])
    base, elbow = bench.read_joints(scara)
    state = bench.load_state()
    writer(point[0], point[1], round(tip[0], 2), round(tip[1], 2), round(base, 2), round(elbow, 2),
           state["wrist"], state["slide"], PICK_HEIGHT_MM, *axis)
    print(f"  tip ({tip[0]:.2f}, {tip[1]:.2f}), {np.hypot(*(tip - point)):.2f} mm from the point; "
          f"base {base:.2f}, elbow {elbow:.2f}", flush=True)


def line_error(jaw_deg, aim_deg):
    """How far a jaw line (deg) is from the aim, deg, -90..90 (a line is the same turned 180)."""
    return float((jaw_deg - aim_deg + 90.0) % 180.0 - 90.0)


def face_on_error(jaw_deg, seen_mm, lens):
    """How far the jaw line (scene deg) is from square to the camera's line of sight to the tip, deg, -90..90."""
    toward = np.degrees(np.arctan2(lens[1] - seen_mm[1], lens[0] - seen_mm[0]))
    return float((jaw_deg - (toward + 90.0) + 90.0) % 180.0 - 90.0)


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
            visit(scara, point, lens, surface, kinematics, rig, correction, landed, writer, args.face_camera)
    drive(scara, "slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
    bench.park(scara)
    print(f"\nSaved {path.relative_to(ROOT)}")
    return path


def run(args):
    tour(args, POINTS, "camera_calibration", RigSettings.from_config(), JointCorrection())


def pairs(rows):
    """(wrist axes scene mm, potentiometer joints deg), each N × 2, from rows of a run or check. The axis is
    the jaws' centre as the camera measured it (gripper v5: from the jaw bodies' corners, so the jaws' real
    direction, not the wrist model's) where the row has one, else the tip less the model's tip offset."""
    tips = np.array([(float(r["tip_x"]), float(r["tip_y"])) for r in rows])
    joints = np.array([(float(r["base"]), float(r["elbow"])) for r in rows])
    axes = np.array([(float(r["centre_x"]), float(r["centre_y"])) if r.get("centre_x") else
                     axis_mm(t, b, e, float(r["wrist"])) for t, (b, e), r in zip(tips, joints, rows)])
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
        # With few pairs the joint fit's two extra numbers overfit: --rigid keeps the rotation and shift only.
        fitter, name = (fit_rigid, "rigid") if args.rigid else (fit_joints, "joint")
        best = fitter(axes, joints, kinematics)
        missing = left_out(fitter, axes, joints, kinematics)
        save_calibration(best, {"date": time.strftime("%Y-%m-%d"), "pairs": Path(args.pairs).name, "fit": name,
                                "rms_mm": round(best.rms_mm, 2), "worst_mm": round(best.worst_mm, 2),
                                "left_out_rms_mm": round(float(np.sqrt((missing ** 2).mean())), 2)})
        print(f"Saved the {name} fit.")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    commands = p.add_subparsers(dest="command", required=True)
    commands.add_parser("plate").set_defaults(go=plate)
    fit_help = "the saved plate fit tip_view started from (default: the newest from plate)"
    r = commands.add_parser("run")
    r.add_argument("--calibration", default=newest_plate_fit(), help=fit_help)
    r.add_argument("--face-camera", action="store_true", help="aim the jaws face-on to the camera, not square to the plate")
    r.set_defaults(go=run)
    c = commands.add_parser("check")
    c.add_argument("--calibration", default=newest_plate_fit(), help=fit_help)
    c.add_argument("--face-camera", action="store_true", help="aim the jaws face-on to the camera, not square to the plate")
    c.set_defaults(go=check)
    f = commands.add_parser("fit")
    f.add_argument("pairs")
    f.add_argument("--save", action="store_true", help="keep the joint fit")
    f.add_argument("--rigid", action="store_true", help="with --save: keep the rigid fit instead (few pairs)")
    f.set_defaults(go=fit)
    args = p.parse_args()
    args.go(args)


if __name__ == "__main__":
    main()
