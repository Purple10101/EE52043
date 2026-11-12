"""Guides the tip onto a point on the plate with the camera, and records a camera/arm pair.

    python scripts/camera_guide.py --height 12                     # onto the crosshair
    python scripts/camera_guide.py --height 12 --target 20 -150    # onto a point, scene mm

Needs scripts/tip_view.py --snapshot --lock --follow running, which holds the camera and writes where
it sees the tip, and the arm started and over the plate with the jaws face-on (scripts/slide_wrist.py).
--height is the tip's height above the plate, mm: the camera sees the tip displaced by it.

Reads the tip, moves the base and elbow by a fraction of the error (robot_control/vision/guide.py),
and repeats until within TOLERANCE_MM. It gives up, without recording, after MAX_TRIES or if the error
grows. Each success is added to logs/camera_pairs_<date>.csv, for robot_control/vision/arm_fit.py.
"""

import argparse
import csv
import os
import re
import statistics
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import slide_wrist as bench  # noqa: E402
from robot_offline.pre import rectify  # noqa: E402

from robot_control.config import ArmSettings, RigSettings  # noqa: E402
from robot_control.kinematics.arm import Kinematics  # noqa: E402
from robot_control.vision.detector import camera_mm  # noqa: E402
from robot_control.vision.guide import step_joints  # noqa: E402
from robot_control.vision.rig import correct_height  # noqa: E402

LIVE = ROOT / "logs" / "tips" / "live.txt"
TOLERANCE_MM = 1.0
MAX_TRIES = 8
WORSE_MM = 2.0              # the error this far above its best means it isn't converging
READINGS = 3                # of the tip, on different snapshots: their median
STEP_S = 1.5
SNAPSHOT_WAIT_S = 6.0


def read_seen():
    """Where the camera sees the tip on the plate now (scene mm), from READINGS snapshots taken after
    this call. Exits if it can't see one."""
    seen, last = [], time.time()
    deadline = last + READINGS * SNAPSHOT_WAIT_S
    while len(seen) < READINGS:
        if time.time() > deadline:
            sys.exit("Stopped: no fresh snapshots. Is scripts/tip_view.py --snapshot running?")
        time.sleep(0.2)
        try:
            changed = os.path.getmtime(LIVE)
            line = LIVE.read_text(encoding="utf-8").splitlines()[1]
        except (OSError, IndexError):
            continue
        if changed <= last:
            continue
        last = changed
        m = re.match(r"tip \((-?[\d.]+), (-?[\d.]+)\)", line)
        if not m:
            sys.exit(f"Stopped: {line}")
        seen.append((float(m[1]), float(m[2])))
    return np.array([statistics.median(c) for c in zip(*seen)])


def record(target, tip, base, elbow, state, height, error):
    path = ROOT / "logs" / f"camera_pairs_{time.strftime('%Y%m%d')}.csv"
    new = not path.exists()
    with path.open("a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["time", "target_x", "target_y", "tip_x", "tip_y", "base", "elbow", "wrist", "slide",
                        "height", "error"])
        w.writerow([time.strftime("%H:%M:%S"), *target, f"{tip[0]:.2f}", f"{tip[1]:.2f}", f"{base:.2f}", f"{elbow:.2f}",
                    state["wrist"], state["slide"], height, f"{error:.2f}"])
    return path


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--height", type=float, required=True, help="the tip's height above the plate, mm")
    p.add_argument("--target", type=float, nargs=2, metavar=("X", "Y"), help="scene mm (default the crosshair)")
    p.add_argument("--calibration", required=True, help="the saved plate fit tip_view started from")
    p.add_argument("--tolerance", type=float, default=TOLERANCE_MM, help=f"mm ({TOLERANCE_MM:g})")
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    args = p.parse_args()

    cal = rectify.Calibration.load(args.calibration)
    lens, surface = camera_mm(cal), cal.settings.top_z_mm
    target = np.array(args.target or cal.settings.cross_mm)
    kinematics, rig = Kinematics(ArmSettings.from_config()), RigSettings.from_config()
    scara = bench.smc_api.scara_motion_controller(port=args.port or bench.find_port(), timeout=120)

    best = np.inf
    for attempt in range(1, MAX_TRIES + 1):
        tip = correct_height([read_seen()], lens, surface, args.height)[0]
        error = float(np.hypot(*(target - tip)))
        base, elbow = bench.read_joints(scara)
        print(f"{attempt}: tip ({tip[0]:.2f}, {tip[1]:.2f}), {error:.2f} mm from the target; "
              f"base {base:.2f}, elbow {elbow:.2f}", flush=True)
        if error <= args.tolerance:
            state = bench.load_state()
            path = record(target, tip, base, elbow, state, args.height, error)
            print(f"On target: recorded in {path.relative_to(ROOT)}")
            return
        if error > best + WORSE_MM:
            sys.exit("Stopped: the error is growing.")
        best = min(best, error)
        new_base, new_elbow = step_joints(tip, target, base, elbow, kinematics, rig)
        bench.move(scara, "base", new_base, STEP_S)
        bench.move(scara, "elbow", new_elbow, STEP_S)
    sys.exit(f"Stopped: not within {args.tolerance:g} mm after {MAX_TRIES} tries.")


if __name__ == "__main__":
    main()
