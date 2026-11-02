"""Proves the arm can reach every corner of the pick plate and every bin, on the real rig.

    python scripts/reach_test.py teach     # servo power OFF: you put the tip on each point by hand
    python scripts/reach_test.py run       # servo power ON, after RESET: the firmware drives there

teach: for each point (the plate's corners, the middle of its edge nearest the base, its centre, the
eight bin slots) you hold the gripper tip over it and press Enter; the base and elbow
potentiometers are read (median of 50). Shows the rig's geometry lets the arm get there, whatever
the CAD says. Saved to logs/reach_points_<time>.json. Type s to skip a point.

run: SCARA_INITIALISE (course values), SCARA_AUTO_CALIBRATE, then to each taught pose in turn,
SCARA_MOVE_JOINT for the base and then the elbow, 3 s each. That command pulses only the joint it
moves, so the slide and wrist get no pulses after the RESET and stay limp where they are: the
gripper's height doesn't change, so set it clear of the plate and the bins first. At each point it
reads the potentiometers and asks whether the tip is over the point. Shows the motors get there.
Saved to logs/reach_test_<time>.csv, with a summary.

Run it with the camera tower OFF the rig. Its keep-out zone (robot_control/config/keep_out, from the
CAD) covers base 0..60 deg at mid elbow angles: the pose SCARA_AUTO_CALIBRATE ends in (base 0, elbow
90) is inside it, and so are bins A1 and A2. Moves here are straight lines in joint space.

Both print where each taught point is against the rig's CAD position (workspace.toml, robot_offline
dataset.toml), which are nominal and were never checked on the arm.
"""

import argparse
import contextlib
import csv
import io
import json
import math
import statistics
import sys
import time
from pathlib import Path

import serial.tools.list_ports

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "hardware_api"))

import scara_motion_controller_api as smc_api  # noqa: E402

ST_LINK_VID = 0x0483
L1, L2 = 0.125, 0.1
Z_MIN, Z_MAX = 0.095, 0.15
ELBOW_STOP = 141.1              # folded by hand after the elbow was refitted: 139.3 on the calibration then
                                # (logs/pot_check_20261102_171153.csv), 141.1 on the one from 18:01 (pots 662/2017)
ELBOW_MAX = ELBOW_STOP - 2.0    # never command closer to the stop than this
CALIBRATED_TO = 120.0           # SCARA_AUTO_CALIBRATE measures the elbow to here; past it, it extrapolates
SETTLE_S, SETTLE_PAST_S = 1.0, 0.2  # past CALIBRATED_TO, a short settle: it pushes the pulse with no limit
MOVE_S = 3.0
TOLERANCE_DEG = 1.0
SHORT_DEG = 2.0                 # past CALIBRATED_TO, this far short means the servo can't get there
BASE_MAX = 67.5                 # robot_real.toml: the base's travel ends at +69.5 (scripts/base_reach.py)

# Robot frame, metres: y out along the arm at base 0, x toward the base's + (clockwise) side, so -x
# is left and +x right looking out from behind the base. The rig scene frame (CAD mm) turned 180 deg
# (workspace.toml [scene_to_robot]): plate corners and crosshair from robot_offline's dataset.toml,
# bins from bins.toml.
POINTS = [
    ("plate far-left corner", (-0.050, 0.180)),
    ("plate far-right corner", (0.050, 0.180)),
    ("plate centre (crosshair)", (0.00048, 0.13037)),
    ("plate near-left corner (nearest the base, - side)", (-0.050, 0.080)),
    ("plate near edge, middle (the closest the plate comes to the base)", (0.0, 0.080)),
    ("plate near-right corner (nearest the base, + side)", (0.050, 0.080)),
    ("bin A1 (the slot furthest from the camera tower)", (0.130, -0.0625)),
    ("bin A2", (0.130, -0.0375)),
    ("bin A3", (0.130, -0.0125)),
    ("bin A4", (0.130, 0.0125)),
    ("bin B1", (0.130, 0.0425)),
    ("bin B2", (0.130, 0.0675)),
    ("bin B3", (0.130, 0.0925)),
    ("bin B4 (the slot beside the camera tower)", (0.130, 0.1175)),
]


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == ST_LINK_VID]
    if not ports:
        sys.exit("No Nucleo found: plug its USB cable in, or give --port.")
    return ports[0]


def fk(base, elbow):
    t1, t12 = math.radians(base), math.radians(base + elbow)
    return L1 * math.sin(t1) + L2 * math.sin(t12), L1 * math.cos(t1) + L2 * math.cos(t12)


def quiet(f, *a):
    with contextlib.redirect_stdout(io.StringIO()):     # the course API prints every command
        return f(*a)


def move(scara, axis, angle, seconds):
    """SCARA_MOVE_JOINT for the base (0) or elbow (1) only: the wrist (2) and slide (3) must never get
    a pulse, with the gripper on (they stay limp from the RESET)."""
    if axis not in (0, 1):
        raise ValueError(f"axis {axis}: this test only moves the base and elbow")
    quiet(scara.SCARA_MOVE_JOINT, axis, angle, seconds)


def read_joints(scara, n=50):
    base = statistics.median(quiet(scara.SCARA_READ_ANGLE, 0) for _ in range(n))
    elbow = statistics.median(quiet(scara.SCARA_READ_ANGLE, 1) for _ in range(n))
    return base, elbow


def vs_cad(name, base, elbow):
    cad = dict(POINTS)[name]
    x, y = fk(base, elbow)
    return f"{1000 * math.hypot(x - cad[0], y - cad[1]):.0f} mm from the CAD position"


def teach(scara, only_bins=False):
    input("Servo power OFF (the arm moves freely by hand)? Enter: ")
    points = {}
    for name, _ in POINTS:
        if only_bins and not name.startswith("bin"):
            continue
        if input(f"\nHold the tip over the {name}, then Enter (s to skip): ").strip().lower() == "s":
            continue
        base, elbow = read_joints(scara)
        points[name] = {"base": round(base, 2), "elbow": round(elbow, 2)}
        note = f"  base {base:6.1f}  elbow {elbow:6.1f} deg  ({vs_cad(name, base, elbow)})"
        if elbow > ELBOW_MAX:
            note += f"\n  past {ELBOW_MAX:.0f} deg: too near the elbow's stop to drive to"
        if base > BASE_MAX:
            note += f"\n  past {BASE_MAX:.1f} deg: further round than the base can drive (its travel ends at 69.5)"
        print(note)
    out = ROOT / "logs" / f"reach_points_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(points, indent=2))
    print(f"\nSaved {out.relative_to(ROOT)}. Next: servo power on, RESET, python scripts/reach_test.py run")


def run(scara, points_file):
    points = json.loads(points_file.read_text())
    print(f"{len(points)} taught points from {points_file.relative_to(ROOT)}")
    input("Camera tower OFF the rig, gripper raised clear of the plate and bins (the slide won't move), "
          "servo power on at 5 V, RESET pressed (switch on PROGRAM), hand on the plug? Enter: ")
    settle = SETTLE_S
    quiet(scara.SCARA_INITIALISE, L1, L2, Z_MIN, Z_MAX, settle, 1.3, 0.01, 0.001, 1.3, 0.01, 0.001)
    print("Calibrating: the base and elbow sweep end to end...")
    quiet(scara.SCARA_AUTO_CALIBRATE)

    out = ROOT / "logs" / f"reach_test_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    log = out.open("w", newline="")
    w = csv.writer(log)
    w.writerow(["point", "taught_base", "taught_elbow", "got_base", "got_elbow", "off_deg", "tip_over_point"])
    rows = []

    def record(*row):           # written as it goes, so a run stopped part way still leaves its results
        rows.append(row)
        w.writerow(row)
        log.flush()

    for name, taught in points.items():
        print(f"\n{name}: base {taught['base']:.1f}, elbow {taught['elbow']:.1f}")
        if taught["elbow"] > ELBOW_MAX:
            print(f"  skipped: elbow past {ELBOW_MAX:.0f} deg, too near the stop")
            record(name, taught["base"], taught["elbow"], "", "", "", "skipped: near the stop")
            continue
        if taught["base"] > BASE_MAX:
            print(f"  skipped: base past {BASE_MAX:.1f} deg, further round than it can drive")
            record(name, taught["base"], taught["elbow"], "", "", "", "skipped: past the base's travel")
            continue
        past = taught["elbow"] > CALIBRATED_TO
        # Settling per joint (SCARA_INITIALISE only sets parameters). The base always gets the full 1 s: its
        # pulse map falls ~8 deg short by 60 (scripts/base_reach.py) and settling makes that up, and every
        # target is within its travel (BASE_MAX). Only the elbow past CALIBRATED_TO gets the short one.
        for axis, angle, s in ((0, taught["base"], SETTLE_S), (1, taught["elbow"], SETTLE_PAST_S if past else SETTLE_S)):
            if s != settle:
                settle = s
                quiet(scara.SCARA_INITIALISE, L1, L2, Z_MIN, Z_MAX, settle, 1.3, 0.01, 0.001, 1.3, 0.01, 0.001)
            move(scara, axis, angle, MOVE_S)
        base, elbow = read_joints(scara)
        if past and taught["elbow"] - elbow > SHORT_DEG:
            print(f"  elbow stopped at {elbow:.1f}: backing off to {CALIBRATED_TO - 5:.0f} so the servo isn't left straining")
            move(scara, 1, CALIBRATED_TO - 5, 1.0)
        err = max(abs(base - taught["base"]), abs(elbow - taught["elbow"]))
        print(f"  got base {base:.1f}, elbow {elbow:.1f}: off {err:.1f} deg  "
              f"({'OK' if err <= TOLERANCE_DEG else 'MISSED'})")
        seen = input("  Is the tip over the point? y/n (or a comment): ").strip()
        record(name, taught["base"], taught["elbow"], f"{base:.2f}", f"{elbow:.2f}", f"{err:.2f}", seen)
    log.close()

    print("\nSummary")
    for name, *_, off, seen in rows:
        verdict = ("PASS" if off and float(off) <= TOLERANCE_DEG and seen.lower().startswith("y")
                   else "check" if off else "not tested")
        print(f"  {verdict:10s} {name}  ({off or '-'} deg; {seen or '-'})")
    print(f"Saved {out.relative_to(ROOT)}. RESET (or unplug servo power) to let the arm go limp.")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["teach", "run"])
    p.add_argument("--bins", action="store_true", help="teach: only the eight bin slots")
    p.add_argument("--points", help="taught points for run (default: the newest logs/reach_points_*.json)")
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    args = p.parse_args()

    scara = smc_api.scara_motion_controller(port=args.port or find_port(), timeout=60)
    if args.mode == "teach":
        teach(scara, only_bins=args.bins)
        return
    if args.points:
        points_file = Path(args.points).resolve()
    else:
        found = sorted((ROOT / "logs").glob("reach_points_*.json"))
        if not found:
            sys.exit("No taught points: run python scripts/reach_test.py teach first.")
        points_file = found[-1]
    run(scara, points_file)


if __name__ == "__main__":
    main()
