"""How far clockwise the base gets under power: steps it toward bin A1 (taught at base 75.4, elbow 91.7)
and reads the potentiometer after each step.

    python scripts/base_reach.py
    python scripts/base_reach.py --to 78 --port COM8

After a RESET: SCARA_INITIALISE (course values, settling 0.2 s), SCARA_AUTO_CALIBRATE, the elbow to
91.7, the base to 60, then the base toward --to (75.4). Only the base and elbow are moved
(SCARA_MOVE_JOINT): the slide and wrist get no pulses and stay limp, so the gripper's height doesn't
change. Writes logs/base_reach_<time>.csv.

The base's pulse map past 0 falls short: SCARA_AUTO_CALIBRATE looks for the pot at +90, which the base
doesn't reach, so it keeps the default 500 us there, and lands about 8 deg short at 60 (2026-11-02). So
it commands past the target by what it fell short, and again, until it lands within 0.5 deg. Commands
never go past 90, which keeps the pulse within 500-2500 us whatever the map. If a bigger command no
longer moves the base, that's the end of its travel: it backs off to 60 and stops. Settling is 0.2 s,
not the course's 1 s: past the servo's end, settling would keep pushing the pulse with no limit.
"""

import argparse
import contextlib
import csv
import io
import statistics
import sys
import time
from pathlib import Path

import serial.tools.list_ports

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "hardware_api"))

import scara_motion_controller_api as smc_api  # noqa: E402

ST_LINK_VID = 0x0483
ELBOW = 91.7                    # bin A1, logs/reach_points_20261102_180907_recal.json
START = 60.0
COMMAND_MAX = 90.0              # the map's end: 500 us at most, however short the map falls
LANDED_DEG = 0.5
STUCK_DEG = 0.3                 # a command 1 deg or more bigger moving the base less than this: travel's end
TRIES = 8


def find_port():
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == ST_LINK_VID]
    if not ports:
        sys.exit("No Nucleo found: plug its USB cable in, or give --port.")
    return ports[0]


def quiet(f, *a):
    with contextlib.redirect_stdout(io.StringIO()):     # the course API prints every command
        return f(*a)


def read(scara, axis, n=30):
    return statistics.median(quiet(scara.SCARA_READ_ANGLE, axis) for _ in range(n))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--to", type=float, default=75.4, help="the base angle to step to (75.4, bin A1)")
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    args = p.parse_args()

    scara = smc_api.scara_motion_controller(port=args.port or find_port(), timeout=60)
    _, _, angle_z, z = quiet(scara.SCARA_READ_COORD)
    if abs(z - 0.128) > 0.001:
        sys.exit("The board hasn't been reset since a move: press RESET (switch on PROGRAM) and run again.")
    input("Gripper raised clear of the bins (the slide won't move), servo power on at 5 V, "
          "hand on the plug? Enter: ")
    quiet(scara.SCARA_INITIALISE, 0.125, 0.1, 0.095, 0.15, 0.2, 1.3, 0.01, 0.001, 1.3, 0.01, 0.001)
    print("Calibrating: the base and elbow sweep end to end...")
    quiet(scara.SCARA_AUTO_CALIBRATE)
    quiet(scara.SCARA_MOVE_JOINT, 1, ELBOW, 2.0)
    quiet(scara.SCARA_MOVE_JOINT, 0, START, 3.0)

    out = ROOT / "logs" / f"base_reach_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    rows, command, before = [], START, None
    for _ in range(TRIES):
        got = read(scara, 0)
        rows.append((args.to, round(command, 2), round(got, 2)))
        print(f"  commanded {command:5.1f}  base at {got:5.1f}  ({got - args.to:+.1f} from {args.to:.1f})")
        if abs(got - args.to) <= LANDED_DEG:
            print(f"Reached {got:.1f} for {args.to:.1f} (commanded {command:.1f}: the map falls {command - got:.1f} short there).")
            break
        if before is not None and command - before[0] >= 1.0 and got - before[1] < STUCK_DEG:
            print(f"A bigger command no longer moves it: the base's travel ends at about {got:.1f}. Backing off to {START:.0f}.")
            quiet(scara.SCARA_MOVE_JOINT, 0, START, 2.0)
            break
        if command >= COMMAND_MAX:
            print(f"Commanded {COMMAND_MAX:.0f}, the map's end, and the base is at {got:.1f}: it gets no further "
                  f"within 500-2500 us. Backing off to {START:.0f}.")
            quiet(scara.SCARA_MOVE_JOINT, 0, START, 2.0)
            break
        before = (command, got)
        command = min(COMMAND_MAX, command + (args.to - got))
        quiet(scara.SCARA_MOVE_JOINT, 0, command, 2.0)
    else:
        print(f"Not within {LANDED_DEG} deg after {TRIES} tries.")

    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["target", "commanded", "base_got"])
        w.writerows(rows)
    print(f"Saved {out.relative_to(ROOT)}. RESET (or unplug servo power) to let the arm go limp.")


if __name__ == "__main__":
    main()
