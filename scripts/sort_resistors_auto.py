"""Sorts the resistors on the plate with the procedure fixed: CALIBRATE on an empty plate, then START.

    python scripts/sort_resistors_auto.py
    python scripts/sort_resistors_auto.py --unclassified-bin B4    # unread and out-of-range resistors into B4
    python scripts/sort_resistors_auto.py --no-ohmmeter            # ignore the ohmmeter (config/ohmmeter.toml)
    python scripts/sort_resistors_auto.py --skip-calibration       # START straight away: arm already started

The same window as scripts/sort_resistors.py (the plate, the trays, the rig with the arm moving), with two
buttons instead of five, and the steps always done in the same order (robot_control/task/auto_app.py):

    CALIBRATE   on an EMPTY plate. With the ohmmeter on, first checks it answers and reads open (its pads clear).
                Then starts the arm (start sequence, elbow check), finds the plate and checks it is empty, then
                checks the arm's aim (5 points, the steady shift taken out). Each step must pass, or it says why
                and what to do. The arm aims by camera_calibration.json; with --cad it aims by
                its CAD geometry instead, and CALIBRATE skips the aim check.
    START       on a FULL plate, once CALIBRATE has passed: sorts until nothing is left that it can sort. With the
                ohmmeter on (config/ohmmeter.toml), a resistor the band reader isn't sure of is picked, its leads
                pressed on the meter's pads (config/ohmmeter_station.toml), and binned by the meter's value; one
                the meter can't place goes in the reject bin (B4).
    STOP        ends a run once the arm is parked again.

Before CALIBRATE, after a power-on: with the servo supply off, slide fully up by hand, elbow folded to 100
degrees or more, gripper turned away from the camera tower; then supply on. Keep a hand on the servo
supply's plug whenever the arm moves: unplugging it is the emergency stop.
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import sort_resistors  # noqa: E402

from robot_control.task.auto_app import BUTTONS, AutoSortingApp  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    p.add_argument("--cad", action="store_true",
                   help="aim by the arm's CAD geometry (and pick_calibration.json if there is one) instead of "
                        "camera_calibration.json (the default)")
    p.add_argument("--unclassified-bin", metavar="BIN", type=str.upper,
                   help="pick resistors whose value couldn't be read, or is outside the sorter's range, into this "
                        "bin (e.g. B4) instead of leaving them on the plate; that bin is never given a value")
    p.add_argument("--no-ohmmeter", action="store_true",
                   help="don't use the ohmmeter even if config/ohmmeter.toml turns it on")
    p.add_argument("--skip-calibration", action="store_true",
                   help="offer START straight away, without a CALIBRATE: only when the arm has been started since "
                        "the Nucleo's last power-on (slide_wrist.py start, or CALIBRATE in an earlier window) and "
                        "the rig hasn't moved since the aim was checked")
    args = p.parse_args()
    supervisor, window, eyes = sort_resistors.build(args.port, buttons=BUTTONS, camera_calibration=not args.cad,
                                                    unclassified_bin=args.unclassified_bin,
                                                    use_ohmmeter=not args.no_ohmmeter)
    sort_resistors.run_window(AutoSortingApp(supervisor, window, args.skip_calibration), eyes, window)


if __name__ == "__main__":
    main()
