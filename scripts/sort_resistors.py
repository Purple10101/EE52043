"""The resistor sorting program: one window to calibrate the arm, check the plan and sort the plate.

    python scripts/sort_resistors.py                 # the arm, the gripper and the camera
    python scripts/sort_resistors.py --look-only     # the camera alone: LOOK only, the arm never moves

The window (robot_control/task/window.py) shows the plate at the last look, what becomes of each
resistor, the TRAYS: which value each bin holds and how many have gone in, and WHERE THE BINS ARE: the
rig from above with each slot labelled (robot_control/task/rig_map.py; the picture is the digital
twin's, made by scripts/twin/render_top_view.py and kept out of git). Its buttons
(robot_control/task/app.py):

    CALIBRATE ARM   after the Nucleo is powered on: start, park, elbow check. Press twice: the first
                    press says what to set by hand first.
    LOOK            one look and its plan, nothing moves.
    START / STOP    the sorting run; STOP takes effect once the arm is parked again.

This file only builds the parts and hands them over:

    Eyes        robot_control/vision/eyes.py        the camera: resistors, where they lie, their values
    Planner     robot_control/planning/planner.py   which resistor next; BinPlan: which bin a value goes in;
                                                    PickMemory: how the last pick went, what to leave alone
    Arm         robot_control/motion/arm.py         start up, park, pick, drop (resistor_nav's guarded moves)
    Supervisor  robot_control/task/supervisor.py    the cycle: look, judge, plan, pick, drop, park, again
    SortingApp  robot_control/task/app.py           the buttons, each running one job in the background
    RunLog      robot_control/task/run_log.py       logs/sort_runs/<time>/

The sorting scheme: the first time a value is sorted it takes the next free tray slot (A1, A2, ... in
bins_real.toml's order, all eight) and keeps it for as long as the program runs. A resistor stays on
the plate if its value isn't read with confidence, is outside 10 ohm to 10 kohm, or has no bin left,
or if the arm has missed it twice or couldn't reach it.

Keep a hand on the servo supply's plug while the arm moves: unplugging it is the emergency stop.
Nothing else may be using the camera.
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav  # noqa: E402
import slide_wrist as bench  # noqa: E402
from robot_offline.post.bands.values import SORTER_OHM  # noqa: E402

from robot_control.config import ArmSettings  # noqa: E402
from robot_control.gripper import Gripper, GripperError  # noqa: E402
from robot_control.hardware.arm_monitor import ArmMonitor  # noqa: E402
from robot_control.kinematics.arm import Kinematics  # noqa: E402
from robot_control.motion.arm import Arm  # noqa: E402
from robot_control.planning.bins import BinPlan  # noqa: E402
from robot_control.planning.memory import PickMemory  # noqa: E402
from robot_control.planning.planner import Planner  # noqa: E402
from robot_control.task.app import SortingApp  # noqa: E402
from robot_control.task.rig_map import RigMap  # noqa: E402
from robot_control.task.run_log import RunLog  # noqa: E402
from robot_control.task.supervisor import Supervisor  # noqa: E402
from robot_control.task.window import RunWindow  # noqa: E402
from robot_control.config import RigSettings  # noqa: E402
from robot_control.vision import pick_fit  # noqa: E402
from robot_control.vision.arm_fit import JointCorrection, load_calibration  # noqa: E402
from robot_control.vision.eyes import Eyes  # noqa: E402


def build_arm(port, slots, camera_calibration=True, need_gripper=True):
    """(Arm, ArmMonitor): the Nucleo and the gripper board, every command passing through the monitor so the
    window can show the arm moving. The arm aims by the camera calibration (config/camera_calibration.json, from
    scripts/camera_calibrate.py, kept up to date by CHECK AIM). On 2026-11-12 its check put the jaws within
    0.9 mm rms of where it aimed, where the CAD geometry with the taught pick calibration was 4.5 mm off and
    missed a resistor twice. camera_calibration=False aims by the CAD geometry instead: the rig from
    workspace.toml, the potentiometers' angles as they are, the gripper a line straight down from the slide's
    shaft, corrected by the taught pick calibration (config/pick_calibration.json, scripts/teach_picks.py) if
    there is one."""
    try:
        gripper = Gripper()
    except (GripperError, OSError) as why:
        if need_gripper:
            sys.exit(f"The gripper board isn't ready ({why}): plug it in, or run with --look-only.")
        print(f"no gripper board ({why}): the jaws stay as they are", flush=True)
        gripper = None
    scara = bench.smc_api.scara_motion_controller(port=port or bench.find_port(), timeout=120)
    monitor = ArmMonitor(scara, gripper)
    state = bench.load_state() or {}
    monitor.know(wrist=state.get("wrist"), slide=state.get("slide"))
    bench.read_joints(monitor.nucleo, bench.QUICK_READS)       # base and elbow, from their potentiometers
    taught = None if camera_calibration else pick_fit.load()
    if camera_calibration:
        rig, correction = load_calibration()
    else:
        rig, correction = (taught.rig, taught.correction) if taught else (RigSettings.from_config(), JointCorrection())
    navigator = resistor_nav.Navigator(monitor.nucleo, Kinematics(ArmSettings.from_config()), rig, correction,
                                       monitor.gripper)
    navigator.geometry = "camera" if camera_calibration else "taught" if taught else "cad"
    if taught:
        navigator.jaw_offset = taught.jaw_offset
    monitor.watch_holding(lambda: navigator.holding)
    return Arm(navigator, slots, bench), monitor


def build(port=None, look_only=False, closeups=True, buttons=None, camera_calibration=True, unclassified_bin=None):
    """(Supervisor, RunWindow, Eyes): every part of the sorting program, built and joined up. Both this
    window and sort_resistors_auto.py's use it. buttons: the window's button bar (None: this window's).
    unclassified_bin: a bin id for resistors whose value couldn't be read or is outside the sorter's range,
    or None to leave them on the plate."""
    slots = resistor_nav.load_bins()
    try:
        bins = BinPlan.from_slots(slots, SORTER_OHM, unclassified=unclassified_bin)
    except ValueError as why:
        sys.exit(str(why))
    planner = Planner(bins, PickMemory())
    arm, monitor = (None, None) if look_only else build_arm(port, slots, camera_calibration)
    eyes = Eyes(closeups=closeups)
    log = RunLog()
    print(f"Logging to {log.folder}", flush=True)
    extra = {} if buttons is None else {"buttons": buttons}
    window = RunWindow(rig_map=RigMap.load(slots), monitor=monitor, **extra)
    supervisor = Supervisor(eyes, arm, planner, log, window)
    eyes.say = supervisor.say           # close-ups take a while: say so in the window
    return supervisor, window, eyes


def run_window(app, eyes, window):
    """Runs an app's window until it is closed, then lets go of the camera and the window."""
    try:
        app.run()
    except KeyboardInterrupt:
        print("Interrupted: the arm stays where it is. Park it with scripts/slide_wrist.py park.")
    finally:
        eyes.close()
        window.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--look-only", action="store_true", help="the camera alone: LOOK only, the arm never moves")
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    p.add_argument("--no-closeups", action="store_true",
                   help="read values from the plate-wide picture alone: quicker first look, less sure readings")
    p.add_argument("--cad", action="store_true",
                   help="aim by the arm's CAD geometry (and pick_calibration.json if there is one) instead of "
                        "camera_calibration.json (the default)")
    args = p.parse_args()
    supervisor, window, eyes = build(args.port, args.look_only, not args.no_closeups,
                                     camera_calibration=not args.cad)
    run_window(SortingApp(supervisor, window), eyes, window)


if __name__ == "__main__":
    main()
