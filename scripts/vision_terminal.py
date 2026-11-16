"""The vision terminal: how well segmentation and band reading do, live, resistor by resistor.

    python scripts/vision_terminal.py                  # the camera, the arm and the gripper
    python scripts/vision_terminal.py --camera-only    # the camera alone: no CALIBRATE or PICK
    python scripts/vision_terminal.py --bin A1         # PICK drops in A1 (default B4)
    python scripts/vision_terminal.py --frames logs/clear_20261112_205808   # no rig: saved pictures, 30 s each

The plate is shown live, every frame segmented and every body's bands read from that frame: those readings
are PROVISIONAL. Click a resistor to make it the subject. Buttons (robot_control/task/vision_terminal.py):

    CALIBRATE   the standard calibration (as sort_resistors_auto.py): on an EMPTY plate, after the arm has
                been set by hand (servo supply off: slide fully up, elbow folded 100+ deg, gripper turned
                from the tower; supply on).
    SEGMENT     the sorter's own look (focus checked, 8 frames averaged), drawn in white over the live view.
    CLASSIFY    the subject read properly: the focus checked, an averaged picture and a close-up, combined.
    PICK        the subject picked and dropped in --bin, then the arm parked. Needs the arm started.
    q / Esc     quit.

Keep a hand on the servo supply's plug whenever the arm moves. Nothing else may be using the camera.
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav  # noqa: E402
import sort_resistors  # noqa: E402

from robot_control.task.run_log import RunLog  # noqa: E402
from robot_control.task.vision_terminal import SavedFrames, TerminalWindow, VisionTerminal  # noqa: E402
from robot_control.vision import eyes as eyes_module  # noqa: E402
from robot_control.vision.eyes import Eyes  # noqa: E402

SAVED_S = 30.0              # each saved picture's turn with --frames: time to click, SEGMENT and CLASSIFY


def saved_eyes(folder):
    """Eyes on saved pictures: the plate's tracker with no exposure to control, no focus check, no close-ups."""
    paths = sorted((p for p in folder.glob("*.jpg") if p.stem != "grid"), key=lambda p: (p.stem == "final", p.stem))
    if not paths:
        sys.exit(f"no .jpg pictures in {folder}")
    print(f"Playing {len(paths)} saved pictures from {folder}, {SAVED_S:.0f} s each", flush=True)
    eyes_module.focus_api = None
    eyes = Eyes(cap=SavedFrames(paths, SAVED_S), closeups=False)
    eyes.tracker.control = None                 # nothing to set: found and settled at once
    return eyes


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--camera-only", action="store_true", help="no arm or gripper: SEGMENT and CLASSIFY only")
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    p.add_argument("--bin", default="B4", type=str.upper, help="the bin PICK drops in (default B4)")
    p.add_argument("--cad", action="store_true", help="aim by the CAD geometry instead of camera_calibration.json")
    p.add_argument("--no-closeups", action="store_true", help="CLASSIFY from the averaged picture alone")
    p.add_argument("--frames", metavar="FOLDER",
                   help="no rig: play the .jpg pictures of the plate in FOLDER (full camera frames, as a run's look_*.jpg) "
                        "in place of the camera, 30 s each; no arm, no focus check, no close-ups")
    args = p.parse_args()
    slots = resistor_nav.load_bins()
    if args.bin not in slots:
        sys.exit(f"no bin {args.bin}: bins_real.toml has {', '.join(slots)}")
    arm = None
    if not (args.camera_only or args.frames):
        arm, _ = sort_resistors.build_arm(args.port, slots, camera_calibration=not args.cad)
    if args.frames:
        eyes = saved_eyes(Path(args.frames))
    else:
        eyes = Eyes(closeups=not args.no_closeups)
    window = TerminalWindow()
    eyes.say = window.say
    log = RunLog()
    print(f"Logging to {log.folder}", flush=True)
    sort_resistors.run_window(VisionTerminal(eyes, window, arm, log, args.bin), eyes, window)


if __name__ == "__main__":
    main()
