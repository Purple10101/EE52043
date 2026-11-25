"""Live view of the gripper's tip as the camera finds it, for checking find_tip by hand.

robot_offline's live_plate window (the fitted plate beside the rectified plate, exposure following
the light), with a green cross where find_tip puts the tip and its distance from the crosshair, or
why it found none. Nothing here drives the arm.

    python scripts/tip_view.py                      # tip touching the plate, at the crosshair
    python scripts/tip_view.py --height 20          # tip this many mm above it
    python scripts/tip_view.py --near 42 -172       # looking around another point (scene mm)
    python scripts/tip_view.py --snapshot           # also keeps logs/tips/live.jpg and live.txt up to date
    python scripts/tip_view.py --lock --follow      # hold the first fit; look around the last tip found
    python scripts/tip_view.py --calibration logs/tips/tip_..._rectify.json --lock   # hold a saved fit from the start
                                                    # (the arm may cover the crosshair)

Writing a label to logs/tips/save.txt saves the next raw frame and the fit, as s does, from another program.

Start with the arm off the plate and press l once the fit looks right, so the arm can't disturb it.

The first frame once the plate is fitted is kept as the empty-plate reference, so start with the arm parked: the
gripper is then found as what changed from it, whatever the light (vision/jaws.py). b takes it again.

Keys:  q / Esc  quit      l  lock the fit, or refit      z  whole camera frame or zoomed on the plate
       s  save the raw frame and the calibration to logs/tips      b  take the reference again (arm parked)
"""

import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from robot_offline import config  # noqa: E402
from robot_offline.pre import rectify  # noqa: E402
from robot_offline.pre.camera import label, open_camera, show  # noqa: E402
from robot_offline.pre import plate_search  # noqa: E402
from robot_offline.pre.live_plate import camera_view, rectified_view, status_lines  # noqa: E402

from robot_control.vision.jaws import find_jaws  # noqa: E402
from robot_control.vision.tip import TipNotFound, configured_finder, find_tip  # noqa: E402

WINDOW = "tip view"
MARK_PX = 90                # the cross's size in the raw frame
OUT = ROOT / "logs" / "tips"
NEAR = OUT / "near.txt"     # "x y", scene mm: where to look for the tip from now on
SAVE = OUT / "save.txt"     # a label: written from outside, it saves the next raw frame as s does, named with it
SNAPSHOT_S = 1.0           # how often --snapshot writes the view, for watching from outside this window


def snapshot(view, lines):
    """The window's view and text, each replaced whole so a reader never gets half of one."""
    OUT.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUT / "live.tmp.jpg"), cv2.resize(view, None, fx=0.6, fy=0.6, interpolation=cv2.INTER_AREA))
    (OUT / "live.tmp.txt").write_text("\n".join([time.strftime("%H:%M:%S")] + lines) + "\n", encoding="utf-8")
    try:
        os.replace(OUT / "live.tmp.jpg", OUT / "live.jpg")
        os.replace(OUT / "live.tmp.txt", OUT / "live.txt")
    except PermissionError:     # Windows: someone is reading it, so this one is skipped
        pass


def mark(frame, tip_mm, cal):
    """A copy of frame with a green cross on the tip, on the plate's surface."""
    marked = frame.copy()
    (x, y), = rectify.to_full(rectify.mm_to_rect([tip_mm], cal.settings), cal)
    cv2.drawMarker(marked, (int(round(x)), int(round(y))), (0, 255, 0), cv2.MARKER_CROSS, MARK_PX, 3, cv2.LINE_AA)
    return marked


def main():
    parser = argparse.ArgumentParser(description="Live view of the gripper's tip.")
    parser.add_argument("--height", type=float, default=0.0, help="the tip's height above the plate, mm")
    parser.add_argument("--near", type=float, nargs=2, metavar=("X", "Y"),
                        help="where the tip should be, scene mm (default the crosshair)")
    parser.add_argument("--snapshot", action="store_true", help="keep logs/tips/live.jpg and live.txt up to date")
    parser.add_argument("--lock", action="store_true", help="hold the first fit, as l does: start with the arm parked")
    parser.add_argument("--follow", action="store_true", help="look around the last tip found, not one fixed point")
    parser.add_argument("--calibration", help="a saved calibration to refit near: more reliable than from scratch")
    args = parser.parse_args()

    print("Opening the camera (a few seconds while exposure and white balance settle)...")
    camera = config.load("camera")
    cap = open_camera(camera)
    settings = rectify.Settings.from_config()
    near = args.near or settings.cross_mm
    given = rectify.Calibration.load(args.calibration) if args.calibration else None
    fitter = plate_search.Tracker(cap, settings, cal=given, camera=camera, refit_seconds=0.0,
                                  locked=args.lock and given is not None)
    print(__doc__.split("Keys:", 1)[1].rstrip())

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.moveWindow(WINDOW, 40, 40)      # else it can open off the screen
    jaws = configured_finder() == "jaws"     # gripper v5: the jaw line is reported too, for keeping it face-on
    zoom, shown, snapped, told = True, None, 0.0, 0.0
    reference = None            # the plate with the arm parked: taken from the first fitted frame, b again
    asked = SAVE.stat().st_mtime if SAVE.exists() else 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        if NEAR.exists() and NEAR.stat().st_mtime != told:      # a calibration run saying where to look next
            told = NEAR.stat().st_mtime
            near = tuple(float(v) for v in NEAR.read_text().split())
        cal = fitter.update(frame)
        if args.lock and fitter.fitted_at and fitter.error is None:
            fitter.locked = True

        marked, line = frame, "looking for the plate..."
        if cal is not None and reference is None:
            reference = frame.copy()
            print("Reference taken: the plate as it is now (the arm must be parked). b takes it again.", flush=True)
        if cal is not None:
            try:
                extra = ""
                if jaws:
                    found = find_jaws(frame, cal, near, args.height, reference=reference)
                    tip = found.tip_mm
                    extra = (f", jaw {found.jaw_deg:.2f} deg, corners {found.spacing_mm:.2f} mm apart, "
                             f"centre ({found.centre_mm[0]:.2f}, {found.centre_mm[1]:.2f}) mm")
                else:
                    tip = find_tip(frame, cal, near, args.height)
                if args.follow:
                    near = tuple(tip)
                # Marked where it's seen on the plate, which is the tip itself only at height 0.
                marked = mark(frame, tip, cal)
                if jaws:
                    marks = [(q, (0, 0, 255)) for q in found.corners_px] + [(q, (255, 0, 255)) for q in found.apexes_px]
                    for q, colour in marks:
                        cv2.drawMarker(marked, tuple(np.round(q).astype(int)), colour, cv2.MARKER_CROSS, 40, 3, cv2.LINE_AA)
                dx, dy = tip - cal.settings.cross_mm
                line = (f"tip ({tip[0]:.2f}, {tip[1]:.2f}) mm, {np.hypot(dx, dy):.2f} mm from the crosshair "
                        f"({dx:+.2f}, {dy:+.2f}){extra}")
            except TipNotFound as e:
                line = f"no tip: {e}"

        left = camera_view(marked, cal, zoom)
        right = rectified_view(marked, cal, True) if cal is not None else np.zeros_like(left[:, :left.shape[0]])
        lines = [line] + status_lines(fitter)[:1] + [fitter.control.status()]
        label(left, lines + ["q quit  l lock  z zoom  s save  b reference"])
        view = np.hstack([left, right])
        shown = show(view, shown, WINDOW)
        if args.snapshot and time.time() - snapped > SNAPSHOT_S:
            snapshot(view, lines)
            snapped = time.time()

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord("l"):
            fitter.locked = not fitter.locked
        elif key == ord("z"):
            zoom = not zoom
        elif key == ord("b") and cal is not None:
            reference = frame.copy()
            print("Reference taken again.", flush=True)
        label_asked = None
        if SAVE.exists() and SAVE.stat().st_mtime != asked:
            asked = SAVE.stat().st_mtime
            label_asked = "".join(c if c.isalnum() or c in "-_." else "_" for c in SAVE.read_text().strip())
        if (key == ord("s") or label_asked is not None) and cal is not None:
            OUT.mkdir(parents=True, exist_ok=True)
            stem = OUT / (time.strftime("tip_%Y%m%d_%H%M%S") + (f"_{label_asked}" if label_asked else ""))
            cv2.imwrite(f"{stem}.png", frame)
            cal.save(f"{stem}_rectify.json")
            print(f"Saved {stem}.png: {line}", flush=True)

    fitter.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
