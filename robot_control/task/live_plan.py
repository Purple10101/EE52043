"""Live plan: the front end of the cycle run on the camera, a few times a second, in a window.
Nothing is sent to the arm.

Like robot_offline's live_detect (same camera settings, background plate fitting and exposure
control), but each detected body is mapped to the robot frame and checked against the arm's limits.
The view shows the arm's reach limit (blue: closer to the base than this the elbow would pass 120
deg) and each body numbered, green if the arm can reach it, red if not, the next pick ringed in
white. The panel on the right lists each body's robot-frame centre, yaw and size, then the joints
and the ScaraController call that would put the open gripper above it, or why it is skipped.

    python -m robot_control.task.live_plan
    python -m robot_control.task.live_plan --frames path\\to\\saved\\frames      # without the rig

Keys:  q / Esc  quit      v  top-down / camera view      g  grid      + / -  confidence threshold
       c  find the plate again      e  exposure following the light, or fixed      n  next saved frame
       s  save the view, the raw frame and the plan (logs/live_plan/)
"""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from robot_control.config import GripperSettings, RigSettings, VisionSettings, WorkspaceSettings
from robot_control.task.plan_picks import BAD_COLOUR, OK_COLOUR, draw_plan, kinematics, plan, text, to_json

WINDOW = "robot_control live plan"
CAMERA_SCALE = 0.4          # the camera view is shown at this share of 4608 x 2592
REFIND_SECONDS = 5.0
PANEL_WIDTH = 560
ROW = 44                    # px per body in the panel


def panel(planned, workspace, height):
    """The plan as text, one two-line row per body, numbered as on the view."""
    out = np.full((height, PANEL_WIDTH, 3), 24, np.uint8)
    text(out, "robot frame (mm, deg)", (10, 22), (220, 220, 220), 0.55)
    rows = (height - 40) // ROW
    for i, (d, r) in enumerate(planned):
        if i >= rows:
            text(out, f"+{len(planned) - i} more", (10, height - 10), (220, 220, 220))
            break
        y = 50 + i * ROW
        colour = OK_COLOUR if r.ok else BAD_COLOUR
        size = f"{d.length * 1000:.1f} x {d.width * 1000:.1f}" + ("" if d.plausible else " ?")
        text(out, f"#{i + 1}  {d.score:.2f}   x {d.centre[0] * 1000:6.1f}  y {d.centre[1] * 1000:6.1f}   "
                  f"yaw {d.yaw:5.1f}   {size}", (10, y), colour)
        if r.ok:
            j = r.joints
            second = (f"base {j.j0:5.1f} elbow {j.j1:5.1f} wrist {j.j2:6.1f}   move_coordinates({d.centre[0]:.4f}, "
                      f"{d.centre[1]:.4f}, {r.tool_angle:.1f}, {workspace.hover_z:.3f}, {workspace.move_time:.1f})")
        else:
            second = f"skip: {r.reason}"
        text(out, second, (28, y + 18), (200, 200, 200), 0.42)
    return out


def render(frame, cal, balanced, planned, kin, rig, workspace, top_down=True, grid=True):
    """The window's picture, without the status lines: the top-down view (balanced: the rectified,
    white-balanced frame) or the camera's, with the plan drawn on, and the panel beside it."""
    from robot_offline.pre import rectify
    if cal is not None and top_down:
        view = rectify.draw_grid(balanced, cal.settings) if grid else balanced.copy()
        draw_plan(view, planned, cal, kin, rig)
    else:
        view = cv2.resize(frame, None, fx=CAMERA_SCALE, fy=CAMERA_SCALE, interpolation=cv2.INTER_AREA)
        if cal is not None:
            cv2.polylines(view, [np.round(rectify.plate_outline_px(cal) * CAMERA_SCALE).astype(np.int32)],
                          True, (255, 255, 255), 1, cv2.LINE_AA)
            draw_plan(view, planned, cal, kin, rig, lambda q: rectify.to_full(q, cal) * CAMERA_SCALE, 0.6)
    return np.hstack([view, panel(planned, workspace, view.shape[0])])


class Frames:
    """The camera, saved frames shown one at a time (n for the next), or the simulator's twin camera."""

    def __init__(self, path=None, twin=False):
        from robot_offline import config as offline_config
        self.camera_cfg = offline_config.load("camera")
        self.cap, self.fixed, self.twin = None, None, None
        if twin:
            from robot_control.vision.camera import TwinCamera
            self.twin = TwinCamera()
            self.follow = False
        elif path is None:
            from robot_offline.pre.camera import open_camera
            print("Opening the camera (a few seconds while exposure and white balance settle)...")
            self.cap = open_camera(self.camera_cfg)
            c = self.camera_cfg["camera"]
            self.fixed = c["exposure"], c["gain"]
        else:
            from robot_control.vision.camera import ReplayCamera
            self.paths = ReplayCamera(path).paths
            self.index, self.current = 0, None
        from robot_offline.pre import plate_search
        # The plate, found and kept as every tool does; on the live camera it sets the exposure too.
        self.tracker = plate_search.Tracker(self.cap, camera=self.camera_cfg, refit_seconds=REFIND_SECONDS)

    @property
    def live(self):
        return self.cap is not None

    def read(self):
        """(id, newest frame) or None. Pass the frame to self.tracker.update."""
        if self.twin is not None:
            return self.twin.grab()
        if not self.live:
            if self.current is None:            # one frame in memory at a time: they are 36 MB each
                path = self.paths[self.index]
                frame = cv2.imread(str(path))
                if frame is None:
                    raise OSError(f"cannot read {path}")
                self.current = path.stem, frame
            return self.current
        ok, frame = self.cap.read()
        if not ok:
            return None
        return time.strftime("%Y%m%d-%H%M%S"), frame

    def next(self):
        if not self.live and self.twin is None:
            self.index, self.current = (self.index + 1) % len(self.paths), None

    def toggle_exposure(self):
        if not self.live:
            return
        self.tracker.follow = not self.tracker.follow
        if not self.tracker.follow:
            self.cap.set(cv2.CAP_PROP_EXPOSURE, self.fixed[0])
            self.cap.set(cv2.CAP_PROP_GAIN, self.fixed[1])

    def status(self):
        if self.twin is not None:
            return "simulator twin camera"
        if not self.live:
            return f"saved frame {self.index + 1} of {len(self.paths)}: {self.paths[self.index].stem}   (n next)"
        if self.tracker.follow:
            return self.tracker.control.status()
        return f"exposure fixed at {self.fixed[0]}, gain {self.fixed[1]}"

    def close(self):
        self.tracker.close()
        if self.live:
            self.cap.release()
        if self.twin is not None:
            self.twin.close()


def main(argv=None):
    p = argparse.ArgumentParser(prog="python -m robot_control.task.live_plan", description=__doc__.split("\n\n")[0])
    p.add_argument("--frames", help="saved raw frames (an image or a folder) instead of the camera")
    p.add_argument("--twin", action="store_true", help="the simulator's twin camera (twin_scene) instead of the camera")
    p.add_argument("--fps", type=float, default=3.0, help="plans per second (3)")
    p.add_argument("--conf", type=float, help="confidence threshold (default: vision.toml)")
    p.add_argument("--out", default="logs/live_plan", help="where s saves")
    args = p.parse_args(argv)

    from robot_offline.pre.camera import label, show
    from robot_control.vision.detector import Detector

    vision = VisionSettings.from_config()
    rig, workspace, gripper = RigSettings.from_config(), WorkspaceSettings.from_config(), GripperSettings.from_config()
    kin = kinematics()
    detector = Detector(vision, rig, workspace)
    print(f"Loading {vision.weights} ...")
    detector.model
    frames = Frames(args.frames, twin=args.twin)
    print(__doc__.split("Keys:", 1)[1].rstrip())
    fitter = frames.tracker

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    conf = args.conf if args.conf is not None else vision.conf
    top_down, grid = True, True
    view, shown, planned, last = None, None, [], None
    next_run, latency, n_runs, t_start = 0.0, 0.0, 0, time.perf_counter()
    try:
        while True:
            got = frames.read()
            if got is None:
                continue
            snapshot_id, frame = got
            cal = fitter.update(frame)
            now = time.perf_counter()

            if now >= next_run:
                next_run = now + 1.0 / args.fps
                t0 = time.perf_counter()
                detector.vision = VisionSettings(vision.weights, conf)
                planned, balanced = [], None
                if cal is not None:
                    detections, balanced = detector.detect(frame, cal, snapshot_id)
                    planned = plan(detections, kin, gripper, workspace)
                    last = (snapshot_id, frame, cal, planned)
                view = render(frame, cal, balanced, planned, kin, rig, workspace, top_down, grid)
                latency = time.perf_counter() - t0
                n_runs += 1
                rate = n_runs / max(time.perf_counter() - t_start, 1e-6)
                plate = fitter.status()
                reachable = sum(r.ok for _, r in planned)
                label(view, [f"{len(planned)} bodies, {reachable} reachable   conf >= {conf:.2f}   "
                             f"{'top-down' if top_down else 'camera view'}   NOT sending to the arm",
                             f"{rate:.1f} plans/s, {latency * 1000:.0f} ms each   {plate}", frames.status(),
                             "q quit  v view  g grid  +/- confidence  c find plate  e exposure  n next frame  s save"])
                shown = show(view, shown, WINDOW)

            key = cv2.waitKey(1 if frames.live else 30) & 0xFF
            if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("v"):
                top_down, next_run = not top_down, 0.0
            elif key == ord("g"):
                grid, next_run = not grid, 0.0
            elif key in (ord("+"), ord("=")):
                conf, next_run = min(0.95, conf + 0.05), 0.0
            elif key in (ord("-"), ord("_")):
                conf, next_run = max(0.05, conf - 0.05), 0.0
            elif key == ord("c"):
                fitter.restart()
            elif key == ord("e"):
                frames.toggle_exposure()
            elif key == ord("n"):
                frames.next()
                fitter.restart()
                next_run = 0.0
            elif key == ord("s") and view is not None:
                out = Path(args.out)
                out.mkdir(parents=True, exist_ok=True)
                stamp = time.strftime("%Y%m%d_%H%M%S")
                cv2.imwrite(str(out / f"plan_{stamp}.jpg"), view)
                if last is not None:
                    sid, raw, cal_s, planned_s = last
                    cv2.imwrite(str(out / f"plan_{stamp}_raw.png"), raw)
                    (out / f"plan_{stamp}.json").write_text(json.dumps(to_json(sid, planned_s, cal_s), indent=1))
                print(f"Saved {out / f'plan_{stamp}'}.*")
    finally:
        frames.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
