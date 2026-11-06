"""Click a resistor and the arm goes to it, with the camera steering.

    python scripts/resistor_nav.py --calibration <saved plate fit>

robot_offline's live detection in one window: the plate from above with each resistor's outline, spine
and centroid, and its cut-outs beside it. Click a resistor to choose it, then PLAY: in one smooth move the
arm takes the jaws' centre, the axis they turn about, over its centroid on the saved calibration
(scripts/camera_calibrate.py), turning the jaws across it and settling at NAV_SLIDE. The tip stays above the resistor: the jaws can't open yet, and at the pick
height the closed tips would hit it. PARK takes the arm back out of the way.

Start with the arm started and parked (scripts/slide_wrist.py) and nothing else using the camera.
Resistors are only looked for while the arm is parked: it covers the plate otherwise. A yellow line
across each resistor shows the line the jaws should close along.
logs/tips/live.jpg and live.txt follow the window, for watching from outside it.

Keys:  q / Esc  quit      p  PLAY      k  PARK
       [ and ]  turn the wrist a little      m  record that the jaws lie along the chosen yellow line
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import slide_wrist as bench  # noqa: E402
import tip_view  # noqa: E402
from robot_offline import config as offline_config  # noqa: E402
from robot_offline.inf import tiling  # noqa: E402
from robot_offline.inf.live_detect import cutout_panel, draw  # noqa: E402
from robot_offline.post import skeleton  # noqa: E402
from robot_offline.pre import rectify  # noqa: E402
from robot_offline.pre.camera import label, open_camera  # noqa: E402
from robot_offline.pre.exposure import ExposureControl  # noqa: E402
from robot_offline.pre.live_plate import Fitter  # noqa: E402

from robot_control.config import ArmSettings, RigSettings, VisionSettings, WorkspaceSettings  # noqa: E402
from robot_control.vision.arm_fit import load_calibration  # noqa: E402
from robot_control.kinematics.arm import Kinematics  # noqa: E402
from robot_control.vision.detector import Detector, camera_mm  # noqa: E402
from robot_control.vision.guide import plan_jaws  # noqa: E402
from robot_control.vision.rig import correct_height  # noqa: E402

WINDOW = "resistor navigation"
PLATE_PX = 820              # the plate view's height: with the buttons, the window's 900
BUTTON_H = 80
BUTTONS = {"PLAY": (0.0, 0.5), "PARK": (0.5, 1.0)}      # shares of the plate view's width
DETECT_S = 0.5
WRIST_CENTRE = 50.0         # jaws are the same turned 180 deg: of the two, the wrist angle nearest this, always
JAW_LINE_MM = 20.0          # the overlay's reach either side of a body's centroid
GLIDE_S = 4.0
NUDGE_DEG = 2.0
NUDGE_S = 0.6
MARKS = ROOT / "logs" / "jaw_marks.csv"
NAV_SLIDE = 45.0            # reached going down: the tip about 8.6 mm above the plate, clear of a body 2.4 mm tall
NAV_HEIGHT_MM = 8.6
SLIDE_MOVE_S = 1.0


def body_at(polygons, point):
    """The index of the outline (rectified px) around point, or of the nearest within reach, or None."""
    best, nearest = None, 15.0
    for i, poly in enumerate(polygons):
        d = cv2.pointPolygonTest(np.asarray(poly, np.float32).reshape(-1, 1, 2), (float(point[0]), float(point[1])), True)
        if d >= -nearest:
            best, nearest = i, -d
    return best


def centroid_mm(poly, body, cal, lens):
    """A body's centre in scene mm: its skeleton's centroid, or the outline's middle, moved for its height."""
    st = cal.settings
    centre = body.centroid if body is not None else np.asarray(poly, float).reshape(-1, 2).mean(0)
    scene = rectify.rect_to_mm(np.asarray(centre, float).reshape(1, 2), st)
    width_mm = (body.width / st.px_per_mm) if body is not None else 2.4
    return correct_height(scene, lens, st.top_z_mm, width_mm / 2)[0]


class Navigator(threading.Thread):
    """Drives the arm, one command at a time, so the window never waits for it."""

    def __init__(self, scara, kinematics, rig, correction):
        super().__init__(daemon=True)
        self.scara, self.kinematics, self.rig, self.correction = scara, kinematics, rig, correction
        self.command = None         # ("go", scene mm) or ("park",)
        self.busy = False
        self.parked = True
        self.status = "click a resistor, then PLAY"
        self.stop = False

    def say(self, text):
        self.status = text
        print(text, flush=True)

    def run(self):
        while not self.stop:
            command, self.command = self.command, None
            if command is None:
                time.sleep(0.05)
                continue
            self.busy = True
            try:
                getattr(self, command[0])(*command[1:])
            except (SystemExit, ValueError) as stopped:
                self.say(f"stopped: {stopped}")
            self.busy = False

    def drive(self, joint, target, seconds):
        for angle in bench.slide_steps(bench.load_state()[joint], target, bench.MAX_STEP[joint]):
            bench.move(self.scara, joint, round(angle, 1), seconds)

    def go(self, target, body_deg):
        """Straight there on the calibration: the jaws across the body, the tip over its centre. Nothing
        moves unless the whole trip is allowed."""
        base, elbow, wrist = plan_jaws(target, body_deg, bench.SAFE["wrist"], self.kinematics, self.rig, WRIST_CENTRE,
                                       self.correction, at_axis=True)
        why = bench.swing_refusal(base, elbow, wrist, (base, elbow), bench.HOVER_SLIDE, self.kinematics.arm, [],
                                  self.kinematics, self.rig)
        if why:
            self.say(f"CAN'T REACH THAT ONE, nothing moved: {why}")
            return
        self.parked = False
        self.say(f"going to ({target[0]:.1f}, {target[1]:.1f}), jaws across {body_deg % 180:.0f} deg")
        bench.glide(self.scara, base, elbow, wrist, NAV_SLIDE, GLIDE_S)
        got_base, got_elbow = bench.read_joints(self.scara)
        self.say(f"arrived, about {NAV_HEIGHT_MM:.0f} mm up: base off {got_base - base:+.2f}, elbow off {got_elbow - elbow:+.2f} deg, "
                 f"wrist {wrist:.1f}")

    def nudge(self, step):
        """Turns the wrist a little, for lining the jaws up by eye."""
        wrist = bench.load_state()["wrist"] + step
        bench.move(self.scara, "wrist", wrist, NUDGE_S)
        self.say(f"wrist {wrist:.1f}: [ and ] turn it, m when the jaws lie along the yellow line")

    def mark(self, body_deg):
        """Records that the jaws lie across a body along body_deg at this pose, for fitting the wrist."""
        base, elbow = bench.read_joints(self.scara)
        wrist = bench.load_state()["wrist"]
        new = not MARKS.exists()
        with MARKS.open("a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["time", "body_deg", "base", "elbow", "wrist"])
            w.writerow([time.strftime("%H:%M:%S"), round(body_deg, 2), round(base, 2), round(elbow, 2), round(wrist, 2)])
        self.say(f"marked: body {body_deg % 180:.0f} deg at wrist {wrist:.1f}")

    def park(self):
        self.say("parking")
        self.drive("slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
        bench.park(self.scara)
        self.parked = True
        self.say("parked: click a resistor, then PLAY")


def body_direction(body, settings):
    """A body's long axis, scene deg from +x anticlockwise, from the ends of its spine."""
    a, b = rectify.rect_to_mm(np.asarray(body.spine, float)[[0, -1]], settings)
    return float(np.degrees(np.arctan2(*(b - a)[::-1])))


def draw_jaw_line(rect, body, settings, chosen):
    """The line the jaws should close along, across the body through its centroid, on the plate."""
    ends = np.asarray(body.spine, float)[[0, -1]]
    along = (ends[1] - ends[0]) / np.linalg.norm(ends[1] - ends[0])
    across = np.array([-along[1], along[0]]) * JAW_LINE_MM * settings.px_per_mm
    centre = np.asarray(body.centroid, float).reshape(2)
    a, b = np.round(centre - across).astype(int), np.round(centre + across).astype(int)
    cv2.line(rect, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (0, 255, 255), 5 if chosen else 2, cv2.LINE_AA)


def buttons(width, busy, chosen):
    """The button bar, width wide."""
    bar = np.full((BUTTON_H, width, 3), 30, np.uint8)
    for name, (a, b) in BUTTONS.items():
        x0, x1 = int(a * width) + 6, int(b * width) - 6
        live = not busy and (chosen or name == "PARK")
        cv2.rectangle(bar, (x0, 8), (x1, BUTTON_H - 8), (60, 150, 60) if live else (70, 70, 70), -1)
        cv2.putText(bar, name, (x0 + (x1 - x0) // 2 - 45, BUTTON_H // 2 + 12), cv2.FONT_HERSHEY_SIMPLEX, 1.1,
                    (255, 255, 255), 2, cv2.LINE_AA)
    return bar


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calibration", required=True, help="a saved plate fit to refit near")
    p.add_argument("--port", help="the Nucleo's serial port (default: found by its USB id)")
    args = p.parse_args()

    if bench.load_state() is None:
        sys.exit("Not started: run scripts/slide_wrist.py start, then park.")
    vision, rig, workspace = VisionSettings.from_config(), RigSettings.from_config(), WorkspaceSettings.from_config()
    detector = Detector(vision, rig, workspace)
    print("Loading the model...")
    model = detector.model
    print("Opening the camera (a few seconds while exposure and white balance settle)...")
    camera = offline_config.load("camera")
    cap = open_camera(camera)
    control = ExposureControl(camera["exposure_control"], camera["camera"]["exposure"], camera["camera"]["gain"], cap)
    start = rectify.Calibration.load(args.calibration)
    fitter = Fitter(start.settings)
    fitter.cal = fitter.first = start
    fitter.start()
    scara = bench.smc_api.scara_motion_controller(port=args.port or bench.find_port(), timeout=120)
    nav = Navigator(scara, Kinematics(ArmSettings.from_config()), *load_calibration())
    nav.start()

    polygons, scores, bodies, chosen = [], [], [], None
    state = {"click": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (x, y)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.moveWindow(WINDOW, 40, 20)      # else it can open off the screen
    cv2.setMouseCallback(WINDOW, on_mouse)
    size = start.settings.size
    scale = PLATE_PX / size[1]
    plate_w = int(size[0] * scale)
    shown, next_detect, snapped = None, 0.0, 0.0
    print(__doc__.split("Keys:", 1)[1].rstrip())
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        fitter.frame = frame
        cal = fitter.cal
        control.meter(frame, cal)
        if fitter.fitted_at and fitter.error is None:
            fitter.locked = True
        now = time.time()

        if nav.parked and not nav.busy and fitter.locked and now >= next_detect:
            next_detect = now + DETECT_S
            model_view, _ = rectify.model_input(frame, cal)
            tiles = tiling.tiles(detector.work_area, (model_view.shape[1], model_view.shape[0]),
                                 detector.tile_size, detector.tile_overlap)
            polygons, scores = tiling.predict_frame(model, model_view, tiles, conf=vision.conf)
            bodies = [skeleton.body_skeleton(q) for q in polygons]
            if chosen is not None and chosen >= len(polygons):
                chosen = None

        rect, _ = rectify.white_balance(rectify.rectify(frame, cal), cal.settings)
        rect = rectify.draw_grid(rect, cal.settings)
        axes = [(b.spine, b.centroid) for b in bodies if b is not None]
        draw(rect, polygons, scores, axes=axes)
        if chosen is not None:
            cv2.polylines(rect, [np.round(polygons[chosen]).astype(np.int32)], True, (0, 255, 0), 5, cv2.LINE_AA)
        for i, body in enumerate(bodies):
            if body is not None:
                draw_jaw_line(rect, body, cal.settings, i == chosen)
        plate = cv2.resize(rect, (plate_w, PLATE_PX), interpolation=cv2.INTER_AREA)
        lines = [nav.status, f"{len(polygons)} resistors" + (f", #{chosen + 1} chosen" if chosen is not None else ""),
                 control.status()]
        label(plate, lines)
        left = np.vstack([plate, buttons(plate_w, nav.busy, chosen is not None)])
        panel = cutout_panel(rectify.white_balance(rectify.rectify(frame, cal), cal.settings)[0], polygons, scores,
                             left.shape[0], bodies, cal.settings)
        view = np.hstack([left, panel])
        size_now = view.shape[1], view.shape[0]
        if shown != size_now:
            cv2.resizeWindow(WINDOW, *size_now)
            shown = size_now
        cv2.imshow(WINDOW, view)
        if now - snapped > tip_view.SNAPSHOT_S:
            tip_view.snapshot(view, lines)
            snapped = now

        play = park = False
        click, state["click"] = state["click"], None
        if click is not None and click[0] < plate_w:
            if click[1] < PLATE_PX:
                if not nav.busy:
                    chosen = body_at(polygons, (click[0] / scale, click[1] / scale))
            else:
                play, park = click[0] < plate_w / 2, click[0] >= plate_w / 2
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
        play, park = play or key == ord("p"), park or key == ord("k")
        if key in (ord("["), ord("]")) and not nav.busy and not nav.parked:
            nav.command = ("nudge", NUDGE_DEG if key == ord("]") else -NUDGE_DEG)
        elif key == ord("m") and not nav.busy and not nav.parked and chosen is not None and bodies[chosen] is not None:
            nav.command = ("mark", body_direction(bodies[chosen], cal.settings))
        if play and chosen is not None and not nav.busy and bodies[chosen] is not None:
            nav.command = ("go", centroid_mm(polygons[chosen], bodies[chosen], cal, camera_mm(cal)),
                           body_direction(bodies[chosen], cal.settings))
        elif park and not nav.busy:
            nav.command = ("park",)

    nav.stop = True
    fitter.stop = True
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
