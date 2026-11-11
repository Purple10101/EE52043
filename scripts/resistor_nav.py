"""Click a resistor and the arm goes to it, with the camera steering.

    python scripts/resistor_nav.py

robot_offline's live detection in one window: the plate from above with each resistor's outline, spine
and centroid, and its cut-outs beside it. Click a resistor to choose it, then PLAY: in one smooth move the
arm takes the jaws' centre, the axis they turn about, over its centroid on the saved calibration
(scripts/camera_calibrate.py), turning the jaws across it and settling at NAV_SLIDE, with the jaws open:
a look at the aim, touching nothing. PICK does the same, then lowers, grips and lifts. Once it holds a
resistor a bin must be chosen: every other command is refused until one drops it, whichever button or key
sent it. PARK takes the arm back out of the way, and OPEN and CLOSE work the jaws. The row below goes to a
bin: the arm lifts to the travel height, goes over the slot's centre, round the camera tower if it has
to, turning the jaws so that what they hold lies along the slot, and lowers the gripper, on the angles
taught at each slot (robot_control/config/bins_real.toml), not the camera's fit, which only covers the
plate. Every move away from a bin lifts the gripper to the travel height first. Resistors are only looked
for while parked, so PARK to see the plate as it is now.

Start with the arm started and parked (scripts/slide_wrist.py) and nothing else using the camera. The
plate is found from the live camera each time, and the fit is held once found; until then the plain
camera image is shown. c finds it again from scratch: park first, so the arm isn't in the way.
Resistors are only looked for while the arm is parked: it covers the plate otherwise. A yellow line
across each resistor shows the line the jaws should close along.
logs/tips/live.jpg and live.txt follow the window, for watching from outside it.

Keys:  q / Esc  quit (the only way out while holding: it leaves the jaws gripping)
       p  PLAY      k  PARK      c  find the plate again
       [ and ]  turn the wrist a little      m  record that the jaws lie along the chosen yellow line
"""

import argparse
import csv
import sys
import threading
import time
import tomllib
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
from robot_offline.pre import plate_search, rectify  # noqa: E402
from robot_offline.pre.camera import label, open_camera  # noqa: E402

from robot_control.config import ArmSettings, KeepOutMap, RigSettings, VisionSettings, WorkspaceSettings  # noqa: E402
from robot_control.gripper import Gripper, GripperError  # noqa: E402
from robot_control.kinematics import KeepOut, route  # noqa: E402
from robot_control.vision.arm_fit import JointCorrection, load_calibration  # noqa: E402
from robot_control.vision.bodies import body_direction, centroid_mm  # noqa: E402
from robot_control.kinematics.arm import JointState, Kinematics  # noqa: E402
from robot_control.vision.detector import Detector, camera_mm  # noqa: E402
from robot_control.vision.guide import plan_jaws, pots_for, wrist_across  # noqa: E402

WINDOW = "resistor navigation"
PLATE_PX = 750              # the plate view's height: with both rows of buttons, the window's 900
BUTTON_H = 80
BIN_BUTTON_H = 70
ACTIONS = ["PLAY", "PICK", "PARK", "OPEN", "CLOSE"]
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
BINS = ROOT / "robot_control" / "config" / "bins_real.toml"
BIN_DEG_PER_S = 20.0        # a long swing takes longer, at a little over the park's base move
BIN_SLIDE = bench.OVER_PLATE_SLIDE      # the slide's lowest: the tip about 19 mm above a bin's rim, which is below the plate
BIN_TOLERANCE_DEG = 2.0     # further than this from a target, the gripper stays up
PICK_SLIDE = bench.OVER_PLATE_SLIDE     # the pick height the user set: the tips about 1 mm above the plate
GRIP_SETTLE_S = 0.3


def body_at(polygons, point):
    """The index of the outline (rectified px) around point, or of the nearest within reach, or None."""
    best, nearest = None, 15.0
    for i, poly in enumerate(polygons):
        d = cv2.pointPolygonTest(np.asarray(poly, np.float32).reshape(-1, 1, 2), (float(point[0]), float(point[1])), True)
        if d >= -nearest:
            best, nearest = i, -d
    return best


def load_bins(path=BINS):
    """{bin id: its entry in the file (centre_mm, long_axis_deg)}, in the file's order."""
    with open(path, "rb") as f:
        return {b["id"]: b for b in tomllib.load(f)["bins"]}


def drop_point(slot):
    """Where to drop into a slot, scene mm: its centre, moved along the slot by drop_along_mm if it has one."""
    along = np.radians(slot["long_axis_deg"])
    return np.asarray(slot["centre_mm"], float) + slot.get("drop_along_mm", 0.0) * np.array([np.cos(along), np.sin(along)])


def bin_path(zones, start, target):
    """(base, elbow) stops, ending at target, that keep a move from start clear of zones, or None."""
    stops = route(zones, JointState(*start, 0.0, 0.0), JointState(*target, 0.0, 0.0), bench.ELBOW_MAX)
    return None if stops is None else [(s.j0, s.j1) for s in stops]


def bin_at(x, width, names):
    """The bin whose button is at x in a row width wide."""
    return names[min(max(int(x * len(names) / width), 0), len(names) - 1)]


class Navigator(threading.Thread):
    """Drives the arm, one command at a time, so the window never waits for it."""

    def __init__(self, scara, kinematics, rig, correction, gripper=None):
        super().__init__(daemon=True)
        self.scara, self.kinematics, self.rig, self.correction = scara, kinematics, rig, correction
        self.gripper = gripper      # None: no jaws, so everything but PICK, OPEN and CLOSE still works
        self.holding = False        # a resistor is in the jaws: only a bin is allowed
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
                self.perform(command)
            except (SystemExit, ValueError, GripperError) as stopped:
                self.say(f"stopped: {stopped}")
            self.busy = False

    def perform(self, command):
        """Runs one command. With a resistor in the jaws only a bin will do, wherever the command came from."""
        if self.holding and command[0] != "to_bin":
            self.say("HOLDING A RESISTOR: choose a bin")
            return
        getattr(self, command[0])(*command[1:])

    def drive(self, joint, target, seconds):
        for angle in bench.slide_steps(bench.load_state()[joint], target, bench.MAX_STEP[joint], joint == "slide"):
            bench.move(self.scara, joint, round(angle, 1), seconds)

    def lift(self):
        """The slide up to the travel height if it is lower; from the park it is higher already, and stays."""
        if bench.load_state()["slide"] > bench.HOVER_SLIDE:
            self.drive("slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)

    def go(self, target, body_deg):
        """There on the calibration: the jaws across the body, the tip over its centre. Coming from a bin, the
        gripper lifts first and the arm goes round the tower if it has to. Nothing moves unless the whole
        trip is allowed. The jaws open first. Returns how far off the worse joint landed, deg, or None if
        nothing moved."""
        base, elbow, wrist = plan_jaws(target, body_deg, bench.SAFE["wrist"], self.kinematics, self.rig, WRIST_CENTRE,
                                       self.correction, at_axis=True)
        why = bench.swing_refusal(base, elbow, wrist, (base, elbow), bench.HOVER_SLIDE, self.kinematics.arm, [],
                                  self.kinematics, self.rig)
        if why:
            self.say(f"CAN'T REACH THAT ONE, nothing moved: {why}")
            return
        here = bench.read_joints(self.scara, bench.QUICK_READS)
        path = bin_path([KeepOut(m) for m in KeepOutMap.from_config()], here, (base, elbow))
        if path is None:
            self.say("CAN'T REACH THAT ONE, nothing moved: no way round the tower from here")
            return
        self.parked = False
        self.say(f"going to ({target[0]:.1f}, {target[1]:.1f}), jaws across {body_deg % 180:.0f} deg")
        if self.gripper is not None:
            self.gripper.open()
        if bench.load_state()["slide"] > NAV_SLIDE:
            self.lift()
        for stop in path[:-1]:
            change = max(abs(stop[0] - here[0]), abs(stop[1] - here[1]))
            if change > 0.1:
                bench.swing(self.scara, *stop, bench.load_state()["wrist"], max(bench.SWING_S, change / BIN_DEG_PER_S))
            here = stop
        if bench.passes_bin(here, (base, elbow), self.kinematics, self.rig):
            # from the park, over the non-resistor bin: out over the plate with the slide still fully up, then down
            change = max(abs(base - here[0]), abs(elbow - here[1]))
            bench.swing(self.scara, base, elbow, bench.load_state()["wrist"], max(bench.SWING_S, change / BIN_DEG_PER_S))
            self.drive("slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
        bench.glide(self.scara, base, elbow, wrist, NAV_SLIDE, GLIDE_S)
        got_base, got_elbow = bench.read_joints(self.scara)
        self.say(f"arrived, about {NAV_HEIGHT_MM:.0f} mm up: base off {got_base - base:+.2f}, elbow off {got_elbow - elbow:+.2f} deg, "
                 f"wrist {wrist:.1f}")
        return max(abs(got_base - base), abs(got_elbow - elbow))

    def pick(self, target, body_deg):
        """Over a resistor as go does, then down, grip and up. Holding from then on, until a bin takes it.
        Returns whether the jaws closed where it lay (they may still have missed it)."""
        if self.gripper is None:
            self.say("CAN'T PICK, nothing moved: the gripper board isn't connected")
            return False
        off = self.go(target, body_deg)
        if off is None:
            return False
        if off > BIN_TOLERANCE_DEG:
            self.say(f"NOT LOWERING, the arm missed by {off:.2f} deg")
            return False
        self.drive("slide", PICK_SLIDE, SLIDE_MOVE_S)
        self.gripper.close()
        time.sleep(GRIP_SETTLE_S)
        self.holding = True
        self.drive("slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
        self.say("HOLDING A RESISTOR: choose a bin")
        return True

    def jaws(self, action):
        """OPEN or CLOSE, on their own."""
        if self.gripper is None:
            self.say("the gripper board isn't connected")
            return
        (self.gripper.open if action == "OPEN" else self.gripper.close)()
        self.say(f"jaws {'open' if action == 'OPEN' else 'closed'}")

    def to_bin(self, name, slot):
        """Over a bin at the travel height, the jaws turned so what they hold lies along the slot, then the
        gripper lowered. Nothing moves unless a way there exists, and it only lowers if the arm got there.
        Returns whether it got there and lowered (and dropped what it held)."""
        base, elbow = pots_for(drop_point(slot), self.kinematics, RigSettings.from_config(), JointCorrection())
        wrist = wrist_across(base, elbow, slot["long_axis_deg"], bench.SAFE["wrist"], WRIST_CENTRE)
        here = bench.read_joints(self.scara, bench.QUICK_READS)
        path = bin_path([KeepOut(m) for m in KeepOutMap.from_config()], here, (base, elbow))
        if path is None:
            self.say(f"CAN'T REACH BIN {name}, nothing moved: base {base:.1f}, elbow {elbow:.1f} is in the keep-out"
                     + (". STILL HOLDING: choose another bin" if self.holding else ""))
            return False
        self.parked = False
        self.say(f"going to bin {name}")
        self.lift()
        turned = bench.load_state()["wrist"]
        for stop in path:
            change = max(abs(stop[0] - here[0]), abs(stop[1] - here[1]), abs(wrist - turned))
            if change > 0.1:
                bench.swing(self.scara, *stop, wrist, max(bench.SWING_S, change / BIN_DEG_PER_S))
            here, turned = stop, wrist
        got_base, got_elbow = bench.read_joints(self.scara)
        off = f"base off {got_base - base:+.2f}, elbow off {got_elbow - elbow:+.2f} deg"
        if max(abs(got_base - base), abs(got_elbow - elbow)) > BIN_TOLERANCE_DEG:
            self.say(f"NOT LOWERING at bin {name}, the arm missed: {off}" + (". STILL HOLDING: choose a bin" if self.holding else ""))
            return False
        self.drive("slide", BIN_SLIDE, SLIDE_MOVE_S)
        if self.holding:
            self.gripper.open()
            time.sleep(GRIP_SETTLE_S)
            self.holding = False
            self.drive("slide", bench.HOVER_SLIDE, SLIDE_MOVE_S)
            self.say(f"dropped in bin {name}: {off}. PARK to look at the plate again")
            return True
        self.say(f"at bin {name}, lowered: {off}, wrist {wrist:.1f}")
        return True

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
        bench.park(self.scara)      # the slide fully up first: the park is over the non-resistor bin
        self.parked = True
        self.say("parked: click a resistor, then PLAY")


def draw_jaw_line(rect, body, settings, chosen):
    """The line the jaws should close along, across the body through its centroid, on the plate."""
    ends = np.asarray(body.spine, float)[[0, -1]]
    along = (ends[1] - ends[0]) / np.linalg.norm(ends[1] - ends[0])
    across = np.array([-along[1], along[0]]) * JAW_LINE_MM * settings.px_per_mm
    centre = np.asarray(body.centroid, float).reshape(2)
    a, b = np.round(centre - across).astype(int), np.round(centre + across).astype(int)
    cv2.line(rect, (int(a[0]), int(a[1])), (int(b[0]), int(b[1])), (0, 255, 255), 5 if chosen else 2, cv2.LINE_AA)


def waiting_view(frame, width, height):
    """The camera's own image, width wide on a black height-tall view, while the plate isn't found yet."""
    shrunk = cv2.resize(frame, (width, frame.shape[0] * width // frame.shape[1]), interpolation=cv2.INTER_AREA)
    view = np.zeros((height, width, 3), np.uint8)
    top = (height - shrunk.shape[0]) // 2
    view[top:top + shrunk.shape[0]] = shrunk
    return view


def buttons(width, busy, chosen, holding=False):
    """The button bar, width wide. Holding, none is live: only a bin will do."""
    bar = np.full((BUTTON_H, width, 3), 30, np.uint8)
    for i, name in enumerate(ACTIONS):
        x0, x1 = i * width // len(ACTIONS) + 4, (i + 1) * width // len(ACTIONS) - 4
        live = not busy and not holding and (chosen or name not in ("PLAY", "PICK"))
        cv2.rectangle(bar, (x0, 8), (x1, BUTTON_H - 8), (60, 150, 60) if live else (70, 70, 70), -1)
        (text_w, _), _ = cv2.getTextSize(name, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.putText(bar, name, (x0 + (x1 - x0 - text_w) // 2, BUTTON_H // 2 + 9), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2, cv2.LINE_AA)
    return bar


def bin_buttons(width, names, busy, holding=False):
    """The row of bin buttons, width wide, bright when one must be chosen."""
    bar = np.full((BIN_BUTTON_H, width, 3), 30, np.uint8)
    for i, name in enumerate(names):
        x0, x1 = i * width // len(names) + 3, (i + 1) * width // len(names) - 3
        colour = (70, 70, 70) if busy else (0, 140, 255) if holding else (150, 100, 40)
        cv2.rectangle(bar, (x0, 4), (x1, BIN_BUTTON_H - 8), colour, -1)
        cv2.putText(bar, name, (x0 + (x1 - x0) // 2 - 18, BIN_BUTTON_H // 2 + 7), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    (255, 255, 255), 2, cv2.LINE_AA)
    return bar


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
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
    settings = rectify.Settings.from_config()
    # robot_offline's plate search: found once (the arm in view would spoil refits); c looks again.
    finder = plate_search.Tracker(cap, settings, camera=camera, refit_seconds=float("inf"))
    scara = bench.smc_api.scara_motion_controller(port=args.port or bench.find_port(), timeout=120)
    try:
        gripper = Gripper()
    except (GripperError, OSError) as why:
        gripper = None
        print(f"No gripper: {why}", flush=True)
    nav = Navigator(scara, Kinematics(ArmSettings.from_config()), *load_calibration(), gripper)
    nav.start()
    bins = load_bins()
    bin_names = list(bins)

    polygons, scores, bodies, chosen = [], [], [], None
    state = {"click": None}

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            state["click"] = (x, y)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.moveWindow(WINDOW, 40, 20)      # else it can open off the screen
    cv2.setMouseCallback(WINDOW, on_mouse)
    size = settings.size
    scale = PLATE_PX / size[1]
    plate_w = int(size[0] * scale)
    shown, next_detect, snapped = None, 0.0, 0.0
    print(__doc__.split("Keys:", 1)[1].rstrip())
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        cal = finder.update(frame)
        now = time.time()

        if nav.parked and not nav.busy and cal is not None and not finder.searching and now >= next_detect:
            next_detect = now + DETECT_S
            model_view, _ = rectify.model_input(frame, cal)
            tiles = tiling.tiles(detector.work_area, (model_view.shape[1], model_view.shape[0]),
                                 detector.tile_size, detector.tile_overlap)
            polygons, scores = tiling.predict_frame(model, model_view, tiles, conf=vision.conf, grey=detector.grey)
            bodies = [skeleton.body_skeleton(q) for q in polygons]
            if chosen is not None and chosen >= len(polygons):
                chosen = None

        if cal is None:
            plate = waiting_view(frame, plate_w, PLATE_PX)
            lines = [nav.status, finder.status(), finder.control.status()]
        else:
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
                     finder.control.status()]
        label(plate, lines)
        view = np.vstack([plate, buttons(plate_w, nav.busy, chosen is not None, nav.holding),
                          bin_buttons(plate_w, bin_names, nav.busy, nav.holding)])
        if cal is not None:
            panel = cutout_panel(rectify.white_balance(rectify.rectify(frame, cal), cal.settings)[0], polygons, scores,
                                 view.shape[0], bodies, cal.settings)
            view = np.hstack([view, panel])
        size_now = view.shape[1], view.shape[0]
        if shown != size_now:
            cv2.resizeWindow(WINDOW, *size_now)
            shown = size_now
        cv2.imshow(WINDOW, view)
        if now - snapped > tip_view.SNAPSHOT_S:
            tip_view.snapshot(view, lines)
            snapped = now

        action = to_bin = None
        click, state["click"] = state["click"], None
        if click is not None and click[0] < plate_w:
            if click[1] < PLATE_PX:
                if not nav.busy:
                    chosen = body_at(polygons, (click[0] / scale, click[1] / scale))
            elif click[1] < PLATE_PX + BUTTON_H:
                action = bin_at(click[0], plate_w, ACTIONS)
            else:
                to_bin = bin_at(click[0], plate_w, bin_names)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
        action = "PLAY" if key == ord("p") else "PARK" if key == ord("k") else action
        if key == ord("c"):
            finder.restart()
            polygons, scores, bodies, chosen = [], [], [], None
        if key in (ord("["), ord("]")) and not nav.busy and not nav.parked:
            nav.command = ("nudge", NUDGE_DEG if key == ord("]") else -NUDGE_DEG)
        elif key == ord("m") and not nav.busy and not nav.parked and chosen is not None and bodies[chosen] is not None:
            nav.command = ("mark", body_direction(bodies[chosen], cal.settings))
        if nav.holding:         # that resistor has gone from where it was seen
            polygons, scores, bodies, chosen = [], [], [], None
        if action in ("PLAY", "PICK") and chosen is not None and not nav.busy and bodies[chosen] is not None:
            nav.command = ("go" if action == "PLAY" else "pick", centroid_mm(polygons[chosen], bodies[chosen], cal, camera_mm(cal)),
                           body_direction(bodies[chosen], cal.settings))
        elif action == "PARK" and not nav.busy:
            nav.command = ("park",)
        elif action in ("OPEN", "CLOSE") and not nav.busy:
            nav.command = ("jaws", action)
        elif to_bin and not nav.busy:
            nav.command = ("to_bin", to_bin, bins[to_bin])

    nav.stop = True
    finder.close()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
