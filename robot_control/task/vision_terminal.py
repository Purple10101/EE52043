"""The vision terminal: a light window for seeing how segmentation and band reading do, resistor by resistor.

    +----------------------------------------+------------------+
    | what it is doing now                   |  SUBJECT         |
    | the arm; segmentation, live and looked |  where, its live |
    +----------------------------------------+  (provisional)   |
    |  the plate LIVE, every frame:          |  reading, its    |
    |  each body outlined, its score and its |  CLASSIFIED one, |
    |  provisional reading                   |  their crops     |
    +----------------------------------------+------------------+
    |  CALIBRATE  |  SEGMENT  |  CLASSIFY  |  PICK              |
    +-----------------------------------------------------------+

Built from the sorter's parts: the camera and its segmentation and band reader (vision.Eyes), the window
(task.RunWindow), the standard calibration (task.auto_app.StandardCalibration) and the arm (motion.Arm).

LIVE      always running: every frame through the plate's tracker, the segmentation model (one frame, not
          averaged) and the band reader on each body's crop from that frame, at whatever focus the lens is at.
          Those readings are PROVISIONAL: a single frame, no focus check, no close-up.
click     a body on the plate makes it the SUBJECT, followed from frame to frame.
CALIBRATE the standard calibration, as in sort_resistors_auto.py: the arm started, the plate found EMPTY, the
          aim checked.
SEGMENT   the sorter's own look (Eyes.look): the focus checked, 8 frames averaged, segmented. Its outlines
          (white) and scores stay over the live view until the next SEGMENT, to compare with frame by frame.
CLASSIFY  the subject read as a sorting run reads it (Eyes.read_values, for one body): the focus checked on the
          plate's cross, an averaged picture, then a close-up at the focus its distance needs, both crops read
          and combined. That reading is CLASSIFIED and stays with the body while it stays where it was.
PICK      the subject picked and dropped in one bin (--bin), then the arm parked. Needs the arm started.

The plate is always live: the camera's own picture until the plate is fitted, then top-down. While CALIBRATE,
SEGMENT, CLASSIFY or PICK's picture use the camera, the live segmentation pauses but the plate keeps showing the
frames the job reads (vision.SharedCamera).
"""

import math
import textwrap
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
from robot_offline.post.bands import closeup, crops, learned
from robot_offline.post.bands.learned import ohms_text
from robot_offline.pre import rectify

from robot_control.task.aim_check import camera_aimed
from robot_control.task.auto_app import CalibrationFailed, StandardCalibration
from robot_control.task.window import (FONT, LIVE_FPS, PLATE_PX, STATUS_CHARS, STATUS_H, TRAY_W, RunWindow,
                                       draw_buttons, fit, live_plate)
from robot_control.vision.eyes import combine
from robot_control.world import Resistor

TITLE = "vision terminal"
BUTTONS = ["CALIBRATE", "SEGMENT", "CLASSIFY", "PICK"]
TRACK_MM = 8.0              # a body this close to where the subject was last is still the subject
SAME_MM = 4.0               # a classified reading stays with a body while it stays this close to where it was read
CLICK_PX = 60               # a click this close (rectified px) to a body's outline selects it
SUBJECT, CLASSIFIED, BODY, FAINT, LOOKED = (0, 255, 255), (0, 230, 0), (0, 160, 255), (150, 150, 150), (255, 255, 255)
READY = ("Click a resistor to make it the subject. CALIBRATE (empty plate) before the first PICK after a power-on.")


@dataclass
class Body:
    """One body in a live frame."""
    outline: np.ndarray         # rectified px
    score: float
    centre: tuple               # scene mm
    direction: float            # deg
    reading: object = None      # learned.Reading from this frame alone, or None if it is too faint to read


@dataclass
class Classified:
    """A reading taken by CLASSIFY, kept with the body it was taken of."""
    centre: tuple
    reading: object
    crop: object
    close_crop: object
    when: str


def nearest(bodies, point, within):
    """The body whose centre is nearest point (scene mm), if within that many mm; else None."""
    if point is None or not bodies:
        return None
    best = min(bodies, key=lambda b: math.dist(b.centre, point))
    return best if math.dist(best.centre, point) <= within else None


def body_at(bodies, click_px, within=CLICK_PX):
    """The body under a click (rectified px), or the nearest outline within `within` px; else None."""
    best, best_d = None, -float("inf")
    for b in bodies:
        d = cv2.pointPolygonTest(np.asarray(b.outline, np.float32).reshape(-1, 1, 2), click_px, True)
        if d > best_d:
            best, best_d = b, d
    return best if best is not None and best_d >= -within else None


def describe(reading):
    """A reading as one short line: its value (or best guess, marked unsure), confidence and bands."""
    if reading is None:
        return "no reading"
    if reading.value is not None:
        text = f"{ohms_text(reading.value)} {reading.confidence:.0%}"
    else:
        text = f"unsure: best {ohms_text(float(reading.best))} {reading.confidence:.0%}"
    colours = " ".join(getattr(reading, "colours", ()) or ())
    return f"{text}  {colours}".strip()


class SavedFrames:
    """A stand-in for the camera: saved pictures of the plate, each served for `seconds` before the next, round
    and round. Takes cv2.VideoCapture's read, set, get and release; set does nothing (no focus or exposure)."""

    def __init__(self, paths, seconds=10.0):
        self.paths = [str(p) for p in paths]
        if not self.paths:
            raise ValueError("no saved frames given")
        self.seconds = seconds
        self.started = time.monotonic()
        self._cache = {}

    def read(self):
        path = self.paths[int((time.monotonic() - self.started) // self.seconds) % len(self.paths)]
        if path not in self._cache:
            self._cache = {path: cv2.imread(path)}
        frame = self._cache[path]
        return frame is not None, None if frame is None else frame.copy()

    def set(self, prop, value):
        return True

    def get(self, prop):
        return 0.0

    def release(self):
        self._cache = {}


def short(reading):
    """A reading for a label on the plate: its value and confidence, "?" after an unsure best guess."""
    if reading is None:
        return "-"
    if reading.value is not None:
        return f"{ohms_text(reading.value)} {reading.confidence:.0%}"
    return f"{ohms_text(float(reading.best))}? {reading.confidence:.0%}"


class TerminalWindow(RunWindow):
    """RunWindow with the live plate on the left, the subject's panel on the right, and clicks on the plate."""

    def __init__(self, buttons=BUTTONS):
        super().__init__(title=TITLE, buttons=buttons)
        self._live = None           # the live plate, drawn, PLATE_PX square
        self._scale = 1.0           # rectified px per window px on the plate
        self._seg_lines = []
        self._subject = []          # (text, colour) lines for the subject panel
        self._crops = []            # (label, image) for the subject panel

    def show_live(self, image, scale, lines):
        with self._lock:
            self._live, self._scale, self._seg_lines = image, scale, list(lines)

    def show_live_image(self, image):
        """A new live plate picture alone, the scale and lines kept (while a job has the camera)."""
        with self._lock:
            self._live = image

    def show_subject(self, lines, crop_images):
        with self._lock:
            self._subject, self._crops = list(lines), list(crop_images)

    def poll(self, enabled):
        """As RunWindow.poll, and ("SELECT", (x, y)) for a click on the plate, in rectified px."""
        with self._lock:
            image = self._compose(enabled)
            scale = self._scale
        size = image.shape[1], image.shape[0]
        if self._size != size:
            cv2.resizeWindow(self.title, *size)
            self._size = size
        cv2.imshow(self.title, image)
        key = cv2.waitKey(15) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) < 1:
            return "QUIT"
        click, self._click = self._click, None
        if click is None or click[1] < STATUS_H:
            return None
        if click[1] < STATUS_H + PLATE_PX:
            return ("SELECT", (click[0] * scale, (click[1] - STATUS_H) * scale)) if click[0] < PLATE_PX else None
        name = self.buttons[min(click[0] * len(self.buttons) // size[0], len(self.buttons) - 1)]
        return name if name in enabled else None

    def _compose(self, enabled):
        status = np.full((STATUS_H, PLATE_PX, 3), 25, np.uint8)
        now = textwrap.wrap(self._status, STATUS_CHARS)[:3]
        rest = [self._arm_line] + self._seg_lines[:2]
        for i, text in enumerate(now):
            cv2.putText(status, fit(text, STATUS_CHARS), (12, 28 + 26 * i), FONT, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        for i, text in enumerate(rest):
            cv2.putText(status, fit(text, STATUS_CHARS + 8), (12, 108 + 24 * i), FONT, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        plate = self._live if self._live is not None else waiting_plate()
        left = np.vstack([status, plate])
        right = draw_subject(self._subject, self._crops, STATUS_H + PLATE_PX)
        return np.vstack([np.hstack([left, right]), draw_buttons(PLATE_PX + TRAY_W, enabled, self.buttons)])


def waiting_plate():
    plate = np.full((PLATE_PX, PLATE_PX, 3), 40, np.uint8)
    cv2.putText(plate, "Finding the plate...", (200, PLATE_PX // 2), FONT, 0.9, (200, 200, 200), 2, cv2.LINE_AA)
    return plate


def draw_subject(lines, crop_images, height):
    """The subject panel: its lines, then its crops, each scaled to the panel's width."""
    panel = np.full((height, TRAY_W, 3), 30, np.uint8)
    cv2.putText(panel, "SUBJECT", (16, 40), FONT, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    y = 76
    for text, colour in lines:
        for part in textwrap.wrap(text, 38) or [""]:
            cv2.putText(panel, part, (16, y), FONT, 0.6, colour, 1 if colour == FAINT else 2, cv2.LINE_AA)
            y += 26
    y += 10
    for label, image in crop_images:
        if image is None or y > height - 60:
            continue
        cv2.putText(panel, label, (16, y), FONT, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
        y += 8
        w = TRAY_W - 32
        h = max(1, int(image.shape[0] * w / image.shape[1]))
        h = min(h, height - y - 10)
        if h < 20:
            break
        panel[y:y + h, 16:16 + w] = cv2.resize(image, (w, h), interpolation=cv2.INTER_AREA)
        y += h + 30
    return panel


class VisionTerminal:
    def __init__(self, eyes, window, arm=None, log=None, bin_id="B4"):
        """eyes: vision.Eyes. window: TerminalWindow. arm: motion.Arm, or None for the camera alone (no
        CALIBRATE or PICK). log: task.RunLog, for the calibration. bin_id: where PICK drops."""
        self.eyes, self.window, self.arm, self.log, self.bin_id = eyes, window, arm, log, bin_id
        self.camera = threading.Lock()      # the live view, or a job's pictures: one at a time
        self.bodies = []                    # the last live frame's bodies (normal confidence and up)
        self.subject = None                 # scene mm, followed frame to frame
        self.classified = []                # Classified readings
        self.looked = None                  # (time, PlateView) of the last SEGMENT
        self.job, self.job_name = None, None
        self.quitting = False
        self.live_note = ""
        self.window.show_arm(self.arm_line())
        self.window.say(READY if arm else "Camera only (no arm): click a resistor, SEGMENT or CLASSIFY")

    # The window's thread.

    def run(self):
        live = threading.Thread(target=self.live_loop, name="live", daemon=True)
        live.start()
        try:
            while True:
                pressed = self.window.poll(self.enabled())
                if pressed == "QUIT":
                    if not self.busy:
                        return
                    self.quitting = True
                    self.window.say("quitting once the job in hand is done...")
                elif isinstance(pressed, tuple):
                    self.select(pressed[1])
                elif pressed == "CALIBRATE":
                    self.launch("CALIBRATE", self.calibrate)
                elif pressed == "SEGMENT":
                    self.launch("SEGMENT", self.segment)
                elif pressed == "CLASSIFY":
                    self.launch("CLASSIFY", self.classify)
                elif pressed == "PICK":
                    self.launch("PICK", self.pick)
                if self.quitting and not self.busy:
                    return
        finally:
            self.quitting = True
            live.join(timeout=5)

    @property
    def busy(self):
        return self.job is not None and self.job.is_alive()

    def enabled(self):
        if self.busy:
            return set()
        on = {"SEGMENT"} | ({"CLASSIFY"} if self.subject is not None else set())
        if self.arm is not None:
            on |= {"CALIBRATE"} | ({"PICK"} if self.subject is not None and self.arm.last_started else set())
        return on

    def select(self, click_px):
        body = body_at(list(self.bodies), click_px)
        if body is None:
            self.window.say("no resistor there: click on one's outline")
            return
        self.subject = body.centre
        self.window.say(f"subject at ({body.centre[0]:.1f}, {body.centre[1]:.1f}) mm: CLASSIFY to read it properly"
                        + (f", PICK to put it in {self.bin_id}" if self.arm else ""))
        self.update_subject()

    def launch(self, name, job):
        def work():
            try:
                job()
            except Exception as why:        # shown, not swallowed
                self.window.say(f"{name} stopped: {type(why).__name__}: {why}")
                raise
        self.job_name = name
        self.job = threading.Thread(target=work, name=name, daemon=True)
        self.job.start()

    # The live view, in its own thread.

    def live_loop(self):
        while not self.quitting:
            if not self.camera.acquire(timeout=0.05):   # a job has the camera: show the frames it reads
                try:
                    self.live_while_busy()
                except Exception as why:
                    print(f"live view: {type(why).__name__}: {why}", flush=True)
                time.sleep(1.0 / LIVE_FPS)
                continue
            try:
                started = time.monotonic()
                try:
                    self.live_frame()
                except Exception as why:            # a bad frame mustn't end the view
                    self.window.say(f"live view: {type(why).__name__}: {why}")
                    time.sleep(0.5)
                self.live_note = f"{1.0 / max(time.monotonic() - started, 1e-3):.1f} frames/s"
            finally:
                self.camera.release()
            time.sleep(0.005)                       # lets a waiting job take the camera

    def live_while_busy(self):
        """The plate while a job has the camera: its newest frame, top-down if the plate is fitted, unmarked."""
        frame, cal = self.eyes.live()
        image = live_plate(frame, cal, note=f"LIVE: {self.job_name or 'a job'} has the camera, segmentation paused")
        if image is not None:
            self.window.show_live_image(image)

    def live_frame(self):
        e = self.eyes
        ok, frame = e.cap.read()
        if not ok:
            raise RuntimeError("the camera returned no frame")
        cal = e.tracker.update(frame)
        if cal is None:
            self.bodies = []
            self.window.show_live(live_plate(frame, None), 1.0, [f"live: finding the plate ({e.tracker.status()})"])
            return
        conf = e.detector.vision.conf
        found = e._segment(frame, cal)
        _, gains, white = closeup.white_of(cal, frame)
        bodies, faint = [], []
        for f in found:
            if f.score < conf:
                faint.append(f)
                continue
            crop = crops.cut(frame, cal, f.outline, gains).image
            reading = learned.decode(learned.log_probs(e.reader, crop, white), e.min_confidence)
            bodies.append(Body(f.outline, f.score, f.centre, f.direction, reading))
        self.bodies = bodies
        e.last_outlines = [b.outline for b in bodies] or e.last_outlines     # the focus check's fallback reference
        followed = nearest(bodies, self.subject, TRACK_MM)
        if followed is not None:
            self.subject = followed.centre
        image, scale = self.draw_live(frame, cal, bodies, faint, followed)
        self.window.show_live(image, scale, self.seg_lines(bodies, faint))
        self.update_subject(followed)

    def draw_live(self, frame, cal, bodies, faint, subject):
        rect, _ = rectify.white_balance(rectify.rectify(frame, cal), cal.settings)
        for f in faint:
            outline = np.round(f.outline).astype(np.int32)
            cv2.polylines(rect, [outline], True, FAINT, 1, cv2.LINE_AA)
            cv2.putText(rect, f"{f.score:.2f}", (int(outline[:, 0].max()) + 6, int(outline[:, 1].mean())), FONT, 0.7,
                        FAINT, 2, cv2.LINE_AA)
        if self.looked is not None:
            for r in self.looked[1].resistors:
                outline = np.round(r.outline_px).astype(np.int32)
                cv2.polylines(rect, [outline], True, LOOKED, 2, cv2.LINE_AA)
                cv2.putText(rect, f"look {r.score:.2f}", (int(outline[:, 0].min()) - 10, int(outline[:, 1].min()) - 14),
                            FONT, 0.7, LOOKED, 2, cv2.LINE_AA)
        for b in bodies:
            done = self.classified_for(b.centre)
            colour = SUBJECT if b is subject else CLASSIFIED if done else BODY
            outline = np.round(b.outline).astype(np.int32)
            cv2.polylines(rect, [outline], True, colour, 6 if b is subject else 3, cv2.LINE_AA)
            label = f"{b.score:.2f} " + (short(done.reading) if done else short(b.reading) + " prov.")
            cv2.putText(rect, label, (int(outline[:, 0].max()) + 8, int(outline[:, 1].mean())), FONT, 0.8, colour, 2,
                        cv2.LINE_AA)
        image = cv2.resize(rect, (PLATE_PX, PLATE_PX), interpolation=cv2.INTER_AREA)
        return image, rect.shape[1] / PLATE_PX

    def seg_lines(self, bodies, faint):
        live = (f"live: {len(bodies)} bodies ({', '.join(f'{b.score:.2f}' for b in bodies) or 'none'}), "
                f"{len(faint)} faint, {self.live_note}")
        if self.looked is None:
            return [live, "SEGMENT: not yet (the sorter's look: focus checked, 8 frames averaged)"]
        when, view = self.looked
        scores = ", ".join(f"{r.score:.2f}" for r in view.resistors) or "none"
        return [live, f"SEGMENT {when}: {len(view.resistors)} bodies ({scores}), {len(view.faint)} faint"]

    def classified_for(self, centre):
        near = [c for c in self.classified if math.dist(c.centre, centre) <= SAME_MM]
        return near[-1] if near else None

    def update_subject(self, body=None):
        if self.subject is None:
            self.window.show_subject([("click a resistor on the plate", FAINT)], [])
            return
        body = body or nearest(list(self.bodies), self.subject, TRACK_MM)
        x, y = self.subject
        lines = [(f"at ({x:.1f}, {y:.1f}) mm", (255, 255, 255))]
        if body is None:
            lines.append(("not seen in this frame", FAINT))
        else:
            lines.append((f"lying {body.direction % 180:.0f} deg, segmentation {body.score:.2f}", (255, 255, 255)))
            lines.append((f"PROVISIONAL: {describe(body.reading)}", BODY))
        done = self.classified_for(self.subject)
        lines.append((f"CLASSIFIED {done.when}: {describe(done.reading)}", CLASSIFIED) if done
                     else ("not classified: press CLASSIFY", FAINT))
        images = [("CLASSIFY: averaged picture", done.crop), ("CLASSIFY: close-up", done.close_crop)] if done else []
        self.window.show_subject(lines, images)

    # The jobs, one at a time, each in its own thread.

    def calibrate(self):
        self.window.say("CALIBRATE: the plate must be EMPTY, the arm set by hand after a power-on")
        with self.camera:
            try:
                result = StandardCalibration(self.arm, self.eyes, self.log, self.window.say,
                                             check_aim=camera_aimed(self.arm)).run()
            except CalibrationFailed as why:
                self.window.say(f"CALIBRATION FAILED: {why}")
                self.window.show_arm(self.arm_line())
                return
        self.window.show_arm(f"arm: {result.summary()}")
        self.window.say("Calibrated. Lay resistors on the plate and click one.")

    def segment(self):
        self.window.say("SEGMENT: the focus checked, 8 frames averaged, segmented...")
        with self.camera:
            view = self.eyes.look()
        self.looked = (time.strftime("%H:%M:%S"), view)
        self.window.say(f"SEGMENT: {len(view.resistors)} bodies in the sorter's look (white), {len(view.faint)} faint; "
                        f"focus: {view.focus}")

    def classify(self):
        e, target = self.eyes, self.subject
        self.window.say("CLASSIFY: checking the focus, then an averaged picture and a close-up of the subject...")
        with self.camera:
            frame, cal = e._picture(sharp=True)
            found = [f for f in e._segment(frame, cal) if f.score >= e.detector.vision.conf]
            hit = nearest([Body(f.outline, f.score, f.centre, f.direction) for f in found], target, TRACK_MM)
            if hit is None:
                self.window.say("CLASSIFY: the subject wasn't found in the averaged picture: is it still there?")
                return
            (lp, crop), = e._main_views(frame, cal, [hit.outline])
            (close_lp, close_crop), = e._closeups(cal, [hit.outline])
        reading = combine(lp, close_lp, e.min_confidence)
        self.classified.append(Classified(hit.centre, reading, crop, close_crop, time.strftime("%H:%M:%S")))
        self.subject = hit.centre
        self.update_subject()
        self.window.say(f"CLASSIFIED: {describe(reading)}" + ("" if close_lp is not None else " (no close-up)"))

    def pick(self):
        arm, target = self.arm, self.subject
        if not arm.navigator.parked:
            arm.park()
        self.window.say("PICK: finding the subject in an averaged picture...")
        with self.camera:
            frame, cal = self.eyes._picture()
            found = [f for f in self.eyes._segment(frame, cal) if f.score >= self.eyes.detector.vision.conf]
        hit = nearest([Body(f.outline, f.score, f.centre, f.direction) for f in found], target, TRACK_MM)
        if hit is None:
            self.window.say("PICK: the subject wasn't found in the averaged picture: is it still there?")
            return
        done = self.classified_for(hit.centre)
        r = Resistor(number=1, centre_mm=hit.centre, direction_deg=hit.direction, score=hit.score,
                     value_ohm=done.reading.value if done else None, best_ohm=0.0, confidence=0.0, outline_px=hit.outline)
        self.window.say(f"PICK: picking at ({hit.centre[0]:.1f}, {hit.centre[1]:.1f}) for {self.bin_id}")
        picked = arm.pick(r)
        if not picked and not arm.holding:
            self.window.say(f"PICK: not picked: {arm.message}")
            arm.park()
            return
        if not arm.drop(self.bin_id):
            self.window.say(f"PICK: COULDN'T DROP, STILL HOLDING: {arm.message}")
            return
        arm.park()
        self.subject = None
        self.update_subject()
        self.window.say(f"PICK: {'dropped in ' + self.bin_id if picked else 'pick stopped, jaws emptied in ' + self.bin_id}"
                        "; parked. Look whether it was taken.")

    def arm_line(self):
        if self.arm is None:
            return "arm: not connected (camera only)"
        started = self.arm.last_started
        return f"arm: started {started}" if started else "arm: NOT STARTED: press CALIBRATE"
