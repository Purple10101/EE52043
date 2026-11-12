"""Eyes: everything the camera can tell the robot about the plate.

    eyes = Eyes()               # opens the camera, loads the segmentation model and the band reader
    view = eyes.read_values()   # at the start of a run: every resistor found AND its value read, once
    view = eyes.look()          # each time the arm is parked: every resistor found, with the value it was read as
    eyes.picked(centre_mm)      # the arm has just picked the body there
    eyes.close()

**Finding the resistors: every look** (look), through robot_offline's API, as its tools/read_plate.py does for
a still plate:
1. The plate: plate_search.Tracker finds it once and keeps it, and keeps the exposure where the band reader
   wants the plate (learned.READING_LEVEL, which segmentation is fine with too). Frames go through it until
   the plate is found and the exposure has settled.
2. The picture: closeup.SETTLE more frames read and dropped (anything buffered from before the arm parked is
   gone by then), then AVERAGE_FRAMES averaged (closeup.mean_frame), the exposure held still. Averaging only
   works on a still scene: anything moving in view (the arm, a hand) blurs.
3. The bodies: the segmentation model on the averaged picture (tiling.predict_frame), down to
   body_memory.FLOOR so a body already known still counts when its score dips; each one's centroid and long
   axis in scene mm (vision.bodies); followed from look to look by BodyMemory.

**Reading the values: once, at the start of a run** (read_values). The camera's focus drifts while it is in
use, about 4 settings a minute, and the bands need it sharp (robot_offline handoff
2026-11-11_ee52043_read_once_plan.md), so the band reader runs once, straight after a focus calibration:
4. The focus: closeup.calibrate sweeps the focus over the bodies found and fits the close-up rule afresh
   (closeup.FocusModel). Kept for this read only: robot_offline's camera.toml isn't touched. An interim step
   until robot_offline's focus check on the plate's centre cross (pre/focus.py) exists.
5. Each body read twice: its crop from the averaged picture (at camera.toml's fixed focus), and a close-up at
   the focus its distance needs by the fresh rule (closeup.take, about 1.5 s a body). Combined (combine): their
   log-probabilities averaged half and half and decoded once, as the reader was scored; where the two
   disagree, the close-up's reading alone, as the handoff says while the main capture's focus is fixed.
6. Each reading kept with its body (BodyMemory.set_reading). From then on a body keeps its value while it
   stays where it was read. One that was unsure, has moved, or wasn't there at the read has no value: it is
   to be measured (Resistor.note says why).

**Focus before every capture** (robot_offline handoff 2026-11-11_ee52043_focus_before_capture.md): the focus
number doesn't hold the lens, which drifts as the camera warms (about 299 to 337 from cold), so every capture
(_picture: looks, the read, the empty-plate check, the aim check's tip pictures) is taken right after a focus
check on the plate's centre cross, by robot_offline's FocusTracker (pre/focus.py, landed 2026-11-11 2117c9d): a
full search the first time (16.4 s on the rig), a narrow re-check after (about 3.5 s). If the cross is covered, the
last look's bodies are its fallback reference. Close-ups then follow the rule plus the check's offset (passed to
closeup.take), so the read's own focus sweep (step 4) is skipped. On the rig, with the check, segmentation found 8
of 8 kit parts against 5 of 8 at the fixed 306, and crops were 3-4 times sharper. Without pre/focus.py (an older
robot_offline) there is no check: captures are at camera.toml's fixed focus and step 4's sweep stands in for the
close-ups. Each PlateView's `focus` says which (the tracker's status, or that there is no check).

Each Resistor keeps its band crops from the read, and the PlateView the detections too faint to be bodies and
the bodies missing, so a run's log can show what the camera saw (task.RunLog).

Look only with the arm parked: over the plate it hides bodies, spoils the plate's fit and blurs the average.
"""

import time
from dataclasses import dataclass

import numpy as np
from robot_offline import config as offline_config
from robot_offline.inf import tiling
from robot_offline.post import skeleton
from robot_offline.post.bands import closeup, crops, learned, values
from robot_offline.pre import plate_search, rectify

try:                                    # robot_offline's focus check on the centre cross, once it exists
    from robot_offline.pre import focus as focus_api
except ImportError:
    focus_api = None

from robot_control.config import RigSettings, VisionSettings, WorkspaceSettings
from robot_control.vision.body_memory import FLOOR, BodyMemory
from robot_control.vision.bodies import body_direction, centroid_mm
from robot_control.vision.detector import Detector, camera_mm
from robot_control.world import PlateView, Resistor

AVERAGE_FRAMES = 8      # frames averaged into each look's picture, as robot_offline's read_plate and collect_bands
FIND_S = 60.0           # how long a look waits for the plate and the exposure before giving up
CLOSEUP_S = 1.5         # about how long one close-up takes (robot_offline's figure), for telling the user
SWEEP_S = 25.0          # about how long the focus sweep takes, for telling the user
NO_FOCUS_CHECK = "fixed focus: no focus check (robot_offline pre/focus.py not installed yet)"
# During a run, a look only finds the parts (values are read once, at START, always after a check), and
# segmentation copes with a few settings of drift; warm, the lens drifts about 0.5 a minute. So a run's looks
# re-check focus only when the last check is this old. On 2026-11-11 21:57 every look checked, and the check
# often fell back to its full 16 s search: 21-25 s parked per look, against 6-9 s with the quick re-check.
FOCUS_RECHECK_S = 90.0


class EyesError(Exception):
    """The camera couldn't give a usable look (no frames, or the plate not found)."""


def combine(main, close, min_confidence):
    """A body's Reading from its two views' log-probabilities: the averaged picture's (main) and its
    close-up's (close, or None without one). Where both name the same value, averaged half and half and
    decoded once, as learned.Views does with a pinned close-up and as the reader was scored. Where they
    disagree, the close-up's alone: its focus is fitted afresh, the main capture's is fixed and drifts."""
    if close is None:
        return learned.decode(main, min_confidence)
    from_close = learned.decode(close, min_confidence)
    if learned.decode(main, min_confidence).best != from_close.best:
        return from_close
    return learned.decode(np.mean([main, close], axis=0), min_confidence)


@dataclass
class _Found:
    """One detection in a look."""
    outline: np.ndarray
    score: float
    skeleton: object
    centre: tuple
    direction: float


class Eyes:
    def __init__(self, detector=None, reader=None, cap=None, closeups=True, say=print):
        """detector, reader, cap: for tests; by default the configured model, the band reader and the camera.
        closeups: a focus sweep and a close-up of each body at the read; False reads from the averaged
        picture alone. say: where to tell what a read is doing (it takes a while)."""
        self.detector = detector or Detector(VisionSettings.from_config(), RigSettings.from_config(),
                                             WorkspaceSettings.from_config())
        print("Loading the segmentation model and the band reader...", flush=True)
        self.detector.model
        self.reader = reader or values.load()
        camera = offline_config.load("camera")
        self.camera = camera
        self.closeups_on, self.say = closeups, say
        # The sorting run's own limit (vision.toml min_confidence) if set, else the reader's.
        limit = getattr(self.detector.vision, "min_confidence", None)
        self.min_confidence = limit if limit is not None else self.reader.min_confidence
        self.memory = BodyMemory(conf=self.detector.vision.conf)
        self.crops = {}                 # body id: (its crop from the read's picture, its close-up's or None)
        self.focus = None               # robot_offline's FocusTracker, made when the plate is first found
        self.last_outlines = []         # the last look's bodies (rectified px): the focus check's fallback reference
        self.focus_note = NO_FOCUS_CHECK    # the last capture's focus, as its tracker said
        self.focus_checked_at = None    # time.monotonic() of the last focus check
        if cap is None:
            from robot_offline.pre.camera import open_camera
            print("Opening the camera (a few seconds while exposure and white balance settle)...", flush=True)
            cap = open_camera(camera)
        self.cap = cap
        # Found once and held: the rig doesn't move during a run, and the arm would spoil refits.
        self.tracker = plate_search.Tracker(cap, rectify.Settings.from_config(), camera=camera,
                                            refit_seconds=float("inf"), level=learned.READING_LEVEL)

    def look(self, sharp=False):
        """A PlateView of the plate as it is now: every resistor found and followed, each with the value it
        was read as at the start of the run (or none, and why). Raises EyesError if the plate can't be found."""
        frame, cal = self._picture(sharp)
        found = self._segment(frame, cal)
        self.last_outlines = [f.outline for f in found if f.score >= self.detector.vision.conf]
        ids = self.memory.match([(f.centre, f.score, f.direction) for f in found])
        followed = [(f, i) for f, i in zip(found, ids) if i is not None]
        resistors = [self._resistor(n, f, i) for n, (f, i) in enumerate(followed, start=1)]
        faint = [(f.outline, f.score, f.centre) for f, i in zip(found, ids) if i is None]
        missing = [b.centre_mm for b in self.memory.missing()]
        return PlateView(time.strftime("%Y%m%d-%H%M%S"), frame, cal, resistors, faint=faint, missing=missing,
                         focus=self.focus_note)

    def read_values(self):
        """The start-of-run read: a look, a focus sweep over the bodies found, then every body read from the
        look's averaged picture and its own close-up. Keeps each reading with its body for the run, and
        returns the look with the values in."""
        view = self.look(sharp=True)          # the bands need the focus checked right before this capture
        if not view.resistors:
            return view
        outlines = [r.outline_px for r in view.resistors]
        main = self._main_views(view.frame, view.cal, outlines)
        close = self._closeups(view.cal, outlines)
        for r, (lp, crop), (close_lp, close_crop) in zip(view.resistors, main, close):
            self.memory.set_reading(r.body_id, combine(lp, close_lp, self.min_confidence))
            self.crops[r.body_id] = (crop, close_crop)
        read = [self._resistor(r.number, _Found(r.outline_px, r.score, None, r.centre_mm, r.direction_deg), r.body_id)
                for r in view.resistors]
        return PlateView(view.id, view.frame, view.cal, read, faint=view.faint, missing=view.missing, focus=view.focus)

    def picked(self, centre_mm):
        """The arm has just picked the body at centre_mm. If the next look finds anything there, even faint,
        the pick failed and it's still that body; if not, it's gone."""
        self.memory.picked_near(centre_mm)

    def count_bodies(self):
        """How many bodies segmentation finds on the plate now, at the normal confidence: a quick check that
        the plate is empty. Leaves the followed bodies and their readings alone."""
        frame, cal = self._picture()
        _, scores = self._find_bodies(frame, cal)
        return sum(s >= self.detector.vision.conf for s in scores)

    def find_tip(self, near_mm, height_mm=0.0):
        """Where the closed gripper's tip is now (scene mm), height_mm above the plate, near near_mm: from an
        averaged picture, by the calibration tools' tip finder (vision/tip.py). The arm must hold still with
        its jaws closed and face-on to the camera. Raises tip.TipNotFound or EyesError."""
        from robot_control.vision.tip import find_tip
        frame, cal = self._picture()
        return find_tip(frame, cal, near_mm, height_mm)

    def lens_mm(self):
        """Where the plate's fit puts the camera, scene mm (finding the plate first if it hasn't been)."""
        cal = self.tracker.cal if self.tracker.cal is not None else self._picture()[1]
        return camera_mm(cal)

    def plate_status(self):
        return self.tracker.status()

    def close(self):
        self.tracker.close()
        self.cap.release()

    # The steps.

    def _resistor(self, number, found, body_id):
        """A Resistor for a followed body, with the value it was read as if it still holds."""
        reading = self.memory.reading(body_id)
        crop, close_crop = self.crops.get(body_id, (None, None))
        return Resistor(
            number=number,
            centre_mm=found.centre,
            direction_deg=found.direction,
            score=found.score,
            value_ohm=None if reading is None else reading.value,
            best_ohm=0.0 if reading is None else float(reading.best),
            confidence=0.0 if reading is None else float(reading.confidence),
            outline_px=found.outline,
            closeup=close_crop is not None,
            colours=() if reading is None else tuple(reading.colours),
            body_id=body_id,
            crop=crop,
            closeup_crop=close_crop,
            note=self.memory.why_no_value(body_id),
        )

    def _segment(self, frame, cal):
        """Every detection with an area to aim at, down to FLOOR: outline, score, skeleton, centre, direction."""
        polygons, scores = self._find_bodies(frame, cal)
        lens = camera_mm(cal)
        found = []
        for poly, score in zip(polygons, scores):
            body = skeleton.body_skeleton(poly)
            if body is not None:
                centre = tuple(float(v) for v in centroid_mm(poly, body, cal, lens))
                found.append(_Found(poly, float(score), body, centre, body_direction(body, cal.settings)))
        return found

    def _main_views(self, frame, cal, polygons):
        """(log-probabilities, band crop) per body from the averaged picture: its crop cut from the raw frame
        with the plate's white balance, as values.read_frame cuts it."""
        if not polygons:
            return []
        _, gains, white = closeup.white_of(cal, frame)
        out = []
        for poly in polygons:
            crop = crops.cut(frame, cal, poly, gains).image
            out.append((learned.log_probs(self.reader, crop, white), crop))
        return out

    def _closeups(self, cal, polygons):
        """(log-probabilities, crop) of each body's close-up, after a focus sweep over them all; (None, None)
        each with close-ups off."""
        if not self.closeups_on:
            return [(None, None)] * len(polygons)
        n = len(polygons)
        sweep = "" if self.focus is not None else f"a focus sweep over them (about {SWEEP_S:.0f} s), then "
        self.say(f"reading the values of {n} resistor{'s' * (n != 1)}: {sweep}a close-up of each "
                 f"(about {CLOSEUP_S * n:.0f} s)")
        # With the focus checked before the capture, closeup.take applies the check's offset to the rule
        # itself; without a check, the rule is fitted afresh by a sweep over these bodies.
        model = None if self.focus is not None else self._focus_model(cal, polygons)
        d = self.detector
        offset = self.focus.offset if self.focus is not None else None      # the check's, before this read
        taken = closeup.take(self.cap, cal, polygons, seg=d.model, grey=d.grey, model=model, camera=self.camera,
                             offset=offset)
        return [(learned.log_probs(self.reader, k.crop.image, k.white), k.crop.image) for k in taken]

    def _focus_model(self, cal, polygons):
        """The close-up rule fitted afresh to these bodies by a focus sweep (closeup.calibrate); camera.toml's
        if the sweep found no best focus for any of them."""
        points, _ = closeup.calibrate(self.cap, cal, polygons, camera=self.camera)
        found = [(dist, focus) for dist, focus in points if focus is not None]
        if not found:
            self.say("focus sweep: no body's best focus found, using camera.toml's close-up rule")
            return closeup.FocusModel.from_config(self.camera)
        return closeup.FocusModel(found)

    def _picture(self, sharp=True):
        """(averaged frame, plate calibration), taken as read_plate takes one: frames through the plate's
        tracker until the plate is found and its exposure has settled, closeup.SETTLE more dropped, then the
        mean of AVERAGE_FRAMES. Raises EyesError if the plate isn't found and settled within FIND_S."""
        deadline = time.monotonic() + FIND_S
        while True:
            ok, frame = self.cap.read()
            if not ok:
                raise EyesError("the camera returned no frame")
            cal = self.tracker.update(frame)
            control = self.tracker.control        # None for a camera it can't set (the twin, saved frames)
            if cal is not None and not self.tracker.searching and (control is None or control.settled):
                break
            if time.monotonic() > deadline:
                raise EyesError(f"the plate wasn't found and settled in {FIND_S:.0f} s: {self.tracker.status()}")
        if sharp or self.focus_checked_at is None or time.monotonic() - self.focus_checked_at > FOCUS_RECHECK_S:
            self._check_focus(cal)
        for _ in range(closeup.SETTLE):             # fresh frames only: nothing from before the arm parked
            self.cap.read()
        return closeup.mean_frame(self.cap, AVERAGE_FRAMES), cal

    def _check_focus(self, cal):
        """The focus checked on the centre cross and set, right before a capture, if robot_offline has the check
        (a full search the first time, a narrow re-check after). Without it, nothing: camera.toml's fixed focus."""
        if focus_api is None:
            return
        if self.focus is None:
            self.focus = focus_api.FocusTracker(self.cap, cal)
        self.focus.cal = cal                    # the plate's fit now (held for the session, but say so)
        self.focus.check(polygons=self.last_outlines or None)
        self.focus_note = self.focus.status()
        self.focus_checked_at = time.monotonic()

    def _find_bodies(self, frame, cal):
        """(outlines in rectified px, scores) from the segmentation model, down to FLOOR."""
        d = self.detector
        model_view, _ = rectify.model_input(frame, cal)
        tiles = tiling.tiles(d.work_area, (model_view.shape[1], model_view.shape[0]), d.tile_size, d.tile_overlap)
        return tiling.predict_frame(d.model, model_view, tiles, conf=FLOOR, grey=d.grey)
