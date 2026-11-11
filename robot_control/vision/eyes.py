"""Eyes: everything the camera can tell the robot about the plate, in one call.

    eyes = Eyes()               # opens the camera, loads the segmentation model and the band reader
    view = eyes.look()          # a PlateView: every resistor on the plate, where it lies and its value
    eyes.picked(centre_mm)      # the arm has just picked the body there
    eyes.close()

A look runs robot_offline's pipeline, through its API, as its tools/read_plate.py does for a still plate:
1. The plate: plate_search.Tracker finds it once and keeps it, and keeps the exposure where the band
   reader wants the plate (learned.READING_LEVEL, which segmentation is fine with too). Frames go through
   it until the plate is found and the exposure has settled.
2. The picture: closeup.SETTLE more frames read and dropped (anything buffered from before the arm
   parked is gone by then), then AVERAGE_FRAMES averaged (closeup.mean_frame), the exposure held still.
   One frame is noisy; the mean of 8 is what the band reader was trained and scored on (the resistor kit
   was collected that way, robot_offline tools/collect_bands.py), and segmentation's scores swing less on
   it. Averaging only works on a still scene: anything moving in view (the arm, a hand) blurs.
3. The bodies: the segmentation model on the averaged picture's rectified plate (tiling.predict_frame),
   run down to body_memory.FLOOR so that a body already known still counts when its score dips.
4. Where each lies: its skeleton's centroid and spine, in scene mm (vision.bodies).
5. Which body it is: BodyMemory follows each body from look to look by where it lies. A new body needs
   the normal confidence; a known one counts while it's found at all; one found nowhere is "missing" until
   it's been missed twice in a row (PlateView.missing), and the run looks again rather than stop.
6. Its close-up: one focus can't suit the whole plate (the front is sharpest at focus 322, the back at
   about 295), and the bands need it sharp. So each body also gets a capture of its own at the focus its
   distance from the lens needs (post.bands.closeup.take: about 1.5 s a body), once, the first time it's
   seen. The first look of a run takes the close-ups; later looks only take them for new bodies.
7. Its value: the band reader on the body's crop from the averaged picture, averaged with its last few
   looks' and its close-up's and decoded once (BodyMemory, as robot_offline's learned.Views). That is how
   the deployed reader was scored (first capture and close-up averaged per body, robot_offline
   experiments/2026-11-11_band_reader_kit), and one look's slip can't flip a reading on its own. A reading
   under the reader's limit gives value_ohm None: the body is seen but its value isn't trusted.

Each Resistor keeps its band crops, and the PlateView the detections too faint to be bodies and the
bodies missing, so a run's log can show what the camera saw (task.RunLog).

Look only with the arm parked: over the plate it hides bodies, spoils the plate's fit and blurs the average.
"""

import time

from robot_offline import config as offline_config
from robot_offline.inf import tiling
from robot_offline.post import skeleton
from robot_offline.post.bands import closeup, crops, learned, values
from robot_offline.pre import plate_search, rectify

from robot_control.config import RigSettings, VisionSettings, WorkspaceSettings
from robot_control.vision.body_memory import FLOOR, BodyMemory
from robot_control.vision.bodies import body_direction, centroid_mm
from robot_control.vision.detector import Detector, camera_mm
from robot_control.world import PlateView, Resistor

AVERAGE_FRAMES = 8      # frames averaged into each look's picture, as robot_offline's read_plate and collect_bands
FIND_S = 60.0           # how long a look waits for the plate and the exposure before giving up
CLOSEUP_S = 1.5         # about how long one close-up takes (robot_offline's figure), for telling the user


class EyesError(Exception):
    """The camera couldn't give a usable look (no frames, or the plate not found)."""


class Eyes:
    def __init__(self, detector=None, reader=None, cap=None, closeups=True, say=print):
        """detector, reader, cap: for tests; by default the configured model, the band reader and the camera.
        closeups: take a close-up of each body (see above); False reads from the averaged picture alone.
        say: where to tell what a look is doing (close-ups take a while)."""
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
        self.memory = BodyMemory(self.min_confidence, conf=self.detector.vision.conf)
        self.closeup_crops = {}         # body id: its close-up's band crop, for the log
        if cap is None:
            from robot_offline.pre.camera import open_camera
            print("Opening the camera (a few seconds while exposure and white balance settle)...", flush=True)
            cap = open_camera(camera)
        self.cap = cap
        # Found once and held: the rig doesn't move during a run, and the arm would spoil refits.
        self.tracker = plate_search.Tracker(cap, rectify.Settings.from_config(), camera=camera,
                                            refit_seconds=float("inf"), level=learned.READING_LEVEL)

    def look(self):
        """A PlateView of the plate as it is now. Raises EyesError if the plate can't be found."""
        frame, cal = self._picture()
        polygons, scores = self._find_bodies(frame, cal)
        lens = camera_mm(cal)
        found = []                      # (outline, score, skeleton, centre mm): everything with an area
        for poly, score in zip(polygons, scores):
            body = skeleton.body_skeleton(poly)
            if body is not None:
                found.append((poly, float(score), body, tuple(float(v) for v in centroid_mm(poly, body, cal, lens))))
        ids = self.memory.match([(f[3], f[1]) for f in found])
        bodies = [(f, i) for f, i in zip(found, ids) if i is not None]
        faint = [(f[0], f[1], f[3]) for f, i in zip(found, ids) if i is None]
        views = self._main_views(frame, cal, [f[0] for f, _ in bodies])
        self._closeups(cal, bodies)
        self.closeup_crops = {i: c for i, c in self.closeup_crops.items() if any(b.id == i for b in self.memory.bodies)}
        resistors = []
        for ((poly, score, body, centre), body_id), (lp, crop) in zip(bodies, views):
            reading = self.memory.read(body_id, lp)
            resistors.append(Resistor(
                number=len(resistors) + 1,
                centre_mm=centre,
                direction_deg=body_direction(body, cal.settings),
                score=score,
                value_ohm=reading.value,
                best_ohm=float(reading.best),
                confidence=float(reading.confidence),
                outline_px=poly,
                closeup=self.memory.has_closeup(body_id),
                colours=tuple(reading.colours),
                body_id=body_id,
                crop=crop,
                closeup_crop=self.closeup_crops.get(body_id),
            ))
        missing = [b.centre_mm for b in self.memory.missing()]
        return PlateView(time.strftime("%Y%m%d-%H%M%S"), frame, cal, resistors, faint=faint, missing=missing)

    def picked(self, centre_mm):
        """The arm has just picked the body at centre_mm. If the next look finds anything there, even faint,
        the pick failed and it's still that body; if not, it's gone."""
        self.memory.picked_near(centre_mm)

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

    def _closeups(self, cal, bodies):
        """A close-up of every body that hasn't had one, pinned into its reading."""
        if not self.closeups_on:
            return
        new = [(f, i) for f, i in bodies if not self.memory.has_closeup(i)]
        if not new:
            return
        self.say(f"close-ups of {len(new)} new resistor{'s' * (len(new) != 1)}, about {CLOSEUP_S * len(new):.0f} s")
        d = self.detector
        taken = closeup.take(self.cap, cal, [f[0] for f, _ in new], seg=d.model, grey=d.grey, camera=self.camera)
        for (_, body_id), k in zip(new, taken):
            self.memory.pin_closeup(body_id, learned.log_probs(self.reader, k.crop.image, k.white))
            self.closeup_crops[body_id] = k.crop.image

    def _picture(self):
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
        for _ in range(closeup.SETTLE):             # fresh frames only: nothing from before the arm parked
            self.cap.read()
        return closeup.mean_frame(self.cap, AVERAGE_FRAMES), cal

    def _find_bodies(self, frame, cal):
        """(outlines in rectified px, scores) from the segmentation model, down to FLOOR."""
        d = self.detector
        model_view, _ = rectify.model_input(frame, cal)
        tiles = tiling.tiles(d.work_area, (model_view.shape[1], model_view.shape[0]), d.tile_size, d.tile_overlap)
        return tiling.predict_frame(d.model, model_view, tiles, conf=FLOOR, grey=d.grey)
