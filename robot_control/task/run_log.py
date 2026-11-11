"""RunLog: a record of a sorting session on disk, enough to work out afterwards what the camera saw."""

import csv
import json
import time
from pathlib import Path

import cv2
from robot_offline.post.bands.learned import ohms_text

LOGS = Path(__file__).resolve().parents[2] / "logs" / "sort_runs"
READING_FIELDS = ["time", "look", "run", "cycle", "kind", "number", "body", "x_mm", "y_mm", "score", "value",
                  "best", "confidence", "closeup", "colours", "fate"]


class RunLog:
    """One folder per session (the window opened), logs/sort_runs/<start time>/:

    - events.csv: one line for each thing that happened (looks, picks, drops, how each pick went, the end);
    - readings.csv: every look, a line per body (its reading: value, best guess, confidence, colours, whether a
      close-up is in it, what became of it), per detection too faint to be a body, and per body missing;
    - look_NNNN.jpg: what the window showed at each look;
    - frames/NNNN.jpg and NNNN_rectify.json: each look's averaged camera frame and the plate's fit, so robot_offline's
      tools can be run on it again (JPEG at 95: the full PNG is about 20 MB);
    - crops/NNNN_bodyB.png: each body's band crop at that look, and crops/bodyB_closeup.png its close-up's.

    Looks are numbered through the session (NNNN), so pressing START again never overwrites anything.
    """

    def __init__(self, root=LOGS, keep_frames=True):
        self.folder = Path(root) / time.strftime("%Y%m%d_%H%M%S")
        for sub in ("", "frames", "crops"):
            (self.folder / sub).mkdir(parents=True, exist_ok=True)
        self.keep_frames = keep_frames
        self.run, self.seq = 0, 0
        self.events = self.folder / "events.csv"
        self.readings = self.folder / "readings.csv"
        with self.events.open("w", newline="") as f:
            csv.writer(f).writerow(["time", "run", "cycle", "event", "details"])
        with self.readings.open("w", newline="") as f:
            csv.writer(f).writerow(READING_FIELDS)

    def start_run(self):
        """A new START: its events and looks are marked with the next run number."""
        self.run += 1

    def event(self, cycle, what, **details):
        with self.events.open("a", newline="") as f:
            csv.writer(f).writerow([time.strftime("%H:%M:%S"), self.run, cycle, what, json.dumps(details, default=str)])

    def look(self, cycle, view, plan, image=None):
        """Everything about one look: the window's picture, the averaged frame, every reading and crop."""
        self.seq += 1
        n = f"{self.seq:04d}"
        if image is not None:
            cv2.imwrite(str(self.folder / f"look_{n}.jpg"), image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if self.keep_frames and view.frame is not None:
            cv2.imwrite(str(self.folder / "frames" / f"{n}.jpg"), view.frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if view.cal is not None and hasattr(view.cal, "save"):
                view.cal.save(self.folder / "frames" / f"{n}_rectify.json")
        rows = []
        stamp = time.strftime("%H:%M:%S")
        for r in view.resistors:
            rows.append([stamp, n, self.run, cycle, "body", r.number, r.body_id, round(r.centre_mm[0], 1),
                         round(r.centre_mm[1], 1), round(r.score, 3), r.value_ohm, r.best_ohm, round(r.confidence, 3),
                         r.closeup, " ".join(r.colours), plan.fates.get(r.number, "")])
            if r.crop is not None:
                cv2.imwrite(str(self.folder / "crops" / f"{n}_body{r.body_id}.png"), r.crop)
            closeup_path = self.folder / "crops" / f"body{r.body_id}_closeup.png"
            if r.closeup_crop is not None and not closeup_path.exists():
                cv2.imwrite(str(closeup_path), r.closeup_crop)
        for _, score, centre in view.faint:
            rows.append([stamp, n, self.run, cycle, "faint", "", "", round(centre[0], 1), round(centre[1], 1),
                         round(score, 3), "", "", "", "", "", "too faint to be a new body"])
        for centre in view.missing:
            rows.append([stamp, n, self.run, cycle, "missing", "", "", round(centre[0], 1), round(centre[1], 1),
                         "", "", "", "", "", "", "seen before, not found in this look"])
        with self.readings.open("a", newline="") as f:
            csv.writer(f).writerows(rows)
        return n


def describe_reading(r):
    """A body's reading in a few words, for messages: '1k5 (0.93)' or 'unsure, best 1k at 0.55'."""
    if r.value_ohm is not None:
        return f"{ohms_text(r.value_ohm)} ({r.confidence:.2f})"
    return f"unsure, best {ohms_text(r.best_ohm)} at {r.confidence:.2f}"
