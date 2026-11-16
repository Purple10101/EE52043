"""The vision terminal: selecting, following and classifying a subject, and picking it, with fakes for the
camera, the window and the arm."""

from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("robot_offline.post.bands.learned")

from robot_control.task import vision_terminal  # noqa: E402
from robot_control.task.vision_terminal import Body, VisionTerminal, body_at, nearest, short  # noqa: E402


def square(cx, cy, half=20):
    return np.array([[cx - half, cy - half], [cx + half, cy - half], [cx + half, cy + half], [cx - half, cy + half]], float)


def reading(value=4700.0, best=4700.0, confidence=0.9, colours=("yellow", "violet", "red", "gold")):
    return SimpleNamespace(value=value, best=best, confidence=confidence, colours=colours)


class Window:
    def __init__(self):
        self.said, self.subject = [], None

    def say(self, text):
        self.said.append(text)

    def show_arm(self, text):
        pass

    def show_subject(self, lines, images):
        self.subject = (lines, images)

    def show_live(self, image, scale, lines):
        pass


class Eyes:
    """Finds one body per centre given, at the outline and direction given."""

    def __init__(self, found):
        self.found = found
        self.detector = SimpleNamespace(vision=SimpleNamespace(conf=0.5))
        self.min_confidence = 0.7
        self.pictures = []

    def _picture(self, sharp=True):
        self.pictures.append(sharp)
        return np.zeros((10, 10, 3), np.uint8), "cal"

    def _segment(self, frame, cal):
        return [SimpleNamespace(outline=square(*c), score=s, centre=c, direction=d) for c, s, d in self.found]

    def _main_views(self, frame, cal, polygons):
        return [("main lp", "main crop") for _ in polygons]

    def _closeups(self, cal, polygons):
        return [("close lp", "close crop") for _ in polygons]


class Arm:
    def __init__(self, picks=True):
        self.navigator = SimpleNamespace(parked=True)
        self.picks, self.done = picks, []
        self.holding, self.message, self.last_started = False, "", "2026-11-16 10:00"

    def pick(self, r):
        self.done.append(("pick", r.centre_mm, r.direction_deg))
        self.holding = self.picks
        return self.picks

    def drop(self, bin_id):
        self.done.append(("drop", bin_id))
        self.holding = False
        return True

    def park(self):
        self.done.append(("park",))


def terminal(found=(), arm=None):
    return VisionTerminal(Eyes(list(found)), Window(), arm=arm, bin_id="B4")


def test_nearest_keeps_to_its_distance():
    bodies = [Body(square(0, 0), 0.9, (0.0, 0.0), 0.0), Body(square(0, 0), 0.9, (10.0, 0.0), 0.0)]
    assert nearest(bodies, (8.0, 0.0), 8.0).centre == (10.0, 0.0)
    assert nearest(bodies, (30.0, 0.0), 8.0) is None
    assert nearest(bodies, None, 8.0) is None


def test_a_click_selects_the_body_under_it_or_near_its_outline():
    bodies = [Body(square(100, 100), 0.9, (1.0, 1.0), 0.0), Body(square(400, 100), 0.9, (2.0, 2.0), 0.0)]
    assert body_at(bodies, (105, 95)).centre == (1.0, 1.0)
    assert body_at(bodies, (470, 100)).centre == (2.0, 2.0)        # 50 px outside its outline
    assert body_at(bodies, (250, 400)) is None


def test_labels_mark_an_unsure_reading():
    assert short(reading()) == "4k7 90%"
    assert short(reading(value=None, best=1000.0, confidence=0.4)) == "1k? 40%"
    assert short(None) == "-"


def test_classify_and_pick_need_a_subject_and_pick_an_arm():
    t = terminal(arm=Arm())
    assert t.enabled() == {"SEGMENT", "CALIBRATE"}
    t.subject = (0.0, -130.0)
    assert t.enabled() == {"SEGMENT", "CALIBRATE", "CLASSIFY", "PICK"}
    camera_only = terminal()
    camera_only.subject = (0.0, -130.0)
    assert camera_only.enabled() == {"SEGMENT", "CLASSIFY"}


def test_classify_reads_the_subject_from_a_focused_picture_and_its_closeup(monkeypatch):
    got = []
    monkeypatch.setattr(vision_terminal, "combine", lambda lp, close, limit: got.append((lp, close, limit)) or reading())
    t = terminal(found=[((0.0, -130.0), 0.9, 10.0), ((30.0, -130.0), 0.9, 0.0)])
    t.subject = (1.0, -131.0)
    t.classify()
    assert t.eyes.pictures == [True]                              # the focus checked before the picture
    assert got == [("main lp", "close lp", 0.7)]
    assert t.classified[0].centre == (0.0, -130.0) and t.classified[0].close_crop == "close crop"
    assert t.classified_for((2.0, -131.0)) is t.classified[0]
    assert t.classified_for((30.0, -130.0)) is None
    assert t.window.said[-1].startswith("CLASSIFIED: 4k7 90%")


def test_classify_says_so_when_the_subject_has_gone():
    t = terminal(found=[((30.0, -130.0), 0.9, 0.0)])
    t.subject = (0.0, -130.0)
    t.classify()
    assert not t.classified
    assert "wasn't found" in t.window.said[-1]


def test_pick_takes_the_subject_to_the_bin_and_parks():
    arm = Arm()
    t = terminal(found=[((0.5, -129.5), 0.9, 25.0)], arm=arm)
    t.subject = (0.0, -130.0)
    t.pick()
    assert arm.done == [("pick", (0.5, -129.5), 25.0), ("drop", "B4"), ("park",)]
    assert t.subject is None


def test_a_refused_pick_parks_without_a_drop():
    arm = Arm(picks=False)
    t = terminal(found=[((0.5, -129.5), 0.9, 25.0)], arm=arm)
    t.subject = (0.0, -130.0)
    t.pick()
    assert arm.done == [("pick", (0.5, -129.5), 25.0), ("park",)]
    assert "not picked" in t.window.said[-1]
