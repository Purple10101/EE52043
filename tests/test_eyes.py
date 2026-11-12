"""Eyes' picture: the plate found and settled, stale frames dropped, then several frames averaged."""

import numpy as np
import pytest

pytest.importorskip("robot_offline.post.bands.closeup")

from robot_offline.post.bands import closeup  # noqa: E402

from robot_control.vision import eyes as eyes_module  # noqa: E402
from robot_control.vision.eyes import AVERAGE_FRAMES, Eyes, EyesError  # noqa: E402


class CountingCamera:
    """Frame i is filled with the value i (small frames), so which frames were used shows in the mean."""

    def __init__(self):
        self.i = -1

    def read(self):
        self.i += 1
        return True, np.full((4, 6, 3), self.i % 256, np.uint8)


class Control:
    def __init__(self, settles_at):
        self.settles_at, self.reads = settles_at, 0

    @property
    def settled(self):
        return self.reads >= self.settles_at


class FakeTracker:
    """Finds the plate on the found_at-th frame; the exposure settles on the settles_at-th."""

    def __init__(self, found_at=3, settles_at=5, control=True):
        self.found_at, self.seen = found_at, 0
        self.control = Control(settles_at) if control else None
        self.searching = True

    def update(self, frame):
        self.seen += 1
        if self.control:
            self.control.reads = self.seen
        found = self.seen >= self.found_at
        self.searching = not found
        return "cal" if found else None

    def status(self):
        return "looking"


def eyes_with(tracker, camera=None):
    eyes = Eyes.__new__(Eyes)            # no models or real camera
    eyes.cap, eyes.tracker = camera or CountingCamera(), tracker
    eyes.focus, eyes.focus_note, eyes.last_outlines = None, "", []
    return eyes


def test_a_look_averages_fresh_frames_once_the_plate_and_exposure_have_settled(monkeypatch):
    monkeypatch.setattr(eyes_mod, "focus_api", None)        # the focus check has its own tests
    eyes = eyes_with(FakeTracker(found_at=3, settles_at=5))
    frame, cal = eyes._picture()
    # frames 0-4 go to the tracker (settled on the 5th), 5 to 5 + SETTLE - 1 are dropped, then the next 8
    first = 5 + closeup.SETTLE
    expected = np.mean(range(first, first + AVERAGE_FRAMES))
    assert cal == "cal"
    assert frame.dtype == np.uint8 and frame.shape == (4, 6, 3)
    assert frame[0, 0, 0] == pytest.approx(expected, abs=0.5)
    assert eyes.cap.i == first + AVERAGE_FRAMES - 1      # nothing read beyond the average


def test_a_camera_that_cant_be_set_needs_only_the_plate(monkeypatch):
    monkeypatch.setattr(eyes_mod, "focus_api", None)        # the focus check has its own tests
    eyes = eyes_with(FakeTracker(found_at=2, control=False))
    frame, _ = eyes._picture()
    first = 2 + closeup.SETTLE
    assert frame[0, 0, 0] == pytest.approx(np.mean(range(first, first + AVERAGE_FRAMES)), abs=0.5)


def test_no_plate_in_time_is_an_error(monkeypatch):
    clock = iter(range(0, 1000, 10))
    monkeypatch.setattr(eyes_module.time, "monotonic", lambda: next(clock))
    eyes = eyes_with(FakeTracker(found_at=10 ** 9))
    with pytest.raises(EyesError, match="wasn't found"):
        eyes._picture()


def test_averaging_takes_out_noise():
    rng = np.random.default_rng(0)

    class NoisyCamera:
        def read(self):
            return True, np.clip(128 + rng.normal(0, 10, (40, 40, 3)), 0, 255).astype(np.uint8)

    one = NoisyCamera().read()[1].astype(float)
    mean = closeup.mean_frame(NoisyCamera(), AVERAGE_FRAMES).astype(float)
    assert mean.std() < one.std() / 2          # 8 frames: noise down by about the square root of 8


# Following bodies from look to look (BodyMemory), and the values read once at the start of a run

from robot_offline.post.bands import learned  # noqa: E402

from robot_control.vision import eyes as eyes_mod  # noqa: E402
from robot_control.vision.body_memory import GONE_LOOKS, BodyMemory  # noqa: E402
from robot_control.vision.eyes import combine  # noqa: E402


def scores_for(ohms, strength):
    """BandCTC-style scores (1-D, per value and way round) that favour `ohms` by `strength`."""
    s = np.zeros(2 * len(learned._OHMS))
    s[int(np.flatnonzero(np.isclose(learned._OHMS, ohms))[0])] = strength
    return s


def at(x, y, score=0.9, direction=30.0):
    return ((x, y), score, direction)


def test_a_new_body_needs_the_confidence_and_a_known_one_counts_while_found_at_all():
    memory = BodyMemory(conf=0.5)
    assert memory.match([at(0.0, -130.0, 0.3)]) == [None]                   # new and faint: not a body
    first = memory.match([at(0.0, -130.0), at(30.0, -150.0, 0.8)])
    assert None not in first and first[0] != first[1]
    again = memory.match([at(0.6, -130.4, 0.2), at(30.0, -150.0, 0.8)])     # its score dipped: still that body
    assert again == first


def test_a_body_found_nowhere_is_missing_then_gone():
    memory = BodyMemory()
    (a,) = memory.match([at(0.0, -130.0)])
    memory.match([])
    assert [b.id for b in memory.missing()] == [a]
    for _ in range(GONE_LOOKS - 1):
        memory.match([])
    assert memory.bodies == [] and memory.missing() == []


def test_a_picked_body_not_found_next_look_is_gone_at_once():
    memory = BodyMemory()
    a, b = memory.match([at(0.0, -130.0), at(30.0, -150.0)])
    memory.picked_near((0.5, -129.5))
    memory.match([at(30.0, -150.0)])
    assert memory.missing() == [] and [x.id for x in memory.bodies] == [b]     # not waited for


def test_a_failed_pick_leaves_the_same_body_even_if_it_now_scores_faint():
    memory = BodyMemory(conf=0.5)
    (a,) = memory.match([at(0.0, -130.0)])
    memory.picked_near((0.0, -130.0))
    assert memory.match([at(0.8, -130.6, 0.12)]) == [a]                     # disturbed, faint: still there
    assert not memory.bodies[0].picked


def test_a_body_keeps_the_value_it_was_read_as_while_it_stays_where_it_was():
    memory = BodyMemory()
    (a,) = memory.match([at(0.0, -130.0, direction=30.0)])
    memory.set_reading(a, learned.decode(scores_for(1000.0, 12.0), 0.9))
    for _ in range(3):                                                    # later looks: no reading, value kept
        memory.match([at(0.5, -130.5, direction=40.0)])
    assert memory.reading(a).value == 1000.0 and memory.why_no_value(a) == ""


def test_a_body_turned_or_shifted_since_its_read_is_to_be_measured():
    memory = BodyMemory()
    (a,) = memory.match([at(0.0, -130.0, direction=30.0)])
    memory.set_reading(a, learned.decode(scores_for(1000.0, 12.0), 0.9))
    memory.match([at(1.0, -130.0, direction=60.0)])                      # knocked round 30 degrees
    assert memory.reading(a) is None and memory.why_no_value(a) == "moved since it was read"
    (b,) = memory.match([at(10.0, -130.0)])                               # knocked 10 mm: a new body
    assert b != a and memory.why_no_value(b).startswith("not read this run")


def test_a_long_axis_has_no_head_or_tail():
    memory = BodyMemory()
    (a,) = memory.match([at(0.0, -130.0, direction=5.0)])
    memory.set_reading(a, learned.decode(scores_for(1000.0, 12.0), 0.9))
    memory.match([at(0.0, -130.0, direction=178.0)])                     # 7 degrees round, not 173
    assert memory.reading(a) is not None


def test_an_unsure_read_is_to_be_measured():
    memory = BodyMemory()
    (a,) = memory.match([at(0.0, -130.0)])
    memory.set_reading(a, learned.decode(np.zeros(2 * len(learned._OHMS)), 0.9))
    assert memory.reading(a).value is None and memory.why_no_value(a) == "unsure"


def test_the_two_views_are_averaged_when_they_agree_and_the_close_up_wins_when_not():
    agree = combine(scores_for(1000.0, 9.0), scores_for(1000.0, 9.0), 0.9)
    assert agree.value == 1000.0
    disagree = combine(scores_for(10.0, 12.0), scores_for(1000.0, 9.0), 0.9)    # the main capture says 10R
    assert disagree.best == 1000.0                                                # the close-up's focus is fresh
    assert combine(scores_for(470.0, 12.0), None, 0.9).value == 470.0             # no close-up: the main alone


class Model:
    pass


class Vision:
    conf = 0.5


class Detector:
    model, grey, vision = Model(), True, Vision()


class Take:
    def __init__(self, crop, white):
        self.crop, self.white = crop, white


class Crop:
    def __init__(self, image):
        self.image = image


def test_the_read_sweeps_the_focus_then_takes_close_ups_by_the_fresh_rule(monkeypatch):
    done = {}

    def calibrate(cap, cal, polygons, camera=None):
        done["swept"] = list(polygons)
        return [(240.0, 318.0), (270.0, 300.0)], None

    def take(cap, cal, polygons, seg=None, grey=False, model=None, camera=None, offset=None):
        done["model"] = model
        return [Take(Crop(f"close {p}"), 200.0) for p in polygons]

    monkeypatch.setattr(eyes_mod.closeup, "calibrate", calibrate)
    monkeypatch.setattr(eyes_mod.closeup, "take", take)
    monkeypatch.setattr(eyes_mod.learned, "log_probs", lambda net, image, white: f"lp {image}")
    eyes = Eyes.__new__(Eyes)
    eyes.detector, eyes.reader, eyes.cap, eyes.camera = Detector(), "net", None, {}
    eyes.closeups_on, eyes.say, eyes.focus = True, lambda text: None, None
    got = eyes._closeups("cal", ["p1", "p2"])
    assert done["swept"] == ["p1", "p2"]
    assert done["model"].focus(240.0) == 318 and done["model"].focus(270.0) == 300     # fitted to the sweep
    assert got == [("lp close p1", "close p1"), ("lp close p2", "close p2")]


def test_close_ups_can_be_turned_off(monkeypatch):
    monkeypatch.setattr(eyes_mod.closeup, "take", lambda *a, **k: pytest.fail("no close-ups wanted"))
    monkeypatch.setattr(eyes_mod.closeup, "calibrate", lambda *a, **k: pytest.fail("no sweep wanted"))
    eyes = Eyes.__new__(Eyes)
    eyes.closeups_on = False
    assert eyes._closeups("cal", ["p1"]) == [(None, None)]


def test_the_sorting_runs_read_limit_is_its_own():
    from robot_control.config import VisionSettings
    limit = VisionSettings.from_config().min_confidence
    assert limit == 0.7                                                 # vision.toml (0.9 for a while on 2026-11-11)
    # 0.82, as body 6 (a 4k7) read 470R: decode shares exp(score) over every value both ways round.
    others = 2 * len(learned._OHMS) - 1
    scores = scores_for(470.0, float(np.log((0.82 * others - 1) / 0.18)))
    assert combine(scores, None, limit).value == 470.0                  # trusted at 0.7
    assert combine(scores, None, 0.9).value is None                     # held back at 0.9


# Focus before every capture (robot_offline's FocusTracker, once pre/focus.py exists)

class FakeFocusTracker:
    """robot_offline's planned pre/focus.FocusTracker, as its handoff specifies it."""
    made, checks = 0, []

    def __init__(self, cap, cal):
        FakeFocusTracker.made += 1
        self.cap = cap

    def check(self, polygons=None):
        FakeFocusTracker.checks.append(self.cap.i)          # how many frames had been read when it ran
        FakeFocusTracker.references = polygons

    def status(self):
        return "focus 331 (+8), checked 0 s ago"


class FakeFocusApi:
    FocusTracker = FakeFocusTracker


def test_every_capture_is_taken_right_after_a_focus_check(monkeypatch):
    monkeypatch.setattr(eyes_mod, "focus_api", FakeFocusApi)
    FakeFocusTracker.made, FakeFocusTracker.checks = 0, []
    eyes = eyes_with(FakeTracker(found_at=2, control=False))
    eyes.focus, eyes.focus_note = None, ""
    eyes._picture()
    first_capture_ends = eyes.cap.i
    eyes.tracker.seen = 0                                   # a second capture, the plate already found
    eyes._picture()
    assert FakeFocusTracker.made == 1                       # one tracker for the session
    assert len(FakeFocusTracker.checks) == 2                # a check before each capture
    assert FakeFocusTracker.checks[0] == 1                  # straight after the plate was found, before the average
    assert FakeFocusTracker.checks[1] > first_capture_ends
    assert eyes.focus_note.startswith("focus 331")


def test_without_robot_offlines_focus_check_captures_stay_at_the_fixed_focus(monkeypatch):
    monkeypatch.setattr(eyes_mod, "focus_api", None)
    eyes = eyes_with(FakeTracker(found_at=2, control=False))
    eyes.focus, eyes.focus_note = None, eyes_mod.NO_FOCUS_CHECK
    eyes._picture()
    assert eyes.focus is None and "no focus check" in eyes.focus_note


def test_with_a_focus_check_the_close_ups_need_no_sweep(monkeypatch):
    monkeypatch.setattr(eyes_mod.closeup, "calibrate", lambda *a, **k: pytest.fail("no sweep with a focus check"))
    used = {}

    def take(cap, cal, polygons, seg=None, grey=False, model=None, camera=None, offset=None):
        used["model"], used["offset"] = model, offset
        return [Take(Crop("c"), 200.0) for _ in polygons]

    monkeypatch.setattr(eyes_mod.closeup, "take", take)
    monkeypatch.setattr(eyes_mod.learned, "log_probs", lambda net, image, white: "lp")
    eyes = Eyes.__new__(Eyes)
    eyes.detector, eyes.reader, eyes.cap, eyes.camera = Detector(), "net", None, {}
    tracker = FakeFocusTracker.__new__(FakeFocusTracker)
    tracker.offset = 8.0
    eyes.closeups_on, eyes.say, eyes.focus = True, lambda text: None, tracker
    eyes._closeups("cal", ["p1"])
    assert used["model"] is None and used["offset"] == 8.0  # closeup.take's own rule, plus the check's offset


def test_a_runs_looks_recheck_focus_only_now_and_then_but_the_read_always(monkeypatch):
    monkeypatch.setattr(eyes_mod, "focus_api", FakeFocusApi)
    clock = {"now": 1000.0}
    monkeypatch.setattr(eyes_mod.time, "monotonic", lambda: clock["now"])
    FakeFocusTracker.made, FakeFocusTracker.checks = 0, []
    eyes = eyes_with(FakeTracker(found_at=1, control=False))
    eyes.focus, eyes.focus_note, eyes.focus_checked_at = None, "", None

    def picture(sharp):
        eyes.tracker.seen = 0
        eyes._picture(sharp)
        return len(FakeFocusTracker.checks)

    assert picture(False) == 1                              # never checked yet: checks
    clock["now"] += 30.0
    assert picture(False) == 1                              # a look 30 s later: no check, the focus kept
    assert picture(True) == 2                               # the start-of-run read: always checks
    clock["now"] += eyes_mod.FOCUS_RECHECK_S + 1
    assert picture(False) == 3                              # stale: checks again
