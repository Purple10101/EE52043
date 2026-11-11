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
    return eyes


def test_a_look_averages_fresh_frames_once_the_plate_and_exposure_have_settled():
    eyes = eyes_with(FakeTracker(found_at=3, settles_at=5))
    frame, cal = eyes._picture()
    # frames 0-4 go to the tracker (settled on the 5th), 5 to 5 + SETTLE - 1 are dropped, then the next 8
    first = 5 + closeup.SETTLE
    expected = np.mean(range(first, first + AVERAGE_FRAMES))
    assert cal == "cal"
    assert frame.dtype == np.uint8 and frame.shape == (4, 6, 3)
    assert frame[0, 0, 0] == pytest.approx(expected, abs=0.5)
    assert eyes.cap.i == first + AVERAGE_FRAMES - 1      # nothing read beyond the average


def test_a_camera_that_cant_be_set_needs_only_the_plate():
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


# Following bodies from look to look (BodyMemory), and their close-ups

from robot_offline.post.bands import learned  # noqa: E402

from robot_control.vision import eyes as eyes_mod  # noqa: E402
from robot_control.vision.body_memory import GONE_LOOKS, BodyMemory  # noqa: E402


def scores_for(ohms, strength):
    """BandCTC-style scores (1-D, per value and way round) that favour `ohms` by `strength`."""
    s = np.zeros(2 * len(learned._OHMS))
    s[int(np.flatnonzero(np.isclose(learned._OHMS, ohms))[0])] = strength
    return s


def test_a_new_body_needs_the_confidence_and_a_known_one_counts_while_found_at_all():
    memory = BodyMemory(0.7, conf=0.5)
    assert memory.match([((0.0, -130.0), 0.3)]) == [None]                   # new and faint: not a body
    first = memory.match([((0.0, -130.0), 0.9), ((30.0, -150.0), 0.8)])
    assert None not in first and first[0] != first[1]
    again = memory.match([((0.6, -130.4), 0.2), ((30.0, -150.0), 0.8)])     # its score dipped: still that body
    assert again == first


def test_a_body_found_nowhere_is_missing_then_gone():
    memory = BodyMemory(0.7)
    (a,) = memory.match([((0.0, -130.0), 0.9)])
    memory.match([])
    assert [b.id for b in memory.missing()] == [a]
    for _ in range(GONE_LOOKS - 1):
        memory.match([])
    assert memory.bodies == [] and memory.missing() == []


def test_a_picked_body_not_found_next_look_is_gone_at_once():
    memory = BodyMemory(0.7)
    a, b = memory.match([((0.0, -130.0), 0.9), ((30.0, -150.0), 0.9)])
    memory.picked_near((0.5, -129.5))
    memory.match([((30.0, -150.0), 0.9)])
    assert memory.missing() == [] and [x.id for x in memory.bodies] == [b]     # not waited for


def test_a_failed_pick_leaves_the_same_body_even_if_it_now_scores_faint():
    memory = BodyMemory(0.7, conf=0.5)
    (a,) = memory.match([((0.0, -130.0), 0.9)])
    memory.picked_near((0.0, -130.0))
    assert memory.match([((0.8, -130.6), 0.12)]) == [a]                        # disturbed, faint: still there
    assert not memory.bodies[0].picked


def test_one_looks_slip_cant_flip_a_reading_on_its_own():
    memory = BodyMemory(0.7)
    (a,) = memory.match([((0.0, -130.0), 0.9)])
    unsure = np.zeros(2 * len(learned._OHMS))           # a look that says nothing
    assert memory.read(a, unsure).value is None
    memory.match([((0.0, -130.0), 0.9)])
    wrong = memory.read(a, scores_for(10.0, 12.0))      # then a confident 10 ohm on its own: it would pass alone
    assert learned.decode(scores_for(10.0, 12.0), 0.7).value == 10.0
    assert wrong.value is None                          # averaged with the unsure look, it doesn't


def test_the_close_up_is_pinned_into_the_reading():
    memory = BodyMemory(0.7)
    (a,) = memory.match([((0.0, -130.0), 0.9)])
    memory.pin_closeup(a, scores_for(1000.0, 12.0))
    assert memory.has_closeup(a)
    assert memory.read(a, scores_for(1000.0, 12.0)).value == 1000.0


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


def eyes_for_closeups(monkeypatch, taken):
    def take(cap, cal, polygons, seg=None, grey=False, camera=None):
        taken.append(list(polygons))
        return [Take(Crop(f"close {p}"), 200.0) for p in polygons]

    monkeypatch.setattr(eyes_mod.closeup, "take", take)
    monkeypatch.setattr(eyes_mod.learned, "log_probs", lambda net, image, white: np.zeros(2 * len(learned._OHMS)))
    eyes = Eyes.__new__(Eyes)
    eyes.detector, eyes.reader, eyes.cap, eyes.camera = Detector(), "net", None, {}
    eyes.closeups_on, eyes.say, eyes.closeup_crops = True, lambda text: None, {}
    eyes.memory = BodyMemory(0.7)
    return eyes


def test_only_bodies_without_a_close_up_get_one(monkeypatch):
    taken = []
    eyes = eyes_for_closeups(monkeypatch, taken)
    ids = eyes.memory.match([((0.0, -100.0), 0.9), ((30.0, -150.0), 0.9)])
    eyes._closeups("cal", [(("p1", 0.9, None, (0.0, -100.0)), ids[0]), (("p2", 0.9, None, (30.0, -150.0)), ids[1])])
    assert taken == [["p1", "p2"]] and eyes.closeup_crops[ids[0]] == "close p1"
    ids2 = eyes.memory.match([((30.0, -150.0), 0.9), ((-30.0, -120.0), 0.9)])   # p1 gone, p3 new
    eyes._closeups("cal", [(("p2", 0.9, None, (30.0, -150.0)), ids2[0]), (("p3", 0.9, None, (-30.0, -120.0)), ids2[1])])
    assert taken[1] == ["p3"]
    eyes._closeups("cal", [(("p2", 0.9, None, (30.0, -150.0)), ids2[0])])
    assert len(taken) == 2                              # nothing new: no close-ups, a quick look


def test_close_ups_can_be_turned_off(monkeypatch):
    monkeypatch.setattr(eyes_mod.closeup, "take", lambda *a, **k: pytest.fail("no close-ups wanted"))
    eyes = Eyes.__new__(Eyes)
    eyes.closeups_on, eyes.memory = False, BodyMemory(0.7)
    eyes._closeups("cal", [(("p1", 0.9, None, (0.0, -100.0)), 1)])


def test_the_sorting_runs_read_limit_is_its_own():
    from robot_control.config import VisionSettings
    assert VisionSettings.from_config().min_confidence == 0.9          # vision.toml, chosen 2026-11-11
    memory = BodyMemory(0.9)
    (a,) = memory.match([((0.0, -130.0), 0.9)])
    # 0.82 on 470R, as body 6 (a 4k7) read: decode shares exp(score) over every value both ways round.
    others = 2 * len(learned._OHMS) - 1
    s = scores_for(470.0, float(np.log((0.82 * others - 1) / 0.18)))
    reading = memory.read(a, s)
    assert 0.75 < reading.confidence < 0.9 and reading.value is None    # held back at 0.9; at 0.7 it passed
