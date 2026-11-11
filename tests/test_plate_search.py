"""The plate is found only by robot_offline's plate_search (tested there); this repo has none of its own."""
import importlib.util

import numpy as np
import pytest

pytest.importorskip("robot_offline.pre.plate_search")
from robot_offline.pre import plate_search, rectify  # noqa: E402


def test_this_repo_has_no_plate_search_of_its_own():
    assert importlib.util.find_spec("robot_control.vision.plate_search") is None


def test_the_detector_finds_the_plate_with_robot_offlines_search_and_doesnt_remember_saved_frames(monkeypatch):
    from robot_control.vision.detector import Detector
    calls = []
    monkeypatch.setattr(plate_search, "find", lambda frame, settings=None, last=None, keep=True: calls.append((last, keep)) or "cal")
    previous = rectify.Calibration(np.eye(3), np.ones((4, 2)), (4608, 2592), rectify.Settings.from_config())
    assert Detector.calibrate(None, "frame", previous) == "cal"
    assert np.array_equal(calls[0][0], previous.corners_px) and calls[0][1] is False
