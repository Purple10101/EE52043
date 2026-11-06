import cv2
import numpy as np
import pytest

pytest.importorskip("robot_offline.pre.rectify")
from robot_control.vision.plate_search import outlines  # noqa: E402

PLATE = np.array([[1739, 803], [3023, 870], [3070, 2085], [1453, 1965]], np.int32)


def frame_with(*shapes):
    frame = np.full((2592, 4608, 3), 40, np.uint8)
    cv2.fillPoly(frame, [PLATE], (210, 210, 210))
    for draw in shapes:
        draw(frame)
    return frame


def nearest(quads):
    return min(np.abs(q - PLATE).max() for q in quads)


def test_the_plate_is_among_the_outlines():
    assert nearest(outlines(frame_with())) < 15


def test_a_lead_across_the_plate_doesnt_hide_it():
    lead = lambda f: cv2.line(f, (1900, 1000), (2900, 1900), (60, 60, 60), 8, cv2.LINE_AA)
    assert nearest(outlines(frame_with(lead))) < 15


def test_a_frame_with_no_long_lines_gives_no_outlines():
    assert outlines(np.full((2592, 4608, 3), 40, np.uint8)) == []
