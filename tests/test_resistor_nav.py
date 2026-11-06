import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("serial")
pytest.importorskip("robot_offline.pre.rectify")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from resistor_nav import body_at  # noqa: E402

A = np.array([[100, 100], [160, 100], [160, 124], [100, 124]], float)
B = A + (300, 200)


def test_clicking_a_body_chooses_it():
    assert body_at([A, B], (130, 110)) == 0
    assert body_at([A, B], (430, 310)) == 1


def test_a_click_just_off_a_body_still_chooses_it():
    assert body_at([A, B], (170, 110)) == 0


def test_a_click_on_bare_plate_chooses_nothing():
    assert body_at([A, B], (300, 500)) is None
    assert body_at([], (10, 10)) is None
