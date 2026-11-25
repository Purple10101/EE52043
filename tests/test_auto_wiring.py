"""2026-11-18's work as the auto script's CALIBRATE uses it: the firmware check at start, and the aim check
taking gripper v5's measured jaw centre as the wrist's axis."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

import slide_wrist  # noqa: E402

from robot_control.task import aim_check  # noqa: E402


class Port:
    def __init__(self, stray=b""):
        self.timeout, self.stray = 120, stray

    @property
    def in_waiting(self):
        return len(self.stray)

    def read(self, n):
        out, self.stray = self.stray[:n], self.stray[n:]
        return out


class Scara:
    def __init__(self, stray=b"", fails=None):
        self.api, self.fails, self.sent = Port(stray), fails, []

    def SCARA_SET_PLAY(self, axis, play, direction):
        self.sent.append((axis, play, direction, self.api.timeout))
        if self.fails:
            raise self.fails


def test_the_play_firmware_takes_the_elbows_play_and_nothing_more(monkeypatch):
    monkeypatch.setattr(slide_wrist.time, "sleep", lambda s: None)
    scara = Scara()
    slide_wrist.set_elbow_play(scara)
    assert scara.sent == [(1, slide_wrist.ELBOW_PLAY_DEG, 1, 3.0)]     # a short wait for the answer
    assert scara.api.timeout == 120                                    # given back


def test_the_kit_firmware_is_refused_by_its_extra_bytes(monkeypatch):
    monkeypatch.setattr(slide_wrist.time, "sleep", lambda s: None)
    with pytest.raises(SystemExit, match="kit firmware"):
        slide_wrist.set_elbow_play(Scara(stray=bytes(8)))


@pytest.mark.parametrize("fails", [TimeoutError("system failure"), ValueError("unexpected flag received: 0")])
def test_a_firmware_without_set_play_is_refused(monkeypatch, fails):
    monkeypatch.setattr(slide_wrist.time, "sleep", lambda s: None)
    scara = Scara(fails=fails)
    with pytest.raises(SystemExit, match="PLAY firmware"):
        slide_wrist.set_elbow_play(scara)
    assert scara.api.timeout == 120


class Eyes:
    def __init__(self, centre):
        self.centre = centre

    def lens_mm(self):
        return (-142.5, -144.6, 179.4)

    def find_tip_and_axis(self, near, height):
        return np.asarray(near, float), (None if self.centre is None else np.asarray(near, float) + self.centre)


class Arm:
    class Bench:
        OVER_PLATE_SLIDE = 59.0

    bench = Bench()
    navigator = object()

    def touch(self, point, lens):
        return -48.0, 110.0, -10.0

    def lift(self):
        pass

    def park(self):
        pass


def visited_axes(tmp_path, monkeypatch, centre):
    seen = {}
    monkeypatch.setattr(aim_check.AimCheck, "judge",
                        lambda self, points, tips, axes, joints, nav, path, say: seen.update(axes=axes, path=path))
    aim_check.AimCheck(Eyes(centre), Arm(), logs=tmp_path).run(say=lambda t: None)
    return seen["axes"], seen["path"].read_text().splitlines()


def test_v5s_measured_centre_is_the_axis_as_it_is(tmp_path, monkeypatch):
    axes, rows = visited_axes(tmp_path, monkeypatch, np.array([5.3, 0.5]))
    assert axes == pytest.approx(np.array(aim_check.CHECK_POINTS) + [5.3, 0.5])
    assert rows[0].endswith("centre_x,centre_y") and rows[1].split(",")[-2:] == ["25.3", "-149.5"]


def test_without_a_measured_centre_the_wrist_model_gives_the_axis(tmp_path, monkeypatch):
    from robot_control.vision.guide import axis_mm
    axes, rows = visited_axes(tmp_path, monkeypatch, None)
    assert axes[0] == pytest.approx(axis_mm(aim_check.CHECK_POINTS[0], -48.0, 110.0, -10.0))
    assert rows[1].split(",")[-2:] == ["", ""]
