"""The auto window's fixed procedure: CALIBRATE (arm, empty plate, aim) before START."""

import numpy as np
import pytest

pytest.importorskip("robot_offline.post.bands.learned")

from robot_control.motion.arm import ElbowCheck  # noqa: E402
from robot_control.task import auto_app  # noqa: E402
from robot_control.task.aim_check import AimReport  # noqa: E402
from robot_control.task.auto_app import AutoSortingApp, CalibrationFailed, StandardCalibration  # noqa: E402

GOOD_ELBOW = ElbowCheck(((105.0, 105.2), (115.0, 114.8), (110.0, 110.1)))
BAD_ELBOW = ElbowCheck(((105.0, 109.0), (115.0, 119.0), (110.0, 114.0)))


def report(after=0.9, found=5):
    return AimReport([None] * 5, [(0, 0)] * found + [None] * (5 - found), np.zeros((found, 2)), np.zeros(2), 1.2, after,
                     verdict=f"aim left {after}")


class Arm:
    def __init__(self, elbow, done):
        self.elbow, self.done = elbow, done
        self.last_started = "2026-11-11 20:00"

    def start_up(self, say=print):
        self.done.append("start_up")
        return self.elbow


class Eyes:
    def __init__(self, bodies, done):
        self.bodies, self.done = bodies, done

    def count_bodies(self):
        self.done.append("count_bodies")
        return self.bodies


class Log:
    def __init__(self):
        self.events = []

    def event(self, cycle, what, **details):
        self.events.append(what)


def calibrate(monkeypatch, elbow=GOOD_ELBOW, bodies=0, aim=None):
    done = []

    class Aim:
        def __init__(self, eyes, arm):
            pass

        def run(self, say=print):
            done.append("aim")
            return aim or report()

    monkeypatch.setattr(auto_app, "AimCheck", Aim)
    log = Log()
    calibration = StandardCalibration(Arm(elbow, done), Eyes(bodies, done), log, say=lambda t: None)
    return calibration, done, log


def test_the_steps_run_in_order_and_all_must_pass(monkeypatch):
    calibration, done, log = calibrate(monkeypatch)
    result = calibration.run()
    assert done == ["start_up", "count_bodies", "aim"]
    assert result.elbow.ok and "calibrated" in result.summary() and "calibrated" in log.events


def test_a_bad_elbow_check_stops_before_the_plate(monkeypatch):
    calibration, done, _ = calibrate(monkeypatch, elbow=BAD_ELBOW)
    with pytest.raises(CalibrationFailed, match="Free the cables"):
        calibration.run()
    assert done == ["start_up"]


def test_resistors_on_the_plate_stop_it_before_the_aim_check(monkeypatch):
    calibration, done, _ = calibrate(monkeypatch, bodies=3)
    with pytest.raises(CalibrationFailed, match="isn't empty: 3 resistors"):
        calibration.run()
    assert "aim" not in done                    # the tip never goes down onto parts


def test_an_aim_that_a_shift_cant_fix_fails(monkeypatch):
    calibration, _, _ = calibrate(monkeypatch, aim=report(after=3.0))
    with pytest.raises(CalibrationFailed, match="aim check"):
        calibration.run()


def test_too_few_tips_found_fails(monkeypatch):
    calibration, _, _ = calibrate(monkeypatch, aim=report(found=2))
    with pytest.raises(CalibrationFailed, match="aim check"):
        calibration.run()


class Window:
    def __init__(self):
        self.status = ""

    def show_arm(self, text):
        pass

    def show_aim(self, text):
        pass

    def say(self, text):
        self.status = text

    def show_trays(self, trays, target=None):
        pass


class Supervisor:
    def __init__(self, arm):
        self.arm, self.eyes, self.log = arm, None, Log()

    def update_trays(self):
        pass

    def say(self, text):
        pass


def test_start_is_only_offered_once_a_calibration_has_passed():
    app = AutoSortingApp(Supervisor(Arm(GOOD_ELBOW, [])), Window())
    assert app.enabled() == {"CALIBRATE"}
    assert "Plate EMPTY" in app.window.status
    app.calibration = object()
    assert app.enabled() == {"CALIBRATE", "START"}


def test_aiming_by_the_cad_geometry_calibrate_skips_the_aim_check(monkeypatch):
    calibration, done, _ = calibrate(monkeypatch)
    calibration.aims, calibration.of = False, 2
    result = calibration.run()
    assert done == ["start_up", "count_bodies"]
    assert result.aim is None and "CAD geometry" in result.summary()
