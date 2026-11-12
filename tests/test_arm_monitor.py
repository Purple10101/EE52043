"""ArmMonitor: the arm's pose for the UI, from the commands passing through it."""

import pytest

from robot_control.hardware import arm_monitor
from robot_control.hardware.arm_monitor import JAWS_OPEN_MM, ArmMonitor


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


class FakeNucleo:
    """Answers like the course API. During a move it calls `during` (to look at the pose mid-move)."""

    def __init__(self, pots=(10.0, 110.0)):
        self.pots, self.during, self.sent = list(pots), None, []

    def SCARA_MOVE_JOINT(self, axis, angle, seconds):
        self.sent.append(("joint", axis, angle))
        if self.during:
            self.during()

    def SCARA_MOVE_JOINTS(self, base, elbow, wrist, slide, seconds):
        self.sent.append(("joints", base, elbow, wrist, slide))
        if self.during:
            self.during()

    def SCARA_READ_ANGLE(self, axis):
        return self.pots[axis]

    def SCARA_AUTO_CALIBRATE(self):
        if self.during:
            self.during()

    def SCARA_INITIALISE(self, *args):
        self.sent.append(("init",) + args)


class FakeJaws:
    def __init__(self):
        self.done = []

    def open(self):
        self.done.append("open")

    def close(self):
        self.done.append("close")

    def status(self):
        return {"cal": 1}


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(arm_monitor.time, "monotonic", c)
    return c


def test_nothing_is_known_until_moved_or_read(clock):
    pose = ArmMonitor(FakeNucleo()).pose()
    assert (pose.base, pose.elbow, pose.wrist, pose.slide) == (None, None, None, None)
    assert not pose.moving


def test_readings_place_the_base_and_elbow_and_the_bench_the_rest(clock):
    monitor = ArmMonitor(FakeNucleo(pots=(-77.0, 110.0)))
    monitor.nucleo.SCARA_READ_ANGLE(0)
    monitor.nucleo.SCARA_READ_ANGLE(1)
    monitor.know(wrist=0.0, slide=-50.0, nonsense=3.0)
    pose = monitor.pose()
    assert (pose.base, pose.elbow, pose.wrist, pose.slide) == (-77.0, 110.0, 0.0, -50.0)


def test_mid_move_the_pose_is_part_way_and_ends_at_the_goal(clock):
    nucleo = FakeNucleo(pots=(0.0, 100.0))
    monitor = ArmMonitor(nucleo)
    monitor.nucleo.SCARA_READ_ANGLE(0)
    monitor.nucleo.SCARA_READ_ANGLE(1)
    monitor.know(wrist=0.0, slide=20.0)
    seen = {}

    def halfway():
        seen["start"] = monitor.pose()
        clock.now += 1.0                    # half of the 2 s move
        seen["half"] = monitor.pose()
        clock.now += 5.0                    # long past its end, still waiting for the reply
        seen["late"] = monitor.pose()

    nucleo.during = halfway
    monitor.nucleo.SCARA_MOVE_JOINTS(40.0, 120.0, 30.0, 20.0, 2.0)
    assert seen["start"].moving and seen["start"].base == pytest.approx(0.0)
    assert seen["start"].goal == {"base": 40.0, "elbow": 120.0, "wrist": 30.0, "slide": 20.0}
    assert seen["half"].base == pytest.approx(20.0) and seen["half"].elbow == pytest.approx(110.0)
    assert seen["late"].base == pytest.approx(40.0)
    after = monitor.pose()
    assert not after.moving and (after.base, after.elbow, after.wrist) == (40.0, 120.0, 30.0)
    assert nucleo.sent == [("joints", 40.0, 120.0, 30.0, 20.0)]      # passed on unchanged


def test_one_joint_moves_alone(clock):
    nucleo = FakeNucleo()
    monitor = ArmMonitor(nucleo)
    monitor.know(base=-77.0, elbow=110.0, wrist=0.0, slide=20.0)
    monitor.nucleo.SCARA_MOVE_JOINT(3, -10.0, 1.0)
    pose = monitor.pose()
    assert (pose.base, pose.slide) == (-77.0, -10.0)
    assert nucleo.sent == [("joint", 3, -10.0)]


def test_a_reading_after_a_move_says_where_the_joint_really_landed(clock):
    nucleo = FakeNucleo(pots=(39.2, 120.4))
    monitor = ArmMonitor(nucleo)
    monitor.nucleo.SCARA_MOVE_JOINTS(40.0, 120.0, 30.0, 20.0, 2.0)
    monitor.nucleo.SCARA_READ_ANGLE(0)
    assert monitor.pose().base == 39.2


def test_while_calibrating_the_base_and_elbow_are_unknown(clock):
    nucleo = FakeNucleo()
    monitor = ArmMonitor(nucleo)
    monitor.know(base=10.0, elbow=100.0)
    seen = {}
    nucleo.during = lambda: seen.setdefault("pose", monitor.pose())
    monitor.nucleo.SCARA_AUTO_CALIBRATE()
    assert seen["pose"].calibrating and seen["pose"].base is None
    assert not monitor.pose().calibrating


def test_other_commands_pass_straight_through(clock):
    nucleo = FakeNucleo()
    ArmMonitor(nucleo).nucleo.SCARA_INITIALISE(0.125, 0.1)
    assert nucleo.sent == [("init", 0.125, 0.1)]


def test_the_jaws_open_and_close_over_a_moment(clock):
    jaws = FakeJaws()
    monitor = ArmMonitor(FakeNucleo(), jaws)
    assert monitor.pose().jaws_mm is None
    monitor.gripper.open()
    assert monitor.pose().jaws_mm == pytest.approx(JAWS_OPEN_MM)        # first move: no start to come from
    monitor.gripper.close()
    assert monitor.pose().jaws_mm == pytest.approx(JAWS_OPEN_MM)
    clock.now += 10.0
    assert monitor.pose().jaws_mm == pytest.approx(0.0)
    assert jaws.done == ["open", "close"] and monitor.gripper.status() == {"cal": 1}


def test_holding_comes_from_the_run(clock):
    monitor = ArmMonitor(FakeNucleo())
    held = {"now": False}
    monitor.watch_holding(lambda: held["now"])
    assert not monitor.pose().holding
    held["now"] = True
    assert monitor.pose().holding


def test_the_map_draws_the_arm_wherever_it_is(clock):
    from robot_control.task.rig_map import RigMap
    monitor = ArmMonitor(FakeNucleo())
    monitor.know(base=-45.0, elbow=110.0, wrist=70.0, slide=20.0)
    rig_map = RigMap.load({"A1": {"centre_mm": [-92.5, 25.0], "long_axis_deg": -90.0}})
    with_arm = rig_map.draw([], None, 440, monitor.pose())
    without = rig_map.draw([], None, 440, None)
    assert with_arm.shape == without.shape
    assert (with_arm != without).any()
    wrist = rig_map.joints_mm(-46.12, 110.42)[2]
    assert wrist == pytest.approx((0.0, -130.0), abs=0.5)      # the CAD pose over the plate centre (workspace.toml)
