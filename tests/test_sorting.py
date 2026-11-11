"""The sorting run's logic, with the camera and the arm faked: BinPlan, PickMemory, Planner, Supervisor."""

import numpy as np
import pytest

pytest.importorskip("robot_offline.post.bands.learned")

from robot_control.motion.arm import ArmError  # noqa: E402
from robot_control.planning.bins import BinPlan  # noqa: E402
from robot_control.planning.memory import MAX_TRIES, Attempt, PickMemory  # noqa: E402
from robot_control.planning.planner import Planner  # noqa: E402
from robot_control.task.supervisor import MAX_FAILS_IN_A_ROW, Supervisor  # noqa: E402
from robot_control.world import PlateView, Resistor  # noqa: E402

SORTER = (10.0, 10_000.0)


def resistor(number, x, y, ohms=1000.0, confidence=0.9):
    """A resistor at scene (x, y) mm; ohms None for one the reader isn't sure of."""
    return Resistor(number, (float(x), float(y)), 0.0, 0.9, ohms, ohms or 1000.0, confidence, np.zeros((4, 2)))


def view(*resistors):
    return PlateView("look", None, None, list(resistors))


def bin_plan(ids=("A1", "A2", "A3", "B4")):
    return BinPlan(list(ids), SORTER)


# BinPlan

def test_a_value_takes_the_next_free_bin_and_keeps_it():
    plan = bin_plan()
    assert plan.bin_for(1500) == "A1"
    assert plan.assign(1500) == "A1"
    assert plan.bin_for(2200) == "A2"
    assert plan.assign(2200) == "A2"
    assert plan.assign(1500) == "A1"
    assert plan.assigned == {1500: "A1", 2200: "A2"}


def test_looking_up_a_bin_decides_nothing():
    plan = bin_plan()
    plan.bin_for(1500)
    assert plan.bin_for(2200) == "A1"


def test_values_outside_the_sorters_range_have_no_bin():
    plan = bin_plan()
    assert plan.bin_for(4.7) is None and plan.bin_for(100_000) is None
    with pytest.raises(ValueError):
        plan.assign(100_000)
    assert plan.assigned == {}
    assert plan.bin_for(1500) == "A1"


def test_once_the_bins_are_taken_a_new_value_has_nowhere_to_go():
    plan = bin_plan(("A1",))
    plan.assign(1500)
    assert plan.bin_for(2200) is None
    assert plan.bin_for(1500) == "A1"
    with pytest.raises(ValueError):
        plan.assign(2200)


def test_every_slot_in_the_file_is_a_bin_in_its_order():
    plan = BinPlan.from_slots({"A1": {}, "A2": {}, "B4": {}}, SORTER)
    assert plan.free == ["A1", "A2", "B4"]


# PickMemory

def test_an_empty_spot_after_a_pick_means_it_was_sorted():
    memory = PickMemory()
    memory.attempted(Attempt((0.0, -130.0), 1000.0, "A1"))
    assert memory.judge(view(resistor(1, 20, -130))) == "sorted"
    assert len(memory.sorted) == 1
    assert memory.judge(view()) is None         # nothing left to judge


def test_a_body_still_there_means_a_miss_and_two_misses_give_it_up():
    memory, there = PickMemory(), view(resistor(1, 0.0, -130.0))
    for outcome in ["missed"] * (MAX_TRIES - 1) + ["gave up"]:
        memory.attempted(Attempt((1.0, -131.0), 1000.0, "A1"))      # the spots differ a little each look
        assert memory.judge(there) == outcome
    assert memory.is_given_up(there.resistors[0]) == f"missed {MAX_TRIES} times"
    assert memory.sorted == []


# Planner

def test_unsure_and_given_up_resistors_stay_on_the_plate():
    memory = PickMemory()
    memory.give_up((40.0, -100.0), "the arm couldn't pick it")
    planner = Planner(bin_plan(), memory)
    plan = planner.plan(view(resistor(1, 0, -130, ohms=None, confidence=0.4), resistor(2, 40, -100)))
    assert plan.next is None and plan.waiting == 0
    assert plan.fates[1].startswith("stays: unsure (best 1k")
    assert plan.fates[2] == "stays: the arm couldn't pick it"


def test_the_most_isolated_resistor_goes_first():
    planner = Planner(bin_plan(), PickMemory())
    crowded = [resistor(1, 0, -130), resistor(2, 5, -130), resistor(3, -40, -100)]
    plan = planner.plan(view(*crowded))
    assert plan.next.resistor.number == 3
    assert plan.fates[3].startswith("NEXT ")
    assert plan.waiting == 3


def test_a_value_outside_the_range_stays():
    plan = Planner(bin_plan(), PickMemory()).plan(view(resistor(1, 0, -130, ohms=100_000)))
    assert plan.next is None
    assert plan.fates[1] == "stays: 100k is outside 10 to 10k"


def test_a_value_with_no_bin_left_stays():
    bins = bin_plan(("A1",))
    bins.assign(1500)
    plan = Planner(bins, PickMemory()).plan(view(resistor(1, 0, -130, ohms=2200)))
    assert plan.next is None
    assert "no bin free" in plan.fates[1]


# Supervisor

class FakeEyes:
    """A plate whose resistors the FakeArm takes away when it picks them."""

    def __init__(self, *resistors):
        self.plate = list(resistors)
        self.looks = 0

    def look(self):
        self.looks += 1
        return view(*self.plate)

    def picked(self, centre_mm):
        pass


class FakeArm:
    """Does what it's told. misses: resistor numbers it never manages to take. refuses: ones it can't reach.
    drops_fail: it never manages to drop."""

    def __init__(self, eyes, misses=(), refuses=(), drops_fail=False):
        self.eyes, self.misses, self.refuses, self.drops_fail = eyes, set(misses), set(refuses), drops_fail
        self.holding, self.message, self.done = False, "", []
        self.last_started = "2026-11-11 15:00"

    def park(self):
        if self.holding:
            raise ArmError("holding")
        self.done.append("park")

    def pick(self, r):
        self.done.append(f"pick #{r.number}")
        if r.number in self.refuses:
            self.message = "CAN'T REACH THAT ONE"
            return False
        self.holding = True
        if r.number not in self.misses:
            self.eyes.plate = [p for p in self.eyes.plate if p.number != r.number]
        return True

    def drop(self, bin_id):
        self.done.append(f"drop {bin_id}")
        if self.drops_fail:
            self.message = "NOT LOWERING"
            return False
        self.holding = False
        return True


class FakeLog:
    def __init__(self):
        self.events = []

    def start_run(self):
        self.events.append("start")

    def event(self, cycle, what, **details):
        self.events.append(what)

    def look(self, cycle, view, plan, image=None):
        return "0001"


def supervisor(eyes, arm, bins=None):
    return Supervisor(eyes, arm, Planner(bins or bin_plan(), PickMemory()), FakeLog())


def test_a_run_sorts_the_plate_by_value_and_ends_when_it_is_empty():
    eyes = FakeEyes(resistor(1, 0, -130, ohms=1500), resistor(2, 30, -100, ohms=2200), resistor(3, -30, -160, ohms=1500))
    arm = FakeArm(eyes)
    boss = supervisor(eyes, arm)
    ended = boss.run()
    assert ended.startswith("done")
    assert eyes.plate == []
    assert boss.bins.assigned == {1500: "A1", 2200: "A2"}
    assert len(boss.memory.sorted) == 3
    drops = [d for d in arm.done if d.startswith("drop")]
    assert sorted(drops) == ["drop A1", "drop A1", "drop A2"]
    assert arm.done[0] == "park" and arm.done[-1] == "park"


def test_unsure_resistors_are_left_and_the_run_still_ends():
    eyes = FakeEyes(resistor(1, 0, -130, ohms=None, confidence=0.3), resistor(2, 30, -100))
    boss = supervisor(eyes, FakeArm(eyes))
    assert boss.run().startswith("done")
    assert [r.number for r in eyes.plate] == [1]


def test_a_resistor_missed_twice_is_left_on_the_plate():
    eyes = FakeEyes(resistor(1, 0, -130), resistor(2, 40, -100))
    arm = FakeArm(eyes, misses={2})
    boss = supervisor(eyes, arm)
    assert boss.run().startswith("done")
    assert [r.number for r in eyes.plate] == [2]
    assert arm.done.count("pick #2") == MAX_TRIES


def test_a_resistor_the_arm_cant_reach_is_given_up_without_moving_on_it_again():
    eyes = FakeEyes(resistor(1, 0, -130), resistor(2, 60, -100))
    arm = FakeArm(eyes, refuses={2})
    boss = supervisor(eyes, arm)
    assert boss.run().startswith("done")
    assert arm.done.count("pick #2") == 1
    assert "pick refused" in boss.log.events


def test_a_resistor_it_cant_drop_stops_the_run():
    eyes = FakeEyes(resistor(1, 0, -130))
    boss = supervisor(eyes, FakeArm(eyes, drops_fail=True))
    ended = boss.run()
    assert ended.startswith("STOPPED") and "couldn't drop" in ended
    assert boss.bins.assigned == {}


def test_picks_that_keep_coming_to_nothing_stop_the_run():
    plate = [resistor(i, 40 * (i % 3) - 40, -100 - 25 * (i // 3), ohms=1000.0) for i in range(1, 7)]
    eyes = FakeEyes(*plate)
    boss = supervisor(eyes, FakeArm(eyes, misses={r.number for r in plate}))
    ended = boss.run()
    assert ended.startswith("STOPPED") and "in a row" in ended
    assert boss.fails_in_a_row == MAX_FAILS_IN_A_ROW


# What the trays hold

def test_the_trays_show_each_bins_value_and_what_went_in():
    eyes = FakeEyes(resistor(1, 0, -130, ohms=1500), resistor(2, 30, -100, ohms=2200), resistor(3, -30, -160, ohms=1500),
                    resistor(4, -30, -100, ohms=1e6))
    boss = supervisor(eyes, FakeArm(eyes))
    boss.run()
    trays = {t.bin_id: t for t in boss.bins.trays(boss.memory.sorted)}
    assert list(trays) == ["A1", "A2", "A3", "B4"]
    assert (trays["A1"].value_ohm, trays["A1"].count) == (1500, 2)
    assert (trays["A2"].value_ohm, trays["A2"].count) == (2200, 1)
    assert (trays["A3"].value_ohm, trays["A3"].count) == (None, 0)
    assert (trays["B4"].value_ohm, trays["B4"].count) == (None, 0)
    assert [r.number for r in eyes.plate] == [4]          # 1M: outside the range, left on the plate


class TrayWatcher:
    """A window that keeps every tray update, to check the run reports them."""

    def __init__(self):
        self.updates = []

    def show(self, view, plan, lines):
        return None

    def show_trays(self, trays, target=None):
        self.updates.append(({t.bin_id: t.count for t in trays}, target))

    def say(self, text):
        pass


def test_the_window_is_told_where_each_resistor_is_going_and_what_each_tray_holds():
    eyes = FakeEyes(resistor(1, 0, -130, ohms=1500))
    window = TrayWatcher()
    Supervisor(eyes, FakeArm(eyes), Planner(bin_plan(), PickMemory()), FakeLog(), window).run()
    assert ({"A1": 0, "A2": 0, "A3": 0, "B4": 0}, "A1") in window.updates      # on its way
    assert window.updates[-1] == ({"A1": 1, "A2": 0, "A3": 0, "B4": 0}, None)  # confirmed by the last look


def test_a_stop_takes_effect_between_picks():
    eyes = FakeEyes(resistor(1, 0, -130), resistor(2, 40, -100))
    arm = FakeArm(eyes)
    boss = supervisor(eyes, arm)
    original = arm.drop

    def drop_then_stop(bin_id):
        boss.request_stop()             # pressed while the arm is busy
        return original(bin_id)

    arm.drop = drop_then_stop
    assert boss.run() == "stopped: you asked"
    assert arm.done.count("park") == 2 and not arm.holding
    assert len(eyes.plate) == 1
    arm.drop = original
    assert boss.run().startswith("done")       # START again carries on with the same bins
    assert boss.bins.assigned == {1000.0: "A1"}


def test_looking_once_moves_nothing():
    eyes = FakeEyes(resistor(1, 0, -130))
    arm = FakeArm(eyes)
    plan = supervisor(eyes, arm).look_once()
    assert plan.next.resistor.number == 1 and arm.done == []


# Starting the arm after a power-on

class FakeBench:
    """slide_wrist's steps, recorded. off: how far the elbow lands from each target in the check."""
    MOVE_S, WRIST_TURN_S = 2.0, 2.0

    def __init__(self, off=0.3):
        self.off, self.steps, self.elbow = off, [], 0.0

    def start(self, scara):
        self.steps.append("start")

    def move(self, scara, joint, angle, seconds):
        self.steps.append(f"{joint} {angle:g}")
        if joint == "elbow":
            self.elbow = angle + self.off

    def park(self, scara):
        self.steps.append("park")

    def read_joints(self, scara, n=30):
        return -77.0, self.elbow

    def load_state(self):
        return {"started": "2026-11-11 15:00"}


class FakeNavigator:
    scara, holding, parked, status = None, False, False, ""


def test_starting_up_runs_the_readme_sequence_then_checks_the_elbow():
    from robot_control.motion.arm import Arm
    bench = FakeBench(off=0.3)
    check = Arm(FakeNavigator(), {}, bench).start_up(say=lambda text: None)
    assert bench.steps == ["start", "elbow 110", "park", "wrist 0", "elbow 105", "elbow 115", "elbow 110"]
    assert check.ok and check.worst_deg == pytest.approx(0.3)
    assert "good" in str(check)


def test_an_elbow_landing_far_off_fails_the_check():
    from robot_control.motion.arm import Arm
    check = Arm(FakeNavigator(), {}, FakeBench(off=4.0)).start_up(say=lambda text: None)
    assert not check.ok and "BAD" in str(check)


def test_the_arm_reports_when_it_was_last_started():
    from robot_control.motion.arm import Arm
    assert Arm(FakeNavigator(), {}, FakeBench()).last_started == "2026-11-11 15:00"


# The buttons

class FakeWindow(TrayWatcher):
    def show_arm(self, text):
        self.arm_line = text

    def show_aim(self, text):
        self.aim_line = text


def test_buttons_live_only_when_they_make_sense():
    from robot_control.task.app import SortingApp
    eyes = FakeEyes()
    app = SortingApp(supervisor(eyes, FakeArm(eyes)), FakeWindow())
    assert app.enabled() == {"CALIBRATE ARM", "CHECK AIM", "LOOK", "START"}
    looking_only = SortingApp(supervisor(eyes, None), FakeWindow())
    assert looking_only.enabled() == {"LOOK"}
    assert "not connected" in looking_only.window.arm_line


# Where the bins are

def test_a_slots_corners_lie_along_its_long_axis():
    from robot_control.task.rig_map import slot_corners
    corners = np.array(slot_corners((0.0, 0.0), 0.0, size=(66.0, 16.0)))
    assert corners[:, 0].max() == pytest.approx(33.0) and corners[:, 1].max() == pytest.approx(8.0)
    turned = np.array(slot_corners((10.0, 5.0), 90.0, size=(66.0, 16.0)))
    assert turned[:, 1].max() == pytest.approx(5.0 + 33.0) and turned[:, 0].max() == pytest.approx(10.0 + 8.0)


def test_the_map_draws_every_slot_without_the_twins_picture():
    from robot_control.planning.bins import Tray
    from robot_control.task.rig_map import RigMap
    slots = {"A1": {"centre_mm": [-130.0, 40.0], "long_axis_deg": 0.0},
             "B4": {"centre_mm": [-130.0, -110.0], "long_axis_deg": 0.0}}
    image = RigMap(slots).draw([Tray("A1", 1500.0, 2), Tray("B4", None, 0)], "A1", 440)
    assert image.shape[1] == 440 and image.shape[2] == 3


# A body seen before but missing from a look (segmentation can miss one look)

class FlickeringEyes(FakeEyes):
    """The plate as FakeEyes, but on the looks listed in `blind` segmentation misses everything, and says
    which bodies it had seen are missing."""

    def __init__(self, *resistors, blind=()):
        super().__init__(*resistors)
        self.blind, self.picked_at = set(blind), []

    def look(self):
        self.looks += 1
        if self.looks in self.blind:
            return PlateView("look", None, None, [], missing=[r.centre_mm for r in self.plate])
        return view(*self.plate)

    def picked(self, centre_mm):
        self.picked_at.append(centre_mm)


def test_the_run_looks_again_before_ending_while_a_known_body_is_missing():
    eyes = FlickeringEyes(resistor(1, 0, -130), resistor(2, 30, -100), blind={2})     # look 2 misses #2
    boss = supervisor(eyes, FakeArm(eyes))
    ended = boss.run()
    assert ended.startswith("done") and eyes.plate == []          # #2 was still picked
    assert "looking again" in boss.log.events


def test_the_eyes_are_told_where_each_pick_was():
    eyes = FlickeringEyes(resistor(1, 0, -130))
    supervisor(eyes, FakeArm(eyes)).run()
    assert eyes.picked_at == [(0.0, -130.0)]


# The run's log keeps the evidence

def test_every_look_is_kept_with_its_readings_and_crops(tmp_path):
    import csv as csv_module

    from robot_control.task.run_log import RunLog
    log = RunLog(root=tmp_path, keep_frames=True)
    crop = np.full((40, 120, 3), 200, np.uint8)
    body = Resistor(1, (0.0, -130.0), 0.0, 0.9, None, 1000.0, 0.55, np.zeros((4, 2)), closeup=True,
                    colours=("brown", "black", "black", "brown", "brown"), body_id=7, crop=crop, closeup_crop=crop)
    look = PlateView("look", np.zeros((10, 10, 3), np.uint8), None, [body],
                     faint=[(np.zeros((4, 2)), 0.31, (20.0, -110.0))], missing=[(-15.0, -130.0)])
    plan = Planner(bin_plan(), PickMemory()).plan(look)
    log.start_run()
    first = log.look(1, look, plan, image=np.zeros((10, 10, 3), np.uint8))
    log.start_run()
    second = log.look(1, look, plan, image=np.zeros((10, 10, 3), np.uint8))     # START again: cycle 1 again
    assert (first, second) == ("0001", "0002")
    assert (log.folder / "look_0001.jpg").exists() and (log.folder / "look_0002.jpg").exists()
    assert (log.folder / "frames" / "0001.jpg").exists()
    assert (log.folder / "crops" / "0001_body7.png").exists() and (log.folder / "crops" / "body7_closeup.png").exists()
    rows = list(csv_module.DictReader(open(log.folder / "readings.csv", newline="")))
    kinds = [r["kind"] for r in rows if r["look"] == "0001"]
    assert kinds == ["body", "faint", "missing"]
    b = rows[0]
    assert b["best"] == "1000.0" and b["confidence"] == "0.55" and b["colours"] == "brown black black brown brown"
    assert b["fate"].startswith("stays: unsure")
