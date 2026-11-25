"""The resistor sorter on one page, without the window: for explaining how the task works.

    .venv\\Scripts\\python base_scripts\\sort.py

It does what scripts/sort_resistors_auto.py does, with the window, the run log, the ohmmeter and the
retries left out. Resistors it can't read stay on the plate. Unplug the servo supply to stop the arm.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav
from sort_resistors import build_arm
from robot_offline.post.bands.values import SORTER_OHM
from robot_control.planning.bins import BinPlan
from robot_control.planning.memory import Attempt, PickMemory
from robot_control.planning.planner import Planner
from robot_control.task.aim_check import AimCheck
from robot_control.vision.eyes import Eyes

# 1. Build the parts.
slots = resistor_nav.load_bins()                # where the eight tray slots are (A1..B4)
bins = BinPlan.from_slots(slots, SORTER_OHM)    # which value goes in which slot
planner = Planner(bins, PickMemory())           # picks the next resistor and its bin
arm, _ = build_arm(None, slots)                 # the Nucleo (base, elbow, slide, wrist) and the gripper
eyes = Eyes()                                   # the camera, the segmentation model and the band reader

# 2. Calibrate, with the plate empty.
input("Plate EMPTY, arm set by hand, servo supply on. Press Enter to calibrate.")
arm.start_up()                                  # the base and elbow sweep end to end, then the elbow check
arm.park()                                      # out of the camera's view
if eyes.count_bodies():                         # finds the plate and counts what's on it
    sys.exit("The plate isn't empty.")
AimCheck(eyes, arm).run()                       # tip down at 5 points; the camera-to-arm shift is corrected

# 3. Read every resistor's value once (a focus sweep, then a close-up of each).
input("Lay the resistors on the plate, keeping the crosshair clear. Press Enter to sort.")
eyes.read_values()

# 4. Sort: look, plan, pick, drop, and again until nothing is left to sort.
while True:
    arm.park()                                  # out of view
    view = eyes.look()                          # every resistor: where it lies and its value
    planner.memory.judge(view)                  # was the last pick a success? (its spot is empty now)
    plan = planner.plan(view)                   # the next resistor, and the bin its value goes in
    if plan.next is None:
        break
    r, bin_id = plan.next.resistor, plan.next.bin_id
    print(f"picking #{r.number} ({r.value_ohm} ohm) for {bin_id}")
    if not arm.pick(r):                         # over it, down, close the jaws, up
        planner.memory.give_up(r.centre_mm, "the arm couldn't reach it")
        continue
    planner.memory.attempted(Attempt(r.centre_mm, r.value_ohm, bin_id))
    eyes.picked(r.centre_mm)
    arm.drop(bin_id)                            # over the bin, down, open the jaws, up
    bins.assign(r.value_ohm)                    # that value keeps this bin from now on

arm.park()
eyes.close()
print("Done: nothing left on the plate that can be sorted.")
