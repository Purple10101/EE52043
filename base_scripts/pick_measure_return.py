"""Find the one resistor on the plate, pick it, measure it on the ohmmeter, then put it back where it lay.

    .venv\\Scripts\\python base_scripts\\pick_measure_return.py

For testing the ohmmeter station. Only segmentation runs: the bands aren't read, so the ohmmeter's reading
is the only value. The arm must already be started (slide_wrist.py start, or CALIBRATE in the sorting window)
and the station taught (config/ohmmeter_station.toml). Unplug the servo supply to stop the arm.
"""

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav
import slide_wrist as bench
from sort_resistors import build_arm
from robot_control.config import OhmmeterSettings
from robot_control.hardware import ohmmeter
from robot_control.vision.eyes import Eyes

if bench.load_state() is None:
    sys.exit("The arm hasn't been started: run scripts/slide_wrist.py start first.")
settings = OhmmeterSettings.from_config()
station = settings.station                      # where the meter's pads are
arm, _ = build_arm(None, resistor_nav.load_bins())
nav = arm.navigator
meter = ohmmeter.from_config(settings)
eyes = Eyes(closeups=False)

try:
    # 1. Find it: the arm out of view, then segmentation only.
    arm.park()
    view = eyes.look(sharp=True)
    if len(view.resistors) != 1:
        sys.exit(f"Expected one resistor on the plate, found {len(view.resistors)}.")
    r = view.resistors[0]
    print(f"found it at ({r.centre_mm[0]:.1f}, {r.centre_mm[1]:.1f}) mm, lying at {r.direction_deg:.0f} deg")

    # 2. Pick it: over it, jaws across it, down, close, up.
    if not arm.pick(r):
        sys.exit(f"Couldn't pick it: {arm.message}")

    # 3. Onto the ohmmeter: over the station, leads along the pads, down, measure, up.
    if arm.press_on_meter(station):
        m = meter.measure()
        print(f"measured {m.ohms} ohm -> {m.nominal_ohm} ohm (E24)" + (f": {m.why}" if m.why else ""))
    else:
        print(f"Couldn't reach the ohmmeter: {arm.message}")
    arm.lift()

    # 4. Put it back where it lay: over the spot, jaws along it as picked, down, open, up.
    base, elbow, wrist = resistor_nav.plan_jaws(r.centre_mm, r.direction_deg, bench.SAFE["wrist"], nav.kinematics,
                                                nav.rig, nav.wrist_centre, nav.correction, at_axis=nav.aim_axis,
                                                jaw_offset=nav.jaw_offset)
    if nav.travel("where it lay", base, elbow, wrist) is None:
        sys.exit("Couldn't get back to its spot: STILL HOLDING it.")
    nav.drive("slide", resistor_nav.PICK_SLIDE, resistor_nav.SLIDE_MOVE_S)
    nav.gripper.open()
    time.sleep(resistor_nav.GRIP_SETTLE_S)
    nav.holding = False
    arm.park()
    print("put back and parked")
finally:
    eyes.close()
    meter.close()
