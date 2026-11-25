"""The CALIBRATE button on one page, without the window: for explaining how calibration works.

    .venv\\Scripts\\python base_scripts\\calibrate_simple.py

It does what CALIBRATE does in scripts/sort_resistors_auto.py (robot_control/task/auto_app.py), without the
ohmmeter check. Before it, with the servo supply off: slide fully up by hand, elbow folded to 100 degrees or
more, gripper turned away from the camera tower; then supply on. Unplug the servo supply to stop the arm.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav
from sort_resistors import build_arm
from robot_control.task.aim_check import AimCheck, LEFT_OK_MM, MIN_POINTS
from robot_control.vision.eyes import Eyes

arm, _ = build_arm(None, resistor_nav.load_bins())  # the Nucleo and the gripper
eyes = Eyes(closeups=False)                         # the camera and the segmentation model

try:
    # 1. Start the arm: the base and elbow sweep end to end, then the elbow goes to 105, 110 and 115 deg.
    elbow = arm.start_up()
    print(elbow)
    if not elbow.ok:
        sys.exit("Elbow check failed: free the cables, set the arm by hand again and rerun.")

    # 2. Find the plate with the arm out of view, and check it's empty.
    arm.park()
    if eyes.count_bodies():
        sys.exit("The plate isn't empty: clear it and rerun.")

    # 3. Check the aim: the closed tip down at 5 points, the camera finds it each time. Any steady shift
    #    between where the arm went and where it was sent is taken out and saved to camera_calibration.json.
    aim = AimCheck(eyes, arm).run()
    print(aim.verdict)
    if aim.found < MIN_POINTS or aim.rms_after_mm > LEFT_OK_MM:
        sys.exit("Aim still off after the shift: the full calibration is needed (README).")
    print("Calibrated.")
finally:
    eyes.close()
