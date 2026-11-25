"""The simplest calibration: start the arm and check the elbow, nothing else.

    .venv\\Scripts\\python base_scripts\\calibrate_start_only.py

Before it, with the servo supply off: slide fully up by hand, elbow folded to 100 degrees or more, gripper
turned away from the camera tower; then supply on. Unplug the servo supply to stop the arm.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

import resistor_nav
from sort_resistors import build_arm

arm, _ = build_arm(None, resistor_nav.load_bins())  # the Nucleo and the gripper

# The base and elbow sweep end to end, the arm parks, then the elbow goes to 105, 110 and 115 deg.
elbow = arm.start_up()
print(elbow)
if not elbow.ok:
    sys.exit("Elbow check failed: free the cables, set the arm by hand again and rerun.")
print("Started. The aim is left as camera_calibration.json has it.")
