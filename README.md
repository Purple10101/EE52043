# EE52043 - Applied Robotics

Project workspace for the EE52043 robot coursework.

## Layout

- `robot_control/` - your own robot control code.
- `hardware_api/` - Python APIs for the SCARA, provided by the course:
  - `scara_motion_controller_api.py` - real robot, over serial (needs `pyserial`).
  - `sim_scara_motion_controller_api.py` - Unity simulator, over TCP (127.0.0.1:9000).
- `course_files/` - everything else from the module repo (examples, CAD, simulator, firmware,
  machine vision). See `course_files/README.md`.
- `tests/` - tests for `robot_control/`.

## Setup

```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

## Usage

Run scripts from the repo root so `hardware_api` is importable:

```python
from hardware_api.sim_scara_motion_controller_api import sim_scara_motion_controller
from hardware_api.scara_motion_controller_api import scara_motion_controller
```
