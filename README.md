# EE52043 - Applied Robotics

Project workspace for the EE52043 robot coursework.

## Layout

- `hardware_api/` - the Python API for talking to the (simulated) SCARA motion controller,
  e.g. `sim_scara_motion_controller_api.py`.
- `robot_control/` - your own robot control code (kinematics, trajectory planning, etc.).
- `course_files/` - reference material from the module's GitHub repo
  (`Applied_Robotics_Files`). Not version-controlled by default - see
  `course_files/README.md` for how to populate it.
- `tests/` - tests for `robot_control/`.

## Setup

```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

## Usage

```python
from hardware_api.sim_scara_motion_controller_api import sim_scara_motion_controller

controller = sim_scara_motion_controller()
```
