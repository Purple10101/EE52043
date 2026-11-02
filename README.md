# EE52043 - Applied Robotics

Project workspace for the EE52043 robot coursework.

## Layout

- `robot_control/` - our robot control code, one subpackage per module (`world`, `config`,
  `hardware`, `kinematics`, `gripper`, `motion`, `vision`, `classify`, `planning`, `task`).
- `docs/architecture.md` - the agreed software architecture. Start here.
- `docs/plans/` - build-out plans, one per module, written before the code.
- `scara_control/` - the team's control interface: `ScaraController` (real robot over serial or
  the simulator over TCP, with workspace limit checks), kinematics in `scara_workspace.py`, and a
  Tkinter test GUI. From AB3849/Applied-Robotics commit fa7c1a1 (Alex).
- `docs/hardware-api.md` - how to set up and use `scara_control/`, and what to know before
  driving the real arm.
- `hardware_api/` - Python APIs for the SCARA, provided by the course:
  - `scara_motion_controller_api.py` - real robot, over serial (needs `pyserial`).
  - `sim_scara_motion_controller_api.py` - Unity simulator, over TCP (127.0.0.1:9000).
- `course_files/` - everything else from the module repo (examples, CAD, simulator, firmware,
  machine vision). See `course_files/README.md`.
- `firmware/` - the motion controller firmware flashed on the arm: the course's Moodle build
  (`servo_motion_controller.zip`, with the DEMO switch) and this arm's potentiometer calibration in
  `potentiometers.c`. Set up by `course_files/lab_scripts/RobotSetup.md`. Builds with CMake and the
  Arm GNU Toolchain (`cmake -S firmware -B firmware/build -G Ninja`, then `cmake --build firmware/build`);
  flash by copying `firmware/build/servo_motion_controller.bin` to the Nucleo's drive.
- `scripts/` - bench tools for the real arm: `pot_check.py` and `pot_calibrate.py` (potentiometers,
  servos off), `go_to.py` (one position), `reach_test.py` and `base_reach.py` (what the arm reaches),
  `elbow_servo_hold.py` (holds the elbow servo at an end, for refitting its link).
- `tests/` - tests for `robot_control/` (`python -m pytest`).
- Our printed parts (gripper, camera tower) aren't in this repo: their CAD, print files and design
  notes are in the sibling `Robot Resources` folder (`gripper/`, `camera_tower/`).

## Setup

```
py -3.11 -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

The vision (`robot_control/vision/`) uses the sibling repo
[EE52043_offline](https://github.com/Purple10101/EE52043_offline) as a package: rectification, the
segmentation model and its camera and plate settings all come from there, so training and the robot
always match. Clone it next to this repo (`Robot Resources/robot_offline`), fetch its model with
`git lfs pull` there, then install the CUDA build of torch first (so ultralytics doesn't pull in the
CPU one) and the package as editable (its `configs/` are found through the source tree):

```
pip install torch==2.14.1 torchvision==0.29.1 --index-url https://download.pytorch.org/whl/cu126
pip install -e "..\Robot Resources\robot_offline"
```

## Dry run: from the camera to arm commands

Snapshot, detect, map each resistor body to the robot frame, check it against the arm's limits, and
print the move it would get. Nothing is sent to the arm.

```
python -m robot_control.task.plan_picks --live                         # one snapshot from the tower camera
python -m robot_control.task.plan_picks path\to\frame.png               # or a saved raw frame, or a folder
```

Overlays (top-down view, reach limit in blue, reachable bodies green) and JSON plans go to
`logs/plan_picks/`.

Design and open points: [docs/plans/06-vision-front-end.md](docs/plans/06-vision-front-end.md).

## Live plan window

The same front end run a few times a second, in a window like robot_offline's `live_detect`. It
shows where every resistor on the plate is in the robot frame and whether the arm can reach it.
Nothing is sent to the arm.

```
python -m robot_control.task.live_plan                         # tower camera
python -m robot_control.task.live_plan --frames <folder>       # saved frames, no rig needed
```

**With the tower camera:** plug the Arducam in and close anything else using it (Camera app, Teams,
OBS). The whole plate must be in view, front face included, with hands and leads off its edges. The
camera takes a few seconds to settle and the plate is found about 1.5 s after that; until then the
plain camera image is shown. The plate is fitted again every 5 s in the background, and the exposure
follows the light, as in `live_detect`.

**With saved frames:** `<folder>` is a folder of raw 4608 × 2592 frames (or one image), e.g. the
robot_offline captures or frames saved with `s`. One is shown at a time; `n` moves to the next and
finds the plate in it again.

**What you see:**

- The top-down view of the plate with a 10 mm grid (`v` switches to the camera's own view).
- The arm's reach limit in blue: closer to the base than this, the elbow would have to bend past its
  120° limit.
- Each body numbered: green if the arm can reach it, red if not, the next pick ringed in white.
- On the right, one row per body: its robot-frame centre (mm), yaw and size, then the joint angles
  and the `move_coordinates(...)` call that would put the open gripper above it, or why it is skipped.
- At the top: bodies found and reachable, plans per second, how the plate was found, the exposure.

| Option | Default | Meaning |
|---|---|---|
| `--frames` | none: the camera | Saved raw frames (an image or a folder) instead of the camera. |
| `--fps` | `3` | Plans per second. |
| `--conf` | `conf` in `robot_control/config/vision.toml` (0.5) | Confidence threshold. |
| `--out` | `logs/live_plan` | Where `s` saves. |

| Key | Action |
|---|---|
| `q` / `Esc` | Quit. |
| `v` | Top-down view or camera view. |
| `g` | Grid on or off. |
| `+` / `-` | Raise or lower the confidence threshold by 0.05. |
| `c` | Find the plate again now, from scratch (after moving the camera or the plate). |
| `e` | Exposure following the light, or fixed at robot_offline's `configs/camera.toml` (camera only). |
| `n` | Next saved frame (`--frames` only). |
| `s` | Save the view (`.jpg`), the raw frame (`_raw.png`, which `--frames` can replay) and the plan (`.json`). |

Bodies whose pose would put the arm into a no-go zone (below) are skipped with the reason.

The `move_coordinates` values aren't ready to send yet: the slide height (`hover_z_m`) and the jaw
direction at tool angle 0 are placeholders, and the scene-to-robot transform still has to be checked
on the arm (see the plan's open points).

## No-go zones (the camera tower)

The arm can hit the camera tower. The firmware moves all joints together on one time-scaled
profile, so every move is a straight line in joint space (base, elbow), not in x/y. A no-go zone is
therefore a map of the joint angles where any part of the arm or gripper comes within a clearance of
the obstacle, at any wrist angle and slide height. No target may be inside it and no move may cross it.

- **The map:** `robot_control/config/keep_out/camera_tower.json` (1° grid, 10 mm clearance, the
  parallelogram gripper fitted), listed in `workspace.toml` as `keep_out = ["camera_tower"]`.
- **Enforced:** `Kinematics.ik` refuses any pose inside a zone, so `plan_picks` and `live_plan` skip it.
- **Moves:** `robot_control.kinematics.first_hit(zones, a, b)` says where a straight move first enters a
  zone; `route(zones, a, b, elbow_max)` returns waypoints that go round it (fold the elbow, swing the
  base, unfold). The motion code must use these for every move once it exists.
- **See it:** `python scripts\show_keep_out.py` writes `logs/keep_out.png`: the zone in joint space,
  with the joint angles the plate and the trays need.

Today the zone leaves the pick plate and tray Body1:3 untouched, but blocks about half of tray
Body1:4 (41% of it stays usable).

**Making or re-making a map** (after moving the tower, changing the gripper, or for a new obstacle):
`scripts/keep_out_map.py` builds it from the CAD in `../Robot Resources`. It needs OpenCascade, kept
out of the project venv:

```
py -3.11 -m venv cadenv
cadenv\Scripts\python -m pip install cadquery-ocp numpy scipy
cadenv\Scripts\python scripts\keep_out_map.py                     # about 6 minutes
cadenv\Scripts\python scripts\keep_out_map.py --margin 15         # more clearance
cadenv\Scripts\python scripts\keep_out_map.py --tower other.step --out robot_control\config\keep_out\other.json
```

A new map is used once its name is added to `keep_out` in `workspace.toml`.

## Usage

Run scripts from the repo root so `hardware_api` is importable:

```python
from hardware_api.sim_scara_motion_controller_api import sim_scara_motion_controller
from hardware_api.scara_motion_controller_api import scara_motion_controller
```
