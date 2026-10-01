# Build-out plans

One plan per module (or tightly linked pair), written before the code. Each plan states its goal,
what is in and out of scope, the design, the config it needs, and when it counts as done.
The overall design is in [../architecture.md](../architecture.md).

## Plans

| Plan | Stage | Status | Depends on |
|---|---|---|---|
| [01 World and config](01-world-and-config.md) | 1 | Draft | nothing |
| [02 Hardware](02-hardware.md) | 1 | Draft | 01 |
| [03 Kinematics](03-kinematics.md) | 1 | Draft | 01 |
| [04 Gripper interface](04-gripper-interface.md) | 1 | Draft | 01 |
| [05 Motion](05-motion.md) | 1 | Draft | 01–04 |
| Supervisor, fake perception, planning | 2 | Not written | stage 1 |
| Vision: camera, calibration, detection | 3 | Not written | 01 |
| Real gripper drivers | 4 | Not written | 04, gripper hardware |
| Classifier | 5 | Not written | 01, vision |

**Stage 1 is done when** the arm, in the simulator, can run a full pick-and-place sequence between a
given table pose and a bin, with the dummy gripper, and refuses any pose it cannot reach before
moving at all.

Plans 02, 03 and 04 only depend on 01, so they can be built in parallel.

## Conventions for all modules

- **Units:** metres and degrees everywhere in our code. Conversions to anything else (servo degrees
  for the real slide, pixels) happen only at the edge that needs them.
- **Robot frame:** the firmware's. Origin at the shoulder axis. Angle 0 points along +y, and positive
  angles turn toward +x (from `calc_forward_kinematics`: x = l1·sin θ1 + l2·sin(θ1+θ2),
  y = l1·cos θ1 + l2·cos(θ1+θ2)). Yaw uses the same convention.
- **Heights:** tool poses use height above the table surface. Only kinematics converts this to a
  slide position.
- **Layering:** a module imports only modules below it. Nothing imports sideways; the Supervisor
  passes results between modules.
- **No magic numbers:** dimensions, limits, heights, times and poses come from `robot_control/config/`.
- **No hard-coded resistor sizes:** sizes come from the Detection (nice-to-have, see architecture).
- **Testable without hardware:** every module except the hardware adapters can be unit-tested with
  no simulator or robot running. Tests go in `tests/`, one file per module, using pytest
  (add it to `requirements.txt` with the first test).
- **Run from the repo root** so `hardware_api` and `robot_control` both import.

## Housekeeping

- `robot_control/sim_joint_test.py` on branch `20260928-joint-sim-test` hard-codes joint limits. Once
  plans 01 and 02 are built, switch it to the config limits and the SimRobot adapter.
