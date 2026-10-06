# Hardware API

How to drive the SCARA arm from Python, on the real robot or in the Unity simulator: what the API is,
how to set it up, and how to use it safely.

The API lives in [`scara_control/`](../scara_control/). It was written by Alex
(AB3849/Applied-Robotics, commit `fa7c1a1`) and confirmed working in the simulator. Its own
[`readme.md`](../scara_control/readme.md) has the full method reference and wire protocol; this page
is the team's guide to setting it up and using it.

## What it is

```
your script / GUI
      │   ScaraController: limit checks, one command at a time, waits for each reply
      ▼
 transport ── serial (COM port, 115200 baud) ──► STM32 Nucleo on the real arm
          └── TCP (127.0.0.1:9000) ─────────────► Unity simulator
```

| File | What it does |
|---|---|
| `scara_controller.py` | `ScaraController`, the class you call. Talks to the robot over serial or to the sim over TCP (`SocketTransport`). Checks every command against the workspace limits before sending it. |
| `scara_workspace.py` | `WorkspaceLimits`: joint ranges, reach, forward/inverse kinematics, limit checks. No hardware needed. |
| `scara_gui.py` | Tkinter GUI with joint sliders and a Simulator/Hardware switch, for manual testing. |

The robot and the simulator speak the same protocol, so a script tested in the sim sends the same
bytes to the robot. The one exception is the slide (Z) in joint moves: see
[Before using the real arm](#before-using-the-real-arm).

## Setup

### Python

From the repo root, once:

```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

That installs `pyserial`, which the real robot needs. `tkinter` (for the GUI) comes with the standard
Windows and macOS Python installers.

### Real robot

1. **Servo power off.** Plug the Nucleo board's USB port into the laptop. That single cable is the
   whole link: the board's on-board ST-Link chip turns it into a serial port connected to the
   STM32's USART2.
2. **Find the COM port.** It appears in Device Manager under *Ports (COM & LPT)* as
   *STMicroelectronics STLink Virtual COM Port (COMx)*. If it doesn't appear, install ST's ST-Link
   driver (STSW-LINK009).
3. **Set the port in the GUI.** Edit `COM_PORT` at the top of `scara_control/scara_gui.py`, or pass
   the port to `ScaraController(port="COMx")` in your script.
4. Only one program can hold the COM port at a time. Close the GUI, serial monitors or other
   scripts before starting another.

### Simulator

1. Open `course_files/scara/mini_scara_simulator/` in Unity Hub, open `main_scene` and press **Play**.
   The simulator listens on `127.0.0.1:9000`.
2. Tip: turn on *Edit > Project Settings > Player > Resolution and Presentation > Run In Background*
   so the sim keeps running when its window loses focus.
3. The sim accepts one client at a time.

## Using it

### The GUI

```
python scara_control/scara_gui.py
```

1. Pick **Simulator** or **Hardware**, then press **Connect**. Connecting sends Initialise.
2. Move the sliders (with *Live Slider Mode* on), or set values and press **Send Current Values**.
   The status line shows the last target, or `Rejected: ...` if a move breaks the limits.
3. **Read Feedback** asks for the current position. Only the real robot answers; the sim doesn't
   support reads.

The GUI only sends joint moves (all four joints at once).

### From your own script

The files import each other by plain module name, so put `scara_control/` on the import path rather
than importing it as a package:

```python
import sys
sys.path.insert(0, "scara_control")   # path from the repo root

from scara_controller import ScaraController, SocketTransport
from scara_workspace import WorkspaceLimits, WorkspaceLimitError

# Simulator
limits = WorkspaceLimits(z=(0.025, 0.075))
scara = ScaraController(transport=SocketTransport("127.0.0.1", 9000), limits=limits)
# Real robot:
# scara = ScaraController(port="COM3", limits=WorkspaceLimits())   # Z 0.095-0.15 m

try:
    scara.initialize_robot(z_min=limits.z[0], z_max=limits.z[1])
    scara.move_joints(45, 30, 0, 0.05, 2.0)          # base deg, elbow deg, wrist deg, Z, seconds
    x, y = scara.limits.fk(20, 40)                   # a point known to be reachable
    scara.move_coordinates(x, y, 0, 0.05, 2.0)       # x m, y m, wrist deg, Z m, seconds
    print(scara.read_coordinates())                  # (x, y, wrist deg, z), or None
except WorkspaceLimitError as e:
    print("Rejected before sending:", e)
finally:
    scara.close()
```

`python scara_control/scara_controller.py sim` (or `... COM3`) runs a short built-in demo.

### Commands

| Method | Does | Returns |
|---|---|---|
| `initialize_robot(link1, link2, z_min, z_max, settling_time, P0, I0, D0, P1, I1, D1)` | Sets geometry, Z range, settling time and PID gains. All arguments have the course defaults. | `True` if acknowledged |
| `move_single_joint(axis, angle, move_time)` | Moves one joint: 0 base, 1 elbow, 2 wrist (degrees), 3 Z | `True` if acknowledged |
| `move_joints(base, elbow, wrist, z, move_time)` | Moves all four joints together | `True` if acknowledged |
| `move_coordinates(x, y, wrist, z, move_time)` | Moves the tip to x, y (metres); the board does the inverse kinematics | `True` if acknowledged |
| `read_joint_angle(axis)` | Reads one joint angle (real robot only) | angle in degrees, or `None` |
| `read_coordinates()` | Reads the tip position (real robot only) | `(x, y, wrist, z)`, or `None` |
| `auto_calibrate()` | Course auto-calibration. **Don't use:** see below | `True` if acknowledged |
| `close()` | Releases the port or socket | |

Things to know:

- **Units:** metres, degrees and seconds.
- **Every call blocks** until the board replies. On a move, the reply comes after the move and
  its settling time have finished. One command is sent at a time; the controller is thread-safe.
- **Limits are checked before sending.** A target outside the workspace raises
  `WorkspaceLimitError` and nothing is sent. A move time of 0 or less raises `ValueError`.
- **`False` means no acknowledgement:** a timeout or a refused command. After a timeout the arm may
  still have moved, or may still be moving.
- **Link failures** (cable pulled, sim closed) raise `serial.SerialException` or `OSError`.

### Workspace limits

From `scara_workspace.py`. Angles are measured from the +Y axis, clockwise (towards +X) positive:
x = L1·sin(q0) + L2·sin(q0 + q1), y = L1·cos(q0) + L2·cos(q0 + q1), with L1 = 0.125 m and
L2 = 0.100 m.

| | Limit |
|---|---|
| Base `q0` | −90° to 90° |
| Elbow `q1` | 0° to 90° (bends one way only) |
| Wrist | ±180° (assumed) |
| Z | sim 0.025–0.075 m, real robot 0.095–0.15 m |
| Tip distance from the base | 0.160–0.225 m |

`limits.contains(x, y)` says whether a point is reachable, and `limits.ik(x, y)` gives the joint
angles for it.

### How the link works

Each command is one 48-byte little-endian frame: an `int32` command ID, then eleven 4-byte values.
The board replies with an `int32` flag (`1` means success), followed by data for the read commands.
The full table is in [`scara_control/readme.md`](../scara_control/readme.md#6-wire-protocol-for-other-languages-or-tools).

The board can only take a new command once it has replied to the last one. While a move runs it
isn't listening, so a command sent then is lost.

## Before using the real arm

**Not damaging the arm comes first.** The API has only been tested in the simulator. These points
come from the first live tests on 2026-10-05 and from reading the firmware. Read them before
connecting to the real arm.

### Rules for every session

- **Two people:** one at the keyboard, one with a hand on the **servo power switch**. Nothing in
  software can stop a move already sent; only cutting servo power can.
- **Clear the workspace** within about 25 cm of the base.
- **The reset rule.** Once the board has sent a servo a position, it keeps sending it, even with
  servo power off. Turning servo power on in that state snaps every servo there at full speed. So
  whenever servo power has been off while the board was running, keep it off, press the Nucleo's
  black **reset** button, and only then turn servo power on.
- **Use slow moves:** a few seconds per move, and small steps at first.

### Known issues with the API on the real arm

These are open; they are being discussed with Alex.

| Issue | What happens | Until it is fixed |
|---|---|---|
| **Z in joint moves.** `move_joints` and `move_single_joint(3, ...)` send Z in metres. The real firmware reads axis 3 in joint moves as the slide servo's angle in degrees. | On the real arm the slide goes to roughly mid-travel, whatever Z you asked for. `move_coordinates` is fine: the firmware converts metres itself. | Use `move_coordinates` for height on the real arm, or don't rely on Z from joint moves. |
| **Fast moves.** The GUI's default move time is 0.5 s, and a click on a slider track can ask for 90° in that time. | Very fast, sudden motion. | Set the GUI's move time to 3 s or more, and drag sliders in small steps. |
| **First move.** The GUI sliders start at all zeros, i.e. arm straight out. | The first send swings every joint from wherever it rests to straight out. | Set the sliders near the arm's resting pose before the first send. |
| **Timeouts on long moves.** The controller waits move time + 5 s. A real move takes about 2.2 × move time plus the settling time (the firmware's 1 ms control loop really takes about 2 ms). | Moves longer than about 3 s return `False` while the arm is still moving. A command sent then is lost. | Keep moves under about 3 s, or wait before sending the next one. |
| **`auto_calibrate()`** sweeps the base and elbow to their limits, trusting the potentiometers. | If the potentiometers are off, it can drive a joint into its end stop. | Don't call it. |
| **Shoulder offset.** On our arm the base servo's zero is about 24° from its potentiometer's zero. | Base moves land up to about 13° short; the firmware can only correct about 10°. The board still acknowledges the move. | Check where it actually went with `read_coordinates()` / `read_joint_angle(0)`. The servo horn may be one spline tooth off. |

### Which firmware is on the board

The firmware is the STM32CubeIDE project in `course_files/scara/servo_motion_controller/`.

On 2026-10-05 the lab robot's Nucleo was flashed with a modified build of that firmware. It answers
commands 1–7 exactly like the course firmware, with these differences:

- It refuses moves outside the joint limits or faster than 90°/s.
- It refuses `auto_calibrate()`.
- It recovers from a partly received command instead of getting stuck.

A refusal shows up in the API as `False`. To return to the course firmware, rebuild and flash the
project in `course_files/` with STM32CubeIDE.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Could not connect to Hardware | Wrong COM port, port held by another program, or ST-Link driver missing |
| Could not connect to Simulator | Unity not in Play mode, wrong port, or another client already connected |
| `Rejected: ...` / `WorkspaceLimitError` | Target outside the workspace; the message says which joint or why |
| Init "NOT ACKNOWLEDGED" | Board not responding (wrong port, unpowered), or the sim didn't acknowledge |
| A move returns `False` | Timed out (see the timeout issue above) or the board refused it |
| Every reply after one failure is wrong or missing | The link is out of step. Press the Nucleo's reset button (with servo power off) and reconnect |
| `ModuleNotFoundError: scara_workspace` | Put `scara_control/` on `sys.path`, or run from inside that folder |

More in [`scara_control/readme.md`](../scara_control/readme.md#8-troubleshooting).
