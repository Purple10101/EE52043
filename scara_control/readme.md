# SCARA Control Tools

Python tools for driving the SCARA robot either on the **real hardware** (STM32 over a serial port) or in the **Unity simulator** (over TCP). The GUI is for manual testing; the same controller class can be called directly from your own scripts (e.g. the ML vision pipeline).

| File | Purpose |
|---|---|
| `scara_workspace.py` | Working limits (joint ranges, reach), forward/inverse kinematics, limit checks |
| `scara_controller.py` | Protocol client: `ScaraController` (the API you call) and `SocketTransport` (TCP for the sim) |
| `scara_gui.py` | Tkinter test GUI with a Simulator/Hardware switch |

All three files must sit in the same folder.

---

## 1. Requirements

- Python 3.8+
- `pyserial` (hardware only): `pip install pyserial`
- `tkinter` (GUI only; ships with the standard Windows/macOS Python installers, on Linux `sudo apt install python3-tk`)

The controller and workspace modules have no other dependencies. `numpy`/`matplotlib` are only needed if you plot the workspace.

---

## 2. Quick start

### Using the GUI

1. **Simulator:** open the Unity scene and press Play so the socket server is listening on `127.0.0.1:9000`. 

    > **NOTE:** Before running the simulation I had better results turning on run in background *(Edit > Project Settings > Player > Platform > Resolution and Presentation > Run In Background)*
   
   
   **Hardware:** connect the Nucleo and note its COM port.
2. Edit the config block at the top of `scara_gui.py` if needed:
   ```python
   COM_PORT = "COM3"
   BAUDRATE = 115200
   SIM_HOST, SIM_PORT = "127.0.0.1", 9000
   LINK_1, LINK_2 = 0.125, 0.1          # metres
   PROFILES = {                         # Z travel per backend, metres
       "Simulator": dict(z_min=0.025, z_max=0.075),
       "Hardware":  dict(z_min=0.095, z_max=0.15),
   }
   ```
3. Run `python scara_gui.py`, pick **Simulator** or **Hardware**, press **Connect**.
4. Drag the sliders (Live Slider Mode on) or use **Send Current Values**. The status line shows the last target, or `Rejected: ...` if a move violates the working limits.

The GUI only does joint-space moves. **Read Feedback** needs read support on the other end (see section 6).

### Using the commands without the GUI

```python
from scara_controller import ScaraController, SocketTransport
from scara_workspace import WorkspaceLimits, WorkspaceLimitError

# --- Simulator ---
limits = WorkspaceLimits(z=(0.025, 0.075))
scara = ScaraController(transport=SocketTransport("127.0.0.1", 9000), limits=limits)

# --- or Hardware ---
# scara = ScaraController(port="COM3", limits=WorkspaceLimits())   # default Z 0.095-0.15 m

try:
    scara.initialize_robot(z_min=limits.z[0], z_max=limits.z[1])
    # scara.auto_calibrate()          # if your setup needs it (see section 6)

    z = 0.05
    scara.move_joints(45, 30, 0, z, 1.5)               # base°, elbow°, wrist°, Z m, time s
    x, y = scara.limits.fk(20, 40)                      # a point known to be reachable
    scara.move_coordinates(x, y, 0, z, 1.0)             # x m, y m, wrist°, Z m, time s
    print(scara.read_coordinates())                     # (x, y, z_angle, z) or None
except WorkspaceLimitError as e:
    print("Rejected before sending:", e)
finally:
    scara.close()
```

Or run the built-in demo: `python scara_controller.py sim` or `python scara_controller.py COM3`.

> Only one program can hold the serial port (and the sim may accept only one client), so close the GUI before running a script against the same backend.

---

## 3. `ScaraController` reference

```python
ScaraController(port=None, baudrate=115200, timeout=2.0,
                transport=None, limits=None, enforce_limits=True)
```

Pass **either** `port` (serial, hardware) **or** `transport` (e.g. `SocketTransport`). `limits` overrides the default `WorkspaceLimits()`; `enforce_limits=False` turns the checks off entirely (not recommended).

All lengths are **metres**, all angles **degrees**, time in **seconds**.

| Method | Command ID | Arguments | Returns |
|---|---|---|---|
| `initialize_robot(...)` | 1 | `link1, link2, z_min, z_max, settling_time, P0, I0, D0, P1, I1, D1` (defaults: 0.125, 0.1, 0.095, 0.15, 1.0, 1.3, 0.01, 0.001, 1.3, 0.01, 0.001) | `bool` ack |
| `auto_calibrate()` | 2 | none | `bool` ack |
| `move_single_joint(axis, angle, move_time)` | 3 | axis 0=base, 1=elbow, 2=wrist (deg), 3=Z (m) | `bool` ack |
| `move_joints(angle0, angle1, angle2, z, move_time)` | 4 | base, elbow, wrist (deg), Z (m), time | `bool` ack |
| `move_coordinates(x, y, z_angle, z, move_time)` | 5 | tip x, y (m), wrist (deg), Z (m), time | `bool` ack |
| `read_joint_angle(axis)` | 6 | axis | `float` or `None` |
| `read_coordinates()` | 7 | none | `(x, y, z_angle, z)` or `None` |
| `close()` | n/a | none | n/a |

Behaviour worth knowing:

- **Return values:** move/init/calibrate return `True` only if the 4-byte ack was `1`. `False` means timeout or a rejected command. Reads return `None` on failure.
- **Limit checks happen first.** A violation raises `WorkspaceLimitError` (a `ValueError` subclass) and **nothing is sent**. A trajectory time of 0 or less raises `ValueError`.
- **Thread-safe.** Each command/reply pair runs under a lock, and stale bytes are flushed before each send, so a late ack from an earlier timeout can't be mistaken for a new reply.
- **Timeouts.** Default is `timeout` (2 s). Moves wait `max(timeout, move_time + 5)` s because the ack may only arrive once the move finishes; `auto_calibrate()` waits 30 s.
- **`scara.limits` updates itself.** After a successful `initialize_robot(...)`, the controller replaces `scara.limits` with a copy using the link lengths and Z range you sent. Read `scara.limits` rather than holding on to the original object.
- **Errors from the link** (socket reset, unplugged serial port) raise the underlying exception (`OSError`, `serial.SerialException`). Catch these if your script should survive a dropped connection.

---

## 4. Working limits (`scara_workspace.py`)

Derived from the workspace-boundary script. Convention (angles from the +Y axis, clockwise positive):

```
x = L1*sin(q0) + L2*sin(q0 + q1)
y = L1*cos(q0) + L2*cos(q0 + q1)
```

| Quantity | Limit |
|---|---|
| Base `q0` | -90° to +90° |
| Elbow `q1` | 0° to +90° (bends one way only) |
| Wrist | ±180° (assumed, not in the original script) |
| Z | per backend (sim 0.025-0.075 m, hardware 0.095-0.15 m) |
| Tip reach | 0.160 m to 0.225 m from the base axis |

The reachable region is asymmetric: the right-hand wing dips below the X axis and the left-hand wing rises above it.

```python
from scara_workspace import WorkspaceLimits
L = WorkspaceLimits()

L.fk(30, 45)              # joint angles (deg) -> tip (x, y) in metres
L.ik(0.05, 0.2)           # tip -> (base°, elbow°) on the elbow>=0 branch, or None
L.contains(0.1, 0.1)      # True/False: is this tip position reachable within limits?
L.reach_range()           # (r_min, r_max) = (0.160, 0.225)
L.project(0.1, 0.1)       # reachable point near an out-of-limits target
L.polygon()               # boundary as [(x, y), ...] in metres, for plotting
L.check_coord(x, y, z_angle, z)   # raises WorkspaceLimitError with the reason
```

Joint moves are checked directly against the joint ranges; Cartesian moves are checked with inverse kinematics plus the joint ranges, so the allowed set is exactly the green region of the original plot.

---

## 5. Hooking up an ML / vision pipeline

A pattern that keeps the arm safe and gives clear failure handling:

```python
from scara_controller import ScaraController, SocketTransport
from scara_workspace import WorkspaceLimits, WorkspaceLimitError

Z_HOVER, Z_PICK = 0.075, 0.03          # sim values; use the hardware Z range on the real arm

limits = WorkspaceLimits(z=(0.025, 0.075))
scara = ScaraController(transport=SocketTransport("127.0.0.1", 9000), limits=limits)
scara.initialize_robot(z_min=0.025, z_max=0.075)

def go_to(x, y, z, t=1.0):
    """Move the tip to (x, y, z). Returns False if unreachable or not acknowledged."""
    if not scara.limits.contains(x, y):
        print(f"skip: ({x:.3f}, {y:.3f}) is outside the workspace")
        return False
    return scara.move_coordinates(x, y, 0.0, z, t)

try:
    while True:
        target = detect_target()          # your ML code -> (x, y) in ROBOT frame, metres
        if target is None:
            continue
        x, y = target
        if go_to(x, y, Z_HOVER) and go_to(x, y, Z_PICK, 0.8):
            ...                           # grasp, then lift
            go_to(x, y, Z_HOVER, 0.8)
except KeyboardInterrupt:
    pass
finally:
    scara.close()
```

Notes for this stage:

- **Pixel to robot frame:** `detect_target()` must return coordinates in the robot's x/y frame (metres). You'll need a camera-to-robot calibration (e.g. a homography from a few known points); the controller doesn't provide one.
- **Unreachable targets:** the example skips them. `scara.limits.project(x, y)` returns a reachable point near the target, but it silently moves the arm somewhere else, so use it only to absorb small overshoots at the edge.
- **Path shape:** limits are checked at the end points only. The inner edge of the workspace is concave, so if the firmware interpolates in Cartesian space, a straight line between two valid points can leave the workspace. Moving in joint space (`move_joints`, using `limits.ik`) keeps the path inside the joint rectangle.
- **Blocking:** every call blocks until the ack arrives. Call from a worker thread if your vision loop must keep running; the lock makes sharing one controller between threads safe.
- **Retries:** `False` is not necessarily a failed move; it can be a timeout where the arm still moved. Use `read_coordinates()` (when supported) to confirm the final position.

---

## 6. Wire protocol (for other languages or tools)

Every command is one **48-byte, little-endian frame**: an `int32` command ID followed by 11 four-byte slots, each a `float32` (or `int32` where noted). Unused slots are zero.

```
Python:  struct.pack('<i11f', cmd_id, v0, ..., v10)
C:       typedef struct { int32_t command_id; int32_t values[11]; } command_t;   // read floats with memcpy
```

| ID | Command | Slots (values[0..]) | Reply |
|---|---|---|---|
| 1 | Initialise | link1, link2, z_min, z_max, settling_time, P0, I0, D0, P1, I1, D1 (floats) | `int32` flag |
| 2 | Auto-calibrate | none | `int32` flag |
| 3 | Move joint | axis (**int32**), angle, time | `int32` flag |
| 4 | Move joints | angle0, angle1, angle2, z, time | `int32` flag |
| 5 | Move coord | x, y, z_angle, z, time | `int32` flag |
| 6 | Read angle | axis (**int32**) | `int32` flag + `float32` angle (8 bytes) |
| 7 | Read coord | none | `int32` flag + 4 x `float32` x, y, z_angle, z (20 bytes) |

The success flag is `1` on success. Note that the firmware's source comments describe 48-byte return frames, but the code actually sends only the bytes listed above, and the Python client follows the code.

Transports: serial 115200 baud on the Nucleo's ST-Link virtual COM port (USART2), or raw TCP to the simulator on port 9000. The protocol is identical, so a script tested against the sim sends the same bytes the hardware will receive.

---

## 7. Known limitations and assumptions

- **Limit convention is unverified against the robot.** The kinematics follow the workspace script. Check once by commanding a joint pose and comparing `read_coordinates()` with `scara.limits.fk(q0, q1)`. If they disagree, the sign or zero offset in `fk` needs adjusting.
- **Older test coordinates fall outside the limits.** Points like `(0.1, 0.1)`, `(0.10, 0.08)`, `(0, 0.13)` are closer than the 0.160 m minimum reach and will be rejected.
- **Sim read support.** The sim client only implements commands 1, 3, 4 and 5. Read Feedback / `read_coordinates()` returns `None` until the Unity side handles commands 6 and 7 with the replies in the table above.
- **Sim Z range** (0.025-0.075 m) is taken from the example calls; set it to match your Unity model.
- **Calibration:** the GUI doesn't call `auto_calibrate()`. Whether the potentiometer-based moves need it first depends on your firmware setup.
- **No stop command.** The firmware has no abort/e-stop command, and moves block it until they finish. Keep trajectory times sensible and have a physical way to cut power on the real arm.
- **Wrist range** (±180°) is an assumption.

---

## 8. Troubleshooting

| Symptom | Likely cause |
|---|---|
| "Could not connect to Simulator" | Unity not in Play mode, wrong port, or another client already connected |
| "Could not connect to Hardware" | Wrong `COM_PORT`, port held by another program (close the GUI/serial monitor) |
| `Rejected: ...` in the GUI, or `WorkspaceLimitError` | Command outside the limits; the message says which joint or why the point is unreachable |
| Init "NOT ACKNOWLEDGED" | Firmware not responding, wrong baud/port, or the sim didn't ack command 1 |
| `False` from a move / "No ack" | Timeout (move longer than the wait) or the other end rejected it |
| "No feedback" | Backend doesn't implement commands 6/7 (sim), or the read timed out |
| Replies seem offset by one command | Shouldn't occur now (buffer is flushed per command); if it does, a frame was partially sent: restart the firmware or the sim connection |
| Sim Z moves rejected | The Z range in `PROFILES` (or your `WorkspaceLimits(z=...)`) doesn't match what you're commanding |