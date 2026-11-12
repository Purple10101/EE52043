# EE52043 - Applied Robotics

Project workspace for the EE52043 robot coursework.

## Running the sorter

Two buttons, always in the same order: **CALIBRATE** on an empty plate, then **START** on a full one. Run every
command from the `EE52043` folder.

1. **Plug in** the camera, the gripper board and the Nucleo (USB). Keep a hand on the servo supply's plug
   whenever the arm moves: unplugging it is the emergency stop.
2. **After a power-on, set the arm by hand** with the servo supply off: slide fully up, elbow folded to 100
   degrees or more, gripper turned away from the camera tower. Then switch the supply on (5 V).
3. **Open the window** (about 20 seconds to load the models and open the camera; nothing else may use it):

   ```
   .venv\Scripts\python scripts\sort_resistors_auto.py
   ```

   Add `--unclassified-bin B4` (any bin) to pick resistors whose value couldn't be read, or is outside
   the sorter's 10 ohm to 10 kohm range, into that bin instead of leaving them on the plate. That bin is then never given a value.

4. **With the plate empty, press CALIBRATE.** Three steps, each checked before the next
   (`robot_control/task/auto_app.py`). If one doesn't pass it stops and says why and what to do:
   1. the arm started (the base and elbow sweep end to end) and the elbow checked (105, 110, 115, each within
      1 degree);
   2. the plate found and checked to be empty, the arm parked out of view;
   3. the aim checked: the closed tip down at five points, and any steady shift between the arm and the plate
      (carrying the rig causes one) taken out of the calibration and saved.

   START stays greyed out until CALIBRATE has passed in this window.
5. **Lay the resistors on the plate**, keeping the crosshair in the middle clear, and **press START.** First it
   reads every resistor's value, once: a focus sweep over the parts (about 25 s), then a close-up of each
   (about 1.5 s a part). Keep hands out of view. Then the arm picks, drops and parks, looking at the plate
   again after every pick, until nothing is left that it can sort.
   - **The plate view:** each resistor is outlined: green is the next pick, orange is picked later, grey stays
     on the plate (the label says why: "measure" for one with no trusted value, outside 10 ohm to 10 kohm, ...).
   - **TRAYS:** which value each bin holds and how many have gone in.
   - **WHERE THE BINS ARE:** the rig from above, each slot labelled, and the arm moving as it does.

   Each value keeps its bin until the window is closed. For another batch, lay it out and press START again.
6. **STOP** ends a run once the arm is parked again. Press **q** to close the window once nothing is running.
   CALIBRATE again if the Nucleo loses power or the rig is moved or knocked.

**The manual window**, `.venv\Scripts\python scripts\sort_resistors.py`, has the same picture with every step
on its own button (CALIBRATE ARM, CHECK AIM, LOOK, START, STOP), in any order: for testing and for finding out
what went wrong. `--look-only` opens the camera alone.

Every session (window opened) is logged in `logs/sort_runs/<time>/`:
- `events.csv`: what happened;
- `readings.csv`: every look's reading of every resistor (value, best guess, confidence, colours), the detections
  too faint to count, and the ones missing;
- `crops/`: the band crops the reader saw;
- `frames/`: each look's averaged camera frame and plate fit;
- a picture of the window per look.

When a resistor is misread or missed, that folder is the evidence to send to robot_offline's band-reading or
segmentation session. `experiments/2026-11-11_full_run_8/` (on this PC) is the first run's investigation.

**A value is trusted at 0.7 confidence or more** (`robot_control/config/vision.toml`, `min_confidence`); under it the
resistor is to be measured. It was 0.9 for a while on 2026-11-11, after a 4k7 read as 470R at 0.82 without the
focus check (`experiments/2026-11-11_full_run_8`, section 6); back to 0.7 now that every capture's focus is
checked. Watch for factor-of-ten misreads, and raise it again if they come back.

**Movement** (`scripts/slide_wrist.py`, `scripts/resistor_nav.py`): the slide climbs in one move and goes down in
moves of at most 75 degrees; from the park the arm swings out fully up and glides straight down to the approach;
it parks in one swing when the checks allow it; long swings run at 40 deg/s. Between picks it doesn't park: after a
drop into an A slot the camera looks from there (the arm hides none of the plate), and after a B slot the arm swings
over A2 first (`robot_control/vision/occlusion.py`). During a run, focus is re-checked only every 90 s; the
start-of-run read always checks. Where the time goes, and why some
moves look jerky: `experiments/2026-11-11_run_speed_smoothness/` (on this PC).

**Values are read once per run; resistors are found on every look.** The camera's focus drifts while it is in
use (about 4 settings a minute), and the bands need it sharp, so the band reader runs once, at START, straight
after a focus sweep (robot_offline's handoff `2026-11-11_ee52043_read_once_plan.md`). Segmentation runs every
time the arm is parked, and each resistor is followed by where it lies. A resistor keeps its value while it
stays where it was read (within 3 mm and 15 degrees).

**Focus before every capture** (robot_offline's handoff `2026-11-11_ee52043_focus_before_capture.md`): the lens
drifts as the camera warms, so every capture should follow a focus check on the plate's centre cross. The code
calls robot_offline's `FocusTracker` (`pre/focus.py`, landed 2026-11-11) before each capture. The first check is a
full search (about 16 s on the rig), each later one about 3.5 s. If the cross is covered, a body near the centre
stands in. Close-ups use the check's offset, so START needs no focus sweep of its own. With the check, segmentation
found 8 of 8 kit parts against 5 of 8 at the old fixed focus. Each look's focus state is in the run's `events.csv`.

**"Measure"** marks a resistor with no trusted value, for the ohmmeter once there is one (until then it stays on
the plate). That covers three cases:
- unsure at the read (under 0.7);
- knocked since the read;
- not there at the read: missed by segmentation, or uncovered later.

A resistor seen before but missing from a look gets a dashed "missed?" ring, and the run looks again rather than
end. A pick the jaws missed is followed, and picked again.

**If something is wrong:**

- *The plate isn't found:* clear the crosshair. In the lab a pale bench behind the far edge can hide it; a
  matte black strip along that edge helps. See "Calibrating the system" below.
- *A pick misses, or "3 picks in a row came to nothing":* the arm isn't going where the camera says. Clear
  the plate and CALIBRATE again (CHECK AIM in the manual window). If the aim check fails, do the full
  calibration ("Calibrating the system").
- *CALIBRATE fails:* it says which step and what to do. Elbow check bad: something dragged on the arm; free
  the cables, set the arm by hand again. Plate not empty: clear it. Aim check failed: the full calibration.
- *A pick is counted as sorted but wasn't:* the run judges a pick only by the resistor's spot being empty
  at the next look (the gripper can't feel what it holds). Don't touch the plate during a run.
- *Stopped holding a resistor it couldn't drop:* the arm is left where it is, the jaws still gripping.
  Close the window, open the jaws with OPEN in `.venv\Scripts\python scripts\resistor_nav.py`, take the
  resistor out, then PARK there.
- *No rig picture under the trays:* it is made from the digital twin, which stays on this PC:
  `.venv\Scripts\python scripts\twin\render_top_view.py`. Without it the slots are drawn on a plain background.

The details of each part are under "The sorting run" below.

## Layout

- `robot_control/` - our robot control code, one subpackage per module (`world`, `config`,
  `hardware`, `kinematics`, `gripper`, `motion`, `vision`, `classify`, `planning`, `task`).
- `scara_control/` - the team's control interface: `ScaraController` (real robot over serial or
  the simulator over TCP, with workspace limit checks), kinematics in `scara_workspace.py`, and a
  Tkinter test GUI. From AB3849/Applied-Robotics commit fa7c1a1 (Alex).
- `hardware_api/` - Python APIs for the SCARA, provided by the course:
  - `scara_motion_controller_api.py` - real robot, over serial (needs `pyserial`).
  - `sim_scara_motion_controller_api.py` - Unity simulator, over TCP (127.0.0.1:9000).
- `course_files/` - everything else from the module repo (examples, CAD, simulator, firmware,
  machine vision). See `course_files/README.md`.
- `firmware/` - the motion controller firmware flashed on the arm: the course's Moodle build
  (`servo_motion_controller.zip`, with the DEMO switch), this arm's potentiometer calibration in
  `potentiometers.c`, and smoother, finer control with the same serial protocol. One source, two builds: the
  standard build (`firmware/build/`) and the play build (`firmware/build_play/`, `-DSCARA_PLAY=ON`), which
  approaches every base/elbow target from one side to work round the joints' play. Since 2026-11-12 the host
  must send matching settings (`TIME_SCALE`, `GAINS` in `hardware_api/scara_motion_controller_api.py`).
  **See `firmware/README.md`** for building, flashing, what changed from the course firmware and why, and the
  play build.
- `gripper_firmware/` - the gripper's ESP32-S3 sketch: limits, ramps and calibration for its SG90. Build with
  the Arduino IDE (board XIAO_ESP32S3, signal on D7).
- `scripts/` - bench tools for the real arm: `pot_check.py` and `pot_calibrate.py` (potentiometers,
  servos off), `go_to.py` (one position), `reach_test.py` and `base_reach.py` (what the arm reaches),
  `elbow_servo_hold.py` (holds the elbow servo at an end, for refitting its link), `slide_wrist.py`
  (start, park and bounded single moves), `tip_view.py` (the tip as the camera finds it),
  `camera_calibrate.py` (fits the arm to the camera), `camera_guide.py` (steers the tip with the camera),
  `resistor_nav.py` (click a resistor or a bin and the arm goes to it), `sort_resistors_auto.py` (the sorting
  run, the procedure fixed: CALIBRATE then START), `sort_resistors.py` (the same with every step on its own button).
- `tests/` - tests for `robot_control/` (`python -m pytest`).
- `experiments/` - investigations, kept on this machine and out of git (as in robot_offline): one folder
  each, `YYYY-MM-DD_topic/`, with a `README.md` (goal, outcome, evidence) and the scripts that reproduce it.
  `2026-11-11_pick_error_pipeline/` traces a missed pick through the camera-to-joints pipeline.
- Our printed parts (gripper, camera tower) aren't in this repo: their CAD, print files and design
  notes are in the sibling `Robot Resources` folder (`gripper/`, `camera_tower/`, `non-res-bin/`).

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
camera takes a few seconds to settle and the plate is found about 5 s after that; until then the
plain camera image is shown. It is found as every tool finds it, by robot_offline's `plate_search`
(robot_offline's README, "live_plate"): looked for with the plate dim, then the exposure brought up for
the model and following the light. It is fitted again every 5 s in the background.

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

The `move_coordinates` values are for reading only: nothing here drives the arm, and the slide height
and jaw direction behind them are placeholders. `resistor_nav.py` (below) is what drives it.

## Before driving the real arm

- **Keep a hand on the servo supply.** Nothing in software can stop a move already sent; only cutting
  servo power can.
- **Servos snap.** The board keeps sending a servo's last position even with servo power off, so turning
  servo power on sends every servo there at full speed. If servo power has been off while the board was
  running, keep it off, press the Nucleo's black reset button, then turn it on, and `start` again.
- **Nothing else guards the arm.** The firmware doesn't check ranges or clamp a servo, so an out-of-range
  command drives it past its limits. The limits are in `robot_real.toml` and `scripts/slide_wrist.py`.
- **One program at a time** can hold the COM port. If replies go out of step after a failure, reset the
  Nucleo with servo power off and reconnect.

## Calibrating the system

The sorting window's **CHECK AIM** covers the usual case, a steady shift after the rig has been carried,
in a minute or two. Do the full procedure below when it says more than a shift is wrong, after the gripper
or the camera has been changed, or the first time on a new rig.

Do this whenever the rig has been moved (a new bench, carried home or to the lab), the Nucleo has lost
power, or picks miss by more than about 2 mm. It takes about 15 minutes. It checks three things in turn,
each needing the one before: the joints, the plate as the camera sees it, then the arm against the plate.
Each step says what a good result looks like; stop at the first one that isn't.

**From a cold start** (everything unplugged, e.g. on the demo day), in this order. About 10 minutes if
the check passes, 20 if it has to recalibrate:

1. Set out the rig: plate and arm where they will stay, cables slack, plate clear (the crosshair
   especially). Plug in the camera, the gripper board and the Nucleo, but not the servo supply yet.
2. With the servos limp: raise the slide fully by hand, fold the elbow to 100 degrees or more, turn the
   gripper away from the camera tower. Folded, the gripper passes over the non-resistor bin in `start`'s base
   sweep, and the bin's back wall stands above the plate.
3. Plug in the servo supply (set to 5 V), then run the start sequence under "Resistor navigation window"
   (`start`, `move elbow 110` at once, `park`, `move wrist 0`).
4. Steps 1 to 4 below: the elbow check, `plate`, the tip view, `check`.
5. If `check` is good, the calibration from last time still holds. If not, step 5 below recalibrates.
6. Quit the tip view and open `resistor_nav.py`.

Calibrate after the rig is set out where it will be used, and don't move the plate or the arm base
afterwards: the calibration is the arm against the plate, so moving either undoes it (on 2026-11-09,
carrying the rig to the lab shifted them about 3 mm).

Run every command from the `EE52043` folder, one at a time, with a hand on the servo supply's plug.
Before starting: the gripper board plugged in, the plate clear, and the cables slack (especially the
gripper's lead), as anything dragging on the arm during `start` spoils the joints' calibration.

**1. The joints.** If the Nucleo has lost power, run the start sequence under "Resistor navigation
window" below. Either way, then try the elbow from park:

```
.venv\Scripts\python scripts\slide_wrist.py move elbow 105
.venv\Scripts\python scripts\slide_wrist.py move elbow 115
.venv\Scripts\python scripts\slide_wrist.py move elbow 110
```

Good: each lands within 1 degree of its target (it prints where it landed). If one lands further off
(on 2026-11-09 the elbow landed 4 degrees past 110 every time), `start` learned the servo wrongly, usually
because something dragged on the arm during its sweep. Free the cables and redo it with the Nucleo still
running: `move slide 0`, then `start`, `move elbow 110`, `park`, `move wrist 0`, and try the elbow again.

**2. The plate.** With the arm parked and nothing else using the camera:

```
.venv\Scripts\python scripts\camera_calibrate.py plate
```

Good: "Found at exposure ...", edges within 1 px rms, the near side found by the crosshair, and the
camera near (-145, -145, 183) mm. It saves the fit to `logs/tips/plate_<time>_rectify.json`, which the
next steps use. The plate is found by robot_offline's `plate_search.find_now`, as every tool here and
in robot_offline finds it: with the plate dim, where it was last found or else where it sits on the rig
(robot_offline's `configs/camera.toml [plate_search]`).

Each edge of the plate's top is found as the white plate against something darker. A pale bench behind
one edge hides it (the lab bench behind the far edge, the one away from the camera, is as bright as the
plate). The fit then does without that edge and uses the crosshair in its place, so the crosshair must
be clear; `edges` then shows that side as `(0.0, None)` and `missing_edge` names it. On a lab frame
with the edge hidden, it put the plate within 0.16 mm on average (0.38 at worst) of the fit with the
edge showing. Two things it needs:

- **A start near the plate.** With an edge missing, the plate's outline can't be found from nothing,
  so the search starts where any tool last found it, or else at the rig's place in the frame. The rig
  carries the camera and the plate together, so that holds even after the rig is moved.
- **The crosshair clear.** A lead or body over it, with an edge hidden, leaves nothing to fit.

If the plate still isn't found, lay a matte black strip of card or tape along the hidden edge, touching
it but not over it, and run `plate` again.

**3. The camera on the tip.** In a second terminal, start the tip view and leave it running (`plate`
prints this line with the file's name filled in):

```
.venv\Scripts\python scripts\tip_view.py --snapshot --lock --calibration logs\tips\plate_<time>_rectify.json
```

It holds the camera and finds the tip by the closed jaws' V. The steps below close the jaws themselves.

**4. Check the arm against the plate.** Clear the middle of the plate (the tip comes down almost onto
it, within 25 mm of the crosshair), then:

```
.venv\Scripts\python scripts\camera_calibrate.py check
```

It goes to 5 points with no correction, reads where the tip landed, and parks. Good: about 1.5 mm rms
and worst 2.5 or less; then the calibration is fine, so skip to 6. It also prints "calibration alone":
the tip seen against where the potentiometers say the arm is. If that is good but the misses are large,
the joints aren't landing (redo 1). If "calibration alone" is above about 1.5 mm, go on to 5.

**5. Recalibrate.** Clear the whole plate, then:

```
.venv\Scripts\python scripts\camera_calibrate.py run
.venv\Scripts\python scripts\camera_calibrate.py fit logs\camera_calibration_<time>.csv --save
.venv\Scripts\python scripts\camera_calibrate.py check
```

`run` visits 12 points round the plate and records where the camera saw the tip at each, against the
potentiometers (a point it can't see is skipped). `fit` prints how well the joints' fit matches them, and
how well each point is predicted when left out. Good: about 1.5 mm rms, worst under 2.5 for both. `--save`
writes it to `robot_control/config/camera_calibration.json`, which everything that plans a move uses.
Then `check` again, as in 4.

**6. Finish.** Press `q` in the tip view to free the camera. `resistor_nav.py` finds the plate again
itself, starting from the fit saved in step 2.

## Resistor navigation window

Click a resistor or a bin and the arm goes there. **This drives the real arm:** keep a hand on the
servo supply's plug, as unplugging it is the emergency stop. The gripper board is found on USB; without it
everything but PICK, OPEN and CLOSE still works.

**To run it,** open a terminal in the `EE52043` folder and enter:

```
.venv\Scripts\python scripts\resistor_nav.py
```

That's all you need unless the Nucleo has lost power (see below). Nothing else may be using the camera.

It takes about 20 seconds to load, then finds the plate from the live camera, showing the plain camera
image until it has. If the arm is over the plate, press PARK so the camera can see it. The fit is held
once found, so check that the red outline sits on the plate's edges. If it doesn't, or the plate is
never found, park and press `c` to find it again from scratch. Keep hands off the plate while it looks.

**If the Nucleo has lost power since the arm last ran,** start the arm before opening the window. The
firmware keeps its calibration only in memory and still accepts moves without it, and the scripts can't
tell. An uncalibrated arm lands many degrees off. Run these with the plate clear and the gripper clear
of the tower:

```
.venv\Scripts\python scripts\slide_wrist.py start             # base and elbow jump to zero, then sweep end to end
.venv\Scripts\python scripts\slide_wrist.py move elbow 110    # at once: the sweep ends inside the tower's keep-out
.venv\Scripts\python scripts\slide_wrist.py park
.venv\Scripts\python scripts\slide_wrist.py move wrist 0
```

After a power-off the slide and wrist are limp, so raise the slide fully by hand before `start`. Their
first moves snap them to 0 at full speed. To redo `start` with the Nucleo still running, bring the slide
to 0 first so its snap is nothing. From the park that means out over the plate first, as the slide may
only go up over the non-resistor bin: `move base -45`, `move slide -20`, `move slide 0`.

**The park and the non-resistor bin.** The bin for parts pushed off the plate stands along the plate's +X edge,
under the park pose, and its back wall rises 6 mm above the plate (CAD: `Robot Resources/non-res-bin`).
So the arm parks with the slide fully up (`PARK_SLIDE`, -50), and `park` raises it before anything else
moves. With the wrist out past that edge the slide only goes up, and no base or elbow move crosses the
bin unless the slide is fully up; `slide_wrist.py` refuses anything else, for every tool. Leaving the
park, the arm swings out over the plate first and lowers there.

**Controls:**

| Control | Action |
|---|---|
| Click a resistor | Choose it. Resistors are only detected while the arm is parked, so PARK to see the plate as it is now. |
| PLAY or `p` | A look at the aim: jaws open, one smooth move, the jaws' centre over the chosen resistor, about 9 mm up, jaws across it. |
| PICK | The same, then lowers, grips the resistor and lifts. **It must then go to a bin:** every other command is refused until one drops it. |
| PARK or `k` | Back out of the camera's way. |
| A1-A4, B1-B4 | Over that bin, round the tower if needed, with the jaws turned so what they hold lies along the slot, then the gripper lowers. It stays up if the arm missed by more than 2 degrees. Every move away from a bin lifts the gripper first. |
| `[` / `]` | Turn the wrist 2 degrees, once the arm has left park. |
| `m` | Record that the jaws lie along the chosen yellow line, to `logs/jaw_marks.csv`. |
| OPEN / CLOSE | Work the jaws. |
| `c` | Find the plate again from scratch. PARK first, so the arm isn't in the way. |
| `q` / `Esc` | Quit. The arm stays where it is, so PARK first if you're finishing. It works while holding, as the way out, and leaves the jaws gripping. |

The yellow line across each resistor is the line the jaws close along. The status line at the top shows
how far the joints landed from their targets: more than about 1 degree means something is wrong. A
target past the arm's limits gives "CAN'T REACH" and nothing moves. Every move is logged to `logs/slide_wrist_<date>.csv`.

Bins use the angles taught at each slot on 2026-11-02 (`robot_control/config/bins_real.toml`), before
the current gripper and firmware, so they may be a little off.

## The sorting run

Sorts the plate into the trays on its own, until nothing is left that it can sort. One window does it
all, from power-on to the end of the run. Not yet run on the arm.

```
.venv\Scripts\python scripts\sort_resistors.py                 # the arm, the gripper and the camera
.venv\Scripts\python scripts\sort_resistors.py --look-only     # the camera alone: LOOK only, the arm never moves
```

Plug in the camera, the gripper board and the Nucleo first, and keep a hand on the servo supply's plug
whenever the arm moves. The window shows what it is doing, when the arm was last started, the plate at the
last look (green: the next pick, orange: picked later, grey: staying, each labelled with why), and the
**TRAYS**: which value each bin holds and how many have gone in. The bin the arm is heading for has an
orange border. All eight slots take values.
Under the trays, **WHERE THE BINS ARE**: the rig from above with each slot labelled with its id and
value, so a bin on screen can be found on the bench. The picture is the digital twin's, with tray A
turned 90 degrees as on the rig (`scripts/twin/render_top_view.py` writes `robot_control/config/rig_map.png`;
both stay out of git with the twin). Without it the slots are drawn on a plain background.
The arm is drawn on the map as it moves: its links, the jaws opening and closing, a resistor between
them while it holds one, an orange ring where a move is heading, and the slide's height on a gauge.
`robot_control/hardware/arm_monitor.py` (`ArmMonitor`) makes that possible: every command to the
Nucleo and the gripper passes through it unchanged, and it works out where each joint is part way
through a move, so the window can ask as often as it draws without any extra serial traffic.

| Button | What it does |
|---|---|
| CALIBRATE ARM | After the Nucleo has been powered on: the start sequence (`start`, elbow to 110, `park`, wrist 0) and the elbow check (105, 115, 110, each within 1 degree). Press it once to see what to set by hand first (supply off: slide fully up, elbow folded, gripper away from the tower, plate clear), then again within 20 s to go. A bad elbow check says so: free the cables and calibrate again |
| CHECK AIM | Does the arm still go where the camera says? The closed tip put down at 5 points, found by the camera; a steady shift is taken out of `camera_calibration.json` and saved (the old file kept in `logs/`), and picks aim by it at once. At the start of a session and after the rig has moved. `robot_control/task/aim_check.py` |
| LOOK | One look and its plan; nothing moves. Do this before the first START |
| START | The sorting run. Pressing it again after a stop carries on with the same bins |
| STOP | Ends the run once the arm is parked again, never mid-move or holding a resistor |
| q / Esc | Quit, once nothing is running |

The program can't tell whether the Nucleo has lost power since the arm was last started (the time is
shown): if it has, press CALIBRATE ARM before anything else. The arm's fit to the camera
(`camera_calibrate.py`, "Calibrating the system" above) is separate, and only needed after the rig moves.

**The cycle** (`robot_control/task/supervisor.py`, the Supervisor): look with the arm parked, judge the last
pick, plan, pick, drop, park, and again. Looking again after every pick means nothing is tracked between
picks: a knocked resistor is just somewhere new, and a missed pick shows as a resistor still where the
jaws went.

**The parts**, one class each, so each can be read on its own:

| Class | File | Its job |
|---|---|---|
| `Eyes` | `vision/eyes.py` | `look()`, every time the arm is parked: the plate found, the mean of 8 fresh frames, the resistors segmented and followed (each with the value it was read as). `read_values()`, once at START: a focus sweep over the parts (robot_offline's `closeup.calibrate`, the rule kept for this read only), then each part read from the averaged picture and from its own close-up, combined (averaged where they agree, the close-up where they don't). The scene must be still: the arm parked, hands clear |
| `BodyMemory` | `vision/body_memory.py` | Follows each resistor from look to look by where it lies (as robot_offline's `Steady`): a known one still counts when its score dips, one missing from a look is waited for, a picked one that's still there is the same resistor. Keeps each one's start-of-run reading; it holds while the resistor stays within 3 mm and 15 degrees of where it was read, else it is to be measured |
| `Resistor`, `PlateView` | `world/types.py` | What a look returns |
| `BinPlan` | `planning/bins.py` | Which bin each value goes in: the first time a value is sorted it takes the next free slot (A1, A2, ... in `bins_real.toml`'s order) and keeps it for the run. All eight slots take values; a value outside 10 ohm to 10 kohm has none |
| `PickMemory` | `planning/memory.py` | The gripper can't feel a resistor, so the next look judges a pick: its spot empty means sorted, a body still there means missed. Two misses, or a pick the arm refused, and that resistor is left |
| `Planner` | `planning/planner.py` | Which resistor next (the most isolated), and what becomes of every other one |
| `Arm` | `motion/arm.py` | Park, pick, drop, each saying whether it worked: resistor_nav's moves, with every check in `slide_wrist.py` |
| `RunWindow` | `task/window.py` | The window: status, plate, trays and buttons. It only draws; the run tells it what to show |
| `SortingApp` | `task/app.py` | The buttons, each running one job (calibrate, look, run) in the background so the window stays alive |
| `RunLog` | `task/run_log.py` | `logs/sort_runs/<time>/`: events.csv and a picture per look |

**What stays on the plate:**
- a resistor to be measured: unsure at the read (under 0.7), moved since, or not there at the read;
- one read outside 10 ohm to 10 kohm;
- one the arm missed twice or couldn't reach;
- one whose value has no bin left once all 8 are taken.

**When it stops:** nothing left to sort; 3 picks in a row that came to nothing (check the calibration); a
resistor it couldn't drop (the arm is left holding it: a person must look); the plate not found; q.

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
