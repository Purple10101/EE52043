# Motion controller firmware

The STM32F446 (Nucleo) firmware that drives the arm's four servos and reads the base and elbow
potentiometers. It started as the course's Moodle build (`servo_motion_controller.zip`) and keeps its
48-byte serial protocol, so the course API and every script still talk to it.

One source tree builds two firmwares:

| Build | Directory | For | What it does at a target |
|---|---|---|---|
| **Standard** | `firmware/build/` | joints with little or no play | Comes onto the target from whichever side the move arrives, then settles both ways |
| **Play** | `firmware/build_play/` | joints with play (now: base ~4.5 deg, elbow ~1.1 deg) | Always comes onto a base/elbow target moving upward in angle, so the servo ends pressed against the same side of the play every time |

The only difference is the default of one setting (`PLAY_APPROACH_DEFAULT`). Either build can be switched to
the other's behaviour at run time with `SCARA_SET_PLAY`, without reflashing.

Everything below marked *model* was measured in host-PC simulations of the firmware (servo lag and deadband,
joint play, potentiometer noise), not on the arm. Check it on the rig before relying on it.

---

## Contents

1. [Building and flashing](#1-building-and-flashing)
2. [What the host must send](#2-what-the-host-must-send)
3. [How a move works](#3-how-a-move-works)
4. [What changed from the course firmware, and why](#4-what-changed-from-the-course-firmware-and-why)
5. [The play build](#5-the-play-build)
6. [Serial protocol](#6-serial-protocol)
7. [Settings](#7-settings)
8. [Not done](#8-not-done)
9. [Testing](#9-testing)
10. [First run on the rig](#10-first-run-on-the-rig)

---

## 1. Building and flashing

Needs CMake, Ninja (in `.venv/Scripts`) and the Arm GNU Toolchain (14.2). From the repo root:

```
cmake -S firmware -B firmware/build -G Ninja                      # standard, once
cmake --build firmware/build

cmake -S firmware -B firmware/build_play -G Ninja -DSCARA_PLAY=ON  # play, once
cmake --build firmware/build_play
```

Each build writes `servo_motion_controller.{elf,bin,hex}` into its own directory. Both use the CubeIDE Debug
settings (`-O0 -g3`), and the CubeIDE project files still open the standard build.

**Flash** by copying the `.bin` onto the Nucleo's USB drive (`D:`). The board resets itself when the copy finishes.
After a flash the calibration is gone, as after any power-off: run `slide_wrist.py start` (it calibrates) before
moving the arm.

**Roll back** the same way, by copying an older image:

| Image | What it is |
|---|---|
| `logs/firmware_backup_ead8085_20261111_215202.bin` | commit ead8085 (0.1 us PWM, averaged pots, proportional settle), the build on the arm before 2026-11-12 |
| `logs/firmware_smooth_start_20261111_215530.bin` | ead8085 plus the first smooth start, flashed once on 2026-11-11 |
| `logs/firmware_course_before_new_loop.bin` | the course build with this arm's pot calibration |

A pre-fix image needs the host settings in section 2 put back (`TIME_SCALE`, `GAINS`).

---

## 2. What the host must send

Two settings in `hardware_api/scara_motion_controller_api.py` go with this firmware:

| Setting | This firmware | A pre-fix build (ead8085 or older) | Why |
|---|---|---|---|
| `TIME_SCALE` | `2.0` | `1.0` | Every move time is multiplied by it. The old firmware ran a move of time T in 2T (section 4.5) and every time in the scripts was tuned against that. 2.0 keeps the arm moving exactly as fast as before; 1.0 makes it twice as fast. |
| `GAINS` (move-phase P, I, D for base, then elbow) | all `0` | `COURSE_GAINS` (1.3, 0.01, 0.001) | The course PID rattles the joints through their play (section 4.4). This firmware corrects with its start offset and settling instead. Don't send 0 to a pre-fix build: its moves start with a kick that the PID partly hides. |

`scripts/slide_wrist.py start` and `scripts/go_to.py` send `GAINS`. The older test scripts (`reach_test.py`,
`base_reach.py`) still send the course gains; that is safe but brings the rattle back while they run.

To speed the arm up later, lower `TIME_SCALE` toward 1.0 one step at a time and check the bins are still
reached. The firmware stops anything faster than the servos can follow (section 4.6).

---

## 3. How a move works

Each command is read whole (48 bytes), carried out, and answered with a 4-byte flag. The host waits for the
flag, so one command runs at a time.

**A base/elbow move** (`SCARA_MOVE_JOINT` axis 0/1, or `SCARA_MOVE_JOINTS`):

1. **Start.** The move starts from where the potentiometer says the joint is, and from the pulse already
   being sent. The difference between the pulse that angle maps to and the pulse being sent (the
   *start offset*: map error, play, the last settle's trim) is subtracted at first, so the first pulse is the one
   already on the pin: no step. If the joint is already within 0.15 deg of the target, the move starts at the
   target instead, so repeated small corrections don't add up.
2. **Profile.** A double-S profile (smooth acceleration and jerk) takes the joint to the target in time T,
   one step per PWM frame (3 ms), in real time. The start offset fades out as the joint travels: all of it over
   a move of 20 deg or more, and in part over a shorter one, which stays near where it was needed. What a move
   keeps at its end is limited to 120 us.
3. **Speed limit.** T is lengthened if the profile would need more than 150 deg/s or 1000 deg/s^2.
4. **Move-phase PID** on the potentiometer, if the host sends gains (it shouldn't: section 2).
5. **Settle.** Every 20 ms the pulse moves in proportion to the remaining error (1.5 us per deg, at most 3 us a
   step), until the joint has stayed within 0.15 deg for 5 steps, or `settling_time` runs out. The pulse never
   strays more than 200 us from the one the target maps to.

**Wrist and slide** have no sensor: their moves follow the same profile, open loop, with no settling.

`SCARA_AUTO_CALIBRATE` sweeps the base and elbow end to end and learns where the pulses for -90, 0 and +90 deg
(base) and 0, 90, 120 deg (elbow) are. It ramps to the start of the sweep instead of jumping.

---

## 4. What changed from the course firmware, and why

The course firmware was reviewed issue by issue (2026-11-12), each issue checked in a model of the firmware.
In the course of three versions (ead8085, the smooth start, and this one) these were fixed:

### 4.1 Pulse resolution and rounding (ead8085)

The course's PWM timer counted 3 us per tick (0.26 deg on the base) and always rounded down, and its settling
asked for +-5 us but got +3 / -6 us. TIM3 now counts 0.1 us per tick (prescaler 9, period 30000, still 333 Hz)
and pulses are rounded.

### 4.2 Potentiometer noise (ead8085)

The course read one raw ADC sample, so each reading carried a few counts of noise straight into the servo
pulse (*model*: the pulse jumped about 45 us peak to peak, about 4 deg, while the arm held still). Each
reading is now the mean of the 512 samples taken over one PWM period, which also cancels ripple at the
PWM rate. The ADC's DMA interrupt now fires every 1.5 ms instead of every 5.5 us.

### 4.3 Settling (ead8085, made absolute here)

The course nudged the pulse 5 us every 50 ms while the error was over 0.5 deg, always for the full settling
time, also for the wrist and slide, with no limit. Now it is proportional, stops once on target, settles only
the base and elbow, and is limited. Since this version the limit is measured from the pulse the target maps to,
not from where the move ended: before, a joint held short of its target (by the plate, a stop, the gripper)
walked its pulse a little further every move and then started each move with a step (*model*: 11 deg).

### 4.4 The start of a move (smooth start, and fixes here)

The course started each move from the last commanded angle, which included the PID's last correction and not
the settling trim, plus a deliberate "half adjustment". So almost every move began with a jump of several degrees
in one PWM frame (*model*: up to 6 deg on the base, and 2-4 deg on the base during a slide-only or wrist-only
`MOVE_JOINTS`). Its half adjustment also read past the end of an array (`tpos[AXES]`), fixed in ead8085, and
compared with the wrong sign.

The smooth start (start from the pot, carry the start offset) removed the jump. Fixed here, from the review:

- **No pulse yet.** After a power-up the pulse register is 0, so the offset came out as a whole pulse,
  limited to 300 us: the first move kicked the base about 8 deg and the elbow about 11 deg the wrong way. Now
  there is nothing to carry until a pulse has been sent.
- **The limit made a step.** The offset was clipped at 300 us before the move, so a bigger one jumped by the
  excess. Now the start is never clipped; only what the move keeps at its end is (120 us).
- **Creep on small corrections.** With the offset kept on tiny moves, each move added a little to the pulse
  that the servo's deadband and the play swallowed, so a run of small corrective moves walked the servo across
  the play and then shoved the link (*model*: +0.9 us per move, about 55 moves to cross 4.5 deg). This fits the
  shaking at rest seen on 2026-11-11, though it isn't proven to be the cause. Fixed by starting at the target when
  already inside the settle band, and by the PID ignoring errors inside that band.

### 4.5 Timing

`HAL_Delay(1)` waits at least one whole tick, so it took 2 ms: every move took twice its time T, and 2 ms
updates against 3 ms PWM frames made the speed uneven from frame to frame. The moves now wait for the timer's
update flag (`servo_wait_frame`), so each step is exactly one PWM frame and the last lands on t = T. The host
doubles its times to keep today's speed (section 2).

### 4.6 Speed limit

Nothing stopped a short T over a long distance, which outruns the servo, overshoots and lands off.
`min_move_time` lengthens T to keep within 150 deg/s and 1000 deg/s^2 (guessed; not measured on these servos).

### 4.7 The move-phase PID

The course runs P 1.3 on the potentiometer during every move, around a servo that already holds its own
position. With play between the servo and the link, and the link's own mass on the far side, this locks into a
rattle after a knock (*model*: in every build). The code is kept, but the host now sends 0. The start offset and
settling take care of the map error the PID was there for.

### 4.8 Calibration start

The sweep jumped straight to the 1500 us start pulse. It now ramps there at 5 us per 5 ms, from where the pot
says the joint is if no pulse has been sent yet.

---

## 5. The play build

### 5.1 The problem

The base and elbow servos turn the links through some play (horn on spline, horn on link): about 4.5 deg at the
base and 1.1 deg at the elbow, measured 2026-11-12. The joints are horizontal, so nothing takes the play up:
where the link stops depends on which way the servo was last moving, and anything that touches the link (cable
drag, the gripper closing, a hand) can move it within the play.

The potentiometers are on the links, so the firmware sees where the link is. But the servo pushes the link only
after crossing the gap, so a correction in the other direction first moves the servo across the play with the
link standing still, then moves the link with a jolt. Settling that corrects both ways hunts across the play.

### 5.2 What the play build does

**It comes onto every base/elbow target from the same side** (machine tools call this unidirectional
positioning). With the default direction +1, the last motion onto a target is always upward in angle. The servo
then always ends pressed against the same side of the play, so the link's position for a given target repeats.

For each base/elbow move:

1. **Does the joint have to turn round?** If the move would end going downward (more than the 0.15 deg band),
   yes. If it isn't known which side of the play the servo is on (after power-up, a calibration, or a
   `SCARA_SET_PLAY`), it must also travel upward at least the play plus a 1 deg margin, or it turns round.
2. **Turn round.** The joint first goes, in the move's time T, to the target less the play and the margin
   (base: 5.5 deg below, elbow: 2.1 deg below), then comes up onto the target over 0.4 s. In
   `SCARA_MOVE_JOINTS` the wrist and slide go straight to their targets during the first part. A joint that
   doesn't have to turn round is already at its target for the second part, which is then only settling.
3. **One-way settling.** Settling only pushes upward, and while the joint is short it pushes at least 2 us a step
   (20 ms). The coming-back move starts from where the pot says the link is, which is still on the near side of the
   play, so the servo may still have the play to cross; the proportional step alone is tiny when the link is only
   a little short and would time out (*model*: 0.5-0.7 deg short); 2 us a step crosses 4.5 deg of play in about
   0.5 s. A joint that has gone past its target can't be brought back down without crossing the play, so it is
   left alone (not reported to the host).
4. **One retry.** A joint that ended past its target turns round once more (down by the play and margin, then up
   over 0.4 s).

The turning point is kept inside base -90..67.5 deg and elbow 0..139 deg, so it never drives a joint into its
stop. Approaching upward means the turning point is always below the target: at the elbow, away from its stop
at 141 deg. If a target is so close to a limit that the turn is cut short, the firmware no longer assumes the
servo has crossed the play, so the next move to it turns round again.

### 5.3 Cost

- **Time.** A move that ends going down takes an extra 0.4 s (plus settling), and a retry another 0.8 s. Moves
  that end going up cost nothing extra.
- **Small corrections downward.** Any correction downward, however small, turns round: a 0.3 deg nudge down
  makes the base dip 5.5 deg and come back (*model*: about 2 deg of link travel and 1.9 s). A correction loop
  (such as `land()`) should aim to undershoot, so its corrections go upward.
- **Path.** The joint dips up to 5.5 deg (base) and 2.1 deg (elbow) below the target before coming up: about
  20 mm at the tip at full reach. **The host's keep-out checks (`swing_refusal`, `passes_bin`) don't know about
  this dip.** Watch it near the camera tower beside the B bins, and near bin walls with the slide down.
- **A blocked joint.** A joint held short of its target (pushing upward) settles as in the standard build, up to the
  200 us limit. One held *above* its target can't be pulled down by settling; the retry turns it round once, and
  after that it is left where it is.

### 5.4 Tuning

Set at run time with `SCARA_SET_PLAY` (`hardware_api`), or in `Core/Inc/servo_controller.h`:

- **The play.** Measure it with `scripts/pot_check.py` (read only) while pushing the link both ways under hold,
  after any mechanical change. Too small a value and the joint may not cross the play before coming back up, so
  it lands on the wrong side; too big and it only costs time and room. The 1 deg margin covers a little error.
- **The direction.** +1 (from below) keeps the turning point away from the elbow's stop at 141 deg and inside the
  base's travel, which ends at about +69.5 deg. Use -1 only if most of the work is approached from above.
- **Off.** Direction 0 makes the joint behave exactly as in the standard build.

### 5.5 Which build to flash

- Flash the **play build** while the base has degrees of play. Its repeatability should be far better than the
  standard build's (section 9), at the cost of a short extra move on some targets.
- Flash the **standard build** once the play is fixed mechanically (tight horns, or a light spring across each
  joint so the link always rests against one side). Then turning round only costs time.
- To compare them, use one build and switch with `SCARA_SET_PLAY` (direction 0 or +1) between runs.

A firmware can only hide play at the end of a move. The real fix is mechanical: preload (a spring across the
joint) or no play.

---

## 6. Serial protocol

115200 baud, little-endian. Every command is 48 bytes: an int32 id and 11 four-byte values. Every command
gets an int32 flag (1 = done) after it has finished; reads then send their data.

| Id | Command | Values | Reply |
|---|---|---|---|
| 1 | `SCARA_INITIALISE` | link_1, link_2, z_min, z_max, settling_time, P0, I0, D0, P1, I1, D1 (floats) | 1 |
| 2 | `SCARA_AUTO_CALIBRATE` | none | 1 |
| 3 | `SCARA_MOVE_JOINT` | axis (int32), angle, time | 1 |
| 4 | `SCARA_MOVE_JOINTS` | base, elbow, wrist, slide angles, time | 1 |
| 5 | `SCARA_MOVE_COORD` | x, y, wrist angle, z, time | 1 |
| 6 | `SCARA_READ_ANGLE` | axis (int32, 0 or 1 only) | 1, then the angle (float) |
| 7 | `SCARA_READ_COORD` | none | 1, then x, y, wrist angle, z (floats) |
| 8 | `SCARA_SET_PLAY` (new) | axis (int32, 0 or 1), play_deg (float, 0-20), direction (int32: +1, -1, 0) | 1, or 0 if refused (nothing changed) |

Times are in seconds, real time (section 2 for what the host sends). Command 8 is not in the course firmware,
which never answers it. An unknown id gets no reply.

---

## 7. Settings

All in `Core/Inc/`. Changing any needs a rebuild.

| Setting | Value | Where | Meaning |
|---|---|---|---|
| `PWM_TICKS_PER_US` | 10 | servo.h | TIM3 at 0.1 us per tick |
| `SERVO_FRAME_S` | 0.003 | servo.h | one PWM period; one move step |
| `MOVE_MAX_SPEED_DPS` | 150 | servo_controller.h | base/elbow speed limit (guessed) |
| `MOVE_MAX_ACCEL_DPS2` | 1000 | servo_controller.h | base/elbow acceleration limit (guessed) |
| `SETTLE_BAND_DEG` | 0.15 | settle.h | on target within this; the PID and the start ignore smaller errors |
| `SETTLE_GAIN_US_PER_DEG` | 1.5 | settle.h | settling, per 20 ms step |
| `SETTLE_MAX_STEP_US` | 3 | settle.h | settling's biggest step |
| `SETTLE_LIMIT_US` | 200 | settle.h | furthest the pulse may stray from the target's mapped pulse |
| `SETTLE_CALM_TICKS` | 5 | settle.h | steps in the band before settling stops |
| `CARRY_FADE_DEG` | 20 | settle.h | a move this long drops all of its start offset |
| `CARRY_LIMIT_US` | 120 | settle.h | the most a move keeps of its start offset at its end |
| `PLAY_APPROACH_DEFAULT` | 0 standard, 1 play | servo_controller.h / CMake | approach direction at power-up |
| `PLAY_BASE_DEG`, `PLAY_ELBOW_DEG` | 4.5, 1.1 | servo_controller.h | play at power-up |
| `APPROACH_MARGIN_DEG` | 1.0 | servo_controller.h | turned round this far past the play |
| `APPROACH_S` | 0.4 | servo_controller.h | the move back up onto the target |
| `APPROACH_RETRIES` | 1 | servo_controller.h | turn-rounds for a joint that ended past its target |
| `SETTLE_MIN_DIR_STEP_US` | 2.0 | servo_controller.h | one-way settling's smallest step while the joint is short |
| `APPROACH_*_MIN/MAX_DEG` | base -90..67.5, elbow 0..139 | servo_controller.h | limits for the turning point |

The settling time and PID gains come from the host in `SCARA_INITIALISE`. This arm's potentiometer calibration
is in `Core/Src/potentiometers.c` (redo it with `scripts/pot_calibrate.py` after a pot or coupling is disturbed).

---

## 8. Not done

These were found in the review and left out on purpose (2026-11-12). The host is relied on for them:

- **No joint limits** in the firmware. The host clips its targets (`robot_control/config/robot_real.toml`).
- **Unreachable `SCARA_MOVE_COORD`** gives NaN angles, which send the base and elbow PWM to 0. The host
  never sends `MOVE_COORD`; it does its own inverse kinematics.
- **No check that a potentiometer is believable.** A stuck or disconnected pot makes every base/elbow move start
  from the wrong angle, and with the smooth start that can swing the joint across its range. If the arm moves
  strangely, check the pots with `scripts/pot_check.py` before anything else.
- **The calibration lives in RAM.** After a power-off or reset the firmware falls back to uncalibrated values and
  says nothing. Run `slide_wrist.py start` after any power-off.
- **No serial resynchronisation.** If a byte is lost, every later command is misread until a RESET. Unknown
  ids get no reply.
- **The calibration map** is three points. The base sweep never reaches +90 deg (its travel ends at about +69.5), so
  the + side keeps the default slope and is about 8 deg short at +60; settling makes it up, at about 0.5 s per
  base move.

---

## 9. Testing

`test/settle_test.c` runs the settling and start-offset maths on a PC against a servo model with a deadband,
backlash and a noisy pot:

```
cd firmware/test
gcc -I../Core/Inc settle_test.c ../Core/Src/settle.c -lm -o settle_test && ./settle_test
```

It also checks that a trim at or beyond `SETTLE_LIMIT_US` is never stepped back by the excess in one go.

**Model results, 2026-11-12.** Two independent host harnesses compiled these sources against a stubbed HAL and
a model of the arm: servo with lag, slew limit and a 2-6 us deadband, the firmware's map off by about 2 deg,
play 4.5 deg (base) / 1.1 deg (elbow) between servo and link, optionally the link's own mass behind the play,
pot noise of 3 counts plus PWM-rate ripple, and the host's gains (0) and doubled times. *These are a model, not
the arm.* The last fixes (one-way minimum step, the settle limit and the cut-short turn) were tested as the same
changes on a copy just before them.

| | Course build | ead8085 | Smooth start (iteration 2) | Standard (this) | Play (this) |
|---|---|---|---|---|---|
| Biggest pulse step at the start of a move | up to 72 us (6 deg) | up to 70 us | 0 | 0 | 0 |
| Teach session (many small corrections), worst shake base / elbow | 16.9 / 9.3 deg | 1.6 / 5.4 deg | 0.5 / 0.3 deg | as iteration 2 | as iteration 2 |
| 6 deg knock with link mass, host's old P 1.3 | rattle, 52-99 servo reversals | rattle | rattle | (P = 0 now) 1-2 reversals, no rattle | as standard |
| Same target repeated 20 times at +60 | - | 79 deg of link travel | 0.12 deg | 0.00 deg | 0.00 deg |
| A move of host time T takes | 2T | 2T | 2T | exactly T x `TIME_SCALE` | as standard |
| Frame-to-frame pulse jitter in a move | - | - | 0.26-0.51 us | 0.06-0.08 us | as standard |
| Same target from above vs from below: gap / spread (far approaches) | - | - | 0.25 / 0.28 deg | 0.25 / 0.28 deg | 0.00 / 0.02-0.2 deg |
| Same, approached from 0.3-1 deg away: spread / landings over 0.35 deg off | - | - | 1.23 deg / 7 of 40 | 1.23 deg / 11 of 40 | 0.10 deg / 0 of 40 |
| Seven-move sequence | 20.1 s | 16.7 s | 17.1 s | 17.1 s | 19.7 s |

With no play in the model the standard build is already repeatable and the play build only costs time (the
sequence 17.7 s against 15.6 s) and the dip below turned targets.

`settling_time` matters: with 0 the standard build lands 1.6-4.8 deg off (the calibration map's error); keep it at
1.0 (`slide_wrist.py` `SETTLE_S`). `robot_control/task/twin_pick.py` sends 0.2 for the Unity twin.

---

## 10. First run on the rig

1. Flash the build (section 1). The host settings in section 2 must match it.
2. Servo power on, hand on the supply. `slide_wrist.py start`: the base and elbow sweep, then park. Watch the
   sweep start: it should ramp, not jump.
3. Move to a few bins and the plate centre with the usual tools. The arm should move as fast as before
   (`TIME_SCALE` 2.0) and land as before or better.
4. Teach-style small corrections: the arm must not shake at rest.
5. Play build only: approach the same target from above and from below several times and compare where the
   jaws land; watch the dip before targets approached from above.
