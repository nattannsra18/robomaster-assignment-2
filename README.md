# RoboMaster Assignment 2

Clean, focused extraction of the Round 1 ToF + camera maze mission from
[`robomaster-autonomous-maze-navigation`](https://github.com/nattannsra18/robomaster-autonomous-maze-navigation/tree/refactor/v05-basic-motion-91fa792)
at source commit `6f8b5d92bb2abcf93b6b2cde6e74d5baae2139f4`.

The Round-1 entrypoint is `final_round1_tof_camera_01.py`. It explores an
unknown fixed-cell maze, builds occupancy/topology outputs, surveys coloured
shape targets, and provides the V05 operator GUI. Real RoboMaster blaster
control is available behind an explicit target allow-list and arm switch. The
Assignment firing mode defaults to `selected`; the explicit arm control still
prevents a shot until the operator enables firing.

Stable V1 movement adds a fresh-ToF preflight before every cell, live
travel-direction ToF braking, an unconditional hard stop, and a recoverable
Moving Gimbal feedback hold. Logical cell arrival is still decided by
odometry. A destination-wall cue may confirm arrival only after at least 75%
cell progress and while cross-track remains within the configured cell-center
tolerance. The GUI checkbox
`Moving Gimbal Check (diagnostic)` defaults to ON and disables only the
in-motion Gimbal angle/age check when switched off. Mission completion now
requires all 36 logical cells in the assignment's exact 6x6 grid; perimeter
wall readings are retained as diagnostics and do not hold a completed map open.

Transient feedback is handled without ending the whole round: a failed Gimbal
scan records that direction as `UNKNOWN`, camera/detector failure skips only
that target survey, zero-motion preflight failures retry from the same cell,
and stale movement feedback gets a stopped re-aim plus fresh preflight. A
preflight edge veto uses three distinct fresh ToF callbacks, is cleared after
progress, and is reconsidered when it is the only route left. Small residual
heading error after two alignment attempts may continue inside the live yaw
limit. Hard stop, odometry loss after translation, yaw runaway, and an
unacknowledged wheel stop remain fatal.

P2 reduces Round-1 cycle time without removing those guards. Translation now
starts odometry endpoint braking 18 cm before the target cell and uses the
slower of endpoint and live-ToF limits. A new cell physically scans only
`UNKNOWN` edges; the just-traversed edge is already confirmed `OPEN`. Camera
work uses a two-frame candidate gate and runs three-frame verification only
when a candidate survives. Every detected wall face gets this quick camera
check even after the configurable 6-8 second cell-scan budget; the budget
gates only optional open-corridor surveys. Open-direction survey defaults OFF
because a distant sign is checked again from its wall cell. Auto-Aim runs only
for the selected, range-confirmed target. A wall survey with no fresh frame or
an unverified candidate is retried once on a later/current-cell revisit before
being reported as exhausted.

The mission clock warns at 420 seconds and enters urgency mode at 525 seconds
without stopping exploration. The run continues until exact completion, an
operator stop, or a physical hard-safety fault. The GUI marks time beyond
10:00 as overtime while the controller keeps trying to complete the task.

### Aggressive field-test branch

Branch `codex/aggressive-field-test` intentionally defaults
`UNSAFE: disable all motion guards` to ON for an operator-supervised foam-maze
diagnosis. It bypasses movement preflight, live ToF/Gimbal holds, ToF braking,
yaw-runaway abort and cross-track abort in both rounds. Odometry endpoint
braking/completion, the GUI Stop button, Ctrl+C cleanup and final zero-wheel
commands remain active. Uncheck the option before Start to restore the normal
guarded controller.

## Requirements

- Python 3.8 (the RoboMaster SDK environment used for this project)
- RoboMaster EP connected over the network
- ToF sensor and camera available through the RoboMaster SDK
- Tkinter for the GUI (`sudo apt install python3-tk` on Ubuntu/Debian)

## Install

```bash
git clone https://github.com/nattannsra18/robomaster-assignment-2.git
cd robomaster-assignment-2
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Run

Start with a bounded low-speed hardware check in a clear area:

```bash
python final_round1_tof_camera_01.py \
  --no-gui --no-camera --max-moves 1 --travel-speed 0.10
```

Then launch the full operator GUI:

```bash
python final_round1_tof_camera_01.py
```

Useful options:

```bash
python final_round1_tof_camera_01.py --help
python final_round1_tof_camera_01.py --no-gui --travel-speed 0.10
python final_round1_tof_camera_01.py --max-moves 1 --max-yaw-correction 5
```

## Target selection and real firing

The GUI exposes firing modes `off`, `selected`, and `all`, an `ir`/`water`
selector, and a 4-color × 3-shape checkbox matrix. `selected` fires only the
checked exact pairs; `all` fires each newly verified target ID. Leave firing
`off` during camera/aim calibration. A shot is permitted only after
the requested color/shape is temporally verified, a fresh horizontal wall
range is confirmed, the estimated distance is no more than two cells, and the
stationary Auto-Aim loop has held the target at the calibrated impact point for
three fresh frames. Auto-Aim moves one Gimbal axis at a time, tolerates brief
detection dropouts, retries one transient failure, and remains bounded by
feedback age, divergence, timeout, and travel limits.

Run the first aim test without chassis translation and without firing:

```bash
python final_round1_tof_camera_01.py --no-gui \
  --stationary-target-test \
  --targets blue:circle
```

Inspect `targets.json`: the selected target should report
`auto_aim_success: true` and `mission_state: READY_DRY_RUN`. If the target is
consistently centered in the camera but a physical shot lands elsewhere,
calibrate `Camera-to-blaster X/Y offset ratio` in the GUI (or use
`--aim-offset-x` and `--aim-offset-y`) in small steps. Do not reverse an
Auto-Aim drive sign during an armed run; prove the sign with this stationary,
unarmed test first.

For a dedicated stationary firing test with the configuration GUI:

```bash
python stationary_target_fire_test.py
```

In `Mission Settings`, set `Target firing mode` to `selected`, choose `ir`,
set the shot count, and tick the exact color/shape pairs to fire. Choose `all`
only when every verified sign should be fired. The launcher forces stationary
mode: it scans with the Gimbal, performs fresh-frame Auto-Aim, fires only after
the normal selection/range/aim gates pass, exports the result, and exits before
the planner can command chassis translation.

The live camera shows a green FPS-style reticle at the calibrated
camera-to-blaster impact point. After the automatic four-direction scan, the
stationary GUI stays connected and enables `MANUAL FIRE`; each click sends the
configured IR/water shot count without target-selection or Auto-Aim gates.
Choose firing mode `off` for manual-only testing, or `selected`/`all` for
automatic firing plus the manual button. The chassis remains in wheel-zero
mode. Use `STOP & SAVE` to end the manual session and export its log.

For the focused water-shot calibration mode (no four-direction scan):

```bash
python stationary_auto_lock_water_test.py
```

Before starting, tick the intended color/shape in the target matrix. This mode
forces automatic firing `off` and water mode `on`, samples FRONT ToF while the
Gimbal is level, looks only at the FRONT camera, verifies the selected target,
and applies distance-dependent vertical parallax for the measured 5 cm camera-
above-blaster separation. When the console/UI reports `AUTO-LOCKED`, inspect
the FPS reticle and press `MANUAL FIRE`. The Gimbal remains locked until
`STOP & SAVE`; the chassis never translates.

Auto-Aim now accepts a lock only after the detected contour centroid remains
within 1.5% of the calibrated impact point for three consecutive fresh frames
(about 10x5 pixels at 640x360). Round 1 and the physical Round 2 executor both
apply distance-dependent vertical parallax from the live ToF range and the
configured 5 cm camera/muzzle separation before an armed shot. `Auto-aim
timeout (s)` defaults to 6 seconds; an old saved value of `0` also falls back
to 6 seconds. A camera that supplies no new frame for 0.30 seconds returns
`AIM_CAMERA_FRAME_STALE` instead of holding the mission indefinitely.
After exact color/shape verification, Auto-Aim may temporarily track the same
color blob nearest the previous centroid if motion blur changes its shape
classification. Firing still requires that centroid to settle at the
calibrated impact point.

`Moving wall-arrival stop (cm)` defaults to `20`. Once odometry has covered the
configured minimum progress (default 75%, or 45 cm of a 60 cm cell) and
cross-track is no greater than the cell-center tolerance, a
travel-direction ToF reading at or below this value sends an immediate wheel
stop and commits the commanded destination cell. Earlier short reflections
and laterally offset poses are ignored as arrival evidence.
This remains active on the aggressive operator-supervised branch even though
the other motion guards are bypassed.
In that guards-off mode, three distinct hard-stop ToF callbacks after 50% cell
progress commit the commanded destination so the mission cannot remain stuck
commanding zero speed at a close foam wall.

Wall-clearance logs now distinguish `CLEARANCE_ADJUST_STARTED`,
`CLEARANCE_TARGET_REACHED`, and a non-successful bounded finish. Each record
includes the before/after range, shifted distance, movement limit, and result.
After the same-direction camera survey, the chassis retraces that temporary
clearance shift to its pre-adjustment scan pose before another direction or
cell move. This prevents one-sided corrections from accumulating as cell
cross-track drift. In a corridor too narrow to satisfy both configured raw ToF
ranges, the robot preserves/retraces the scan pose instead of forcing itself
into the opposite wall. Gradual travel-direction ToF braking remains active in
the operator-supervised guards-off mode; the other diagnostic vetoes stay off.
A distant `SIGHTING_ONLY` target is also promoted into a later near-wall
observation when color, shape, view direction, and the hinted approach cell
identify exactly one candidate; ambiguous candidates keep separate IDs.

Bounded Gimbal scan failures, missing fresh horizontal ToF, pitch drift, and a
failed return from camera pitch now stop the wheels, leave that direction
`UNKNOWN`, and continue with the remaining directions. Operator stop and a
zero-wheel command without acknowledgement remain terminal.

`Camera-to-blaster Y offset ratio` is an additional empirical correction on
top of the 5 cm geometric correction. Increase it in small positive steps if
water still lands below the target; decrease it if shots land above. Water
trajectory drop, launcher mounting angle, and gel-bead variation cannot be
derived from camera height alone.

The equivalent terminal command is:

```bash
python final_round1_tof_camera_01.py --no-gui \
  --stationary-target-test \
  --targets blue:circle,red:rectangle \
  --arm-fire --fire-type ir --fire-times 1
```

This stationary command can rotate the Gimbal and fire the blaster, but cannot
translate the chassis. Remove `--stationary-target-test` only after the dry-run
and stationary firing test pass. A successful SDK return is recorded as a
command acknowledgement; it is not treated as proof that the projectile
physically hit the target. Use `--fire-type water` only for a correctly loaded
gel-bead blaster and test with a safe backstop.

## Build a Round-2 plan

After a Round-1 run finishes with `CLOSED_MAZE_COMPLETE`, open the map-based
planner:

```bash
python final_round2_target_plan_01.py \
  --run-dir classwork8_output/RUN_DIRECTORY \
  --gui
```

The GUI lets the operator choose the saved `start_cell`, saved `final_cell`, or
click any custom cell in the 6x6 map. It also records where the physical front
of the robot points on the saved map and converts every route/view direction
to the corresponding body-relative command. Required targets are selected by
exact `color:shape` checkboxes.

The same planner can run without a GUI:

```bash
python final_round2_target_plan_01.py \
  --run-dir classwork8_output/RUN_DIRECTORY \
  --targets blue:circle,red:rectangle \
  --start-from final --facing right

python final_round2_target_plan_01.py \
  --run-dir classwork8_output/RUN_DIRECTORY \
  --targets blue:circle \
  --start-cell 2,3 --facing front
```

It writes `round2_plan.json`. The planning command never connects to the robot.

## Execute Round 2 on the robot

Validate the saved plan first. This command never connects to the robot:

```bash
python final_round2_target_execute_01.py \
  --run-dir classwork8_output/RUN_DIRECTORY
```

For the first physical test, place the robot at the start cell and facing shown
by the validator, keep firing OFF, cap the route at one cell, and use 0.10 m/s:

```bash
python final_round2_target_execute_01.py \
  --run-dir classwork8_output/RUN_DIRECTORY \
  --execute --confirm-start \
  --max-route-steps 1 --travel-speed 0.10
```

The bounded test intentionally reports `ROUTE_STEP_LIMIT_REACHED` after the
cell. Once the measured cell motion is correct, run the full route without
`--max-route-steps`. It will move and Auto-Aim but will not fire:

```bash
python final_round2_target_execute_01.py \
  --run-dir classwork8_output/RUN_DIRECTORY \
  --execute --confirm-start --travel-speed 0.10
```

After the unarmed route passes, arm the required real blaster mode explicitly:

```bash
python final_round2_target_execute_01.py \
  --run-dir classwork8_output/RUN_DIRECTORY \
  --execute --confirm-start --arm-fire \
  --fire-type water --fire-times 1
```

Add `--gui` to choose the saved start/final/custom cell, physical facing, and
targets immediately before validation or execution. The executor reloads the
Round-1 movement calibration from `summary.json`, rebuilds the route from the
current topology/targets, and rejects stale or edited plans. At every approach
pose it stops the wheels, obtains fresh horizontal ToF, verifies the exact
color/shape near its Round-1 image location, Auto-Aims on fresh frames, and only
then permits a shot. A target range/camera/aim/fire failure is retried once and
then recorded while the robot continues to later targets from its still-known
approach cell. A movement failure after translation still stops the route
because the logical pose can no longer be trusted. Logs are written under
`round2_execution_TIMESTAMP` inside the Round-1 run directory.

The program sends real chassis commands. Keep the robot lifted or in a clear,
controlled test area for the first run, keep an operator ready to stop it, and
do not bypass the wheel-stop acknowledgement checks.

The first physical validation should remain limited to one cell at 0.10 m/s.
Offline tests do not certify stopping distance, odometry scale, Gimbal sign,
or collision clearance on the real robot.

## Verify without hardware

```bash
python -m unittest discover -s tests -p 'test_*.py' -v
python -m compileall -q final_round1_tof_camera_01.py \
  final_round2_target_plan_01.py final_round2_target_execute_01.py classwork8
```
