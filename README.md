# RoboMaster Assignment 2

Clean, focused extraction of the Round 1 ToF + camera maze mission from
[`robomaster-autonomous-maze-navigation`](https://github.com/nattannsra18/robomaster-autonomous-maze-navigation/tree/refactor/v05-basic-motion-91fa792)
at source commit `6f8b5d92bb2abcf93b6b2cde6e74d5baae2139f4`.

The Round-1 entrypoint is `final_round1_tof_camera_01.py`. It explores an
unknown fixed-cell maze, builds occupancy/topology outputs, surveys coloured
shape targets, and provides the V05 operator GUI. Real RoboMaster blaster
control is available behind an explicit target allow-list and arm switch; it
is fail-safe OFF by default.

Stable V1 movement adds a fresh-ToF preflight before every cell, live
travel-direction ToF braking, an unconditional hard stop, and a recoverable
Moving Gimbal feedback hold. Logical cell arrival is still decided by
odometry; seeing a wall never marks a cell as reached. The GUI checkbox
`Moving Gimbal Check (diagnostic)` defaults to ON and disables only the
in-motion Gimbal angle/age check when switched off. Mission completion now
requires all 36 logical cells in the assignment's exact 6x6 grid; perimeter
wall readings are retained as diagnostics and do not hold a completed map open.

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

The GUI exposes `Arm target firing` and `Required targets (color:shape)`.
Leave firing OFF during camera/aim calibration. A shot is permitted only after
the requested color/shape is temporally verified, a fresh horizontal wall
range is confirmed, the estimated distance is no more than two cells, and the
stationary Auto-Aim loop has held the target at the calibrated impact point for
three fresh frames. Auto-Aim moves one Gimbal axis at a time and aborts on
stale feedback, target loss, increasing error, timeout, or its travel limit.

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
then permits a shot. Any movement, range, camera, aim, or fire failure stops the
remaining route. Logs are written under `round2_execution_TIMESTAMP` inside the
Round-1 run directory.

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
