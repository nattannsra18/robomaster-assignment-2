# RoboMaster Assignment 2

Clean, focused extraction of the Round 1 ToF + camera maze mission from
[`robomaster-autonomous-maze-navigation`](https://github.com/nattannsra18/robomaster-autonomous-maze-navigation/tree/refactor/v05-basic-motion-91fa792)
at source commit `6f8b5d92bb2abcf93b6b2cde6e74d5baae2139f4`.

The entrypoint is `final_round1_tof_camera_01.py`. It explores an unknown
fixed-cell maze, builds occupancy/topology outputs, surveys coloured shape
targets, and provides the V05 operator GUI. Blaster control and unrelated
pickup/drop mission code are intentionally excluded.

Stable V1 movement adds a fresh-ToF preflight before every cell, live
travel-direction ToF braking, an unconditional hard stop, and a recoverable
Moving Gimbal feedback hold. Logical cell arrival is still decided by
odometry; seeing a wall never marks a cell as reached. The GUI checkbox
`Moving Gimbal Check (diagnostic)` defaults to ON and disables only the
in-motion Gimbal angle/age check when switched off.

## Requirements

- Python 3.10+ (CI uses 3.11; offline checks also passed on 3.12)
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

The program sends real chassis commands. Keep the robot lifted or in a clear,
controlled test area for the first run, keep an operator ready to stop it, and
do not bypass the wheel-stop acknowledgement checks.

The first physical validation should remain limited to one cell at 0.10 m/s.
Offline tests do not certify stopping distance, odometry scale, Gimbal sign,
or collision clearance on the real robot.

## Verify without hardware

```bash
python -m unittest discover -s tests -p 'test_*.py' -v
python -m compileall -q final_round1_tof_camera_01.py classwork8
```
