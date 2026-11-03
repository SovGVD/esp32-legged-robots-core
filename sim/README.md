# Robot dog and hexapod physics simulation (MuJoCo)

MuJoCo models of the ESP32 small robot dog and the hexapod for testing motion
code before it goes on the real robots. Python, `pip install -r requirements.txt`.
The hexapod has [its own section](#hexapod) below; everything else is about the dog.

| | |
|---|---|
| ![Standing](docs/standing.png) | ![Walking with the firmware gait](docs/walking.gif) |
| Standing in the neutral pose, built from the printed STL parts | Trotting forward, driven by the firmware's own gait and IK (`lib/robot_core`) |
| ![Side view](docs/side.png) | ![Shoulder close-up](docs/shoulder.png) |
| Side view | Front holder: alpha servo gear and the 2:1 shoulder gear sector, hip-pitch servo in its bracket |

## Quick start

Needs Python 3.9+, a display for the viewer, and a C++17 compiler for the
firmware bridge (built automatically on first use): `g++` on Linux
(`sudo apt install g++`), Xcode command line tools on macOS
(`xcode-select --install`), MinGW-w64 `g++` on Windows (untested; WSL works
like Linux). Set `CXX` to pick another compiler.

```
git clone -b development https://github.com/SovGVD/esp32-legged-robots-core.git
cd esp32-legged-robots-core/sim
python -m venv .venv
source .venv/bin/activate            # Windows: .venv\Scripts\activate
pip install -r requirements.txt
python -m pytest -q tests            # checks everything works, builds the firmware bridge
python examples/drive.py             # viewer + control window: drive with WASD/arrows or an Xbox pad
```

`drive.py` runs the real firmware gait (lib/robot_core) on the sim; the
controls are in the table under "Manual control" below. To just look at the
model, run `python -m mujoco.viewer --mjcf models/dog.xml` (on macOS use
`mjpython` instead of `python` for any viewer command; `drive.py` doesn't run
on macOS, use `trot.py --view` or a video there).

## Commands

```
python -m pytest -q tests            # 63 checks, ~30 s (builds the firmware bridges on first run)
python examples/drive.py             # drive the firmware by keyboard or Xbox 360 pad (needs a display)
python examples/trot.py --view       # Python demo trot in the viewer (needs a display)
python examples/trot.py --video trot.mp4
python examples/snapshot.py dog.png  # still images of the standing robot
python examples/build_model.py       # regenerate models/dog.xml after editing params
```

`models/dog.xml` can also be opened directly in MuJoCo's `simulate` app.

## What is modelled

| Part | Source | Notes |
|---|---|---|
| Geometry: hips at (±39.5, ±70.5, 0) mm, L2 = 51.9, L3 = 50, neutral foot (±56, ±70.5, −80) | `model/dog/model.h`, checked against the STLs | The parts confirm all of these, including the "not sure" hip Y (see below) |
| Hip pitch axis level with the roll axis (`hip_pitch_drop` = 0) | STLs + photos (shoulder bracket) | Same as the firmware IK. The knob stays in `legsim/params.py` to try a misaligned build |
| 3 joints per leg: alpha = hip roll, beta = hip pitch, gamma = knee | firmware IK | Joint angles map 1:1 to firmware leg angles (see below) |
| Masses: torso 280 g, shoulder 20 g (incl. hip-pitch servo), upper leg 3.5 g, lower leg 16.1 g (incl. knee servo); total 438 g | STL volumes + datasheets | Centres of mass and inertias come from the placed STL parts and servo/battery boxes. A scale reading would only be a check |
| MG90D servo: 0.21 N·m stall, 0.10 s/60°, ~4° proportional band, 0.5° deadband, 50 Hz pulse latching, torque-speed line, rotor inertia, gear friction | datasheet + estimates | Hip roll gets the 2:1 gear: twice the torque, half the speed |
| Pulse → angle: `servoMainProfile` calibration table, `getHALAngle` with mid/ratio/inverse | `hal.cpp`, `servo.cpp` | The sim servo is assumed to follow the calibration exactly |
| IMU: accelerometer + gyro at the middle frame, body axes X right / Y forward / Z up | MPU9250 orientation in the IK comments | Optional Gaussian noise |
| Foot contacts: touch sensor per foot | — | Exposed but optional; the real robot has none yet |
| Looks: Gleb's printed parts (STL meshes) plus boxes for servos, battery and electronics | Thingiverse 4822059 | `models/meshes/`, placed by `legsim/assembly.py` |
| Collisions: torso box, battery box, leg capsules, 6 mm foot spheres, friction 0.8 | shoe STL | Hidden (geom group 3); no self collisions |

Every number in `legsim/params.py` is tagged with where it came from: `[code]`, `[stl]`, `[ds]` (datasheet) or `[est]` (guess, tune later).

## Printed parts (STL meshes)

The FreeCAD design (github.com/SovGVD/esp32-small-robot-dog-hardware) has the
parts but no assembly, so `legsim/assembly.py` places them from their
geometry. Each STL's origin turned out to sit on a joint axis:

| Part | What it shows |
|---|---|
| `leg_top` | servo hub holes 51.94 mm apart, exactly L2 |
| `leg_bottom` | knee servo pocket with the shaft ~50 mm from the foot ring, L3 |
| `legs_holder_part1/2` | roll bearings (12 mm) at x = ±39.5 on the roll axis; alpha servo gear at x = ±11 (28.5 mm centre distance, 2:1 with the shoulder sector) |
| `body` | 86.6 mm long; its ends meet the inner holder plates, which puts the hip pitch axis at y = ±70.4, so the "not sure" 70.5 is right |
| `shoulder_part2` | the U bracket lies in the plane of its gear sector and holds the hip-pitch servo centred on the roll axis, shaft pointing outward, so the pitch axis is level with the roll axis |

Estimated, not visible in the parts: where the hip-pitch servo sits in the
bracket (±1.5 mm), how leg_top / knee servo / leg_bottom stack sideways, and
the electronics/battery positions. Left legs use mirrored meshes, and the hind
holders and shoulders are the front ones turned around.

`models/meshes/` holds decimated copies made by `tools/prepare_meshes.py`
from the Thingiverse files (CC BY-NC, by the robot's author).

## Joint convention

The sim joints are zero with the leg straight down, and convert 1:1 to the
firmware's leg angles:

```
q_roll  = pi/2 - alpha   (+ = foot outward)
q_pitch = pi/2 - beta    (+ = upper leg backward)
q_knee  = pi   - gamma   (0 = straight leg)
```

`tests/test_kinematics.py` checks this both ways: a Python port of
`QuadrupedLegIK` is fed foot targets, and the MuJoCo model's feet land exactly
on them. It also round-trips leg angle → pulse → leg angle through the HAL for
all four legs with their `inverse` flags.

## Interfaces (for the firmware bridge)

`legsim.SimDog` exposes the robot the way the hardware layer sees it:

| Firmware side | SimDog |
|---|---|
| servo bus: pulse per channel (`leg*3 + joint`), 0 = off | `write_pulse(ch, us)`, `write_pulses(us[12])` |
| IMU: accel + gyro | `read_imu()` → `{"accel": m/s², "gyro": rad/s}` |
| contact sensors (optional, future) | `foot_contacts()` |
| loop timing | `step(seconds)`; firmware `LOOP_TIME` is 4.4 ms |

Plus helpers: `set_leg_angles()` (runs the HAL like `runServoHAL`),
`set_feet()` (Python IK port → HAL), `set_joint_targets()`, and ground truth
(`body_position_mm()`, `body_orientation()` in the firmware's pitch/roll/yaw
convention, `foot_positions_body_mm()`).

## Running the firmware in the sim

`legsim.FirmwareCore` runs the firmware's own motion code: `lib/robot_core`
(`MotionController`: GaitEngine, BodyKinematics, QuadrupedLegIK, plus the
servo HAL) with the dog's legs, servo profile and gait parameters from
`src/esp32-legged-robot/model/dog/model_data.h`. It is compiled with the host
C++ compiler into `sim/build/librobot_core_bridge.so` (`sim/bridge/`), rebuilt
automatically when any of those sources change, and called through ctypes.

```python
from legsim import SimDog, FirmwareCore
dog, fw = SimDog(), FirmwareCore()
fw.drive(dog, (0, 1, 0, 0, 0, 0), duration=3.0)   # forward at full speed for 3 s
```

Each tick is `rc_update(dt, command, imu) -> pulses[12]`, the same as
`updateMotion()` + `servoSet()` on the robot, then `SimDog.write_pulses()`.
The command is `LR_moveVector`: move x (+ right), y (+ forward), z, rotate
pitch, roll, yaw (+ turns right), each in [-1, 1], like the web client.
The IMU reading is the simulated accelerometer and gyro run through the
firmware's own `TiltEstimator` (`fw.imu_angles(dog)`); it feeds the IMU
leveler, which is off unless `fw.set_leveling(True)` (or the model's
`HORIZON_LEVEL`).
`fw.select_gait("walk")` picks the dog's statically stable walk
(`GAIT_PARAMS_WALK`), `fw.support_margin()` reports how far the centre of
mass is inside the support polygon, and `fw.tune_gait(...)` /
`fw.set_stability_gain(...)` change timing and shift for experiments.

`tests/test_firmware.py` checks that the C++ IK + HAL produce the same
pulses as the Python port, that the firmware stands, trots, strafes,
turns and stops in the sim, and that the static walk keeps the centre of
mass inside the triangle and tilts less than the same walk without the
shift, and that leveling takes out most of an 8° slope (gravity tilted)
without rocking the robot on flat ground.

### Manual control

`python examples/drive.py` opens the MuJoCo viewer and a small "robot
control" window (`pip install pygame-ce`; if plain `pygame` is also installed, `pip uninstall pygame` first, they share the `pygame` module). Focus that window for the keyboard:

| Keyboard | Xbox 360 pad | Action |
|---|---|---|
| W / S, Up / Down | left stick up / down | forward / back |
| A / D | left stick left / right | strafe |
| Q / E, Left / Right | right stick X | turn |
| I / K, J / L | hold LB + right stick | tilt body (pitch, roll) |
| Shift | | slow (40 %) |
| G | Y | switch gait: trot / static walk (while standing) |
| R | Back | reset robot |
| P | Start | pause |
| Esc | | quit |

Keyboard and pad add up. Any pad SDL knows as a game controller works
(Xbox 360/One, most others). Linux and Windows; on macOS the MuJoCo viewer
needs `mjpython`, which conflicts with pygame's window.

## Hexapod

| | |
|---|---|
| ![Hexapod standing](docs/hexapod_standing.png) | ![Hexapod walking with the firmware gait](docs/hexapod_walking.gif) |
| Neutral stance, built from Gleb's printed parts | Walking forward, driven by the firmware's own gait and `HexapodLegIK` |

```
python examples/drive.py --hexapod             # drive it by keyboard / Xbox pad (needs a display)
python examples/hexapod_walk.py                # scripted walk, prints distance, tilt, servo load
python examples/hexapod_walk.py --video walk.mp4
python examples/snapshot.py hexapod.png --hexapod
python -m pytest -q tests/test_hexapod.py
```

`models/hexapod.xml` opens in MuJoCo's `simulate` app. In code:

```python
from legsim import FirmwareCore
from legsim.hexapod import SimHexapod
fw = FirmwareCore(model="hexapod")             # builds sim/build/librobot_core_bridge_hexapod.so
robot = fw.place(SimHexapod())
fw.drive(robot, (0, 1, 0, 0, 0, 0), duration=3.0)   # forward, full speed (75 mm/s), 3 s
```

`SimHexapod` has the same interface as `SimDog` (`write_pulses`, `read_imu`,
`set_leg_angles`, `set_feet`, ground truth) with 18 channels, `leg*3 + joint`
in the firmware leg order LF, LM, LR, RF, RM, RR (not the PCA9685 channel;
the second board at 0x42 only matters on hardware). `joint_torque_ratio()`
says how close each servo is to stall.

| Part | Source | Notes |
|---|---|---|
| Coxa axes at (±46, ±80) and (±57.5, 0) mm, coxa 35, femur 55, tibia 110 | `model/hexapod/config.h`, checked on the STLs | Bearing holes in the body, the coxa's yaw and femur axes, the knee axis and tibia tip all land on the firmware numbers |
| Body origin at the femur axis height, 28.7 mm above the body bottom | STLs | Standing height 67.3 mm (IK), 38.6 mm ground clearance |
| Gears: coxa 2:1, femur 2:1, tibia 1.5:1 | STLs (tooth tips: 18-tooth servo gears, 36-tooth coxa sector and femur gear, 27-tooth tibia gear, module 1) | Same as the HAL ratios in `model_data.h` |
| Servos: coxa servos upside down in the body bottom, femur servo in the coxa, tibia servo in the femur | servo screw holes in the STLs | Shafts placed at the gear centre distances (27 and 22.5 mm) |
| MG90S clone: 0.176 N·m stall, 0.10 s/60° | datasheet at 4.8 V | Same servo model as the dog otherwise |
| Pulse → angle: hexapod `servoMainProfile` (0-180°, 590-2200 µs) | `model_data.h` | |
| Masses: ~892 g total | STL volumes + datasheets + estimates | Body halves 124 g, 18 servos 241 g, 2x18650 + holder 106 g, electronics 47 g; per leg coxa 31, femur 29, tibia 19 g |
| Right legs mirrored, left legs turned | photos | All femur and tibia gears face forward, like on the real robot |

The STLs (`tmp-hexapod-*`, `tmp-Femur`, servo gear) are the ones that match
the photos; `tools/prepare_hexapod_meshes.py` decimates them into
`models/meshes/hexapod/`. The `hexapodv2` body in the FreeCAD file is a
newer, different body and isn't used. Estimated: the electronics and battery
positions, the IMU at the body centre, and everything that needs a scale.

First findings (with the estimated 892 g at ~5 V):

* The firmware gait walks it as configured: about 270 mm forward in 4 s
  (75 mm/s), 190 mm strafing in 3 s, turning at 0.75 rad/s, with under 3°
  of tilt. No IK failures.
* **Femur servos are at their limit.** Standing on six legs they use ~46 %
  of stall torque; on three legs (the 2:1 femur gear gives 0.35 N·m) they
  are saturated about 10 % of the time and the body sags 5-9 mm per step.
  The real robot's weight and servo voltage decide how bad this is; less
  mass or 6 V on the servos would help most.
* Strafing and turning occasionally ask for more than the coxa's ±45°
  (2:1 gear on a 180° servo); the firmware clamps those (`servo_range` count).
* Not checked by the sim: servo directions. The model sets no angle
  `inverse` flags, so a pulse moves the left and right legs' servos the same
  way. With mirrored legs on the real robot one side may need them; the sim
  follows the firmware's HAL either way.

## Plan

1. **Model (done).** Parametric MJCF, servo and IMU models, tests that
   pin the joint convention to the firmware IK and HAL.
2. **Firmware bridge (done, see above).** Build `lib/robot_core` (Arduino-free since the
   C++ module refactor) for the host as a small shared library with a C API:
   `rc_init(config)`, `rc_update(dt, command, imu) -> pulses[12]`. Load it
   from Python with `ctypes` and drive `SimDog.write_pulses` from it at the
   4.4 ms loop rate. Then the sim runs the real `GaitEngine`, IK and balance
   code, not the Python demo gait. The sim side won't need changes: the C API maps onto
   `write_pulses` / `read_imu`.
3. **Scenario tests.** Run in CI on the host: stand, trot forward/turn,
   step over a 5 mm obstacle, push recovery, slope. Check distance, drift,
   max tilt, and servo torque saturation (the sim can report how close each
   servo is to stall, which hardware can't).
4. **Calibrate against the real robot** when convenient: one video of a trot
   in place and a forward trot, and optionally a weight. Tune masses, servo
   kp/deadband and foot friction until drift and tilt match.

## Known limits and first findings

* Masses are estimates from STL volumes (20 % infill); inertias come from
  the meshes as shells, which suits printed parts.
* **Hip geometry:** the hip pitch axis is level with the roll axis and the
  leg plane is within ~2 mm of it sideways, so the firmware IK's L1 = 0 holds.
  `hip_pitch_drop` can model a pitch axis below the roll axis if a build
  differs (the sim then stands that much taller than the IK expects).
* Servo voltage is assumed ~5 V (datasheet 4.8 V values).
* **Body wobble while trotting** came from the servos, not the gait plan. An
  MG90D turns ~10 rad/s (the hip roll, with its 2:1 gear, ~5 rad/s) and reads
  the pulse at 50 Hz. A swing that asks for more lands the foot 50-100 ms late,
  the body stands on one diagonal that much longer and tips over it. The dog's
  gait keeps within that: 15 mm step height, and `maxSideSpeed` = 50 mm/s
  limits strafing plus turning (the hip roll joints do all the sideways work).
* The firmware trot's backward drift (~13 mm/s in place, ~60 mm/s strafing)
  came from the centre of mass sitting ~5 mm behind the middle of the feet;
  `centerOfMass` in the gait params now moves the body over it.
* **Trotting in place drifts backward** in the sim (about 30 mm/s with the
  Python demo trot, duty 0.5, 15 mm step height). The most likely cause is stance-leg
  compliance: the hobby servos sag under load, and with all knees bending the
  same way the sag shifts the body backward a bit every step. It shrinks
  with lower steps or more leg overlap (duty 0.6). Worth checking whether the
  real robot does the same, since that is the first calibration point.
