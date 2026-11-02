# Robot dog physics simulation (MuJoCo)

A MuJoCo model of the ESP32 small robot dog for testing motion code before it
goes on the real robot. Python, `pip install -r requirements.txt`.

```
python -m pytest -q tests            # 19 checks, ~2 s
python examples/trot.py --view       # interactive viewer (needs a display)
python examples/trot.py --video trot.mp4
python examples/build_model.py       # regenerate models/dog.xml after editing params
```

`models/dog.xml` can also be opened directly in MuJoCo's `simulate` app.

## What is modelled

| Part | Source | Notes |
|---|---|---|
| Geometry: hips at (±39.5, ±70.5, 0) mm, L2 = 51.9, L3 = 50, neutral foot (±56, ±70.5, −80) | `model/dog/model.h` | Hip Y is still "not sure" in the firmware; change it in `legsim/params.py` |
| 3 joints per leg: alpha = hip roll, beta = hip pitch, gamma = knee | firmware IK | Joint angles map 1:1 to firmware leg angles (see below) |
| Masses: torso 280 g, shoulder 20 g (incl. hip-pitch servo), upper leg 3.5 g, lower leg 16 g (incl. knee servo); total 438 g | STL volumes + datasheets | Estimates. A scale reading would only be a check |
| MG90D servo: 0.21 N·m stall, 0.10 s/60°, ~4° proportional band, 0.5° deadband, 50 Hz pulse latching, torque-speed line, rotor inertia, gear friction | datasheet + estimates | Hip roll gets the 2:1 gear: twice the torque, half the speed |
| Pulse → angle: `servoMainProfile` calibration table, `getHALAngle` with mid/ratio/inverse | `hal.cpp`, `servo.cpp` | The sim servo is assumed to follow the calibration exactly |
| IMU: accelerometer + gyro at the middle frame, body axes X right / Y forward / Z up | MPU9250 orientation in the IK comments | Optional Gaussian noise |
| Foot contacts: touch sensor per foot | — | Exposed but optional; the real robot has none yet |
| Feet: 5.5 mm spheres, friction 0.8 | shoe STL | Only feet, legs and torso box touch the floor; no self collisions |

Every number in `legsim/params.py` is tagged with where it came from: `[code]`, `[stl]`, `[ds]` (datasheet) or `[est]` (guess, tune later).

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

## Plan

1. **Model (this change).** Parametric MJCF, servo and IMU models, tests that
   pin the joint convention to the firmware IK and HAL.
2. **Firmware bridge.** Build `lib/robot_core` (Arduino-free since the
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

* Masses and inertias are estimates from STL volumes; link shapes are boxes
  and capsules, not the STL meshes.
* The L1 = 0 simplification of the firmware IK is used as the true geometry.
  If the real hip-pitch axis is offset from the roll axis, both the IK and the
  sim need the offset.
* Servo voltage is assumed ~5 V (datasheet 4.8 V values).
* **Trotting in place drifts backward** in the sim (about 30 mm/s with the
  Python demo trot, duty 0.5, 15 mm step height). The most likely cause is stance-leg
  compliance: the hobby servos sag under load, and with all knees bending the
  same way the sag shifts the body backward a bit every step. It shrinks
  with lower steps or more leg overlap (duty 0.6). Worth checking whether the
  real robot does the same, since that is the first calibration point.
