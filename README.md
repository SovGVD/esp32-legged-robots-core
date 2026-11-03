# Legged robot
Universal code for any kind of legged robots (not only legged now) and mobile friendly web interface included. Robot create WiFi Access Point.

## Supported legged robots

 - [ ] biped - ESP32CAM, 2 legs, 6 DOF, PCA9685 (WIP, mecanical issues, required redesign to continue)
 - [x] quadruped - ESP32, 4 legs, 3 DOF, ESP32PWM
 - [ ] hexapod - ESP32CAM, 6 legs, 3 DOF, PCA9685 (WIP, nothing tested)
 - [x] tank - ESP32, 2 tracks, DC motor driver e.g. MX1508 (unstable speed due to using gaits that is make no sense for tank)

## Supported boards and sensors

### Boards
 - ESP32 (ESP32 doit devkit and similair)
 - ESP32CAM (ESP32-CAM - Ai-Thinker)

### Sensors and modules
 - Camera modules:
   - ESP32 camera (ESP32-CAM - Ai-Thinker)
 - PWM
   - via ESP32 itself
   - PCA9685
 - DC motor drivers
   - MX1508 and similar
 - IMU
   - MPU9650
 - Other sensors
   - IMA219 - power sensor

## Build

The firmware is built with [PlatformIO](https://platformio.org/) on Arduino-ESP32 core 3.x
(via the [pioarduino](https://github.com/pioarduino/platform-espressif32) platform). Libraries
are fetched automatically, there is nothing to install by hand except PlatformIO itself
(`pip install -U platformio`, or the PlatformIO extension for VS Code).

1. Copy `src/esp32-legged-robot/config_wifi.example.h` to `src/esp32-legged-robot/config_wifi.h`
   and set the access point SSID/password (without it the example credentials are used).
2. Build and upload for your robot, one environment per model
   (`robot-dog`, `robot-tank`, `robot-biped`, `robot-hexapod`):

```
pio run -e robot-dog                 # compile
pio run -e robot-dog -t upload       # compile + upload
pio device monitor                   # serial CLI, 115200
```

`./build.sh` is kept as a thin wrapper (`./build.sh compile robot-dog`, `./build.sh upload robot-dog [port]`,
`./build.sh cli [port]`) and still builds the web client (`./build.sh build-client-env`, needs Node.js),
which regenerates `src/esp32-legged-robot/web/index.html.gz.h`.

### Flash from the browser

Every merge into `main` builds the firmware for all robots and publishes a flasher page on GitHub Pages
(`.github/workflows/pages.yml`, nothing is published from `development` or pull requests). Open
<https://sovgvd.github.io/esp32-legged-robots-core/> in Chrome or Edge on a desktop, connect the robot with USB,
pick the model and press **Connect and install** ([ESP Web Tools](https://esphome.github.io/esp-web-tools/),
Web Serial). Leave **Erase device** unchecked to keep the servo trims in NVS.

These builds use the WiFi credentials from `config_wifi.example.h`, since `config_wifi.h` is not in git.

### Tests

The motion core (`lib/robot_core`: geometry, inverse kinematics, body kinematics, gait engine) has no Arduino
dependency and is unit tested on the PC:

```
pio test -e native
```

### Code layout

- `lib/robot_core/` - hardware independent motion code, shared by the firmware and the native tests
- `src/esp32-legged-robot/` - firmware: `main.cpp` (fixed-rate motion task on an `esp_timer`), `motion.cpp`
  (gait -> body -> IK pipeline, or the tank's differential drive), drivers behind the `ServoBus`, `Imu` and
  `PowerSensor` interfaces from `lib/robot_core/src/hardware.h` (`hal_esp32pwm.cpp`, `hal_pca9685.cpp`,
  `hal_dcmotor.cpp`, `sensors.cpp`), WiFi/web server, CLI, settings (NVS)
- `src/esp32-legged-robot/model/<name>/config.h` - compile time config of a robot (pins, sizes, hardware)
- `src/esp32-legged-robot/model/<name>/model_data.h` - robot description: legs, servo profile, gait and leveling
  parameters (no Arduino, also used by the tests and the simulator); `model.cpp` adds the IK solver and CLI
- `test/` - native unit tests

### Gait

Every model has `GAIT_PARAMS` in its `model_data.h`: cycle time, duty factor, per leg phase offsets, step height,
max speed/turn rate and acceleration. The move command from the web client is a velocity; the body moves
continuously and feet swing on a cycloid path to footholds placed ahead of the body (Raibert style). The gait is
time based (open loop); set `useContactSensors` to `true` once a robot has foot contact sensors filling
`leg.sensor.onGround`, and swings then end on touchdown.

**Static stability.** With `staticStability` the body shifts so the centre of mass stays over the feet that carry
it: each foot gets a support weight (0 in swing, ramping in after touchdown and out before lift-off) and the
centre of mass is steered to the weighted feet centroid, scaled per axis by `stabilityGain`. It is for slow gaits
with three feet on the ground. The dog has a second gait, `GAIT_PARAMS_WALK` (crawl: LH, LF, RH, RF, 1.2 s cycle,
20 mm/s, sideways shift only, about 16 mm stability margin); switch with the CLI command `gait walk` / `gait trot` (`gait state` shows the active one)
(applied once the robot stands still). In the simulator this walk tilts about 2° instead of 3-6° without the shift.

**IMU leveling.** `level on` in the CLI (or `HORIZON_LEVEL` in `config.h` to start with it on) tilts the body
against the slope the IMU measures, on top of the gait and static stability. The tilt comes from the accelerometer
and gyro fused by `TiltEstimator` (the accelerometer alone is thrown off by the robot's own accelerations), and
`BodyLeveler` is mostly integral with a 1.5° deadband, so it removes a steady slope without chasing the gait's
rocking; it holds its value during a trot. Parameters are `LEVEL_PARAMS` in `model_data.h`. In the simulator an
8° slope leaves under 3° of tilt (standing) and the static walk on a 6° side slope tilts half as much. Before
using it on a robot, check the IMU mounting with `level state`: raise the left side and the pitch must change,
raise the nose and the roll must change (the firmware's pitch is the sideways lean, as on the web client); fix
it with `IMU_SWAP_ROLL_PITCH`, `IMU_PITCH_SIGN` and `IMU_ROLL_SIGN`. `calib imu` offsets are saved in NVS.

**Battery protection.** The INA219 voltage is read at 10 Hz and filtered (servo current makes it sag every
step). Below `BATTERY_PARAMS.lowVoltage` for 3 s the robot warns (CLI, web page); below `criticalVoltage` it
ignores the controller, finishes the step, lowers the body by `SIT_HEIGHT` and turns the servos off, so it rests
low instead of dropping when the battery cuts out. Critical is latched until restart. Both are in each model's
`model_data.h`; the defaults are for a 2S LiPo (6.8 V / 6.4 V), change them for your pack. Without a power
sensor reading (e.g. USB powered on the bench) nothing happens.

**Telemetry.** The web page asks for telemetry every second (protocol in `src/web/src/protocol.ts`, version 2):
voltage, current, loop time, status bits (HAL, failsafe, gait and whether it is stepping, leveling, IMU and
power sensor found, battery state, resting, missed motion ticks), IMU tilt, missed tick count and the IK
counters (feet out of reach / joint limit / servo range clamps). The header shows them, e.g.
`walk · stepping · level · tilt 0.4° -1.2° · IK 0/3/0`.

**Motion loop timing.** The motion task runs every `LOOP_TIME` (dog 4.4 ms). The gait, leveler and tank drive
get the real time since the previous tick (capped at 50 ms), so a slow loop makes motion coarser, not slower.
`loop` in the CLI prints the last second: ticks run and missed, the longest gap between ticks, and the average
and slowest time of each stage (failsafe, power sensor, IMU read, motion pipeline, servo output). Every second
that misses ticks is also printed on the serial port with the same breakdown. `gait state` prints the gait the
loop really runs (cycle, step height, speeds, CoM offset). I2C transfers time out after `I2C_TIMEOUT_MS` (10 ms)
so one stuck read can't hold the loop for long.

**Tank.** No gait: the move command is mixed into left/right track speeds, slew limited (`TANK_DRIVE_PARAMS` in
`model/tank/model_data.h`: acceleration, deadband, minimum duty that starts the motors) and written as H-bridge
duties.

**Settings** are stored in NVS (`Preferences`, namespace `robot`) with a version and per-field defaults; the first
boot after updating imports the trims saved by the old EEPROM code.

Servos on plain ESP32 boards (`PWM_CONTROLLER_TYPE ESP32PWM`) are driven by the LEDC hardware PWM
peripheral (up to 16 servos, 50Hz by default, override with `ESP32PWM_SERVO_FREQ`).

### Hardware notes

- **Dog servo pins.** The dog drives servos from GPIO 12, 2 and 15, which are
  ESP32 strapping pins (GPIO 12 high at reset selects 1.8 V flash). The real
  dog boots and works fine with this wiring, so it stays as is; the boot pin
  check prints a warning for them, which is expected. Keep it in mind if you
  build a new board or use other servos that pull the line high at power on.
- **Hexapod PCA9685 boards.** 18 servos need two boards: servo pins 0-15 go to
  `PWM_CONTROLLER_ADDRESS` (default 0x40) and 16-31 to
  `PWM_CONTROLLER_ADDRESS_2` (default 0x42, since 0x41 is the INA219). Override
  either in `platformio.ini`, e.g. `build_flags = ${esp32.build_flags} -DROBOT_MODEL=MODEL_HEXAPOD -DPWM_CONTROLLER_ADDRESS_2=0x44`.
  Remove `PWM_CONTROLLER_ADDRESS_2` from `model/hexapod/config.h` for a single board.

## Purposouse of the project
This my attempt to make building legged robots as simple as possible (inspired by ArduPilot project).
This code mostly based on [ESP32 robot dog](https://github.com/SovGVD/esp32-robot-dog-code) code, but is not hardcoded to four legged robots. Also licence has been changed.
