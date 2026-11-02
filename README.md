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

### Tests

The motion core (`lib/robot_core`: geometry, inverse kinematics, body kinematics, gait engine) has no Arduino
dependency and is unit tested on the PC:

```
pio test -e native
```

### Code layout

- `lib/robot_core/` - hardware independent motion code, shared by the firmware and the native tests
- `src/esp32-legged-robot/` - firmware: `main.cpp`, `motion.cpp` (gait -> body -> IK pipeline), HAL backends
  (`hal_esp32pwm.cpp`, `hal_pca9685.cpp`, `hal_dcmotor.cpp`), sensors, WiFi/web server, CLI, settings
- `src/esp32-legged-robot/model/<name>/config.h` - compile time config of a robot (pins, sizes, hardware)
- `src/esp32-legged-robot/model/<name>/model.cpp` - robot description: legs, servo profile, gait parameters, IK solver
- `test/` - native unit tests

### Gait

Every model has `GAIT_PARAMS` in its `model.cpp`: cycle time, duty factor, per leg phase offsets, step height,
max speed/turn rate and acceleration. The move command from the web client is a velocity; the body moves
continuously and feet swing on a cycloid path to footholds placed ahead of the body (Raibert style). The gait is
time based (open loop); set `useContactSensors` to `true` once a robot has foot contact sensors filling
`leg.sensor.onGround`, and swings then end on touchdown.

**Static stability.** With `staticStability` the body shifts so the centre of mass stays over the feet that carry
it: each foot gets a support weight (0 in swing, ramping in after touchdown and out before lift-off) and the
centre of mass is steered to the weighted feet centroid, scaled per axis by `stabilityGain`. It is for slow gaits
with three feet on the ground. The dog has a second gait, `GAIT_PARAMS_WALK` (crawl: LH, LF, RH, RF, 1.2 s cycle,
20 mm/s, sideways shift only, about 16 mm stability margin); switch with the CLI command `gait walk` / `gait trot`
(applied once the robot stands still). In the simulator this walk tilts about 2° instead of 3-6° without the shift.

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
