# Legged robot
Universal code for any kind of legged robots (not only legged now) and mobile friendly web interface included. Robot create WiFi Access Point.

## Supported legged robots

 - [ ] biped - ESP32CAM, 2 legs, 6 DOF, PCA9685 (WIP, mecanical issues, required redesign to continue)
 - [x] quadruped - ESP32, 4 legs, 3 DOF, ESP32PWM
 - [ ] hexapod - any ESP32, 6 legs, 3 DOF, MG90S direct drive, 2x PCA9685 (new 2026-11 design, sim only, not built yet)
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
   - MPU9250 (GY-91): accelerometer, gyro and the AK8963 magnetometer (heading)
 - Distance
   - VL53L5CX 8x8 multizone ToF (terrain map: obstacles and gaps; indoor map by room scans)
   - LDROBOT LD06 / LD19 360° lidar (indoor map, localization, walking to a point), optional
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

**Heading.** The gyro and the GY-91's magnetometer (AK8963 inside the MPU9250) give a compass heading
(`HeadingEstimator`: the gyro for the fast part, pulled toward the tilt compensated magnetometer over ~2 s;
readings whose field strength is off by more than 25 % from the calibrated one, e.g. next to a servo or steel,
are ignored). Many "GY-91" boards carry an MPU6500 without a magnetometer: then the heading is the gyro alone,
relative to the start, and drifts. Calibrate once on the robot, servos on, away from steel: `heading cal`
turns it two full turns in place (needs the web client connected; `heading hand` instead collects for 60 s
while you turn it by hand), and the result is saved in NVS. `heading on` holds the heading: while you turn
with the stick the target follows, and when you let go the robot keeps the heading it has then.
`turnto <deg>` turns to a compass heading and holds it (`heading off` stops). `heading state` prints the
heading, the field strength and whether it is used. The hold pauses while the failsafe is on (no client).

**Terrain (ToF).** `terrain on` (once standing) switches to the rough gait (`GAIT_PARAMS_ROUGH`, slower and
higher steps), turns leveling on and feeds the VL53L5CX frames into the terrain map: feet step onto
obstacles instead of into them, footholds keep away from gaps, and a gap too wide to step over stops the
robot in that direction until the stick lets go (web page: "GAP AHEAD: stopped"). It is the code the
simulator results come from (`sim/README.md`, "Rough ground"), not tested on the real robots yet. The dog has
one sensor on the nose, the hexapod two on the front (`TOF_SENSORS` in `model_data.h`: position, angles).
Wiring: the sensors share the main I2C bus by default (reads are split into 32 byte transfers so the IMU
never waits long; `loop` shows a `tof` stage); `TOF_I2C_SDA`/`TOF_I2C_SCL` in `config.h` put them on the
second I2C controller instead. Two sensors need `TOF_LPN_PIN` wired to the second sensor's LPn (hexapod
default GPIO 14, an assumption). Check the zone order with `tof print`: cover the left half of a sensor and the
left columns must get short, cover the top half and the top rows; fix with `TOF_ZONE_MIRROR_X` /
`TOF_ZONE_MIRROR_Y`. Facing a flat wall the corner zones should read ~10 % more than the middle (distance along
each zone's ray); if they read the same, set `TOF_DISTANCE_PERPENDICULAR 1`.

**Indoor map and walking to a point.** The robot keeps a 20 x 20 m map of the walls and furniture around
where it started (10 cm cells, 40 KB, allocated when the first sensor data arrives), corrects its odometry
(gait velocity turned by the gyro) by matching scans against it, and walks to a map point on a path planned
around what it knows (a 10 x 10 m window of the map around the robot and the goal, 30 KB, replanned every
second). All of it runs on core 0; the motion task only takes the result, so the loop timing doesn't change.
`goto <x m> <y m>` walks to a map point (map frame: x right, y forward of where the map started; `lidar reset`
starts a new map here). Moving the stick, `lidar stop` or the web page's Stop cancels it; without a web
client (failsafe) the robot stands. The web page's **Map** toggle shows the map, the robot, the last scan and
the path; tap the map to walk there. `lidar state` prints the match quality, pose and goal, `lidar freeze`
toggles keeping the map as it is (localize only). Parameters: `NAV_PARAMS` and the map parameters in
`model_data.h`. Simulator only so far (`sim/README.md`, "Lidar" and "ToF room scans"). Two ways to scan:

- *ToF room scans* (the default, no extra hardware: no `LIDAR_TYPE` in `config.h`). The terrain ToF sensors
  look down at the ground, so a scan stops the robot, tilts its nose up until the sensors' top zone row looks
  level, turns it once around (~15 s) and puts what they saw near level into the map; walking to a goal it
  scans again every metre (`TOF_SCAN_PARAMS.scanEvery`) and walks on odometry in between, stopping for
  anything higher than 4 cm right ahead (and marking it in the map). `lidar scan` (or the Map view's **Scan**)
  scans once. The gyro's bias is learned whenever the robot stands. The sensors are mounted flatter than for
  terrain alone (`TOF_SENSORS`): the legs only tilt the dog's nose ~15° and the hexapod's ~10°, and the scan
  wants the top row's lower edge level (nose up = mount angle - 17°). **Dog: the nose sensor 25° down** (nose
  up 8° to scan). **Hexapod: the right sensor 22° down** (6°), the left one stays 35° down for terrain (both
  at 22° missed 70 mm gaps). In the sim both reached every goal (dog's map pose ~10 cm off, hexapod's ~30 cm and 6-11° with one sensor scanning); at 40° / 35° walls further than
  ~1 m are lost and the robots got lost, and a middle ground (dog 30°, hexapod 27°) got lost too. The VL53L5CX reaches ~2.5 m indoors: rooms are mapped from the middle
  out, wide halls poorly. Parameters: `TOF_SCAN_PARAMS`, `TOF_MAP_PARAMS`.
- *Lidar* (LD06 / LD19, `#define LIDAR_TYPE LD06` in `config.h`, or `pio run -e robot-dog-lidar`): scans 10
  times a second while walking, cm accurate. On top of the body, above every part of the robot (the scan
  plane must not cross the legs or the head; readings closer than `minRange`, dog 100 mm, hexapod 150 mm,
  count as the robot itself and are dropped). Wiring: lidar TX to `LIDAR_RX_PIN` (dog GPIO 34, input only and
  free; hexapod GPIO 15, free on the ESP32-CAM without an SD card), 5 V (~180 mA), GND, and the lidar's PWM
  pin to GND so it runs its own 10 rev/s speed control. It only sends (UART2, 230400 baud). `lidar print`
  prints the closest reading in 8 directions: put a box 30 cm in front of the robot and it must show as
  `front`, else fix `LIDAR_PARAMS.yaw` in `model_data.h`. The lidar sees only its own height (~12-15 cm):
  lower things (steps, cables) are for the ToF terrain map. Parameters: `LIDAR_PARAMS`, `LIDAR_MAP_PARAMS`.

**Battery protection.** The INA219 voltage is read at 10 Hz and filtered (servo current makes it sag every
step). Below `BATTERY_PARAMS.lowVoltage` for 3 s the robot warns (CLI, web page); below `criticalVoltage` it
ignores the controller, finishes the step, lowers the body by `SIT_HEIGHT` and turns the servos off, so it rests
low instead of dropping when the battery cuts out. Critical is latched until restart. Both are in each model's
`model_data.h`; the defaults are for a 2S LiPo (6.8 V / 6.4 V), change them for your pack. Without a power
sensor reading (e.g. USB powered on the bench) nothing happens.

**Telemetry.** The web page asks for telemetry every second (protocol in `src/web/src/protocol.ts`, version 3):
voltage, current, loop time, status bits (HAL, failsafe, gait and whether it is stepping, leveling, IMU and
power sensor found, battery state, resting, missed motion ticks, terrain mode, gap ahead, heading hold,
magnetometer heading, ToF running), IMU tilt, missed tick count, the IK counters (feet out of reach / joint
limit / servo range clamps), the heading and the hold target. The header shows them, e.g.
`walk · stepping · level · tilt 0.4° -1.2° · IK 0/3/0`.

**Motion loop timing.** The motion task runs every `LOOP_TIME` (dog 4.4 ms). The gait, leveler and tank drive
get the real time since the previous tick (capped at 50 ms), so a slow loop makes motion coarser, not slower.
`loop` in the CLI prints the last second: ticks run and missed, the longest gap between ticks, and the average
and slowest time of each stage (failsafe, power sensor, IMU read, motion pipeline, servo output). Every second
that misses ticks is also printed on the serial port with the same breakdown. `gait state` prints the gait the
loop really runs (cycle, step height, speeds, CoM offset). I2C transfers time out after `I2C_TIMEOUT_MS` (10 ms)
so one stuck read can't hold the loop for long. On the ESP32, `ledcWrite()` waits for the previous duty of
that channel to reach the output (up to one 20 ms servo period), so the LEDC servo bus skips unchanged pulses
and leaves a new one for the next tick while the previous is still pending (it cost the dog ~40 ms per tick).

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
