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

Servos on plain ESP32 boards (`PWM_CONTROLLER_TYPE ESP32PWM`) are driven by the LEDC hardware PWM
peripheral (up to 16 servos, 50Hz by default, override with `ESP32PWM_SERVO_FREQ`).

## Purposouse of the project
This my attempt to make building legged robots as simple as possible (inspired by ArduPilot project).
This code mostly based on [ESP32 robot dog](https://github.com/SovGVD/esp32-robot-dog-code) code, but is not hardcoded to four legged robots. Also licence has been changed.
