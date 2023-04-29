# Legged robot
Universal code for any kind of legged robots (not only legged now) and mobile friendly web interface included. Robot create WiFi Access Point.

## Supported legged robots

 - [ ] biped - 2 legs, 6 DOF, PCA9685 (WIP, mecanical issues, required redesign to continue)
 - [x] quadruped - 4 legs, 3 DOF, ESP32PWM
 - [ ] hexapod - 6 legs, 3 DOF, PCA9685 (WIP, nothing tested)
 - [x] tank - 2 tracks, DC motor driver e.g. MX1508 (unstable speed due to using gaits that is make no sense for tank)

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

## Build CLI (./build.sh)
Command line interface tool to setup dev environment and install required libraries.

## Purposouse of the project
This my attempt to make building legged robots as simple as possible (inspired by ArduPilot project).
This code mostly based on [ESP32 robot dog](https://github.com/SovGVD/esp32-robot-dog-code) code, but is not hardcoded to four legged robots. Also licence has been changed.
