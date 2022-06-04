# Legged robot
Universal code for any kind of legged robots. Mobile friendly web interface included.

## Purposouse of the project
This my attempt to make building legged robots as simple as possible (inspired by ArduPilot project). 
This code mostly based on [ESP32 robot dog](https://github.com/SovGVD/esp32-robot-dog-code) code, but is not hardcoded to four legged robots. Also licence has been changed. 

## Supported boards and sensors

### Boards
 - ESP32 (ESP32 doit devkit and similair)
 - ESP32CAM (ESP32-CAM - Ai-Thinker)

### Sensors and modules
 - Camera modules:
   - ESP32 camera (ESP32-CAM - Ai-Thinker)
 - PWM
   - PCA9685
 - IMU
   - MPU9650
 - Other sensors
   - IMA219 - power sensor

## Build CLI (./build.sh)
Command line interface tool to setup dev environment and install required libraries.
