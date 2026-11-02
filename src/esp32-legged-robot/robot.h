/**
 * Firmware wide declarations: configuration, shared state and the functions
 * each module exposes. Every firmware .cpp file includes this header.
 *
 * Hardware independent motion code (geometry, IK, gait) lives in
 * lib/robot_core and is unit tested natively (`pio test -e native`).
 */
#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <robot_core.h>

#include "def/def.h"
#include "def/cli.h"
#include "config.h"

// ROBOT_MODEL normally comes from platformio.ini (-DROBOT_MODEL=MODEL_*).
#if !defined(ROBOT_MODEL) && __has_include("config_model.h")
	#include "config_model.h"
#endif
#ifndef ROBOT_MODEL
	#error "ROBOT_MODEL is not set. Build with `pio run -e robot-dog` (or another env from platformio.ini)."
#endif

#include "model/model.h"
#include "model/settings.h"

#define LOOP_DT (LOOP_TIME / 1000000.0f)   // motion loop period, seconds

/**
 * Robot description (model/<name>/model.cpp)
 */
extern leg               legs[LEG_NUM];
extern servoProfile      servoMainProfile;
extern const GaitParams  GAIT_PARAMS;
extern LegIK            &MODEL_IK;
#if PWM_CONTROLLER_TYPE == DCMOTOR
extern const DiffDriveParams TANK_DRIVE_PARAMS;
#else
extern const LevelerParams   LEVEL_PARAMS;
extern const float           SIT_HEIGHT;
#endif
extern const BatteryParams   BATTERY_PARAMS;
extern const cliCommand *modelCliCommands;
extern const uint8_t     modelCliCommandsCount;
void modelSettingsPrint();
void modelSettingsLoad();
void modelSettingsSave();

/**
 * Shared state (main.cpp)
 */
extern LR_angle      IMU_DATA;
extern LR_angle      imuCorrection;
extern uint16_t      servoPulse[LEG_NUM][LEG_DOF];
extern bool          isHalDoReady;
extern unsigned long loopTime;
extern modelSettings settings;
uint32_t motionOverrunCount();   // motion ticks missed because a tick ran longer than LOOP_TIME

/**
 * Motion (motion.cpp): gait, body kinematics, IK
 */
void initMotion();
void updateMotion();
const LR_figure &motionBody();
void requestGait(const GaitParams &params);   // applied once the robot stands still

/**
 * HAL (hal.cpp, servo.cpp, hal_*.cpp)
 */
void initHAL();
void updateHAL();
void doHAL();
bool isHALEnabled();
void enableHAL();
void disableHAL();
void setLegPWM(leg &_leg);
void runServoCalibrate(leg &_leg);
float limitServoAngle(float angle);
float getHALAngle(float angle, float mid, float trimAngle, float gearRatio, bool inverse);

void initServo();
void setMotorsToCalibratePosition();
void servoSet();
void doMotors();
bool servoReady();
uint16_t angleToPulse(float angleRad);

void disableServos();
void enableServos();

// the bus of the selected PWM_CONTROLLER_TYPE (hal_esp32pwm/hal_pca9685/hal_dcmotor.cpp)
ServoBus &servoBus();

/**
 * Sensors (sensors.cpp)
 */
// Motion state for telemetry (motion.cpp)
struct MotionStatus {
	bool     gaitActive;
	bool     staticGait;
	bool     sitting;
	uint32_t unreachable;
	uint32_t jointLimit;
	uint32_t servoRange;
};
MotionStatus motionStatus();

// Battery protection (battery.cpp)
void         initBattery();
void         updateBattery(float dt);
BatteryState batteryState();

// IMU leveling (motion.cpp), on with HORIZON_LEVEL or `level on`
void     setLeveling(bool on);
bool     levelingEnabled();
LR_angle levelCorrection();
Imu         &imu();           // IMU_TYPE, or NoImu
PowerSensor &powerSensor();   // POWER_SENSOR, or NoPowerSensor
void initIMU();
void updateIMU();
void calibrateIMU();
void initPowerSensor();
void updatePower();
float getPowerSensorVoltage();
float getPowerSensorCurrent();

/**
 * Failsafe (failsafe.cpp)
 */
extern bool FS_FAIL;
void failsafeFeed();      // call on every package from the controlling client
void updateFailsafe();
void setFailsafe();

/**
 * Move command (command.cpp), safe to use from any task
 */
void setMoveCommand(const LR_moveVector &value);
LR_moveVector getMoveCommand();

/**
 * Network: WiFi AP, web server, websocket packages (wifi.cpp, web_server.cpp, packages.cpp)
 */
extern IPAddress WiFiIP;
extern uint8_t   telemetryPackage[P_TELEMETRY_LEN];
void initWiFi();
void WiFiInfo();
void initWebServer();
void updateWebServer();
void pMove(uint8_t *data, size_t len);
#ifdef ESP32CAMERA
void initCameraStream();   // web_server.cpp, needs the esp32camera library
#endif
void int16ToBytes(int value, uint8_t _offset);
void pTelemetry();
void pCli(uint8_t *data, size_t len);

/**
 * CLI (cli.cpp, cli_commands.cpp)
 */
#include "cli_stream.h"
extern cliStream *cliSerial;
void initCLI();
void cliInitHelp();
void updateCLI();
void CLI_doCommand();
char *CLI_readWord();
int CLI_readInt();
float CLI_readFloat();
uint16_t CLI_readUInt16();
void printAvailableLegs();
void cliHelp();
void cliCalibrate();
void cliPwmServo();
void cliAngleServo();
void cliSetTrim();
void cliHal();
void cliLevel();
void cliDisplayState();
void cliRunServoCalibrate();
void cliDebug();

/**
 * Settings (settings.cpp)
 */
void initSettings();
void settingsPrint();
void settingsSave();
void settingsPrintTrim();
void settingsLoadTrim();
void settingsSaveTrim();

/**
 * Helpers (helpers.cpp)
 */
float radToDeg(float angleRad);
float degToRad(float angleDeg);
float mapf(float val, float in_min, float in_max, float out_min, float out_max);

#endif
