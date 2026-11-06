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
extern const GaitParams *const MODEL_WALK_GAIT;   // statically stable walk, nullptr if the model has only GAIT_PARAMS

// Rough ground (model_data.h TOF_SENSORS, TERRAIN_*): tofCount 0 = not supported
struct ModelTerrain {
	const TofSensor        *tof;
	uint8_t                 tofCount;
	const TerrainMapParams *map;
	const TerrainParams    *terrain;
	const GaitParams       *roughGait;
	TerrainTiltMode         tilt;
};
extern const ModelTerrain MODEL_TERRAIN;

// Lidar mapping and navigation (model_data.h LIDAR_*): all nullptr = not supported
struct ModelLidar {
	const LidarParams    *lidar;
	const LidarMapParams *map;
	const NavParams      *nav;
	const TofScanParams  *tofScan;   // ToF room scans (no LIDAR_TYPE), nullptr = not supported
	const LidarMapParams *tofMap;
};
extern const ModelLidar MODEL_LIDAR;
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
 * Motion loop profiling (main.cpp): time spent in each part of a motion tick,
 * to find what makes the loop overrun. CLI `loop` prints it, the serial port
 * prints the slowest tick of every second that missed ticks.
 */
enum LoopStage : uint8_t {
	LOOP_STAGE_FAILSAFE = 0,
	LOOP_STAGE_POWER,      // INA219 + battery (I2C, 10 Hz)
	LOOP_STAGE_IMU,        // MPU9250 read (I2C), part of MOTION
	LOOP_STAGE_TOF,        // ToF frames into the terrain map, part of MOTION
	LOOP_STAGE_MOTION,     // leveler, gait, body kinematics, IK (includes IMU and ToF)
	LOOP_STAGE_HAL,        // pulses -> servo bus (LEDC / PCA9685 over I2C)
	LOOP_STAGE_COUNT
};
void loopProfileMark(LoopStage stage, uint32_t us);   // from the motion task only
void cliLoop();

/**
 * Motion (motion.cpp): gait, body kinematics, IK
 */
void initMotion();
void updateMotion(float dt);   // dt: seconds since the previous motion tick (wall clock)
const LR_figure &motionBody();
void requestGait(const GaitParams &params);   // applied once the robot stands still
void printGaitState();                        // CLI: active gait parameters
void cliTerrain();                            // CLI: terrain on|off|state
void cliHeading();                            // CLI: heading on|off|state|cal|hand
void cliTurnTo();                             // CLI: turnto <deg>
bool motionUsesContacts();                    // the gait runs on the foot switches now

/**
 * Foot contact switches on GPIO (feet.cpp, FOOT_SWITCH_PINS in config.h)
 */
void initFeet();
void updateFeet(leg *legsOut, uint8_t legCount, bool apply);   // motion task, every tick
bool footSwitchesFitted();                    // FOOT_SWITCH_PINS set
bool footSwitchesInUse();                     // fitted and `feet on`
void cliFeet();                               // CLI: feet on|off|state
FootInput &footInput();                       // GPIO switches (feet.cpp), or the simulated ones (HIL)
// Odometry for the lidar / ToF room scans (motion.cpp): gait velocity turned by the gyro, any task
struct MotionOdometry {
	LR_pose2 pose;         // mm, rad (counter-clockwise), since boot
	float    maxYawRate;   // rad/s of the current gait at yaw command 1
	LR_angle tilt;         // body tilt now (IMU, firmware convention: roll = nose up)
	float    floorZ;       // mm, the floor below the body origin (< 0)
	bool     gaitActive;   // the legs are stepping
	bool     gapBlocked;   // terrain mode: a gap too wide ahead stopped the robot (GaitEngine::isBlocked)
};
MotionOdometry motionOdometry();
float motionOdometryBias();   // rad/s, gyro bias the odometry learned while standing

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
	bool     terrain;        // terrain mode on
	bool     gapBlocked;     // a gap too wide ahead: stopped in that direction
	bool     headingHold;
	bool     headingTarget;  // the hold has a target (turnto, or the heading when the stick was let go)
	float    target;         // rad, compass
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
// ToF (tof.cpp), VL53L5CX with TOF_TYPE
TofArray &tofArray();   // the VL53L5CX driver (tof.cpp), or the simulated sensors (HIL)
void    initTof();
uint8_t tofSensorCount();
bool    tofReady(uint8_t i);
bool    tofTakeFrame(uint8_t i, uint32_t &seq, float *mm);   // true and the 64 zones (mm, 0 = none) if newer than seq
bool    tofPeekFrame(uint8_t i, uint32_t &seq, int16_t *mm);   // newest frame (mm, 0 = none), any task; false if not running
uint16_t tofMaxRange();   // mm, sensor 0 (MODEL_TERRAIN.tof), 0 without ToF
void    cliTof();
// Indoor map, localization, walking to a point (lidar.cpp): LD06 / LD19 lidar with
// LIDAR_TYPE, else ToF room scans (TOF_SCAN_PARAMS and a VL53L5CX)
// (and a model with LIDAR_PARAMS / TOF_SCAN_PARAMS: MODEL_LIDAR, checked at run time)
#if defined(LIDAR_TYPE) && LIDAR_TYPE == LD06
	#define NAV_LIDAR 1
#elif defined(TOF_TYPE) && TOF_TYPE == VL53L5CX
	#define NAV_TOF 1
#endif
void initLidar();
bool lidarNavCommand(LR_moveVector &cmd);   // motion task: the navigator's command while it walks to a goal (or scans)
bool lidarNavBody(float &tilt, float &crouch);   // motion task: ToF scan nose up (rad) and body lowered (mm); false: no ToF scans
void lidarNavCancel();                      // motion task: the operator took over
size_t pMap(uint8_t *out, size_t room, uint8_t part);   // P_MAP package (packages.cpp format), 0: no map
size_t pNav(uint8_t *out, size_t room);                 // P_NAV package
bool lidarGoto(float x, float y);           // map frame, mm
void lidarStop();
bool lidarScan();                           // ToF: one room scan now (false: not ToF / no map)
void cliLidar();
void cliGoto();
void initPowerSensor();
void updatePower();
float getPowerSensorVoltage();
float getPowerSensorCurrent();

/**
 * Hardware in the loop (hil.cpp, -DHIL): the devices are simulated on the PC
 */
#ifdef HIL
void initHil();                     // after Serial.begin()
void hilReceive();                  // motion task, start of a tick: sensor / ToF / move frames from the PC
void hilSend(uint32_t dtUs);        // motion task, end of a tick: the servo pulses
// newest simulated sensors; newMag: the magnetometer reading is new (takeMag: and now it is taken)
bool hilSensors(HilSensors &out, bool &newMag, bool takeMag = true);
#endif
void cliHil();                      // CLI: hil (link state)

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
void pGait(uint8_t *data, size_t len);
void pLevel(uint8_t *data, size_t len);
void pGoto(uint8_t *data, size_t len);
size_t pTof(uint8_t *out);   // fills a P_TOF package, returns its length (0: no ToF on this model)

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
