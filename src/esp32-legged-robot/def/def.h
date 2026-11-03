#ifndef DEF_DEF_H
#define DEF_DEF_H

#include <stdint.h>

/**
 * Some usuful defines
 */

#define DISPLAY_DIGITS 1  // Number of digits to display after the decimal point


#define PCA9685  100
#define MPU9250  101
#define MPU9650  MPU9250   // old misspelling
#define SD1306   102
#define ESP32PWM 103
#define INA219   104
#define DCMOTOR  105
#define VL53L5CX 106

/**
 * Robot models
 */
#define MODEL_DOG     100
#define MODEL_BIPED   101
#define MODEL_TANK    102
#define MODEL_HEXAPOD 103

/**
 * Number of legs/tracks
 */
#define BIPEDAL_ROBOT   2
#define QUADRUPED_ROBOT 4
#define TANK_ROBOT      2
#define HEXAPOD_ROBOT   6

/**
 * Axis
 */
#define ROLL  0
#define PITCH 1
#define YAW   2

#define SOFTWARE_CORE_VERSION 1

/* Math */
#define RADTODEG 57.295779513f   // 180/PI
#define DEGTORAD 0.0174532925f  // PI/180

/* WiFi */
#define AP_MODE 0

/* Packages code */
#define P_TELEMETRY_LEN 26
#define P_TELEMETRY_VERSION 3

#define P_MOVE         77  // ASCII "M"
#define P_TELEMETRY    84  //       "T"
#define P_START_VIDEO  86  //       "V" (capital V)
#define P_STOP_VIDEO  118  //       "v" (small V)
#define P_CLI          67  //       "C"
#define P_GAIT         71  //       "G" [id, walk]: 1 = statically stable walk, 0 = default gait
#define P_LEVEL        76  //       "L" [id, on]: IMU leveling on/off

/* Telemetry status bits */
#define TS_HAL          0x0001  // leg calculation on (`hal on`)
#define TS_FAILSAFE     0x0002  // no commands from the client, stopped
#define TS_GAIT_ACTIVE  0x0004  // stepping
#define TS_GAIT_STATIC  0x0008  // statically stable gait (walk), else trot
#define TS_LEVELING     0x0010  // IMU leveling on
#define TS_IMU          0x0020  // IMU found
#define TS_POWER        0x0040  // power sensor found
#define TS_BATTERY_SHIFT 7      // 2 bits: BatteryState (0 unknown, 1 ok, 2 low, 3 critical)
#define TS_SITTING      0x0200  // battery critical: sat down / servos off
#define TS_OVERRUN      0x0400  // the motion loop missed a tick in the last second
#define TS_TERRAIN      0x0800  // terrain mode: ToF map and rough gait (`terrain on`)
#define TS_GAP_BLOCKED  0x1000  // a gap too wide to step over: stopped in that direction
#define TS_HEADING_HOLD 0x2000  // heading hold on (`heading on`, `turnto`)
#define TS_HEADING_ABS  0x4000  // heading from the magnetometer (else gyro only, relative)
#define TS_TOF          0x8000  // ToF sensor(s) running

/* Debug (SLOW!!!)*/
//#define DEBUG_HAL_LEG

/**
 * Hardware option titles
 */
#define TITLE_SERVO "servo"
#define TITLE_IMU   "imu"
#define TITLE_LEG   "leg"

/**
 * Other titles
 */
#define TITLE_ON    "on"
#define TITLE_OFF   "off"
#define TITLE_STATE "state"

#define WEB_SERVER_VERSION "0.0.1"

// servoProfile (servo calibration table) is defined in lib/robot_core/src/servo_hal.h

#endif
