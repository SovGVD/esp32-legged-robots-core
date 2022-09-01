/**
 * Some usuful defines
 */

#define DISPLAY_DIGITS 1  // Number of digits to display after the decimal point


#define PCA9685  100
#define MPU9650  101
#define SD1306   102
#define ESP32PWM 103
#define INA219   104

/**
 * Robot models
 */
#define MODEL_DOG   100
#define MODEL_BIPED 101

/**
 * Number of legs
 */
#define BIPEDAL_ROBOT   2
#define QUADRUPED_ROBOT 4

/**
 * Axis
 */
#define ROLL  0
#define PITCH 1
#define YAW   2

#define SOFTWARE_CORE_VERSION 1

/* Math */
#define RADTODEG 180/PI
#define DEGTORAD PI/180

/* WiFi */
#define AP_MODE 0

/* Packages code */
#define P_TELEMETRY_LEN 14

#define P_MOVE         77  // ASCII "M"
#define P_TELEMETRY    84  //       "T"
#define P_START_VIDEO  86  //       "V" (capital V)
#define P_STOP_VIDEO  118  //       "v" (small V)
#define P_CLI          67  //       "C"

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

typedef struct {
  uint8_t minAngle; // minimal servo angle, deg
  uint8_t maxAngle; // maximal servo angle, deg
  uint16_t degMin;  // PWM pulse at minAngle deg
  uint16_t deg30;   // PWM pulse at 30 deg
  uint16_t deg50;
  uint16_t deg70;
  uint16_t deg90;   // PWM pusle at 90 deg
  uint16_t deg110;
  uint16_t deg130;
  uint16_t deg150;  // PWM pulse at 150 deg
  uint16_t degMax;  // PWM pusle at maxAngle deg
} servoProfile;
