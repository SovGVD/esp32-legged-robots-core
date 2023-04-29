/**
 * Hexapod robot configuration
 */

// default legs state
#define LEG_BODY_X      46.0
#define LEG_BODY_Y      80.0
#define LEG_BODY_Z       0.0
#define LEG_FOOT_X    222.67
#define LEG_FOOT_Y    173.75
#define LEG_FOOT_Z     67.31

#define LEG_MIDDLE_BODY_X    57.5
#define LEG_MIDDLE_BODY_Y     0.0
#define LEG_MIDDLE_BODY_Z     0.0
#define LEG_MIDDLE_FOOT_X   267.5
#define LEG_MIDDLE_FOOT_Y     0.0
#define LEG_MIDDLE_FOOT_Z    67.31

// Robot config
#define LEG_NUM HEXAPOD_ROBOT
#define LEG_DOF 3

// ESP32 cores config
#define MOTOR_CORE   1	// @TODO same as i2c
#define DO_HAL_CORE  1


// PID
#define PID_LEVEL_P 0.01
#define PID_LEVEL_I 0.008
#define PID_LEVEL_D 0.3


// Size of leg parts in mm
#define LEG_SIZE_COXA   35.0
#define LEG_SIZE_FEMUR  55.0
#define LEG_SIZE_TIBIA 110.0


// Just human-friendly contstants
#define LEGLF 0   // left  front
#define LEGLM 1   // left  middle
#define LEGLR 2   // left  rear
#define LEGRF 3   // right front
#define LEGRM 4   // right middle
#define LEGRR 5   // right rear

// Balance settings
//#define HORIZON_LEVEL

/**
 * @TODO: set servo inverse and pins more suitable for body (avoid servo extentions)
 */
leg legs[LEG_NUM] = {
  // left front leg
  {
    { LEGLF, "LF" },                                              // ID: id, title
    {   -LEG_BODY_X,     LEG_BODY_Y,     LEG_BODY_Z },            // default LEG POSITION ON BODY
    {   -LEG_FOOT_X,     LEG_FOOT_Y,    -LEG_FOOT_Z },            // default LEG ON GROUND: x, y ,z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },            // Leg part sizes
    {             0,              0,              0 },            // MIN ANGLES FOR EACH JOINT: @TODO this is not in use
    {          M_PI,           M_PI,           M_PI },            // MAX ANGLES FOR EACH JOINT: @TODO this is not in use
    {        M_PI_2,         M_PI_2,         M_PI_2 },            // Calibration position
    {
      { M_PI_2, M_PI_2, M_PI_2 },                                 // hardware middle (looks like never used, or not?)
      {      0,      1,      2 },                                 // 3 servo channels on PCA board, if this is second board, then pin + 16
      {      0,      0,      0 },                                 // servo middle trim (this will be override anyway with setup trim... why do I need it???)
      {      2,      2,    1.5 }                                  // gear ratio, first and last one is 16 groves on servo and 24 groves on leg
    },
    {  -LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },                    // initial LEG POSITION ON BODY, same as LEG POSITION ON BODY, not sure why I need it
    {  -LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },                    // initial LEG JOING CLOSED TO THE GROUND, same as above, but at least it make sense
    {       M_PI_2,     M_PI_2,      M_PI_2 },                    // leg angles, will be override by HAL update
    { true, true, true },                                         // inverse leg calculations (depends on servo, is servo use gear and leg position): x, y, z and 6 angles
    { true, 0, 0 }                                                // FOOT SENSOR: is on ground state, pin, threshold for sensor
  },
  // left middle leg
  {
    { LEGLM, "LM" },
    { -LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y,  LEG_MIDDLE_BODY_Z },
    { -LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_MIDDLE_FOOT_Z },
    {      LEG_SIZE_COXA,    LEG_SIZE_FEMUR,     LEG_SIZE_TIBIA },
    {                  0,                 0,                  0 },
    {               M_PI,              M_PI,               M_PI },
    {             M_PI_2,            M_PI_2,             M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {      4,      5,      6 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    {  -LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y,  LEG_MIDDLE_BODY_Z },
    {  -LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_MIDDLE_FOOT_Z },
    {              M_PI_2,            M_PI_2,             M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },
  // left rear leg
  {
    { LEGLR, "LR" },
    {   -LEG_BODY_X,    -LEG_BODY_Y,     LEG_BODY_Z },
    {   -LEG_FOOT_X,    -LEG_FOOT_Y,    -LEG_FOOT_Z },
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    {             0,              0,              0 },
    {          M_PI,           M_PI,           M_PI },
    {        M_PI_2,         M_PI_2,         M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {      8,      9,     10 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    { -LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },
    { -LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },
    {       M_PI_2,    M_PI_2,      M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },

  // right front leg
  {
    { LEGRF, "RF" },
    {    LEG_BODY_X,     LEG_BODY_Y,     LEG_BODY_Z },
    {    LEG_FOOT_X,     LEG_FOOT_Y,    -LEG_FOOT_Z },
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    {             0,              0,              0 },
    {          M_PI,           M_PI,           M_PI },
    {        M_PI_2,         M_PI_2,         M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {     16,     17,     18 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    {  LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },
    {  LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },
    {      M_PI_2,     M_PI_2,      M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },
  // right middle leg
  {
    { LEGRM, "RM" },
    { LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y,  LEG_MIDDLE_BODY_Z },
    { LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_MIDDLE_FOOT_Z },
    {     LEG_SIZE_COXA,    LEG_SIZE_FEMUR,     LEG_SIZE_TIBIA },
    {                 0,                 0,                  0 },
    {              M_PI,              M_PI,               M_PI },
    {            M_PI_2,            M_PI_2,             M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {     20,     21,     22 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    {  LEG_MIDDLE_BODY_X, LEG_MIDDLE_BODY_Y,  LEG_MIDDLE_BODY_Z },
    {  LEG_MIDDLE_FOOT_X, LEG_MIDDLE_FOOT_Y, -LEG_MIDDLE_FOOT_Z },
    {             M_PI_2,            M_PI_2,             M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },
  // right rear leg
  {
    { LEGRR, "RR" },
    {    LEG_BODY_X,    -LEG_BODY_Y,     LEG_BODY_Z },
    {    LEG_FOOT_X,    -LEG_FOOT_Y,    -LEG_FOOT_Z },
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },
    {             0,              0,              0 },
    {          M_PI,           M_PI,           M_PI },
    {        M_PI_2,         M_PI_2,         M_PI_2 },
    {
      { M_PI_2, M_PI_2, M_PI_2 },
      {     14,     15,     16 },
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    { LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },
    { LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },
    {      M_PI_2,    M_PI_2,      M_PI_2 },
    { true, true, true },
    { true, 0, 0 }
  },
};

/**
 * Not all servos are perfect
 *
 * This is for Noname MG90S
 * @TODO could be wrong, need test
 */
servoProfile servoMainProfile = {
    0,    // minAngle
    180,  // maxAngle

    590,  // degMin
    870,  // deg30
    1080, // deg50
    1290, // deg70
    1500, // deg90 - middle
    1710, // deg110
    1890, // deg130
    2100, // deg150
    2200  // degMax
};

/**
 * Hardware
 */
// @TODO temporary disabled just to not thing about it right now
//#define ESP32CAMERA
//#define ESP32CAMERA_FRAMESIZE    FRAMESIZE_QVGA
//#define ESP32CAMERA_JPEG_QUALITY 12
//#define ESP32CAMERA_FPS          25

#define I2C_SDA 3
#define I2C_SCL 13

#define PWM_CONTROLLER_ADDRESS   0x40
#define PWM_CONTROLLER_TYPE      PCA9685

#define IMU_ADDRESS              0x68
#define IMU_TYPE                 MPU9250

#define POWER_SENSOR_ADDRESS     0x41
#define POWER_SENSOR             INA219

#define IGNORE_BROWNOUT_CONDITION

//Cores loop time
#define LOOP_TIME 7000   // microseconds (10^-6 seconds)
#define SERVICE_LOOP_TIME 100000
#define SERVICE_FAST_LOOP_TIME 7000

#include "../../libraries/IK/IK.h"
#include "../../libraries/MODEL_hexapod/IK/IK.cpp"
