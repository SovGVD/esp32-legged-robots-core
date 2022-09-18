/**
 * Tank rover
 */
// position of tracks
#define TRACK_BODY_X      80.0
#define TRACK_BODY_Y       0.0
#define TRACK_BODY_Z       0.0
#define TRACK_POINT_X     80.0
#define TRACK_POINT_Y      0.0
#define TRACK_POINT_Z      0.0

// Robot config
#define LEG_NUM TANK_ROBOT
#define LEG_DOF 2 // 2 pins for DC motor controller

// ESP32 cores config
#define MOTOR_CORE  1
#define DO_HAL_CORE 1

// PID
#define PID_LEVEL_P 0.01
#define PID_LEVEL_I 0.008
#define PID_LEVEL_D 0.3

// Track length
#define TRACK_LENGTH 164.0
#define TRACK_WIDTH   63.0


// Just human-friendly contstants
#define TRACKL 0   // left  track
#define TRACKR 1   // right track

// Balance settings
//#define HORIZON_LEVEL


leg legs[LEG_NUM] = {
  // left track
  {
    { TRACKL, "LEFT" },                              // ID: id, title
    {  -TRACK_BODY_X,    TRACK_BODY_Y,   TRACK_BODY_Z }, // place of middle of the track mounted to body
    { -TRACK_POINT_X,   TRACK_POINT_Y,  TRACK_POINT_Z }, // middle of the track, so same
    {              0,    TRACK_LENGTH,    TRACK_WIDTH },
    {    0,    0 },                                  // min power
    { M_PI, M_PI },                                  // max power, yes, weird, but basically we are going to use PWM anyway
    {    0,    0 },                                  // not used
    {
      { M_PI_2, M_PI_2 },                            // power middle (this needs to be removed everywhere)
      {     27,     26 },                            // 2 pins of DC motor control board, like L298N IN1...IN4
      {      0,      0 },                            // trim, for DC motor tank robot is it minimal power to start
      {      1,      1 }                             // not used
    },
    { 0, 0, 0 },                                     // not used
    { 0, 0, 0 },                                     // not used
    { 0, 0 },                                        // initital speed
    { false, false, false, false, false },           // not used, should be all false
    { true, 0, 0 }                                   // not used
  },
  // right track
  {
    { TRACKR, "RIGHT" },
    {  TRACK_BODY_X,    TRACK_BODY_Y,   TRACK_BODY_Z },
    { TRACK_POINT_X,   TRACK_POINT_Y,  TRACK_POINT_Z },
    {             0,    TRACK_LENGTH,    TRACK_WIDTH },
    {    0,    0 },
    { M_PI, M_PI },
    {    0,    0 },
    {
      { M_PI_2, M_PI_2 },
      {     33,     25 },
      {      0,      0 },
      {      1,      1 }
    },
    { 0, 0, 0 },
    { 0, 0, 0 },
    { 0, 0 },
    { false, false, false, false, false },
    { true, 0, 0 }
  }
};

/**
 * It is called servo, but as DC motor driver use 8bit (0 to 255) PWM anyway, so this-is-fine
 * 0 to 180 deg is just mapped/interpolate to 0 to 255
 */
servoProfile servoMainProfile = {
    0,    // minAngle
    180,  // maxAngle

    0,   // degMin
    42,  // deg30
    71,  // deg50
    99,  // deg70
    127, // deg90 - middle
    156, // deg110
    184, // deg130
    212, // deg150
    255  // degMax
};
 
/**
 * Hardware
 */
#define PWM_CONTROLLER_TYPE  DCMOTOR

#define IMU_TYPE             MPU9250
#define IMU_ADDRESS          0x68

#define POWER_SENSOR         INA219
#define POWER_SENSOR_ADDRESS 0x40

//Cores loop time
#define LOOP_TIME 2000   // microseconds (10^-6 seconds)
#define SERVICE_LOOP_TIME 100000
#define SERVICE_FAST_LOOP_TIME 5000

#include "../../libraries/IK/IK.h"
#include "../../libraries/MODEL_tank/IK/IK.cpp"
