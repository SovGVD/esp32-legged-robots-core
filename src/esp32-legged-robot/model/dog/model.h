/**
 * Specific for robot config
 * https://www.instructables.com/ESP32-Small-Robot-Dog/
 */
// default legs state (TODO measure it!!!)
#define LEG_BODY_X      39.5
#define LEG_BODY_Y_F    70.5  // not sure
#define LEG_BODY_Y_H    70.5
#define LEG_BODY_Z       0.0
#define LEG_POINT_X     56.0
#define LEG_POINT_Y_F   70.5 // not sure
#define LEG_POINT_Y_H   70.5
#define LEG_POINT_Z     80.0

// Robot config
#define LEG_NUM QUADRUPED_ROBOT
#define LEG_DOF 3

// ESP32 cores config
#define SERVO_CORE   1
#define DO_HAL_CORE  1

// PID
#define PID_LEVEL_P 0.01
#define PID_LEVEL_I 0.008
#define PID_LEVEL_D 0.3

// Size of leg parts in mm
#define LEG_SIZE_L1   0.0
#define LEG_SIZE_L2  51.9
#define LEG_SIZE_L3  50.0


// Just human-friendly contstants
#define LEGLF 0   // left  front leg
#define LEGRF 1   // right front leg
#define LEGLH 2   // left  hind  leg
#define LEGRH 3   // right hind  leg

// Balance settings
//#define HORIZON_LEVEL

LR_point balanceOffset = {0, 0, 0};

leg legs[LEG_NUM] = {
  // LEFT FRONT
  {
    {LEGLF, "LF"},
    { -LEG_BODY_X,  LEG_BODY_Y_F,   LEG_BODY_Z},
    {-LEG_POINT_X, LEG_POINT_Y_F, -LEG_POINT_Z},
    { LEG_SIZE_L1,   LEG_SIZE_L2,  LEG_SIZE_L3},
    {    0,    0,    0 },
    { M_PI, M_PI, M_PI },
    {
      { M_PI_2, M_PI_2, M_PI_2 },             // hardware middle
      {     25,     26,     27 },             // 3 servo pins
      {      0,      0,      0 },             // servo middle trim
      {      2,      1,      1 }              // gear ratio, first and last one is 16 groves on servo and 24 groves on leg
    },
    {  -LEG_BODY_X,  LEG_BODY_Y_F,   LEG_BODY_Z },
    { -LEG_POINT_X, LEG_POINT_Y_F, -LEG_POINT_Z },
    {       M_PI_2,        M_PI_2,       M_PI_2 },
    {true, true, false, false, true, true},
    {true, 0, 0}
  },
  // RIGHT FRONT
  {
    {LEGRF, "RF"},
    { LEG_BODY_X,  LEG_BODY_Y_F,   LEG_BODY_Z},
    {LEG_POINT_X, LEG_POINT_Y_F, -LEG_POINT_Z},
    {LEG_SIZE_L1,   LEG_SIZE_L2,  LEG_SIZE_L3},
    {0, 0, 0},
    {M_PI, M_PI, M_PI},
    {
      {M_PI_2, M_PI_2, M_PI_2},
      { 16, 18, 17},
      {  0,  0,  0},
      {  2,  1,  1}
    },
    {     LEG_BODY_X,   LEG_BODY_Y_F,      LEG_BODY_Z},
    {    LEG_POINT_X,  LEG_POINT_Y_F,    -LEG_POINT_Z},
    {M_PI_2, M_PI_2, M_PI_2},
    {false, true, false, true, false, false},
    {true, 0, 0}
  },
  // LEFT HIND
  {
    {LEGLH, "LH"},
    { -LEG_BODY_X,  -LEG_BODY_Y_H,   LEG_BODY_Z},
    {-LEG_POINT_X, -LEG_POINT_Y_H, -LEG_POINT_Z},
    { LEG_SIZE_L1,    LEG_SIZE_L2,  LEG_SIZE_L3},
    {0, 0, 0},
    {M_PI, M_PI, M_PI},
    {
      {M_PI_2, M_PI_2, M_PI_2},
      { 13, 12, 14},
      {  0,  0,  0},
      {  2,  1,  1}
    },
    {    -LEG_BODY_X,  -LEG_BODY_Y_H,      LEG_BODY_Z},
    {   -LEG_POINT_X, -LEG_POINT_Y_H,    -LEG_POINT_Z},
    {M_PI_2, M_PI_2, M_PI_2},
    {true, true, false, true, true, true},
    {true, 0, 0}
  },
  // RIGHT HIND
  {
    {LEGRH, "RH"},
    { LEG_BODY_X,  -LEG_BODY_Y_H,   LEG_BODY_Z},
    {LEG_POINT_X, -LEG_POINT_Y_H, -LEG_POINT_Z},
    {LEG_SIZE_L1,    LEG_SIZE_L2,  LEG_SIZE_L3},
    {0, 0, 0},
    {M_PI, M_PI, M_PI},
    {
      {M_PI_2, M_PI_2, M_PI_2},
      {  4,  2, 15},
      {  0,  0,  0},
      {  2,  1,  1}
    },
    {     LEG_BODY_X,  -LEG_BODY_Y_H,      LEG_BODY_Z},
    {    LEG_POINT_X, -LEG_POINT_Y_H,    -LEG_POINT_Z},
    {M_PI_2, M_PI_2, M_PI_2},
    {false, true, false, false, false, false},
    {true, 0, 0}
  }
};

/**
 * Not all servos are perfect
 * 
 * This is for TowerPro MG90D (or maybe I have clone, who knows)
 */
servoProfile servoMainProfile = {
    20,   // minAngle
    160,  // maxAngle
    780,  // degMin
    890,  // deg30
    1090, // deg50
    1300, // deg70
    1500, // deg90 - middle
    1700, // deg110
    1900, // deg130
    2110, // deg150
    2200  // degMax
};

/**
 * My Trim settings
 * 0 leg trim {-11.000790, -3.093972, 2.979381}
 * 1 leg trim {-0.572958, 4.927437, -3.093972}
 * 2 leg trim {9.969466, 0.000000, 2.979381}
 * 3 leg trim {4.927437, 0.000000, -7.104677}
 */
 
/**
 * Hardware
 */
#define PWM_CONTROLLER_TYPE  ESP32PWM
#define IMU_ADDRESS          0x68
#define IMU_TYPE             MPU9250
#define POWER_SENSOR         INA219


//Cores loop time
#define LOOP_TIME 4400   // microseconds (10^-6 seconds)
#define SERVICE_LOOP_TIME 100000
#define SERVICE_FAST_LOOP_TIME 5000


#include "../../libraries/MODEL_dog/IK/IK.h"
#include "../../libraries/MODEL_dog/IK/IK.cpp"
