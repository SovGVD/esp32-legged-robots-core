/**
 * Bipedal robot configuration
 */

/**
 * LEFT side view
 *
 *  <- front
 * 
 *   |        |
 *   +--------*
 *       |
 *       o               <- 3DOF (alpha, beta, gamma), LEG_BODY_(X,Y,Z)
 *        \
 *         \  L1
 *          o            <- 1DOF (delta)
 *         /
 *        / L2
 *       o               <- 2 DOF (epsilon, zeta), LEG_FOOT_(X,Y,Z)
 *       | L3            <- from joint to the ground
 *    -------
 *    / / / /
 *                    ^ z
 *                    |
 *               y <--*
 */

// default legs state
#define LEG_BODY_X      60.5
#define LEG_BODY_Y       0.0
#define LEG_BODY_Z       0.0
#define LEG_POINT_X     60.5
#define LEG_POINT_Y      0.0
#define LEG_POINT_Z    141.42		// sqrt(L1^2+L2^2) (straight leg on ground, but distance only between joints, not to the ground, so 90deg between L1 and L2)

// Robot config
#define LEG_NUM BIPEDAL_ROBOT
#define LEG_DOF 6

// PID
#define PID_LEVEL_P 0.01
#define PID_LEVEL_I 0.008
#define PID_LEVEL_D 0.3


// Size of leg parts in mm
#define LEG_SIZE_L1  100.0
#define LEG_SIZE_L2  100.0
#define LEG_SIZE_L3   40.0


// Just human-friendly contstants
#define LEGL 0   // left  leg
#define LEGR 1   // right leg

LR_point balanceOffset = {0, 0, 0};

leg legs[LEG_NUM] = {
  // left leg
  {
    { LEGL, "LEFT" },                                                                     // ID: id, title
    {  -LEG_BODY_X,    LEG_BODY_Y,   LEG_BODY_Z },                                        // default LEG POSITION ON BODY
    { -LEG_POINT_X,   LEG_POINT_Y, -LEG_POINT_Z },                                        // default LEG JOING CLOSED TO THE GROUND: x, y ,z
    {  LEG_SIZE_L1,   LEG_SIZE_L2,  LEG_SIZE_L3 },                                        // LEG SIZES (that we need):
    {    0,    0,    0,    0,    0,    0 },                                               // MIN ANGLES FOR EACH JOINT: @TODO this is not in use
    { M_PI, M_PI, M_PI, M_PI, M_PI, M_PI },                                               // MAX ANGLES FOR EACH JOINT: @TODO this is not in use
    {
      { M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2 },                                 // hardware middle
      {      0,      1,      2,      3,      4,      5 },                                 // 6 servo channels
      {      0,      0,      0,      0,      0,      0 },                                 // servo middle trim (this will be override anyway with setup trim... why do I need it???)
      {      1,      1,      1,      1,      1,      1 }                                  // gear ratio, first and last one is 16 groves on servo and 24 groves on leg
    },
    {  -LEG_BODY_X,  LEG_BODY_Y,   LEG_BODY_Z },                                          // initial LEG POSITION ON BODY, same as LEG POSITION ON BODY, not sure why I need it
    { -LEG_POINT_X, LEG_POINT_Y, -LEG_POINT_Z },                                          // initial LEG JOING CLOSED TO THE GROUND, same as above, but at least it make sense
    { M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2 },                                   // leg angles, will be override by HAL update
    { true, true, false, false, true, true, false, false, false },                        // inverse leg calculations (depends on servo, is servo use gear and leg position): x, y, z and 6 angles
    { true, 0, 0 }                                                                        // FOOT SENSOR: is on ground state, pin, threshold for sensor
  },
  // right leg
  {
    { LEGR, "RIGHT" },
    {  LEG_BODY_X,  LEG_BODY_Y,   LEG_BODY_Z },
    { LEG_POINT_X, LEG_POINT_Y, -LEG_POINT_Z },
    { LEG_SIZE_L1, LEG_SIZE_L2,  LEG_SIZE_L3 },
    {    0,    0,    0,    0,    0,    0 },
    { M_PI, M_PI, M_PI, M_PI, M_PI, M_PI },
    {
      { M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2 },
      {     15,     14,     13,     12,     11,     10 },
      {      0,      0,      0,      0,      0,      0 },
      {      1,      1,      1,      1,      1,      1 }
    },
    {  LEG_BODY_X,  LEG_BODY_Y,   LEG_BODY_Z },
    { LEG_POINT_X, LEG_POINT_Y, -LEG_POINT_Z },
    { M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2, M_PI_2 },
    { false, true, false, true, false, false, true, true, true },
    { true, 0, 0 }
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
 * Hardware
 */
#define ESP32CAMERA
#define ESP32CAMERA_FRAMESIZE    FRAMESIZE_QVGA
#define ESP32CAMERA_JPEG_QUALITY 12
#define ESP32CAMERA_FPS          25

#define PWM_CONTROLLER_ADDRESS   0x40
#define PWM_CONTROLLER_TYPE      PCA9685

#define IMU_ADDRESS              0x68
#define IMU_TYPE                 MPU9250

#define POWER_SENSOR             INA219


#include "../../libraries/MODEL_biped/IK/IK.h"
#include "../../libraries/MODEL_biped/IK/IK.cpp"
