/**
 * Hexapod description: legs, servo calibration and gait. Only types from
 * lib/robot_core and macros from def.h + config.h, no Arduino, so the same
 * data can be compiled into the firmware (model.cpp), native tests and the
 * simulator bridge. Include it from exactly one .cpp file per program.
 */
#ifndef MODEL_HEXAPOD_MODEL_DATA_H
#define MODEL_HEXAPOD_MODEL_DATA_H

leg legs[LEG_NUM] = {
  // left front leg
  {
    { LEGLF, "LF" },                                              // ID: id, title
    {   -LEG_BODY_X,     LEG_BODY_Y,     LEG_BODY_Z },            // default LEG POSITION ON BODY
    {   -LEG_FOOT_X,     LEG_FOOT_Y,    -LEG_FOOT_Z },            // default LEG ON GROUND: x, y ,z
    { LEG_SIZE_COXA, LEG_SIZE_FEMUR, LEG_SIZE_TIBIA },            // Leg part sizes
    {             0,              0,              0 },            // MIN ANGLES FOR EACH JOINT (enforced by MotionController)
    {          M_PI,           M_PI,           M_PI },            // MAX ANGLES FOR EACH JOINT (enforced by MotionController)
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
    { true, true, true, false, false, true },                     // inverse x, y, z, alpha, beta, gamma. Right legs are mirror images of the left (gears face forward)
                                                                  // and every gear pair reverses, so left: tibia inverted, right: femur inverted. Worked out from the parts, check on the robot
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
    { true, true, true, false, false, true },                     // inverse x, y, z, alpha, beta, gamma: tibia servo mirrored vs the right legs
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
    { true, true, true, false, false, true },                     // inverse x, y, z, alpha, beta, gamma: tibia servo mirrored vs the right legs
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
    { true, true, true, false, true, false },                     // inverse x, y, z, alpha, beta, gamma: femur servo mirrored vs the left legs
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
    { true, true, true, false, true, false },                     // inverse x, y, z, alpha, beta, gamma: femur servo mirrored vs the left legs
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
      {     24,     25,     26 },                                 // second board, channels 8-10 (mirrors left rear)
      {      0,      0,      0 },
      {      2,      2,    1.5 }
    },
    { LEG_BODY_X, LEG_BODY_Y,  LEG_BODY_Z },
    { LEG_FOOT_X, LEG_FOOT_Y, -LEG_FOOT_Z },
    {      M_PI_2,    M_PI_2,      M_PI_2 },
    { true, true, true, false, true, false },                     // inverse x, y, z, alpha, beta, gamma: femur servo mirrored vs the left legs
    { true, 0, 0 }
  },
};

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

const GaitParams GAIT_PARAMS = {
	true,     // enabled
	0.4,      // cycle time, s
	0.75,     // duty factor
	{ 0.5, 0.0, 0.5, 0.0, 0.5, 0.0 },   // phase offsets LF, LM, LR, RF, RM, RR (LF, LR, RM swing together)
	30,       // step height, mm
	75,       // max speed, mm/s (sideways and turning beyond this the 2:1 coxa servos run out of range)
	0.75,     // max yaw rate, rad/s
	600,      // max acceleration, mm/s^2
	6,        // max yaw acceleration, rad/s^2
	3,        // idle tolerance, mm
	false     // no foot contact sensors
};

// IMU leveling (HORIZON_LEVEL or CLI `level on`), not tuned for this model yet:
// the dog's values (tuned in the simulator) as a starting point
const LevelerParams LEVEL_PARAMS = {
	0.0,   // kp
	1.0,   // ki, 1/s
	0.0,   // kd, s
	0.25,  // limit, rad
	0.5,   // rate, rad/s
	0.0,   // IMU low-pass, s
	0.026  // deadband, rad
};

// Battery protection (motion.cpp): warn below lowVoltage, below
// criticalVoltage stop, sit down by SIT_HEIGHT and turn the servos off.
// Defaults are for a 2S LiPo (3.4 / 3.2 V per cell); change for your pack.
const BatteryParams BATTERY_PARAMS = {
	6.8,   // lowVoltage, V
	6.4,   // criticalVoltage, V
	2.0,   // filter, s (servo current makes the voltage sag every step)
	3.0    // holdTime, s
};

// Body lowered this much (mm) to sit down when the battery is empty
const float SIT_HEIGHT = -30;

#endif
