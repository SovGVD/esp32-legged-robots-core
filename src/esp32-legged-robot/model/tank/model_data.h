/**
 * Tank description: legs, servo calibration and gait. Only types from
 * lib/robot_core and macros from def.h + config.h, no Arduino, so the same
 * data can be compiled into the firmware (model.cpp), native tests and the
 * simulator bridge. Include it from exactly one .cpp file per program.
 */
#ifndef MODEL_TANK_MODEL_DATA_H
#define MODEL_TANK_MODEL_DATA_H

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

// Tracks are driven directly from the move command, no gait
const GaitParams GAIT_PARAMS = {
	false,    // enabled
	1, 0.5, { 0, 0, 0, 0, 0, 0 }, 0, 0, 0, 0, 0, 0, false
};

#endif
