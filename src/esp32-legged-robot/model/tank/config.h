#ifndef MODEL_TANK_CONFIG_H
#define MODEL_TANK_CONFIG_H

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



/**
 * It is called servo, but as DC motor driver use 8bit (0 to 255) PWM anyway, so this-is-fine
 * 0 to 180 deg is just mapped/interpolate to 0 to 255
 */
 
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

#endif
