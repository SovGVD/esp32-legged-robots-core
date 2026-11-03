#ifndef MODEL_HEXAPOD_CONFIG_H
#define MODEL_HEXAPOD_CONFIG_H

/**
 * Hexapod robot configuration
 */

// default legs state
#define LEG_BODY_X      46.0
#define LEG_BODY_Y      80.0
#define LEG_BODY_Z       0.0
// Neutral feet 118 mm out from the coxa axis (same direction as before), 95 mm
// below it: femur 11.6 deg up, tibia ~15 deg off vertical. Taller than the old
// 160 / 67 mm stance and the femur servos carry far less torque, since the foot
// is nearly under the knee (checked in sim/, examples/hexapod_walk.py).
// Lower or with a higher knee doesn't fit: the femur goes at most 45 deg up
// (2:1 gear on a 180 deg servo), and from ~12 deg up a 30 mm step hits it.
#define LEG_FOOT_X    150.24
#define LEG_FOOT_Y    135.31
#define LEG_FOOT_Z    95.00

#define LEG_MIDDLE_BODY_X    57.5
#define LEG_MIDDLE_BODY_Y     0.0
#define LEG_MIDDLE_BODY_Z     0.0
#define LEG_MIDDLE_FOOT_X   175.5
#define LEG_MIDDLE_FOOT_Y     0.0
#define LEG_MIDDLE_FOOT_Z   95.00

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

/**
 * Not all servos are perfect
 *
 * This is for Noname MG90S
 * @TODO could be wrong, need test
 */

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

// I2C addresses can be overridden from platformio.ini build_flags,
// e.g. -DPWM_CONTROLLER_ADDRESS_2=0x44
#ifndef PWM_CONTROLLER_ADDRESS
	#define PWM_CONTROLLER_ADDRESS   0x40
#endif
// 18 servos need a second board, channels 16-31. 0x41 is taken by the INA219
// below, so the default for the second PCA9685 is 0x42 (A1 jumper).
#ifndef PWM_CONTROLLER_ADDRESS_2
	#define PWM_CONTROLLER_ADDRESS_2 0x42
#endif
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

#endif
