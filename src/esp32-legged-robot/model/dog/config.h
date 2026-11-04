#ifndef MODEL_DOG_CONFIG_H
#define MODEL_DOG_CONFIG_H

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
#define MOTOR_CORE   1
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


/**
 * Not all servos are perfect
 * 
 * This is for TowerPro MG90D (or maybe I have clone, who knows)
 */

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
// GY-91 flat on the top board, pin header to the front. The MPU9250 sits
// rotated so its X axis points backward and Y to the right (from the chip's
// pin 1 mark), so sensor X is the firmware pitch axis, reversed.
// Check with `level state`: lift the left side -> pitch +, lift the nose -> roll +.
#define IMU_SWAP_ROLL_PITCH  true
#define IMU_PITCH_SIGN       -1

#define POWER_SENSOR         INA219
#define POWER_SENSOR_ADDRESS 0x40

// ToF terrain sensor: one VL53L5CX (8x8) on the nose, 40 deg down (TOF_SENSORS
// in model_data.h). On the main I2C bus (21/22) at its default address 0x29,
// LPn tied high. For its own bus (faster, no waiting on the IMU) wire it to
// two free pins and set them here, e.g. 32 / 33.
#define TOF_TYPE             VL53L5CX
//#define TOF_I2C_SDA          32
//#define TOF_I2C_SCL          33
// zone order, check with `tof print` (see tof.cpp)
//#define TOF_ZONE_MIRROR_X    1
//#define TOF_ZONE_MIRROR_Y    0

// 2D lidar (LD06 / LD19, LIDAR_PARAMS in model_data.h): its TX wire to GPIO 34
// (input only, free on the dog, not a strapping pin), 5 V, GND, its PWM pin
// to GND. Only receives (UART2, 230400 baud). Comment out LIDAR_TYPE to drop
// the driver; without a lidar connected it costs no map memory.
#define LIDAR_TYPE           LD06
#define LIDAR_RX_PIN         34

//Cores loop time
#define LOOP_TIME 4400   // microseconds (10^-6 seconds)
#define SERVICE_LOOP_TIME 100000
#define SERVICE_FAST_LOOP_TIME 5000

#endif
