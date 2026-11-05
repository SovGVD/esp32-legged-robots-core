#ifndef MODEL_HEXAPOD2_CONFIG_H
#define MODEL_HEXAPOD2_CONFIG_H

/**
 * New hexapod (design "Recommended", 2026-11-04): 3D printed hexagon body,
 * 18 MG90S driving the joints directly (no printed gears), 2x 18650, one
 * Mini360 per leg, a PCA9685 + 3 Mini360 board standing along each side,
 * GY-91 inside the body centre, 2 VL53L5CX at the front, lever switch feet.
 * Not built yet: every number is the design, checked in the simulator only
 * (sim/, `python examples/drive.py --hexapod2`).
 */

// Coxa axes on the body, mm (X right, Y forward, Z up, origin at the femur
// axis height): corner legs at (+-35, +-58), middle legs at (+-45, 0).
// Neutral feet 69 mm out from the coxa axis (coxa 60 + 9: the foot is nearly
// under the femur axis, so the femur servo carries little torque), 100 mm
// below it. Rear and middle legs point straight out from the body centre,
// the front legs 20 deg further out to the side, out of the ToF view.
#define LEG_FRONT_BODY_X    35.0
#define LEG_FRONT_BODY_Y    58.0
#define LEG_FRONT_FOOT_X    88.71
#define LEG_FRONT_FOOT_Y   101.32

#define LEG_REAR_BODY_X     35.0
#define LEG_REAR_BODY_Y    -58.0
#define LEG_REAR_FOOT_X     70.65
#define LEG_REAR_FOOT_Y   -117.08

#define LEG_MIDDLE_BODY_X   45.0
#define LEG_MIDDLE_BODY_Y    0.0
#define LEG_MIDDLE_FOOT_X  114.0
#define LEG_MIDDLE_FOOT_Y    0.0

#define LEG_BODY_Z           0.0
#define LEG_FOOT_Z         100.0

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
#define LEG_SIZE_COXA   60.0
#define LEG_SIZE_FEMUR  50.0
#define LEG_SIZE_TIBIA  60.0


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
 * Hardware. Any ESP32 board (platformio.ini env robot-hexapod2: a plain ESP32
 * devkit). I2C on the ESP32 default pins, SDA 21 / SCL 22 (model/model.h);
 * define I2C_SDA / I2C_SCL here for other pins.
 */

// I2C addresses can be overridden from platformio.ini build_flags,
// e.g. -DPWM_CONTROLLER_ADDRESS_2=0x44
#ifndef PWM_CONTROLLER_ADDRESS
	#define PWM_CONTROLLER_ADDRESS   0x40
#endif
// One PCA9685 per side: left legs on the first board (channels 0-10), right
// legs on the second (16-26). 0x41 is taken by the INA219 below, so the second
// board defaults to 0x42 (A1 jumper).
#ifndef PWM_CONTROLLER_ADDRESS_2
	#define PWM_CONTROLLER_ADDRESS_2 0x42
#endif
#define PWM_CONTROLLER_TYPE      PCA9685

// GY-91 (MPU9250 + BMP280) in the body centre, under the top cover, away from
// the servo and battery wires (magnetometer heading)
#define IMU_ADDRESS              0x68
#define IMU_TYPE                 MPU9250

#define POWER_SENSOR_ADDRESS     0x41
#define POWER_SENSOR             INA219

// ToF terrain sensors: two VL53L5CX (8x8) on the front, turned 20 deg left and
// right, 35 deg down (TOF_SENSORS in model_data.h; sensor 0 is the left one).
// Both on the main I2C bus. Sensor 0: LPn tied high, moved to 0x2A at boot.
// Sensor 1: LPn on TOF_LPN_PIN, starts at 0x29 (change it to the pin you use).
#define TOF_TYPE                 VL53L5CX
#define TOF_LPN_PIN              14

// Indoor map and walking to a point (`lidar`, `goto`, web Map view). Without
// LIDAR_TYPE the ToF sensors do it: room scans (stop, nose up, turn around,
// TOF_SCAN_PARAMS in model_data.h). With an LD06 / LD19 lidar (LIDAR_PARAMS)
// uncomment LIDAR_TYPE: its TX wire to GPIO 15 (high at boot is the normal
// state of this strapping pin), 5 V,
// GND, its PWM pin to GND. Only receives (UART2, 230400 baud). Either way the
// map memory is taken only once data arrives.
//#define LIDAR_TYPE               LD06
#define LIDAR_RX_PIN             15

#define IGNORE_BROWNOUT_CONDITION

//Cores loop time
#define LOOP_TIME 7000   // microseconds (10^-6 seconds)
#define SERVICE_LOOP_TIME 100000
#define SERVICE_FAST_LOOP_TIME 7000

#endif
