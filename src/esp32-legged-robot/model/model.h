/**
 * Compile time configuration of the selected robot model (ROBOT_MODEL comes
 * from platformio.ini). Robot description data is in model/<name>/model.cpp.
 */
#ifndef MODEL_MODEL_H
#define MODEL_MODEL_H

#if ROBOT_MODEL == MODEL_DOG
	#include "dog/config.h"
#elif ROBOT_MODEL == MODEL_BIPED
	#include "biped/config.h"
#elif ROBOT_MODEL == MODEL_TANK
	#include "tank/config.h"
#elif ROBOT_MODEL == MODEL_HEXAPOD
	#include "hexapod/config.h"
#else
	#error "Unknown ROBOT_MODEL"
#endif

/**
 * Hardware in the loop (-DHIL, the robot-dog-hil / robot-hexapod-hil envs):
 * the ESP32 runs the whole firmware, but the servos, IMU, power sensor, ToF
 * sensors and foot switches are simulated in MuJoCo on a PC, over the USB
 * serial port (hil.cpp, sim/examples/hil.py). The model's pins and I2C
 * devices are not used.
 */
#ifdef HIL
	#if ROBOT_MODEL != MODEL_DOG && ROBOT_MODEL != MODEL_HEXAPOD
		#error "HIL: the simulator has the dog and the hexapod only"
	#endif
	#undef  PWM_CONTROLLER_TYPE
	#define PWM_CONTROLLER_TYPE HIL_SIM
	#undef  IMU_TYPE
	#define IMU_TYPE            HIL_SIM
	#undef  POWER_SENSOR
	#define POWER_SENSOR        HIL_SIM
	// TOF_TYPE stays VL53L5CX (room scans, terrain map): its frames come from the sim
	#undef  LIDAR_TYPE           // the LD06 is not simulated over the link
	#ifndef HIL_BAUD
		#define HIL_BAUD 921600
	#endif
	#undef  SERIAL_BAUD
	#define SERIAL_BAUD HIL_BAUD
#endif

#ifndef I2C_SDA
	#define I2C_SDA 21
#endif

#ifndef I2C_SCL
	#define I2C_SCL 22
#endif

#endif
