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
#elif ROBOT_MODEL == MODEL_HEXAPOD2
	#include "hexapod2/config.h"
#else
	#error "Unknown ROBOT_MODEL"
#endif

#ifndef I2C_SDA
	#define I2C_SDA 21
#endif

#ifndef I2C_SCL
	#define I2C_SCL 22
#endif

#endif
