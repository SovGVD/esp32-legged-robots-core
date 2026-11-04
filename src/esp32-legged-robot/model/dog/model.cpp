/**
 * Robot description data for the dog model.
 * Compile time configuration (pins, sizes, hardware) lives in config.h.
 */
#include "../../robot.h"

#if ROBOT_MODEL == MODEL_DOG

#include "model_data.h"

static QuadrupedLegIK modelLegIK;
LegIK &MODEL_IK = modelLegIK;

const ModelTerrain MODEL_TERRAIN = {
	TOF_SENSORS, TOF_SENSOR_COUNT, &TERRAIN_MAP_PARAMS, &TERRAIN_PARAMS, &GAIT_PARAMS_ROUGH, TERRAIN_TILT_MODE
};

const ModelLidar MODEL_LIDAR = { &LIDAR_PARAMS, &LIDAR_MAP_PARAMS, &NAV_PARAMS, &TOF_SCAN_PARAMS, &TOF_MAP_PARAMS };

const GaitParams *const MODEL_WALK_GAIT = &GAIT_PARAMS_WALK;

static void cliGait()
{
	char *name = CLI_readWord();
	if (strcmp(name, "walk") == 0) {
		requestGait(GAIT_PARAMS_WALK);
	} else if (strcmp(name, "trot") == 0) {
		requestGait(GAIT_PARAMS);
	} else if (strcmp(name, "state") == 0) {
		printGaitState();
		return;
	} else {
		cliSerial->println("Usage: gait trot|walk|state");
		return;
	}
	cliSerial->printf("Gait: %s (switches when the robot stands still)\n", name);
}

static const cliCommand dogCliCommands[] = {
	{ "gait", cliGait, "Gait [trot|walk|state]", 1, 0 }
};

const cliCommand *modelCliCommands     = dogCliCommands;
const uint8_t     modelCliCommandsCount = sizeof(dogCliCommands) / sizeof(dogCliCommands[0]);

void modelSettingsPrint()
{
}

void modelSettingsLoad()
{
}

void modelSettingsSave()
{
}

#endif
