/**
 * Robot description data for the hexapod model.
 * Compile time configuration (pins, sizes, hardware) lives in config.h.
 */
#include "../../robot.h"

#if ROBOT_MODEL == MODEL_HEXAPOD

#include "model_data.h"

static HexapodLegIK modelLegIK;
LegIK &MODEL_IK = modelLegIK;

const ModelTerrain MODEL_TERRAIN = {
	TOF_SENSORS, TOF_SENSOR_COUNT, &TERRAIN_MAP_PARAMS, &TERRAIN_PARAMS, &GAIT_PARAMS_ROUGH, TERRAIN_TILT_MODE
};

const ModelLidar MODEL_LIDAR = { &LIDAR_PARAMS, &LIDAR_MAP_PARAMS, &NAV_PARAMS, &TOF_SCAN_PARAMS, &TOF_MAP_PARAMS };

const GaitParams *const MODEL_WALK_GAIT = nullptr;   // one gait only

const cliCommand *modelCliCommands     = nullptr;
const uint8_t     modelCliCommandsCount = 0;

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
