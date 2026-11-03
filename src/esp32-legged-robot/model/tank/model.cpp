/**
 * Robot description data for the tank model.
 * Compile time configuration (pins, sizes, hardware) lives in config.h.
 */
#include "../../robot.h"

#if ROBOT_MODEL == MODEL_TANK

#include "model_data.h"

static TankDriveIK modelLegIK;
LegIK &MODEL_IK = modelLegIK;

const ModelTerrain MODEL_TERRAIN = { nullptr, 0, nullptr, nullptr, nullptr, TERRAIN_TILT_FILTERED };   // no rough ground support

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
