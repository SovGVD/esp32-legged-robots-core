/**
 * Robot description data for the biped model.
 * Compile time configuration (pins, sizes, hardware) lives in config.h.
 */
#include "../../robot.h"

#if ROBOT_MODEL == MODEL_BIPED

#include "model_data.h"

static BipedLegIK modelLegIK;
LegIK &MODEL_IK = modelLegIK;

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
