/**
 * The new hexapod model description (src/esp32-legged-robot/model/hexapod2, MG90S direct drive) with the
 * motion core: neutral stance, default gait within reach, joint and servo limits.
 */
#include <robot_core.h>
#include "def/def.h"
#include "model/hexapod2/config.h"
#include "model/hexapod2/model_data.h"

static HexapodLegIK modelIK;

#include "../common/model_checks.h"

int main(int, char **)
{
	return runModelChecks();
}
