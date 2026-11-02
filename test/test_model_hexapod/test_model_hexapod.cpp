/**
 * The hexapod model description (src/esp32-legged-robot/model/hexapod) with the
 * motion core: neutral stance, default gait within reach, joint and servo limits.
 */
#include <robot_core.h>
#include "def/def.h"
#include "model/hexapod/config.h"
#include "model/hexapod/model_data.h"

static HexapodLegIK modelIK;

#include "../common/model_checks.h"

int main(int, char **)
{
	return runModelChecks();
}
