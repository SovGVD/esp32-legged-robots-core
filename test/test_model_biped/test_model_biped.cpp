/**
 * The biped model description (src/esp32-legged-robot/model/biped) with the
 * motion core: neutral stance, default gait within reach, joint and servo limits.
 */
#include <robot_core.h>
#include "def/def.h"
#include "model/biped/config.h"
#include "model/biped/model_data.h"

static BipedLegIK modelIK;

#include "../common/model_checks.h"

int main(int, char **)
{
	return runModelChecks();
}
