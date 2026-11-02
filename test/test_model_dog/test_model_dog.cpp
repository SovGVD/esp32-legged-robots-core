/**
 * The dog model description (src/esp32-legged-robot/model/dog) with the
 * motion core: neutral stance, default gait within reach, joint and servo limits.
 */
#include <robot_core.h>
#include "def/def.h"
#include "model/dog/config.h"
#include "model/dog/model_data.h"

static QuadrupedLegIK modelIK;

#include "../common/model_checks.h"

int main(int, char **)
{
	return runModelChecks();
}
