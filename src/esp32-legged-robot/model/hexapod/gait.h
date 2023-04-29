/**
 * Hexapod robot gait config
 * @TODO gait change
 */
#include "../../libraries/gait/gait.h"

gaitConfig GAIT_CONFIG = {
  {
    // tripod
    { SWING,  STANCE,  SWING, STANCE,  SWING, STANCE },
    { STANCE,  SWING, STANCE,  SWING, STANCE,  SWING }
  },
  2,          // items in sequence
  LOOP_TIME,  // loop time in microseconds to correclty calculate number of sub moves per gait item
  30,         // above the ground on swing, 25mm
  100000,     // swing duration, microseconds
  200000,     // sequence items duration, microseconds
  {0.4, 0.4}  // transition progress steps @see transition.h
};
