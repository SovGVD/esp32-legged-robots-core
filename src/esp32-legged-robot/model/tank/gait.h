#include "../../libraries/gait/gait.h"

gaitConfig GAIT_CONFIG = {
  {
    { SWING, SWING }
  },
  1,          // items in sequence
  LOOP_TIME,  // loop time in microseconds to correclty calculate number of sub moves per gait item
  0,          // above the ground on swing, 20mm
  LOOP_TIME,  // swing duration, microseconds
  LOOP_TIME,  // sequence items duration, microseconds
  {0.0, 1.0}  // transition progress steps @see transition.h
};
