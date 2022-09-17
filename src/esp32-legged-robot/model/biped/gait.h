/**
 * Bipedal robot gait config
 */

gaitConfig GAIT_CONFIG = {
  {
    {  SWING, STANCE },
    { STANCE,  SWING }
  },
  2,          // items in sequence
  LOOP_TIME,  // loop time in microseconds to correclty calculate number of sub moves per gait item
  40,         // above the ground on swing, 20mm
  1000000,    // swing duration, microseconds
  2000000,    // sequence items duration, microseconds
  {0.3, 0.7}, // transition progress steps @see transition.h
};
