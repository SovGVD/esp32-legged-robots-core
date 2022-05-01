/**
 * Bipedal robot gait config
 */

gaitConfig GAIT_CONFIG = {
  {
    {  SWING, STANCE },
    { STANCE,  SWING }
  },
  2,          // items in sequence
  LOOP_TIME,  // loop time in microseconds to correclty calculate number of sub moves per gate item
  30,         // above the ground on swing, 25mm
  100000,     // swing duration, microseconds
  200000      // sequence items duration, microseconds
};
