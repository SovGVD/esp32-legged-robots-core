#if ROBOT_MODEL == MODEL_DOG
  #include "dog/gait.h"
#endif

#if ROBOT_MODEL == MODEL_BIPED
  #include "biped/gait.h"
#endif

#if ROBOT_MODEL == MODEL_TANK
  #include "tank/gait.h"
#endif
