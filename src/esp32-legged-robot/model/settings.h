#if ROBOT_MODEL == MODEL_DOG
  #include "dog/settings.h"
#endif

#if ROBOT_MODEL == MODEL_BIPED
  #include "biped/settings.h"
#endif

#if ROBOT_MODEL == MODEL_TANK
  #include "tank/settings.h"
#endif

#if ROBOT_MODEL == MODEL_HEXAPOD
  #include "hexapod/settings.h"
#endif
