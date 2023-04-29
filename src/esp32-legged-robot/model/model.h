#if ROBOT_MODEL == MODEL_DOG
  #include "dog/model.h"
#endif

#if ROBOT_MODEL == MODEL_BIPED
  #include "biped/model.h"
#endif

#if ROBOT_MODEL == MODEL_TANK
  #include "tank/model.h"
#endif

#if ROBOT_MODEL == MODEL_HEXAPOD
  #include "hexapod/model.h"
#endif

#ifndef I2C_SDA
    #define I2C_SDA 21
#endif

#ifndef I2C_SCL
    #define I2C_SCL 22
#endif
