#if ROBOT_MODEL == MODEL_DOG
  #include "dog/settings.cpp"
#endif

#if ROBOT_MODEL == MODEL_BIPED
  #include "biped/settings.cpp"
#endif

#if ROBOT_MODEL == MODEL_TANK
  #include "tank/settings.cpp"
#endif

#if ROBOT_MODEL == MODEL_HEXAPOD
  #include "hexapod/settings.cpp"
#endif
