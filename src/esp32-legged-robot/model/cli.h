#if ROBOT_MODEL == MODEL_DOG
  #include "dog/cli.cpp"
#endif

#if ROBOT_MODEL == MODEL_BIPED
  #include "biped/cli.cpp"
#endif

#if ROBOT_MODEL == MODEL_TANK
  #include "tank/cli.cpp"
#endif
