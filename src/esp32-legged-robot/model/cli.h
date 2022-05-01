#if ROBOT_MODEL == MODEL_DOG
  #include "dog/cliGet.cpp"
  #include "dog/cliSet.cpp"
#endif

#if ROBOT_MODEL == MODEL_PIPED
  #include "biped/cliGet.cpp"
  #include "biped/cliSet.cpp"
#endif
