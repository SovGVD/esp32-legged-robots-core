#define CLI_MENU_COMMANDS_SET 28

// TODO... too much copy-paste, IDK how to avoid it=(
// HAL
// HAL - trim

bool _cliSetHALTrim (leg &_leg, int angleId, double deg) {
  return setHALTrim(_leg, angleId, deg);
}

double _cliSetHALTrimError()
{
  cliError("Out of limit");

  return 0;
}

double cliSetHALTrimLFAlpha(double deg) {
  if (_cliSetHALTrim (legs[LEGLF], ALPHA, deg)) {
    return _cliGetHALTrim(legs[LEGLF], ALPHA);
  }

  return _cliSetHALTrimError();
}

double cliSetHALTrimLFBeta(double deg) {
  if (_cliSetHALTrim (legs[LEGLF], BETA, deg)) {
    return _cliGetHALTrim(legs[LEGLF], BETA);
  }

  return _cliSetHALTrimError();
}

double cliSetHALTrimLFGamma(double deg) {
  if (_cliSetHALTrim (legs[LEGLF], GAMMA, deg)) {
    return _cliGetHALTrim(legs[LEGLF], GAMMA);
  }

  return _cliSetHALTrimError();
}


double cliSetHALTrimRFAlpha(double deg) {
  if (_cliSetHALTrim (legs[LEGRF], ALPHA, deg)) {
    return _cliGetHALTrim(legs[LEGRF], ALPHA);
  }

  return _cliSetHALTrimError();
}

double cliSetHALTrimRFBeta(double deg) {
  if (_cliSetHALTrim (legs[LEGRF], BETA, deg)) {
    return _cliGetHALTrim(legs[LEGRF], BETA);
  }

  return _cliSetHALTrimError();
}

double cliSetHALTrimRFGamma(double deg) {
  if (_cliSetHALTrim (legs[LEGRF], GAMMA, deg)) {
    return _cliGetHALTrim(legs[LEGRF], GAMMA);
  }

  return _cliSetHALTrimError();
}


double cliSetHALTrimLHAlpha(double deg) {
  if (_cliSetHALTrim (legs[LEGLH], ALPHA, deg)) {
    return _cliGetHALTrim(legs[LEGLH], ALPHA);
  }

  return _cliSetHALTrimError();
}

double cliSetHALTrimLHBeta(double deg) {
  if (_cliSetHALTrim (legs[LEGLH], BETA, deg)) {
    return _cliGetHALTrim(legs[LEGLH], BETA);
  }

  return _cliSetHALTrimError();
}

double cliSetHALTrimLHGamma(double deg) {
  if (_cliSetHALTrim (legs[LEGLH], GAMMA, deg)) {
    return _cliGetHALTrim(legs[LEGLH], GAMMA);
  }

  return _cliSetHALTrimError();
}


double cliSetHALTrimRHAlpha(double deg) {
  if (_cliSetHALTrim (legs[LEGRH], ALPHA, deg)) {
    return _cliGetHALTrim(legs[LEGRH], ALPHA);
  }

  return _cliSetHALTrimError();
}

double cliSetHALTrimRHBeta(double deg) {
  if (_cliSetHALTrim (legs[LEGRH], BETA, deg)) {
    return _cliGetHALTrim(legs[LEGRH], BETA);
  }

  return _cliSetHALTrimError();
}

double cliSetHALTrimRHGamma(double deg) {
  if (_cliSetHALTrim (legs[LEGRH], GAMMA, deg)) {
    return _cliGetHALTrim(legs[LEGRH], GAMMA);
  }

  return _cliSetHALTrimError();
}

// HAL - state

double cliSetHALState(double state)
{
  HALEnabled = state == 0 ? false : true;
  return cliGetHALState(state);
}

// Leg angle

double _cliSetAngle(leg &_leg, int angleId, double deg)
{
  if (!cliSetHALState(0) && setAngleDeg(_leg, angleId, deg)) {
    return _cliGetAngle(_leg, angleId);
  }

  return _cliSetAngleError();
}

double cliSetAngleLFAlpha(double deg) { return _cliSetAngle(legs[LEGLF], ALPHA, deg); }
double cliSetAngleLFBeta(double deg)  { return _cliSetAngle(legs[LEGLF], BETA,  deg); }
double cliSetAngleLFGamma(double deg) { return _cliSetAngle(legs[LEGLF], GAMMA, deg); }

double cliSetAngleRFAlpha(double deg) { return _cliSetAngle(legs[LEGRF], ALPHA, deg); }
double cliSetAngleRFBeta(double deg)  { return _cliSetAngle(legs[LEGRF], BETA,  deg); }
double cliSetAngleRFGamma(double deg) { return _cliSetAngle(legs[LEGRF], GAMMA, deg); }

double cliSetAngleLHAlpha(double deg) { return _cliSetAngle(legs[LEGLH], ALPHA, deg); }
double cliSetAngleLHBeta(double deg)  { return _cliSetAngle(legs[LEGLH], BETA,  deg); }
double cliSetAngleLHGamma(double deg) { return _cliSetAngle(legs[LEGLH], GAMMA, deg); }

double cliSetAngleRHAlpha(double deg) { return _cliSetAngle(legs[LEGRH], ALPHA, deg); }
double cliSetAngleRHBeta(double deg)  { return _cliSetAngle(legs[LEGRH], BETA,  deg); }
double cliSetAngleRHGamma(double deg) { return _cliSetAngle(legs[LEGRH], GAMMA, deg); }

// Servo calibration
double cliSetServoCalib(double i) {
  HALEnabled = false;
  delay(1000); // that it terrible, but we need to wait to make sure HAL disabled

  setServoToMiddle();

  return cliGetHALState(1);
}

double cliSetServoToInit(double i) {
  HALEnabled = false;
  delay(1000); // that it terrible, but we need to wait to make sure HAL disabled

  setServoToInit();

  return cliGetHALState(1);
  
}

const cliCommand cliCommandsSet[CLI_MENU_COMMANDS_SET] = {
  { "help",              cliSetHelp           },
  { "LF_HAL_trim_alpha", cliSetHALTrimLFAlpha },
  { "LF_HAL_trim_beta",  cliSetHALTrimLFBeta  },
  { "LF_HAL_trim_gamma", cliSetHALTrimLFGamma },
  { "RF_HAL_trim_alpha", cliSetHALTrimRFAlpha },
  { "RF_HAL_trim_beta",  cliSetHALTrimRFBeta  },
  { "RF_HAL_trim_gamma", cliSetHALTrimRFGamma },
  { "LH_HAL_trim_alpha", cliSetHALTrimLHAlpha },
  { "LH_HAL_trim_beta",  cliSetHALTrimLHBeta  },
  { "LH_HAL_trim_gamma", cliSetHALTrimLHGamma },
  { "RH_HAL_trim_alpha", cliSetHALTrimRHAlpha },
  { "RH_HAL_trim_beta",  cliSetHALTrimRHBeta  },
  { "RH_HAL_trim_gamma", cliSetHALTrimRHGamma },
  { "HAL",               cliSetHALState       },
  { "LF_angle_alpha",    cliSetAngleLFAlpha   },
  { "LF_angle_beta",     cliSetAngleLFBeta    },
  { "LF_angle_gamma",    cliSetAngleLFGamma   },
  { "RF_angle_alpha",    cliSetAngleRFAlpha   },
  { "RF_angle_beta",     cliSetAngleRFBeta    },
  { "RF_angle_gamma",    cliSetAngleRFGamma   },
  { "LH_angle_alpha",    cliSetAngleLHAlpha   },
  { "LH_angle_beta",     cliSetAngleLHBeta    },
  { "LH_angle_gamma",    cliSetAngleLHGamma   },
  { "RH_angle_alpha",    cliSetAngleRHAlpha   },
  { "RH_angle_beta",     cliSetAngleRHBeta    },
  { "RH_angle_gamma",    cliSetAngleRHGamma   },
  { "servo_calib",       cliSetServoCalib     },
  { "servo_to_init",     cliSetServoToInit    }
};
