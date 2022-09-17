void cliDebug()
{
  char* option = CLI_readWord();
  if (strcmp(option, "angle") == 0) {
    cliDebugAngle();
    return;
  }

  cliSerial->printf("Unknown debug option [%s]\n", option);
}


void cliDebugAngle()
{
  for (uint8_t i = 0; i < LEG_NUM; i++) {
    #if LEG_DOF == 6
      cliSerial->printf(
        "[%s] angle {%.2f, %.2f, %.2f, %.2f, %.2f, %.2f}\n",
        legs[i].id.title,
        radToDeg(legs[i].angle.alpha),
        radToDeg(legs[i].angle.beta),
        radToDeg(legs[i].angle.gamma),
        radToDeg(legs[i].angle.delta),
        radToDeg(legs[i].angle.epsilon),
        radToDeg(legs[i].angle.zeta)
      );
    #elif LEG_DOF == 3
      cliSerial->printf(
        "[%s] angle {%.2f, %.2f, %.2f}\n",
        legs[i].id.title,
        radToDeg(legs[i].angle.alpha),
        radToDeg(legs[i].angle.beta),
        radToDeg(legs[i].angle.gamma)
      );
    #elif LEG_DOF == 1
      cliSerial->printf(
        "[%s] angle {%.2f}\n",
        legs[i].id.title,
        radToDeg(legs[i].angle.alpha)
      );
    #endif
  }
}
