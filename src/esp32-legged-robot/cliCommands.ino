void cliHelp()
{
  for (int i = 0; i < CLI_COMMANDS; i++) {
    cliSerial->printf(" - [%-8s] %s\n", cliCommands[i].commandName, cliCommands[i].description);
  }
  for (int i = 0; i < MODEL_CLI_MENU_COMMANDS; i++) {
    cliSerial->printf(" - [%-8s] %s\n",  modelCliCommands[i].commandName, modelCliCommands[i].description);
  }
}

void cliI2cScan()
{
  cliSerial->println("i2c:");
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      cliSerial->print (i, DEC);
      cliSerial->print (" (0x");
      cliSerial->print (i, HEX);
      cliSerial->print ("), ");
    }
  }
}
