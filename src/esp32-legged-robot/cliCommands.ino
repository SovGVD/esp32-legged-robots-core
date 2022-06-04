void cliHelp()
{
  for (int i = 0; i < CLI_COMMANDS; i++) {
    cliSerial->print(cliCommands[i].commandName);
    cliSerial->print(" ");
    cliSerial->println(cliCommands[i].params);
  }
  for (int i = 0; i < MODEL_CLI_MENU_COMMANDS; i++) {
    cliSerial->print(modelCliCommands[i].commandName);
    cliSerial->print(" ");
    cliSerial->println(modelCliCommands[i].params);
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
