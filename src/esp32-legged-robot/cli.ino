#define CLI_COMMANDS 8

const cliCommand cliCommands[CLI_COMMANDS] = {
  { "help",      cliHelp,       "Show list of methods",              0 },
  { "wifi",      WiFiInfo,      "WiFi AP information",               0 },
  { "trim",      cliSetTrim,    "Trim servos",                       3 },
  { "pwm",       cliHalServo,   "Servo position in microsec(uS)",    2 },
  { "show",      settingsPrint, "Display current settings",          0 },
  { "save",      settingsSave,  "Save setting to EEPROM",            0 },
  { "calib",     cliCalibrate,  "Calibrate [imu|servo|leg]",         1 },
  { "hal",       cliHal,        "[on|off|state] leg calculation",    1 }
};

#include "model/cli.h"

void initCLI()
{
  cliInitHelp();
}

void cliInitHelp()
{
  cliSerial->print("CLI: type `help` to see available commands.");
}

bool CLI_get(char * CLI_BUFFER)
{
  static uint8_t CLI_charsRead = 0;
  while (cliSerial->available()) {
    char CLI_c = cliSerial->read();
    switch (CLI_c) {
      case CLI_CR:      //likely have full command in buffer now, commands are terminated by CR and/or LS
      case CLI_LF:
        CLI_BUFFER[CLI_charsRead] = CLI_NULLCHAR;       //null terminate our command char array
        if (CLI_charsRead > 0)  {
          CLI_charsRead = 0;                           //charsRead is static, so have to reset
          cliSerial->print("> ");
          cliSerial->println(CLI_BUFFER);
          return true;
        }
        break;
      default:
        if (CLI_c == 32) {
          CLI_params++;
        }
        if (CLI_charsRead < CLI_BUFFER_LENGTH) {
          CLI_BUFFER[CLI_charsRead++] = CLI_c;
        }
        CLI_BUFFER[CLI_charsRead] = CLI_NULLCHAR;     //just in case
        break;
    }
  }
  return false;
}

int CLI_readInt() {
  char * numTextPtr = strtok(NULL, CLI_delimiters);   //K&R string.h  pg. 250
  return atoi(numTextPtr);                            //K&R string.h  pg. 251
}

float CLI_readFloat() {
  char * floatTextPtr = strtok(NULL, CLI_delimiters);
  return atof(floatTextPtr);
}

char * CLI_readWord() {
  char * word = strtok(NULL, CLI_delimiters);         //K&R string.h  pg. 250
  return word;
}

uint16_t CLI_readUInt16() {
  char * numTextPtr = strtok(NULL, CLI_delimiters);
  return (uint16_t)atol(numTextPtr);
}

void CLI_doCommand()
{
  char * commandName = strtok(CLI_BUFFER, CLI_delimiters);
  /**
   * Default commands
   */
  for (int i = 0; i < CLI_COMMANDS; i++) {
    if (strcmp(commandName, cliCommands[i].commandName) == 0) {
      if (cliCommands[i].params != CLI_params) {
        cliSerial->printf("%u incorrect params number, expected %u.\n", CLI_params, cliCommands[i].params);
        CLI_params = 0;
        return;
      }
      CLI_params = 0;
      cliCommands[i].func();
      return;
    }
  }

  /**
   * Model commands
   */
  for (int i = 0; i < MODEL_CLI_MENU_COMMANDS; i++) {
    if (strcmp(commandName, modelCliCommands[i].commandName) == 0) {
      if (modelCliCommands[i].params != CLI_params) {
        cliSerial->print(CLI_params);
        cliSerial->printf("%u incorrect params number, expected %u.\n", CLI_params, modelCliCommands[i].params);
        CLI_params = 0;
        return;
      }
      CLI_params = 0;
      modelCliCommands[i].func();
      return;
    }
  }

  CLI_params = 0;
  cliSerial->println("Unknown command.");
}

void updateCLI()
{
	if (CLI_get(CLI_BUFFER)){
	  CLI_doCommand();
	}
}

void printAvailableLegs()
{
	cliSerial->print("Legs: ");
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		cliSerial->printf("[%s],", legs[i].id.title);
	}
	cliSerial->println();
}
