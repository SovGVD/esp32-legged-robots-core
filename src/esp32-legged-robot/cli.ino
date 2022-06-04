#define CLI_COMMANDS 2

const cliCommand cliCommands[CLI_COMMANDS] = {
  { "help",         cliHelp   , 0 },
  { "i2cscan",      cliI2cScan, 0 }
};

#include "model/cli.h"

void initCLI() {
  cliInitHelp();
}

void cliInitHelp()
{
  cliSerial->println();
  cliSerial->println(" CLI: type `help` to see available methods.");
  cliSerial->println();
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

void CLI_doCommand() {
  char * commandName = strtok(CLI_BUFFER, CLI_delimiters);
  /**
   * Default commands
   */
  for (int i = 0; i < CLI_COMMANDS; i++) {
    if (strcmp(commandName, cliCommands[i].commandName) == 0) {
      if (cliCommands[i].params != CLI_params) {
        cliSerial->print(CLI_params);
        cliSerial->println(" incorrect params number.");
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
        cliSerial->println(" incorrect params number.");
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
