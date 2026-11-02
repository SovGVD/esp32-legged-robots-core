#include "robot.h"

static bool CLI_get(char * CLI_BUFFER);
static void CLI_prepareCommand();

const char *CLI_delimiters = " \n";
char CLI_BUFFER[CLI_BUFFER_LENGTH + 1];

#define CLI_COMMANDS 10
#define CLI_MAIN_SET 0
#define CLI_MODEL_SET 100

/**
 * @TODO could it be just pointer to cliCommand?
 */
volatile bool CLI_commandReady = false;   // set on core 0, may be cleared on core 1
uint8_t CLI_commandSet    = 0;
uint8_t CLI_commandCoreId = 0;
uint8_t CLI_commandIdx    = 0;

const cliCommand cliCommands[CLI_COMMANDS] = {
  { "help",      cliHelp,       "Show list of commands",             0, 0 },
  { "wifi",      WiFiInfo,      "WiFi AP information",               0, 0 },
  { "trim",      cliSetTrim,    "Trim servos",                       3, 0 },
  { "pwm",       cliPwmServo,   "Servo position in microsec(uS)",    3, 0 },
  { "angle",     cliAngleServo, "Servo position in degrees",         3, 0 },
  { "show",      settingsPrint, "Display current settings",          0, 0 },
  { "save",      settingsSave,  "Save setting to EEPROM",            0, 0 },
  { "calib",     cliCalibrate,  "Calibrate [imu|leg]",               1, 0 },
  { "debug",     cliDebug,      "Show debug information [angle]",    1, 0 },
  { "hal",       cliHal,        "[on|off|state] leg calculation",    1, 0 }
};


void initCLI()
{
  cliInitHelp();
}

void cliInitHelp()
{
  cliSerial->print("\nCLI: type `help` to see available commands.\n");
}

static bool CLI_get(char * CLI_BUFFER)
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
        if (CLI_charsRead < CLI_BUFFER_LENGTH) {
          CLI_BUFFER[CLI_charsRead++] = CLI_c;
        }
        CLI_BUFFER[CLI_charsRead] = CLI_NULLCHAR;     //just in case
        break;
    }
  }
  return false;
}

/**
 * The command line is split into tokens once, when it is received. Commands
 * read their parameters from that list, so the count check is exact (repeated
 * or trailing spaces used to be counted as parameters) and nothing depends on
 * strtok() state, which is per task and the command may run on the other core.
 */
#define CLI_MAX_TOKENS 8
static char    CLI_tokens[CLI_MAX_TOKENS][CLI_BUFFER_LENGTH + 1];
static uint8_t CLI_tokenCount = 0;
static uint8_t CLI_tokenRead  = 0;

static void CLI_tokenize(char *line)
{
  CLI_tokenCount = 0;
  CLI_tokenRead  = 1;   // token 0 is the command name
  char *save = nullptr;
  for (char *t = strtok_r(line, CLI_delimiters, &save); t != nullptr; t = strtok_r(nullptr, CLI_delimiters, &save)) {
    if (CLI_tokenCount == CLI_MAX_TOKENS) {
      CLI_tokenCount++;   // too many, reported as a wrong parameter count
      break;
    }
    strlcpy(CLI_tokens[CLI_tokenCount++], t, sizeof(CLI_tokens[0]));
  }
}

static const char *CLI_nextToken()
{
  if (CLI_tokenRead >= CLI_tokenCount || CLI_tokenRead >= CLI_MAX_TOKENS) {
    return "";
  }
  return CLI_tokens[CLI_tokenRead++];
}

int CLI_readInt() {
  return atoi(CLI_nextToken());
}

float CLI_readFloat() {
  return atof(CLI_nextToken());
}

char* CLI_readWord() {
  return (char *)CLI_nextToken();
}

uint16_t CLI_readUInt16() {
  return (uint16_t)atol(CLI_nextToken());
}

static bool CLI_matchCommand(const cliCommand &cmd, uint8_t set, uint8_t idx, uint8_t params)
{
  if (strcmp(CLI_tokens[0], cmd.commandName) != 0) {
    return false;
  }
  if (cmd.params != params) {
    cliSerial->printf("%u incorrect params number, expected %u.\n", params, cmd.params);
    return true;
  }
  CLI_commandSet    = set;
  CLI_commandCoreId = cmd.coreId;
  CLI_commandIdx    = idx;
  CLI_commandReady  = true;
  return true;
}

static void CLI_prepareCommand()
{
  if (CLI_commandReady) {
    cliSerial->println("Busy, previous command is still pending.");
    return;
  }

  CLI_tokenize(CLI_BUFFER);
  if (CLI_tokenCount == 0) {
    return;   // only spaces
  }
  uint8_t params = CLI_tokenCount - 1;

  for (uint8_t i = 0; i < CLI_COMMANDS; i++) {
    if (CLI_matchCommand(cliCommands[i], CLI_MAIN_SET, i, params)) {
      return;
    }
  }
  for (uint8_t i = 0; i < modelCliCommandsCount; i++) {
    if (CLI_matchCommand(modelCliCommands[i], CLI_MODEL_SET, i, params)) {
      return;
    }
  }

  cliSerial->println("Unknown command.");
}

void updateCLI()
{
	if (CLI_get(CLI_BUFFER)){
	  CLI_prepareCommand();
	}
}

void CLI_doCommand()
{
  if (!CLI_commandReady) {
    return;
  }

  if (xPortGetCoreID() != CLI_commandCoreId) {
    return;
  }

  CLI_commandReady = false;

  if (CLI_commandSet == CLI_MODEL_SET) {
     modelCliCommands[CLI_commandIdx].func();

     return;
  }

  cliCommands[CLI_commandIdx].func();
}

void printAvailableLegs()
{
	cliSerial->print("Legs: ");
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		cliSerial->printf("[%s],", legs[i].id.title);
	}
	cliSerial->println();
}

void cliHelp()
{
  for (int i = 0; i < CLI_COMMANDS; i++) {
    cliSerial->printf(" - [%-8s] %s\n", cliCommands[i].commandName, cliCommands[i].description);
  }
  for (int i = 0; i < modelCliCommandsCount; i++) {
    cliSerial->printf(" - [%-8s] %s\n",  modelCliCommands[i].commandName, modelCliCommands[i].description);
  }
}
