/**
 * Inspired and copy-pasted https://gist.github.com/ilovetogetspamed/f64b72a274794600f0ad63b6d6b7c7eb
 */
#ifndef DEF_CLI_H
#define DEF_CLI_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define CLI_CR '\r'
#define CLI_LF '\n'
#define CLI_BS '\b'
#define CLI_NULLCHAR '\0'
#define CLI_SPACE ' '
#define CLI_DP 2

#ifndef CLI_BUFFER_LENGTH
  #define CLI_BUFFER_LENGTH 64
#endif

typedef void (*cliFunction)();

typedef struct cliCommand_t {
  char commandName[8];
  cliFunction func;
  char description[32];
  uint8_t params;	// number of parameters
  uint8_t coreId;
} cliCommand;

#endif
