/**
 * Move command shared between the websocket handler (AsyncTCP task) and the
 * motion loop (core 1). Every access goes through a spinlock so the motion
 * loop never sees a half-written command.
 */
#include "robot.h"

static portMUX_TYPE  commandLock = portMUX_INITIALIZER_UNLOCKED;
static LR_moveVector command     = { {0, 0, 0}, {0, 0, 0} };

void setMoveCommand(const LR_moveVector &value)
{
	portENTER_CRITICAL(&commandLock);
	command = value;
	portEXIT_CRITICAL(&commandLock);
}

LR_moveVector getMoveCommand()
{
	portENTER_CRITICAL(&commandLock);
	LR_moveVector value = command;
	portEXIT_CRITICAL(&commandLock);
	return value;
}
