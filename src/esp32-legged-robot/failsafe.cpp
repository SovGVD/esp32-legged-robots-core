/**
 * Failsafe: stop the robot when the controlling client goes quiet.
 *
 * The web client sends a move package every 50 ms. If nothing arrives for
 * FS_TIMEOUT_MS (or the client disconnects, or it sends "disabled"), the move
 * command is reset to zero and the gait settles the feet.
 */
#include "robot.h"

static volatile uint32_t lastCommandMs = 0;
static volatile bool     everFed       = false;
bool FS_FAIL = true;   // no client yet

void failsafeFeed()
{
	lastCommandMs = millis();
	everFed       = true;
}

void updateFailsafe()
{
	bool timedOut = !everFed || (uint32_t)(millis() - lastCommandMs) > FS_TIMEOUT_MS;
	if (timedOut) {
		if (!FS_FAIL) {
			Serial.println("Failsafe: no commands, stopping");
		}
		setFailsafe();
		return;
	}

	FS_FAIL = false;
}

void setFailsafe()
{
	LR_moveVector stop = { {0, 0, 0}, {0, 0, 0} };
	setMoveCommand(stop);
	everFed = false;
	FS_FAIL = true;
}
