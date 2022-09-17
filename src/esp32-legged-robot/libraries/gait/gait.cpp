/**
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 *
 * @TODO why this class have ALL legs? It should be class by leg
 */

#include "gait.h"

gait::gait(gaitConfig &config, leg (&legs)[LEG_NUM])
{
	_config = &config;
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		_legs[i] = &legs[i];
	}
}

/**
 * Return state if this gait is in progress
 */
void gait::next() {
	for (uint8_t i = 0; i < LEG_NUM; i++) {

		if (!shouldProcess(_progress[i].progress)) {
			markedAsFinished(i);
			continue;
		}

		if (shouldProcess(_progress[i].beforeSwing)) {
			// just do nothing
		} else if (shouldProcess(_progress[i].swing)) {
			_legs[i]->foot = _transition[i].swing(_progress[i].swing.tickProgress).position;
			_legs[i]->sensor.onGround = false;
		} else if (shouldProcess(_progress[i].afterSwing)) {
			// TODO use real sesors and stop gait if leg touch the ground
			_legs[i]->sensor.onGround = true;
		}
	}
}

gaitState* gait::getState()
{
	return _progress;
}

bool gait::shouldProcess(gaitTick &progress)
{
	if (progress.tickCurrent != 0) {
		progress.tickCurrent--;
		progress.tickProgress = 1 - (float)progress.tickCurrent / (float)progress.tickMax;
		progress.isInProgress = true;

		return true;
	}

	progress.tickProgress = 1.0;
	progress.isInProgress = false;
	progress.isDone       = true;

	return false;
}

void gait::markedAsFinished(uint8_t i)
{
	_progress[i].beforeSwing.tickProgress = 1.0;
	_progress[i].beforeSwing.isInProgress = false;
	_progress[i].beforeSwing.isDone       = true;

	_progress[i].swing.tickProgress = 1.0;
	_progress[i].swing.isInProgress = false;
	_progress[i].swing.isDone       = true;

	_progress[i].afterSwing.tickProgress = 1.0;
	_progress[i].afterSwing.isInProgress = false;
	_progress[i].afterSwing.isDone       = true;
}

void gait::start(uint8_t legId, LR_point from, LR_point to) {
	uint16_t tickTotal        = 0;
	uint16_t ticksBeforeSwing = 0;
	uint16_t ticksSwing       = 0;
	uint16_t ticksAfterSwing  = 0;

	/**
	 * Here double to uint16 - kind of dangerous
	 */

	if (from.x == to.x && from.y == to.y && from.z == to.z) {
		// nothing to do
	} else {
		tickTotal        = _config->duration / _config->loopTime;
		ticksBeforeSwing = (_config->duration - _config->swingDuration) / 2 /_config->loopTime;
		ticksSwing       = _config->swingDuration / _config->loopTime;
		ticksAfterSwing  = tickTotal - ticksSwing - ticksBeforeSwing;
	}

	_progress[legId] = {
		{
			tickTotal,
			tickTotal,
			0.0,
			false,
			false
		},
		{
			ticksBeforeSwing,
			ticksBeforeSwing,
			0.0,
			false,
			false
		},
		{
			ticksSwing,
			ticksSwing,
			0.0,
			false,
			false
		},
		{
			ticksAfterSwing,
			ticksAfterSwing,
			0.0,
			false,
			false
		}
	};
	
	_transitionParams = {
		{from, {0,0,0}},
		{to,   {0,0,0}},
		_config->offTheGround,
		{_config->transitionProgressStep[0], _config->transitionProgressStep[1]}
	};

	_transition[legId].set(_transitionParams);
}
