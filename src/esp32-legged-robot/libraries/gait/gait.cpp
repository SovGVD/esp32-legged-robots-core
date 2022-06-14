/**
 * Developed by Gleb Devyatkin (SovGVD) in 2022
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
double gait::next(uint8_t legId) {
	if (ticksToStop[legId] > 0) {
		ticksToStop[legId]--;
		progress[legId] = 1 - (float)ticksToStop[legId]/(float)ticksMax[legId];
		
		_legs[legId]->foot = _transition[legId].swing(progress[legId]).position;
		_legs[legId]->sensor.onGround = false;
	} else {
		progress[legId] = 0;
		// TODO use real sesors and stop gait if leg touch the ground
		_legs[legId]->sensor.onGround = true;
	}
	
	return progress[legId];
}

void gait::start(uint8_t legId, LR_point from, LR_point to) {
	ticksToStop[legId] = _config->swingDuration/_config->loopTime;
	ticksMax[legId]    = ticksToStop[legId];
	
	tParams = {
		{from, {0,0,0}},
		{to,   {0,0,0}},
		_config->offTheGround
	};
	// TODO add something to compare `point`
	if (from.x == to.x && from.y == to.y && from.z == to.z) {
		// don't move if from and to equal
		ticksToStop[legId] = 0;
	}
	_transition[legId].set(tParams);
}
