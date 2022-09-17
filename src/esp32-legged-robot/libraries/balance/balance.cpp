/**
 * Balance helper methods
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#include "balance.h"

balance::balance(LR_figure &bodyObj, leg (&legs)[LEG_NUM])
{
	_body   = &bodyObj;
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		_legs[i] = &legs[i];
	}
}


LR_point balance::getOffset(gaitState* state)
{
	_legsOnGround = 0;
	_offset = {0, 0, 0};

	// this is terrible
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		// Use only legs that are on ground
		if ((!state[i].progress.isDone && !state[i].beforeSwing.isDone) || _legs[i]->sensor.onGround) {
			_legToBalance(_legs[i]);
		}
	}

	_offset.x = _offset.x/_legsOnGround;
	_offset.y = _offset.y/_legsOnGround;
	//_offset.z = (_offset.z/_legsOnGround) * _progress;
	
	return _offset;
}

void balance::_legToBalance(leg *_leg) {
	_offset.x += _leg->foot.x - _body->position.x;
	_offset.y += _leg->foot.y - _body->position.y;
	//_offset.z += _leg->foot.z;
	_legsOnGround++;
}
