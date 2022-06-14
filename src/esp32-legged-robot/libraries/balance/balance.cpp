/**
 * Balance helper methods
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#include "balance.h"

balance::balance(LR_point &offset, LR_figure &bodyObj, leg (&legs)[LEG_NUM])
{
	_offset = &offset;	// this should be calculated? base on servos and legs positions and weight
	_body   = &bodyObj;
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		_legs[i] = &legs[i];
	}
}


LR_point balance::getCenter()
{
	// TODO... can we ignore Z? as it will be projection of BODY center of mass (CoM) to legs-on-ground plain
	// This is not technicaly correct, we are not geting center of mass, but just the point where it should be as projection
	_legsOnGround = 0;
	_CoM = {0, 0, 0};

	// this is terrible
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		_legToBalance(_legs[i]);
	}

	_CoM.x = _CoM.x/_legsOnGround + _offset->x;
	_CoM.y = _CoM.y/_legsOnGround + _offset->y;
	_CoM.z = _CoM.z/_legsOnGround + _offset->z;
	
	/*Serial.print(_legsOnGround);
	Serial.print("|CoM ");
	Serial.print(_CoM.x);
	Serial.print(",");
	Serial.print(_CoM.y);
	Serial.print(",");
	Serial.println(_CoM.z);*/
	
	return _CoM;
}

void balance::_legToBalance(leg *_leg) {
	if (_leg->sensor.onGround) {
		_CoM.x += _leg->foot.x;
		_CoM.y += _leg->foot.y;
		_CoM.z += _leg->foot.z;
		_legsOnGround++;
	}
}

void balance::setBody(LR_point CoM)
{
	// TODO that is kind of wrong, as we need to use body projection
	_body->position.x = CoM.x;
	_body->position.y = CoM.y;
	//_body->position.z = CoM.z;
}
