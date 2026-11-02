/**
 * This is not IK
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#include "../../IK/IK.h"

IK::IK(LR_moveVector &vector, LR_figure &bodyObj, LR_point &bodyBalanceOffset, leg (&legs)[LEG_NUM])
{
	_vector     = &vector;
	_body       = &bodyObj;
	_bodyOffset = &bodyBalanceOffset;
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		_legs[i] = &legs[i];
	}
}

iksolver IK::solve(uint8_t legId)
{
	LR_legAngle angle;
	iksolver s;
	// This is too simple and ignoring all ideas of using coordinates... but who cares, currently it is just for fun

	float left  = constrain(-(_vector->move.y + _vector->rotate.yaw), -1.0, 1.0);	// inverted because forward is -1, but logicaly it should be +1
	float right = constrain(-(_vector->move.y - _vector->rotate.yaw), -1.0, 1.0);

	if (legId == TRACKL) {
		angle.alpha = (left >= 0 ?  left : 0) * _legs[legId]->max.alpha;
		angle.beta  = (left <  0 ? -left : 0) * _legs[legId]->max.alpha;
	} else if (legId == TRACKR) {
		angle.alpha = (right >= 0 ?  right : 0) * _legs[legId]->max.beta;
		angle.beta  = (right <  0 ? -right : 0) * _legs[legId]->max.beta;
	} else {
		angle.alpha = 0;
		angle.beta  = 0;
	}

	s.isSolved = true;	// @TODO
	s.angle = angle;

	return s;
}


// @TODO move all that service methods into other class or even extend IK classes from some base class

float IK::normalizeAngleRad(float angleRad)
{
	// TODO It is not working as expected
    //angleRad = fmod(angleRad,M_2_PI);
    //if (angleRad < 0) angleRad += M_2_PI;
    return angleRad;
}

// this can be update to use table of angles (faster) insted of using trigonometry functions
// or at least add cache
float IK::ikAtan2(float x, float y)
{
	return normalizeAngleRad(atan2f(x, y));
}

/**
 * acos/asin are only defined on [-1, 1]. Outside of it (target out of reach,
 * or 0/0 when the foot sits exactly on a joint) they return NaN, which used to
 * end up as a servo pulse. Clamp instead and mark the solution as not solved.
 */
float IK::ikClampUnit(float value)
{
	if (isnan(value)) {
		_clamped = true;
		return 1.0f;
	}
	if (value > 1.0f) {
		_clamped = true;
		return 1.0f;
	}
	if (value < -1.0f) {
		_clamped = true;
		return -1.0f;
	}
	return value;
}

float IK::ikAcos(float value)
{
	return normalizeAngleRad(acosf(ikClampUnit(value)));
}

float IK::ikAsin(float value)
{
	return normalizeAngleRad(asinf(ikClampUnit(value)));
}
