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

	double left  = constrain(-(_vector->move.y + _vector->rotate.yaw), -1.0, 1.0);	// inverted because forward is -1, but logicaly it should be +1
	double right = constrain(-(_vector->move.y - _vector->rotate.yaw), -1.0, 1.0);

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

double IK::normalizeAngleRad(double angleRad)
{
	// TODO It is not working as expected
    //angleRad = fmod(angleRad,M_2_PI);
    //if (angleRad < 0) angleRad += M_2_PI;
    return angleRad;
}

// this can be update to use table of angles (faster) insted of using trigonometry functions
// or at least add cache
double IK::ikAtan2(double x, double y)
{
	return normalizeAngleRad(atan2(x, y));
}

double IK::ikAcos(double angleRad)
{
	return normalizeAngleRad(acos(angleRad));
}

double IK::ikAsin(double angleRad)
{
	return normalizeAngleRad(asin(angleRad));
}
