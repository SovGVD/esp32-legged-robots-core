/**
 * Inverse Kinematics for hexapod (six legs) robot
 * WARNING!!! Was not tested on real robot!
 *
 * All axis aligned with MPU9250
 *
 *
 *               |   Z-> Y
 *               |   |
 *    --------0-+    v X
 *            | - Coxa (alpha, l1)
 *            -
 *            | - Femur (beta, l2)
 *            -
 *            |
 *            | - Tibia (gamma, l3)
 *            + - Sensor
 *
 *
 *  
 *             0                 z ^
 *            / \                  |
 *           /   \                 y--> x
 *          /     \
 *         /       \          |
 *        /     tA2 0---------0  <-- leg.body
 *       /      .--`:         :
 *      /   ,--` tA1:         :
 *     /.--`  ^     :         :
 *    *-------|-----*---------*
 *    ^       |               :
 *    |       lTibia          :
 *    leg.foot                :
 *    |<---- legLength ------>|
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2023
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
	_clamped = false;

	// normalize by body rotation
	// We need it to make IK simple ignoring body transitions, and at the same time still have absolute positions for each robot points
	float tmpSin = sin(_body->orientation.yaw * -1);
	float tmpCos = cos(_body->orientation.yaw * -1);

	LR_point legFoot = {
			(_legs[legId]->foot.x - _body->position.x) * tmpCos - (_legs[legId]->foot.y - _body->position.y) * tmpSin,
			(_legs[legId]->foot.x - _body->position.x) * tmpSin + (_legs[legId]->foot.y - _body->position.y) * tmpCos,
			_legs[legId]->foot.z
		};

	LR_point legBody = {
			(_legs[legId]->body.x - _body->position.x) * tmpCos - (_legs[legId]->body.y - _body->position.y) * tmpSin,
			(_legs[legId]->body.x - _body->position.x) * tmpSin + (_legs[legId]->body.y - _body->position.y) * tmpCos,
			_legs[legId]->body.z
		};


	// TODO: what can I do with limits?
	float lx = legFoot.x - legBody.x; if (_legs[legId]->inverse.x) { lx = -lx; };
	float ly = legFoot.y - legBody.y; if (_legs[legId]->inverse.y) { ly = -ly; };
	float lz = legFoot.z - legBody.z; if (_legs[legId]->inverse.z) { lz = -lz; };


	float legLengthSq = sq(lx) + sq(ly);                                                                     // square of leg length (points between leg.body and leg.foot in XY-plane)
	float legLength   = sqrt(legLengthSq);
	float lTibiaSq    = sq(legLength - _legs[legId]->size.l1) + sq(lz);                                      // distance between leg.foot and Coxa joint in XZ leg plane
	float lTibia      = sqrt(lTibiaSq);
	float femurSq     = sq(_legs[legId]->size.l2);
	float tibiaSq     = sq(_legs[legId]->size.l3);
	float tmpAngle1   = ikAcos(lz/(lTibia));                                                                  // see tA1
	float tmpAngle2   = ikAcos((femurSq + lTibiaSq - tibiaSq) / (2 * _legs[legId]->size.l2 * lTibia));       // see tA2

	// Coxa Angle
	angle.alpha = atan2(ly, lx);
	// Femut Angle
	angle.beta = M_PI - (tmpAngle1 + tmpAngle2);
	// Tibia Angle
	angle.gamma = ikAcos((tibiaSq + femurSq - lTibiaSq) / (2 * _legs[legId]->size.l2 * _legs[legId]->size.l3));

	s.isSolved = !_clamped;
	s.angle = angle;

	return s;
}

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
