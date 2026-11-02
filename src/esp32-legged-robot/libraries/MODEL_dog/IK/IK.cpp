/**
 * Inverse Kinematics for quadruped (four legs) robot
 * Boston Dynamics Spot style
 * 
 * All axis aligned with MPU9250
 *
 *
 *               |   Z-> Y
 *               |   |
 *    --------0-+   v X
 *            | ...l1
 *        0---0
 *
 *
 *      beta.                 alpha.
 *           .                      .
 *            .  |   ^ Z             .|         ^ Z
 *             0 |   |          ^  0--0         |
 *     ------ / -+   X-> Y      .  |  +---  X <-Y
 *           / ...l2            d  |
 * gamma...0                    .  0
 *          \                   .  |
 *           \ ...l3            v  |
 *
 *
 *
 *          0---0 (legLH)       0---0 (legLF)
 *              |               |
 *           +--0---------------0--+
 *           |                     |
 *           |                     |
 *           |                     |
 *           +--0---------------0--*
 *              |               |
 *          0---0 (legRH)       0---0 (legRF)
 *
 *
 * This IK is simple because leg is in one plane.
 * 
 *                _
 *              .|\      .
 *     -----| .    \      . L           -----|
 * --- -----0/ \    a      . E          -----0--. ---
 *  ^  ----/..  \    \      . G         ----/.  |  ^
 *  |  ---` . .  \   _\|     .          ---` .  |  |
 *  lz      .  . /   .        . P            0--`  a
 *  |       .   0   .          . L           |     |     ^Z
 *  v       .    \ .            . A          |     v     |
 * -------- .     0              . N         0 ------    0-->X
 *          |<lx->|               . E
 *                                 .
 * 
 * LEG PLANE
 * 
 *  -----0             0
 *  ^     `.           ^`.
 *  |       `.         \  `.
 *  a         0         b   0
 *  |        /           \ /       ^legplane
 *  v       /            v/        |
 *  -------0             0     Y<--0
 *  |<-ly->|
 * 
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


	float a = sq(lx) + sq(lz);        // square of hypotenuse (points between leg.body and leg.foot in XZ-plane)
	float sqrta = sqrt(a);
	float b = sq(ly) + a;             // square of hypotenuse (points between leg.body and leg.foot in YLeg-plane)
	float sqrtb = sqrt(b);            // square of hypotenuse (points between leg.body and leg.foot in YZ-plane)
	float l2p2 = sq(_legs[legId]->size.l2);   // square of l2		TODO this is const actually
	float l3p2 = sq(_legs[legId]->size.l3);   // square of l3		TODO this is const actually


	// XZ plane
	angle.alpha = M_PI_2 - ikAsin(lx/sqrta);

	// YLeg plane
	angle.beta  = M_PI_2 - ikAsin(ly/sqrtb) - ikAcos( (b+l2p2-l3p2) / (2 * sqrtb * _legs[legId]->size.l2));

	// YLeg plane
	angle.gamma = ikAcos((l2p2 + l3p2 - b) / (2 * _legs[legId]->size.l2 * _legs[legId]->size.l3));

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
