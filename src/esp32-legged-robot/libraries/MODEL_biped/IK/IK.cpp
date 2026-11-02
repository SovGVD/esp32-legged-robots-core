/**
 * Inverse Kinematics for bipedad (two legs) robot with 6DOF leg
 * 
 * This IK is simple because:
 *  - leg is in one plane
 *  - leg joints (even 3DOF) are single point
 *  - foot joint calculated based on assuption that ground is flat and parrallel to robot's body
 * 
 * 
 * LEFT side view
 *
 *  <- front
 * 
 *   |        |
 *   +--------*
 *       |
 *       o               <- 3DOF (alpha, beta, gamma), LEG_BODY_(X,Y,Z)
 *        \
 *         \  L1
 *          o            <- 1DOF (delta)
 *         /
 *        / L2
 *       o               <- 2 DOF (epsilon, zeta), LEG_FOOT_(X,Y,Z)
 *       | L3            <- from joint to the ground
 *    -------
 *    / / / /
 *                    ^ z
 *                    |
 *               y <--*
 * 
 * 
 * 3DOF joint
 * 
 * 
 *        <- | ->  - Alpha
 *           |
 *      +----+----+
 *      |         |   ^   (G) - Gamma
 *      | +-----+ |   |   
 *      \=| (G) |=/ -----  - Beta
 *        +---\-+     |
 *             \      v
 *              `
 *
 * 1DOF joint
 *        .
 *         \
 *          \
 *           o - Delta
 *          /
 *         /
 *        `
 * 
 * 2DOF joint
 * 
 *                 .
 *                /             ^    (E) - Epsilon
 *           +---/-+            |
 *         /=| (E) |=\  ---------- - Zeta
 *        |  +-----+  |         |
 *    +---+           +---+     v
 *    |    `---------`    |
 *    |  [ + battery - ]  |
 *    +-------------------+
 * ////////////////////////////
 * 
 *
 *  Leg Plane
 *     A         `-` - L1
 *     ,.\       `=` - L2
 *     ,.  \     `.` - Lxyz
 *     , .   B (aka Delta)
 *     ,  . //   LegPlaneTmpAngle1 - between `,` and `.`
 *     ,  .//    LegPlaneTmpAngle2 - between `.` and `-`
 *     D  C
 *
 * So Espilon is angle DCB, where
 *  - ACD = 90 - LegPlaneTmpAngle1
 *  - ABC = 180 - LegPlaneTmpAngle2 - Delta
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
			(_legs[legId]->body.x + _bodyOffset->x - _body->position.x) * tmpCos - (_legs[legId]->body.y + _bodyOffset->y - _body->position.y) * tmpSin,
			(_legs[legId]->body.x + _bodyOffset->x - _body->position.x) * tmpSin + (_legs[legId]->body.y + _bodyOffset->y - _body->position.y) * tmpCos,
			_legs[legId]->body.z + _bodyOffset->z
		};


	// TODO: what can I do with limits?
	float Lx = legFoot.x - legBody.x; if (_legs[legId]->inverse.x) { Lx = -Lx; };
	float Ly = legFoot.y - legBody.y; if (_legs[legId]->inverse.y) { Ly = -Ly; };
	float Lz = legFoot.z - legBody.z; if (_legs[legId]->inverse.z) { Lz = -Lz; };


	float sqLxz = sq(Lx) + sq(Lz);            // square of hypotenuse (points between leg.body and leg.foot in XZ-plane)
	float sqLxyz = sq(Ly) + sqLxz;            // square of hypotenuse (points between leg.body and leg.foot in 3D or leg plane)
	float Lxz = sqrt(sqLxz);
	float Lxyz = sqrt(sqLxyz);                // square of hypotenuse (points between leg.body and leg.foot in 3D or leg plane)
	float sqL1 = sq(_legs[legId]->size.l1);   // square of l1		TODO this is const actually
	float sqL2 = sq(_legs[legId]->size.l2);   // square of l2		TODO this is const actually
	float LegPlaneTmpAngle1 = ikAsin(Ly/Lxyz);
	float LegPlaneTmpAngle2 = ikAcos((sqLxyz+sqL1-sqL2) / (2 * Lxyz * _legs[legId]->size.l1));

	angle.alpha   = (Ly == 0 && Lx == 0) ? M_PI_2 : ikAtan2(Lx,fabsf(Ly)); if (Ly < 0) { angle.alpha = M_PI - angle.alpha; };	// @TODO this is going crazy, should be based on input yaw probably
	angle.beta    = ikAsin(Lx/Lxz) + M_PI_2;
	angle.gamma   = M_PI_2 + LegPlaneTmpAngle1 + LegPlaneTmpAngle2;
	angle.delta   = ikAcos((sqL1 + sqL2 - sqLxyz) / (2 * _legs[legId]->size.l1 * _legs[legId]->size.l2));
	angle.epsilon = (M_PI_2 - LegPlaneTmpAngle1)+(M_PI - angle.delta - LegPlaneTmpAngle2);
	angle.zeta    = M_PI - angle.beta;	// backward to angle.beta

	s.isSolved = !_clamped;
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
