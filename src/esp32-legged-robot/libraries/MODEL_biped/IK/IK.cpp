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
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#include "IK.h"

IK::IK(LR_figure &bodyObj, leg (&legs)[LEG_NUM])
{
	_body = &bodyObj;
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		_legs[i] = &legs[i];
	}
}

iksolver IK::solve(uint8_t legId)
{
	LR_legAngle angle;
	iksolver s;

	// normalize by body rotation
	// We need it to make IK simple ignoring body transitions, and at the same time still have absolute positions for each robot points
	double tmpSin = sin(_body->orientation.yaw * -1);
	double tmpCos = cos(_body->orientation.yaw * -1);

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
	double lx = legFoot.x - legBody.x; if (_legs[legId]->inverse.x) { lx = -lx; };
	double ly = legFoot.y - legBody.y; if (_legs[legId]->inverse.y) { ly = -ly; };
	double lz = legFoot.z - legBody.z; if (_legs[legId]->inverse.z) { lz = -lz; };


	double a = sq(lx) + sq(lz);        // square of hypotenuse (points between leg.body and leg.foot in XZ-plane)
	double sqrta = sqrt(a);
	double b = sq(ly) + a;             // square of hypotenuse (points between leg.body and leg.foot in YLeg-plane)
	double sqrtb = sqrt(b);            // square of hypotenuse (points between leg.body and leg.foot in YZ-plane)
	double l2p2 = sq(_legs[legId]->size.l2);   // square of l2		TODO this is const actually
	double l3p2 = sq(_legs[legId]->size.l3);   // square of l3		TODO this is const actually


	// XZ plane
	angle.alpha = M_PI_2 - ikAsin(lx/sqrta);

	// YLeg plane
	angle.beta  = M_PI_2 - ikAsin(ly/sqrtb) - ikAcos( (b+l2p2-l3p2) / (2 * sqrtb * _legs[legId]->size.l2));

	// YLeg plane
	angle.gamma = ikAcos((l2p2 + l3p2 - b) / (2 * _legs[legId]->size.l2 * _legs[legId]->size.l3));

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
