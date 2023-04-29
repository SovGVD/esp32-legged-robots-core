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


	double legLengthSq = sq(lx) + sq(ly);                                                                     // square of leg length (points between leg.body and leg.foot in XY-plane)
	double legLength   = sqrt(legLengthSq);
	double lTibiaSq    = sq(legLength - _legs[legId]->size.l1) + sq(lz);                                      // distance between leg.foot and Coxa joint in XZ leg plane
	double lTibia      = sqrt(lTibiaSq);
	double femurSq     = sq(_legs[legId]->size.l2);
	double tibiaSq     = sq(_legs[legId]->size.l3);
	double tmpAngle1   = ikAcos(lz/(lTibia));                                                                  // see tA1
	double tmpAngle2   = ikAcos((femurSq + lTibiaSq - tibiaSq) / (2 * _legs[legId]->size.l2 * lTibia));       // see tA2

	// Coxa Angle
	angle.alpha = atan2(ly, lx);
	// Femut Angle
	angle.beta = M_PI - (tmpAngle1 + tmpAngle2);
	// Tibia Angle
	angle.gamma = ikAcos((tibiaSq + femurSq - lTibiaSq) / (2 * _legs[legId]->size.l2 + _legs[legId]->size.l3));

	s.isSolved = true;	// TODO
	s.angle = angle;

	return s;
}

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
