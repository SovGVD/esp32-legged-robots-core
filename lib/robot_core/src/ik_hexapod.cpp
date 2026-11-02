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

#include "ik.h"

iksolver HexapodLegIK::solve(const leg &l, const LR_figure &body, const LR_moveVector &command)
{
	(void)command;
	LR_legAngle angle = {};
	iksolver s;
	_clamped = false;

	LR_point v = legVector(l, body);
	float lx = v.x;
	float ly = v.y;
	float lz = v.z;

	float legLengthSq = sq(lx) + sq(ly);                               // square of leg length (XY-plane)
	float legLength   = sqrtf(legLengthSq);
	float lTibiaSq    = sq(legLength - l.size.l1) + sq(lz);            // distance between leg.foot and Coxa joint in XZ leg plane
	float lTibia      = sqrtf(lTibiaSq);
	float femurSq     = sq(l.size.l2);
	float tibiaSq     = sq(l.size.l3);
	float tmpAngle1   = ikAcos(lz/lTibia);                                                 // see tA1
	float tmpAngle2   = ikAcos((femurSq + lTibiaSq - tibiaSq) / (2 * l.size.l2 * lTibia)); // see tA2

	// Coxa Angle
	angle.alpha = atan2f(ly, lx);
	// Femur Angle
	angle.beta  = M_PI - (tmpAngle1 + tmpAngle2);
	// Tibia Angle
	angle.gamma = ikAcos((tibiaSq + femurSq - lTibiaSq) / (2 * l.size.l2 * l.size.l3));

	s.isSolved = !_clamped;
	s.angle    = angle;

	return s;
}
