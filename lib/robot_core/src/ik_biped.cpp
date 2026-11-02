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

#include "ik.h"

iksolver BipedLegIK::solve(const leg &l, const LR_figure &body, const LR_moveVector &command)
{
	(void)command;
	LR_legAngle angle = {};
	iksolver s;
	_clamped = false;

	LR_point v = legVector(l, body);
	float Lx = v.x;
	float Ly = v.y;
	float Lz = v.z;

	float sqLxz  = sq(Lx) + sq(Lz);   // square of hypotenuse (XZ-plane)
	float sqLxyz = sq(Ly) + sqLxz;    // square of hypotenuse (3D / leg plane)
	float Lxz    = sqrtf(sqLxz);
	float Lxyz   = sqrtf(sqLxyz);
	float sqL1   = sq(l.size.l1);
	float sqL2   = sq(l.size.l2);
	float LegPlaneTmpAngle1 = ikAsin(Ly/Lxyz);
	float LegPlaneTmpAngle2 = ikAcos((sqLxyz + sqL1 - sqL2) / (2 * Lxyz * l.size.l1));

	angle.alpha   = (Ly == 0 && Lx == 0) ? M_PI_2 : atan2f(Lx, fabsf(Ly)); if (Ly < 0) { angle.alpha = M_PI - angle.alpha; };	// @TODO this is going crazy, should be based on input yaw probably
	angle.beta    = ikAsin(Lx/Lxz) + M_PI_2;
	angle.gamma   = M_PI_2 + LegPlaneTmpAngle1 + LegPlaneTmpAngle2;
	angle.delta   = ikAcos((sqL1 + sqL2 - sqLxyz) / (2 * l.size.l1 * l.size.l2));
	angle.epsilon = (M_PI_2 - LegPlaneTmpAngle1) + (M_PI - angle.delta - LegPlaneTmpAngle2);
	angle.zeta    = M_PI - angle.beta;	// backward to angle.beta

	s.isSolved = !_clamped;
	s.angle    = angle;

	return s;
}
