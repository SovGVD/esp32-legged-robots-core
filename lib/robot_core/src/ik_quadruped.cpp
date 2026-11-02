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

#include "ik.h"

iksolver QuadrupedLegIK::solve(const leg &l, const LR_figure &body, const LR_moveVector &command)
{
	(void)command;
	LR_legAngle angle = {};
	iksolver s;
	_clamped = false;

	LR_point v = legVector(l, body);
	float lx = v.x;
	float ly = v.y;
	float lz = v.z;

	float a     = sq(lx) + sq(lz);   // square of hypotenuse (points between leg.body and leg.foot in XZ-plane)
	float sqrta = sqrtf(a);
	float b     = sq(ly) + a;        // square of hypotenuse (points between leg.body and leg.foot in YLeg-plane)
	float sqrtb = sqrtf(b);
	float l2p2  = sq(l.size.l2);
	float l3p2  = sq(l.size.l3);

	// XZ plane
	angle.alpha = M_PI_2 - ikAsin(lx/sqrta);

	// YLeg plane
	angle.beta  = M_PI_2 - ikAsin(ly/sqrtb) - ikAcos((b + l2p2 - l3p2) / (2 * sqrtb * l.size.l2));

	// YLeg plane
	angle.gamma = ikAcos((l2p2 + l3p2 - b) / (2 * l.size.l2 * l.size.l3));

	s.isSolved = !_clamped;
	s.angle    = angle;

	return s;
}
