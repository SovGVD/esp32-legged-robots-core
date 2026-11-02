/**
 * This is not IK: tracks are driven straight from the move command.
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#include "ik.h"

iksolver TankDriveIK::solve(const leg &l, const LR_figure &body, const LR_moveVector &command)
{
	(void)body;
	LR_legAngle angle = {};
	iksolver s;

	// inverted because forward is -1, but logically it should be +1
	float left  = constrain(-(command.move.y + command.rotate.yaw), -1.0f, 1.0f);
	float right = constrain(-(command.move.y - command.rotate.yaw), -1.0f, 1.0f);

	if (l.id.id == 0) {
		angle.alpha = (left >= 0 ?  left : 0) * l.max.alpha;
		angle.beta  = (left <  0 ? -left : 0) * l.max.alpha;
	} else if (l.id.id == 1) {
		angle.alpha = (right >= 0 ?  right : 0) * l.max.beta;
		angle.beta  = (right <  0 ? -right : 0) * l.max.beta;
	}

	s.isSolved = true;
	s.angle    = angle;

	return s;
}
