/**
 * Body state recalculate: where each shoulder is in world coordinates for a
 * given body pose, command tilt and IMU correction.
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#ifndef LR_BODY_KINEMATICS_H
#define LR_BODY_KINEMATICS_H

#include "leg.h"

class BodyKinematics
{
	public:
		/**
		 * How much of the pitch/roll command (normalized [-1, 1]) is applied as
		 * body tilt, in radians.
		 */
		float commandTilt = 0.2f;

		void update(leg *legs, uint8_t legCount, const LR_figure &body, const LR_angle &correction, const LR_moveVector &command);
		/** Body tilt the last update() used (pitch, roll; yaw unused), rad: the IK needs it too. */
		const LR_angle &tilt() const { return _tilt; }

	private:
		LR_angle _tilt = { 0, 0, 0 };
};

#endif
