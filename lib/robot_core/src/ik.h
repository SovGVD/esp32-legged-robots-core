/**
 * Inverse Kinematics
 *
 * One solver per leg *type*. Every solver receives the leg (its shoulder
 * position `leg.body` and foot target `leg.foot`, both in world coordinates),
 * the body pose in world coordinates and the move command.
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */

#ifndef LR_IK_H
#define LR_IK_H

#include "leg.h"

typedef struct {
	bool        isSolved;	// false when the target is out of reach (angles are clamped)
	LR_legAngle angle;
} iksolver;

class LegIK
{
	public:
		virtual ~LegIK() {}
		virtual iksolver solve(const leg &l, const LR_figure &body, const LR_moveVector &command) = 0;

	protected:
		bool _clamped = false;

		/**
		 * Foot relative to shoulder, with body yaw removed and the leg inverse
		 * flags applied. Pitch/roll are already applied to `leg.body`.
		 */
		LR_point legVector(const leg &l, const LR_figure &body);

		/**
		 * acos/asin are only defined on [-1, 1]. Outside of it (target out of
		 * reach, or 0/0 when the foot sits exactly on a joint) they return
		 * NaN, which used to end up as a servo pulse. Clamp instead and mark
		 * the solution as not solved.
		 */
		float clampUnit(float value);
		float ikAcos(float value) { return acosf(clampUnit(value)); }
		float ikAsin(float value) { return asinf(clampUnit(value)); }
};

/**
 * Joint angle limits from `leg.min` / `leg.max` for the first `dof` joints.
 * Clamps `angle` into them; returns true if any joint was clamped.
 */
bool lrClampJointLimits(const leg &l, LR_legAngle &angle, uint8_t dof);

/**
 * Quadruped, Boston Dynamics Spot style 3 DOF leg (see ik_quadruped.cpp).
 */
class QuadrupedLegIK : public LegIK
{
	public:
		iksolver solve(const leg &l, const LR_figure &body, const LR_moveVector &command) override;
};

/**
 * Hexapod coxa/femur/tibia 3 DOF leg (see ik_hexapod.cpp).
 */
class HexapodLegIK : public LegIK
{
	public:
		iksolver solve(const leg &l, const LR_figure &body, const LR_moveVector &command) override;
};

/**
 * Biped 6 DOF leg (see ik_biped.cpp).
 */
class BipedLegIK : public LegIK
{
	public:
		iksolver solve(const leg &l, const LR_figure &body, const LR_moveVector &command) override;
};

/**
 * Not an IK: two tracks driven directly from the move command.
 * Leg id 0 is the left track, 1 the right one.
 */
class TankDriveIK : public LegIK
{
	public:
		iksolver solve(const leg &l, const LR_figure &body, const LR_moveVector &command) override;
};

#endif
