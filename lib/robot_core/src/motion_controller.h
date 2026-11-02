/**
 * The motion pipeline run every control tick:
 * gait engine (body pose, foot targets) -> body kinematics (shoulder
 * positions with tilt and balance correction) -> IK (joint angles).
 *
 * The firmware (motion.cpp) and the simulator bridge both run this class,
 * so the sim exercises the same code that drives the servos.
 */
#ifndef LR_MOTION_CONTROLLER_H
#define LR_MOTION_CONTROLLER_H

#include "gait_engine.h"
#include "body_kinematics.h"
#include "ik.h"

class MotionController
{
	public:
		void begin(const GaitParams &params, leg *legs, uint8_t legCount, LegIK &ik);

		/**
		 * @param dt         seconds since the previous update
		 * @param command    normalized move command, see LR_moveVector
		 * @param correction body tilt correction (balance), rad
		 * @return number of legs that had no IK solution and kept their previous angles
		 */
		uint8_t update(float dt, const LR_moveVector &command, const LR_angle &correction);

		const LR_figure &body() const { return _gait.body(); }
		GaitEngine &gait() { return _gait; }

	private:
		GaitEngine     _gait;
		BodyKinematics _bodyKinematics;
		leg           *_legs     = nullptr;
		uint8_t        _legCount = 0;
		LegIK         *_ik       = nullptr;
};

#endif
