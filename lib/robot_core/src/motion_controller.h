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
#include "servo_hal.h"

class MotionController
{
	public:
		/**
		 * @param dof     joints per leg checked against limits
		 * @param profile servo calibration; when given, joints are clamped to
		 *                what the servos can reach. nullptr: no servo check (tank)
		 */
		void begin(const GaitParams &params, leg *legs, uint8_t legCount, LegIK &ik,
		           uint8_t dof = 3, const servoProfile *profile = nullptr);

		/**
		 * @param dt         seconds since the previous update
		 * @param command    normalized move command, see LR_moveVector
		 * @param correction body tilt correction (balance), rad
		 * @return number of legs without an IK solution (out of reach); they
		 *         keep their previous angles. Joints beyond `leg.min/max` or the
		 *         servo range are clamped to the limit and counted separately.
		 */
		uint8_t update(float dt, const LR_moveVector &command, const LR_angle &correction);

		/**
		 * Lower (negative) or raise the body by `mm`, approached at `rate`
		 * mm/s; the feet stay where they are. Used to sit down.
		 */
		void setHeightOffset(float mm, float rate = 20.0f) { _heightTarget = mm; _heightRate = rate; }
		float heightOffset() const { return _height; }
		bool  heightSettled() const { return _height == _heightTarget; }

		const LR_figure &body() const { return _body; }
		GaitEngine &gait() { return _gait; }

		/** Since begin(): legs out of reach (kept), legs clamped to joint limits / servo range. */
		uint32_t unreachableCount() const { return _unreachable; }
		uint32_t jointLimitCount()  const { return _jointLimit; }
		uint32_t servoRangeCount()  const { return _servoRange; }

	private:
		GaitEngine     _gait;
		BodyKinematics _bodyKinematics;
		leg           *_legs     = nullptr;
		uint8_t        _legCount = 0;
		LegIK         *_ik       = nullptr;
		uint8_t        _dof      = 3;
		const servoProfile *_profile = nullptr;
		uint32_t       _unreachable = 0;
		uint32_t       _jointLimit  = 0;
		uint32_t       _servoRange  = 0;
		LR_figure      _body        = {};
		float          _height       = 0;
		float          _heightTarget = 0;
		float          _heightRate   = 20.0f;
};

#endif
