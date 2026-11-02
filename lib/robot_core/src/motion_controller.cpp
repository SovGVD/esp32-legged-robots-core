#include "motion_controller.h"

void MotionController::begin(const GaitParams &params, leg *legs, uint8_t legCount, LegIK &ik)
{
	_legs     = legs;
	_legCount = legCount;
	_ik       = &ik;
	_gait.begin(params, legs, legCount);
}

uint8_t MotionController::update(float dt, const LR_moveVector &command, const LR_angle &correction)
{
	_gait.update(dt, command);
	_bodyKinematics.update(_legs, _legCount, _gait.body(), correction, command);

	uint8_t unsolved = 0;
	for (uint8_t i = 0; i < _legCount; i++) {
		iksolver solution = _ik->solve(_legs[i], _gait.body(), command);
		// Out of reach: keep the last valid angles instead of a clamped pose
		if (solution.isSolved) {
			_legs[i].angle = solution.angle;
		} else {
			unsolved++;
		}
	}
	return unsolved;
}
