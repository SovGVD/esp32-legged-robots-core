#include "motion_controller.h"

void MotionController::begin(const GaitParams &params, leg *legs, uint8_t legCount, LegIK &ik,
                             uint8_t dof, const servoProfile *profile)
{
	_legs     = legs;
	_legCount = legCount;
	_ik       = &ik;
	_dof      = dof;
	_profile  = profile;
	_unreachable = _jointLimit = _servoRange = 0;
	_height = _heightTarget = 0;
	_gait.begin(params, legs, legCount);
}

uint8_t MotionController::update(float dt, const LR_moveVector &command, const LR_angle &correction)
{
	_gait.update(dt, command);

	float step = _heightRate * dt;
	float d    = _heightTarget - _height;
	_height    = (d > step) ? _height + step : (d < -step) ? _height - step : _heightTarget;
	_body = _gait.body();
	_body.position.z += _height;

	_bodyKinematics.update(_legs, _legCount, _body, correction, command);
	// the IK works in the tilted body frame (LegIK::legVector)
	LR_figure ikBody = _body;
	ikBody.orientation.pitch = _bodyKinematics.tilt().pitch;
	ikBody.orientation.roll  = _bodyKinematics.tilt().roll;

	uint8_t unsolved = 0;
	for (uint8_t i = 0; i < _legCount; i++) {
		iksolver solution = _ik->solve(_legs[i], ikBody, command);
		// Out of reach: keep the last valid angles instead of a pose bent by
		// the solver's clamping
		if (!solution.isSolved) {
			_unreachable++;
			unsolved++;
			continue;
		}
		// Joint and servo limits: clamp here, where it is counted, so the
		// angles are what the leg really does (the pulse is no longer
		// clamped silently later)
		if (lrClampJointLimits(_legs[i], solution.angle, _dof)) {
			_jointLimit++;
		}
		if (_profile && lrClampServoRange(_legs[i], solution.angle, _dof, *_profile)) {
			_servoRange++;
		}
		_legs[i].angle = solution.angle;
	}
	return unsolved;
}
