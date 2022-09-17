/**
 * Transitions for leg movement
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2020
 */

#include "transition.h"

transition::transition()
{
}

void transition::set(transitionParameters param)
{
	_param = param;

	dPx = _param.targetValue.position.x - _param.initialValue.position.x;
	dPy = _param.targetValue.position.y - _param.initialValue.position.y;
	dPz = _param.targetValue.position.z - _param.initialValue.position.z;

	dOp = _param.targetValue.orientation.pitch - _param.initialValue.orientation.pitch;
	dOr = _param.targetValue.orientation.roll  - _param.initialValue.orientation.roll;
	dOy = _param.targetValue.orientation.yaw   - _param.initialValue.orientation.yaw;

	P_z1 = _param.initialValue.position.z + _param.offTheGround;
	P_z2 = _param.targetValue.position.z  + _param.offTheGround;
}


/**
 * progress [0,1];
 */
LR_figure transition::swing(double progress)
{
	f.position.x = progress*dPx + _param.initialValue.position.x;
	f.position.y = progress*dPy + _param.initialValue.position.y;
	
	if (progress <= _param.transitionProgressStep[0]) {
		stepProgress = progress/_param.transitionProgressStep[0];
		dPz = P_z1 - _param.initialValue.position.z;
		z = _param.initialValue.position.z;

	} else if (progress <= _param.transitionProgressStep[1]) {
		stepProgress = (progress - _param.transitionProgressStep[0])/(_param.transitionProgressStep[1] - _param.transitionProgressStep[0]);
		dPz = P_z2 - P_z1;
		z   = P_z1;

	} else {
		stepProgress = (progress - _param.transitionProgressStep[1])/(1 - _param.transitionProgressStep[1]);
		dPz = _param.targetValue.position.z - P_z2;
		z   = P_z2;

	}
	f.position.z = stepProgress*dPz + z;

	return f;
}

LR_figure transition::linear(double progress)
{
	f.position.x = progress*dPx + _param.initialValue.position.x;
	f.position.y = progress*dPy + _param.initialValue.position.y;
	f.position.z = progress*dPz + _param.initialValue.position.z;

	f.orientation.pitch = progress*dOp + _param.initialValue.orientation.pitch;
	f.orientation.roll  = progress*dOr + _param.initialValue.orientation.roll;
	f.orientation.yaw   = progress*dOy + _param.initialValue.orientation.yaw;

	return f;
}
