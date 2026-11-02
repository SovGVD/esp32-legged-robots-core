/**
 * Motion pipeline, run every LOOP_TIME on the motor core:
 * gait engine (body pose + foot targets) -> body kinematics (shoulder
 * positions with tilt and IMU correction) -> IK (joint angles).
 */
#include "robot.h"

static GaitEngine     gaitEngine;
static BodyKinematics bodyKinematics;

#ifdef HORIZON_LEVEL
	// Gains were tuned per loop tick; convert to per second for the dt based PID
	static AnglePID imuCorrectionPID(PID_LEVEL_P, PID_LEVEL_I / LOOP_DT, PID_LEVEL_D * LOOP_DT, -0.2, 0.2);
	static LR_angle imuTarget = {0.0, 0.0, 0.0};
#endif

void initMotion()
{
	Serial.println("Init gait");
	gaitEngine.begin(GAIT_PARAMS, legs, LEG_NUM);
}

const LR_figure &motionBody()
{
	return gaitEngine.body();
}

void updateMotion()
{
	#ifdef HORIZON_LEVEL
		updateIMU();
		imuCorrection = imuCorrectionPID.update(IMU_DATA, imuTarget, LOOP_DT);
	#endif

	if (!isHALEnabled()) {
		return;
	}

	gaitEngine.update(LOOP_DT, vector);
	bodyKinematics.update(legs, LEG_NUM, gaitEngine.body(), imuCorrection, vector);

	for (uint8_t i = 0; i < LEG_NUM; i++) {
		iksolver solution = MODEL_IK.solve(legs[i], gaitEngine.body(), vector);
		// Out of reach: keep the last valid angles instead of a clamped pose
		if (solution.isSolved) {
			legs[i].angle = solution.angle;
		}
	}
}
