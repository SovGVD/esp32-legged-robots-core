/**
 * Motion pipeline, run every LOOP_TIME on the motor core. The pipeline itself
 * (gait -> body kinematics -> IK) is MotionController in lib/robot_core, the
 * same code the simulator runs; this file feeds it the command and IMU.
 */
#include "robot.h"

static MotionController motion;

#ifdef HORIZON_LEVEL
	// Gains were tuned per loop tick; convert to per second for the dt based PID
	static AnglePID imuCorrectionPID(PID_LEVEL_P, PID_LEVEL_I / LOOP_DT, PID_LEVEL_D * LOOP_DT, -0.2, 0.2);
	static LR_angle imuTarget = {0.0, 0.0, 0.0};
#endif

void initMotion()
{
	Serial.println("Init gait");
	motion.begin(GAIT_PARAMS, legs, LEG_NUM, MODEL_IK);
}

const LR_figure &motionBody()
{
	return motion.body();
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

	motion.update(LOOP_DT, getMoveCommand(), imuCorrection);
}
