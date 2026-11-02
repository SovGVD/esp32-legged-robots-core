/**
 * Motion pipeline, run every LOOP_TIME on the motor core. The pipeline itself
 * (gait -> body kinematics -> IK) is MotionController in lib/robot_core, the
 * same code the simulator runs; this file feeds it the command and IMU.
 */
#include "robot.h"

static MotionController motion;
static const GaitParams * volatile pendingGait = nullptr;   // set from the CLI (core 0)

#ifdef HORIZON_LEVEL
	// Gains were tuned per loop tick; convert to per second for the dt based PID
	static AnglePID imuCorrectionPID(PID_LEVEL_P, PID_LEVEL_I / LOOP_DT, PID_LEVEL_D * LOOP_DT, -0.2, 0.2);
	static LR_angle imuTarget = {0.0, 0.0, 0.0};
#endif

#if PWM_CONTROLLER_TYPE == DCMOTOR
/**
 * Tracks: no gait or IK, the move command is mixed straight into track
 * speeds and written as H-bridge duties (servoPulse holds duties here).
 */
static DiffDrive drive;

static void writeTrack(uint8_t id, const DiffDriveDuty &d)
{
	servoPulse[id][ALPHA] = d.in1;
	servoPulse[id][BETA]  = d.in2;
}

void initMotion()
{
	Serial.println("Init differential drive");
	drive.begin(TANK_DRIVE_PARAMS);
	writeTrack(TRACKL, drive.leftDuty());
	writeTrack(TRACKR, drive.rightDuty());
}

void requestGait(const GaitParams &params)
{
	(void)params;
}

const LR_figure &motionBody()
{
	static const LR_figure origin = {};
	return origin;
}

void updateMotion()
{
	if (!isHALEnabled()) {
		drive.stop();
	} else {
		LR_moveVector cmd = getMoveCommand();
		// forward is -1 on the web client
		drive.update(LOOP_DT, -cmd.move.y, -cmd.rotate.yaw);
	}
	writeTrack(TRACKL, drive.leftDuty());
	writeTrack(TRACKR, drive.rightDuty());
}

#else

void initMotion()
{
	Serial.println("Init gait");
	motion.begin(GAIT_PARAMS, legs, LEG_NUM, MODEL_IK, LEG_DOF, &servoMainProfile);
}

void requestGait(const GaitParams &params)
{
	pendingGait = &params;
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

	// switch gaits only while standing, so no leg is in the air
	const GaitParams *next = pendingGait;
	if (next != nullptr && !motion.gait().isActive()) {
		motion.gait().setParams(*next);
		pendingGait = nullptr;
	}

	motion.update(LOOP_DT, getMoveCommand(), imuCorrection);
}

#endif
