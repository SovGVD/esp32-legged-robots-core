/**
 * Motion pipeline, run every LOOP_TIME on the motor core. The pipeline itself
 * (gait -> body kinematics -> IK) is MotionController in lib/robot_core, the
 * same code the simulator runs; this file feeds it the command and IMU.
 */
#include "robot.h"

static MotionController motion;
static const GaitParams * volatile pendingGait = nullptr;   // set from the CLI (core 0)


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

// tracks don't level
void setLeveling(bool on)
{
	(void)on;
}

bool levelingEnabled()
{
	return false;
}

LR_angle levelCorrection()
{
	LR_angle zero = { 0, 0, 0 };
	return zero;
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

static BodyLeveler leveler;
static volatile bool levelRequest =
#ifdef HORIZON_LEVEL
	true;
#else
	false;
#endif

void setLeveling(bool on)
{
	levelRequest = on;
}

bool levelingEnabled()
{
	return levelRequest;
}

LR_angle levelCorrection()
{
	return leveler.correction();
}

void initMotion()
{
	Serial.println("Init gait");
	leveler.begin(LEVEL_PARAMS);
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
	if (!isHALEnabled()) {
		return;
	}

	// IMU leveling: tilt the body against the measured slope. Held during a
	// dynamic gait (trot), where the body rocks every step.
	// the IMU is read every tick anyway: the gyro integration needs it
	if (imu().ready()) {
		updateIMU();
	}
	leveler.setEnabled(levelRequest && imu().ready());
	static const LR_angle levelTarget = { 0, 0, 0 };
	bool hold = motion.gait().isActive() && !motion.gait().params().staticStability;
	imuCorrection = leveler.update(LOOP_DT, IMU_DATA, levelTarget, hold);

	// switch gaits only while standing, so no leg is in the air
	const GaitParams *next = pendingGait;
	if (next != nullptr && !motion.gait().isActive()) {
		motion.gait().setParams(*next);
		pendingGait = nullptr;
	}

	motion.update(LOOP_DT, getMoveCommand(), imuCorrection);
}

#endif
