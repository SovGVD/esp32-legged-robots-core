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

MotionStatus motionStatus()
{
	MotionStatus st = {};
	st.gaitActive = drive.left() != 0 || drive.right() != 0;
	st.sitting    = batteryState() == BATTERY_CRITICAL;
	return st;
}

void updateMotion(float dt)
{
	// battery empty: motors off for good (BatteryMonitor latches it)
	if (!isHALEnabled() || batteryState() == BATTERY_CRITICAL) {
		drive.stop();
	} else {
		LR_moveVector cmd = getMoveCommand();
		// forward is -1 on the web client
		drive.update(dt, -cmd.move.y, -cmd.rotate.yaw);
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

/**
 * CLI `gait state`: the gait the motion loop really runs, to tell whether a
 * flashed build has the expected parameters.
 */
void printGaitState()
{
	const GaitParams &p = motion.gait().params();
	cliSerial->printf("Gait: %s%s, cycle %.2f s, duty %.2f, step height %.0f mm\n",
	                  p.staticStability ? "walk (static)" : "trot (dynamic)",
	                  pendingGait != nullptr ? ", switch pending" : "",
	                  p.cycleTime, p.dutyFactor, p.stepHeight);
	cliSerial->printf("Max speed %.0f mm/s, side %.0f mm/s, yaw %.2f rad/s, CoM {%.0f, %.0f}\n",
	                  p.maxSpeed, p.maxSideSpeed, p.maxYawRate, p.centerOfMass.x, p.centerOfMass.y);
	LR_point v = motion.gait().velocity();
	cliSerial->printf("Now: %s, velocity {%.0f, %.0f} mm/s, yaw %.2f rad/s\n",
	                  motion.gait().isActive() ? "stepping" : "standing", v.x, v.y, motion.gait().yawRate());
}

static volatile bool sitting = false;

MotionStatus motionStatus()
{
	MotionStatus st;
	st.gaitActive  = motion.gait().isActive();
	st.staticGait  = motion.gait().params().staticStability;
	st.sitting     = sitting;
	st.unreachable = motion.unreachableCount();
	st.jointLimit  = motion.jointLimitCount();
	st.servoRange  = motion.servoRangeCount();
	return st;
}

/**
 * Battery critical: ignore the client, let the gait put every foot down,
 * lower the body by SIT_HEIGHT, then turn the servos off so the robot rests
 * low instead of dropping from full height when the battery cuts out.
 * Returns the command to use.
 */
static LR_moveVector batteryGuard(const LR_moveVector &cmd)
{
	static uint32_t settledAt = 0;
	static bool     servosOff = false;

	if (batteryState() != BATTERY_CRITICAL) {
		return cmd;
	}
	LR_moveVector stop = { { 0, 0, 0 }, { 0, 0, 0 } };
	sitting = true;
	setLeveling(false);
	if (!motion.gait().isActive()) {
		motion.setHeightOffset(SIT_HEIGHT, 20.0f);
	}
	if (!servosOff && motion.heightOffset() == SIT_HEIGHT) {
		if (settledAt == 0) {
			settledAt = millis() | 1;
		} else if (millis() - settledAt > 1000) {
			disableServos();
			servosOff = true;
		}
	}
	return stop;
}

void updateMotion(float dt)
{
	if (!isHALEnabled()) {
		return;
	}

	// IMU leveling: tilt the body against the measured slope. Held during a
	// dynamic gait (trot), where the body rocks every step.
	// the IMU is read every tick anyway: the gyro integration needs it
	if (imu().ready()) {
		uint32_t t = micros();
		updateIMU();
		loopProfileMark(LOOP_STAGE_IMU, micros() - t);
	}
	leveler.setEnabled(levelRequest && imu().ready());
	static const LR_angle levelTarget = { 0, 0, 0 };
	bool hold = motion.gait().isActive() && !motion.gait().params().staticStability;
	imuCorrection = leveler.update(dt, IMU_DATA, levelTarget, hold);

	// switch gaits only while standing, so no leg is in the air
	const GaitParams *next = pendingGait;
	if (next != nullptr && !motion.gait().isActive()) {
		motion.gait().setParams(*next);
		pendingGait = nullptr;
	}

	motion.update(dt, batteryGuard(getMoveCommand()), imuCorrection);
}

#endif
