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

MotionOdometry motionOdometry()
{
	MotionOdometry o = {};
	return o;
}

float motionOdometryBias() { return 0; }

void cliTerrain() { cliSerial->println("Tracks: no terrain mode."); }
void cliHeading() { cliSerial->println("Tracks: no heading hold yet."); }
void cliTurnTo()  { cliSerial->println("Tracks: no heading hold yet."); }

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

// Rough ground: ToF terrain map (terrain mode, `terrain on`)
static TerrainMap  terrainMap;
static TerrainTilt terrainTilt;
static FrameTilt   frameTilt;     // the IMU tilt's offset from the floor the ToF sees
static bool        terrainOn = false;
static volatile int8_t terrainRequest = -1;   // 1 on, 0 off; applied while standing (core 0 -> motion task)
static uint32_t    tofSeq[2] = { 0, 0 };
static float       tofMm[64];

// Heading hold and magnetometer calibration (`heading`, `turnto`)
#ifndef HEADING_HOLD_PARAMS
	// gain 1/s, max turn rate rad/s, deadband standing rad (4 deg), stick deadband
	#define HEADING_HOLD_PARAMS { 1.5f, 0.6f, 0.07f, 0.05f }
#endif
#ifndef HEADING_CAL_TURN
	#define HEADING_CAL_TURN 0.5f   // yaw command while turning for the magnetometer calibration
#endif
enum HeadingCalState : uint8_t { CAL_IDLE, CAL_TURN, CAL_HAND, CAL_DONE, CAL_FAILED, CAL_STOPPED };
static HeadingHold      headingHold;
static volatile int8_t  holdRequest   = -1;       // 1 on, 0 off
static volatile bool    targetRequest = false;
static volatile float   targetValue   = 0;
static volatile int8_t  calRequest    = -1;       // CAL_TURN or CAL_HAND
static volatile uint8_t calState      = CAL_IDLE;
static float            calTurned     = 0;
static float            calTime       = 0;

// Odometry for the lidar (motionOdometry), written every tick
static Odometry2D     odometry;
static MotionOdometry odometryOut = {};
static portMUX_TYPE   odometryLock = portMUX_INITIALIZER_UNLOCKED;

MotionOdometry motionOdometry()
{
	portENTER_CRITICAL(&odometryLock);
	MotionOdometry o = odometryOut;
	portEXIT_CRITICAL(&odometryLock);
	return o;
}

float motionOdometryBias()
{
	return odometry.bias();   // a float: read whole on the ESP32
}

static void updateOdometry(float dt, float scanTilt)
{
	// the gait's velocity, turned by the gyro (the gait's own yaw is only what it was told)
	bool gyro = imu().ready();
	LR_angle tilt = { 0, scanTilt, 0 };
	float yawRate = motion.gait().yawRate();
	if (gyro) {
		tilt.pitch = IMU_DATA.pitch;
		tilt.roll  = IMU_DATA.roll;
		// the gyro's z axis tilts with the body (a ToF scan's nose up): a turn
		// about the vertical shows on it as rate * cos(tilt)
		yawRate = (imu().yawRate() - imu().gyroBias()) / (cosf(tilt.pitch) * cosf(tilt.roll));
	}
	bool standing = gyro && !motion.gait().isActive();   // still: the odometry learns the gyro's bias
	odometry.update(dt, motion.gait().velocity(), yawRate, standing);
	portENTER_CRITICAL(&odometryLock);
	odometryOut.pose       = odometry.pose();
	odometryOut.maxYawRate = motion.gait().params().maxYawRate;
	odometryOut.tilt       = tilt;
	odometryOut.floorZ     = legs[0].defaultFoot.z;   // TofNavigator adds its own crouch
	odometryOut.gaitActive = motion.gait().isActive();
	odometryOut.gapBlocked = terrainOn && motion.gait().isBlocked();
	portEXIT_CRITICAL(&odometryLock);
}

/**
 * Walking to a lidar map point (`goto`, web map view): the navigator's
 * command replaces the operator's. Moving the stick cancels it; without a
 * client (failsafe) the robot stands. Returns true while navigating.
 */
static bool navigate(LR_moveVector &cmd)
{
	LR_moveVector nav;
	if (!lidarNavCommand(nav)) {
		return false;
	}
	if (fabsf(cmd.move.x) > 0.1f || fabsf(cmd.move.y) > 0.1f || fabsf(cmd.rotate.yaw) > 0.1f) {
		lidarNavCancel();
		return false;
	}
	if (!FS_FAIL && batteryState() != BATTERY_CRITICAL) {
		cmd = nav;
	}
	return true;
}

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
	const HeadingHoldParams hp = HEADING_HOLD_PARAMS;
	headingHold.begin(hp);
	if (MODEL_TERRAIN.map != nullptr) {
		terrainMap.begin(*MODEL_TERRAIN.map);
		terrainTilt.begin(MODEL_TERRAIN.tilt);
	}
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
	st.terrain       = terrainOn;
	st.gapBlocked    = motion.gait().isBlocked();
	st.headingHold   = headingHold.enabled();
	st.headingTarget = headingHold.hasTarget();
	st.target        = headingHold.target();
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

/**
 * Terrain mode on/off, applied while standing (no foot in the air): the
 * rough gait (slower, higher steps), the ToF map, leveling on.
 */
static void applyTerrainRequest()
{
	int8_t req = terrainRequest;
	if (req < 0 || motion.gait().isActive() || MODEL_TERRAIN.terrain == nullptr) {
		return;
	}
	terrainRequest = -1;
	if (req == 1) {
		terrainMap.clear();
		terrainTilt.begin(MODEL_TERRAIN.tilt);
		frameTilt.reset();
		motion.gait().setParams(*MODEL_TERRAIN.roughGait);
		motion.gait().setTerrain(MODEL_TERRAIN.terrain, &terrainMap);
		levelRequest = true;   // the terrain code needs the body level (feet stand at different heights)
		terrainOn = true;
	} else {
		motion.gait().setTerrain(nullptr, nullptr);
		motion.gait().setParams(GAIT_PARAMS);
		terrainOn = false;
	}
}

/**
 * Terrain mode, every tick: the map tilt filter (and the map ages while
 * walking), new ToF frames into the map with this tick's body pose, and the
 * tilt the gait didn't plan for the touchdown heights. The same steps as
 * sim/bridge/robot_core_bridge.cpp, which the sim results come from.
 */
static void updateTerrain(float dt, float scanTilt)
{
	if (!terrainOn) {
		return;
	}
	// the commanded tilt includes a ToF room scan's nose up (BodyKinematics subtracts the correction)
	LR_angle commanded = imuCorrection;
	commanded.roll -= scanTilt;
	// the map ages only while walking (not turning around for a room scan: paused then, below)
	terrainTilt.update(dt, IMU_DATA, commanded, motion.gait().isActive() && scanTilt <= 0, &terrainMap);
	uint32_t t = micros();
	for (uint8_t i = 0; i < tofSensorCount() && i < 2; i++) {
		if (!tofTakeFrame(i, tofSeq[i], tofMm) || scanTilt > 0) {
			// (not during a ToF room scan: the robot turns on the ground it has mapped,
			// and the nose up it may not quite reach would place the readings wrong)
			continue;
		}
		const TofSensor &sensor = MODEL_TERRAIN.tof[i];
		LR_figure pose = motion.body();
		// the floor's own tilt in this frame where there is a floor: the IMU's is
		// 1-2 deg off while trotting, which made the map creep up on long walks
		LR_angle tilt = frameTilt.place(sensor, tofMm, terrainTilt.tilt());
		pose.orientation.pitch = tilt.pitch;
		pose.orientation.roll  = tilt.roll;
		// and at the height the map already has: the body is only planned to be at pose z
		pose.position.z -= terrainMap.frameOffset(sensor, tofMm, pose);
		for (uint8_t z = 0; z < sensor.zones * sensor.zones && z < 64; z++) {
			terrainMap.addReading(sensor, z, tofMm[z], pose);
		}
	}
	loopProfileMark(LOOP_STAGE_TOF, micros() - t);
	// tilt the gait didn't ask for: measured minus commanded (-correction)
	LR_angle unplanned = { IMU_DATA.pitch + commanded.pitch, IMU_DATA.roll + commanded.roll, 0 };
	motion.gait().setTilt(unplanned);
}

/**
 * Heading: the magnetometer calibration turn, and the heading hold, which
 * writes the yaw of the move command. Nothing turns on its own while the
 * failsafe is on (no client): the hold pauses and the calibration turn stops.
 */
static LR_moveVector steer(float dt, LR_moveVector cmd)
{
	if (holdRequest >= 0) {
		headingHold.setEnabled(holdRequest == 1);
		holdRequest = -1;
	}
	if (targetRequest) {
		headingHold.setEnabled(true);
		headingHold.setTarget(targetValue);
		targetRequest = false;
	}
	if (calRequest >= 0) {
		calState  = calRequest;
		calRequest = -1;
		calTurned = 0;
		calTime   = 0;
		imu().magCalibrationStart();
	}

	if (calState == CAL_TURN || calState == CAL_HAND) {
		calTime   += dt;
		calTurned += imu().yawRate() * dt;
		bool turned = fabsf(calTurned) >= 4 * PI;   // two full turns
		if (calState == CAL_TURN) {
			if (FS_FAIL || batteryState() == BATTERY_CRITICAL) {
				calState = CAL_STOPPED;
				Serial.println("Magnetometer calibration stopped (no client / battery)");
				return cmd;
			}
			LR_moveVector turn = { { 0, 0, 0 }, { 0, 0, HEADING_CAL_TURN } };
			cmd = turn;
		}
		if (turned || calTime > 60) {
			bool turning = calState == CAL_TURN;
			bool ok = turned && imu().magCalibrationFinish();
			calState = ok ? CAL_DONE : CAL_FAILED;
			Serial.printf("Magnetometer calibration %s (turned %.0f deg)\n", ok ? "saved" : "failed", calTurned * RADTODEG);
			LR_moveVector stop = { { 0, 0, 0 }, { 0, 0, 0 } };
			return turning ? stop : cmd;
		}
		return cmd;
	}

	float heading;
	if (FS_FAIL || !imu().heading(heading)) {
		return cmd;
	}
	bool moving = fabsf(cmd.move.x) > 0.05f || fabsf(cmd.move.y) > 0.05f;
	cmd.rotate.yaw = headingHold.update(cmd.rotate.yaw, heading, motion.gait().params().maxYawRate, moving);
	return cmd;
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
	// a ToF room scan wants the nose up: not something to level away
	float wantTilt = 0, wantCrouch = 0;
	lidarNavBody(wantTilt, wantCrouch);
	const LR_angle levelTarget = { 0, wantTilt, 0 };
	// (on rough ground it keeps working: the feet stand at different heights)
	bool hold = motion.gait().isActive() && !motion.gait().params().staticStability && !terrainOn;
	imuCorrection = leveler.update(dt, IMU_DATA, levelTarget, hold);

	// switch gaits only while standing, so no leg is in the air
	const GaitParams *next = pendingGait;
	if (next != nullptr && !motion.gait().isActive()) {
		motion.gait().setParams(*next);
		pendingGait = nullptr;
	}

	applyTerrainRequest();
	updateTerrain(dt, batteryState() != BATTERY_CRITICAL ? wantTilt : 0);

	LR_moveVector cmd = getMoveCommand();
	if (!navigate(cmd)) {
		cmd = steer(dt, cmd);   // heading hold: not while the navigator steers
	}
	// ToF room scan: nose up (and the body a little lower) while it turns around
	LR_angle correction = imuCorrection;
	float scanTilt = 0, crouch = 0;
	if (lidarNavBody(scanTilt, crouch) && batteryState() != BATTERY_CRITICAL) {
		correction.roll -= scanTilt;   // BodyKinematics subtracts the correction: nose up
		motion.setHeightOffset(-crouch, 200.0f);
	}
	motion.update(dt, batteryGuard(cmd), correction);
	updateOdometry(dt, scanTilt);
}

void cliTerrain()
{
	char *mode = CLI_readWord();
	if (MODEL_TERRAIN.terrain == nullptr) {
		cliSerial->println("This model has no terrain mode.");
		return;
	}
	if (strcmp(mode, TITLE_ON) == 0) {
		if (!imu().ready()) {
			cliSerial->println("No IMU: terrain mode needs leveling, stays off.");
			return;
		}
		if (!tofReady(0)) {
			cliSerial->println("Warning: no ToF sensor running, the map stays empty (`tof state`).");
		}
		terrainRequest = 1;
		cliSerial->println("Terrain mode on once the robot stands still: rough gait, ToF map, leveling on.");
	} else if (strcmp(mode, TITLE_OFF) == 0) {
		terrainRequest = 0;
		cliSerial->println("Terrain mode off once the robot stands still.");
	}
	cliSerial->printf("Terrain: %s%s, map %u cells, tilt mode %u\n", terrainOn ? TITLE_ON : TITLE_OFF,
	                  terrainRequest >= 0 ? " (switch pending)" : "", terrainMap.knownCells(), (unsigned)terrainTilt.mode());
	cliSerial->printf("Gap ahead: %s\n", motion.gait().isBlocked() ? "yes, stopped in that direction (let go of the stick)" : "no");
	for (uint8_t i = 0; i < tofSensorCount(); i++) {
		cliSerial->printf("ToF %u: %s\n", i, tofReady(i) ? "running" : "not running");
	}
}

static const char *const calStateNames[] = { "not run", "turning", "turn it by hand", "saved", "failed", "stopped" };

void cliHeading()
{
	char *mode = CLI_readWord();
	if (!imu().ready()) {
		cliSerial->println("No IMU, no heading.");
		return;
	}
	if (strcmp(mode, TITLE_ON) == 0) {
		holdRequest = 1;
	} else if (strcmp(mode, TITLE_OFF) == 0) {
		holdRequest = 0;
	} else if (strcmp(mode, "cal") == 0 || strcmp(mode, "hand") == 0) {
		if (!imu().hasMagnetometer()) {
			cliSerial->println("No magnetometer (an MPU6500 board?): nothing to calibrate.");
			return;
		}
		bool hand = strcmp(mode, "hand") == 0;
		if (!hand && FS_FAIL) {
			cliSerial->println("Connect the web client first (failsafe stops the turn), or use `heading hand`.");
			return;
		}
		calRequest = hand ? CAL_HAND : CAL_TURN;
		cliSerial->println(hand ? "Turn the robot slowly two full turns on flat ground (60 s)."
		                        : "Turning two full turns in place on flat ground, away from steel. Stop: `hal off`.");
		return;
	}
	float h = 0;
	imu().heading(h);
	bool accepted;
	float field = imu().magField(accepted);
	cliSerial->printf("Heading %.1f deg (%s)\n", h * RADTODEG,
	                  imu().headingAbsolute() ? "magnetic north" : "relative to the start, gyro only");
	cliSerial->printf("Magnetometer: %s, field %.0f uT %s, calibration %s\n",
	                  imu().hasMagnetometer() ? "yes" : "no", field, accepted ? "(used)" : "(ignored)", calStateNames[calState]);
	cliSerial->printf("Hold: %s", headingHold.enabled() ? TITLE_ON : TITLE_OFF);
	if (headingHold.enabled() && headingHold.hasTarget()) {
		cliSerial->printf(", target %.1f deg, error %.1f deg", headingHold.target() * RADTODEG, headingHold.error() * RADTODEG);
	}
	cliSerial->println(FS_FAIL ? " (failsafe: no client, not turning)" : "");
}

void cliTurnTo()
{
	float deg = CLI_readFloat();
	if (!imu().ready()) {
		cliSerial->println("No IMU, no heading.");
		return;
	}
	targetValue = lrWrapAngle(deg * DEGTORAD);
	if (targetValue < 0) {
		targetValue += 2 * PI;
	}
	targetRequest = true;
	cliSerial->printf("Turning to %.1f deg%s and holding it (`heading off` to stop).\n", deg,
	                  imu().headingAbsolute() ? "" : " relative to the start (no magnetometer heading)");
}

#endif
