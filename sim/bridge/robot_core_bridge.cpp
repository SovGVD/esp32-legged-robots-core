/**
 * C API around the firmware motion code, built for the host as a shared
 * library and loaded by the simulator (sim/legsim/firmware.py, ctypes).
 *
 * Everything between the move command and the servo pulse widths is the
 * firmware's own code: MotionController (gait engine, body kinematics, IK)
 * and the servo HAL from lib/robot_core, with the dog's legs, servo profile
 * and gait parameters from src/esp32-legged-robot/model/<model>/model_data.h
 * (ROBOT_MODEL: MODEL_DOG or MODEL_HEXAPOD).
 * Only the hardware is replaced: pulses go to SimDog.write_pulses() and the
 * IMU angles come from the sim.
 */
#include <string.h>
#include <robot_core.h>
#include "def/def.h"

#ifndef ROBOT_MODEL
	#define ROBOT_MODEL MODEL_DOG
#endif
#if ROBOT_MODEL == MODEL_DOG
	#include "model/dog/config.h"
	#include "model/dog/model_data.h"
	static QuadrupedLegIK modelIK;
	#define BRIDGE_HAS_WALK
#elif ROBOT_MODEL == MODEL_HEXAPOD
	#include "model/hexapod/config.h"
	#include "model/hexapod/model_data.h"
	static HexapodLegIK modelIK;
#else
	#error "The simulator bridge supports the dog and hexapod models only"
#endif

#define BRIDGE_LOOP_DT (LOOP_TIME / 1000000.0f)

static MotionController motion;
static const GaitParams *gaitParams = &GAIT_PARAMS;
static LR_angle         imuCorrection = { 0, 0, 0 };

static BodyLeveler      leveler;
#ifdef HORIZON_LEVEL
	static bool levelOn = true;
#else
	static bool levelOn = false;
#endif

// Rough ground: foot switches and ToF terrain map, both off by default and
// kept across rc_init() (like leveling)
static TerrainMap terrainMap;
static bool contactOn = false;
static bool terrainOn = false;
static bool mapOn     = false;
static LR_angle lastImu = { 0, 0, 0 };
static float tiltGain = 1.0f;
// body tilt for the terrain map and its aging, the same code as motion.cpp
// (TERRAIN_TILT_MODE in model_data.h; rc_set_map_tilt_mode for experiments)
static TerrainTilt terrainTilt;
static TerrainTiltMode mapTiltMode = TERRAIN_TILT_MODE;
static bool floorTiltOn = true;    // ToF frames placed with the floor's own tilt (FrameTilt); rc_set_floor_tilt
static FrameTilt frameTilt;
static float frameGain = 1.0f;    // FrameTilt gain; rc_set_frame_gain
static float regDeadband = 2.0f;
static bool registerOn = true;    // frames placed at the map's height (TerrainMap::frameOffset); rc_set_frame_register
static float biasTau = 1.5f;
static float unplannedTau = 0.3f;
// Heading (gyro + magnetometer) and heading hold, as in sensors.cpp / motion.cpp
static HeadingEstimator headingEst;
static HeadingHold      headingHold;
static bool             headingFed = false;
static const HeadingHoldParams HEADING_HOLD = { 1.5f, 0.6f, 0.07f, 0.05f };   // motion.cpp HEADING_HOLD_PARAMS
// Lidar mapping and walking to a point, as in lidar.cpp / motion.cpp
static Ld06Parser       lidarParser;
static LidarScanBuilder lidarScans;
static LidarSlam        slam;
static Navigator        nav;
static Odometry2D       odom;
static LR_pose2         scanStartOdom = { 0, 0, 0 };
static LR_moveVector    navCmd = {};
static bool             lidarOn = false;
static float            lastYawRate = 0;
static float            lastGyro[3] = { 0, 0, 0 };   // body frame, rad/s (rc_heading_feed)
static float            simTime = 0;
static float            lastScanTime = 0;
// ToF room scans instead of the lidar (tof_scan.h), as in lidar.cpp / motion.cpp without LIDAR_TYPE
static TofNavigator     tofNav;
static TofScanParams    tofParams = TOF_SCAN_PARAMS;   // tunable from Python (rc_tofnav_tune)
static NavParams        navParams = NAV_PARAMS;        // tunable from Python (rc_nav_tune)
static bool             tofNavOn = false;
static LR_moveVector    tofCmd = {};
static const LidarParams TOF_ORIGIN = { { 0, 0, 0 }, 0, false, 0, 0, 0 };   // the map's beams start at the body origin
// TERRAIN_PARAMS, tunable from Python (rc_tune_terrain); kept across rc_init()
static TerrainParams terrainParams = TERRAIN_PARAMS;

static void applyTerrain()
{
	GaitParams p = motion.gait().params();
	p.useContactSensors = contactOn;
	motion.gait().setParams(p);
	motion.gait().setTerrain(terrainOn ? &terrainParams : nullptr, mapOn ? &terrainMap : nullptr);
}

// Pristine copy of the model legs, so rc_init() can start over
alignas(leg) static unsigned char initialLegs[sizeof(legs)];
static bool initialSaved = false;

#define RC_API extern "C" __attribute__((visibility("default")))

RC_API int rc_leg_count() { return LEG_NUM; }
RC_API int rc_leg_dof()   { return LEG_DOF; }
RC_API float rc_loop_dt() { return BRIDGE_LOOP_DT; }

/**
 * Experiment: the neutral stance's feet |x| mm out from the centre and z mm
 * down (< 0: keep), all legs (config.h LEG_POINT_X / LEG_POINT_Z). Call rc_init() after.
 */
RC_API void rc_set_stance(float x, float z)
{
	if (!initialSaved) {
		memcpy((void *)initialLegs, (const void *)legs, sizeof(legs));
		initialSaved = true;
	}
	leg *l = (leg *)(void *)initialLegs;
	for (int i = 0; i < LEG_NUM; i++) {
		LR_point *d = (LR_point *)(void *)&l[i].defaultFoot;   // const in the model: the saved copy is ours
		if (x >= 0) d->x = l[i].foot.x = d->x < 0 ? -x : x;
		if (z >= 0) d->z = l[i].foot.z = -z;
	}
}

/**
 * (Re)start: legs back to the model's defaults, gait idle, joint angles
 * solved for the neutral stance.
 */
RC_API void rc_init()
{
	if (!initialSaved) {
		memcpy((void *)initialLegs, (const void *)legs, sizeof(legs));
		initialSaved = true;
	} else {
		memcpy((void *)legs, (const void *)initialLegs, sizeof(legs));
	}
	imuCorrection = { 0, 0, 0 };
	leveler.begin(LEVEL_PARAMS);
	leveler.setEnabled(levelOn);
	motion.begin(*gaitParams, legs, LEG_NUM, modelIK, LEG_DOF, &servoMainProfile);
	terrainMap.begin(TERRAIN_MAP_PARAMS);
	lastImu = { 0, 0, 0 };
	terrainTilt.begin(mapTiltMode, biasTau, unplannedTau);
	frameTilt.reset();
	headingEst.begin({ 2.0f, 0.25f });   // sensors.cpp HEADING_MAG_TAU, HEADING_FIELD_TOLERANCE
	headingHold.begin(HEADING_HOLD);
	headingFed = false;
	odom = Odometry2D();
	scanStartOdom = { 0, 0, 0 };
	navCmd = {};
	nav.stop();
	simTime = lastScanTime = 0;
	if (lidarOn) {
		slam.reset();
		lidarParser.reset();
		lidarScans.reset();
	}
	if (tofNavOn) {
		slam.reset();
		tofNav.begin(tofParams, navParams);
		tofCmd = {};
	}
	applyTerrain();
	LR_moveVector zero = {};
	motion.update(0.0f, zero, imuCorrection);
}

/**
 * One control tick, like updateMotion() + servoSet() in the firmware.
 *
 * @param dt      seconds since the previous tick (firmware: rc_loop_dt())
 * @param command 6 floats in [-1, 1]: move x, y, z, rotate pitch, roll, yaw
 * @param imu     3 floats, body pitch, roll, yaw in rad in the firmware
 *                convention (pitch about forward Y), used by the leveler
 *                (rc_set_leveling), may be NULL
 * @param pulses  out, LEG_NUM * LEG_DOF pulse widths in us, leg*dof + joint
 * @return        number of legs without an IK solution this tick
 */
RC_API int rc_update(float dt, const float *command, const float *imu, uint16_t *pulses)
{
	LR_moveVector cmd = {
		{ command[0], command[1], command[2] },
		{ command[3], command[4], command[5] }
	};
	simTime += dt;
	// walking to a point (motion.cpp navigate()): the stick takes over and cancels it
	bool navigating = false;
	bool stick = fabsf(cmd.move.x) > 0.1f || fabsf(cmd.move.y) > 0.1f || fabsf(cmd.rotate.yaw) > 0.1f;
	if (nav.active()) {
		if (stick) {
			nav.stop();
		} else {
			cmd = navCmd;
			navigating = true;
		}
	}
	// ToF room scan / walking to a point (lidar.cpp tofNavTask + motion.cpp navigate())
	float scanTilt = 0;
	if (tofNavOn) {
		if (tofNav.active() && stick) {
			tofNav.stop();
		}
		if (terrainOn && motion.gait().isBlocked()) {
			tofNav.gapAhead(navParams.stopDistance);   // lidar.cpp: terrain mode stopped at a gap
		}
		tofCmd = tofNav.update(dt, slam, odom.pose(), motion.gait().params().maxYawRate, motion.gait().isActive());
		scanTilt = tofNav.bodyTilt();   // 0 with a sensor servo: the sensors tilt, the body stays level
		motion.setHeightOffset(-tofNav.crouch(), 200.0f);
		if (tofNav.active()) {
			cmd = tofCmd;
			navigating = true;
		}
	}
	if (imu || !leveler.enabled()) {
		LR_angle current = { 0, 0, 0 };
		if (imu) {
			current = { imu[0], imu[1], imu[2] };
			lastImu = current;
		}
		LR_angle target = { 0, scanTilt, 0 };   // the scan's nose up is wanted, not leveled away
		// same rule as the firmware (motion.cpp): hold during a dynamic gait
		// (on rough ground the leveler keeps working: the feet stand at different heights)
		bool hold = motion.gait().isActive() && !motion.gait().params().staticStability && !terrainOn;
		imuCorrection = leveler.update(dt, current, target, hold);
	}

	// the commanded tilt includes a ToF room scan's nose up (BodyKinematics subtracts the correction)
	LR_angle commanded = imuCorrection;
	commanded.roll -= scanTilt;
	// map tilt filter, and the map ages only while walking (not turning around for a room scan:
	// the map is paused then, see rc_tof_frame)
	terrainTilt.update(dt, lastImu, commanded, motion.gait().isActive() && scanTilt <= 0, &terrainMap);
	// tilt the gait didn't ask for: measured minus commanded (-correction)
	LR_angle unplanned = { (lastImu.pitch + commanded.pitch) * tiltGain, (lastImu.roll + commanded.roll) * tiltGain, 0 };
	motion.gait().setTilt(unplanned);
	if (headingFed && headingHold.enabled() && !navigating) {
		// motion.cpp steer(): the hold writes the yaw of the command
		bool moving = fabsf(cmd.move.x) > 0.05f || fabsf(cmd.move.y) > 0.05f;
		cmd.rotate.yaw = headingHold.update(cmd.rotate.yaw, headingEst.heading(), motion.gait().params().maxYawRate, moving);
	}
	LR_angle correction = commanded;   // with the scan's nose up
	int unsolved = motion.update(dt, cmd, correction);
	// odometry for the lidar: gait velocity, turned by the gyro (or the gait's own yaw without one)
	// the heading rate (also with a scan's nose up), both biases off the gyro z first
	float yawRate = motion.gait().yawRate();
	if (headingFed) {
		yawRate = lrHeadingRate(lastGyro, lastImu, headingEst.gyroBias() + odom.bias()) - headingEst.gyroBias();
	}
	odom.update(dt, motion.gait().velocity(), yawRate, headingFed && !motion.gait().isActive());
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		lrLegPulses(legs[i], LEG_DOF, servoMainProfile, &pulses[i * LEG_DOF]);
	}
	return unsolved;
}

/** Joint angles, LEG_NUM * 3 floats (alpha, beta, gamma), rad. */
RC_API void rc_leg_angles(float *out)
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		out[i * 3 + 0] = legs[i].angle.alpha;
		out[i * 3 + 1] = legs[i].angle.beta;
		out[i * 3 + 2] = legs[i].angle.gamma;
	}
}

/** Foot targets in the body frame, LEG_NUM * 3 floats, mm. */
RC_API void rc_feet(float *out)
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		out[i * 3 + 0] = legs[i].foot.x;
		out[i * 3 + 1] = legs[i].foot.y;
		out[i * 3 + 2] = legs[i].foot.z;
	}
}

/** Gait body pose: x, y, z (mm), pitch, roll, yaw (rad). */
RC_API void rc_body(float *out)
{
	const LR_figure &b = motion.body();
	out[0] = b.position.x;    out[1] = b.position.y;    out[2] = b.position.z;
	out[3] = b.orientation.pitch; out[4] = b.orientation.roll; out[5] = b.orientation.yaw;
}

/** 1 while the gait is stepping, 0 when idle. */
RC_API int rc_gait_active()
{
	return motion.gait().isActive() ? 1 : 0;
}

/**
 * Pick the gait for the next rc_init(): 0 = the model default (GAIT_PARAMS),
 * 1 = slow walk with static stability (GAIT_PARAMS_WALK, dog only), 2 = rough
 * ground (GAIT_PARAMS_ROUGH). Returns 0 if unknown.
 */
RC_API int rc_select_gait(int id)
{
	switch (id) {
		case 0: gaitParams = &GAIT_PARAMS; return 1;
#ifdef BRIDGE_HAS_WALK
		case 1: gaitParams = &GAIT_PARAMS_WALK; return 1;
#endif
		case 2: gaitParams = &GAIT_PARAMS_ROUGH; return 1;
	}
	return 0;
}

/** Centre of mass distance to the support polygon edge, mm (see GaitEngine::supportMargin). */
RC_API float rc_support_margin()
{
	return motion.gait().supportMargin();
}

/** Static stability body shift, world x, y in mm. */
RC_API void rc_shift(float *out)
{
	LR_point s = motion.gait().shift();
	out[0] = s.x;
	out[1] = s.y;
}

/**
 * Override the static stability gains (0..1, sideways and fore-aft) of the
 * current gait, for experiments; rc_init() restores the model values.
 * Negative sideways gain: switch static stability off.
 */
RC_API void rc_set_stability_gain(float sideways, float foreAft)
{
	GaitParams p = motion.gait().params();
	p.staticStability = sideways >= 0;
	p.stabilityGain.x = sideways < 0 ? 0 : sideways;
	p.stabilityGain.y = foreAft < 0 ? 0 : foreAft;
	motion.gait().setParams(p);
}

/**
 * The firmware IMU driver's tilt estimate from raw sensor values, so the
 * simulator feeds the leveler what the robot would. accel in g,
 * gyro in rad/s, sensor frame X right, Y forward, Z up. out: pitch, roll in
 * the firmware convention (pitch = lean about the forward axis).
 */
static TiltEstimator tilt;
RC_API void rc_tilt_estimate(float dt, const float *accel, const float *gyro, float *out)
{
	tilt.update(dt, accel, gyro);
	out[0] = tilt.aboutY();
	out[1] = tilt.aboutX();
}

/**
 * Heading sensors for one tick, like Mpu9250Imu::read(): the gyro in the body
 * frame (rad/s, x right, y forward, z up) and, when there is a new reading,
 * the magnetometer in the body frame (uT).
 */
RC_API void rc_heading_feed(float dt, const float *gyro, const float *mag)
{
	float yawRate = lrHeadingRate(gyro, lastImu, headingEst.gyroBias());
	for (int i = 0; i < 3; i++) lastGyro[i] = gyro[i];
	headingEst.update(dt, yawRate, lastImu, mag);
	headingFed = true;
	lastYawRate = yawRate;
}

RC_API void rc_heading_hold(int on) { headingHold.setEnabled(on != 0); }
RC_API void rc_turn_to(float rad)
{
	headingHold.setEnabled(true);
	headingHold.setTarget(rad);
}

/** heading (rad, compass), target (rad), hold on, has target, absolute (magnetometer), field (uT) */
RC_API void rc_heading_state(float *out)
{
	out[0] = headingEst.heading();
	out[1] = headingHold.target();
	out[2] = headingHold.enabled();
	out[3] = headingHold.hasTarget();
	out[4] = headingEst.absolute();
	out[5] = headingEst.field();
}

RC_API void rc_tilt_reset()
{
	tilt.reset();
}

/** Body height offset (mm, negative = lower), approached at rate mm/s: the battery sit-down. */
RC_API void rc_set_height_offset(float mm, float rate)
{
	motion.setHeightOffset(mm, rate);
}

/** The model's sit-down depth (SIT_HEIGHT, mm). */
RC_API float rc_sit_height()
{
	return SIT_HEIGHT;
}

/** IMU leveling on (1) or off (0); kept across rc_init(). */
RC_API void rc_set_leveling(int on)
{
	levelOn = on != 0;
	leveler.setEnabled(levelOn);
}

/** Leveler correction: pitch, roll (rad, firmware convention). */
RC_API void rc_level_correction(float *out)
{
	out[0] = imuCorrection.pitch;
	out[1] = imuCorrection.roll;
}

/** Override the current gait's centre of mass (mm, body frame); rc_init() restores it. */
RC_API void rc_set_center_of_mass(float x, float y)
{
	GaitParams p = motion.gait().params();
	p.centerOfMass.x = x;
	p.centerOfMass.y = y;
	motion.gait().setParams(p);
}

/** Override the current gait's sideways foot speed limit (mm/s, 0 = off) and max yaw rate (rad/s, < 0 = keep). */
RC_API void rc_set_side_limits(float maxSideSpeed, float maxYawRate)
{
	GaitParams p = motion.gait().params();
	p.maxSideSpeed = maxSideSpeed < 0 ? 0 : maxSideSpeed;
	if (maxYawRate >= 0) p.maxYawRate = maxYawRate;
	motion.gait().setParams(p);
}

/**
 * Tune the current gait for experiments (rc_init() restores the model
 * values). Values < 0 are left unchanged.
 */
RC_API void rc_tune_gait(float cycleTime, float dutyFactor, float maxSpeed, float stepHeight, float stabilityBlend)
{
	GaitParams p = motion.gait().params();
	if (cycleTime >= 0) p.cycleTime = cycleTime;
	if (maxSpeed >= 0) p.maxSpeed = maxSpeed;
	if (stepHeight >= 0) p.stepHeight = stepHeight;
	if (stabilityBlend >= 0) p.stabilityBlend = stabilityBlend;
	if (dutyFactor >= 0) {
		// keep each leg's swing start, move the offsets with the duty factor
		for (uint8_t i = 0; i < LEG_NUM; i++) {
			float start = p.dutyFactor - p.phaseOffset[i];
			float o = dutyFactor - start;
			p.phaseOffset[i] = o - floorf(o);
		}
		p.dutyFactor = dutyFactor;
	}
	motion.gait().setParams(p);
}

/** Rejected leg poses since rc_init(): out[0] out of reach, out[1] joint limits, out[2] servo range. */
RC_API void rc_reject_counts(uint32_t *out)
{
	out[0] = motion.unreachableCount();
	out[1] = motion.jointLimitCount();
	out[2] = motion.servoRangeCount();
}

/**
 * Leg test: pulses for one leg with its foot at defaultFoot + offset (mm, body
 * frame), body at the origin, through the model's IK and servo HAL (inverse
 * flags, gear ratios, servo profile). The gait and the other legs are left
 * alone. Returns 1 if the IK solved it.
 */
RC_API int rc_leg_test_pulses(int i, const float *offset, uint16_t *pulses)
{
	if (i < 0 || i >= LEG_NUM) return 0;
	leg l = legs[i];
	l.body = l.defaultBody;
	l.foot = { l.defaultFoot.x + offset[0], l.defaultFoot.y + offset[1], l.defaultFoot.z + offset[2] };
	LR_figure origin = {};
	LR_moveVector none = {};
	iksolver s = modelIK.solve(l, origin, none);
	l.angle = s.angle;
	lrLegPulses(l, LEG_DOF, servoMainProfile, pulses);
	return s.isSolved ? 1 : 0;
}

/** The model's angle inverse flags for leg i: out[0..2] = alpha, beta, gamma (0/1). */
RC_API void rc_leg_inverse(int i, int *out)
{
	out[0] = legs[i].inverse.alpha;
	out[1] = legs[i].inverse.beta;
	out[2] = legs[i].inverse.gamma;
}

/* ------------------------------------------------------------ rough ground */

/**
 * Foot contact switches on (1) / off (0): GaitParams.useContactSensors of the
 * current gait. Kept across rc_init(). Feed the switches with rc_set_contacts().
 */
RC_API void rc_set_contact_sensing(int on)
{
	contactOn = on != 0;
	applyTerrain();
}

/** Switch states before a tick, LEG_NUM ints (1 = pressed), like reading the switch pins. */
RC_API void rc_set_contacts(const int *on)
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		legs[i].sensor.onGround = on[i] != 0;
	}
}

/**
 * Terrain adaptation (TERRAIN_PARAMS) on/off, and whether it uses the ToF
 * terrain map. Kept across rc_init().
 */
RC_API void rc_set_terrain(int on, int useMap)
{
	terrainOn = on != 0;
	mapOn = useMap != 0;
	applyTerrain();
}

RC_API int rc_tof_count() { return TOF_SENSOR_COUNT; }

// TOF_SENSORS, with the mount angles tunable from Python (rc_tof_mount) for experiments
static TofSensor tofSensors[TOF_SENSOR_COUNT];
static bool      tofSensorsSet = false;
static const TofSensor &tofSensor(int i)
{
	if (!tofSensorsSet) {
		memcpy((void *)tofSensors, (const void *)TOF_SENSORS, sizeof(tofSensors));
		tofSensorsSet = true;
	}
	return tofSensors[i];
}

/** Sensor i as it looks now: a sensor servo (TofScanParams::sensorServo) tilts it up for a room scan. */
static TofSensor tofSensorNow(int i)
{
	TofSensor t = tofSensor(i);
	if (tofNavOn) t.tilt -= tofNav.sensorTilt();
	return t;
}

/** Sensor i looks tilt rad down (< 0: back to TOF_SENSORS). */
RC_API void rc_tof_mount(int i, float tilt)
{
	if (i < 0 || i >= TOF_SENSOR_COUNT) return;
	tofSensor(i);
	tofSensors[i].tilt = tilt < 0 ? TOF_SENSORS[i].tilt : tilt;
}

/**
 * ToF sensor i: out[0..2] position (mm, body frame), out[3] max range (mm);
 * returns the number of zones (zones per side squared), 0 if no such sensor.
 */
RC_API int rc_tof_sensor(int i, float *out)
{
	if (i < 0 || i >= TOF_SENSOR_COUNT) return 0;
	const TofSensor &t = tofSensor(i);
	out[0] = t.position.x; out[1] = t.position.y; out[2] = t.position.z;
	out[3] = t.maxRange;
	return t.zones * t.zones;
}

/** Zone ray directions of sensor i, body frame unit vectors, zones * 3 floats. */
RC_API void rc_tof_rays(int i, float *out)
{
	if (i < 0 || i >= TOF_SENSOR_COUNT) return;
	const TofSensor t = tofSensorNow(i);
	for (uint8_t z = 0; z < t.zones * t.zones; z++) {
		LR_point d = TerrainMap::zoneDirection(t, z);
		out[z * 3 + 0] = d.x; out[z * 3 + 1] = d.y; out[z * 3 + 2] = d.z;
	}
}

/**
 * One frame of sensor i: a distance per zone (mm, <= 0 = no target), added
 * to the terrain map with the gait's body pose and the last IMU tilt
 * (what the firmware would know).
 */
RC_API void rc_tof_frame(int i, const float *distances)
{
	if (i < 0 || i >= TOF_SENSOR_COUNT) return;
	const TofSensor t = tofSensorNow(i);
	LR_figure pose = motion.body();
	// the floor's own tilt in this frame where there is a floor (motion.cpp updateTerrain)
	LR_angle tilt = floorTiltOn ? frameTilt.place(t, distances, terrainTilt.tilt(), frameGain)
	                            : terrainTilt.tilt();
	pose.orientation.pitch = tilt.pitch;
	pose.orientation.roll  = tilt.roll;
	// and at the height the map already has (motion.cpp updateTerrain)
	if (registerOn) pose.position.z -= terrainMap.frameOffset(t, distances, pose, regDeadband);
	// not during a room scan (motion.cpp updateTerrain): the robot turns on the
	// ground it has mapped, and the nose up it may not quite reach would place
	// the readings wrong
	bool scanning = tofNavOn && tofNav.tilt() > 0;
	for (uint8_t z = 0; z < t.zones * t.zones && !scanning; z++) {
		terrainMap.addReading(t, z, distances[z], pose);
	}
	if (tofNavOn) {
		LR_angle tilt = { lastImu.pitch, lastImu.roll, 0 };
		tofNav.addFrame(t, distances, odom.pose(), tilt, legs[0].defaultFoot.z);
	}
}

/** Mapped ground at world (x, y) of the gait frame, mm; returns 0 if unknown. */
RC_API int rc_terrain_height(float x, float y, float *z)
{
	return terrainMap.heightAt(x, y, *z) ? 1 : 0;
}

/** Number of known map cells. */
RC_API int rc_terrain_cells()
{
	return terrainMap.knownCells();
}

/**
 * Per leg: out[i*3] ground height the leg stands on / aims for (mm, gait
 * frame), out[i*3+1] 1 while reaching down for contact, out[i*3+2] 1 while
 * swinging. out[LEG_NUM*3] the body's terrain height offset (mm).
 */
RC_API void rc_terrain_state(float *out)
{
	for (uint8_t i = 0; i < LEG_NUM; i++) {
		out[i * 3 + 0] = motion.gait().groundHeight(i);
		out[i * 3 + 1] = motion.gait().isSearching(i) ? 1 : 0;
		out[i * 3 + 2] = motion.gait().isSwinging(i) ? 1 : 0;
	}
	out[LEG_NUM * 3] = motion.gait().terrainHeight();
	out[LEG_NUM * 3 + 1] = motion.gait().isBlocked() ? 1 : 0;
}

/** Experiments: scale of the IMU tilt used for contact heights (1 = as measured). */
RC_API void rc_set_tilt_gain(float g) { tiltGain = g; }

/**
 * Experiments: change one TERRAIN_PARAMS field (index in declaration order:
 * searchDepth, searchSpeed, clearance, maxStep, bodyRate, edgeLimit,
 * footholdShift, footRadius, tolerance, latency, gapMargin); index < 0 restores all.
 */
RC_API void rc_tune_terrain(int index, float value)
{
	const int n = sizeof(TerrainParams) / sizeof(float);
	if (index < 0) {
		terrainParams = TERRAIN_PARAMS;
	} else if (index < n) {
		reinterpret_cast<float *>(&terrainParams)[index] = value;
	}
}

/** Experiments: place ToF frames with the floor's tilt (1, the firmware) or only the map tilt (0). */
RC_API void rc_set_floor_tilt(int on) { floorTiltOn = on != 0; }

RC_API void rc_set_frame_register(int on, float deadband) { registerOn = on != 0; regDeadband = deadband; }
RC_API void rc_set_frame_gain(float g) { frameGain = g; }

/**
 * Experiments: body tilt for the terrain map. m 0..2: mode (see mapTiltMode);
 * 10..999: IMU bias filter time constant m / 10 s; 1000+: unplanned tilt
 * filter time constant (m - 1000) / 1000 s.
 */
RC_API void rc_set_map_tilt_mode(int m)
{
	if (m >= 1000) {
		unplannedTau = (m - 1000) / 1000.0f;
	} else if (m >= 10) {
		biasTau = m / 10.0f;
	} else {
		mapTiltMode = (TerrainTiltMode)m;
	}
	terrainTilt.setMode(mapTiltMode);
	terrainTilt.setTimeConstants(biasTau, unplannedTau);
}

/* ------------------------------------------------------------------ lidar */

/** Lidar on (1) / off (0): LD06 bytes from rc_lidar_bytes() feed the map. Kept across rc_init() (which clears the map). */
RC_API int rc_lidar_enable(int on)
{
	lidarOn = on != 0;
	if (lidarOn) {
		lidarScans.begin(LIDAR_PARAMS);
		lidarParser.reset();
		nav.begin(navParams);
		if (!slam.begin(LIDAR_PARAMS, LIDAR_MAP_PARAMS)) return 0;
	}
	return 1;
}

/** Lidar mount: position xyz (mm), yaw (rad), clockwise, min range, max range (mm). */
RC_API void rc_lidar_params(float *out)
{
	out[0] = LIDAR_PARAMS.position.x; out[1] = LIDAR_PARAMS.position.y; out[2] = LIDAR_PARAMS.position.z;
	out[3] = LIDAR_PARAMS.yaw; out[4] = LIDAR_PARAMS.clockwise ? 1 : 0;
	out[5] = LIDAR_PARAMS.minRange; out[6] = LIDAR_PARAMS.maxRange;
}

static void lidarScanDone()
{
	slam.addScan(lidarScans.scan(), scanStartOdom, odom.pose());
	scanStartOdom = odom.pose();
	float dt = simTime - lastScanTime;
	lastScanTime = simTime;
	navCmd = nav.update(dt, slam, slam.pose(odom.pose()), motion.gait().params().maxYawRate);
}

/** Bytes from the lidar's UART (LD06 packets), as they arrive. Returns the number of finished scans. */
RC_API int rc_lidar_bytes(const uint8_t *data, int n)
{
	int scans = 0;
	if (!lidarOn) return 0;
	for (int k = 0; k < n; k++) {
		if (!lidarParser.feed(data[k])) continue;
		for (uint8_t i = 0; i < LD06_POINTS; i++) {
			if (lidarScans.add(lidarParser.angle(i), lidarParser.distance(i), lidarParser.intensity(i))) {
				lidarScanDone();
				scans++;
			}
		}
	}
	return scans;
}

/** Map pose now (x, y mm, theta rad) and the odometry pose (x, y, theta). */
RC_API void rc_lidar_pose(float *out)
{
	LR_pose2 p = slam.pose(odom.pose());
	out[0] = p.x; out[1] = p.y; out[2] = p.theta;
	out[3] = odom.pose().x; out[4] = odom.pose().y; out[5] = odom.pose().theta;
}

/** match, score, scans, rejected, occupied cells, last jump (mm), last turn (rad), packets, CRC errors */
RC_API void rc_lidar_state(float *out)
{
	out[0] = slam.lastMatch(); out[1] = slam.lastScore(); out[2] = slam.scans(); out[3] = slam.rejected();
	out[4] = slam.ready() ? slam.grid().occupiedCells() : 0;
	out[5] = slam.lastJump(); out[6] = slam.lastTurn();
	out[7] = lidarParser.packets(); out[8] = lidarParser.crcErrors();
}

/** Grid size (cells per side) and cell size (mm). */
RC_API void rc_lidar_grid(float *out)
{
	out[0] = LR_LIDAR_GRID;
	out[1] = tofNavOn ? TOF_MAP_PARAMS.cellSize : LIDAR_MAP_PARAMS.cellSize;
}

/** The map, LR_LIDAR_GRID^2 log-odds, row y = 0 first (map y grows with the row). */
RC_API void rc_lidar_map(int8_t *out)
{
	if (slam.ready()) memcpy(out, slam.grid().cells(), LR_LIDAR_GRID * LR_LIDAR_GRID);
}

/** Last scan's points, body frame at its end (mm): out x0, y0, x1, ... Returns the count. */
RC_API int rc_lidar_points(float *out)
{
	const float *x, *y;
	uint16_t n = slam.points(x, y);
	for (uint16_t i = 0; i < n; i++) { out[2 * i] = x[i]; out[2 * i + 1] = y[i]; }
	return n;
}

RC_API void rc_lidar_freeze(int on) { slam.setFrozen(on != 0); }

/** Walk to (x, y), mm in the map frame (`goto`). */
RC_API int rc_nav_goal(float x, float y)
{
	if (tofNavOn) return tofNav.setGoal(x, y) ? 1 : 0;
	return nav.setGoal(x, y) ? 1 : 0;
}
RC_API void rc_nav_stop() { nav.stop(); navCmd = {}; if (tofNavOn) tofNav.stop(); }

static const Navigator &activeNav() { return tofNavOn ? tofNav.nav() : nav; }

/**
 * state (NavState), blocked, distance (mm), goal x, goal y, forward command,
 * yaw command, ToF scan phase (TofScanPhase), scan tilt (rad), points of the last ToF scan,
 * the ToF scan running now is a partial sweep (1) or a full turn (0)
 */
RC_API void rc_nav_state(float *out)
{
	const Navigator &n = activeNav();
	const LR_moveVector &c = tofNavOn ? tofCmd : navCmd;
	out[0] = n.state(); out[1] = tofNavOn ? tofNav.blocked() : n.blocked(); out[2] = n.distance();
	out[3] = n.goalX(); out[4] = n.goalY();
	out[5] = c.move.y; out[6] = c.rotate.yaw;
	out[7] = tofNavOn ? tofNav.phase() : 0; out[8] = tofNav.tilt(); out[9] = tofNav.lastPoints();
	out[10] = tofNavOn && tofNav.partialScan();
	out[11] = tofNavOn ? tofNav.sensorTilt() : 0;
}

/** ToF room scans on (1) / off (0) instead of the lidar; kept across rc_init() (which clears the map). */
RC_API int rc_tofnav_enable(int on)
{
	tofNavOn = on != 0;
	if (tofNavOn) {
		tofNav.begin(tofParams, navParams);
		if (!slam.begin(TOF_ORIGIN, TOF_MAP_PARAMS)) return 0;
	}
	return 1;
}

/** One ToF room scan now (`map scan`). */
RC_API void rc_tofnav_scan() { if (tofNavOn) tofNav.scan(); }

/** The model's TOF_SCAN_PARAMS: tilt, tilt rate, turn rate, turn angle, band, min range, max range, min height, scan every, settle, obstacle height, crouch, rescan angle */
RC_API void rc_tofnav_params(float *out)
{
	const TofScanParams &p = tofParams;
	out[0] = p.tilt; out[1] = p.tiltRate; out[2] = p.turnRate; out[3] = p.turnAngle; out[4] = p.band;
	out[5] = p.minRange; out[6] = p.maxRange; out[7] = p.minHeight; out[8] = p.scanEvery; out[9] = p.settleTime;
	out[10] = p.obstacleHeight; out[11] = p.crouch; out[12] = p.rescanAngle;
	out[13] = p.sensorServo;
}

/** Override TOF_SCAN_PARAMS fields (same order as rc_tofnav_params, < 0 = keep; reset = 1 back to the model's) */
RC_API void rc_tofnav_tune(const float *v, int reset)
{
	if (reset) tofParams = TOF_SCAN_PARAMS;
	float *f[] = { &tofParams.tilt, &tofParams.tiltRate, &tofParams.turnRate, &tofParams.turnAngle, &tofParams.band,
	               &tofParams.minRange, &tofParams.maxRange, &tofParams.minHeight, &tofParams.scanEvery,
	               &tofParams.settleTime, &tofParams.obstacleHeight, &tofParams.crouch,
	               &tofParams.rescanAngle, &tofParams.sensorServo };
	for (int i = 0; i < 14; i++) if (v[i] >= 0) *f[i] = v[i];
	if (tofNavOn) tofNav.begin(tofParams, navParams);
}

/** Override NAV_PARAMS fields (robot radius, margin, lookahead, goal tolerance, speed, turn gain,
 *  turn in place, stop distance, replan period, doubtful; < 0 = keep; reset = 1 back to the model's) */
RC_API void rc_nav_tune(const float *v, int reset)
{
	if (reset) navParams = NAV_PARAMS;
	float *f[] = { &navParams.robotRadius, &navParams.margin, &navParams.lookahead, &navParams.goalTolerance,
	               &navParams.speed, &navParams.turnGain, &navParams.turnInPlace, &navParams.stopDistance,
	               &navParams.replanPeriod, &navParams.doubtful };
	for (int i = 0; i < 10; i++) if (v[i] >= 0) *f[i] = v[i];
	if (tofNavOn) tofNav.begin(tofParams, navParams);
}

/** Planned path, map frame mm: out x0, y0, ... Returns the count. */
RC_API int rc_nav_path(float *out)
{
	const float *x, *y;
	uint8_t n = activeNav().path(x, y);
	for (uint8_t i = 0; i < n; i++) { out[2 * i] = x[i]; out[2 * i + 1] = y[i]; }
	return n;
}

/**
 * Hardware in the loop link (lib/robot_core hil_link.h), for the Python
 * side's tests (sim/legsim/hil.py must speak the same bytes as the firmware).
 */
RC_API int rc_hil_encode_servos(uint32_t seq, uint32_t dtUs, int enabled, int count, const uint16_t *us, uint8_t *out)
{
	HilServos s;
	s.seq = seq;
	s.dtUs = dtUs;
	s.enabled = enabled ? 1 : 0;
	s.count = (uint8_t)count;
	for (int i = 0; i < count && i < HIL_MAX_SERVOS; i++) s.us[i] = us[i];
	return (int)hilEncodeServos(s, out);
}

RC_API int rc_hil_encode_hello(uint8_t *out)
{
	HilHello h = { HIL_VERSION, (uint8_t)ROBOT_MODEL, LEG_NUM, LEG_DOF, LOOP_TIME, (uint8_t)TOF_SENSOR_COUNT };
	return (int)hilEncodeHello(h, out);
}

/**
 * Parse bytes with HilParser: the first good frame's type and decoded values.
 * out (floats): sensors -> accel 3, gyro 3, mag 3, flags, feet, voltage, current;
 * ToF -> sensor, 64 zones; move -> 6. Returns the frame type, 0 if none, -1 if it didn't decode.
 */
RC_API int rc_hil_parse(const uint8_t *data, int n, float *out, uint32_t *errors)
{
	HilParser p;
	for (int i = 0; i < n; i++) {
		if (!p.feed(data[i])) {
			continue;
		}
		errors[0] = p.crcErrors();
		errors[1] = p.skipped();
		switch (p.type()) {
			case HIL_SENSORS: {
				HilSensors s;
				if (!hilDecodeSensors(p.payload(), p.length(), s)) return -1;
				for (int k = 0; k < 3; k++) { out[k] = s.accel[k]; out[3 + k] = s.gyro[k]; out[6 + k] = s.mag[k]; }
				out[9] = s.flags; out[10] = s.feet; out[11] = s.voltage; out[12] = s.current;
				return p.type();
			}
			case HIL_TOF: {
				HilTof t;
				if (!hilDecodeTof(p.payload(), p.length(), t)) return -1;
				out[0] = t.sensor;
				for (int k = 0; k < HIL_TOF_ZONES; k++) out[1 + k] = t.mm[k];
				return p.type();
			}
			case HIL_MOVE: {
				HilMove m;
				if (!hilDecodeMove(p.payload(), p.length(), m)) return -1;
				for (int k = 0; k < 6; k++) out[k] = m.v[k];
				return p.type();
			}
			default:
				return -1;
		}
	}
	errors[0] = p.crcErrors();
	errors[1] = p.skipped();
	return 0;
}
