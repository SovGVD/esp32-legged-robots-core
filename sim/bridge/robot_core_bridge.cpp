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
static float biasTau = 1.5f;
static float unplannedTau = 0.3f;
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
	if (imu || !leveler.enabled()) {
		LR_angle current = { 0, 0, 0 };
		if (imu) {
			current = { imu[0], imu[1], imu[2] };
			lastImu = current;
		}
		LR_angle target = { 0, 0, 0 };
		// same rule as the firmware (motion.cpp): hold during a dynamic gait
		// (on rough ground the leveler keeps working: the feet stand at different heights)
		bool hold = motion.gait().isActive() && !motion.gait().params().staticStability && !terrainOn;
		imuCorrection = leveler.update(dt, current, target, hold);
	}

	// map tilt filter, and the map ages only while walking
	terrainTilt.update(dt, lastImu, imuCorrection, motion.gait().isActive(), &terrainMap);
	// tilt the gait didn't ask for: measured minus commanded (-correction)
	LR_angle unplanned = { (lastImu.pitch + imuCorrection.pitch) * tiltGain, (lastImu.roll + imuCorrection.roll) * tiltGain, 0 };
	motion.gait().setTilt(unplanned);
	int unsolved = motion.update(dt, cmd, imuCorrection);
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

/**
 * ToF sensor i: out[0..2] position (mm, body frame), out[3] max range (mm);
 * returns the number of zones (zones per side squared), 0 if no such sensor.
 */
RC_API int rc_tof_sensor(int i, float *out)
{
	if (i < 0 || i >= TOF_SENSOR_COUNT) return 0;
	const TofSensor &t = TOF_SENSORS[i];
	out[0] = t.position.x; out[1] = t.position.y; out[2] = t.position.z;
	out[3] = t.maxRange;
	return t.zones * t.zones;
}

/** Zone ray directions of sensor i, body frame unit vectors, zones * 3 floats. */
RC_API void rc_tof_rays(int i, float *out)
{
	if (i < 0 || i >= TOF_SENSOR_COUNT) return;
	const TofSensor &t = TOF_SENSORS[i];
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
	const TofSensor &t = TOF_SENSORS[i];
	LR_figure pose = motion.body();
	pose.orientation.pitch = terrainTilt.tilt().pitch;
	pose.orientation.roll  = terrainTilt.tilt().roll;
	for (uint8_t z = 0; z < t.zones * t.zones; z++) {
		terrainMap.addReading(t, z, distances[z], pose);
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
