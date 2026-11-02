/**
 * C API around the firmware motion code, built for the host as a shared
 * library and loaded by the simulator (sim/legsim/firmware.py, ctypes).
 *
 * Everything between the move command and the servo pulse widths is the
 * firmware's own code: MotionController (gait engine, body kinematics, IK)
 * and the servo HAL from lib/robot_core, with the dog's legs, servo profile
 * and gait parameters from src/esp32-legged-robot/model/dog/model_data.h.
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
#else
	#error "The simulator bridge supports the dog model only"
#endif

#define BRIDGE_LOOP_DT (LOOP_TIME / 1000000.0f)

static MotionController motion;
static const GaitParams *gaitParams = &GAIT_PARAMS;
static LR_angle         imuCorrection = { 0, 0, 0 };

#ifdef HORIZON_LEVEL
	// same gains and conversion as src/esp32-legged-robot/motion.cpp
	static AnglePID imuCorrectionPID(PID_LEVEL_P, PID_LEVEL_I / BRIDGE_LOOP_DT, PID_LEVEL_D * BRIDGE_LOOP_DT, -0.2, 0.2);
#endif

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
	#ifdef HORIZON_LEVEL
		imuCorrectionPID = AnglePID(PID_LEVEL_P, PID_LEVEL_I / BRIDGE_LOOP_DT, PID_LEVEL_D * BRIDGE_LOOP_DT, -0.2, 0.2);
	#endif
	motion.begin(*gaitParams, legs, LEG_NUM, modelIK, LEG_DOF, &servoMainProfile);
	LR_moveVector zero = {};
	motion.update(0.0f, zero, imuCorrection);
}

/**
 * One control tick, like updateMotion() + servoSet() in the firmware.
 *
 * @param dt      seconds since the previous tick (firmware: rc_loop_dt())
 * @param command 6 floats in [-1, 1]: move x, y, z, rotate pitch, roll, yaw
 * @param imu     3 floats, body pitch, roll, yaw in rad (used only when the
 *                model enables HORIZON_LEVEL), may be NULL
 * @param pulses  out, LEG_NUM * LEG_DOF pulse widths in us, leg*dof + joint
 * @return        number of legs without an IK solution this tick
 */
RC_API int rc_update(float dt, const float *command, const float *imu, uint16_t *pulses)
{
	LR_moveVector cmd = {
		{ command[0], command[1], command[2] },
		{ command[3], command[4], command[5] }
	};
	#ifdef HORIZON_LEVEL
		if (imu) {
			LR_angle current = { imu[0], imu[1], imu[2] };
			LR_angle target  = { 0, 0, 0 };
			imuCorrection = imuCorrectionPID.update(current, target, dt);
		}
	#else
		(void)imu;
	#endif

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
 * 1 = slow walk with static stability (GAIT_PARAMS_WALK). Returns 0 if unknown.
 */
RC_API int rc_select_gait(int id)
{
	switch (id) {
		case 0: gaitParams = &GAIT_PARAMS; return 1;
		case 1: gaitParams = &GAIT_PARAMS_WALK; return 1;
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
