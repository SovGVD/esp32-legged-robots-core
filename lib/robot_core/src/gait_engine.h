/**
 * Phase based gait engine.
 *
 * A single gait clock (phase 0..1 per cycle) drives every leg. Each leg has a
 * phase offset; while its local phase is below the duty factor the leg is in
 * stance (foot fixed in the world), otherwise it swings to a new foothold.
 * Trot, walk, tripod, wave and ripple gaits are just different parameters.
 *
 * The body moves continuously: the move command is a velocity, integrated
 * every update with acceleration limits. Footholds are placed where the
 * neutral stance will be at mid-stance (Raibert style), and the swing path is
 * a cycloid, so feet lift off and touch down with zero velocity.
 *
 * The gait is open loop (time based) by default. Foot contact sensors are
 * optional: with `useContactSensors` a swing ends early once the leg reports
 * ground contact in the second half of the swing.
 *
 * Static stability (optional, `staticStability`): the body is shifted so the
 * centre of mass stays over the legs that carry it. Every foot gets a support
 * weight: 1 in mid stance, 0 in swing, and a smooth ramp from touchdown and
 * towards lift-off over `stabilityBlend` of the cycle, shortened so the ramps
 * fit in the phases when all the other feet are down. The centre of mass is
 * steered to the weighted average of the feet, so before a leg lifts the body
 * has already moved over the triangle of the other three. `stabilityGain`
 * scales the shift per body axis (1 puts the centre of mass on the triangle
 * centroid; the constant `centerOfMass` offset is always compensated in full,
 * in every gait). For a quadruped walk a sideways-only shift is usually best:
 * the support triangles share the diagonal through the body centre, so a
 * sideways shift alone gives a margin, while a fore-aft sway makes the body
 * rush back and forth in the four-foot phases. Meant for slow
 * gaits where three feet carry the robot (walk, ripple); a trot or tripod
 * only has a line or a triangle that moves with the body anyway.
 *
 * All positions are in mm, world coordinates; `leg.defaultFoot` is the
 * neutral foot position in body coordinates.
 */

#ifndef LR_GAIT_ENGINE_H
#define LR_GAIT_ENGINE_H

#include "leg.h"

typedef struct {
	bool  enabled;                    // false: body and feet never move (e.g. tank)
	float cycleTime;                  // s, one full gait cycle
	float dutyFactor;                 // fraction of the cycle a leg is on the ground, (0, 1)
	float phaseOffset[LR_MAX_LEGS];   // per leg, [0, 1)
	float stepHeight;                 // mm, foot lift during swing
	float maxSpeed;                   // mm/s at move command 1.0
	float maxYawRate;                 // rad/s at rotate.yaw command 1.0
	float maxAccel;                   // mm/s^2
	float maxYawAccel;                // rad/s^2
	float idleTolerance;              // mm, feet closer than this to neutral do not step when idle
	bool  useContactSensors;          // end swing early on leg.sensor.onGround
	bool  staticStability;            // shift the body over the supporting feet (see above)
	float stabilityBlend;             // max fraction of the cycle a support weight ramps over, e.g. 0.1
	LR_point stabilityGain;           // 0..1 per body axis (x sideways, y fore-aft): how far towards the support centroid the body shifts
	LR_point centerOfMass;            // mm, centre of mass relative to the body origin, body frame (z unused);
	                                  // without static stability the body is offset by -centerOfMass so the
	                                  // centre of mass stays over the middle of the neutral stance
	float maxSideSpeed;               // mm/s, 0 = off: limit on the feet's sideways speed relative to the body
	                                  // from strafing plus turning (|move.x| + |yaw rate| * foot Y); both are
	                                  // scaled down together. The hip roll joints do all the sideways work and
	                                  // may be slower than the others (dog: 2:1 gear, half speed)
} GaitParams;

class GaitEngine
{
	public:
		void begin(const GaitParams &params, leg *legs, uint8_t legCount);

		/**
		 * Body back to the origin, feet to their neutral positions, gait stopped.
		 */
		void reset();

		/**
		 * @param dt seconds since previous update
		 * @param command move.x/move.y [-1, 1] velocity in body frame, rotate.yaw [-1, 1] turn rate
		 */
		void update(float dt, const LR_moveVector &command);

		/** Takes effect immediately; switch gaits while !isActive() for a clean start. */
		void setParams(const GaitParams &params) { _params = params; computeRamps(); }
		const GaitParams &params() const { return _params; }

		/** Body pose to solve the legs for: odometry pose plus the stability shift. */
		const LR_figure &body() const { return _body; }
		/** Odometry pose without the stability shift. */
		const LR_point &basePosition() const { return _base; }
		/** Current stability shift, world frame, mm. */
		LR_point shift() const { return _shift; }

		/**
		 * Distance (mm) from the centre of mass projection to the nearest edge
		 * of the support polygon of the feet on the ground. Negative: outside,
		 * the robot would tip over without dynamic balance. Fewer than three
		 * feet on the ground: minus the distance to the support line/point.
		 */
		float supportMargin() const;
		float phase() const { return _phase; }
		bool  isActive() const { return _active; }
		bool  isSwinging(uint8_t legId) const { return _state[legId].swing; }
		LR_point velocity() const { return _velocity; }   // body frame, mm/s
		float yawRate() const { return _yawRate; }        // rad/s

		/**
		 * Neutral (default) foot position in world coordinates for a body pose.
		 */
		LR_point neutralFoot(uint8_t legId, const LR_point &bodyPosition, float bodyYaw) const;

	private:
		typedef struct {
			bool     swing;       // currently in swing
			bool     skipped;     // swing window skipped this cycle (idle and already at neutral)
			bool     landed;      // touched down this stance (its support weight ramps in)
			LR_point liftOff;     // foot position at lift off
		} legState;

		GaitParams _params = {};
		leg       *_legs     = nullptr;
		uint8_t    _legCount = 0;
		legState   _state[LR_MAX_LEGS] = {};
		float      _rampUp[LR_MAX_LEGS]   = {};   // support weight ramp after touchdown, cycle fraction
		float      _rampDown[LR_MAX_LEGS] = {};   // ramp before lift-off

		LR_figure  _body     = {};     // _base + _shift
		LR_point   _base     = {0, 0, 0};
		LR_point   _shift    = {0, 0, 0};
		LR_point   _velocity = {0, 0, 0};
		float      _yawRate  = 0;
		float      _phase    = 0;
		bool       _active   = false;

		float    _sideLever = 0;   // mm, largest |defaultFoot.y|: sideways foot speed per rad/s of yaw
		void     rampVelocity(float dt, const LR_moveVector &command);
		LR_point touchdownTarget(uint8_t legId, float swingProgress) const;
		bool     isAtNeutral(uint8_t legId) const;
		float    supportWeight(uint8_t legId) const;
		void     updateShift(float dt);
		void     computeRamps();
		LR_point centerOfMass() const;
};

/**
 * Cycloid swing interpolation from `from` to `to` lifting the foot by `height`.
 * Horizontal and vertical velocities are zero at s = 0 and s = 1.
 *
 * @param s swing progress [0, 1]
 */
LR_point lrSwingPoint(const LR_point &from, const LR_point &to, float height, float s);

/**
 * Signed distance (XY, mm) from `p` to the boundary of the convex hull of
 * `points`: positive inside. For one or two points: minus the distance to
 * that point or segment. Zero points: -1e9.
 */
float lrSupportMargin(const LR_point *points, uint8_t count, const LR_point &p);

#endif
