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

		void setParams(const GaitParams &params) { _params = params; }
		const GaitParams &params() const { return _params; }

		const LR_figure &body() const { return _body; }
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
			LR_point liftOff;     // foot position at lift off
		} legState;

		GaitParams _params = {};
		leg       *_legs     = nullptr;
		uint8_t    _legCount = 0;
		legState   _state[LR_MAX_LEGS] = {};

		LR_figure  _body     = {};
		LR_point   _velocity = {0, 0, 0};
		float      _yawRate  = 0;
		float      _phase    = 0;
		bool       _active   = false;

		void     rampVelocity(float dt, const LR_moveVector &command);
		LR_point touchdownTarget(uint8_t legId, float swingProgress) const;
		bool     isAtNeutral(uint8_t legId) const;
};

/**
 * Cycloid swing interpolation from `from` to `to` lifting the foot by `height`.
 * Horizontal and vertical velocities are zero at s = 0 and s = 1.
 *
 * @param s swing progress [0, 1]
 */
LR_point lrSwingPoint(const LR_point &from, const LR_point &to, float height, float s);

#endif
