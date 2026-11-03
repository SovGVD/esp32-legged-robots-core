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
 * Terrain (optional, setTerrain()): every leg remembers the height of the
 * ground it last stood on. With contact sensors, a foot that has not touched
 * down when its swing time is over keeps lowering (up to `searchDepth`) until
 * it does, and a foot that touches down early stays there; either way the
 * leg's ground height is where the foot really landed. With a TerrainMap
 * (ToF sensors) the foot is aimed at the mapped height of its foothold, lifted
 * over the highest mapped point between lift-off and touchdown, and its
 * foothold is moved off an edge. The body height follows the average ground
 * height under the feet. Without a terrain map or contact sensors this
 * changes nothing: every leg's ground stays at its neutral height.
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
#include "terrain_map.h"

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

typedef struct {
	float searchDepth;    // mm: with contact sensors, how far below its planned touchdown a foot keeps
	                      // reaching for the ground after its swing time is over
	float searchSpeed;    // mm/s, lowering speed while reaching
	float clearance;      // mm: a swinging foot passes this far above the highest mapped point on its way
	float maxStep;        // mm: mapped heights further than this from the leg's own ground are ignored
	                      // (sensor glitches, steps the leg can't reach anyway)
	float bodyRate;       // mm/s: body height follows the average ground under the feet (0: fixed height)
	float edgeLimit;      // mm: a foothold whose mapped surroundings vary more than this is on an edge
	                      // and is moved along the step (0: never)
	float footholdShift;  // mm, how far a foothold may move to get off an edge
	float footRadius;     // mm, foot point (leg.foot) above the surface it stands on (ball foot: its radius)
	float tolerance;      // mm: a foot touching down closer than this to where it was meant to keeps the
	                      // planned ground height (servo sag and IMU noise would make it wander)
	float latency;        // s, how long the servos take to follow: when a reaching foot touches down it is
	                      // put back up by searchSpeed * latency (else the ground estimate sinks every step)
	float gapMargin;      // mm, a foothold keeps this far from ground that drops away (lower than its own
	                      // by more than `maxStep`, or unseen between known ground: a gap, a cliff);
	                      // 0: footRadius + one map cell
} TerrainParams;

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

		/**
		 * Terrain adaptation (see the top of this file). params nullptr: off,
		 * the legs' ground heights go back to neutral. map may be nullptr
		 * (contact sensors only). Both must outlive the gait.
		 */
		void setTerrain(const TerrainParams *params, const TerrainMap *map);
		const TerrainParams *terrain() const { return _terrain; }
		/** Ground height (world z, mm) the leg last stood on or is stepping to. */
		float groundHeight(uint8_t legId) const { return _state[legId].ground; }
		/** Leg is past its swing time and still reaching down for the ground. */
		bool  isSearching(uint8_t legId) const { return _state[legId].searching; }
		/**
		 * Body tilt the gait didn't plan (IMU tilt minus the commanded body
		 * tilt, rad; yaw unused). Contact heights are measured with it, so a
		 * tilted body doesn't read the feet in front as standing higher.
		 */
		void  setTilt(const LR_angle &tilt) { _tilt = tilt; }

		/** Body height offset from the terrain, mm. */
		float terrainHeight() const { return _terrainZ; }
		/**
		 * A leg found a gap ahead it can't step over (the map shows a hole
		 * deeper than maxStep and no flat foothold within footholdShift):
		 * the gait won't move further that way (body frame unit vector)
		 * until the move command stops pushing that way.
		 */
		bool  isBlocked() const { return _blocked; }
		LR_point blockedDirection() const { return _blockedDir; }

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
			bool     searching;   // swing time over, lowering until contact (contact sensors)
			LR_point liftOff;     // foot position at lift off
			float    ground;      // z the leg stands on / plans to land on, mm
			float    apex;        // highest z of the current swing, mm (terrain)
			LR_point offset;      // foothold moved off an edge, world XY, mm
			float    lost;        // s the switch has been open while standing (slipped off an edge)
			bool     hold;        // a gap ahead: this swing puts the foot back where it lifted off
			bool     released;    // contact switch opened during this swing (the servos lag: a foot
			                      // still reads "down" for a while after its swing has started)
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
		const TerrainParams *_terrain = nullptr;
		const TerrainMap    *_map     = nullptr;
		float      _terrainZ = 0;
		LR_angle   _tilt     = { 0, 0, 0 };
		bool       _blocked  = false;
		LR_point   _blockedDir = { 0, 0, 0 };
		bool       footholdOk(float x, float y, float dx, float dy, float own) const;
		bool       dropsAway(float x, float y, float dx, float dy, float own) const;
		bool       isShadow(float x, float y, float dx, float dy, float own) const;
		float      contactHeight(uint8_t legId) const;

		void     planSwing(uint8_t legId);
		float    plannedGround(uint8_t legId, const LR_point &at) const;
		void     updateTerrainHeight(float dt);
		void     updateSearch(float dt);

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
 * Swing path for rough ground: straight up to `apex` (world z), across, then
 * straight down, so the foot doesn't catch the edge of what it steps onto.
 * Zero velocity at s = 0 and s = 1.
 */
LR_point lrSwingPointHigh(const LR_point &from, const LR_point &to, float apex, float s);

/**
 * Signed distance (XY, mm) from `p` to the boundary of the convex hull of
 * `points`: positive inside. For one or two points: minus the distance to
 * that point or segment. Zero points: -1e9.
 */
float lrSupportMargin(const LR_point *points, uint8_t count, const LR_point &p);

#endif
