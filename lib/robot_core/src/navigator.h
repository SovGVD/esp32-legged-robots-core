/**
 * Walk to a point of the lidar map: path planning on the occupancy grid and
 * a simple follower that writes the move command.
 *
 * Planning works on a coarser copy of the map (LR_NAV_FACTOR x the map
 * cell, 10 cm): cells closer than robotRadius to a wall are blocked, cells
 * a little further cost more (the path keeps to the middle of a doorway),
 * unknown cells cost more than free ones but are allowed (the goal may be
 * behind a corner the lidar hasn't seen yet). A distance field from the goal
 * over the whole grid is filled by repeated sweeps (no queue, fixed memory);
 * the robot then always walks downhill on it, so a replan once a second is
 * enough to follow a map that changes.
 *
 * The follower turns toward a point `lookahead` mm down the path and walks
 * forward once roughly facing it; lidar points straight ahead closer than
 * stopDistance stop the forward motion (turning is still allowed).
 */
#ifndef LR_NAVIGATOR_H
#define LR_NAVIGATOR_H

#include "lidar_slam.h"

#ifndef LR_NAV_FACTOR
	#define LR_NAV_FACTOR 2   // planning cell = 2 map cells
#endif
#define LR_NAV_GRID (LR_LIDAR_GRID / LR_NAV_FACTOR)
#define LR_NAV_PATH 64        // path points kept for the web map view

typedef struct {
	float robotRadius;    // mm, how close the robot's centre may come to a wall
	float margin;         // mm, extra distance that still costs more (stay in the middle)
	float lookahead;      // mm down the path the robot turns toward
	float goalTolerance;  // mm, arrived when this close
	float speed;          // 0..1, forward command while walking
	float turnGain;       // 1/s, turn rate per rad of heading error
	float turnInPlace;    // rad, larger heading errors turn without walking
	float stopDistance;   // mm, lidar points closer than this ahead stop the forward motion
	float replanPeriod;   // s
} NavParams;

enum NavState : uint8_t {
	NAV_IDLE = 0,
	NAV_ACTIVE,      // walking to the goal
	NAV_ARRIVED,
	NAV_NO_PATH,     // goal blocked or not reachable on the map
	NAV_NO_MEMORY
};

class Navigator {
	public:
		~Navigator();
		void begin(const NavParams &params) { _params = params; }
		const NavParams &params() const { return _params; }
		/** Walk to (x, y), mm in the map frame. false: no memory for the planner (30 KB). */
		bool setGoal(float x, float y);
		void stop() { _state = NAV_IDLE; }
		NavState state() const { return _state; }
		bool active() const { return _state == NAV_ACTIVE; }
		float goalX() const { return _goalX; }
		float goalY() const { return _goalY; }
		/** Something close ahead stopped the forward motion on the last update. */
		bool blocked() const { return _blocked; }
		/** Straight line distance to the goal at the last update, mm. */
		float distance() const { return _distance; }

		/**
		 * Plan (every replanPeriod) and steer. Call after each scan.
		 * @param pose       robot pose in the map now
		 * @param maxYawRate rad/s the gait turns at a yaw command of 1
		 * @return move command (move.y forward, rotate.yaw > 0 clockwise), zero unless active
		 */
		LR_moveVector update(float dt, const LidarSlam &slam, const LR_pose2 &pose, float maxYawRate);

		/** Path from the robot toward the goal at the last update, map frame mm. */
		uint8_t path(const float *&x, const float *&y) const { x = _pathX; y = _pathY; return _pathLen; }

		/** Run the planner now (tests): false if the goal can't be reached from pose. */
		bool plan(const OccupancyGrid &grid, const LR_pose2 &pose);

	private:
		NavParams _params = { 120, 100, 400, 120, 0.6f, 1.5f, 0.6f, 200, 1.0f };
		NavState  _state = NAV_IDLE;
		float     _goalX = 0, _goalY = 0;
		float     _sinceplan = 0;
		bool      _replan = false;
		bool      _blocked = false;
		float     _distance = 0;
		uint8_t  *_cost = nullptr;    // 0 = blocked, else cost of crossing the cell
		uint16_t *_dist = nullptr;    // cost from the cell to the goal, 0xFFFF = unreachable
		float     _pathX[LR_NAV_PATH];
		float     _pathY[LR_NAV_PATH];
		uint8_t   _pathLen = 0;
		float     _cell = 100;        // planning cell, mm

		bool allocate();
		void buildCost(const OccupancyGrid &grid);
		void fillDistance(int gx, int gy);
		bool cellOf(float x, float y, int &ix, int &iy) const;
		void centre(int ix, int iy, float &x, float &y) const;
		bool startCell(const LR_pose2 &pose, int &ix, int &iy) const;
		bool downhill(int &ix, int &iy) const;
		void tracePath(const LR_pose2 &pose);
};

#endif
