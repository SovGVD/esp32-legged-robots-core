/**
 * Room scans with the terrain ToF sensors (VL53L5CX, 8x8 zones), for the
 * lidar map, localization and walking to a point without a lidar.
 *
 * The ToF sensors look down at the ground ahead (TOF_SENSORS: the dog's
 * nose sensor 25 deg down, the hexapod's right one 22 deg down). To see the walls
 * the robot stops, tilts its nose up (tilt) until the top zone row looks
 * level, and turns in place once around. Every frame, the zones whose ray is
 * within `band` of level and that hit something above the floor become
 * points of a 360 degree scan, placed with the odometry (gait + gyro) of
 * that moment. Level zones that get nothing back (no target, or beyond
 * maxRange) count as empty up to maxRange (or to where their lower edge
 * would reach the floor): the map marks that free, so the planner can walk
 * there instead of treating it as unknown. The finished scan goes into the same map and scan matcher as
 * a lidar scan (LidarSlam::addPoints).
 *
 * Walking to a goal (TofNavigator): scan, walk scanEvery mm along the planned
 * path on odometry alone, stop, scan again (which corrects the pose), and so
 * on until the goal. Once the map has a full scan, rescans on the way only
 * sweep rescanAngle around the heading (the matcher needs the map, not a
 * whole circle). A partial sweep that doesn't fit the map turns on all the
 * way round on the spot; scan() and a rejected full scan ask for a full turn again. While walking the sensors look down at the ground as
 * usual and stop the robot in front of anything higher than obstacleHeight.
 *
 * Frames: map frame and LR_pose2 as in lidar.h. Body tilt in the firmware
 * convention (LR_angle: pitch = lean about the forward axis, left side up
 * > 0; roll = nose up > 0), as the IMU measures it.
 */
#ifndef LR_TOF_SCAN_H
#define LR_TOF_SCAN_H

#include "lidar_slam.h"
#include "navigator.h"
#include "terrain_map.h"

typedef struct {
	float tilt;           // rad, nose up while scanning: the sensors' top zone row looks level
	float tiltRate;       // rad/s, how fast the body tilts up and back down
	float turnRate;       // rad/s, turning while scanning (counter-clockwise)
	float turnAngle;      // rad, one scan turns this far (a full turn and a little overlap)
	float band;           // rad, zone rays within this of level are used
	float minRange;       // mm, closer readings are ignored
	float maxRange;       // mm, further readings are ignored (the sensor's reach indoors)
	float minHeight;      // mm above the floor: lower hits are the floor
	float scanEvery;      // mm walked between scans on the way to a goal
	float settleTime;     // s standing still before tilting up
	float obstacleHeight; // mm above the floor: something this high ahead stops the walk
	float crouch;         // mm the body goes down while scanning (the legs reach a steeper tilt)
	float rescanAngle;    // rad, later scans sweep only this much around the heading (turn right half,
	                      // then left all of it), matched against the map; 0 = a full turn every time
} TofScanParams;

/** One room scan: collects points from ToF frames while the robot turns. */
class TofScanBuilder {
	public:
		void begin(const TofScanParams &params) { _params = params; start({ 0, 0, 0 }); }
		/** A new scan around the robot's position (odometry). */
		void start(const LR_pose2 &odom);
		/**
		 * One frame of a sensor.
		 * @param mm     zones (TofSensor order), mm, <= 0 = no target
		 * @param odom   odometry pose when the frame was measured
		 * @param tilt   body tilt then (IMU)
		 * @param floorZ the floor's height below the body origin (< 0), mm
		 */
		void addFrame(const TofSensor &sensor, const float *mm, const LR_pose2 &odom, const LR_angle &tilt, float floorZ);
		/**
		 * The scan in the body frame of odomEnd, one point per degree around the
		 * scan's centre at most (x, y: LR_LIDAR_BINS each). Returns the count.
		 */
		uint16_t finish(const LR_pose2 &odomEnd, float *x, float *y) const;
		/**
		 * Free space: for every degree without a point, where the level zones
		 * looked and got nothing back (no target, or further than maxRange),
		 * the far end of what they saw empty, in the body frame of odomEnd.
		 * The map marks the cells up to it free (not occupied): "nothing
		 * there as far as the sensor reaches". Returns the count.
		 */
		uint16_t finishClear(const LR_pose2 &odomEnd, float *x, float *y) const;
		uint16_t points() const { return _points; }
		uint32_t readings() const { return _readings; }

	private:
		TofScanParams _params;
		float    _cx = 0, _cy = 0;             // scan centre, odometry frame
		float    _x[LR_LIDAR_BINS];            // closest point per degree, odometry frame
		float    _y[LR_LIDAR_BINS];
		float    _r[LR_LIDAR_BINS];            // its distance from the centre, 0 = none
		float    _clear[LR_LIDAR_BINS];        // seen empty this far from the centre, 0 = not looked
		uint16_t _points = 0;
		uint32_t _readings = 0;
};

/**
 * Something higher than `height` above the floor within `distance` mm ahead
 * of the body origin and `halfWidth` to either side, seen in this frame (at
 * least two zones, one is noise). at: the closest such point, body frame (mm).
 */
bool lrTofObstacleAhead(const TofSensor &sensor, const float *mm, const LR_angle &tilt, float floorZ,
                        float distance, float halfWidth, float height, LR_point *at = nullptr);

enum TofScanPhase : uint8_t {
	TOF_SCAN_IDLE = 0,
	TOF_SCAN_STOP,      // standing still before the scan
	TOF_SCAN_TILT,      // nose going up
	TOF_SCAN_TURN,      // turning around, collecting
	TOF_SCAN_LEVEL      // nose going back down
};

/**
 * Scans and walks to a goal: the move command and the body tilt for the
 * motion code, from the ToF frames, the odometry and the map.
 */
class TofNavigator {
	public:
		void begin(const TofScanParams &scan, const NavParams &nav);
		const TofScanParams &params() const { return _params; }
		/** One full scan now (no goal): the map grows, the pose is corrected. */
		void scan();
		/** The scan running now is a partial sweep (rescanAngle), not a full turn. */
		bool partialScan() const { return _sweep > 0; }
		/**
		 * Walk to (x, y), mm in the map frame. Scans first, unless the last scan
		 * fit the map and the robot walked under scanEvery / 2 since (a new goal
		 * right after arriving at the last one starts walking at once).
		 * false: no memory for the planner.
		 */
		bool setGoal(float x, float y);
		/** Stop walking and scanning (the nose comes back down). */
		void stop();
		/** Scanning or walking to a goal: the move command is ours. */
		bool active() const { return _phase != TOF_SCAN_IDLE || _nav.active(); }
		TofScanPhase phase() const { return _phase; }
		const Navigator &nav() const { return _nav; }
		/** Something close ahead (live ToF) or a gap (gapAhead), walking stopped. */
		bool blocked() const { return _blocked || _sinceGap < 0.1f; }
		/** Nose up to command now, rad (0 while not scanning). */
		float tilt() const { return _tilt; }
		/** Body lowered by this now, mm (with the tilt). */
		float crouch() const { return _params.tilt > 0 ? _params.crouch * _tilt / _params.tilt : 0; }
		/** Points the last finished scan had. */
		uint16_t lastPoints() const { return _lastPoints; }
		const TofScanBuilder &builder() const { return _builder; }

		/**
		 * The terrain code (GaitEngine::isBlocked) stopped the robot at a gap ahead:
		 * call every tick while it does. Treated like an obstacle `distance` mm
		 * ahead: walking stops, after a second it goes into the map and the path
		 * goes around.
		 */
		void gapAhead(float distance) { _gap = { 0, distance, 0 }; _sinceGap = 0; }
		/** A ToF frame with the odometry and tilt when it was measured (see TofScanBuilder::addFrame). */
		void addFrame(const TofSensor &sensor, const float *mm, const LR_pose2 &odom, const LR_angle &tilt, float floorZ);
		/**
		 * Every control tick (or at least ~20 times a second).
		 * @param odom       odometry pose now
		 * @param maxYawRate rad/s the gait turns at a yaw command of 1
		 * @param gaitActive the legs are stepping
		 * @return move command (zero when not active)
		 */
		LR_moveVector update(float dt, LidarSlam &slam, const LR_pose2 &odom, float maxYawRate, bool gaitActive);

	private:
		TofScanParams  _params;
		TofScanBuilder _builder;
		Navigator      _nav;
		TofScanPhase   _phase = TOF_SCAN_IDLE;
		float    _tilt = 0;
		float    _time = 0;        // in this phase, s
		float    _turned = 0;      // rad, this scan (counter-clockwise +)
		float    _sweep = 0;       // rad: this scan is a partial sweep of this much (0 = full turn)
		bool     _sweepBack = false;   // partial sweep: turning back left after the first half to the right
		bool     _haveFull = false;    // a full scan went into the map: rescans may be partial
		bool     _wantFull = true;     // the next scan turns all the way round
		float    _walked = 0;      // mm since the last scan
		float    _sinceFrame = 1;  // s since the last obstacle check
		bool     _blocked = false;
		float    _blockedTime = 0; // s blocked in a row
		LR_point _obstacle = { 0, 0, 0 };   // closest obstacle ahead, body frame
		LR_point _gap = { 0, 0, 0 };        // gap ahead (gapAhead), body frame
		float    _sinceGap = 1;             // s since gapAhead
		bool     _havePrev = false;
		LR_pose2 _prev = { 0, 0, 0 };
		uint16_t _lastPoints = 0;
		float    _px[LR_LIDAR_BINS];
		float    _py[LR_LIDAR_BINS];
		float    _fx[LR_LIDAR_BINS];   // free space ends (finishClear)
		float    _fy[LR_LIDAR_BINS];

		void startScan();
};

#endif
