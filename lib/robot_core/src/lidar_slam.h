/**
 * 2D mapping and localization with a 360 degree scan (a lidar, or the ToF
 * room scan: tof_scan.h), small enough for an
 * ESP32: an occupancy grid (one signed byte per cell) and a scan matcher
 * that corrects the walking odometry against it (Gauss-Newton on the
 * smoothed map, like Hector SLAM, coarse then fine).
 *
 * Every lidar revolution:
 *   1. the odometry (gait velocity + gyro yaw rate, Odometry2D) moved the
 *      robot since the last scan: that is the guess of where it is now;
 *   2. the scan is straightened out (each degree was measured at a slightly
 *      different pose while the robot walked and turned);
 *   3. the matcher moves the guess until the scan's points lie on the walls
 *      already in the map; a poor fit (few points on known walls, or a jump
 *      too large) is thrown away and the odometry guess kept;
 *   4. once the robot moved far enough, the scan is put into the map: the
 *      cells along each beam become more "free", the cell at its end more
 *      "occupied".
 *
 * The map is fixed in size (LR_LIDAR_GRID cells per side, 200 x 200 x 10 cm =
 * 20 x 20 m, 40 KB, allocated by begin()), centred on the place it was
 * started. Readings outside it are ignored.
 */
#ifndef LR_LIDAR_SLAM_H
#define LR_LIDAR_SLAM_H

#include "lidar.h"

#ifndef LR_LIDAR_GRID
	#define LR_LIDAR_GRID 200   // cells per side
#endif

typedef struct {
	float   cellSize;        // mm
	float   mapRange;        // mm, readings further than this are not put into the map
	int8_t  hit;             // log-odds added to the cell a beam ends in
	int8_t  miss;            // log-odds taken from each cell a beam passes
	int8_t  maxLogOdds;      // cells stay within +-this
	float   updateDistance;  // mm, the map is updated after the robot moved this far...
	float   updateAngle;     // rad, ...or turned this much
	float   minScore;        // 0..1, matches that fit worse are thrown away
	float   maxJump;         // mm, matches that move the guess further are thrown away
	float   maxTurn;         // rad, same for the angle
} LidarMapParams;

/** Occupancy grid, log-odds per cell: < 0 free, 0 unknown, > 0 occupied. */
class OccupancyGrid {
	public:
		~OccupancyGrid();
		/** Allocates the cells (LR_LIDAR_GRID^2 bytes); false if out of memory. */
		bool begin(float cellSize);
		bool ready() const { return _cells != nullptr; }
		void clear();
		int  size() const { return LR_LIDAR_GRID; }
		float cellSize() const { return _cell; }
		/** Cell of map point (x, y) mm; false outside the map. */
		bool cellOf(float x, float y, int &ix, int &iy) const;
		/** Centre of cell (ix, iy), mm. */
		void cellCentre(int ix, int iy, float &x, float &y) const;
		int8_t at(int ix, int iy) const { return _cells[iy * LR_LIDAR_GRID + ix]; }
		const int8_t *cells() const { return _cells; }
		/** Beam from (x0, y0) to (x1, y1), mm: cells along it free, the end cell occupied if hit. */
		void addBeam(float x0, float y0, float x1, float y1, bool hit, int8_t hitStep, int8_t missStep, int8_t limit);
		/**
		 * Occupancy 0..1 at (x, y), bilinear between cell centres, and its
		 * gradient (per mm). level > 0 reads a 2^level times coarser map
		 * (each coarse cell the highest of its fine cells): a wider, smoother
		 * pull for the first matcher steps.
		 */
		float sample(float x, float y, int level, float &gx, float &gy) const;
		/** Cells that are occupied (> 0) / known (!= 0). */
		uint32_t occupiedCells() const;
		uint32_t knownCells() const;

	private:
		int8_t *_cells = nullptr;
		float   _cell = 50;
		float   _occScale = 1.0f / 40;    // log-odds -> 0..1 (capped)
		friend class LidarSlam;
		float value(int ix, int iy, int level) const;
};

/**
 * Dead reckoning in the plane: the gait's body velocity turned by the gyro
 * heading (the gait's own yaw is only what it was told to turn).
 *
 * While the robot stands (the gait idle) the yaw rate it reads is the gyro's
 * bias (a MEMS gyro reads ~1 deg/s while still): it is learned then and
 * taken off every reading. Without that a slow scan turn of 30 s would come
 * out 30 deg short or long, and the scan smeared.
 */
class Odometry2D {
	public:
		void reset() { _pose.x = 0; _pose.y = 0; _pose.theta = 0; }
		/**
		 * v: body frame velocity mm/s (x right, y forward), yawRate rad/s counter-clockwise (gyro),
		 * standing: the feet are planted (gait idle), the body isn't turning
		 */
		void update(float dt, const LR_point &v, float yawRate, bool standing = false);
		const LR_pose2 &pose() const { return _pose; }
		/** gyro bias learned while standing, rad/s (taken off the yaw rate) */
		float bias() const { return _bias; }
		void setBiasTau(float s) { _tau = s; }
	private:
		LR_pose2 _pose = { 0, 0, 0 };
		float    _bias = 0;
		float    _still = 0;     // s standing
		float    _learned = 0;   // s of standing used for the bias (fast start, then the time constant)
		float    _tau = 5.0f;
};

enum LidarMatch : uint8_t {
	LIDAR_MATCH_NONE = 0,   // no scan yet
	LIDAR_MATCH_OK,         // the scan fit the map, pose corrected
	LIDAR_MATCH_NEW,        // mostly unmapped area: odometry kept, scan added
	LIDAR_MATCH_REJECTED,   // poor fit: odometry kept, scan not added
	LIDAR_MATCH_FEW         // too few points in the scan
};

class LidarSlam {
	public:
		/** false if the map could not be allocated */
		bool begin(const LidarParams &lidar, const LidarMapParams &map);
		bool ready() const { return _grid.ready(); }
		/** Clear the map; the robot is at the map origin, facing +y. */
		void reset();
		/** Keep the map as it is (localize only), or keep adding to it. */
		void setFrozen(bool on) { _frozen = on; }
		bool frozen() const { return _frozen; }

		/**
		 * One revolution. odomStart/odomEnd: odometry at the start and the
		 * end of the revolution (any frame, only the motion between scans
		 * and within the scan is used).
		 */
		LidarMatch addScan(const LidarScan &scan, const LR_pose2 &odomStart, const LR_pose2 &odomEnd);
		/**
		 * A scan collected some other way (the ToF room scan, TofScanBuilder):
		 * n points (at most LR_LIDAR_BINS) already in the body frame of
		 * odomEnd, mm; beams start at the body origin.
		 */
		LidarMatch addPoints(const float *x, const float *y, uint16_t n, const LR_pose2 &odomEnd);
		/**
		 * Something the scans can't see (too low, or the scan was thrown away)
		 * stopped the robot at map point (x, y): make that cell occupied so the
		 * planner goes around it. Not while frozen.
		 */
		void markOccupied(float x, float y);
		/** Robot pose in the map now, from the odometry now (between scans). */
		LR_pose2 pose(const LR_pose2 &odomNow) const { return lrPoseCompose(_mapFromOdom, odomNow); }
		/** Robot pose in the map at the end of the last scan. */
		const LR_pose2 &lastPose() const { return _pose; }

		const OccupancyGrid &grid() const { return _grid; }
		const LidarParams &lidar() const { return _lidar; }
		LidarMatch lastMatch() const { return _last; }
		float lastScore() const { return _score; }
		/** Correction of the last accepted match against the odometry guess, mm / rad */
		float lastJump() const { return _jump; }
		float lastTurn() const { return _turn; }
		uint32_t scans() const { return _scans; }
		uint32_t rejected() const { return _rejected; }
		/** Scan points of the last scan in the body frame of its end pose, mm (count = return). */
		uint16_t points(const float *&x, const float *&y) const { x = _px; y = _py; return _n; }

	private:
		LidarParams    _lidar;
		LidarMapParams _params;
		OccupancyGrid  _grid;
		LR_pose2 _mapFromOdom = { 0, 0, 0 };
		LR_pose2 _pose = { 0, 0, 0 };
		LR_pose2 _lastInsert = { 0, 0, 0 };
		bool     _frozen = false;
		bool     _empty = true;
		LidarMatch _last = LIDAR_MATCH_NONE;
		float    _score = 0;
		float    _jump = 0;
		float    _turn = 0;
		uint32_t _scans = 0;
		uint32_t _rejected = 0;
		float    _px[LR_LIDAR_BINS];
		float    _py[LR_LIDAR_BINS];
		uint16_t _n = 0;
		float    _dirX[LR_LIDAR_BINS];
		float    _dirY[LR_LIDAR_BINS];
		float    _originX = 0;   // body frame start of the beams (the sensor), mm
		float    _originY = 0;

		void  deskew(const LidarScan &scan, const LR_pose2 &odomStart, const LR_pose2 &odomEnd);
		void  match(LR_pose2 &pose, int level, int iterations) const;
		float score(const LR_pose2 &pose, float &known, float &freeShare) const;
		void  insert(const LR_pose2 &pose);
		LidarMatch process(const LR_pose2 &odomEnd);
};

#endif
