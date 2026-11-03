/**
 * Small height map of the ground around the robot, built from distance
 * sensors (ToF: VL53L1X with ROI zones, VL53L5CX 4x4/8x8 zones) that look
 * forward and down. The gait asks it for the ground height where a foot will
 * land and the highest point along the swing, so it can lift a foot over an
 * obstacle and put it down on top of it before touching it.
 *
 * Heights are in the gait's world frame (odometry, mm), the same frame as
 * leg.foot: z = -80 is flat ground under the dog's neutral stance. Odometry
 * drifts on a real robot, so cells are forgotten after `maxAge` seconds;
 * the map only has to be right for the next second or two of steps.
 *
 * The grid scrolls with the robot: a cell is addressed by its world cell
 * index modulo the grid size, and remembers which world cell it holds.
 */
#ifndef LR_TERRAIN_MAP_H
#define LR_TERRAIN_MAP_H

#include "geometry.h"

#ifndef LR_TERRAIN_GRID
	#define LR_TERRAIN_GRID 40   // cells per side (40 x 40 x 12 bytes = 19 KB)
#endif

/**
 * One multizone ToF sensor fixed to the body. The sensor looks along body +Y
 * (forward) when yaw = 0 and tilt = 0; tilt > 0 turns it down, yaw > 0 turns
 * it left (counter-clockwise seen from above). Zones are row-major from the
 * top-left of the sensor's view: zone = row * zones + col, row 0 looks
 * highest, col 0 leftmost.
 */
typedef struct {
	LR_point position;   // mm, body frame
	float    yaw;        // rad
	float    tilt;       // rad, down
	float    fov;        // rad, full field of view (square)
	uint8_t  zones;      // zones per side: 1 (one ray), 4 (4x4) or 8 (8x8)
	float    maxRange;   // mm, readings beyond are ignored
} TofSensor;

typedef struct {
	float cellSize;      // mm
	float maxAge;        // s, cells older than this are unknown
	float smoothing;     // 0..1, weight of a new reading that is lower than the cell (noise)
	float jump;          // mm, a reading this much higher than the cell replaces it at once (obstacle top)
} TerrainMapParams;

class TerrainMap
{
	public:
		void begin(const TerrainMapParams &params);
		void clear();
		/** Advance the map clock (ages the cells). */
		void tick(float dt) { _time += dt; }

		/**
		 * Unit ray of one zone in the body frame (sensor at s.position).
		 */
		static LR_point zoneDirection(const TofSensor &s, uint8_t zone);

		/**
		 * Add one zone reading.
		 * @param body    body pose in the gait world frame: position (mm) and
		 *                yaw from the gait, pitch/roll = the measured tilt
		 *                (IMU, firmware convention, same as BodyKinematics)
		 * @param distance mm along the zone ray; <= 0 or > maxRange: no target
		 */
		void addReading(const TofSensor &s, uint8_t zone, float distance, const LR_figure &body);
		/** Add a point (gait world frame, mm). */
		void addPoint(const LR_point &p);

		/** Ground height at (x, y); false if the cell is unknown or too old. */
		bool heightAt(float x, float y, float &z) const;
		/**
		 * Highest known point within `radius` mm of the segment a-b (XY).
		 * false if no cell there is known.
		 */
		bool maxHeightAlong(const LR_point &a, const LR_point &b, float radius, float &z) const;
		/**
		 * Height spread (max - min) of the known points `reach` mm before,
		 * at and after (x, y) along the unit direction (dx, dy): large when
		 * (x, y) is on or next to an edge across that direction. -1 if fewer
		 * than two of them are known.
		 */
		float edge(float x, float y, float dx, float dy, float reach) const;

		uint16_t knownCells() const;
		const TerrainMapParams &params() const { return _params; }

	private:
		typedef struct {
			int16_t ix;
			int16_t iy;
			float   z;
			float   time;   // < 0: empty
		} cell;

		TerrainMapParams _params = { 20.0f, 3.0f, 0.3f, 8.0f };
		cell  _cells[LR_TERRAIN_GRID * LR_TERRAIN_GRID];
		float _time = 0;

		int16_t index(float v) const { return (int16_t)floorf(v / _params.cellSize); }
		const cell *find(int16_t ix, int16_t iy) const;
};

/**
 * Rotate a body-frame vector by a body orientation, the way BodyKinematics
 * places the shoulders (pitch about Y, roll about X, then yaw about Z).
 */
LR_point lrRotateBody(const LR_point &p, const LR_angle &o);

#endif
