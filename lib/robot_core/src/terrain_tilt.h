/**
 * The body tilt the terrain map uses to place ToF readings. 1 deg of tilt
 * moves a reading 4 mm at 250 mm, enough to put a foot into a gap, so the
 * choice matters (measured in sim/, see sim/README "Map pose"):
 *
 *  FILTERED  the leveler's commanded tilt plus the IMU's unplanned tilt with
 *            the slow walking bias (1-2 deg) taken out and smoothed. With the
 *            raw IMU the hexapod's mapped floor crept up 10-20 mm in 10 s.
 *  COMMANDED the commanded tilt only (noisy on steps and bars).
 *  RAW       the IMU as measured. Best for the dog, whose trot rocks faster
 *            than the filter.
 *
 * Also ages the map only while the gait steps: odometry drifts with the
 * steps, not the time standing, and a robot waiting at a gap must still
 * remember it.
 */
#ifndef LR_TERRAIN_TILT_H
#define LR_TERRAIN_TILT_H

#include "geometry.h"
#include "terrain_map.h"

enum TerrainTiltMode : uint8_t {
	TERRAIN_TILT_FILTERED  = 0,
	TERRAIN_TILT_COMMANDED = 1,
	TERRAIN_TILT_RAW       = 2,
};

class TerrainTilt {
	public:
		void begin(TerrainTiltMode mode, float biasTau = 1.5f, float unplannedTau = 0.3f);
		void reset();
		void setMode(TerrainTiltMode mode) { _mode = mode; }
		TerrainTiltMode mode() const { return _mode; }
		void setTimeConstants(float biasTau, float unplannedTau) { _biasTau = biasTau; _unplannedTau = unplannedTau; }
		/**
		 * Every motion tick.
		 * @param imu        measured tilt (firmware convention)
		 * @param correction leveler output (imuCorrection); the commanded tilt is -correction
		 * @param walking    the gait is stepping
		 * @param map        aged while walking, may be nullptr
		 */
		void update(float dt, const LR_angle &imu, const LR_angle &correction, bool walking, TerrainMap *map);
		/** Body tilt for TerrainMap::addReading (pitch, roll; yaw untouched = 0). */
		LR_angle tilt() const { return _tilt; }

	private:
		TerrainTiltMode _mode = TERRAIN_TILT_FILTERED;
		float    _biasTau = 1.5f;
		float    _unplannedTau = 0.3f;
		LR_angle _bias      = { 0, 0, 0 };
		LR_angle _unplanned = { 0, 0, 0 };
		LR_angle _tilt      = { 0, 0, 0 };
};

#endif
