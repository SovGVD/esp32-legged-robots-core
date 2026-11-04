#include "terrain_tilt.h"

void TerrainTilt::begin(TerrainTiltMode mode, float biasTau, float unplannedTau)
{
	_mode = mode;
	_biasTau = biasTau;
	_unplannedTau = unplannedTau;
	reset();
}

void TerrainTilt::reset()
{
	_bias = _unplanned = _tilt = { 0, 0, 0 };
}

void TerrainTilt::update(float dt, const LR_angle &imu, const LR_angle &correction, bool walking, TerrainMap *map)
{
	// measured minus commanded: the tilt the gait didn't ask for
	float up = imu.pitch + correction.pitch;
	float ur = imu.roll  + correction.roll;
	if (walking && _biasTau > 0) {
		// over a second or two the body's unplanned tilt averages out, the IMU bias doesn't
		float k = dt / _biasTau;
		_bias.pitch += (up - _bias.pitch) * k;
		_bias.roll  += (ur - _bias.roll)  * k;
	}
	float k = _unplannedTau > 0 ? dt / _unplannedTau : 1.0f;
	if (k > 1) {
		k = 1;
	}
	_unplanned.pitch += (up - _bias.pitch - _unplanned.pitch) * k;
	_unplanned.roll  += (ur - _bias.roll  - _unplanned.roll)  * k;

	switch (_mode) {
		case TERRAIN_TILT_COMMANDED:
			_tilt.pitch = -correction.pitch;
			_tilt.roll  = -correction.roll;
			break;
		case TERRAIN_TILT_RAW:
			_tilt.pitch = imu.pitch;
			_tilt.roll  = imu.roll;
			break;
		default:
			_tilt.pitch = -correction.pitch + _unplanned.pitch;
			_tilt.roll  = -correction.roll  + _unplanned.roll;
			break;
	}
	_tilt.yaw = 0;
	if (walking && map != nullptr) {
		map->tick(dt);
	}
}

LR_angle FrameTilt::place(const TofSensor &s, const float *mm, const LR_angle &tilt, float gain, float maxDiff)
{
	LR_angle floor;
	LR_angle guess = { tilt.pitch + _offset.pitch, tilt.roll + _offset.roll, 0 };
	if (TerrainMap::floorTilt(s, mm, guess, floor) && fabsf(floor.pitch - tilt.pitch) < maxDiff
	    && fabsf(floor.roll - tilt.roll) < maxDiff) {
		_offset.pitch += (floor.pitch - tilt.pitch - _offset.pitch) * gain;
		_offset.roll  += (floor.roll  - tilt.roll  - _offset.roll)  * gain;
	}
	return { tilt.pitch + _offset.pitch, tilt.roll + _offset.roll, 0 };
}
