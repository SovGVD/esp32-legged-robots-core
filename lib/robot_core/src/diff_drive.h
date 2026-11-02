/**
 * Differential drive for tracked or wheeled robots (the tank model).
 *
 * The move command is mixed into left and right track speeds in -1..1,
 * desaturated so a full turn while driving keeps its turn ratio instead of
 * clipping one side, and slew limited so the motors don't see a step from
 * full forward to full reverse. Each track then becomes a pair of duties for
 * a two-input H-bridge (L298N, DRV8833, ...): one input driven, the other low.
 */
#ifndef LR_DIFF_DRIVE_H
#define LR_DIFF_DRIVE_H

#include "lr_compat.h"
#include "geometry.h"

struct DiffDriveParams {
	float    maxAccel;   // track speed change per second, full scale = 1
	float    deadband;   // speeds below this stop the motor
	uint16_t minDuty;    // duty that just starts the motor moving
	uint16_t maxDuty;    // duty at full speed (255 for 8 bit PWM)
};

struct DiffDriveDuty {
	uint16_t in1;        // forward input
	uint16_t in2;        // reverse input
};

// Mix forward speed and turn rate (both -1..1, positive turn = left track faster) into track speeds
void lrDiffDriveMix(float forward, float turn, float &left, float &right);

// One track speed (-1..1) into H-bridge duties
DiffDriveDuty lrDiffDriveDuty(float speed, const DiffDriveParams &params);

class DiffDrive {
	public:
		void  begin(const DiffDriveParams &params);
		// forward and turn are -1..1; returns after applying the slew limit
		void  update(float dt, float forward, float turn);
		// stop at once, no ramp (failsafe, HAL disabled)
		void  stop();
		float left()  const { return _left; }
		float right() const { return _right; }
		DiffDriveDuty leftDuty()  const { return lrDiffDriveDuty(_left,  _params); }
		DiffDriveDuty rightDuty() const { return lrDiffDriveDuty(_right, _params); }

	private:
		DiffDriveParams _params = { 2.0f, 0.05f, 0, 255 };
		float _left  = 0;
		float _right = 0;
};

#endif
