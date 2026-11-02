/**
 * Keeps the body level with the IMU: tilts the body against the measured
 * pitch and roll (slope, uneven feet, sag on one side). Runs on top of the
 * gait and static stability; its output is the `correction` that
 * BodyKinematics subtracts from the body orientation.
 *
 * Angles use the firmware convention (the one BodyKinematics and the move
 * command use): pitch rotates about the forward (Y) axis, roll about the
 * sideways (X) axis. Map the IMU to it with IMU_SWAP_ROLL_PITCH / *_SIGN.
 */
#ifndef LR_BODY_LEVELER_H
#define LR_BODY_LEVELER_H

#include "geometry.h"

struct LevelerParams {
	float kp;       // correction per rad of tilt
	float ki;       // 1/s, the main term: removes a steady tilt
	float kd;       // s, damping on the measured tilt rate
	float limit;    // max correction, rad
	float rate;     // max correction change, rad/s
	float filter;   // low-pass time constant on the IMU angle, s (0 = off)
	float deadband; // tilt error the integral ignores, rad (IMU estimate noise while walking)
};

class BodyLeveler {
	public:
		void begin(const LevelerParams &params);
		void setEnabled(bool enabled);
		bool enabled() const { return _enabled; }
		/**
		 * @param measured body tilt from the IMU, rad
		 * @param target   wanted tilt, usually 0 (the command tilt is added separately)
		 * @param hold     keep the current correction (dynamic gaits: the
		 *                body rocks every step and the IMU estimate with it)
		 * Disabled, the correction ramps back to 0 at `rate`.
		 */
		const LR_angle &update(float dt, const LR_angle &measured, const LR_angle &target, bool hold = false);
		const LR_angle &correction() const { return _correction; }
		const LR_angle &filtered() const { return _filtered; }
		void reset();

	private:
		float axis(float dt, float measured, float previous, float target, float &integral, float current);

		LevelerParams _params = { 0.0f, 0.5f, 0.0f, 0.25f, 0.3f, 0.0f, 0.035f };
		bool     _enabled     = false;
		bool     _hasPrevious = false;
		LR_angle _filtered    = { 0, 0, 0 };
		LR_angle _integral    = { 0, 0, 0 };
		LR_angle _correction  = { 0, 0, 0 };
};

#endif
