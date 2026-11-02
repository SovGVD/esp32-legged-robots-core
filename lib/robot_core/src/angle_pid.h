/**
 * PID controller for pitch/roll/yaw.
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2021
 */

#ifndef LR_ANGLE_PID_H
#define LR_ANGLE_PID_H

#include "geometry.h"

class AnglePID
{
	public:
		AnglePID(float Kp, float Ki, float Kd, float minIntegral, float maxIntegral);
		void set(float Kp, float Ki, float Kd);
		/**
		 * @param dt seconds since the previous update
		 */
		LR_angle update(const LR_angle &current, const LR_angle &target, float dt);
		void reset();
	private:
		float    _Kp;
		float    _Ki;
		float    _Kd;
		float    _minIntegral;
		float    _maxIntegral;
		bool     _hasPrevious = false;
		LR_angle _previous    = {0, 0, 0};
		LR_angle _integral    = {0, 0, 0};
		float    axis(float current, float target, float previous, float &integral, float dt);
};

#endif
