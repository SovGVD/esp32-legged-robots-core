/**
 * PID
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2021
 */
 
#ifndef ANGLE_PID_h
#define ANGLE_PID_h

#include "../geometry/geometry.h"

class AnglePID
{
	public:
		AnglePID(LR_angle &current, LR_angle &target, LR_angle &output, double Kp, double Ki, double Kd, double minError, double maxError);
		void set(double Kp, double Ki, double Kd);
		void update();
	private:
		LR_angle      *_current;
		LR_angle      *_target;
		LR_angle      *_output;
		double        _Kp = 0.0;
		double        _Ki = 0.0;
		double        _Kd = 0.0;
		double        _minError;
		double        _maxError;
		LR_angle      P   = {0.0, 0.0, 0.0};
		LR_angle      I   = {0.0, 0.0, 0.0};
		LR_angle      D   = {0.0, 0.0, 0.0};
		LR_angle      err = {0.0, 0.0, 0.0};
		LR_angle      preCurrent = {0.0, 0.0, 0.0};
		LR_angle      dInput = {0.0, 0.0, 0.0};
		LR_angle      outputSum = {0.0, 0.0, 0.0};
		unsigned long _t  = 0;
		bool          pOnE = true;
};

#endif
