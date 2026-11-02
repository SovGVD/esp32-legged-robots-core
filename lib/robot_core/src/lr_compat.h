/**
 * Portability shim so the motion libraries (geometry, IK, gait, transition,
 * planner) compile both in the Arduino firmware and in native PC builds
 * (unit tests, simulation) without Arduino.h.
 */
#ifndef LR_COMPAT_H
#define LR_COMPAT_H

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
	#include <Arduino.h>
#else
	#ifndef sq
		#define sq(x) ((x)*(x))
	#endif
	#ifndef constrain
		#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
	#endif
	#ifndef PI
		#define PI 3.1415926535897932384626433832795
	#endif
#endif

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif
#ifndef M_PI_2
	#define M_PI_2 1.57079632679489661923
#endif
#ifndef M_PI_4
	#define M_PI_4 0.78539816339744830962
#endif

#endif
