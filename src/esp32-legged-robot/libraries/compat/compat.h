/**
 * Portability shim so the motion libraries (geometry, IK, gait, transition,
 * planner) compile both in the Arduino firmware and in native PC builds
 * (unit tests, simulation) without Arduino.h.
 */
#ifndef LR_COMPAT_h
#define LR_COMPAT_h

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

#endif
