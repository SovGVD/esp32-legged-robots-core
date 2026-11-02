/**
 * Leg angle -> servo angle -> pulse width, the last step before the PWM
 * hardware. Shared by the firmware HAL and the simulator bridge, so both
 * produce exactly the same pulses.
 */
#ifndef LR_SERVO_HAL_H
#define LR_SERVO_HAL_H

#include <stdint.h>
#include "leg.h"

/**
 * Servo calibration: measured pulse widths at fixed servo angles, linear
 * in between.
 */
typedef struct {
	uint8_t minAngle; // minimal servo angle, deg
	uint8_t maxAngle; // maximal servo angle, deg
	uint16_t degMin;  // PWM pulse at minAngle deg
	uint16_t deg30;   // PWM pulse at 30 deg
	uint16_t deg50;
	uint16_t deg70;
	uint16_t deg90;   // PWM pulse at 90 deg
	uint16_t deg110;
	uint16_t deg130;
	uint16_t deg150;  // PWM pulse at 150 deg
	uint16_t degMax;  // PWM pulse at maxAngle deg
} servoProfile;

/** Servo can only reach 0..PI. */
float lrLimitServoAngle(float angle);

/** Leg joint angle -> servo shaft angle (middle, gear ratio, mirroring, trim). */
float lrHalAngle(float angle, float mid, float trimAngle, float gearRatio, bool inverse);

/** Servo shaft angle (rad) -> pulse width (us) using the calibration profile. */
uint16_t lrAngleToPulse(const servoProfile &profile, float angleRad);

/** All joints of a leg (first `dof` of alpha..zeta) -> pulse widths. */
void lrLegPulses(const leg &l, uint8_t dof, const servoProfile &profile, uint16_t *pulses);

#endif
