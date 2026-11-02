/**
 * Leg angle -> servo angle -> pulse width, the last step before the PWM
 * hardware. Shared by the firmware HAL and the simulator bridge, so both
 * produce exactly the same pulses.
 */
#ifndef LR_SERVO_HAL_H
#define LR_SERVO_HAL_H

#include <stdint.h>
#include "leg.h"
#include "hardware.h"

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

/**
 * Clamps each joint angle so its servo angle (after middle, gear ratio,
 * mirroring and trim) stays inside the profile's minAngle..maxAngle.
 * Returns true if any joint was clamped. The result is the pose the servos
 * really take, so the controller knows it instead of the pulse being
 * clamped silently in lrAngleToPulse.
 */
bool lrClampServoRange(const leg &l, LR_legAngle &angle, uint8_t dof, const servoProfile &profile);

/** Servo shaft angle -> leg joint angle, the inverse of lrHalAngle. */
float lrLegAngleFromServo(float servoAngle, float mid, float trimAngle, float gearRatio, bool inverse);

/** All joints of a leg (first `dof` of alpha..zeta) -> pulse widths. */
void lrLegPulses(const leg &l, uint8_t dof, const servoProfile &profile, uint16_t *pulses);

#endif
