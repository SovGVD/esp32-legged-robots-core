#include "body_kinematics.h"

void BodyKinematics::update(leg *legs, uint8_t legCount, const LR_figure &body, const LR_angle &correction, const LR_moveVector &command)
{
	/**
	 * x' = x * (cosYaw*cosPitch) + y * (cosYaw*sinPitch*sinRoll - sinYaw*cosRoll) + z * (cosYaw*sinPitch*cosRoll + sinYaw*sinRoll)
	 * y' = x * (sinYaw*cosPitch) + y * (sinYaw*sinPitch*sinRoll + cosYaw*cosRoll) + z * (sinYaw*sinPitch*cosRoll - cosYaw*sinRoll)
	 * z' = x * (-sinPitch)       + y * (cosPitch*sinRoll)                         + z * (cosPitch*cosRoll)
	 * @see: https://en.wikipedia.org/wiki/Rotation_matrix#General_rotations
	 */
	float pitch = body.orientation.pitch - correction.pitch + command.rotate.pitch * commandTilt;
	float roll  = body.orientation.roll  - correction.roll  + command.rotate.roll  * commandTilt;

	float sinPitch = sinf(pitch);
	float cosPitch = cosf(pitch);
	float sinRoll  = sinf(roll);
	float cosRoll  = cosf(roll);
	float sinYaw   = sinf(body.orientation.yaw);
	float cosYaw   = cosf(body.orientation.yaw);

	for (uint8_t i = 0; i < legCount; i++) {
		const LR_point &d = legs[i].defaultBody;
		legs[i].body.x = body.position.x
		               + d.x * (cosYaw*cosPitch)
		               + d.y * (cosYaw*sinPitch*sinRoll - sinYaw*cosRoll)
		               + d.z * (cosYaw*sinPitch*cosRoll + sinYaw*sinRoll);

		legs[i].body.y = body.position.y
		               + d.x * (sinYaw*cosPitch)
		               + d.y * (sinYaw*sinPitch*sinRoll + cosYaw*cosRoll)
		               + d.z * (sinYaw*sinPitch*cosRoll - cosYaw*sinRoll);

		legs[i].body.z = body.position.z
		               + d.x * (-sinPitch)
		               + d.y * (cosPitch*sinRoll)
		               + d.z * (cosPitch*cosRoll);
	}
}
