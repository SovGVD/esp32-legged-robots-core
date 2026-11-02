#ifndef LR_GEOMETRY_H
#define LR_GEOMETRY_H

#include "lr_compat.h"

// Up to 6 dof should be enought for now
#define ALPHA   0
#define BETA    1
#define GAMMA   2
#define DELTA   3
#define EPSILON 4
#define ZETA    5
#define UNKNOWN_ANGLE 127

#define TITLE_ALPHA   "alpha"
#define TITLE_BETA    "beta"
#define TITLE_GAMMA   "gamma"
#define TITLE_DELTA   "delta"
#define TITLE_EPSILON "epsilon"
#define TITLE_ZETA    "zeta"

#define XAXIS     0
#define YAXIS     1
#define ZAXIS     2
#define PITCHAXIS 3
#define ROLLAXIS  4
#define YAWAXIS   5

/**
 * Maximal number of legs any supported robot has (hexapod).
 */
#define LR_MAX_LEGS 6

// point in 3D, mm
typedef struct {
	float x;
	float y;
	float z;
} LR_point;

// angle in 3D, rad
typedef struct {
	float pitch;   // X axis
	float roll;    // Y axis
	float yaw;     // Z axis
} LR_angle;

//body
typedef struct {
	LR_point position;
	LR_angle orientation;
} LR_figure;

// Move command, every value is normalized to [-1, 1]
typedef struct {
	LR_point move;
	LR_angle rotate;
} LR_moveVector;

/**
 * Rotate a point around Z axis.
 */
static inline LR_point lrRotateZ(LR_point p, float s, float c)
{
	LR_point r = { p.x * c - p.y * s, p.x * s + p.y * c, p.z };
	return r;
}

#endif
