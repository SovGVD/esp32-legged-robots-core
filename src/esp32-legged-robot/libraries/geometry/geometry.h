#ifndef geometry_h
#define geometry_h

// Up to 6 dof should be enought for now
#define ALPHA 0
#define BETA  1
#define GAMMA 2
#define DELTA 3
#define EPSILON 4
#define ZETA 5

#define XAXIS     0
#define YAXIS     1
#define ZAXIS     2
#define PITCHAXIS 3
#define ROLLAXIS  4
#define YAWAXIS   5

// pount in 3D
typedef struct {
	double x;
	double y;
	double z;
} LR_point;

// angle in 3D
typedef struct {
	double pitch;   // X axis
	double roll;    // Y axis
	double yaw;     // Z axis
} LR_angle;

//body
typedef struct {
	LR_point position;
	LR_angle orientation;
} LR_figure;

//move
typedef struct {
	LR_point move;
	LR_angle rotate;
} LR_moveVector;

#endif
