/**
 * Inverse Kinematics for quadruped (four legs) robot
 * Boston Dynamics Spot style
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2021
 */
 
 /**
 * All axis aligned with MPU9250
 *
 *
 *               |   Z-> Y
 *               |   |
 *    --------0-+   v X
 *            | ...l1
 *        0---0
 *
 *
 *      beta.                 alpha.
 *           .                      .
 *            .  |   ^ Z             .|         ^ Z
 *             0 |   |          ^  0--0         |
 *     ------ / -+   X-> Y      .  |  +---  X <-Y
 *           / ...l2            d  |
 * gamma...0                    .  0
 *          \                   .  |
 *           \ ...l3            v  |
 *
 *
 */
 
/**
 *          0---0 (legLH)       0---0 (legLF)
 *              |               |
 *           +--0---------------0--+
 *           |                     |
 *           |                     |
 *           |                     |
 *           +--0---------------0--*
 *              |               |
 *          0---0 (legRH)       0---0 (legRF)
 */
#ifndef IK_h
#define IK_h

typedef struct {
	bool isSolved;
	LR_legAngle angle;
} iksolver;

class IK
{
	public:
		IK(LR_figure &bodyObj, leg (&legs)[LEG_NUM]);
		//void set(LR_figure &bodyObj, leg &legObj);
		iksolver solve(uint8_t legId);
		//point solveByAngle();	// use to init position by initial angles
		double normalizeAngleRad(double angleRad);
	private:
		LR_figure *_body;
		leg       *_legs[LEG_NUM];
		double    ikAtan2(double x, double y);
		double    ikAcos(double angleRad);
		double    ikAsin(double angleRad);
		LR_point  bodyBalance = {0, 0, 0};	// TODO this is just for tests
};

#endif
