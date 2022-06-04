/**
 * Inverse Kinematics for bipedad (two legs) robot with 6DOF leg
 *
 * Developed by Gleb Devyatkin (SovGVD) in 2022
 */
 
 /**
  * All axis aligned with MPU9250

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
		IK(LR_figure &bodyObj, leg legs[LEG_NUM]);
		iksolver solve(uint8_t legId);
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
