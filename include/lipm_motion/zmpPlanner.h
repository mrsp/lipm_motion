#ifndef  __ZMPPLANER_H__
#define  __ZMPPLANER_H__

#include <lipm_motion/KWalkMat.h>
#include <lipm_motion/motionDefines.h>
#include <lipm_motion/RobotParameters.h>
#include <queue>          // std::queue
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

#define M_PI 3.14159265358979323846264338327950288
//#include "StepAdjustment.h"
using namespace Eigen;
using namespace std;

class zmpPlanner
{
    
private:
    RobotParameters &robot;
    WalkInstruction planned;
    KWalkMat interp;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VectorXd start, target, startL, startR, targetR, targetL, footR, footL, ZMPref;
    /** ZMP Buffer for the Prediction Horizon **/
    boost::circular_buffer<VectorXd> ZMPbuffer,footRbuffer, footLbuffer;
    /** Queues needed for planning **/
    std::queue<WalkInstruction> stepAnkleQ;
    std::queue<VectorXd> zmpQ, stepLQ, stepRQ;
    bool planAvailable;

    //Methods
    void setFeet(VectorXd  sl, VectorXd sr);
    VectorXd computeDesiredZMP(VectorXd sl, VectorXd sr, WalkInstruction i);
    void generatePlan();
    void emptyPlan();
    void plan();
    zmpPlanner(RobotParameters &robot_);
    //Step Adjustment
    //StepAdjustment sa;
    Vector2d dx, tempV;

    
    double anglemean(double l, double r) const
    {
        return (double) ((double) l + anglediff2( (double) r, (double) l) / 2.0);
    }
    float cropStep(double f_, double max_, double min_)
    {
        return fmax(min_, fmin(f_, max_));
    }

    inline static double wrapToPi(double angle) {
		while (angle > M_PI)
			angle -= 2.0 * M_PI;

		while (angle < -M_PI)
			angle += 2.0 * M_PI;

		return angle;
	}

	inline static double wrapTo2Pi(double angle) {
		while (angle > 2.0 * M_PI)
			angle -= 2.0 * M_PI;

		while (angle < -2.0 * M_PI)
			angle += 2.0 * M_PI;

		return angle;
	}

	inline static double wrapTo0_2Pi(double angle) {
		while (angle > 2.0 * M_PI)
			angle -= 2.0 * M_PI;

		while (angle < 0)
			angle += 2.0 * M_PI;

		return angle;
	}

	inline static double anglediff2(double a1, double a2) {
		return wrapToPi(wrapToPi(a1 + M_PI - a2) - M_PI);
	}

	inline static double anglediff(double a1, double a2) {
		return fabs(wrapTo0_2Pi(a1 + M_PI - a2) - M_PI);
	}
};
#endif
