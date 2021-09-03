#ifndef  __ZMPPLANER_H__
#define  __ZMPPLANER_H__

#include <lipm_motion/KWalkMat.h>
#include <lipm_motion/motionDefines.h>
#include <queue>          // std::queue
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

//#include "StepAdjustment.h"
using namespace Eigen;
using namespace std;

class zmpPlanner
{
    
private:
    WalkInstruction planned;
    KWalkMat interp;
    double HX, HY, DS_Instructions,  MaxStepX,  MinStepX,  MaxStepY, MinStepY, MaxStepZ, dt;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VectorXd start, target, startL, startR, targetR, targetL, footR, footL, ZMPref, footR_, footL_, v, omega;
    /** ZMP Buffer for the Prediction Horizon **/
    boost::circular_buffer<VectorXd> ZMPbuffer,footRbuffer, footLbuffer;
    /** Queues needed for planning **/
    std::queue<WalkInstruction> stepAnkleQ;
    std::queue<VectorXd> zmpQ, stepLQ, stepRQ;
    bool planAvailable;

    Quaterniond qR, qR_, qL, qL_;

    //Methods
    void setFeet(VectorXd  sl, VectorXd sr);
    VectorXd computeDesiredZMP(VectorXd sl, VectorXd sr, WalkInstruction i);
    void emptyPlan();
    void plan(Vector2d COP, VectorXd lf, VectorXd rf);
    void setParams(double HX_, double HY_, int DS_Instructions_, double MaxStepX_, double MinStepX_, double MaxStepY_, double MinStepY_,double MaxStepZ_, double dt_);
    zmpPlanner(int bsize);
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

    inline Eigen::Vector3d logMap(const Eigen::Matrix3d &R_)
    {
        Eigen::Vector3d w;
        double acosV = (R_(0, 0) + R_(1, 1) + R_(2, 2) - 1.) * 0.5;
        double theta = std::acos(std::min(std::max(acosV, -1.), 1.));

        w = Eigen::Vector3d(R_(2, 1) - R_(1, 2), R_(0, 2) - R_(2, 0), R_(1, 0) - R_(0, 1));
        w *= sinc_inv(theta) * 0.5;

        return w;
    }

    double sinc_inv(const double x)
    {
    constexpr double taylor_0_bound = std::numeric_limits<double>::epsilon();
    constexpr double taylor_2_bound = std::sqrt(taylor_0_bound);
    constexpr double taylor_n_bound = std::sqrt(taylor_2_bound);

    // We use the 4th order taylor series around 0 of x/sin(x) to compute
    // this function:
    //
    //     x^2  7x^4
    // 1 + ── + ──── + O(x^6)
    //     6    360
    // this approximation is valid around 0.
    // if x is far from 0, our approximation is not valid
    // since x^6 becomes non neglectable we use the normal computation of the function
    // (i.e. taylor_2_bound^6 + taylor_0_bound == taylor_0_bound but
    //       taylor_n_bound^6 + taylor_0_bound != taylor_0).

    if(std::abs(x) >= taylor_n_bound)
    {
        return (x / std::sin(x));
    }
    else
    {
        // x is bellow taylor_n_bound so we don't care of the 6th order term of
        // the taylor series.
        // We set the 0 order term.
        double result = 1;

        if(std::abs(x) >= taylor_0_bound)
        {
        // x is above the machine epsilon so x^2 is meaningful.
        double x2 = x * x;
        result += x2 / 6;

        if(std::abs(x) >= taylor_2_bound)
        {
            // x is upper the machine sqrt(epsilon) so x^4 is meaningful.
            result += 7 * (x2 * x2) /360;
        }
        }

        return (result);
    }
    }












};
#endif
