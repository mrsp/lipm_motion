#ifndef  __DCMDYNAMICS__H__
#define  __DCMDYNAMICS__H__

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <lipm_motion/RobotParameters.h>
#include <queue>

using namespace Eigen;
using namespace std;


class dcmDynamics
{
    
private:
    RobotParameters &robot;
    Matrix3d  A, I;
    Matrix<double,3,1> B;
    Vector3d x;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setState(Vector3d x_);
    void integrate(double u_);
    dcmDynamics(RobotParameters &robot_);
    double com, vrp, dcm;
    Vector3d getState();
};
#endif
