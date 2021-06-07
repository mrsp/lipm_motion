#ifndef  __LIPMDynamics__H__
#define  __LIPMDynamics__H__

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <queue>

using namespace Eigen;
using namespace std;

class LIPMDynamics
{
private:
    Vector4d x;
    double omega, dt;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Matrix4d  A, Ad, I;
    Vector4d B, Bd;
    Vector4d C, Cd;
    void setState(Vector4d x_);
    void integrate(double u_);
    LIPMDynamics();
    double com, zmp, vcom, acom;
    Vector4d getState();
    void init();
    void setParams(double omega_, double dt_);
};
#endif
