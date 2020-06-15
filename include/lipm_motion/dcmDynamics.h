#ifndef  __DCMDYNAMICS__H__
#define  __DCMDYNAMICS__H__

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <queue>

using namespace Eigen;
using namespace std;

class dcmDynamics
{
private:
    Matrix3d  A, I;
    Matrix<double,3,1> B;
    Vector3d x;
    double omega, dt;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void setState(Vector3d x_);
    void integrate(double u_);
    dcmDynamics();
    double com, vrp, dcm;
    Vector3d getState();
    void init();
    void setParams(double omega_, double dt_);
};
#endif
