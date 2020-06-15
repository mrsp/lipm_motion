#ifndef __DCMPLANNER_H__
#define __DCMPLANNER_H__
#include <eigen3/Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <lipm_motion/dcmDynamics.h>
#define Np 102
using namespace Eigen;
using namespace std;
class dcmPlanner
{
    private:
        dcmDynamics dcmDynamicsX, dcmDynamicsY;
        double   du_x, du_y, qx, qv, u_x, u_y;
        Vector2d DCM_, VRP_;
 	    Vector3d x,y,x_,y_;
        Vector3d xe, ye;
        MatrixXd Fx, Fv, Fxu, Fvu, R, Qx, Qv, K_X, K_V,H , tmpb, H_inv, Ad, Ae, Cd, A, L;
        VectorXd K_v, VRPRefX, VRPRefY, Cksi,  Ce, Cx, Be, Bd, temp, K_x, B, C;
        bool planAvailable;
        VectorXd DCM_d, CoM_d, VRP_d;
        double comZ, g, dt, omega;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double vrpx_d,comx_d,comdx_d,dcmx_d,dcmdx_d, vrpy_d,comy_d,comdy_d,dcmy_d,dcmdy_d,  comddx_d, comddy_d;
        bool firstrun;
        boost::circular_buffer<VectorXd> DCMBuffer, CoMBuffer, VRPBuffer;

        dcmPlanner(int bsize);
        
        void setState(Vector2d DCM, Vector2d CoM, Vector2d ZMP);
        
        void plan(boost::circular_buffer<VectorXd> & VRPRef);
        
        void setParams(double comZ_, double g_, double dt_);
        void emptyPlan();
        void init();
};
#endif
