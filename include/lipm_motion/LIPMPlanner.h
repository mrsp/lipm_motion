#ifndef __LIPMPlanner_H__
#define __LIPMPlanner_H__
#include <eigen3/Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <lipm_motion/LIPMDynamics.h>
#define Np 152
using namespace Eigen;
using namespace std;
class LIPMPlanner
{
    private:
        LIPMDynamics LIPMDynamicsX, LIPMDynamicsY;
        double   qv, u_x, u_y;
        Vector2d DCM_, VRP_;
 	    Vector4d x,y,x_,y_;
        Matrix4d A;
        Vector4d B,C;
        MatrixXd  Fv, Fvu, R, Qv, H , tmpb, H_inv;
        VectorXd K_v, ZMPRefX, ZMPRefY, temp,  U_x, U_y;
        bool planAvailable;
        VectorXd dCoM_d, CoM_d, ZMP_d, DCM_d;
        double comZ, g, dt, omega;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double ZMPx_d,comx_d,comdx_d,dcmx_d,dcmdx_d, ZMPy_d,comy_d,comdy_d,dcmy_d,dcmdy_d,  comddx_d, comddy_d;
        bool firstrun;
        boost::circular_buffer<VectorXd> DCMBuffer, CoMBuffer, ZMPBuffer;

        LIPMPlanner(int bsize);
        
        void setState(Vector2d CoM, Vector2d vCoM, Vector2d ZMP);
        
        void plan(boost::circular_buffer<VectorXd> & ZMPRef);
        
        void setParams(double comZ_, double g_, double dt_);
        void emptyPlan();
        void init();
};
#endif
