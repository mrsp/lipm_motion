#ifndef __LIPMMOTION_H__
#define __LIPMMOTION_H__
#include <lipm_motion/zmpPlanner.h>
#include <lipm_motion/LIPMPlanner.h>
//#include <lipm_motion/dcmPlanner.h>

#include <iostream>

struct DesiredStepTarget
{
    Vector3d position;
    Quaterniond orientation;
    uint leg; //support leg - 0 is right, 1 is left
};

struct MotionPlanTarget
{
    Vector3d lfoot_position;
    Quaterniond lfoot_orientation;
    Vector3d rfoot_position;
    Quaterniond rfoot_orientation;
    Vector3d CoM_position;
    Vector3d CoM_velocity;
    Vector3d COP;
    std::vector<DesiredStepTarget> footsteps;
};



class lipm
{
private:
    zmpPlanner* zp;
    LIPMPlanner* dp;
    //dcmPlanner* dp;

    int SS_Instructions, DS_Instructions;
    Quaterniond q;
    double g, h, MaxStepX, MaxStepY, MinStepX, MinStepY, MaxStepZ;
    double dt, HX, HY, Tss, Tds;
public:
    bool isPlanAvailable;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ~lipm();
    lipm();
    void desiredFootstepsCb(MotionPlanTarget* goal, boost::circular_buffer<VectorXd>& ZMPdBuffer, boost::circular_buffer<VectorXd>& DCMBuffer, boost::circular_buffer<VectorXd>& CoMBuffer, 
        boost::circular_buffer<VectorXd>& ZMPBuffer, boost::circular_buffer<VectorXd>& footLBuffer, boost::circular_buffer<VectorXd>& footRBuffer);
};
#endif
