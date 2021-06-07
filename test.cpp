#include <lipm_motion/lipm.h>
#include <ros/ros.h>
#include "/home/master/catkin_ws/src/matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lipm_motion_node");
    ros::NodeHandle nh;
   
    lipm* lm;
    lm = new lipm();
    MotionPlanTarget* goal = new MotionPlanTarget;

    goal->lfoot_position  = Vector3d(0,0.05,0);
    goal->lfoot_orientation = Quaterniond(1.0,0,0,0);
    goal->rfoot_position  = Vector3d(0,-0.05,0);
    goal->rfoot_orientation = Quaterniond(1.0,0,0,0);
    goal->CoM_position = Vector3d(0,0,0.26818);
    goal->CoM_velocity = Vector3d(0,0,0);
    goal->COP = Vector3d(0,0,0);
    

    goal->footsteps.resize(4);
    goal->footsteps[0].leg = 0; //SWING LLEG
    goal->footsteps[0].position  = Vector3d(0.05,0.05,0);
    goal->footsteps[0].orientation = Quaterniond(1.0,0,0,0);

    goal->footsteps[1].leg = 1;
    goal->footsteps[1].position  = Vector3d(0.05,-0.05,0);
    goal->footsteps[1].orientation = Quaterniond(1.0,0,0,0);

    goal->footsteps[2].leg = 0;
    goal->footsteps[2].position  = Vector3d(0.1,0.05,0);
    goal->footsteps[2].orientation = Quaterniond(1.0,0,0,0);

    goal->footsteps[3].leg = 1;
    goal->footsteps[3].position  = Vector3d(0.1,-0.05,0);
    goal->footsteps[3].orientation = Quaterniond(1.0,0,0,0);
    
    boost::circular_buffer<VectorXd> ZMPBuffer, DCMBuffer,CoMBuffer, VRPBuffer, footLBuffer, footRBuffer;
    lm->desiredFootstepsCb(goal, ZMPBuffer, DCMBuffer,CoMBuffer, VRPBuffer, footLBuffer, footRBuffer);
    int j = 0;
    std::vector<double> CoMx, CoMy, ZMPx, ZMPy,  ZMPx_ref, ZMPy_ref, footLx,  footLz, footLy, footRx, footRy, footRz;

    while(j<CoMBuffer.size())
    {
        CoMx.push_back(CoMBuffer[j](0));
        CoMy.push_back(CoMBuffer[j](1));
        ZMPx.push_back(VRPBuffer[j](0));
        ZMPy.push_back(VRPBuffer[j](1));
        ZMPx_ref.push_back(ZMPBuffer[j](0));
        ZMPy_ref.push_back(ZMPBuffer[j](1));
        footLx.push_back(footLBuffer[j](0));
        footLy.push_back(footLBuffer[j](1));
        footRx.push_back(footRBuffer[j](0));
        footRy.push_back(footRBuffer[j](1));
        
        footLz.push_back(footLBuffer[j](2));
        footRz.push_back(footRBuffer[j](2));
        j++;
    }

    plt::plot(CoMx);
    plt::plot(ZMPx);
    plt::plot(footRx);
    plt::plot(footLx);
    plt::plot(ZMPx_ref);
    plt::show();
    plt::plot(CoMy);
    plt::plot(ZMPy);
    plt::plot(footRy);
    plt::plot(footLy);
    plt::plot(ZMPy_ref);
    plt::show();
    plt::plot(footRz);
    plt::plot(footLz);
    plt::show();


}
