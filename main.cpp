#include "RobotParameters.h"
#include "zmpPlanner.h"
#include "dcmPlanner.h"
#include "postureStabilizer.h"
#include <iostream>

int main()
{


    /** Compute Reference Points: x-y-z-roll-pitch-yaw **/
    VectorXd planL;
    planL.resize(6);
    planL.setZero();
    planL(1) = 0.05;
    VectorXd planR;
    planR.resize(6);
    planR.setZero();
    planR(1) = -0.05;



    RobotParameters robot;
    zmpPlanner tp(robot);
    dcmPlanner dp(robot);
    tp.setFeet(planL, planR);
    Vector2f DCM, CoM, VRP;
    DCM.setZero();
    CoM.setZero();
    VRP.setZero();
    dp.setState(DCM,  CoM,  VRP);

    while (tp.stepAnkleQ.size() < 3)
    {

            /** Initial Walking Instruction **/
            WalkInstruction i;
            i.target = planL;
            i.targetSupport = SUPPORT_LEG_RIGHT;
            /** ZMP in the Middle of Convex Hull **/
            i.targetZMP = SUPPORT_LEG_RIGHT;
            /** Number of Discrete Time steps of the Initial Walking Instruction **/
            i.steps = 100;
            i.step_id = -1;
            /** Adding the Walking Instruction to the Walking Buffer for Execution **/
            tp.stepAnkleQ.push(i);

    }
    tp.plan();
    dp.plan(tp.ZMPbuffer,  DCM,  CoM,   VRP);
    std::cout<<" "<<tp.ZMPbuffer.size()<<std::endl;
    std::cout<<" "<<dp.VRPBuffer.size()<<std::endl;
    std::cout<<" "<<dp.DCMBuffer.size()<<std::endl;
    std::cout<<" "<<dp.CoMBuffer.size()<<std::endl;
    std::cout<<" "<<tp.footLbuffer.size()<<std::endl;
    std::cout<<" "<<tp.footRbuffer.size()<<std::endl;

    VectorXf comX, comY;
    comX.resize(dp.CoMBuffer.size());
    comY.resize(dp.CoMBuffer.size());
    int j=0;
    while(dp.CoMBuffer.size()>0)
    {
        comX(j) = dp.CoMBuffer[j](0);
        comY(j) = dp.CoMBuffer[j](1);
        dp.CoMBuffer.pop_front();
    }

    return 0;
}
