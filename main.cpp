#include "RobotParameters.h"
#include "KMat.hpp"
#include "zmpPlanner.h"
#include "dcmPlanner.h"
#include "postureStabilizer.h"
#include <iostream>
#define DEF_STEP_ID -1
int main()
{
    RobotParameters robot;
    zmpPlanner tp(robot);
    tp.firstrun = false;
    /** Compute Reference Points in the x-y plane with orientation about the vertical axis z **/
    KVecFloat3 planL = KVecFloat3(0, 0.045, 0);
    KVecFloat3 planR = KVecFloat3(0, -0.045, 0);
    /** Trajectory Planner **/
    tp.setFeet(planL, planR);

    if (tp.planAvailable)
        tp.emptyPlan();

    while (tp.stepAnkleQ.size() < 3)
    {

            /** Initial Walking Instruction **/
            WalkInstruction i;
            i.target = planL;
            /** ZMP in the Middle of Convex Hull **/
            i.targetZMP = SUPPORT_LEG_RIGHT;
            /** Number of Discrete Time steps of the Initial Walking Instruction **/
            i.steps = 100;
            i.step_id = DEF_STEP_ID;
            /** Adding the Walking Instruction to the Walking Buffer for Execution **/
            tp.stepAnkleQ.push(i);

    }
    tp.plan(KVecFloat2(0, 0), KVecFloat2(0,0), false);
 

    dcmPlanner dp(robot);
    Vector2f DCM, CoM, VRP;
    DCM.setZero();
    CoM.setZero();
    VRP.setZero();
    dp.setState(DCM,  CoM,  VRP);

    std::cout<<" "<<tp.ZMPbuffer.size()<<std::endl;
    while(tp.ZMPbuffer.size()>0)
    {
        std::cout<<" "<<tp.ZMPbuffer.front()(0)<<" "<<tp.ZMPbuffer.front()(1)<<std::endl;
        dp.Control(tp.ZMPbuffer,  DCM,  CoM,   VRP);
        tp.ZMPbuffer.pop_front();
    }
    postureStabilizer ps(robot);

    return 0;
}
