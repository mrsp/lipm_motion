#include "footMotionPlanner.h"

footMotionPlanner::footMotionPlanner(RobotParameters &robot_) : robot(robot_)
{
    default_params();
    std::cout << "footMotionPlanner Module Initialized Successfully" << std::endl;
}
void footMotionPlanner::reset()
{
    default_params();
    std::cout << "footMotionPlanner Module Reseted Successfully" << std::endl;
}

void footMotionPlanner::default_params()
{
    StepZ_ = robot.getWalkParameter(StepZ);
    FootL.zero();
    FootR.zero();
    startR.zero();
    startL.zero();
    planR.zero();
    planL.zero();
    startLz = 0.000;
    startRz = 0.000;
    planLz = 0.000;
    planRz = 0.000;
    FootLz = 0.00;
    FootRz = 0.00;
}

void footMotionPlanner::setFootStartXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight)
{
    if (isRight)
    {
        startR(0) = xytheta(0);
        startR(1) = xytheta(1);
        startR(2) = xytheta(2);
    }
    else
    {
        startL(0) = xytheta(0);
        startL(1) = xytheta(1);
        startL(2) = xytheta(2);
    }
}

void footMotionPlanner::setFootDestXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight)
{
    if (isRight)
    {
        planR(0) = xytheta(0);
        planR(1) = xytheta(1);
        planR(2) = xytheta(2);
    }
    else
    {
        planL(0) = xytheta(0);
        planL(1) = xytheta(1);
        planL(2) = xytheta(2);
    }
}

void footMotionPlanner::setFootXYTheta(KMath::KMat::GenMatrix<float, 3, 1> xytheta, bool isRight)
{
    if (isRight)
    {
        FootR(0) = xytheta(0);
        FootR(1) = xytheta(1);
        FootR(2) = xytheta(2);
    }
    else
    {
        FootL(0) = xytheta(0);
        FootL(1) = xytheta(1);
        FootL(2) = xytheta(2);
    }
}

void footMotionPlanner::setFootZ(float z, bool isRight)
{
    if (isRight)
        FootRz = z;
    else
        FootLz = z;
}

void footMotionPlanner::setFootStartZ(float z, bool isRight)
{
    if (isRight)
        startRz = z;
    else
        startLz = z;
}

void footMotionPlanner::setFootDestZ(float z, bool isRight)
{
    if (isRight)
        planRz = z;
    else
        planLz = z;
}

KVecFloat3 footMotionPlanner::getFootDestXYTheta(bool isRight)
{
    KVecFloat3 res;
    if (isRight)
    {
        res(0) = planR(0);
        res(1) = planR(1);
        res(2) = planR(2);
    }
    else
    {
        res(0) = planL(0);
        res(1) = planL(1);
        res(2) = planL(2);
    }

    return res;
}

float footMotionPlanner::getFootDestTheta(bool isRight)
{
    float res;
    if (isRight)
        res = planR(2);
    else
        res = planL(2);

    return res;
}

float footMotionPlanner::getFootDestZ(bool isRight)
{
    float res;
    if (isRight)
        res = planRz;
    else
        res = planLz;

    return res;
}

float footMotionPlanner::getFootTheta(bool isRight)
{
    float res;
    if (isRight)
        res = FootR(2);
    else
        res = FootL(2);

    return res;
}

float footMotionPlanner::getFootZ(bool isRight)
{
    float res;
    if (isRight)
        res = FootRz;
    else
        res = FootLz;

    return res;
}

KVecFloat3 footMotionPlanner::getFootXYTheta(bool isRight)
{
    KVecFloat3 res;
    if (isRight)
        res = FootR;
    else
        res = FootL;

    return res;
}

void footMotionPlanner::MotionPlan(KVecFloat3 target, unsigned step, unsigned totalsteps, bool right_support, bool double_support)
{

    //Generate Feet Trajectories while swing
    if (!right_support && !double_support)
    {

        /** Right Foot Interpolation **/
        float diff = KMath::anglediff2(target(2), startR(2));

        FootR(0) = interp.planFeetTrajectoryXY((float)step, target(0), startR(0), totalsteps - 1.0);

        FootR(1) = interp.planFeetTrajectoryXY((float)step, target(1), startR(1), totalsteps - 1.0);

        FootR(2) = startR(2) + interp.LinearInterpolation((float)step, diff, 0.0, totalsteps - 1.0);

      
        FootRz = interp.CubicSplineInterpolation((float)step, 0.000, StepZ_ / 2.0, 1.25 * StepZ_, StepZ_ / 3.0, 0.000, totalsteps - 1.0);
    
    }
    else if (right_support && !double_support)
    {

        /** Left Foot Interpolation **/
        float diff = anglediff2(target(2), startL(2));

        FootL(0) = interp.planFeetTrajectoryXY((float)step, target(0), startL(0), totalsteps - 1.0);

        FootL(1) = interp.planFeetTrajectoryXY((float)step, target(1), startL(1), totalsteps - 1.0);

        FootL(2) = startL(2) + interp.LinearInterpolation((float)step, diff, 0.0, totalsteps - 1.0);

        
        FootLz = interp.CubicSplineInterpolation((float)step, 0.000, StepZ_ / 2.0, StepZ_, StepZ_ / 3.0, 0.000, totalsteps - 1.0);
        // FootLz=interp.planFeetTrajectoryZ((float) step, robot.getWalkParameter(StepZ),0.000, totalsteps);
        // FootLz=interp.BezierZ((float) step, robot.getWalkParameter(StepZ), totalsteps-1.0);
    }
    else
    {
        FootR(0)=startR(0);
    }
}
