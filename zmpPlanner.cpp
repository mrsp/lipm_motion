#include <lipm_motion/zmpPlanner.h>

zmpPlanner::zmpPlanner(int bsize) : ZMPbuffer(bsize), footRbuffer(bsize), footLbuffer(bsize)
{ //sa(robot){
    planAvailable = false;
    start.resize(3);
    start.setZero();
    target.resize(3);
    target.setZero();
    startL.resize(6);
    startL.setZero();
    targetL.resize(6);
    targetL.setZero();
    startR.resize(6);
    startR.setZero();
    targetR.resize(6);
    targetR.setZero();
    footR.resize(6);
    footR.setZero();
    footL.resize(6);
    footL.setZero();
    ZMPref.resize(3);
    ZMPref.setZero();
    planned.targetZMP = SUPPORT_LEG_NONE;
    planned.targetSupport = SUPPORT_LEG_NONE;
    planned.step_id = -1;
}

void zmpPlanner::setParams(double HX_, double HY_, int DS_Instructions_, double MaxStepX_, double MinStepX_, double MaxStepY_, double MinStepY_, double MaxStepZ_)
{
    HX = HX_;
    HY = HY_;
    DS_Instructions = DS_Instructions_;
    MaxStepX = MaxStepX_;
    MinStepX = MinStepX_;
    MaxStepY = MaxStepY_;
    MinStepY = MinStepY_;
    MaxStepZ = MaxStepZ_;
}

void zmpPlanner::setFeet(VectorXd sl, VectorXd sr)
{
    startL = sl;
    startR = sr;
    /** Double Support Phase **/
    float meanangle = anglemean(startL(5), startR(5));
    /** planL,planR are the ankle positions, transforming them to Reference ZMP **/
    Rotation2D<double> rotR(startR(5));

    Vector2d rr = rotR * Vector2d(-HX, -HY);
    start(0) = rr(0) + startR(0);
    start(1) = rr(1) + startR(1);

    Rotation2D<double> rotL(startL(5));

    rr = rotL * Vector2d(-HX, HY);
    start(0) = rr(0) + startL(0);
    start(1) = rr(1) + startL(1);
    start *= 0.5;
    start(2) = meanangle;
}

void zmpPlanner::plan(Vector2d actual_COP, VectorXd actual_footL, VectorXd actual_footR)
{
    if (stepAnkleQ.size() == 0)
    {
        std::cout << "Steps needed for motion planning" << std::endl;
        return;
    }
    bool add_initial_Transition = true;
    start.head(2) = actual_COP;
    startR = actual_footR;
    startL = actual_footL;

    while (stepAnkleQ.size() > 0)
    {
        /** Robot is Ready to begin the walking Cycle **/
        WalkInstruction i =   stepAnkleQ.front();
         stepAnkleQ.pop();
        /** Add an initial ZMP transition **/
        if (add_initial_Transition)
        {
            target = computeDesiredZMP(startL, startR, i);
            footR = startR;
            footL = startL;
            unsigned int p = 0;

            while (p < 3*DS_Instructions)
            {

                /** Angle between the ending and starting foot orientation **/
                float adiff =  anglediff2(target(2), start(2));

                /** ZMP Trajectory Generation **/
                ZMPref(0) = interp.planFeetTrajectoryXY((float)p, target(0), start(0), 3*DS_Instructions - 1.0);
                ZMPref(1) = interp.planFeetTrajectoryXY((float)p, target(1), start(1), 3*DS_Instructions - 1.0);
                ZMPref(2) = start(2) + interp.LinearInterpolation((float)p, adiff, 0.000, 3*DS_Instructions - 1.0);

                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }

            add_initial_Transition = false;
            start = target;
        }

        if (i.targetSupport == SUPPORT_LEG_LEFT && i.targetZMP != SUPPORT_LEG_BOTH)
        {

            targetR = i.target;
            // // //Check for Kinematic Bounds on steps
            // Rotation2D<double> rotL(startL(5));

            // dx = Vector2d(targetR(0) - startL(0), targetR(1) - startL(1));
            // tempV = rotL.inverse() * dx;
            // tempV(0) = cropStep(tempV(0), MaxStepX, MinStepX);
            // tempV(1) = cropStep(tempV(1), -MinStepY, -MaxStepY);
            // tempV = rotL * tempV;
            // targetR(0) = startL(0) + tempV(0);
            // targetR(1) = startL(1) + tempV(1);
            ZMPref = start;
            footL = startL;
            unsigned int p = 0;
            while (p < i.steps)
            {

                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                float adiff = anglediff2(targetR(5), startR(5));
                footR(0) = interp.planFeetTrajectoryXY((float)p, targetR(0), startR(0), i.steps - 1.0);
                footR(1) = interp.planFeetTrajectoryXY((float)p, targetR(1), startR(1), i.steps - 1.0);
                footR(2) = interp.CubicSplineInterpolation((float)p, startR(2), startR(2) + MaxStepZ / 2.0, startR(2) + MaxStepZ, targetR(2) + MaxStepZ / 3.0, targetR(2), i.steps - 1.0);
                footR(3) = 0.0;
                footR(4) = 0.0;
                footR(5) = startR(5) + interp.LinearInterpolation((float)p, adiff, 0.0, i.steps - 1.0);

                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }

            i.steps = DS_Instructions;
            target.head(2) = targetR.head(2);
            target(2) = targetR(5);

            footR = targetR;
            footL = startL;
            p = 0;
            while (p < i.steps)
            {
                /** Angle between the ending and starting foot orientation **/
                float adiff = anglediff2(target(2), start(2));
                /** ZMP Trajectory Generation **/
                ZMPref(0) = interp.planFeetTrajectoryXY((float)p, target(0), start(0), i.steps - 1.0);
                ZMPref(1) = interp.planFeetTrajectoryXY((float)p, target(1), start(1), i.steps - 1.0);
                ZMPref(2) = start(2) + interp.LinearInterpolation((float)p, adiff, 0.000, i.steps - 1.0);
                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }

            start = target;
            startR = targetR;
        }
        else if (i.targetSupport == SUPPORT_LEG_RIGHT && i.targetZMP != SUPPORT_LEG_BOTH)
        {

            // Check for Kinematic Bounds on steps
            targetL = i.target;
            // Rotation2D<double> rotR(startR(5));
            // dx = Vector2d(targetL(0) - startR(0), targetR(1) - startR(1));
            // tempV = rotR.inverse() * dx;

            // tempV(0) = cropStep(tempV(0), MaxStepX, MinStepX);
            // tempV(1) = cropStep(tempV(1), MaxStepY, MinStepY);

            // tempV = rotR * tempV;
            // targetL(0) = startR(0) + tempV(0);
            // targetL(1) = startR(1) + tempV(1);
            ZMPref = start;
            footR = startR;

            unsigned int p = 0;
            while (p < i.steps)
            {
                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                float adiff = anglediff2(targetL(5), startL(5));
                footL(0) = interp.planFeetTrajectoryXY((float)p, targetL(0), startL(0), i.steps - 1.0);
                footL(1) = interp.planFeetTrajectoryXY((float)p, targetL(1), startL(1), i.steps - 1.0);
                footL(2) = interp.CubicSplineInterpolation((float)p, startL(2), startL(2) + MaxStepZ / 2.0, startL(2) + MaxStepZ, targetL(2) + MaxStepZ / 3.0, targetL(2), i.steps - 1.0);
                footL(3) = 0.0;
                footL(4) = 0.0;
                footL(5) = startL(5) + interp.LinearInterpolation((float)p, adiff, 0.0, i.steps - 1.0);
                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }

            i.steps = DS_Instructions;
            target.head(2) = targetL.head(2);
            target(2) = targetL(5);


            footR = startR;
            footL = targetL;
            p = 0;
            while (p < i.steps)
            {
                /** Angle between the ending and starting foot orientation **/
                float adiff = anglediff2(target(2), start(2));
                /** ZMP Trajectory Generation **/
                ZMPref(0) = interp.planFeetTrajectoryXY((float)p, target(0), start(0), i.steps - 1.0);
                ZMPref(1) = interp.planFeetTrajectoryXY((float)p, target(1), start(1), i.steps - 1.0);
                ZMPref(2) = start(2) + interp.LinearInterpolation((float)p, adiff, 0.000, i.steps - 1.0);
                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }

            start = target;
            startL = targetL;
        }
        else
        {
            footR = startR;
            footL = startL;
            unsigned int p = 0;
            target.head(2) = 0.5*(startR.head(2) + startL.head(2));
            target(2) = anglemean(startL(5), startR(5));
            while (p < 2.0*DS_Instructions)
            {
                /** Angle between the ending and starting foot orientation **/
                float adiff = anglediff2(target(2), start(2));
                /** ZMP Trajectory Generation **/
                ZMPref(0) = interp.planFeetTrajectoryXY((float)p, target(0), start(0),  2.0*DS_Instructions - 1.0);
                ZMPref(1) = interp.planFeetTrajectoryXY((float)p, target(1), start(1),  2.0*DS_Instructions - 1.0);
                ZMPref(2) = start(2) + interp.LinearInterpolation((float)p, adiff, 0.000,  2.0*DS_Instructions - 1.0);
                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }
            p = 0;
            while (p < 2.0*i.steps)
            {
                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(target);
                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }
            start = target;
        }


    }
    planAvailable = true;
}

VectorXd zmpPlanner::computeDesiredZMP(VectorXd sl, VectorXd sr, WalkInstruction i)
{
    VectorXd t;
    t.resize(3);
    t.setZero();

    /** Computing the  Reference ZMP point **/

    Rotation2D<double> rotL(sl(5));
    Rotation2D<double> rotR(sr(5));
    if (i.targetZMP == SUPPORT_LEG_RIGHT)
    {
        /** Right Support Phase **/
        Vector2d rr = rotR * Vector2d(-HX, HY);
        t(0) = rr(0) + sr(0); //x
        t(1) = rr(1) + sr(1); //y
        t(2) = sr(5);         //theta
    }
    else if (i.targetZMP == SUPPORT_LEG_LEFT)
    {
        /** Left Support Phase **/
        Vector2d rr = rotL * Vector2d(-HX, -HY);
        t(0) = rr(0) + sl(0);
        t(1) = rr(1) + sl(1);
        t(2) = sl(5);
    }
    else
    {
        /** Double Support Phase **/
        Vector2d rr = rotL * Vector2d(-HX, -HY);
        t(0) = rr(0) + sl(0);
        t(1) = rr(1) + sl(1);
        rr = rotR * Vector2d(-HX, HY);
        t(0) += rr(0) + sr(0);
        t(1) += rr(1) + sr(1);
        t *= 0.5;
        t(2) = anglemean(sl(5), sr(5));
    }
    return t;
}

void zmpPlanner::emptyPlan()
{
    if (!planAvailable)
        return;
    while (stepAnkleQ.size() > 0)
        stepAnkleQ.pop();

    while (stepLQ.size() > 0)
        stepLQ.pop();

    while (stepRQ.size() > 0)
        stepRQ.pop();

    while (zmpQ.size() > 0)
        zmpQ.pop();

    while (ZMPbuffer.size() > 0)
        ZMPbuffer.pop_front();

    while (footRbuffer.size() > 0)
        footRbuffer.pop_front();

    while (footLbuffer.size() > 0)
        footLbuffer.pop_front();

    planned.targetZMP = SUPPORT_LEG_NONE;
    planned.targetSupport = SUPPORT_LEG_NONE;
    planAvailable = false;
    planned.step_id = -1;

    start.setZero();
    target.setZero();
    startL.setZero();
    targetL.setZero();
    startR.setZero();
    targetR.setZero();
    footR.setZero();
    footL.setZero();
    ZMPref.setZero();
}
