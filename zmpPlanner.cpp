#include <lipm_motion/zmpPlanner.h>

zmpPlanner::zmpPlanner(RobotParameters &robot_) : robot(robot_), ZMPbuffer(10 * (int)robot.getWalkParameter(PreviewWindow)), footRbuffer(10 * (int)robot.getWalkParameter(PreviewWindow)), footLbuffer(10 * (int)robot.getWalkParameter(PreviewWindow))
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

void zmpPlanner::setFeet(VectorXd sl, VectorXd sr)
{
    startL = sl;
    startR = sr;
    /** Double Support Phase **/
    //KMath::KMat::GenMatrix<float, 2, 2> rot;
    float meanangle = anglemean(startL(5), startR(5));
    /** planL,planR are the ankle positions, transforming them to Reference ZMP **/
    Rotation2D<double> rotR(startR(5));

    Vector2d rr = rotR * Vector2d(-robot.getWalkParameter(HX), -robot.getWalkParameter(HY));
    start(0) = rr(0) + startR(0);
    start(1) = rr(1) + startR(1);

    Rotation2D<double> rotL(startL(5));

    rr = rotL * Vector2d(-robot.getWalkParameter(HX), robot.getWalkParameter(HY));
    start(0) = rr(0) + startL(0);
    start(1) = rr(1) + startL(1);
    start *= 0.5;
    start(2) = meanangle;
}

void zmpPlanner::generatePlan()
{
    if (stepAnkleQ.size() == 0)
    {
        std::cout << "Steps needed for motion planning" << std::endl;
        return;
    }
    bool add_DS_instruction = false;

    while (stepAnkleQ.size() > 0)
    {
        /** Robot is Ready to begin the walking Cycle **/
        WalkInstruction i = stepAnkleQ.front();
        /** Add a double support phase in the i instruction **/
        if (!add_DS_instruction)
        {
            std::cout << "Planning DS" << std::endl;
            cout<<"STARTL STARTR "<<startL<<" "<<startR<<endl;

            i.steps = robot.getWalkParameter(DS_instructions);
            i.phase = double_support;
            target = computeDesiredZMP(startL, startR, i);
            footR = startR;
            footL = startL;
            unsigned int p = 0;
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

            add_DS_instruction = true;
            start = target;
            zmpQ.push(target);
        }
        else
        {
            std::cout << "Planning SS" << std::endl;

            if (i.targetZMP == SUPPORT_LEG_BOTH)
                i.phase = double_support;
            else
                i.phase = single_support;

            if (i.targetSupport == SUPPORT_LEG_LEFT)
            {
                targetR = i.target;
                // //Check for Kinematic Bounds on steps
                Rotation2D<double> rotL(startL(5));

                dx = Vector2d(targetR(0) - startL(0), targetR(1) - startL(1));
                tempV = rotL.inverse() * dx;
                tempV(0) = cropStep(tempV(0), robot.getWalkParameter(MaxStepX), robot.getWalkParameter(MinStepX));
                tempV(1) = cropStep(tempV(1), -robot.getWalkParameter(MinStepY), -robot.getWalkParameter(MaxStepY));
                tempV = rotL * tempV;
                targetR(0) = startL(0) + tempV(0);
                targetR(1) = startL(1) + tempV(1);
                ZMPref = target;
                footL = startL;
                unsigned int p = 0;
                while (p < i.steps)
                {

                    /** ZMP Point pushed to ZMP buffer **/
                    ZMPbuffer.push_back(ZMPref);
                    float adiff = anglediff2(targetR(2), startR(2));
                    footR(0) = interp.planFeetTrajectoryXY((float)p, targetR(0), startR(0), i.steps - 1.0);
                    footR(1) = interp.planFeetTrajectoryXY((float)p, targetR(1), startR(1), i.steps - 1.0);
                    footR(2) = interp.CubicSplineInterpolation((float)p, 0.000, robot.getWalkParameter(StepZ) / 2.0, 1.25 * robot.getWalkParameter(StepZ), robot.getWalkParameter(StepZ) / 3.0, 0.000, i.steps - 1.0);
                    footR(3) = 0.0;
                    footR(4) = 0.0;
                    footR(5) = startR(2) + interp.LinearInterpolation((float)p, adiff, 0.0, i.steps - 1.0);

                    footRbuffer.push_back(footR);
                    footLbuffer.push_back(footL);
                    p++;
                }
                cout<<"Target R Foot"<<endl;
                cout<<targetR<<endl;
                startR= targetR;  
                stepRQ.push(targetR);
            }
            else if (i.targetSupport == SUPPORT_LEG_RIGHT)
            {
                // Check for Kinematic Bounds on steps
                targetL = i.target;
                Rotation2D<double> rotR(startR(5));
                dx = Vector2d(targetL(0) - startR(0), targetR(1) - startR(1));
                tempV = rotR.inverse() * dx;

                tempV(0) = cropStep(tempV(0), robot.getWalkParameter(MaxStepX), robot.getWalkParameter(MinStepX));
                tempV(1) = cropStep(tempV(1),robot.getWalkParameter(MaxStepY),robot.getWalkParameter(MinStepY));

                tempV = rotR * tempV;
                targetL(0) = startR(0) + tempV(0);
                targetL(1) = startR(1) + tempV(1);
                ZMPref = target;
                footR = startR;
                unsigned int p = 0;
                while (p < i.steps)
                {
                    /** ZMP Point pushed to ZMP buffer **/
                    ZMPbuffer.push_back(ZMPref);
                    float adiff = anglediff2(targetL(2), startL(2));
                    footL(0) = interp.planFeetTrajectoryXY((float)p, targetL(0), startL(0), i.steps - 1.0);
                    footL(1) = interp.planFeetTrajectoryXY((float)p, targetL(1), startL(1), i.steps - 1.0);
                    footL(2) = interp.CubicSplineInterpolation((float)p, 0.000, robot.getWalkParameter(StepZ) / 2.0, 1.25 * robot.getWalkParameter(StepZ), robot.getWalkParameter(StepZ) / 3.0, 0.000, i.steps - 1.0);
                    footL(3) = 0.0;
                    footL(4) = 0.0;
                    footL(5) = startL(2) + interp.LinearInterpolation((float)p, adiff, 0.0, i.steps - 1.0);
                    footRbuffer.push_back(footR);
                    footLbuffer.push_back(footL);
                    p++;
                }
                cout<<"Target LFOOT"<<endl;
                cout<<targetL<<endl;
                startL = targetL;

                stepLQ.push(targetL);
            }
           
                /**  Pop walking command i **/
            stepAnkleQ.pop();
            planned = i;
            add_DS_instruction = false;
        }

    }
     planAvailable = true;

}

void zmpPlanner::plan()
{
 
    generatePlan();
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
        Vector2d rr = rotR * Vector2d(-robot.getWalkParameter(HX), robot.getWalkParameter(HY));
        t(0) = rr(0) + sr(0); //x
        t(1) = rr(1) + sr(1); //y
        t(2) = sr(5);         //theta
    }
    else if (i.targetZMP == SUPPORT_LEG_LEFT)
    {
        /** Left Support Phase **/

        Vector2d rr = rotL * Vector2d(-robot.getWalkParameter(HX), -robot.getWalkParameter(HY));
        t(0) = rr(0) + sl(0);
        t(1) = rr(1) + sl(1);
        t(2) = sl(5);
    }
    else
    {
        /** Double Support Phase **/
        Vector2d rr = rotL * Vector2d(-robot.getWalkParameter(HX), -robot.getWalkParameter(HY));
        t(0) = rr(0) + sl(0);
        t(1) = rr(1) + sl(1);
        rr = rotR * Vector2d(-robot.getWalkParameter(HX), robot.getWalkParameter(HY));
        t(0) += rr(0) + sr(0);
        t(1) += rr(1) + sr(1);
        t *= 0.5;
        t(2) = anglemean(sl(5), sr(5));
        
    }
    return t;
}

void zmpPlanner::emptyPlan()
{
    if(!planAvailable)
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
