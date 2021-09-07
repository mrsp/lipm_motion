#include <lipm_motion/zmpPlanner.h>

zmpPlanner::zmpPlanner(int bsize) : ZMPbuffer(bsize), footRbuffer(bsize), footLbuffer(bsize)
{ 
    planAvailable = false;
    start.resize(3);
    start.setZero();
    target.resize(3);
    target.setZero();

    startL.resize(13);
    startL.setZero();
    targetL.resize(13);
    targetL.setZero();
    
    startR.resize(13);
    startR.setZero();
    targetR.resize(13);
    targetR.setZero();
    
    footR.resize(13);
    footR.setZero();
    footL.resize(13);
    footL.setZero();
    
    footR_.resize(13);
    footR_.setZero();
    footL_.resize(13);
    footL_.setZero();
    
    ZMPref.resize(3);
    ZMPref.setZero();

    v.resize(3);
    v.setZero();
    omega.resize(3);
    omega.setZero();
    
    planned.targetZMP = SUPPORT_LEG_NONE;
    planned.targetSupport = SUPPORT_LEG_NONE;
    planned.step_id = -1;
}

void zmpPlanner::setParams(double HX_, double HY_, int DS_Instructions_, double MaxStepX_, double MinStepX_, double MaxStepY_, double MinStepY_, double MaxStepZ_, double dt_)
{
    HX = HX_;
    HY = HY_;
    DS_Instructions = DS_Instructions_;
    MaxStepX = MaxStepX_;
    MinStepX = MinStepX_;
    MaxStepY = MaxStepY_;
    MinStepY = MinStepY_;
    MaxStepZ = MaxStepZ_;
    dt = dt_;
}

void zmpPlanner::setFeet(VectorXd sl, VectorXd sr)
{
    startL = sl;
    startR = sr;
    /** Double Support Phase **/
    //Convention is w,x,y,z
    Quaterniond ql(sl(3),sl(4),sl(5),sl(6)), qr(sr(3),sr(4),sr(5),sr(6));

    double yawl = ql.toRotationMatrix().eulerAngles(0, 1, 2)(2);
    double yawr = qr.toRotationMatrix().eulerAngles(0, 1, 2)(2);

    float meanangle = anglemean(yawl, yawr);
    /** planL,planR are the ankle positions, transforming them to Reference ZMP **/
    Rotation2D<double> rotR(yawr);

    Vector2d rr = rotR * Vector2d(-HX, -HY);
    start(0) = rr(0) + startR(0);
    start(1) = rr(1) + startR(1);

    Rotation2D<double> rotL(yawl);

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
            Quaterniond startqR = Quaterniond(startR(3), startR(4),  startR(5),  startR(6));
            Quaterniond targetqR = Quaterniond(targetR(3), targetR(4),  targetR(5),  targetR(6));
            
            
            // Check for Kinematic Bounds on steps
            Quaterniond startqL = Quaterniond(startL(3), startL(4),  startL(5),  startL(6));
            Matrix3d rotL = startqL.toRotationMatrix(); 
            
            // cout<<"Rot L "<<rotL<<endl;
            Vector3d Upper, Lower;
            Upper(0) = MaxStepX;
            Upper(1) = -MinStepY;
            Upper(2) = 0;
            Upper =  rotL * Upper + startL.head(2);
            
            Lower(0) = MinStepX;
            Lower(1) = -MaxStepY;
            Lower(2) = 0;
            Lower = rotL * Lower + startL.head(2);


            // cout<<"Target R before Crop "<<targetR.transpose()<<endl;
            targetR(0) = cropStep(targetR(0), Upper(0), Lower(0));
            targetR(1) = cropStep(targetR(1), Upper(1), Lower(1));
            // cout<<"Target R after Crop "<<targetR.transpose()<<endl;




            ZMPref = start;
            footL = startL;
            unsigned int p = 0;
            while (p < i.steps)
            {

                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                footR(0) = interp.planFeetTrajectoryXY((float)p, targetR(0), startR(0), i.steps - 1.0);
                footR(1) = interp.planFeetTrajectoryXY((float)p, targetR(1), startR(1), i.steps - 1.0);
                footR(2) = interp.CubicSplineInterpolation((float)p, startR(2), startR(2) + MaxStepZ / 2.0, startR(2) + MaxStepZ, targetR(2) + MaxStepZ / 3.0, targetR(2), i.steps - 1.0);
                
                qR = startqR.slerp( (float) p/(i.steps-1.0), targetqR);
                footR(3) = qR.w();
                footR(4) = qR.x();
                footR(5) = qR.y();
                footR(6) = qR.z();

                //Compute Desired Right Leg Velocity
                if(p==0)
                {
                    v = (footR.head(3) - startR.head(3))/dt;
                    omega = logMap( (startqR.inverse()*qR).toRotationMatrix() )/dt;
                }
                else
                {
                    v = (footR.head(3) - footR_.head(3))/dt;
                    omega = logMap( (qR_.inverse()*qR).toRotationMatrix() )/dt;
                }


                footR(7) = v(0);
                footR(8) = v(1);
                footR(9) = v(2);

                footR(10) = omega(0);
                footR(11) = omega(1);
                footR(12) = omega(2);

                footR_ = footR;
                qR_ = qR;

                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }

            i.steps = DS_Instructions;
            target.head(2) = targetR.head(2);
            target(2) = targetqR.toRotationMatrix().eulerAngles(0,1,2)(2);

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

            targetL = i.target;
            Quaterniond startqL = Quaterniond(startL(3), startL(4),  startL(5),  startL(6));
            Quaterniond targetqL = Quaterniond(targetL(3), targetL(4),  targetL(5),  targetL(6));


            // Check for Kinematic Bounds on steps
            Quaterniond startqR = Quaterniond(startR(3), startR(4),  startR(5),  startR(6));
            Matrix3d rotR = startqR.toRotationMatrix();
            // cout<<"Rot R "<<rotR<<endl;

            Vector3d Upper, Lower;
            Upper(0) = MaxStepX;
            Upper(1) = MaxStepY;
            Upper(2) = 0;
            Upper =  rotR * Upper + startR.head(3);
            
            Lower(0) = MinStepX;
            Lower(1) = MinStepY;
            Lower(2) = 0;
            Lower =  rotR * Lower + startR.head(3);


            // cout<<"Target L before Crop "<<targetL.transpose()<<endl;
            targetL(0) = cropStep(targetL(0), Upper(0), Lower(0));
            targetL(1) = cropStep(targetL(1), Upper(1), Lower(1));
            // cout<<"Target L after Crop "<<targetL.transpose()<<endl;






            ZMPref = start;
            footR = startR;

            unsigned int p = 0;
            while (p < i.steps)
            {
                /** ZMP Point pushed to ZMP buffer **/
                ZMPbuffer.push_back(ZMPref);
                footL(0) = interp.planFeetTrajectoryXY((float)p, targetL(0), startL(0), i.steps - 1.0);
                footL(1) = interp.planFeetTrajectoryXY((float)p, targetL(1), startL(1), i.steps - 1.0);
                footL(2) = interp.CubicSplineInterpolation((float)p, startL(2), startL(2) + MaxStepZ / 2.0, startL(2) + MaxStepZ, targetL(2) + MaxStepZ / 3.0, targetL(2), i.steps - 1.0);
            
                qL = startqL.slerp( (float) p/(i.steps-1.0), targetqL);
                footL(3) = qL.w();
                footL(4) = qL.x();
                footL(5) = qL.y();
                footL(6) = qL.z();

                //Compute Desired Right Leg Velocity
                if(p==0)
                {
                    v = (footL.head(3) - startL.head(3))/dt;
                    omega = logMap( (startqL.inverse()*qL).toRotationMatrix() )/dt;
                }
                else
                {
                    v = (footL.head(3) - footL_.head(3))/dt;
                    omega = logMap( (qL_.inverse()*qL).toRotationMatrix() )/dt;
                }


                footL(7) = v(0);
                footL(8) = v(1);
                footL(9) = v(2);

                footL(10) = omega(0);
                footL(11) = omega(1);
                footL(12) = omega(2);

                footL_ = footL;
                qL_ = qL;

                footRbuffer.push_back(footR);
                footLbuffer.push_back(footL);
                p++;
            }

            i.steps = DS_Instructions;
            target.head(2) = targetL.head(2);
            target(2) = targetqL.toRotationMatrix().eulerAngles(0,1,2)(2);


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
            Quaterniond startqL = Quaterniond(startL(3), startL(4),  startL(5),  startL(6));
            Quaterniond startqR = Quaterniond(startR(3), startR(4),  startR(5),  startR(6));
            unsigned int p = 0;
            target.head(2) = 0.5*(startR.head(2) + startL.head(2));
            target(2) = anglemean(startqL.toRotationMatrix().eulerAngles(0,1,2)(2), startqR.toRotationMatrix().eulerAngles(0,1,2)(2));
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
    //Convention is w,x,y,z
    Quaterniond ql(sl(3),sl(4),sl(5),sl(6)), qr(sr(3),sr(4),sr(5),sr(6));
    double yawl = ql.toRotationMatrix().eulerAngles(0, 1, 2)(2);
    double yawr = qr.toRotationMatrix().eulerAngles(0, 1, 2)(2);
    /** Computing the  Reference ZMP point (x,y,yaw) **/

    Rotation2D<double> rotL(yawl);
    Rotation2D<double> rotR(yawr);
    if (i.targetZMP == SUPPORT_LEG_RIGHT)
    {
        /** Right Support Phase **/
        Vector2d rr = rotR * Vector2d(-HX, HY);
        t(0) = rr(0) + sr(0); //x
        t(1) = rr(1) + sr(1); //y
        t(2) = yawr;         //theta
    }
    else if (i.targetZMP == SUPPORT_LEG_LEFT)
    {
        /** Left Support Phase **/
        Vector2d rr = rotL * Vector2d(-HX, -HY);
        t(0) = rr(0) + sl(0);
        t(1) = rr(1) + sl(1);
        t(2) = yawl;
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
        t(2) = anglemean(yawl, yawr);
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
