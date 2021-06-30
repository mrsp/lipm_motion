#include "lipm_motion/lipm.h"

lipm::lipm()
{
    isPlanAvailable = false;
    zp = new zmpPlanner(15000);
    dp = new LIPMPlanner(15000);

    g = 9.80665;
    dt = 0.01;


     //h = 1.14398;
    // Tss = 3.0;
    // Tds = 1.0;
    // MaxStepX = 0.4;
    // MinStepX = -0.2;
    // MinStepY = 0.4;
    // MinStepY = 0.2;
    //MaxStepZ = 0.1;

    h = 0.26818;

    MaxStepX = 0.2;
    MinStepX = -0.1;
    MinStepY = 0.3;
    MinStepY = 0.2;
    MaxStepZ = 0.02;
    Tss = 0.5;
    Tds = 0.15;
    HX = -0.01;
    HY = 0;

    SS_Instructions = ceil(Tss / dt);
    DS_Instructions = ceil(Tds / dt);
    zp->setParams(HX, HY, DS_Instructions, MaxStepX, MinStepX, MaxStepY, MinStepY, MaxStepZ);
    dp->setParams(h, g, dt);
    dp->init();
    std::cout << "LIPM Motion Planning Initialized" << std::endl;
}

void lipm::desiredFootstepsCb(MotionPlanTarget *goal, boost::circular_buffer<VectorXd> &ZMPdBuffer, boost::circular_buffer<VectorXd> &DCMBuffer, boost::circular_buffer<VectorXd> &CoMBuffer,
                              boost::circular_buffer<VectorXd> &ZMPBuffer, boost::circular_buffer<VectorXd> &footLBuffer, boost::circular_buffer<VectorXd> &footRBuffer)
{
    std::cout << "Motion Planning" << std::endl;
    zp->emptyPlan();
    dp->emptyPlan();

    VectorXd rfoot;
    rfoot.resize(6);
    rfoot.setZero();
    VectorXd lfoot;
    lfoot.resize(6);
    lfoot.setZero();

    //Initial Foot Poses
    lfoot.head(3) = goal->lfoot_position;
    lfoot.tail(3) = goal->lfoot_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    rfoot.head(3) = goal->rfoot_position;
    rfoot.tail(3) = goal->rfoot_orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    WalkInstruction i;
    i.target.resize(6);
    i.steps = SS_Instructions;

    unsigned int j = 0;

    while (j < goal->footsteps.size())
    {
        i.target.head(3) = goal->footsteps[j].position;
        i.target.tail(3) = goal->footsteps[j].orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        if (goal->footsteps[j].leg == 0) //Swing LLeg
        {
            i.targetSupport = SUPPORT_LEG_RIGHT;
            i.targetZMP = SUPPORT_LEG_RIGHT;
            zp->stepAnkleQ.push(i);
            if (j == goal->footsteps.size() -1)
            {
                i.targetZMP = SUPPORT_LEG_BOTH;
                zp->stepAnkleQ.push(i);
            }

        }
        else if (goal->footsteps[j].leg == 1) //Swing RLeg
        {
            i.targetSupport = SUPPORT_LEG_LEFT;
            i.targetZMP = SUPPORT_LEG_LEFT;
            zp->stepAnkleQ.push(i);
            if (j == goal->footsteps.size() -1)
            {
                i.targetZMP = SUPPORT_LEG_BOTH;
                zp->stepAnkleQ.push(i);
            }
       

        }
        else
        {
            cout<<"Invalid Step"<<endl;
        }
        j++;


    }

    zp->plan(goal->COP.head(2), lfoot, rfoot);
    ZMPBuffer = zp->ZMPbuffer;
    j = 0;

    Vector3d DCM;
    DCM = goal->CoM_position + 1 / sqrt(g / h) * goal->CoM_velocity;
    dp->setState(goal->CoM_position.head(2), goal->CoM_velocity.head(2), goal->COP.head(2));
    //dp->setState(DCM.head(2), goal->CoM_position.head(2), goal->COP.head(2));

    dp->plan(zp->ZMPbuffer);
    DCMBuffer = dp->DCMBuffer;
    CoMBuffer = dp->CoMBuffer;
    ZMPdBuffer = dp->VRPBuffer;
    //ZMPdBuffer = dp->VRPBuffer;

    footLBuffer = zp->footLbuffer;
    footRBuffer = zp->footRbuffer;

    isPlanAvailable = true;
    std::cout << "Motion Plan Completed" << std::endl;
}

lipm::~lipm()
{
    delete zp;
    delete dp;
}