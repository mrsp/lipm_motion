#include "lipm_motion/lipm.h"

lipm::lipm(ros::NodeHandle nh_)
{
    nh = nh_;
    ros::NodeHandle n_p("~");
    isPlanAvailable = false;

    zp = new zmpPlanner(5000);
    dp = new dcmPlanner(5000);
    /*
    CoM_pub = nh.advertise<nav_msgs::Path>("lipm_motion/CoM", 1000);
    DCM_pub = nh.advertise<nav_msgs::Path>("lipm_motion/DCM", 1000);
    VRP_pub = nh.advertise<nav_msgs::Path>("lipm_motion/VRP", 1000);
    footL_pub = nh.advertise<nav_msgs::Path>("lipm_motion/LLeg", 1000);
    footR_pub = nh.advertise<nav_msgs::Path>("lipm_motion/RLeg", 1000);
    */
    CoM_pub = nh.advertise<lipm_msgs::TrajectoryPoints>("lipm_motion/CoM", 1000);
    DCM_pub = nh.advertise<lipm_msgs::TrajectoryPoints>("lipm_motion/DCM", 1000);
    VRP_pub = nh.advertise<lipm_msgs::TrajectoryPoints>("lipm_motion/VRP", 1000);
    footL_pub = nh.advertise<lipm_msgs::TrajectoryPoints>("lipm_motion/LLeg", 1000);
    footR_pub = nh.advertise<lipm_msgs::TrajectoryPoints>("lipm_motion/RLeg", 1000);

    double dt;
    n_p.param<double>("gravity", g, 9.80665);
    n_p.param<double>("comZ", comZ, 0.858);
    n_p.param<double>("dt", dt, 0.02);
    double MaxStepZ, MaxStepX, MaxStepY, MinStepX, MinStepY, Tss, Tds, HX,HY;
    n_p.param<double>("MaxStepZ", MaxStepZ, 0.03);
    n_p.param<double>("MaxStepX", MaxStepX, 0.05);
    n_p.param<double>("MaxStepY", MaxStepY, 0.11);
    n_p.param<double>("MinStepX", MinStepX, -0.02);
    n_p.param<double>("MinStepY", MinStepY, 0.10);
    n_p.param<double>("Tss", Tss, 3.0);
    n_p.param<double>("Tds", Tds, 1.0);
    n_p.param<double>("HX", HX, 0.0);
    n_p.param<double>("HY", HY, 0.0);

    SS_Instructions = ceil(Tss/dt);
    DS_Instructions = ceil(Tds/dt);
    zp->setParams( HX,  HY,  DS_Instructions,  MaxStepX,  MinStepX,  MaxStepY,  MinStepY, MaxStepZ);
    dp->setParams(comZ,g,dt);
    dp->init();
    as_ = new actionlib::SimpleActionServer<lipm_msgs::MotionPlanAction>(nh, "lipm_motion/plan", boost::bind(&lipm::desiredFootstepsCb, this, _1), false);
    as_->start();

    ac_ = new actionlib::SimpleActionClient<lipm_msgs::MotionControlAction>("lipm_control/plan", true);
    ac_->waitForServer();
    std::cout<<"LIPM Motion Planning Initialized"<<std::endl;

}



void lipm::desiredFootstepsCb(const lipm_msgs::MotionPlanGoalConstPtr &goal)
{
    std::cout<<"Motion Planning"<<std::endl;
    zp->emptyPlan();
    dp->emptyPlan();

    Vector3d lpos;
    lpos << goal->lfoot.position.x, goal->lfoot.position.y, goal->lfoot.position.z;
    Quaterniond lq(goal->lfoot.orientation.w, goal->lfoot.orientation.x, goal->lfoot.orientation.y, goal->lfoot.orientation.z);

    Vector3d rpos;
    rpos << goal->rfoot.position.x, goal->rfoot.position.y, goal->rfoot.position.z;
    Quaterniond rq(goal->rfoot.orientation.w, goal->rfoot.orientation.x, goal->rfoot.orientation.y, goal->rfoot.orientation.z);

    VectorXd rfoot;
    rfoot.resize(6);
    rfoot.setZero();
    VectorXd lfoot;
    lfoot.resize(6);
    lfoot.setZero();

    //Initial Foot Poses
    lfoot.head(3) = lpos;
    lfoot.tail(3) = lq.toRotationMatrix().eulerAngles(0, 1, 2);
    rfoot.head(3) = rpos;
    rfoot.tail(3) = rq.toRotationMatrix().eulerAngles(0, 1, 2);
    zp->setFeet(lfoot, rfoot);
    unsigned int j = 0;
    WalkInstruction i;
    i.target.resize(6);
    i.steps = SS_Instructions;
    feedback_.percent_completed = 0;
    result_.status = 0;

    while (j < goal->footsteps.size())
    {
        if (goal->footsteps[j].leg == 0)
        {
            lpos(0) = goal->footsteps[j].pose.position.x;
            lpos(1) = goal->footsteps[j].pose.position.y;
            lpos(2) = goal->footsteps[j].pose.position.z;
            lq.w() = goal->footsteps[j].pose.orientation.w;
            lq.x() = goal->footsteps[j].pose.orientation.x;
            lq.y() = goal->footsteps[j].pose.orientation.y;
            lq.z() = goal->footsteps[j].pose.orientation.z;
            i.target.head(3) = lpos;
            i.target.tail(3) = lq.toRotationMatrix().eulerAngles(0, 1, 2);
            i.targetSupport = SUPPORT_LEG_RIGHT;
            if (j == goal->footsteps.size() - 1)
            {
                i.targetZMP = SUPPORT_LEG_BOTH;
            }
            else
            {
                i.targetZMP = SUPPORT_LEG_RIGHT;
            }
        }
        else
        {
            rpos(0) = goal->footsteps[j].pose.position.x;
            rpos(1) = goal->footsteps[j].pose.position.y;
            rpos(2) = goal->footsteps[j].pose.position.z;
            rq.w() = goal->footsteps[j].pose.orientation.w;
            rq.x() = goal->footsteps[j].pose.orientation.x;
            rq.y() = goal->footsteps[j].pose.orientation.y;
            rq.z() = goal->footsteps[j].pose.orientation.z;

            i.target.head(3) = rpos;
            i.target.tail(3) = rq.toRotationMatrix().eulerAngles(0, 1, 2);
            i.targetSupport = SUPPORT_LEG_LEFT;
            if (j == goal->footsteps.size() - 1)
            {
                i.targetZMP = SUPPORT_LEG_BOTH;
            }
            else
            {
                i.targetZMP = SUPPORT_LEG_LEFT;
            }
        }
        i.step_id = j;
        zp->stepAnkleQ.push(i);
        j++;
        feedback_.percent_completed = j/goal->footsteps.size();
        as_->publishFeedback(feedback_);
    }
    
    zp->plan();

    Vector2d DCM, CoM, VRP;
    DCM.setZero();
    CoM.setZero();
    VRP.setZero();
    CoM(0) = goal->CoM.pose.pose.position.x;
    CoM(1) = goal->CoM.pose.pose.position.y;
    VRP(0) = goal->COP.x;
    VRP(1) = goal->COP.y;
    DCM(0) = goal->CoM.pose.pose.position.x + 1/sqrt(g/comZ) * goal->CoM.twist.twist.linear.x;
    DCM(1) = goal->CoM.pose.pose.position.y + 1/sqrt(g/comZ) * goal->CoM.twist.twist.linear.y;

    dp->setState( DCM,  CoM,  VRP);

    dp->plan(zp->ZMPbuffer);




    /*
    nav_msgs::Path CoM_path, footL_path, footR_path, DCM_path, VRP_path;
    CoM_path.poses.resize(dp->CoMBuffer.size());
    CoM_path.header.stamp = ros::Time::now();
    CoM_path.header.frame_id = "odom";

    footL_path.poses.resize(zp->footLbuffer.size());
    footL_path.header.stamp = ros::Time::now();
    footL_path.header.frame_id = "odom";
    footR_path.poses.resize(zp->footRbuffer.size());
    footR_path.header.stamp = ros::Time::now();
    footR_path.header.frame_id = "odom";

    VRP_path.poses.resize(dp->VRPBuffer.size());
    VRP_path.header.stamp = ros::Time::now();
    VRP_path.header.frame_id = "odom";
    DCM_path.poses.resize(dp->DCMBuffer.size());
    DCM_path.header.stamp = ros::Time::now();
    DCM_path.header.frame_id = "odom";
    */
    lipm_msgs::TrajectoryPoints CoM_msg, VRP_msg, DCM_msg, footL_msg, footR_msg;
    CoM_msg.positions.resize(dp->CoMBuffer.size());
    CoM_msg.velocities.resize(dp->CoMBuffer.size());
    CoM_msg.accelerations.resize(dp->CoMBuffer.size());
    CoM_msg.header.frame_id = "odom";
    VRP_msg.positions.resize(dp->VRPBuffer.size());
    VRP_msg.header.frame_id = "odom";

    DCM_msg.positions.resize(dp->DCMBuffer.size());
    DCM_msg.velocities.resize(dp->DCMBuffer.size());
    DCM_msg.header.frame_id = "odom";

    footL_msg.positions.resize(zp->footLbuffer.size());
    footL_msg.header.stamp = ros::Time::now();
    footL_msg.header.frame_id = "odom";

    footR_msg.positions.resize(zp->footRbuffer.size());
    footR_msg.header.stamp = ros::Time::now();
    footR_msg.header.frame_id = "odom";

    j = 0;
    while (j < dp->CoMBuffer.size())
    {

        /*
        //Msg for rviz
        CoM_path.poses[j].pose.position.x = dp->CoMBuffer[j](0);
        CoM_path.poses[j].pose.position.y = dp->CoMBuffer[j](1);
        CoM_path.poses[j].pose.position.z = dp->CoMBuffer[j](2);
        VRP_path.poses[j].pose.position.x = dp->VRPBuffer[j](0);
        VRP_path.poses[j].pose.position.y = dp->VRPBuffer[j](1);
        VRP_path.poses[j].pose.position.z = dp->VRPBuffer[j](2);
        DCM_path.poses[j].pose.position.x = dp->DCMBuffer[j](0);
        DCM_path.poses[j].pose.position.y = dp->DCMBuffer[j](1);
        DCM_path.poses[j].pose.position.z = dp->DCMBuffer[j](2);
        footL_path.poses[j].pose.position.x = zp->footLbuffer[j](0);
        footL_path.poses[j].pose.position.y = zp->footLbuffer[j](1);
        footL_path.poses[j].pose.position.z = zp->footLbuffer[j](2);
        footR_path.poses[j].pose.position.x = zp->footRbuffer[j](0);
        footR_path.poses[j].pose.position.y = zp->footRbuffer[j](1);
        footR_path.poses[j].pose.position.z = zp->footRbuffer[j](2);
        */
        ///Msgs for Control Loop
        ///CoM Position/Velocity/Acceleration
        CoM_msg.positions[j].x = dp->CoMBuffer[j](0);
        CoM_msg.positions[j].y = dp->CoMBuffer[j](1);
        CoM_msg.positions[j].z = dp->CoMBuffer[j](2);
        CoM_msg.velocities[j].x = dp->CoMBuffer[j](3);
        CoM_msg.velocities[j].y = dp->CoMBuffer[j](4);
        CoM_msg.velocities[j].z = dp->CoMBuffer[j](5);
        CoM_msg.accelerations[j].x = dp->CoMBuffer[j](6);
        CoM_msg.accelerations[j].y = dp->CoMBuffer[j](7);
        CoM_msg.accelerations[j].z = dp->CoMBuffer[j](8);

        ///VRP Position
        VRP_msg.positions[j].x = dp->VRPBuffer[j](0);
        VRP_msg.positions[j].y = dp->VRPBuffer[j](1);
        VRP_msg.positions[j].z = dp->VRPBuffer[j](2);

        ///DCM Position/Velocity
        DCM_msg.positions[j].x = dp->DCMBuffer[j](0);
        DCM_msg.positions[j].y = dp->DCMBuffer[j](1);
        DCM_msg.positions[j].z = dp->DCMBuffer[j](2);
        DCM_msg.velocities[j].x = dp->DCMBuffer[j](3);
        DCM_msg.velocities[j].y = dp->DCMBuffer[j](4);
        DCM_msg.velocities[j].z = dp->DCMBuffer[j](5);

        ///Left Foot Position
        footL_msg.positions[j].x = zp->footLbuffer[j](0);
        footL_msg.positions[j].y = zp->footLbuffer[j](1);
        footL_msg.positions[j].z = zp->footLbuffer[j](2);
        //q = AngleAxisd(zp->footLbuffer[j](3), Vector3d::UnitX())* AngleAxisd(zp->footLbuffer[j](4), Vector3d::UnitY())* AngleAxisd(zp->footLbuffer[j](5), Vector3d::UnitZ());
        // footL_msg.quaternions[j].x = q.x();
        // footL_msg.quaternions[j].y = q.y();
        // footL_msg.quaternions[j].z = q.z();
        // footL_msg.quaternions[j].w = q.w();
        ///Right Foot Position
        footR_msg.positions[j].x = zp->footRbuffer[j](0);
        footR_msg.positions[j].y = zp->footRbuffer[j](1);
        footR_msg.positions[j].z = zp->footRbuffer[j](2);
        //q = AngleAxisd(zp->footRbuffer[j](3), Vector3d::UnitX())* AngleAxisd(zp->footRbuffer[j](4), Vector3d::UnitY())* AngleAxisd(zp->footRbuffer[j](5), Vector3d::UnitZ());
        // footR_msg.quaternions[j].x = q.x();
        // footR_msg.quaternions[j].y = q.y();
        // footR_msg.quaternions[j].z = q.z();
        // footR_msg.quaternions[j].w = q.w();


        /*
        dp->CoMBuffer.pop_front();
        dp->VRPBuffer.pop_front();
        dp->DCMBuffer.pop_front();
        zp->footLbuffer.pop_front();
        zp->footRbuffer.pop_front();
        */
        j++;
    }
    isPlanAvailable = true;
    result_.status = 1;
    as_->setSucceeded(result_);
    CoM_msg.header.stamp = ros::Time::now();
    CoM_pub.publish(CoM_msg);
    DCM_msg.header.stamp = ros::Time::now();
    DCM_pub.publish(DCM_msg);
    VRP_msg.header.stamp = ros::Time::now();
    VRP_pub.publish(VRP_msg);
    footL_msg.header.stamp = ros::Time::now();
    footL_pub.publish(footL_msg);
    footR_msg.header.stamp = ros::Time::now();
    footR_pub.publish(footR_msg);
    TrajectoryGoal.CoM = CoM_msg;
    TrajectoryGoal.DCM = DCM_msg;
    TrajectoryGoal.VRP = VRP_msg;
    TrajectoryGoal.LLeg = footL_msg;
    TrajectoryGoal.RLeg = footR_msg;
    ac_->sendGoal(TrajectoryGoal);
    std::cout<<"Motion Plan Completed"<<std::endl;
}

lipm::~lipm()
{
    delete zp;
    delete dp;
}