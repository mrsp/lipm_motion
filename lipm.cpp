#include "lipm_motion/lipm.h"

lipm::lipm(ros::NodeHandle nh_, RobotParameters robot_)
{
    nh = nh_;
    robot = robot_;
    ros::NodeHandle n_p("~");
    isPlanAvailable = false;


    zp = new zmpPlanner(robot);
    dp = new dcmPlanner(robot);
    CoM_pub = nh.advertise<nav_msgs::Path>("lipm_motion/CoM",1000);
    DCM_pub = nh.advertise<nav_msgs::Path>("lipm_motion/DCM",1000);
    VRP_pub = nh.advertise<nav_msgs::Path>("lipm_motion/VRP",1000);
    footL_pub = nh.advertise<nav_msgs::Path>("lipm_motion/LLeg",1000);
    footR_pub = nh.advertise<nav_msgs::Path>("lipm_motion/RLeg",1000);
}

void lipm::desiredFootstepsCb()
{
    VectorXd planL;
    planL.resize(6);
    planL.setZero();
    planL(1) = 0.05;
    VectorXd planR;
    planR.resize(6);
    planR.setZero();
    planR(1) = -0.05;
    VectorXd temp;
    temp.resize(6);


    zp->setFeet(planL, planR);

    /** Initial Walking Instruction **/
    WalkInstruction i;
    temp(0) = 0.05;
    i.target = planL + temp;
    i.targetSupport = SUPPORT_LEG_RIGHT;
    /** ZMP in the Middle of Convex Hull **/
    i.targetZMP = SUPPORT_LEG_RIGHT;
    /** Number of Discrete Time steps of the Initial Walking Instruction **/
    i.steps = 100;
    i.step_id = -1;
    /** Adding the Walking Instruction to the Walking Buffer for Execution **/
    zp->stepAnkleQ.push(i);

    temp(0) = 0.06;
    i.target = planR + temp;
    i.targetSupport = SUPPORT_LEG_LEFT;
    /** ZMP in the Middle of Convex Hull **/
    i.targetZMP = SUPPORT_LEG_LEFT;
    /** Number of Discrete Time steps of the Initial Walking Instruction **/
    i.steps = 100;
    i.step_id = -1;
    /** Adding the Walking Instruction to the Walking Buffer for Execution **/
    zp->stepAnkleQ.push(i);

    temp(0) = 0.06;
    i.target = planL + temp;
    i.targetSupport = SUPPORT_LEG_RIGHT;
    /** ZMP in the Middle of Convex Hull **/
    i.targetZMP = SUPPORT_LEG_RIGHT;
    /** Number of Discrete Time steps of the Initial Walking Instruction **/
    i.steps = 100;
    i.step_id = -1;
    /** Adding the Walking Instruction to the Walking Buffer for Execution **/
    zp->stepAnkleQ.push(i);

    i.target = planL + temp;
    i.targetSupport = SUPPORT_LEG_RIGHT;
    /** ZMP in the Middle of Convex Hull **/
    i.targetZMP = SUPPORT_LEG_BOTH;
    /** Number of Discrete Time steps of the Initial Walking Instruction **/
    i.steps = 100;
    i.step_id = -1;
    /** Adding the Walking Instruction to the Walking Buffer for Execution **/
    zp->stepAnkleQ.push(i);




    zp->plan();
    Vector2f DCM, CoM, VRP;
    DCM.setZero();
    CoM.setZero();
    VRP.setZero();
    dp->plan(zp->ZMPbuffer, DCM, CoM, VRP);


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

    CoM_msg.positions.resize(dp->CoMBuffer.size());
    CoM_msg.velocities.resize(dp->CoMBuffer.size());
    CoM_msg.accelerations.resize(dp->CoMBuffer.size());
    CoM_msg.header.stamp = ros::Time::now();
    CoM_msg.header.frame_id = "odom";
    VRP_msg.positions.resize(dp->VRPBuffer.size());
    VRP_msg.header.stamp = ros::Time::now();
    VRP_msg.header.frame_id = "odom";

    DCM_msg.positions.resize(dp->DCMBuffer.size());
    DCM_msg.velocities.resize(dp->DCMBuffer.size());
    DCM_msg.header.stamp = ros::Time::now();
    DCM_msg.header.frame_id = "odom";

    footL_msg.positions.resize(zp->footLbuffer.size());
    footL_msg.header.stamp = ros::Time::now();
    footL_msg.header.frame_id = "odom";

    footR_msg.positions.resize(zp->footRbuffer.size());
    footR_msg.header.stamp = ros::Time::now();
    footR_msg.header.frame_id = "odom";

    int j = 0;
    while (j < dp->CoMBuffer.size())
    {

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
        ///Right Foot Position
        footR_msg.positions[j].x = zp->footRbuffer[j](0);
        footR_msg.positions[j].y = zp->footRbuffer[j](1);
        footR_msg.positions[j].z = zp->footRbuffer[j](2);

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
}


void lipm::publishPath()
{
    if(!isPlanAvailable)
        return;
    
    CoM_pub.publish(CoM_path);
    DCM_pub.publish(DCM_path);
    VRP_pub.publish(VRP_path);
    footL_pub.publish(footL_path);
    footR_pub.publish(footR_path);
}


lipm::~lipm()
{
    delete zp;
    delete dp;
}