#include "RobotParameters.h"
#include "zmpPlanner.h"
#include "dcmPlanner.h"
#include "postureStabilizer.h"
#include <iostream>
#include <ros/ros.h>
#include <lipm_motion/TrajectoryPoints.h>
#include <nav_msgs/Path.h>

int main(int argc, char** argv)
{


    ros::init(argc,argv,"lipm_motion_node");

    /** Compute Reference Points: x-y-z-roll-pitch-yaw **/
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


    RobotParameters robot;
    zmpPlanner tp(robot);
    dcmPlanner dp(robot);
    tp.setFeet(planL, planR);
    Vector2f DCM, CoM, VRP;
    DCM.setZero();
    CoM.setZero();
    VRP.setZero();
    dp.setState(DCM,  CoM,  VRP);
    
    ros::NodeHandle n;

    ros::Publisher CoM_pub = n.advertise<nav_msgs::Path>("lipm_motion/CoM",1000);
    ros::Publisher DCM_pub = n.advertise<nav_msgs::Path>("lipm_motion/DCM",1000);
    ros::Publisher VRP_pub = n.advertise<nav_msgs::Path>("lipm_motion/VRP",1000);
    ros::Publisher footL_pub = n.advertise<nav_msgs::Path>("lipm_motion/LLeg",1000);
    ros::Publisher footR_pub = n.advertise<nav_msgs::Path>("lipm_motion/RLeg",1000);


 

            /** Initial Walking Instruction **/
            WalkInstruction i;
            temp(0) = 0.05;
            i.target = planL +temp;
            i.targetSupport = SUPPORT_LEG_RIGHT;
            /** ZMP in the Middle of Convex Hull **/
            i.targetZMP = SUPPORT_LEG_RIGHT;
            /** Number of Discrete Time steps of the Initial Walking Instruction **/
            i.steps = 100;
            i.step_id = -1;
            /** Adding the Walking Instruction to the Walking Buffer for Execution **/
            tp.stepAnkleQ.push(i);

            temp(0) = 0.06;
            i.target = planR+ temp;
            i.targetSupport = SUPPORT_LEG_LEFT;
            /** ZMP in the Middle of Convex Hull **/
            i.targetZMP = SUPPORT_LEG_LEFT;
            /** Number of Discrete Time steps of the Initial Walking Instruction **/
            i.steps = 100;
            i.step_id = -1;
            /** Adding the Walking Instruction to the Walking Buffer for Execution **/
            tp.stepAnkleQ.push(i);

            temp(0) = 0.06;
            i.target = planL+ temp;
            i.targetSupport = SUPPORT_LEG_RIGHT;
            /** ZMP in the Middle of Convex Hull **/
            i.targetZMP = SUPPORT_LEG_RIGHT;
            /** Number of Discrete Time steps of the Initial Walking Instruction **/
            i.steps = 100;
            i.step_id = -1;
            /** Adding the Walking Instruction to the Walking Buffer for Execution **/
            tp.stepAnkleQ.push(i);

            i.target = planL + temp;
            i.targetSupport = SUPPORT_LEG_RIGHT;
            /** ZMP in the Middle of Convex Hull **/
            i.targetZMP = SUPPORT_LEG_BOTH;
            /** Number of Discrete Time steps of the Initial Walking Instruction **/
            i.steps = 100;
            i.step_id = -1;
            /** Adding the Walking Instruction to the Walking Buffer for Execution **/
            tp.stepAnkleQ.push(i);


    tp.plan();
    dp.plan(tp.ZMPbuffer,  DCM,  CoM,   VRP);
    std::cout<<" "<<tp.ZMPbuffer.size()<<std::endl;
    std::cout<<" "<<dp.VRPBuffer.size()<<std::endl;
    std::cout<<" "<<dp.DCMBuffer.size()<<std::endl;
    std::cout<<" "<<dp.CoMBuffer.size()<<std::endl;
    std::cout<<" "<<tp.footLbuffer.size()<<std::endl;
    std::cout<<" "<<tp.footRbuffer.size()<<std::endl;

    lipm_motion::TrajectoryPoints CoM_msg, VRP_msg, DCM_msg, footL_msg, footR_msg;
    nav_msgs::Path CoM_path, footL_path, footR_path, DCM_path, VRP_path;
    CoM_path.poses.resize(dp.CoMBuffer.size());
    //CoM_msg.velocities.resize(dp.CoMBuffer.size());
    //CoM_msg.accelerations.resize(dp.CoMBuffer.size());
    CoM_path.header.stamp = ros::Time::now();
    CoM_path.header.frame_id = "odom";
    
    footL_path.poses.resize(tp.footLbuffer.size());
    footL_path.header.stamp = ros::Time::now();
    footL_path.header.frame_id = "odom";
    footR_path.poses.resize(tp.footRbuffer.size());
    footR_path.header.stamp = ros::Time::now();
    footR_path.header.frame_id = "odom";

    VRP_path.poses.resize(dp.VRPBuffer.size());
    VRP_path.header.stamp = ros::Time::now();
    VRP_path.header.frame_id = "odom";
    DCM_path.poses.resize(dp.DCMBuffer.size());
    DCM_path.header.stamp = ros::Time::now();
    DCM_path.header.frame_id = "odom";

    CoM_msg.positions.resize(dp.CoMBuffer.size());
    CoM_msg.velocities.resize(dp.CoMBuffer.size());
    CoM_msg.accelerations.resize(dp.CoMBuffer.size());
    CoM_msg.header.stamp = ros::Time::now();
    CoM_msg.header.frame_id = "odom";
    VRP_msg.positions.resize(dp.VRPBuffer.size());
    VRP_msg.header.stamp = ros::Time::now();
    VRP_msg.header.frame_id = "odom";

    DCM_msg.positions.resize(dp.DCMBuffer.size());
    DCM_msg.velocities.resize(dp.DCMBuffer.size());
    DCM_msg.header.stamp = ros::Time::now();
    DCM_msg.header.frame_id = "odom";

    footL_msg.positions.resize(tp.footLbuffer.size());
    footL_msg.header.stamp = ros::Time::now();
    footL_msg.header.frame_id = "odom";

    footR_msg.positions.resize(tp.footRbuffer.size());
    footR_msg.header.stamp = ros::Time::now();
    footR_msg.header.frame_id = "odom";

   
    int j=0;
    while(j<dp.CoMBuffer.size())
    {
       
        CoM_path.poses[j].pose.position.x =  dp.CoMBuffer[j](0);
        CoM_path.poses[j].pose.position.y = dp.CoMBuffer[j](1);
        CoM_path.poses[j].pose.position.z = dp.CoMBuffer[j](2);
        
        CoM_msg.positions[j].x = dp.CoMBuffer[j](0);
        CoM_msg.positions[j].y = dp.CoMBuffer[j](1);
        CoM_msg.positions[j].z = dp.CoMBuffer[j](2);

        CoM_msg.velocities[j].x = dp.CoMBuffer[j](3);
        CoM_msg.velocities[j].y = dp.CoMBuffer[j](4);
        CoM_msg.velocities[j].z = dp.CoMBuffer[j](5);

        CoM_msg.accelerations[j].x = dp.CoMBuffer[j](6);
        CoM_msg.accelerations[j].y = dp.CoMBuffer[j](7);
        CoM_msg.accelerations[j].z = dp.CoMBuffer[j](8);    
        
        VRP_path.poses[j].pose.position.x =  dp.VRPBuffer[j](0);
        VRP_path.poses[j].pose.position.y = dp.VRPBuffer[j](1);
        VRP_path.poses[j].pose.position.z = dp.VRPBuffer[j](2);
        DCM_path.poses[j].pose.position.x =  dp.DCMBuffer[j](0);
        DCM_path.poses[j].pose.position.y = dp.DCMBuffer[j](1);
        DCM_path.poses[j].pose.position.z = dp.DCMBuffer[j](2);

        footL_path.poses[j].pose.position.x =  tp.footLbuffer[j](0);
        footL_path.poses[j].pose.position.y = tp.footLbuffer[j](1);
        footL_path.poses[j].pose.position.z = tp.footLbuffer[j](2);

        footR_path.poses[j].pose.position.x =  tp.footRbuffer[j](0);
        footR_path.poses[j].pose.position.y = tp.footRbuffer[j](1);
        footR_path.poses[j].pose.position.z = tp.footRbuffer[j](2);

        VRP_msg.positions[j].x = dp.VRPBuffer[j](0);
        VRP_msg.positions[j].y = dp.VRPBuffer[j](1);
        VRP_msg.positions[j].z = dp.VRPBuffer[j](2);

        DCM_msg.positions[j].x = dp.DCMBuffer[j](0);
        DCM_msg.positions[j].y = dp.DCMBuffer[j](1);
        DCM_msg.positions[j].z = dp.DCMBuffer[j](2);

        DCM_msg.velocities[j].x = dp.DCMBuffer[j](3);
        DCM_msg.velocities[j].y = dp.DCMBuffer[j](4);
        DCM_msg.velocities[j].z = dp.DCMBuffer[j](5);   

        footL_msg.positions[j].x = tp.footLbuffer[j](0);
        footL_msg.positions[j].y = tp.footLbuffer[j](1);
        footL_msg.positions[j].z = tp.footLbuffer[j](2);


        footR_msg.positions[j].x = tp.footRbuffer[j](0);
        footR_msg.positions[j].y = tp.footRbuffer[j](1);
        footR_msg.positions[j].z = tp.footRbuffer[j](2);

        /*
        dp.CoMBuffer.pop_front();
        dp.VRPBuffer.pop_front();
        dp.DCMBuffer.pop_front();
        tp.footLbuffer.pop_front();
        tp.footRbuffer.pop_front();
        */
        j++;
    }
    static ros::Rate rate(50);
    while(ros::ok())
    {
        CoM_pub.publish(CoM_path);
        DCM_pub.publish(DCM_path);
        VRP_pub.publish(VRP_path);
        footL_pub.publish(footL_path);
        footR_pub.publish(footR_path);

        rate.sleep();
    }

    return 0;
}
