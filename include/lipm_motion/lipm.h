#ifndef __LIPMMOTION_H__
#define __LIPMMOTION_H__
#include <ros/ros.h>
#include <lipm_motion/RobotParameters.h>
#include <lipm_motion/zmpPlanner.h>
#include <lipm_motion/dcmPlanner.h>
#include <iostream>
#include <lipm_motion/TrajectoryPoints.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>
#include <lipm_motion/MotionPlanAction.h>
class lipm
{
private:
    /// ROS nodehanlder
    ros::NodeHandle nh;
    RobotParameters robot;
    zmpPlanner* zp;
    dcmPlanner* dp;
    bool isPlanAvailable;
    ros::Publisher CoM_pub,DCM_pub,VRP_pub,footL_pub,footR_pub;
    lipm_motion::TrajectoryPoints CoM_msg, VRP_msg, DCM_msg, footL_msg, footR_msg;
    nav_msgs::Path CoM_path, footL_path, footR_path, DCM_path, VRP_path;
    actionlib::SimpleActionServer<lipm_motion::MotionPlanAction> *as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
    lipm_motion::MotionPlanResult result_;
    lipm_motion::MotionPlanFeedback feedback_;

public:
    ~lipm();
    lipm(ros::NodeHandle nh_, RobotParameters robot_);
    /** @fn desiredFootstepsCb()
     * @brief computes a desired motion plan for CoM/DCM/VRP and legs
     */
    void desiredFootstepsCb(const lipm_motion::MotionPlanGoalConstPtr &goal);
    /** @fn void publishPath()
     *  @brief publish the computed desired motion
     */
  
    void publishPath();
};
#endif