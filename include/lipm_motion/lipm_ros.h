#ifndef __LIPMMOTIONROS_H__
#define __LIPMMOTIONROS_H__
#include <ros/ros.h>
#include <lipm_motion/zmpPlanner.h>
//#include <lipm_motion/dcmPlanner.h>
#include <lipm_motion/LIPMPlanner.h>

#include <iostream>
#include <lipm_msgs/TrajectoryPoints.h>
#include <nav_msgs/Path.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <lipm_msgs/MotionPlanAction.h>
#include <lipm_msgs/MotionControlAction.h>

class lipm_ros
{
private:
    /// ROS nodehanlder
    ros::NodeHandle nh;
    zmpPlanner* zp;
    //dcmPlanner* dp;
    LIPMPlanner* dp;

    bool debug;
    bool isPlanAvailable;
    ros::Publisher CoM_pub,DCM_pub,VRP_pub,footL_pub,footR_pub;
    ros::Publisher CoM_path_pub,DCM_path_pub,VRP_path_pub,footL_path_pub,footR_path_pub;
    nav_msgs::Path CoM_path, footL_path, footR_path, DCM_path, VRP_path;

    int SS_Instructions, DS_Instructions;
    lipm_msgs::MotionPlanResult result_;
    lipm_msgs::MotionPlanFeedback feedback_;
    Quaterniond q;
    lipm_msgs::MotionControlGoal TrajectoryGoal;
    double g, comZ;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    actionlib::SimpleActionServer<lipm_msgs::MotionPlanAction> *as_; 
    actionlib::SimpleActionClient<lipm_msgs::MotionControlAction> *ac_;

    ~lipm_ros();
    lipm_ros(ros::NodeHandle nh_);
    /** @fn desiredFootstepsCb(const lipm_motion::MotionPlanGoalConstPtr &goal)
     * @brief computes a desired motion plan for CoM/DCM/VRP and legs
     */
    void desiredFootstepsCb(const lipm_msgs::MotionPlanGoalConstPtr &goal);
    /** @fn void publishPath()
     *  @brief publish the computed desired motion
     */
  
    void publishPath();
};
#endif
