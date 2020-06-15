#ifndef __LIPMMOTION_H__
#define __LIPMMOTION_H__
#include <ros/ros.h>
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
    zmpPlanner* zp;
    dcmPlanner* dp;
    bool isPlanAvailable;
    ros::Publisher CoM_pub,DCM_pub,VRP_pub,footL_pub,footR_pub;
    int SS_Instructions, DS_Instructions;
    lipm_motion::MotionPlanResult result_;
    lipm_motion::MotionPlanFeedback feedback_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    actionlib::SimpleActionServer<lipm_motion::MotionPlanAction> *as_; 

    ~lipm();
    lipm(ros::NodeHandle nh_);
    /** @fn desiredFootstepsCb(const lipm_motion::MotionPlanGoalConstPtr &goal)
     * @brief computes a desired motion plan for CoM/DCM/VRP and legs
     */
    void desiredFootstepsCb(const lipm_motion::MotionPlanGoalConstPtr &goal);
    /** @fn void publishPath()
     *  @brief publish the computed desired motion
     */
  
    void publishPath();
};
#endif