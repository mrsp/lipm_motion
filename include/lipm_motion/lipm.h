#ifndef __LIPMMOTION_H__
#define __LIPMMOTION_H__
#include <ros/ros.h>
#include <lipm_motion/RobotParameters.h>
#include <lipm_motion/zmpPlanner.h>
#include <lipm_motion/dcmPlanner.h>
#include <iostream>
#include <lipm_motion/TrajectoryPoints.h>
#include <nav_msgs/Path.h>

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
public:
    ~lipm();
    lipm(ros::NodeHandle nh_, RobotParameters robot_);
    /** @fn desiredFootstepsCb()
     * @brief computes a desired motion plan for CoM/DCM/VRP and legs
     */
    void desiredFootstepsCb();
    /** @fn void publishPath()
     *  @brief publish the computed desired motion
     */
  
    void publishPath();
};
#endif