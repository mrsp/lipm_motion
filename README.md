# README
Linear Inverted Pendulum Model (LIPM) Motion Planning for Humanoid/Bipedal legged robots. The code is open-source (BSD License). Please note that this work is an on-going research and thus some parts are not fully developed yet. Furthermore, the code will be subject to changes in the future which could include greater re-factoring.


# Getting Started
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

## Prerequisites
* Ubuntu 16.04 and later
* ROS kinetic and later
* Eigen 3.2.0 and later

## Installing
* git clone https://github.com/mrsp/lipm_msgs.git
* git clone https://github.com/mrsp/lipm_motion.git
* catkin_make -DCMAKE_BUILD_TYPE=Release 
* If you are using catkin tools run: catkin build  --cmake-args -DCMAKE_BUILD_TYPE=Release 



## ROS Examples
### Example Motion Plan for the Atlas humanoid robot
<p align="center">
<img src="img/MotionPlan.png" /img>
</p>
