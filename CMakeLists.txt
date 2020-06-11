cmake_minimum_required(VERSION 3.5)
project(lipm_motion)

add_compile_options(-std=c++11 )
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
geometry_msgs
nav_msgs
message_generation
actionlib
actionlib_msgs
)

add_message_files(
FILES
TrajectoryPoints.msg
StepTarget.msg
)
add_action_files(
  DIRECTORY action
  FILES MotionPlan.action
)

generate_messages(
DEPENDENCIES
std_msgs
geometry_msgs
lipm_motion
actionlib_msgs
)



find_package(Eigen3)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lipm_motion
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs message_runtime
  DEPENDS Eigen3 Boost
)


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIR})

add_executable(lipm_motion main.cpp lipm.cpp zmpPlanner.cpp dcmPlanner.cpp delayedObserverDCM.cpp RobotParameters.cpp)
target_link_libraries(lipm_motion ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${Boost_LIBRARIES})