cmake_minimum_required(VERSION 3.5)
project(lipm_motion)

add_compile_options(-std=c++11 )
add_definitions("-DBOOST_MPL_LIMIT_LIST_SIZE=30")

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
geometry_msgs
nav_msgs
actionlib
lipm_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lipm_motion
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs  lipm_msgs
  DEPENDS EIGEN3 Boost
)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_executable(lipm_motion_node src/main.cpp   src/lipm_ros.cpp  src/zmpPlanner.cpp  src/LIPMPlanner.cpp  src/LIPMDynamics.cpp)
add_dependencies(lipm_motion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lipm_motion_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
