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
find_package(PythonLibs 2.7)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lipm_motion
  CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs  lipm_msgs
  DEPENDS EIGEN3 Boost
)






include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_library(lipm_motion main.cpp  lipm_ros.cpp zmpPlanner.cpp dcmPlanner.cpp dcmDynamics.cpp)
add_dependencies(lipm_motion ${catkin_EXPORTED_TARGETS})
target_link_libraries(lipm_motion ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})

add_executable(lipm_motion_node main.cpp  lipm_ros.cpp zmpPlanner.cpp dcmPlanner.cpp dcmDynamics.cpp LIPMPlanner.cpp LIPMDynamics.cpp)
add_dependencies(lipm_motion_node ${catkin_EXPORTED_TARGETS})



add_executable(lipm_motion_test test.cpp lipm.cpp  zmpPlanner.cpp dcmPlanner.cpp dcmDynamics.cpp LIPMPlanner.cpp LIPMDynamics.cpp)
add_dependencies(lipm_motion_test ${catkin_EXPORTED_TARGETS})


target_include_directories(lipm_motion_test PRIVATE ${PYTHON_INCLUDE_DIRS})


target_link_libraries(lipm_motion_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(lipm_motion_test ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
