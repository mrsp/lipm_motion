cmake_minimum_required(VERSION 3.6)
project(lipm_motion)

find_package(Boost)

include_directories(include ${Boost_INCLUDE_DIRS})

add_executable(lipm_motion main.cpp zmpPlanner.cpp dcmPlanner.cpp delayedObserverDCM.cpp postureStabilizer.cpp RobotParameters.cpp)
target_link_libraries(lipm_motion ${Boost_LIBRARIES})
