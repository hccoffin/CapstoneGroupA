cmake_minimum_required(VERSION 2.8.3)
project(sim_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
	roscpp
	std_msgs
	sensor_msgs
	geometry_msgs
  controller_manager
  joint_state_controller
  robot_state_publisher
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)


catkin_package(
	CATKIN_DEPENDS
		sensor_msgs
		std_msgs
		geometry_msgs
)

###########
## Build ##
###########

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

add_library(gazebo_simulator src/simulator.cpp)
target_link_libraries(gazebo_simulator ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(
  TARGETS gazebo_simulator
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#install(
#  DIRECTORY launch rviz urdf models
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
