cmake_minimum_required(VERSION 2.8.3)
project(panda_softhand_description)

find_package(catkin REQUIRED
	franka_description
	soft_hand_description
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lwr_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)