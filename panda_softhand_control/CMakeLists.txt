cmake_minimum_required(VERSION 2.8.3)
project(panda_softhand_control)

add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  rviz
  rviz_visual_tools
  message_generation
  actionlib_msgs
  geometry_msgs
  actionlib
  sensor_msgs
  tf
  tf_conversions
  moveit_msgs
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_visual_tools
  interactive_markers 
  visualization_msgs
  moveit_core
  franka_msgs
  franka_control
  eigen_conversions
)

add_service_files(FILES
  hand_plan.srv
  hand_wait.srv
  hand_control.srv
  arm_wait.srv
  arm_control.srv
  slerp_plan.srv
  pose_plan.srv
  joint_plan.srv
  set_object.srv
  complex_grasp.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  tf
  geometry_msgs
  trajectory_msgs
)

# find_package(SDFormat REQUIRED)
find_package(sdformat10 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs moveit_ros_planning_interface message_runtime std_msgs std_srvs
  LIBRARIES ${PROJECT_NAME}_lib
)

set(OBJECTS_SOURCE_FILES
  src/utils/parsing_utilities.cpp
  src/HandPlan.cpp
  src/HandControl.cpp
  src/ArmControl.cpp
  src/SlerpPlan.cpp 
  src/PosePlan.cpp
  src/JointPlan.cpp
  src/PandaSoftHandClient.cpp
  src/TaskSequencer.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include/eigen3
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(${SDFormat_LIBRARY_DIRS})

## Declare a C++ library
set(LIB_SOURCE_FILES
    src/PandaSoftHandClient.cpp
)
add_library(${PROJECT_NAME}_lib
  ${LIB_SOURCE_FILES}
)
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# NODE: Throwing angle listener
add_executable(my_throwing_angle_listener src/throwing_angle_listener.cpp)
target_link_libraries(my_throwing_angle_listener ${catkin_LIBRARIES})

# NODE: Lower level position controls
add_executable(control_server src/control_server.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(control_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_server ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# NODE: Higher level sequences of position controls
add_executable(task_server src/task_server.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(task_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_server ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# NODE: cpp file related to Throwing Task Service

add_executable(task_server_throwing src/task_server_throwing.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(task_server_throwing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_server_throwing ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# NODE: Utility to publish collision objects to planning scene from world file
add_executable(world_to_planning_scene_node src/SdfToPlanningScene.cpp)
target_link_libraries(world_to_planning_scene_node ${catkin_LIBRARIES} ${ROS_LIBRARIES} ${SDFormat_LIBRARIES})

# Mark cpp header files for installation
install(TARGETS ${PROJECT_NAME}_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)