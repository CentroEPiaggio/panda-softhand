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
)

add_service_files(FILES
  slerp_control.srv
  hand_control.srv
  pose_control.srv
  joint_control.srv
  set_object.srv
)

generate_messages(DEPENDENCIES
  std_msgs
  tf
  geometry_msgs
)

find_package(SDFormat REQUIRED)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs moveit_ros_planning_interface message_runtime std_msgs std_srvs
  LIBRARIES moveit_ros_planning_interface sdformat
)

set(OBJECTS_SOURCE_FILES
  src/utils/parsing_utilities.cpp
  src/SlerpControl.cpp 
  src/HandControl.cpp 
  src/PoseControl.cpp
  src/JointControl.cpp
  src/PandaSoftHandClient.cpp
  src/TaskSequencer.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} /usr/include/eigen3 ${SDFormat_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS})

# NODE: Lower level position controls
add_executable(control_server src/control_server.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(control_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_server ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# NODE: Higher level sequences of position controls
add_executable(task_server src/task_server.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(task_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(task_server ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# NODE: Utility to publish collision objects to planning scene from world file
add_executable(world_to_planning_scene_node src/SdfToPlanningScene.cpp)
target_link_libraries(world_to_planning_scene_node ${catkin_LIBRARIES} ${ROS_LIBRARIES} ${SDFormat_LIBRARIES}) 