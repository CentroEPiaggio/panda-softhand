cmake_minimum_required(VERSION 2.8.3)
project(panda_softhand_control)

add_definitions(-std=c++17)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  panda_softhand_msgs
  rviz
  rviz_visual_tools
  actionlib_msgs
  geometry_msgs
  actionlib
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

# add_service_files(FILES
# )

# generate_messages(DEPENDENCIES
#   std_msgs
#   tf
#   geometry_msgs
#   trajectory_msgs
# )

# find_package(SDFormat REQUIRED)
find_package(sdformat10 REQUIRED)
find_package(panda_softhand_msgs REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS panda_softhand_msgs actionlib_msgs moveit_ros_planning_interface message_runtime 
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

target_include_directories(${PROJECT_NAME}_lib PRIVATE
    ${catkin_INCLUDE_DIRS} ${panda_softhand_msgs_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${ROS_LIBRARIES} ${panda_softhand_msgs_LIBRARIES})

add_dependencies(${PROJECT_NAME}_lib panda_softhand_msgs_generate_messages_cpp)

# NODE: Lower level position controls
add_executable(control_server src/control_server.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(control_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} panda_softhand_msgs_generate_messages_cpp)
target_link_libraries(control_server ${catkin_LIBRARIES} ${ROS_LIBRARIES})

# NODE: Higher level sequences of position controls
add_executable(task_server src/task_server.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(task_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} panda_softhand_msgs_generate_messages_cpp)
target_link_libraries(task_server ${catkin_LIBRARIES} ${ROS_LIBRARIES})


# NODE: cpp file related to Throwing Task Service

add_executable(task_server_demo_only_pickandthrow src/task_server_demo_only_pickandthrow.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(task_server_demo_only_pickandthrow ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} panda_softhand_msgs_generate_messages_cpp)
target_link_libraries(task_server_demo_only_pickandthrow ${catkin_LIBRARIES} ${ROS_LIBRARIES})

add_executable(task_server_demo_only_pickandplace src/task_server_demo_only_pickandplace.cpp ${OBJECTS_SOURCE_FILES})
add_dependencies(task_server_demo_only_pickandplace ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} panda_softhand_msgs_generate_messages_cpp)
target_link_libraries(task_server_demo_only_pickandplace ${catkin_LIBRARIES} ${ROS_LIBRARIES})

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