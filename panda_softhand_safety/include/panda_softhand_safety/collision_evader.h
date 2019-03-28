
/*  
    COLLISION EVADER CLASS
    This object checks for collisions of a robot with the environment listening from the planning scene.
*/

#ifndef COLLISION_EVADER_H
#define COLLISION_EVADER_H

// ROS includes
#include <ros/ros.h>
#include <string>

// MSG includes
#include "std_msgs/String.h"

// MoveIt! includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Other includes
#include <eigen_conversions/eigen_msg.h>

namespace panda_softhand_safety {

class CollisionEvader {

    public:

        CollisionEvader(ros::NodeHandle& nh);

        ~CollisionEvader();

        // Function that spins to check for collisions
        bool EvadeCollision();

    private:

        // Callback to joint_states topic
        void callback_joint_states(sensor_msgs::JointStateConstPtr msg);

        // Function to parse parameters
        bool parse_parameters(ros::NodeHandle& nh);

        // Function to initialize all main variables
        bool initialize();

        // ROS variables
        ros::NodeHandle ce_nh_;

        // Messages
        sensor_msgs::JointState current_joints_;

        // Moveit! variables


        // Parsed variables
        std::string group_name_;                // Robots MoveIt! group
        std::string safety_topic_name_;         // Topic where safety emergency messages are published

};

}

#endif // COLLISION_EVADER_H