
/*  
    COLLISION EVADER CLASS
    This object checks for collisions of a robot with the environment listening from the planning scene.
*/

#ifndef COLLISION_EVADER_H
#define COLLISION_EVADER_H

// ROS includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <string>

// MSG includes
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>

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
        bool CheckCollision();

        // Callback to joint_states topic
        void joints_callback(const sensor_msgs::JointState::ConstPtr &msg);

    private:

        // Function to parse parameters
        bool parse_parameters(ros::NodeHandle& nh);

        // Function to initialize all main variables
        bool initialize();

        // ROS variables
        ros::NodeHandle ce_nh_;
        ros::Subscriber sub_joint_states_;

        // Messages
        sensor_msgs::JointState current_joints_;
        boost::shared_ptr<sensor_msgs::JointState const> first_joints_;

        // Moveit! variables
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group_ptr_;
        std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
        robot_model::RobotModelPtr kinematic_model_;
        std::shared_ptr<planning_scene::PlanningScene> planning_scene_;
        std::shared_ptr<robot_state::RobotState> current_state_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

        std::map<std::string, moveit_msgs::CollisionObject> collision_objects_map_;

        // MoveIt! messages
        collision_detection::CollisionRequest collision_request_;
        collision_detection::CollisionResult collision_result_;

        // Constants
        std::string joints_topic_ = "/joint_states";

        // Other variables
        bool collision_found_ = false;          // Set by CheckCollision()
        double distance_to_env_;                // Current distance from environment
        double distance_to_self_;               // Current distance from self

        // Parsed variables
        std::string group_name_;                // Robots MoveIt! group
        double collision_threshold_;            // Minimum distance from collision

};

}

#endif // COLLISION_EVADER_H