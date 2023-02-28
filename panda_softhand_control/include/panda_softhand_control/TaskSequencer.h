/* TASK SEQUENCER - Contains all recepies for grasping, handshake and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic Includes
#include "ros/ros.h"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "panda_softhand_control/set_object.h"
#include "geometry_msgs/Pose.h"
#include <controller_manager_msgs/SwitchController.h>
#include <franka_msgs/FrankaState.h>
#include <franka_msgs/ErrorRecoveryActionGoal.h>

// Parsing includes
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

// Custom Includes
#include "PandaSoftHandClient.h"

// Other Includes

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz

class TaskSequencer {

    /// public variables and functions ------------------------------------------------------------
	public:
		TaskSequencer(ros::NodeHandle& nh_);

        ~TaskSequencer();

        // Parameters parsing
        bool parse_task_params();

        // Convert xyzrpy vector to geometry_msgs Pose
        geometry_msgs::Pose convert_vector_to_pose(std::vector<double> input_vec);

        // To switch the controllers
        bool switch_controllers(std::string robot_name, std::string from_controller, std::string to_controller);

        // Callback for object pose subscriber
        void get_object_pose(const geometry_msgs::Pose::ConstPtr &msg);

        // Callback for franka state subscriber
        void get_franka_state(const franka_msgs::FrankaState::ConstPtr &msg);

        // Callback for simple grasp task service
        bool call_simple_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

        // Callback for set object service
        bool call_set_object(panda_softhand_control::set_object::Request &req, panda_softhand_control::set_object::Response &res);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Subscriber to object pose and the pose
        ros::Subscriber object_sub;
        geometry_msgs::Pose object_pose_T;

        // Subscriber to franka_states for getting tau_ext on joints and other info and Publisher of its norm
        ros::Subscriber franka_state_sub;
        franka_msgs::FrankaState latest_franka_state;
        bool franka_ok = true;
        double tau_ext_norm = 0.0;
        ros::Publisher pub_tau_ext_norm;
        ros::Publisher pub_franka_recovery;         // TODO: Recover from error automatically

        // The Panda SoftHand Client
        PandaSoftHandClient panda_softhand_client;

        // A controller_mangager msg for switching controllers
        controller_manager_msgs::SwitchController switch_controller;

        // The switch controller service name
        std::string switch_service_name = "/controller_manager/switch_controller";

        // Topic and service names
        std::string object_topic_name;
        std::string franka_state_topic_name = "/franka_state_controller/franka_states";
        std::string grasp_task_service_name;
        std::string set_object_service_name;

        // Service Servers
        ros::ServiceServer grasp_task_server;
        ros::ServiceServer set_object_server;

        // The XmlRpc value for parsing complex params
        XmlRpc::XmlRpcValue task_seq_params;

        // Parsed task sequence variables
        std::string robot_name;                     // Name of the robot (namespace)
        std::string robot_joints_name;              // Name of the robot joints (without the number of the joints)
        std::string pos_controller;                 // Name of position controller
        std::string imp_controller;                 // Name of impedance controller
        std::vector<double> home_joints;
        std::vector<double> grasp_transform;
        geometry_msgs::Pose grasp_T;
        std::vector<double> pre_grasp_transform;
        geometry_msgs::Pose pre_grasp_T;
        std::vector<double> handover_joints;
        double handover_thresh;

        std::map<std::string, std::vector<double>> poses_map;     // The map containing the notable poses

};