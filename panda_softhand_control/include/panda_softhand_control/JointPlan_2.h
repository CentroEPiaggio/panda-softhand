/* JOINT CONTROL - Uses moveit movegroupinterface to plan towards a joint configuration
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic includes
#include <ros/service.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// ROS msg includes
#include <trajectory_msgs/JointTrajectory.h>

// Custom msg and srv includes
#include "panda_softhand_control/joint_plan.h"

// MoveIt! includes
#include <moveit/move_group_interface/move_group_interface.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz
// #define     PROMPT          // Waits for confermation in RViz before execution

class JointPlan_2 {

    /// public variables and functions ------------------------------------------------------------
	public:
		JointPlan_2(ros::NodeHandle& nh_, std::string group_name_);

        ~JointPlan_2();

        // This is the callback function of the joint plan service
	  	bool call_joint_plan(panda_softhand_control::joint_plan::Request &req, panda_softhand_control::joint_plan::Response &res);

	  	// Initialize the things for motion planning. It is called by the callback
	  	bool initialize(panda_softhand_control::joint_plan::Request &req);

		// Performs motion planning for the joints towards goal
		bool performMotionPlan();


	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Important names and values
        std::string group_name;                                 // Name of the MoveIt group

        // The present joint config and the goal joint config
		std::vector<double> joint_now;							// The current joint config
	  	std::vector<double> joint_goal;							// The goal joint config given by service call

        // Joint trajectory computed to be sent to robot
        trajectory_msgs::JointTrajectory computed_trajectory;  

		// INLINE PRIVATE FUCTIONS
        // Needed to check if the start joint config has been arbitrarily chosen as null (this means to plan from present joint position)
        inline bool is_really_null_config(std::vector<double> joints){
			bool all_zero = true;

            for (int n : joints){
				if (n == 0.0) all_zero = false;
			}

            return all_zero;
        }
	
};