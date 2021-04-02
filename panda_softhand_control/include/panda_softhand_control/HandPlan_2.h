/* HAND PLAN - For closing SoftHand in to a desired position or at a desired velocity
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic includes
#include <ros/service.h>
#include "ros/ros.h"

// ROS msg includes
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>

// Custom msg and srv includes
#include "panda_softhand_control/hand_plan.h"

// Defines
#define     DEBUG   1       // Prints out additional stuff

class HandPlan_2 {

    /// public variables and functions ------------------------------------------------------------
	public:
		HandPlan_2(ros::NodeHandle& nh_, int n_wp_, std::string synergy_joint_name_);

        ~HandPlan_2();

        // This is the callback function of the hand plan service
	  	bool call_hand_plan(panda_softhand_control::hand_plan::Request &req, panda_softhand_control::hand_plan::Response &res);

        // The callback function for the joint states subscriber
	  	void joints_callback(const sensor_msgs::JointState::ConstPtr &jnt_msg);

	  	// Initialize the things for setting up things. It is called by the callback
	  	bool initialize();

		// Performs computation of points towards goal
		void computeTrajectory(double present_syn, double goal_syn, double time);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Subscriber to the joint states
        ros::Subscriber joints_sub;

        // The name and the present value of synergy joint
        std::string synergy_joint_name;
        double present_syn;

        // Latest saved joint states message
        sensor_msgs::JointState::ConstPtr saved_jnt_msg;

        // Number of waypoints (needed for 0 to 1 synergy) for trajectory points
        int n_wp;

        // The goal stuff and closing time (in seconds)
        double goal_value;
        double goal_duration;

        // Basic times
        ros::Duration nanosecond = ros::Duration(0, 1);
        ros::Duration millisecond = ros::Duration(0, 1000000);

        // Joint trajectory computed to be sent to robot
        trajectory_msgs::JointTrajectory computed_trajectory;  
	
};