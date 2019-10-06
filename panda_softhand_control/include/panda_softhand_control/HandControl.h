/* HAND CONTROL - For closing SoftHand in to a desired position or at a desired velocity
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic includes
#include <ros/service.h>

// ROS msg includes
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

// Custom msg and srv includes
#include "panda_softhand_control/hand_control.h"

// ROS action includes
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

// Defines
#define     DEBUG   1       // Prints out additional stuff

class HandControl {

    /// public variables and functions ------------------------------------------------------------
	public:
		HandControl(ros::NodeHandle& nh_,
            boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr_);

        ~HandControl();

        // This is the callback function of the hand control service
	  	bool call_hand_control(panda_softhand_control::hand_control::Request &req, panda_softhand_control::hand_control::Response &res);

		// Sends trajectory to the hand joint trajectory controller
		bool sendHandTrajectory(trajectory_msgs::JointTrajectory trajectory);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // The hand action client
        boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr;

        // Joint trajectory computed to be sent to robot
        trajectory_msgs::JointTrajectory computed_trajectory;  
	
};