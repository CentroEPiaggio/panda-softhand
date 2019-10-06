/* PANDA SOFTHAND CLIENT - Contains all necessary objects and functions to call the services to 
control Panda + SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic includes
#include <ros/service.h>

// ROS msg includes
#include <geometry_msgs/Pose.h>

// Custom msg and srv includes
#include "panda_softhand_control/hand_control.h"
#include "panda_softhand_control/joint_control.h"
#include "panda_softhand_control/pose_control.h"
#include "panda_softhand_control/slerp_control.h"

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz

class PandaSoftHandClient {

    /// public variables and functions ------------------------------------------------------------
	public:
        PandaSoftHandClient();

		PandaSoftHandClient(ros::NodeHandle& nh_);

        ~PandaSoftHandClient();

        // Initializing function
        bool initialize(ros::NodeHandle& nh_);

        // Service call function for hand plan
        bool call_hand_plan_service(double goal_syn, double goal_duration);

        // Service call function for hand control
        bool call_hand_control_service(double goal_syn, double goal_duration);

        // Service call function for joint control
        bool call_joint_service(std::vector<double> joint_goal);

        // Service call function for pose control
        bool call_pose_service(geometry_msgs::Pose goal_pose, bool is_goal_relative);

        // Service call function for slerp control
        bool call_slerp_service(geometry_msgs::Pose goal_pose, bool is_goal_relative);

	/// private variables -------------------------------------------------------------------------
	private:
		ros::NodeHandle nh;

        // Service names
        std::string hand_plan_service_name;
        std::string hand_control_service_name;
        std::string joint_service_name;
        std::string pose_service_name;
        std::string slerp_service_name;

        // Service clients
        ros::ServiceClient hand_plan_client;                // Client for hand plan service
        ros::ServiceClient hand_control_client;             // Client for hand control service
        ros::ServiceClient joint_client;                    // Client for joint control service
        ros::ServiceClient pose_client;                     // Client for pose control service
        ros::ServiceClient slerp_client;                    // Client for slerp control service
	
};