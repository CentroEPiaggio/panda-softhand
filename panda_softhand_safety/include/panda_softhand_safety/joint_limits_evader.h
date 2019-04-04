
/*  
    JOINT LIMITS EVADER CLASS
    This object checks for joints' position, velocity, acceleration of a robot listening from /joint_states
    and getting the limits from the urdf.
*/

#ifndef JOINT_LIMITS_EVADER_H
#define JOINT_LIMITS_EVADER_H

// ROS includes
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <string>

// MSG includes
#include <sensor_msgs/JointState.h>
#include "panda_softhand_safety/SafetyInfo.h"

// KDL includes
#include <kdl/kdl.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chain.hpp>

namespace panda_softhand_safety {

class JointLimitsEvader {

    public: 

        JointLimitsEvader(ros::NodeHandle& nh);

        ~JointLimitsEvader();

        // Function that spins to check for joint limits violations
        bool CheckLimitsViolations(panda_softhand_safety::SafetyInfo &safety_msg);

        // Callback to joint_states topic
        void joints_callback(const sensor_msgs::JointState::ConstPtr &msg);

    private:

        // Function to parse parameters
        bool parse_parameters(ros::NodeHandle& nh);

        // Function to initialize all main variables
        bool initialize();

        // ROS variables
        ros::NodeHandle jle_nh_;
        ros::Subscriber sub_joint_states_;

        // Messages
        sensor_msgs::JointState current_joints_;
        boost::shared_ptr<sensor_msgs::JointState const> first_joints_;

        // Constants
        std::string joints_topic_ = "/joint_states";

        // Parsed variables
        
        double joint_pos_thresh_;                // Joint position limit violation threshold
        double joint_vel_thresh_;                // Joint velocity limit violation threshold
        double joint_acc_thresh_;                // Joint acceleration limit violation threshold
        double acceleration_limit_;             // Maximum acceleration for each joint

        // Joint limits structure
        struct limits_ {

			KDL::JntArray pos_min;
			KDL::JntArray pos_max;
			KDL::JntArray pos_center;
			KDL::JntArray vel_max;
            KDL::JntArray acc_max;
    
		} joint_limits_;

        // Present and past joint values
        struct joint_values_ {

			std::vector<double> pos;
			std::vector<double> vel;
            std::vector<double> acc;
    
		};

        joint_values_ current_joint_values_;
        joint_values_ previous_joint_values_;

        // Time variables for computing acceleration
        ros::Time time_now_;
        ros::Time time_before_;
        ros::Duration period_;

        // Other KDL stuff
        KDL::Chain kdl_chain_;

        // Print out bools
        std::vector<bool> violation_print_ = {true, true, true, true};

};

}



#endif // JOINT_LIMITS_EVADER_H