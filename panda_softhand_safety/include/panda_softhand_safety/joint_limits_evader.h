
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

// KDL includes
#include <kdl/kdl.hpp>
#include <kdl/jntarray.hpp>

namespace panda_softhand_safety {

class JointLimitsEvader {

    public: 

        JointLimitsEvader(ros::NodeHandle& nh);

        ~JointLimitsEvader();

        // Function that spins to check for joint limits violations
        bool CheckLimitsViolations();

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
        double acceleration_limit_;            // Maximum acceleration for each joint

        // Joint limits structure
        struct limits_ {

			KDL::JntArray pos_min;
			KDL::JntArray pos_max;
			KDL::JntArray pos_center;
			KDL::JntArray vel_max;
            KDL::JntArray acc_max;
    
		} joint_limits_;

        // Other KDL stuff
        KDL::Chain kdl_chain_;

};

}



#endif // JOINT_LIMITS_EVADER_H