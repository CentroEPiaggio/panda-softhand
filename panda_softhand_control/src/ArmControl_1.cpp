/* ARM CONTROL - Uses actionlib control the arm
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "panda_softhand_control/ArmControl_1.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

ArmControl_1::ArmControl_1(ros::NodeHandle& nh_,
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_1){
        
        ROS_INFO("Starting to create ArmControl_1 object");

        // Initializing node handle
        this->nh = nh_;

        // Initializing the arm client
        this->arm_client_ptr1 = arm_client_ptr_1;

        ROS_INFO("Finished creating ArmControl_1 object");
}

ArmControl_1::~ArmControl_1(){
    
    // Nothing to do here yet
}

// This is the callback function of the arm control service
bool ArmControl_1::call_arm_control(panda_softhand_control::arm_control::Request &req, panda_softhand_control::arm_control::Response &res){

    // Saving the callback msg (Here we hope that the traj has been created correctly)
    this->computed_trajectory = req.computed_trajectory;

    // Sending the trajectory to hand
    if(!this->sendJointTrajectory(this->computed_trajectory)){
        ROS_ERROR("Could not send computed trajectory from HandControl object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return true
    res.answer = true;
    return true;
}

// Sends trajectory to the joint_traj controller
bool ArmControl_1::sendJointTrajectory(trajectory_msgs::JointTrajectory trajectory){
    
    // Waiting for the arm server to be ready
    if(!this->arm_client_ptr1->waitForServer(ros::Duration(1,0))){
        ROS_ERROR("The arm client is taking too much to get ready. Returning...");
        return false;
    }

    // Setting the most recent time to the trajectory header
    std_msgs::Header empty_header;
    trajectory.header.stamp = ros::Time::now();
    // trajectory.header = empty_header;  + ros::Duration(2.0);
    ROS_INFO_STREAM("In ArmControl_1::sendJointTrajectory, the traj header stamp is " << trajectory.header.stamp
        << " and the time_from_start of first point is " << trajectory.points[1].time_from_start << ".");

	// Send the message and wait for the result
	control_msgs::FollowJointTrajectoryGoal goalmsg;
	goalmsg.trajectory = trajectory;

    this->arm_client_ptr1->sendGoal(goalmsg);

    // Not waiting for result here as it would be blocking

    return true;
}


// This is the callback function of the arm wait service
bool ArmControl_1::call_arm_wait(panda_softhand_control::arm_wait::Request &req, panda_softhand_control::arm_wait::Response &res){

     if(!this->arm_client_ptr1->waitForResult(req.wait_duration.data)){
        ROS_ERROR("The arm client is taking too to complete goal execution. Returning...");
        res.answer = false;
        return false;
    }

    res.answer = true;
    return true;

}
