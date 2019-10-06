/* HAND CONTROL - For closing SoftHand in to a desired position or at a desired velocity
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "panda_softhand_control/HandControl.h"

HandControl::HandControl(ros::NodeHandle& nh_,
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr_){
        
        // Initializing the class node
        this->nh = nh_;

        // Setting the action client
        this->hand_client_ptr = hand_client_ptr_;
}

HandControl::~HandControl(){
    // Nothing to do here yet
}

// This is the callback function of the hand control service
bool HandControl::call_hand_control(panda_softhand_control::hand_control::Request &req, panda_softhand_control::hand_control::Response &res){

    // Saving the callback msg (Here we hope that the traj has been created correctly)
    this->computed_trajectory = req.computed_trajectory;

    // Sending the trajectory to hand
    if(!this->sendHandTrajectory(this->computed_trajectory)){
        ROS_ERROR("Could not send computed trajectory from HandControl object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return true
    res.answer = true;
    return true;
}

// Sends trajectory to the hand joint trajectory controller
bool HandControl::sendHandTrajectory(trajectory_msgs::JointTrajectory trajectory){

    // Waiting for the hand server to be ready
    if(!this->hand_client_ptr->waitForServer(ros::Duration(1,0))){
        ROS_ERROR("The hand client is taking too much to get ready. Returning...");
        return false;
    }

	// Send the message and wait for the result
	control_msgs::FollowJointTrajectoryGoal goalmsg;
	goalmsg.trajectory = trajectory;

    this->hand_client_ptr->sendGoal(goalmsg);

    if(!this->hand_client_ptr->waitForResult(ros::Duration(20, 0))){
        ROS_ERROR("The hand client is taking too to complete goal execution. Returning...");
        return false;
    }

    return true;
}