/* HAND CONTROL - For closing SoftHand in to a desired position or at a desired velocity
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "panda_softhand_control/HandControl_2.h"

HandControl_2::HandControl_2(ros::NodeHandle& nh_,
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr_2){
        
        // Initializing the class node
        this->nh = nh_;

        // Setting the action client
        this->hand_client_ptr2 = hand_client_ptr_2;
}

HandControl_2::~HandControl_2(){
    // Nothing to do here yet
}

// This is the callback function of the hand control service
bool HandControl_2::call_hand_control(panda_softhand_control::hand_control::Request &req, panda_softhand_control::hand_control::Response &res){

    // Saving the callback msg (Here we hope that the traj has been created correctly)
    this->computed_trajectory = req.computed_trajectory;

    // Sending the trajectory to hand
    if(!this->sendHandTrajectory(this->computed_trajectory)){
        ROS_ERROR("Could not send computed trajectory from HandControl_2 object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return true
    res.answer = true;
    return true;
}

// Sends trajectory to the hand joint trajectory controller
bool HandControl_2::sendHandTrajectory(trajectory_msgs::JointTrajectory trajectory){

    // Waiting for the hand server to be ready
    if(!this->hand_client_ptr2->waitForServer(ros::Duration(1,0))){
        ROS_ERROR("The hand client is taking too much to get ready. Returning...");
        return false;
    }

    // Setting the most recent time to the trajectory header
    std_msgs::Header empty_header;
    trajectory.header.stamp = ros::Time::now();
    // trajectory.header = empty_header;  + ros::Duration(2.0);
    // ROS_INFO_STREAM("In HandControl_2::sendJointTrajectory, the traj header stamp is " << trajectory.header.stamp
    //     << " and the time_from_start of first point is " << trajectory.points[0].time_from_start << ".");

	// Send the message and wait for the result
	control_msgs::FollowJointTrajectoryGoal goalmsg;
	goalmsg.trajectory = trajectory;

    this->hand_client_ptr2->sendGoal(goalmsg);

    // Not waiting for result here as it would be blocking

    return true;
}

// Waits for the completion of the execution by hand joint trajectory controller
bool HandControl_2::call_hand_wait(panda_softhand_control::hand_wait::Request &req, panda_softhand_control::hand_wait::Response &res){
    if(!this->hand_client_ptr2->waitForResult(req.wait_duration.data)){
        ROS_ERROR("The hand client is taking too to complete goal execution. Returning...");
        res.answer = false;
        return false;
    }

    res.answer = true;
    return true;
}