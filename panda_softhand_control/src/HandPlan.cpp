/* HAND PLAN - For closing SoftHand in to a desired position or at a desired velocity
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "panda_softhand_control/HandPlan.h"

HandPlan::HandPlan(ros::NodeHandle& nh_, int n_wp_, std::string first_synergy_joint_name_, std::string second_synergy_joint_name_){
        
        // Initializing the class node
        this->nh = nh_;

        // Saving the first and second syn joint
        this->first_synergy_joint_name = first_synergy_joint_name_;
        this->second_synergy_joint_name = second_synergy_joint_name_;
        
        // Initializing the subscriber and waiting for a message
        this->joints_sub = this->nh.subscribe("/joint_states", 1, &HandPlan::joints_callback, this);
        this->saved_jnt_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", this->nh);

        // Setting the number of waypoints, closing time and the action client
        this->n_wp = n_wp_;
}

HandPlan::~HandPlan(){
    // Nothing to do here yet
}

// This is the callback function of the hand plan service
bool HandPlan::call_hand_plan(panda_softhand_msgs::hand_plan::Request &req, panda_softhand_msgs::hand_plan::Response &res){

    // Saving the callback msgs and checking limits for the first synergy (saturating)
    this->goal_value_first_syn = req.goal_first_syn;
    if(this->goal_value_first_syn < 0.0) this->goal_value_first_syn = 0.0;
    if(this->goal_value_first_syn > 1.0) this->goal_value_first_syn = 1.0;

    // Saving the callback msgs and checking limits for the second synergy (saturating)
    this->goal_value_second_syn = req.goal_second_syn;
    if(this->goal_value_second_syn < -1.0) this->goal_value_second_syn = -1.0;
    if(this->goal_value_second_syn > 1.0) this->goal_value_second_syn = 1.0;

    this->goal_duration = req.goal_duration;
    if(this->goal_duration <= 0.0){
        ROS_ERROR("The given hand closing duration is not positive. Returning...");
        res.answer = false;
        return false;
    }

    // Setting things by saving joint states and synergy value, ecc.
    if(!this->initialize()){
        ROS_ERROR("Could not initialize HandPlan object. Returning...");
        res.answer = false;
        return false;
    }

    // Computing the joint trajectory from present_syn to goal_value (or with wanted vel until complete closure)
    this->computeTrajectory(this->present_first_syn, this->goal_value_first_syn,this->present_second_syn, this->goal_value_second_syn, this->goal_duration);

    // At this point all is fine, return the computed trajectory
    res.computed_trajectory = this->computed_trajectory;
    res.answer = true;
    return true;
}

// The callback function for the joint states subscriber
void HandPlan::joints_callback(const sensor_msgs::JointState::ConstPtr &jnt_msg){

    // Saving the message
    this->saved_jnt_msg = jnt_msg;
    // if(DEBUG) ROS_WARN_STREAM("Inside the joint state subscriber callback.");
}

// Initialize the things for setting up things. It is called by the callback
bool HandPlan::initialize(){

    // Spinning once for message
    ros::spinOnce();

    // Getting the first synergy value from the latest saved joint message
    int index_first_syn = std::find(saved_jnt_msg->name.begin(),saved_jnt_msg->name.end(),
		this->first_synergy_joint_name) - saved_jnt_msg->name.begin();

    // Getting the synergy value, making sure that it exists
    try {
        this->present_first_syn = saved_jnt_msg->position[index_first_syn];
    } catch (const std::exception& e) {
        std::cout << e.what();
        ROS_ERROR_STREAM("Could not find the joint " << this->first_synergy_joint_name << " in the joint states (saved msg). Returning...");
        return false;
    }

    // Getting the second synergy value from the latest saved joint message
    int index_second_syn = std::find(saved_jnt_msg->name.begin(),saved_jnt_msg->name.end(),
		this->second_synergy_joint_name) - saved_jnt_msg->name.begin();

    // Getting the synergy value, making sure that it exists
    try {
        this->present_second_syn = saved_jnt_msg->position[index_second_syn];
    } catch (const std::exception& e) {
        std::cout << e.what();
        ROS_ERROR_STREAM("Could not find the joint " << this->second_synergy_joint_name << " in the joint states (saved msg). Returning...");
        return false;
    }

    // Cout and return
    if(DEBUG) ROS_INFO_STREAM("The first  present synergy value is " << this->present_first_syn << ".");
    if(DEBUG) ROS_INFO_STREAM("The second present synergy value is " << this->present_second_syn << ".");
    return true;
}

// Performs computation of points towards goal
void HandPlan::computeTrajectory(double present_first_syn, double goal_first_syn, double present_second_syn, double goal_second_syn, double time){

    // Computing the real number of waypoints with proportion
    int real_n_wp = std::max(std::ceil(std::abs(goal_value_first_syn - present_first_syn) * this->n_wp),std::ceil(std::abs(goal_value_second_syn - present_second_syn) * this->n_wp));

    // Objects needed for generating trajectory
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = {this->second_synergy_joint_name, this->first_synergy_joint_name};
    traj.header.stamp = ros::Time::now();
    trajectory_msgs::JointTrajectoryPoint point;

    // Generating waypoints
    for(int i = 1; i <= real_n_wp; i++){

        //Computing the position according to hand closing or opening
        double position_first_syn, position_second_syn;
        position_first_syn = present_first_syn + ((double (i) / double (real_n_wp)) * (goal_first_syn - present_first_syn));
        position_second_syn = present_second_syn + ((double (i) / double (real_n_wp)) * (goal_second_syn - present_second_syn));

        // Computing the time for the waypoint
        double time_wp = (double (i) / double (real_n_wp)) * time;

        // Debug cout
        if(DEBUG) ROS_INFO_STREAM("The " << i << "th computed position of the first syn is " << position_first_syn << " at time " << time_wp << ".");
        if(DEBUG) ROS_INFO_STREAM("The " << i << "th computed position of the second syn is " << position_second_syn << " at time " << time_wp << ".");
        
        // Pushing back position and time in point
        point.positions.clear();
        point.positions = {position_second_syn, position_first_syn};
        point.velocities = {0.0, 0.0};
        point.accelerations = {0.0, 0.0};
        point.effort = {0.0, 0.0};
        point.time_from_start = ros::Duration(this->millisecond * 1000 * time_wp);

        // Pushing back point into traj
        traj.points.push_back(point);
    }
    std::cout << "Pippo" << std::endl;

    this->computed_trajectory = traj;

    std::cout << "Pippo2" << std::endl;
}