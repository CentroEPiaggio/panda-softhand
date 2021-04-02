/* JOINT CONTROL - Uses moveit movegroupinterface to plan towards a joint configuration
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "panda_softhand_control/JointPlan_2.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

JointPlan_2::JointPlan_2(ros::NodeHandle& nh_, std::string group_name_){
        
        ROS_INFO("Starting to create JointPlan_2 object");

        // Initializing node handle
        this->nh = nh_;

        // Initializing names
        this->group_name = group_name_;

        ROS_INFO("Finished creating JointPlan_2 object");
}

JointPlan_2::~JointPlan_2(){
    
    // Nothing to do here yet
}

// This is the callback function of the joint plan service
bool JointPlan_2::call_joint_plan(panda_softhand_control::joint_plan::Request &req, panda_softhand_control::joint_plan::Response &res){

    // Setting up things
    if(!this->initialize(req)){
        ROS_ERROR("Could not initialize JointPlan_2 object. Returning...");
        res.answer = false;
        return false;
    }

	// Perform motion plan towards the goal joint
    if(!this->performMotionPlan()){
        ROS_ERROR("Could not perform motion planning in JointPlan_2 object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return the computed trajectory
    res.computed_trajectory = this->computed_trajectory;
    res.answer = true;
    return true;
}


// Initialize the things for motion planning. Is called by the callback
bool JointPlan_2::initialize(panda_softhand_control::joint_plan::Request &req){

    // Converting the float array of request to std vector
    this->joint_goal = req.joint_goal;
    this->joint_now = req.joint_start;  
    
    // Print the goal end-effector pose
    if(DEBUG){
        std::cout << "Requested joint configuration goal: [ ";
        for(auto i : this->joint_goal) std::cout << i << " ";
        std::cout << "]" << std::endl;
    }

    return true;
}

// Performs motion planning for the joints towards goal
bool JointPlan_2::performMotionPlan(){

    // Move group interface
    moveit::planning_interface::MoveGroupInterface group(this->group_name);
    
    // Getting current joint state
    ros::spinOnce();                                    // May not be necessary
    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(this->group_name);

    if (this->is_really_null_config(this->joint_now)) {
        ROS_WARN("The start pose is NULL! PLANNING FROM CURRENT POSE!");
        current_state->copyJointGroupPositions(joint_model_group, this->joint_now);
    }

    // Checking if the dimensions of the request are correct
    if(this->joint_now.size() != this->joint_goal.size()){
        ROS_ERROR("The size of the requested joint configuration is not correct!");
        return false;
    }

    /* If VISUAL is enabled */
    #ifdef VISUAL

    // Visual tools
    moveit_visual_tools::MoveItVisualTools visual_tools("world");
    visual_tools.deleteAllMarkers();

    // Loading the remote control for visual tools and promting a message
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    #endif

	// Printing the planning group frame and the group ee frame
	if(DEBUG) ROS_INFO("MoveIt Group Reference frame: %s", group.getPlanningFrame().c_str());
	if(DEBUG) ROS_INFO("MoveIt Group End-effector frame: %s", group.getEndEffectorLink().c_str());

    // Setting the joint config target of the move group
    group.setJointValueTarget(this->joint_goal);

    // Planning to joint configuration
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Motion Plan towards goal joint configuration %s.", success ? "SUCCEDED" : "FAILED");

    // If complete path is not achieved return false, true otherwise
	if(!success) return false;

    /* If VISUAL is enabled */
    #ifdef VISUAL

    ROS_INFO("Visualizing the computed plan as trajectory line.");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    
    #ifdef PROMPT
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to execute the motion on the robot.");
    #endif

    #endif

    // Saving the computed trajectory and returning true
    this->computed_trajectory = my_plan.trajectory_.joint_trajectory;
    return true;
}