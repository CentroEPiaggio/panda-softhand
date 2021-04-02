/* POSE CONTROL - Uses moveit movegroupinterface to plan towards a pose
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "ros/ros.h"
#include <cstdlib>
#include <sstream>
#include <string>

#include "panda_softhand_control/PosePlan_1.h"

#include <moveit_visual_tools/moveit_visual_tools.h>

PosePlan_1::PosePlan_1(ros::NodeHandle& nh_, std::string group_name_, std::string end_effector_name_){
        
        ROS_INFO("Starting to create PosePlan_1 object");

        // Initializing node handle
        this->nh = nh_;

        // Initializing names
        this->end_effector_name = end_effector_name_;
        this->group_name = group_name_;

        ROS_INFO("Finished creating PosePlan_1 object");
}

PosePlan_1::~PosePlan_1(){
    
    // Nothing to do here yet
}

// This is the callback function of the pose plan service
bool PosePlan_1::call_pose_plan(panda_softhand_control::pose_plan::Request &req, panda_softhand_control::pose_plan::Response &res){

    // Setting up things
    if(!this->initialize(req.goal_pose, req.start_pose, req.is_goal_relative, req.past_trajectory)){
        ROS_ERROR("Could not initialize PosePlan_1 object. Returning...");
        res.answer = false;
        return false;
    }

	// Perform motion plan towards the goal pose
    if(!this->performMotionPlan()){
        ROS_ERROR("Could not perform motion planning in PosePlan_1 object. Returning...");
        res.answer = false;
        return false;
    }

    // At this point all is fine, return the computed trajectory
    res.computed_trajectory = this->computed_trajectory;
    res.answer = true;
    return true;
}


// Initialize the things for motion planning. Is called by the callback
bool PosePlan_1::initialize(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, trajectory_msgs::JointTrajectory past_trajectory){

    // Getting the current ee transform
    try {
		this->tf_listener.waitForTransform("/world", this->end_effector_name, ros::Time(0), ros::Duration(10.0) );
		this->tf_listener.lookupTransform("/world", this->end_effector_name, ros::Time(0), this->stamp_ee_transform);
    } catch (tf::TransformException ex){
      	ROS_ERROR("%s", ex.what());
      	ros::Duration(1.0).sleep();
        return false;
    }

    tf::Transform ee_transform(this->stamp_ee_transform.getRotation(), this->stamp_ee_transform.getOrigin());
    tf::transformTFToEigen(ee_transform, this->end_effector_state);

	// Print the current end-effector pose
	if(DEBUG) ROS_INFO_STREAM("Endeffector current Translation: \n" << this->end_effector_state.translation());
	if(DEBUG) ROS_INFO_STREAM("Endeffector current Rotation: \n" << this->end_effector_state.rotation());

    // Setting the start pose
    if (this->is_really_null_pose(start_pose)){
        ROS_WARN("The start pose is NULL! PLANNING FROM CURRENT POSE!");
        this->startAff = this->end_effector_state;
        this->was_really_null = true;
    } else {
        tf::poseMsgToEigen(start_pose, this->startAff);
        this->was_really_null = false;
    }

	// Setting the goal pose
    tf::poseMsgToEigen(goal_pose, this->goalPoseAff);

	// If the goal is relative, get the global goal pose by multiplying it with ee pose (end_effector_state)
	if(is_goal_relative){
		this->goalPoseAff = this->end_effector_state * this->goalPoseAff;
        this->startAff = this->end_effector_state * this->startAff;
	}

    // Reconvert to geometry_msgs Pose
    tf::poseEigenToMsg(this->goalPoseAff, this->goalPose);

    // Setting the past trajectory
    this->past_trajectory = past_trajectory;

    // Print the goal end-effector pose
    if(DEBUG) ROS_INFO_STREAM("Endeffector goal Translation: \n" << this->goalPoseAff.translation());
	if(DEBUG) ROS_INFO_STREAM("Endeffector goal Rotation: \n" << this->goalPoseAff.linear());

    return true;
}

// Performs motion planning for the end-effector towards goal
bool PosePlan_1::performMotionPlan(){

    // Move group interface
    moveit::planning_interface::MoveGroupInterface group(this->group_name);

    // Getting the robot joint model
    ros::spinOnce();                    // May not be necessary
    const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup(this->group_name);

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

    // Setting the pose target of the move group
    group.setPoseTarget(this->goalPose);

    if(DEBUG) ROS_INFO("Done setting the target pose in MoveIt Group.");

    // Setting the start state in the moveit group
    robot_state::RobotState start_state(*group.getCurrentState());
    if (!this->was_really_null) {
        std::vector<double> last_joints = this->past_trajectory.points.back().positions;
        // geometry_msgs::Pose starting_pose;
        // tf::poseEigenToMsg(this->startAff, starting_pose);
        start_state.setJointGroupPositions(joint_model_group, last_joints);
        group.setStartState(start_state);
    }

    // Planning to Pose
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Motion Plan towards goal pose %s.", success ? "SUCCEDED" : "FAILED");

    #ifdef VISUAL

    ROS_INFO("Visualizing the Goal.");
    visual_tools.publishAxisLabeled(this->goalPose, "goal pose");
    visual_tools.trigger();

    #endif

    // If complete path is not achieved return false, true otherwise
	if(!success) return false;

    /* If VISUAL is enabled */
    #ifdef VISUAL

    ROS_INFO("Visualizing the computed plan as trajectory line.");
    visual_tools.publishAxisLabeled(this->goalPose, "goal pose");
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
