/* PANDA SOFTHAND CLIENT - Contains all necessary objects and functions to call the services to 
control Panda + SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "panda_softhand_control/PandaSoftHandClient_1.h"
#include <std_msgs/Duration.h>

PandaSoftHandClient_1::PandaSoftHandClient_1(){

    // Nothing to do here
}

PandaSoftHandClient_1::PandaSoftHandClient_1(ros::NodeHandle& nh_){

    // Initializing object
    if(!this->initialize(nh_)){
        ROS_ERROR("The PandaSoftHandClient_1 was not initialized successfully. Some servers are missing...");
        ros::shutdown();
    } 
}

PandaSoftHandClient_1::~PandaSoftHandClient_1(){
    
    // Nothing to do here yet
}

// Initializing function
bool PandaSoftHandClient_1::initialize(ros::NodeHandle& nh_){

    // Initializing node handle
    this->nh = nh_;

    // Initializing the service names (TODO: Change the hard coded names of the services and parse them from same yaml file of main_server)
    this->hand_plan_service_name = "hand_plan_service";
    this->hand_control_service_name = "hand_control_service";
    this->hand_wait_service_name = "hand_wait_service";

    this->arm_control_service_name = "arm_control_service";
    this->arm_wait_service_name = "arm_wait_service";

    this->joint_service_name = "joint_plan_service";
    this->pose_service_name = "pose_plan_service";
    this->slerp_service_name = "slerp_plan_service";

    // Initializing service clients after waiting
    if(!ros::service::waitForService(this->hand_plan_service_name, ros::Duration(1.0))) return false;
    this->hand_plan_client = this->nh.serviceClient<panda_softhand_control::hand_plan>(this->hand_plan_service_name);

    if(!ros::service::waitForService(this->hand_control_service_name, ros::Duration(1.0))) return false;
    this->hand_control_client = this->nh.serviceClient<panda_softhand_control::hand_control>(this->hand_control_service_name);

    if(!ros::service::waitForService(this->hand_wait_service_name, ros::Duration(1.0))) return false;
    this->hand_wait_client = this->nh.serviceClient<panda_softhand_control::hand_wait>(this->hand_wait_service_name);

    if(!ros::service::waitForService(this->arm_control_service_name, ros::Duration(1.0))) return false;
    this->arm_control_client = this->nh.serviceClient<panda_softhand_control::arm_control>(this->arm_control_service_name);

    if(!ros::service::waitForService(this->arm_wait_service_name, ros::Duration(1.0))) return false;
    this->arm_wait_client = this->nh.serviceClient<panda_softhand_control::arm_wait>(this->arm_wait_service_name);

    if(!ros::service::waitForService(this->joint_service_name, ros::Duration(1.0))) return false;
    this->joint_client = this->nh.serviceClient<panda_softhand_control::joint_plan>(this->joint_service_name);

    if(!ros::service::waitForService(this->pose_service_name, ros::Duration(1.0))) return false;
    this->pose_client = this->nh.serviceClient<panda_softhand_control::pose_plan>(this->pose_service_name);

    if(!ros::service::waitForService(this->slerp_service_name, ros::Duration(1.0))) return false;
    this->slerp_client = this->nh.serviceClient<panda_softhand_control::slerp_plan>(this->slerp_service_name);

    // At this point initializing completed
    return true;
}

// Service call function for hand plan
bool PandaSoftHandClient_1::call_hand_plan_service(double goal_syn, double goal_duration, trajectory_msgs::JointTrajectory& computed_trajectory){

    // Creating and filling up the request
    panda_softhand_control::hand_plan hand_plan_srv;
    hand_plan_srv.request.goal_syn = goal_syn;
    hand_plan_srv.request.goal_duration = goal_duration;

    // Calling the service
    if(!this->hand_plan_client.call(hand_plan_srv)){
        ROS_ERROR("Failed to contact the hand plan server. Returning...");
        return false;
    }

    computed_trajectory = hand_plan_srv.response.computed_trajectory;
    return hand_plan_srv.response.answer;
}

// Service call function for hand control
bool PandaSoftHandClient_1::call_hand_control_service(trajectory_msgs::JointTrajectory& computed_trajectory){

    // Creating and filling up the request
    panda_softhand_control::hand_control hand_control_srv;
    hand_control_srv.request.computed_trajectory = computed_trajectory;

    // Calling the service
    if(!this->hand_control_client.call(hand_control_srv)){
        ROS_ERROR("Failed to contact the hand control server. Returning...");
        return false;
    }

    return hand_control_srv.response.answer;
}

// Service call function for hand wait
bool PandaSoftHandClient_1::call_hand_wait_service(ros::Duration wait_time){

    // Creating and filling up the request
    panda_softhand_control::hand_wait hand_wait_srv;
    std_msgs::Duration wait_duration_msg;
    wait_duration_msg.data = wait_time;
    hand_wait_srv.request.wait_duration = wait_duration_msg;

    // Calling the service
    if(!this->hand_wait_client.call(hand_wait_srv)){
        ROS_ERROR("Failed to contact the hand wait server. Returning...");
        return false;
    }

    return hand_wait_srv.response.answer;
}

// Service call function for arm control
bool PandaSoftHandClient_1::call_arm_control_service(trajectory_msgs::JointTrajectory& computed_trajectory){

    // Creating and filling up the request
    panda_softhand_control::arm_control arm_control_srv;
    arm_control_srv.request.computed_trajectory = computed_trajectory;

    // Calling the service
    if(!this->arm_control_client.call(arm_control_srv)){
        ROS_ERROR("Failed to contact the arm control server. Returning...");
        return false;
    }

    return arm_control_srv.response.answer;
}

// Service call function for arm wait
bool PandaSoftHandClient_1::call_arm_wait_service(ros::Duration wait_time){

    // Creating and filling up the request
    panda_softhand_control::arm_wait arm_wait_srv;
    std_msgs::Duration wait_duration_msg;
    wait_duration_msg.data = wait_time;
    arm_wait_srv.request.wait_duration = wait_duration_msg;

    // Calling the service
    if(!this->arm_wait_client.call(arm_wait_srv)){
        ROS_ERROR("Failed to contact the arm wait server. Returning...");
        return false;
    }

    return arm_wait_srv.response.answer;
}

// Service call function for joint plan
bool PandaSoftHandClient_1::call_joint_service(std::vector<double> joint_goal, std::vector<double> joint_start, trajectory_msgs::JointTrajectory& computed_trajectory){

    // Creating and filling up the request
    panda_softhand_control::joint_plan joint_plan_srv;
    joint_plan_srv.request.joint_goal = joint_goal;
    joint_plan_srv.request.joint_start = joint_start;

    // Calling the service
    if(!this->joint_client.call(joint_plan_srv)){
        ROS_ERROR("Failed to contact the joint plan server. Returning...");
        return false;
    }

    computed_trajectory = joint_plan_srv.response.computed_trajectory;
    return joint_plan_srv.response.answer;
}

// Service call function for pose plan
bool PandaSoftHandClient_1::call_pose_service(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, 
                                            trajectory_msgs::JointTrajectory& computed_trajectory, trajectory_msgs::JointTrajectory past_trajectory){

    // Creating and filling up the request
    panda_softhand_control::pose_plan pose_plan_srv;
    pose_plan_srv.request.goal_pose = goal_pose;
    pose_plan_srv.request.start_pose = start_pose;
    pose_plan_srv.request.is_goal_relative = is_goal_relative;
    pose_plan_srv.request.past_trajectory = past_trajectory;

    // Calling the service
    if(!this->pose_client.call(pose_plan_srv)){
        ROS_ERROR("Failed to contact the pose plan server. Returning...");
        return false;
    }

    computed_trajectory = pose_plan_srv.response.computed_trajectory;
    return pose_plan_srv.response.answer;
}

// Service call function for slerp plan
bool PandaSoftHandClient_1::call_slerp_service(geometry_msgs::Pose goal_pose, geometry_msgs::Pose start_pose, bool is_goal_relative, 
                                            trajectory_msgs::JointTrajectory& computed_trajectory, trajectory_msgs::JointTrajectory past_trajectory){

    // Creating and filling up the request
    panda_softhand_control::slerp_plan slerp_plan_srv;
    slerp_plan_srv.request.goal_pose = goal_pose;
    slerp_plan_srv.request.start_pose = start_pose;
    slerp_plan_srv.request.is_goal_relative = is_goal_relative;
    slerp_plan_srv.request.past_trajectory = past_trajectory;

    // Calling the service
    if(!this->slerp_client.call(slerp_plan_srv)){
        ROS_ERROR("Failed to contact the slerp plan server. Returning...");
        return false;
    }

    computed_trajectory = slerp_plan_srv.response.computed_trajectory;
    return slerp_plan_srv.response.answer;
}