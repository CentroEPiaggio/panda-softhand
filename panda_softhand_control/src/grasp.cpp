#include "ros/ros.h"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>


// ROS Service and Message Includes
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/SetBool.h"
#include "panda_softhand_control/set_object.h"
#include "panda_softhand_control/complex_grasp.h"
#include "geometry_msgs/Pose.h"
#include <controller_manager_msgs/SwitchController.h>
#include <franka_msgs/FrankaState.h>
#include <franka_msgs/ErrorRecoveryActionGoal.h>

// Parsing includes
#include <XmlRpcValue.h>
#include <XmlRpcException.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// Custom Includes
#include "panda_softhand_control/PandaSoftHandClient.h"


// Other Includes

// Defines
#define     DEBUG   1       // Prints out additional stuff
#define     VISUAL          // Publishes visual info on RViz

// Subscriber to object pose and the pose
    ros::Subscriber object_sub;
    geometry_msgs::Pose object_pose_T;

 // Subscriber to franka_states for getting tau_ext on joints and other info and Publisher of its norm
    ros::Subscriber franka_state_sub;
    franka_msgs::FrankaState latest_franka_state;
    bool franka_ok = true;
    double tau_ext_norm = 0.0;
    ros::Publisher pub_tau_ext_norm;
    ros::Publisher pub_franka_recovery;         // TODO: Recover from error automatically
    
// The Panda SoftHand Client
    PandaSoftHandClient panda_softhand_client;
   
// Service names
    std::string franka_state_topic_name = "/franka_state_controller/franka_states";
    std::string grasp_task_service_name;
       
 // Service Servers
    ros::ServiceServer grasp_task_server;
        
// Parsed task sequence variables
    std::string robot_name;                     // Name of the robot (namespace)
    std::string robot_joints_name;              // Name of the robot joints (without the number of the joints)
    std::string pos_controller;                 // Name of position controller
    std::string imp_controller;                 // Name of impedance controller
    std::string object_topic_name;              // Name of the topic where the object pose is published
    std::vector<double> home_joints;
    std::vector<double> grasp_transform;
    geometry_msgs::Pose grasp_T;
    std::vector<double> pre_grasp_transform;
    geometry_msgs::Pose pre_grasp_T;
    //std::vector<double> place_joints;
    //std::vector<double> handover_joints;
    double handover_thresh;

// MoveIt stuff and functions for FK and IK
    std::string group_name;
    std::string end_effector_name;
    std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_ptr;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

// Other execution vars
    ros::Duration waiting_time;
    trajectory_msgs::JointTrajectory tmp_traj;

    
// Convert xyzrpy vector to geometry_msgs Pose
    geometry_msgs::Pose convert_vector_to_pose(std::vector<double> input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
    output_affine.linear() = rotation;    
    
    // Converting to geometry_msgs and returning
    tf::poseEigenToMsg(output_affine, output_pose);
    return output_pose;
}

// Parameters parsing
bool parse_task_params(){
    bool success = true;

    if(!ros::param::get("/task_sequencer/robot_name", robot_name)){
		ROS_WARN("The param 'robot_name' not found in param server! Using default.");
		robot_name = "panda_arm";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/robot_joints_name", robot_joints_name)){
		ROS_WARN("The param 'robot_joints_name' not found in param server! Using default.");
		robot_joints_name = "panda_joint";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/pos_controller", pos_controller)){
		ROS_WARN("The param 'pos_controller' not found in param server! Using default.");
		pos_controller = "position_joint_trajectory_controller";
		success = false;
	}


    if(!ros::param::get("/task_sequencer/object_topic_name", object_topic_name)){
		ROS_WARN("The param 'object_topic_name' not found in param server! Using default.");
		object_topic_name = "/irim_demo/chosen_object";
		success = false;
	}

	if(!ros::param::get("/task_sequencer/home_joints", home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		home_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/grasp_transform", grasp_transform)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		grasp_transform.resize(6);
        std::fill(grasp_transform.begin(), grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    grasp_T = convert_vector_to_pose(grasp_transform);

    if(!ros::param::get("/task_sequencer/pre_grasp_transform", pre_grasp_transform)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		pre_grasp_transform.resize(6);
        std::fill(pre_grasp_transform.begin(), pre_grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    pre_grasp_T = convert_vector_to_pose(pre_grasp_transform);

    /*if(!ros::param::get("/task_sequencer/handover_joints", handover_joints)){
		ROS_WARN("The param 'handover_joints' not found in param server! Using default.");
		handover_joints = {-0.101, 0.161, 0.159, -1.651, 2.023, 2.419, -0.006};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/place_joints", place_joints)){
		ROS_WARN("The param 'place_joints' not found in param server! Using default.");
		place_joints = {-0.136, 0.794, -0.115, -1.337, 0.250, 2.217, -0.479};
		success = false;
	}*/
    return success;
}
 

// Callback for object pose subscriber
void get_object_pose(const geometry_msgs::Pose::ConstPtr &msg){
    // Saving the message
    object_pose_T = *msg;
}

// Callback for franka state subscriber
void get_franka_state(const franka_msgs::FrankaState::ConstPtr &msg){
    // Saving the message
    latest_franka_state = *msg;

    // Checking for libfranka errors
    if(msg->robot_mode != 2 && msg->robot_mode != 5){       // The robot state is not "automatic" or "manual guiding"
        franka_ok = false;
        if(DEBUG && false) ROS_ERROR("Something happened to the robot!");
    }else if(msg->robot_mode == 2){
        franka_ok = true;
        if(DEBUG && false) ROS_WARN("Now Franka is in a good mood!");
    }
    
}


// FK and IK Functions which makes use of MoveIt
geometry_msgs::Pose performFK(std::vector<double> joints_in){
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
    kinematic_state->setJointGroupPositions(joint_model_group, joints_in);
    const Eigen::Affine3d& end_effector_eigen = kinematic_state->getGlobalLinkTransform(end_effector_name);
    geometry_msgs::Pose end_effector_pose;
    tf::poseEigenToMsg(end_effector_eigen, end_effector_pose);
    return end_effector_pose;
}

bool performIK(geometry_msgs::Pose pose_in, double timeout, std::vector<double>& joints_out){
    Eigen::Isometry3d end_effector_state;
    tf::poseMsgToEigen(pose_in, end_effector_state);
    const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if (!found_ik){
        ROS_ERROR("Could not find IK solution in TaskSequencer...");
        return false;
    }

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);
    kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);

    if (DEBUG){
        ROS_INFO("Found an IK solution in TaskSequencer: ");
        for (std::size_t i = 0; i < joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints_out[i]);
        }
    }

    return true;
}


// Callback for simple grasp task service

bool call_simple_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_grasp_task done correctly with false request!";
        return true;
    }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(object_pose_T, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff, grasp_pose);

    // Couting object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;

    // Setting zero pose as starting from present
    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;

    // 2) Going to pregrasp pose
    if(!panda_softhand_client.call_pose_service(pre_grasp_pose, present_pose, false, tmp_traj, tmp_traj) || !franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!panda_softhand_client.call_arm_control_service(tmp_traj) || !franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    // 3) Going to grasp pose
    if(!panda_softhand_client.call_slerp_service(grasp_pose, pre_grasp_pose, false, tmp_traj, tmp_traj) || !franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!panda_softhand_client.call_arm_wait_service(waiting_time) || !franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    if(!panda_softhand_client.call_arm_control_service(tmp_traj) || !franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    // 4) Performing simple grasp with planning, executing and waiting

    if(!panda_softhand_client.call_hand_plan_service(0.8, 2.0, tmp_traj) || !franka_ok){
        ROS_ERROR("Could not plan the simple grasp.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    if(!panda_softhand_client.call_arm_wait_service(waiting_time) || !franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    if(!panda_softhand_client.call_hand_control_service(tmp_traj) || !franka_ok){
        ROS_ERROR("Could not perform the grasping.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    // 5) Returning to pre grasp pose
    if(!panda_softhand_client.call_slerp_service(pre_grasp_pose, grasp_pose, false, tmp_traj, tmp_traj) || !franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!panda_softhand_client.call_arm_wait_service(waiting_time) || !franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    if(!panda_softhand_client.call_arm_control_service(tmp_traj) || !franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }


    // 6) Performing simple grasp with planning, executing and waiting

    if(!panda_softhand_client.call_hand_plan_service(0, 2.0, tmp_traj) || !franka_ok){
        ROS_ERROR("Could not plan the simple grasp.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    if(!panda_softhand_client.call_arm_wait_service(waiting_time) || !franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;

    }
//NUOVO
    if(!panda_softhand_client.call_hand_wait_service(ros::Duration(3.0)) || !franka_ok){
        ROS_ERROR("Could not perform the simple grasp.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand wait.";
        return false;
    }

    if(!panda_softhand_client.call_hand_control_service(tmp_traj) || !franka_ok){
        ROS_ERROR("Could not perform the grasping.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }


    // Getting current joints
    std::vector<double> now_joints;
    double timeout = 3.0;
    bool found_ik_now = performIK(grasp_pose, timeout, now_joints);
    if (!found_ik_now){
        ROS_ERROR("Could not perform IK.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    // 7) Lifting the grasped? object
    if(!panda_softhand_client.call_joint_service(home_joints, now_joints, tmp_traj) || !franka_ok){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }


    if(!panda_softhand_client.call_arm_control_service(tmp_traj) || !franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    if(!panda_softhand_client.call_arm_wait_service(waiting_time) || !franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }


// Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_grasp_task was correctly performed!";
    return true;
}



/**********************************************
ROS NODE MAIN GRASP
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_softhand_grasp");

    ros::NodeHandle nh;

     if(!parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Be careful, using default values...");
    }

 // Initializing the object subscriber and waiting (the object topic name is parsed now)
    object_sub = nh.subscribe(object_topic_name, 1, &get_object_pose);
    ros::topic::waitForMessage<geometry_msgs::Pose>(object_topic_name, ros::Duration(2.0));

// Initializing the franka_state_sub subscriber and waiting
    franka_state_sub = nh.subscribe("/" + robot_name + franka_state_topic_name, 1, &get_franka_state);
    ros::topic::waitForMessage<franka_msgs::FrankaState>("/" + robot_name + franka_state_topic_name, ros::Duration(2.0));

// Initializing the tau_ext norm and franka recovery publishers
    pub_franka_recovery = nh.advertise<franka_msgs::ErrorRecoveryActionGoal>("/" + robot_name + "/franka_control/error_recovery/goal", 1);
    pub_tau_ext_norm = nh.advertise<std_msgs::Float64>("tau_ext_norm", 1);

// Initializing Panda SoftHand Client (TODO: Return error if initialize returns false)
    panda_softhand_client.initialize(nh);

// Moveit names
    group_name = "panda_arm";
    end_effector_name = "right_hand_ee_link";

// Initializing other moveit stuff (robot model, kinematic model and state)
    robot_model_loader_ptr.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    kinematic_model = robot_model_loader_ptr->getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    kinematic_state.reset(new robot_state::RobotState(kinematic_model));

// Setting the task service names
    grasp_task_service_name = "grasp_task_service";
 
// Advertising the services
    grasp_task_server = nh.advertiseService(grasp_task_service_name, &call_simple_grasp_task);
 

// Initializing other control values
    waiting_time = ros::Duration(5.0);
    
    // tmp_traj.points.push_back(empty_joints_point);

// ROS Async spinnedr (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while(ros::ok()){
        // Nothing to do here
    }

    spinner.stop();

    return 0;
}