
/*  
    COLLISION EVADER CLASS
    This object checks for collisions of a robot with the environment listening from the planning scene.
*/

#include "panda_softhand_safety/collision_evader.h"

#define     DEBUG_CE        1       // Prints out additional info

using namespace panda_softhand_safety;

CollisionEvader::CollisionEvader(ros::NodeHandle& nh){

    this->ce_nh_ = nh;

    // Initializing ROS elements
    this->sub_joint_states_ = this->ce_nh_.subscribe<sensor_msgs::JointState>(this->joints_topic_, 1, &CollisionEvader::joints_callback, this);
    this->first_joints_ = ros::topic::waitForMessage<sensor_msgs::JointState>(this->joints_topic_, ros::Duration(2.0));
    if (this->first_joints_ == NULL) {
        ROS_ERROR("Cannot contact the joint states topic! This is not safe anymore.");
    }
    this->current_joints_ = *this->first_joints_;

    // Getting necessary parameters for initializing
    if (!this->parse_parameters(this->ce_nh_)) {
        ROS_ERROR("Could not parse the needed parameters! Using default ones.");
    }

    // Initializing the needed elements
    if (!this->initialize()) {
        ROS_FATAL("Failed to initialize some of the variables for collision check! This is not safe anymore.");
    }

}

CollisionEvader::~CollisionEvader(){

    // Nothing to do here yet

}

// Function that spins to check for collisions
bool CollisionEvader::CheckCollision(){

    // Spinning to get the latest joints
    ros::spinOnce();

    // Setting the joints in robot state
    for (int i = 0; i < 7; i++){
        this->current_state_->setJointPositions(this->current_joints_.name[i], &this->current_joints_.position[i]);
    }

    // Set the current robot state to planning scene
    this->planning_scene_->setCurrentState(*this->current_state_);

    // Get collision objects in scene
    this->collision_objects_map_ = this->planning_scene_interface_.getObjects();
    if (DEBUG_CE) ROS_INFO("Checking for collision objects in the planning scene...");
    for (auto it : this->collision_objects_map_) {
        this->planning_scene_->processCollisionObjectMsg(it.second);
        if (DEBUG_CE) {
            std::cout << "Found collision object " << it.first << " with id " << it.second.id << "." << std::endl;
        }
    }

    // Checking for collision
    this->collision_result_.clear();
    this->planning_scene_->checkCollision(this->collision_request_, this->collision_result_, *this->current_state_);

    this->distance_to_env_ = this->planning_scene_->distanceToCollision(*this->current_state_);
    this->distance_to_self_ = this->collision_result_.distance;

    // Print distance to collision
    if (DEBUG_CE) {
        ROS_INFO_STREAM("Distance to world collision: " << this->distance_to_env_);
        ROS_INFO_STREAM("Distance to self collision: " << this->distance_to_self_);
    }

    // TODO: Use only checkCollision() for both self and world collisions

    // Now we know if collision
    this->collision_found_ = 
        (this->distance_to_env_ < this->collision_threshold_ || this->distance_to_self_ < this->collision_threshold_);
    
    return this->collision_found_;
    
}

// Callback to joint_states topic
void CollisionEvader::joints_callback(const sensor_msgs::JointState::ConstPtr &msg){
    
    // Saving the joints message
    this->current_joints_ = *msg;

}

// Function to parse parameters
bool CollisionEvader::parse_parameters(ros::NodeHandle& nh){

    bool success = true;

    if(!ros::param::get("/panda_softhand_safety/group_name", this->group_name_)){
		ROS_WARN("The param 'group_name' not found in param server! Using default.");
		this->group_name_ = "panda_arm";
		success = false;
	}

    if(!ros::param::get("/panda_softhand_safety/collision_threshold", this->collision_threshold_)){
		ROS_WARN("The param 'collision_threshold' not found in param server! Using default.");
		this->collision_threshold_ = 0.01;
		success = false;
	}

    return success;
    
}

// Function to initialize all main variables
bool CollisionEvader::initialize(){

    // Initializing MoveIt variables
    this->group_ptr_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->group_name_);
    this->robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");
    this->kinematic_model_ = this->robot_model_loader_->getModel();
    this->planning_scene_ = std::make_shared<planning_scene::PlanningScene>(this->kinematic_model_);
    this->current_state_ = std::make_shared<robot_state::RobotState>(this->planning_scene_->getCurrentStateNonConst());

    // Filling up part of collision request
    this->collision_request_.group_name = this->group_name_;
    this->collision_request_.contacts = true;
    this->collision_request_.distance = true;
    this->collision_request_.max_contacts = 100;
    this->collision_request_.max_contacts_per_pair = 5;
    this->collision_request_.verbose = false;

    return true;
}