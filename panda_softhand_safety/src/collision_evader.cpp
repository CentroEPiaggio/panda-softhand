
/*  
    COLLISION EVADER CLASS
    This object checks for collisions of a robot with the environment listening from the planning scene.
*/

#include "panda_softhand_safety/collision_evader.h"

using namespace panda_softhand_safety;

CollisionEvader::CollisionEvader(ros::NodeHandle& nh){

    this->ce_nh_ = nh;

    // Getting necessary parameters for initializing
    if(!this->parse_parameters(this->ce_nh_)){
        ROS_ERROR("Could not parse the needed parameters! Using default ones.");
    }

    // Initializing the needed elements
    if(!this->initialize()){
        ROS_FATAL("Failed to initialize some of the variables for collision check! This is not safe anymore.");
    }

}

CollisionEvader::~CollisionEvader(){

    // Nothing to do here yet

}

// Function that spins to check for collisions
bool CollisionEvader::EvadeCollision(){
    
}

// Callback to joint_states topic
void CollisionEvader::callback_joint_states(sensor_msgs::JointStateConstPtr msg){
    
}

// Function to parse parameters
bool CollisionEvader::parse_parameters(ros::NodeHandle& nh){
    
}

// Function to initialize all main variables
bool CollisionEvader::initialize(){

}