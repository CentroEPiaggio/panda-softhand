
/*  
    JOINT LIMITS EVADER CLASS
    This object checks for joints' position, velocity, acceleration of a robot listening from /joint_states
    and getting the limits from the urdf.
*/

#include "panda_softhand_safety/joint_limits_evader.h"

// Includes for this cpp
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#define     DEBUG_JLE        0       // Prints out additional info

using namespace panda_softhand_safety;

JointLimitsEvader::JointLimitsEvader(ros::NodeHandle& nh){

    this->jle_nh_ = nh;

    // Initializing ROS elements
    this->sub_joint_states_ = this->jle_nh_.subscribe<sensor_msgs::JointState>(this->joints_topic_, 1, &JointLimitsEvader::joints_callback, this);
    this->first_joints_ = ros::topic::waitForMessage<sensor_msgs::JointState>(this->joints_topic_, ros::Duration(2.0));
    if (this->first_joints_ == NULL) {
        ROS_ERROR("Cannot contact the joint states topic! This is not safe anymore.");
    }
    this->current_joints_ = *this->first_joints_;

    // Getting necessary parameters for initializing
    if (!this->parse_parameters(this->jle_nh_)) {
        ROS_ERROR("Could not parse the needed parameters! Using default ones.");
    }

    // Initializing the needed elements
    if (!this->initialize()) {
        ROS_FATAL("Failed to initialize some of the variables for collision check! This is not safe anymore.");
    }

}

JointLimitsEvader::~JointLimitsEvader(){

    // Nothing to do here yet

}



// Callback to joint_states topic
void JointLimitsEvader::joints_callback(const sensor_msgs::JointState::ConstPtr &msg){
    
    // Saving the joints message
    this->current_joints_ = *msg;

}

// Function to parse parameters
bool JointLimitsEvader::parse_parameters(ros::NodeHandle& nh){

    bool success = true;

    if(!ros::param::get("/panda_softhand_safety/acceleration_limit", this->acceleration_limit_)){
		ROS_WARN("The param 'acceleration_limit' not found in param server! Using default.");
		this->acceleration_limit_ = 20;
		success = false;
	}

    return success;
    
}

// Function to initialize all main variables
bool JointLimitsEvader::initialize(){

    // Get urdf from the parameter server
    std::string robot_description;
    std::string root_name;
    std::string tip_name;
    if (!ros::param::search(this->jle_nh_.getNamespace(),"robot_description", robot_description)) {
        ROS_ERROR_STREAM("JointLimitsEvader: No robot description (urdf) found on parameter server (" << this->jle_nh_.getNamespace() << "/robot_description)");
        return false;
    }
    if (!this->jle_nh_.getParam("root_name", root_name)) {
        ROS_ERROR_STREAM("JointLimitsEvader: No root name found on parameter server (" << this->jle_nh_.getNamespace() << "/root_name)");
        return false;
    }
    if (!this->jle_nh_.getParam("tip_name", tip_name))
    {
        ROS_ERROR_STREAM("JointLimitsEvader: No tip name found on parameter server (" << this->jle_nh_.getNamespace() << "/tip_name)");
        return false;
    }

    // Construct urdf model from parsed xml string
    std::string xml_string;
    if (this->jle_nh_.hasParam(robot_description)) {
        this->jle_nh_.getParam(robot_description.c_str(), xml_string);
    } else {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
        this->jle_nh_.shutdown();
        return false;
    }

    if (xml_string.size() == 0) {
        ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
        this->jle_nh_.shutdown();
        return false;
    }

    ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());
    
    // Get urdf model from robot_description
    urdf::Model model;
    if (!model.initString(xml_string)) {
        ROS_ERROR("Failed to parse urdf file");
        this->jle_nh_.shutdown();
        return false;
    }
    ROS_INFO("Successfully parsed urdf file");
    
    // Create a kdl tree from the urdf model
    KDL::Tree kdl_tree_;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_)) {
        ROS_ERROR("Failed to construct kdl tree");
        this->jle_nh_.shutdown();
        return false;
    }

    // Populate the kdl chain
    if (!kdl_tree_.getChain(root_name, tip_name, this->kdl_chain_)) {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
        ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
        ROS_ERROR_STREAM("  The segments are:");
        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;
        for ( it=segment_map.begin(); it != segment_map.end(); it++ ) ROS_ERROR_STREAM( "    " << (*it).first);
        return false;
    }

    ROS_DEBUG("Number of segments: %d", this->kdl_chain_.getNrOfSegments());
    ROS_DEBUG("Number of joints in chain: %d", this->kdl_chain_.getNrOfJoints());
    
    // Parsing joint limits from urdf model along kdl chain
    boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint_;
    this->joint_limits_.pos_min.resize(this->kdl_chain_.getNrOfJoints());
    this->joint_limits_.pos_max.resize(this->kdl_chain_.getNrOfJoints());
    this->joint_limits_.pos_center.resize(this->kdl_chain_.getNrOfJoints());
    this->joint_limits_.vel_max.resize(this->kdl_chain_.getNrOfJoints());
    this->joint_limits_.acc_max.resize(this->kdl_chain_.getNrOfJoints());

    // Getting the position and velocity limits (the acceleration limits are now all the same)
    int index;
    for (int i = 0; i < this->kdl_chain_.getNrOfJoints() && link_; i++) {
        joint_ = model.getJoint(link_->parent_joint->name);  
        ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());
        index = this->kdl_chain_.getNrOfJoints() - i - 1;
        this->joint_limits_.pos_min(index) = joint_->limits->lower;
        this->joint_limits_.pos_max(index) = joint_->limits->upper;
        this->joint_limits_.pos_center(index) = (this->joint_limits_.pos_min(index) + this->joint_limits_.pos_max(index))/2;
        this->joint_limits_.vel_max(index) = joint_->limits->velocity;
        this->joint_limits_.acc_max(index) = this->acceleration_limit_;
        link_ = model.getLink(link_->getParent()->name);
    }

    // TODO : get the correct acceleration limits from a yaml file (from Franka Documentation)

    return true;
}