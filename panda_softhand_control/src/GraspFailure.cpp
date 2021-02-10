#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "utils/parsing_utilities.h"
#include "panda_softhand_control/GraspFailure.h"


GraspFailure::GraspFailure(ros::NodeHandle& nh_){
    
    // Initializing Node Handle
    this->nh = nh_;

    // Parsing the task params
    if(!this->parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Be careful, using default values...");
    }


    // Initializing the object subscriber and waiting (the object topic name is parsed now)
    ros::Subscriber object_sub = this->nh.subscribe(this->object_topic_name, 1, &GraspFailure::get_object_pose, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>(this->object_topic_name, ros::Duration(2.0));

    // Initializing the franka_state_sub subscriber and waiting
    ros::Subscriber franka_state_sub = this->nh.subscribe("/" + this->robot_name + this->franka_state_topic_name, 1, &GraspFailure::get_franka_state, this);
    ros::topic::waitForMessage<franka_msgs::FrankaState>("/" + this->robot_name + this->franka_state_topic_name, ros::Duration(2.0));

    // Initializing Panda SoftHand Client (TODO: Return error if initialize returns false)
    this->panda_softhand_client.initialize(this->nh);

    // Moveit names
    this->group_name = "panda_arm";
    this->end_effector_name = "right_hand_ee_link";

    // Initializing other moveit stuff (robot model, kinematic model and state)
    this->robot_model_loader_ptr.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    this->kinematic_model = this->robot_model_loader_ptr->getModel();
    ROS_INFO("Model frame: %s", this->kinematic_model->getModelFrame().c_str());
    this->kinematic_state.reset(new robot_state::RobotState(this->kinematic_model));

    // Setting the task service names
    std::cout << "Starting to advertise!!!" << std::endl;
    this->grasp_task_service_name = "/grasp_task_service";
    ros::ServiceServer grasp_task_server = this->nh.advertiseService(this->grasp_task_service_name, &GraspFailure::call_simple_grasp_task, this);
    std::cout << "Finished to advertise!!!" << std::endl;

    // Initializing other control values
    this->waiting_time = ros::Duration(5.0);
    this->waiting_time2 = ros::Duration(2.0);
    this->null_joints.resize(7);
    std::fill(this->null_joints.begin(), this->null_joints.end(), 0.0);
    // trajectory_msgs::JointTrajectoryPoint empty_joints_point;
    // empty_joints_point.positions = this->null_joints;
    // this->tmp_traj.points.push_back(empty_joints_point);

    // Spinning once
    ros::spinOnce();
}

GraspFailure::~GraspFailure(){
    
    // Nothing to do here yet
}

// Parameters parsing
bool GraspFailure::parse_task_params(){
    bool success = true;

    if(!ros::param::get("/grasp_params/robot_name", this->robot_name)){
		ROS_WARN("The param 'robot_name' not found in param server! Using default.");
		this->robot_name = "panda_arm";
		success = false;
	}

    if(!ros::param::get("/grasp_params/robot_joints_name", this->robot_joints_name)){
		ROS_WARN("The param 'robot_joints_name' not found in param server! Using default.");
		this->robot_joints_name = "panda_joint";
		success = false;
	}

    if(!ros::param::get("/grasp_params/pos_controller", this->pos_controller)){
		ROS_WARN("The param 'pos_controller' not found in param server! Using default.");
		this->pos_controller = "position_joint_trajectory_controller";
		success = false;
	}

    if(!ros::param::get("/grasp_params/imp_controller", this->imp_controller)){
		ROS_WARN("The param 'imp_controller' not found in param server! Using default.");
		this->imp_controller = "cartesian_impedance_controller_softbots_stiff_matrix";
		success = false;
	}

    if(!ros::param::get("/grasp_params/object_topic_name", this->object_topic_name)){
		ROS_WARN("The param 'object_topic_name' not found in param server! Using default.");
		this->object_topic_name = "/irim_demo/chosen_object";
		success = false;
	}

	if(!ros::param::get("/grasp_params/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/grasp_params/grasp_transform", this->grasp_transform)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		this->grasp_transform.resize(6);
        std::fill(this->grasp_transform.begin(), this->grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    if(!ros::param::get("/grasp_params/pre_grasp_transform", this->pre_grasp_transform)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		this->pre_grasp_transform.resize(6);
        std::fill(this->pre_grasp_transform.begin(), this->pre_grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T = this->convert_vector_to_pose(this->pre_grasp_transform);

    return success;
}

// Convert xyzrpy vector to geometry_msgs Pose
geometry_msgs::Pose GraspFailure::convert_vector_to_pose(std::vector<double> input_vec){
    
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


// Callback for object pose subscriber
void GraspFailure::get_object_pose(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T = *msg;
}

// Callback for franka state subscriber
void GraspFailure::get_franka_state(const franka_msgs::FrankaState::ConstPtr &msg){

    // Saving the message
    this->latest_franka_state = *msg;

    // Checking for libfranka errors
    if(msg->robot_mode != 2 && msg->robot_mode != 5){       // The robot state is not "automatic" or "manual guiding"
        this->franka_ok = false;
        if(DEBUG && false) ROS_ERROR("Something happened to the robot!");
    }else if(msg->robot_mode == 2){
        this->franka_ok = true;
        if(DEBUG && false) ROS_WARN("Now Franka is in a good mood!");
    }

    
}

// Callback for simple grasp task service
bool GraspFailure::call_simple_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_grasp_task done correctly with false request!";
        return true;
    }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff, grasp_pose);

    // Couting object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;

    // Setting zero pose as starting from present
    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;

    // 1) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, present_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    // 2) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, pre_grasp_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    // 3) Performing simple grasp with planning, executing and waiting
    if(!this->panda_softhand_client.call_hand_plan_service(0.8, 2.0, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan the simple grasp.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not perform the grasping.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    // 4) Returning to pre grasp pose
    if(!this->panda_softhand_client.call_slerp_service(pre_grasp_pose, grasp_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    // 5) Performing simple grasp with planning, executing and waiting
    if(!this->panda_softhand_client.call_hand_plan_service(0, 2.0, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan the simple grasp.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;

    }
    //NEW WAIT BEFORE OPENING HAND
    if(!this->panda_softhand_client.call_hand_wait_service(this->waiting_time2) || !this->franka_ok){
        ROS_ERROR("Could not perform the simple grasp.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand wait.";
        return false;
    }

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not perform the grasping.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }


    // Getting current joints
    std::vector<double> now_joints;
    double timeout = 3.0;
    bool found_ik_now = this->performIK(grasp_pose, timeout, now_joints);
    if (!found_ik_now){
        ROS_ERROR("Could not perform IK.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    // 7) Lifting the grasped? object
    if(!this->panda_softhand_client.call_joint_service(this->home_joints, now_joints, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
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



bool GraspFailure::performIK(geometry_msgs::Pose pose_in, double timeout, std::vector<double>& joints_out){
    Eigen::Isometry3d end_effector_state;
    tf::poseMsgToEigen(pose_in, end_effector_state);
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_model->getJointModelGroup(this->group_name);
    bool found_ik = this->kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if (!found_ik){
        ROS_ERROR("Could not find IK solution in GraspFailure...");
        return false;
    }

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);
    this->kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);

    if (DEBUG){
        ROS_INFO("Found an IK solution in GraspFailure: ");
        for (std::size_t i = 0; i < joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints_out[i]);
        }
    }

    return true;
}