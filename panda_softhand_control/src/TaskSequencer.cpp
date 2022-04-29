/* TASK SEQUENCER - Contains all recepies for grasping, and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "sensor_msgs/JointState.h"

#include "utils/parsing_utilities.h"
#include "panda_softhand_control/TaskSequencer.h"

TaskSequencer::TaskSequencer(ros::NodeHandle& nh_){

    // Initializing Node Handle
    this->nh = nh_;

    // Parsing the task params
    if(!this->parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Be careful, using default values...");
    }

    // Initializing the object subscriber and waiting (the object topic name is parsed now)
    this->object_sub = this->nh.subscribe(this->object_topic_name, 1, &TaskSequencer::get_object_pose, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>(this->object_topic_name, ros::Duration(2.0));
    
    // Initializing the object subscriber and waiting (the object topic name is parsed now) vacuuming
    this->object_sub_vacuuming = this->nh.subscribe(this->object_topic_name, 1, &TaskSequencer::get_object_pose_vacuuming, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>(this->object_topic_name, ros::Duration(2.0));
    

    // Initializing the object subscriber and waiting (the object topic name is parsed now) prethrown
    this->object_sub_prethrown = this->nh.subscribe(this->object_topic_name, 1, &TaskSequencer::get_object_pose_prethrown, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>(this->object_topic_name, ros::Duration(2.0));
    
    // Initializing the object subscriber and waiting (the object topic name is parsed now) throwing
    
    this->object_sub_thrown = this->nh.subscribe(this->object_topic_name, 1, &TaskSequencer::get_object_pose_thrown, this);
    ros::topic::waitForMessage<geometry_msgs::Pose>(this->object_topic_name, ros::Duration(2.0));

    // Initializing the franka_state_sub subscriber and waiting
    this->franka_state_sub = this->nh.subscribe("/" + this->robot_name + this->franka_state_topic_name, 1, &TaskSequencer::get_franka_state, this);
    ros::topic::waitForMessage<franka_msgs::FrankaState>("/" + this->robot_name + this->franka_state_topic_name, ros::Duration(2.0));

    // Initializing the tau_ext norm and franka recovery publishers
    this->pub_franka_recovery = this->nh.advertise<franka_msgs::ErrorRecoveryActionGoal>("/" + this->robot_name + "/franka_control/error_recovery/goal", 1);
    this->pub_tau_ext_norm = this->nh.advertise<std_msgs::Float64>("tau_ext_norm", 1);

    // Initializing Panda SoftHand Client (TODO: Return error if initialize returns false)
    this->panda_softhand_client.initialize(this->nh);

    // Setting the task service names
    this->grasp_task_service_name = "grasp_task_service";
    this->complex_grasp_task_service_name = "complex_grasp_task_service";
    this->place_task_service_name = "place_task_service";
    this->home_task_service_name = "home_task_service";
    this->handover_task_service_name = "handover_task_service";
    this->set_object_service_name = "set_object_service";

    this->vacuum_task_service_name = "vacuum_task_service";
    this->prethrown_task_service_name = "prethrown_service";
    this->throwing_task_service_name = "throwing_task_service_name";

    // Advertising the services
    this->grasp_task_server = this->nh.advertiseService(this->grasp_task_service_name, &TaskSequencer::call_simple_grasp_task, this);
    this->complex_grasp_task_server = this->nh.advertiseService(this->complex_grasp_task_service_name, &TaskSequencer::call_complex_grasp_task, this);
    this->place_task_server = this->nh.advertiseService(this->place_task_service_name, &TaskSequencer::call_simple_place_task, this);
    this->home_task_server = this->nh.advertiseService(this->home_task_service_name, &TaskSequencer::call_simple_home_task, this);
    this->handover_task_server = this->nh.advertiseService(this->handover_task_service_name, &TaskSequencer::call_simple_handover_task, this);
    this->set_object_server = this->nh.advertiseService(this->set_object_service_name, &TaskSequencer::call_set_object, this);
    
    this->vacuum_task_server = this->nh.advertiseService(this->vacuum_task_service_name,     &TaskSequencer::call_simple_vacuum_task, this );
    this->prethrowing_task_server = this->nh.advertiseService(this->prethrown_task_service_name,   &TaskSequencer::call_simple_prethrowing_task, this );
    this->throwing_task_server = this->nh.advertiseService(this->throwing_task_service_name, &TaskSequencer::call_simple_throwing_task, this );
    
    // Spinning once
    ros::spinOnce();

}

TaskSequencer::~TaskSequencer(){

    // Nothing to do here yet
}

// Parameters parsing
bool TaskSequencer::parse_task_params(){
    bool success = true;

    if(!ros::param::get("/task_sequencer/robot_name", this->robot_name)){
		ROS_WARN("The param 'robot_name' not found in param server! Using default.");
		this->robot_name = "panda_arm";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/robot_joints_name", this->robot_joints_name)){
		ROS_WARN("The param 'robot_joints_name' not found in param server! Using default.");
		this->robot_joints_name = "panda_joint";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/pos_controller", this->pos_controller)){
		ROS_WARN("The param 'pos_controller' not found in param server! Using default.");
		this->pos_controller = "position_joint_trajectory_controller";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/imp_controller", this->imp_controller)){
		ROS_WARN("The param 'imp_controller' not found in param server! Using default.");
		this->imp_controller = "cartesian_impedance_controller_softbots_stiff_matrix";
		success = false;
	}

    if(!ros::param::get("/task_sequencer/object_topic_name", this->object_topic_name)){
		ROS_WARN("The param 'object_topic_name' not found in param server! Using default.");
		this->object_topic_name = "/irim_demo/chosen_object";
		success = false;
	}

	if(!ros::param::get("/task_sequencer/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/grasp_transform", this->grasp_transform)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		this->grasp_transform.resize(6);
        std::fill(this->grasp_transform.begin(), this->grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);


    if(!ros::param::get("/task_sequencer/pre_grasp_transform", this->pre_grasp_transform)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		this->pre_grasp_transform.resize(6);
        std::fill(this->pre_grasp_transform.begin(), this->pre_grasp_transform.end(), 0.0);
		success = false;
	}
    
    
    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T = this->convert_vector_to_pose(this->pre_grasp_transform);
    
    /*------------------------------------------------------------------------*/
    /* For Vacuuming service..not sure if it will work*/
    
    if(!ros::param::get("/task_sequencer/grasp_transform_vacuum", this->grasp_transform_vacuuming)){
		ROS_WARN("The param 'grasp_transform_vacuum' not found in param server! Using default.");
		this->grasp_transform_vacuuming.resize(6);
        std::fill(this->grasp_transform_vacuuming.begin(), this->grasp_transform_vacuuming.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T_vacuuming = this->convert_vector_to_pose(this->grasp_transform_vacuuming);


    if(!ros::param::get("/task_sequencer/pre_grasp_transform_vacuum", this->pre_grasp_transform_vacuuming)){
		ROS_WARN("The param 'pre_grasp_transform_vacuum' not found in param server! Using default.");
		this->pre_grasp_transform_vacuuming.resize(6);
        std::fill(this->pre_grasp_transform_vacuuming.begin(), this->pre_grasp_transform_vacuuming.end(), 0.0);
		success = false;
	}
    
    
    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T_vacuuming = this->convert_vector_to_pose(this->pre_grasp_transform_vacuuming);

   /*-------------------------------------------------------------------------------*/
   

   /*------------------------------------------------------------------------*/
    /* For Prethrowing task service..not sure if it will work*/
    
    if(!ros::param::get("/task_sequencer/grasp_transform_prethrown", this->grasp_transform_prethrown)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		this->grasp_transform_prethrown.resize(6);
        std::fill(this->grasp_transform_prethrown.begin(), this->grasp_transform_prethrown.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T_prethrown = this->convert_vector_to_pose(this->grasp_transform_prethrown);


    if(!ros::param::get("/task_sequencer/pre_grasp_transform_prethrown", this->pre_grasp_transform_prethrown)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		this->pre_grasp_transform_prethrown.resize(6);
        std::fill(this->pre_grasp_transform_prethrown.begin(), this->pre_grasp_transform_prethrown.end(), 0.0);
		success = false;
	}
    
    
    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T_prethrown = this->convert_vector_to_pose(this->pre_grasp_transform_prethrown);

   /*-------------------------------------------------------------------------------*/
    
    /*------------------------------------------------------------------------*/
    /* For Throwing task service..not sure if it will work*/
    
    if(!ros::param::get("/task_sequencer/grasp_transform_thrown", this->grasp_transform_thrown)){
		ROS_WARN("The param 'grasp_transform_thrown' not found in param server! Using default.");
		this->grasp_transform_thrown.resize(6);
        std::fill(this->grasp_transform_thrown.begin(), this->grasp_transform_thrown.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T_thrown = this->convert_vector_to_pose(this->grasp_transform_thrown);


    if(!ros::param::get("/task_sequencer/pre_grasp_transform_thrown", this->pre_grasp_transform_thrown)){
		ROS_WARN("The param 'pre_grasp_transform_thrown' not found in param server! Using default.");
		this->pre_grasp_transform_thrown.resize(6);
        std::fill(this->pre_grasp_transform_thrown.begin(), this->pre_grasp_transform_thrown.end(), 0.0);
		success = false;
	}
    
    
    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T_thrown = this->convert_vector_to_pose(this->pre_grasp_transform_thrown);
















    if(!ros::param::get("/task_sequencer/handover_joints", this->handover_joints)){
		ROS_WARN("The param 'handover_joints' not found in param server! Using default.");
		this->handover_joints = {-0.101, 0.161, 0.159, -1.651, 2.023, 2.419, -0.006};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/place_joints", this->place_joints)){
		ROS_WARN("The param 'place_joints' not found in param server! Using default.");
		this->place_joints = {-0.136, 0.794, -0.115, -1.337, 0.250, 2.217, -0.479};
		success = false;
	}
    
    if(!ros::param::get("/task_sequencer/throwing_joints", this->throwing_joints)){
		ROS_WARN("The param 'throwing_joints' not found in param server! Using default.");
		this->throwing_joints = {-0.136, 0.794, -0.115, -1.337, 0.250, 2.217, -0.479};
		success = false;
	}


    if(!ros::param::get("/task_sequencer/handover_thresh", this->handover_thresh)){
		ROS_WARN("The param 'handover_thresh' not found in param server! Using default.");
		this->handover_thresh = 4.5;
		success = false;
	}

    // Getting the XmlRpc value and parsing
    if(!this->nh.getParam("/task_sequencer", this->task_seq_params)){
        ROS_ERROR("Could not get the XmlRpc value.");
        success = false;
    }

    if(!parseParameter(this->task_seq_params, this->poses_map, "poses_map")){
        ROS_ERROR("Could not parse the poses map.");
        success = false;
    }

    if(DEBUG){
        ROS_INFO_STREAM("The poses map is");
        for(auto it : this->poses_map){
            std::cout << "]" << std::endl;
        }
    }

    return success;
}

// Convert xyzrpy vector to geometry_msgs Pose
geometry_msgs::Pose TaskSequencer::convert_vector_to_pose(std::vector<double> input_vec){

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

// To switch the controllers
bool TaskSequencer::switch_controllers(std::string robot_name, std::string from_controller, std::string to_controller){

    // Temporary bool to be returned
    bool success = false;

    // Clearing the switch message
    this->switch_controller.request.start_controllers.clear();
    this->switch_controller.request.stop_controllers.clear();

    // Filling up the switch message
    this->switch_controller.request.start_controllers.push_back(to_controller);
    this->switch_controller.request.stop_controllers.push_back(from_controller);
    this->switch_controller.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

    // Swithching controller by calling the service
    return ros::service::call<controller_manager_msgs::SwitchController>(robot_name + this->switch_service_name, this->switch_controller);
}

// Callback for object pose subscriber
void TaskSequencer::get_object_pose(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T = *msg;
}

//*Callback for vacuuming object pose

void TaskSequencer::get_object_pose_vacuuming(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T_vacuuming = *msg;
}

//*Callback for prethrown object pose

void TaskSequencer::get_object_pose_prethrown(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T_prethrown = *msg;
}

//*Callback for thrown object pose

void TaskSequencer::get_object_pose_thrown(const geometry_msgs::Pose::ConstPtr &msg){

    // Saving the message
    this->object_pose_T_thrown = *msg;
}


// Callback for franka state subscriber
void TaskSequencer::get_franka_state(const franka_msgs::FrankaState::ConstPtr &msg){

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

    // Getting the tau ext
    if(DEBUG && false){
        std::cout << "The latest tau ext vector is \n [ ";
        for(auto it : this->latest_franka_state.tau_ext_hat_filtered)  std::cout << it << " ";
        std::cout << "]" << std::endl;
    }

    // Computing the norm
    this->tau_ext_norm = 0.0;
    for(auto it : this->latest_franka_state.tau_ext_hat_filtered){
        this->tau_ext_norm += std::pow(it, 2);
    }
    this->tau_ext_norm = std::sqrt(this->tau_ext_norm);

    // Publishing norm
    std_msgs::Float64 norm_msg; norm_msg.data = this->tau_ext_norm;
    this->pub_tau_ext_norm.publish(norm_msg);

}

// Callback for simple grasp task service
bool TaskSequencer::call_simple_grasp_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_grasp_task done correctly with false request!";
        return true;
    }

    // // 1) Going to home configuration
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
    //     ROS_ERROR("Could not go to the specified home joint configuration.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff, grasp_pose);

    // Computing object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // 4) Performing simple grasp
    if(!this->panda_softhand_client.call_hand_service(1.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not perform the simple grasp.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // // 4) Lifting the grasped? object
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
    //     ROS_ERROR("Could not lift to the specified pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_grasp_task was correctly performed!";
    return true;
}

/*------------Vacuuming Service------------------*/
bool TaskSequencer::call_simple_vacuum_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

// Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_grasp_task done correctly with false request!";
        return true;
    }

    // // 1) Going to home configuration
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
    //     ROS_ERROR("Could not go to the specified home joint configuration.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff_vacuuming; tf::poseMsgToEigen(this->object_pose_T_vacuuming, object_pose_aff_vacuuming);
    Eigen::Affine3d grasp_transform_aff_vacuuming; tf::poseMsgToEigen(this->grasp_T_vacuuming, grasp_transform_aff_vacuuming);
    Eigen::Affine3d pre_grasp_transform_aff_vacuuming; tf::poseMsgToEigen(this->pre_grasp_T_vacuuming, pre_grasp_transform_aff_vacuuming);

    geometry_msgs::Pose pre_grasp_pose_vacuuming; geometry_msgs::Pose grasp_pose_vacuuming;
    tf::poseEigenToMsg(object_pose_aff_vacuuming * grasp_transform_aff_vacuuming * pre_grasp_transform_aff_vacuuming, pre_grasp_pose_vacuuming);
    tf::poseEigenToMsg(object_pose_aff_vacuuming * grasp_transform_aff_vacuuming, grasp_pose_vacuuming);

    // Computing object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff_vacuuming.translation() << std::endl;

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose_vacuuming, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose_vacuuming, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }

    // // 4) Performing simple grasp
    // if(!this->panda_softhand_client.call_hand_service(1.0, 2.0) || !this->franka_ok){
    //     ROS_ERROR("Could not perform the simple grasp.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // // 4) Lifting the grasped? object
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
    //     ROS_ERROR("Could not lift to the specified pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_grasp_task was correctly performed!";
    return true;
}

/*------------Prethrowing Service------------------*/

bool TaskSequencer::call_simple_prethrowing_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

   // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple prethrowning task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_prethrowing_task done correctly with false request!";
        return true;
    }

    // // 1) Going to home configuration
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
    //     ROS_ERROR("Could not go to the specified home joint configuration.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff_prethrown; tf::poseMsgToEigen(this->object_pose_T_prethrown, object_pose_aff_prethrown);
    Eigen::Affine3d grasp_transform_aff_prethrown; tf::poseMsgToEigen(this->grasp_T_prethrown, grasp_transform_aff_prethrown);
    Eigen::Affine3d pre_grasp_transform_aff_prethrown; tf::poseMsgToEigen(this->pre_grasp_T_prethrown, pre_grasp_transform_aff_prethrown);

    geometry_msgs::Pose pre_grasp_pose_prethrown; geometry_msgs::Pose grasp_pose_prethrown;
    tf::poseEigenToMsg(object_pose_aff_prethrown * grasp_transform_aff_prethrown * pre_grasp_transform_aff_prethrown, pre_grasp_pose_prethrown);
    tf::poseEigenToMsg(object_pose_aff_prethrown * grasp_transform_aff_prethrown, grasp_pose_prethrown);

    // Computing object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff_prethrown.translation() << std::endl;

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose_prethrown, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_prethrown_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose_prethrown, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_prethrown_task was NOT performed correctly!";
        return false;
    }

    // // 4) Performing simple grasp
    // if(!this->panda_softhand_client.call_hand_service(1.0, 2.0) || !this->franka_ok){
    //     ROS_ERROR("Could not perform the simple grasp.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // // 4) Lifting the grasped? object
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
    //     ROS_ERROR("Could not lift to the specified pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_prethrown_task was correctly performed!";
    return true;
};


/*------------Throwing Service -------------------*/
bool TaskSequencer::call_simple_throwing_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple throwning task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_throwing_task done correctly with false request!";
        return true;
    }

    // // 1) Going to home configuration
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
    //     ROS_ERROR("Could not go to the specified home joint configuration.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff_thrown; tf::poseMsgToEigen(this->object_pose_T_thrown, object_pose_aff_thrown);
    Eigen::Affine3d grasp_transform_aff_thrown; tf::poseMsgToEigen(this->grasp_T_thrown, grasp_transform_aff_thrown);
    Eigen::Affine3d pre_grasp_transform_aff_thrown; tf::poseMsgToEigen(this->pre_grasp_T_thrown, pre_grasp_transform_aff_thrown);

    geometry_msgs::Pose pre_grasp_pose_thrown; geometry_msgs::Pose grasp_pose_thrown;
    tf::poseEigenToMsg(object_pose_aff_thrown * grasp_transform_aff_thrown * pre_grasp_transform_aff_thrown, pre_grasp_pose_thrown);
    tf::poseEigenToMsg(object_pose_aff_thrown * grasp_transform_aff_thrown, grasp_pose_thrown);

    // Computing object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff_thrown.translation() << std::endl;

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose_thrown, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_thrown_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose_thrown, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_simple_thrown_task was NOT performed correctly!";
        return false;
    }

    // // 4) Performing simple grasp
    // if(!this->panda_softhand_client.call_hand_service(1.0, 2.0) || !this->franka_ok){
    //     ROS_ERROR("Could not perform the simple grasp.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // // 4) Lifting the grasped? object
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
    //     ROS_ERROR("Could not lift to the specified pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_prethrown_task was correctly performed!";
    return true;

};

// Callback for complex grasp task service (goes to specified pose)
bool TaskSequencer::call_complex_grasp_task(panda_softhand_control::complex_grasp::Request &req, panda_softhand_control::complex_grasp::Response &res){

    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the complex grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_complex_grasp_task done correctly with false request!";
        return true;
    }

    // 1) Going to home configuration
    if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified home joint configuration.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly!";
        return false;
    }

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose
    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(req.object_pose, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff, grasp_pose);

    // Couting object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly!";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, false) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly!";
        return false;
    }

    // 4) Performing complex grasp
    if(!this->panda_softhand_client.call_hand_service(1.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not perform the complex grasp.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly!";
        return false;
    }

    // 4) Lifting the grasped? object
    if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly!";
        return false;
    }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_complex_grasp_task was correctly performed!";
    return true;

}

// Callback for simple place task service
bool TaskSequencer::call_simple_place_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple place task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_place_task done correctly with false request!";
        return true;
    }

    // 1) Going to place joint config
    if(!this->panda_softhand_client.call_joint_service(this->place_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified place joint config.");
        res.success = false;
        res.message = "The service call_simple_place_task was NOT performed correctly!";
        return false;
    }

    // // 2) Opening hand
    // if(!this->panda_softhand_client.call_hand_service(0.0, 2.0) || !this->franka_ok){
    //     ROS_ERROR("Could not open the hand.");
    //     res.success = false;
    //     res.message = "The service call_simple_place_task was NOT performed correctly!";
    //     return false;
    // }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_place_task was correctly performed!";
    return true;

}

// Callback for simple home task service
bool TaskSequencer::call_simple_home_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple home task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_home_task done correctly with false request!";
        return true;
    }

    // 1) Going to home joint config
    if(!this->panda_softhand_client.call_joint_service(this->home_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified home joint config.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly!";
        return false;
    }

    // 2) Opening hand
    if(!this->panda_softhand_client.call_hand_service(0.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not open the hand.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly!";
        return false;
    }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_home_task was correctly performed!";
    return true;

}

// Callback for simple handover task service
bool TaskSequencer::call_simple_handover_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple handover task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_handover_task done correctly with false request!";
        return true;
    }

    // 1) Going to handover joint config
    if(!this->panda_softhand_client.call_joint_service(this->handover_joints) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified handover joint config.");
        res.success = false;
        res.message = "The service call_simple_handover_task was NOT performed correctly!";
        return false;
    }

    // 2) Waiting for threshold or for some time
    sleep(1);       // Sleeping for a second to avoid robot stopping peaks
    bool hand_open = false; ros::Time init_time = ros::Time::now(); ros::Time now_time;
    double base_tau_ext = this->tau_ext_norm;           // Saving the present tau for later computation of variation
    while(!hand_open){
        now_time = ros::Time::now();
        usleep(500);                         // Don't know why, but the threshold works with this sleeping
        if(std::abs(this->tau_ext_norm - base_tau_ext) > this->handover_thresh){
            hand_open = true;
            if(DEBUG) ROS_WARN_STREAM("Opening condition reached!" << " SOMEONE PULLED!");
            if(DEBUG) ROS_WARN_STREAM("The tau_ext difference is " << std::abs(this->tau_ext_norm - base_tau_ext) << " and the threshold is " << this->handover_thresh << ".");
        }
        if((now_time - init_time) > ros::Duration(10, 0)){
            hand_open = true;
            if(DEBUG) ROS_WARN_STREAM("Opening condition reached!" << " TIMEOUT!");
            if(DEBUG) ROS_WARN_STREAM("The initial time was " << init_time << ", now it is " << now_time
                << ", the difference is " << (now_time - init_time) << " and the timeout thresh is " << ros::Duration(10, 0));
        }
    }

    // 3) Opening hand
    if(!this->panda_softhand_client.call_hand_service(0.0, 2.0) || !this->franka_ok){
        ROS_ERROR("Could not open the hand.");
        res.success = false;
        res.message = "The service call_simple_handover_task was NOT performed correctly!";
        return false;
    }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_handover_task was correctly performed!";
    return true;

}

// Callback for set object task service
bool TaskSequencer::call_set_object(panda_softhand_control::set_object::Request &req, panda_softhand_control::set_object::Response &res){

    // Checking if the parsed map contains the requested object
    auto search = this->poses_map.find(req.object_name);
    if(search == this->poses_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }    
    // Setting the grasp pose as requested
    this->grasp_transform = this->poses_map.at(req.object_name);

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    // Now, everything is ok
    ROS_INFO_STREAM("Grasp pose changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;
}
