/* TASK SEQUENCER - Contains all recepies for grasping, and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "utils/parsing_utilities.h"
#include "panda_softhand_control/TaskSequencer.h"
#include <std_msgs/UInt8.h>

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
    
    this->pub_blow = (this->nh).advertise<std_msgs::Empty>(this->blow_off,1);
    this->pub_suction = (this->nh).advertise<std_msgs::Empty>(this->suction,1);

    this->pub_duty = (this->nh).advertise<std_msgs::UInt8>(this->duty,1);

    // Initializing the franka_state_sub subscriber and waiting
    this->franka_state_sub = this->nh.subscribe("/" + this->robot_name + this->franka_state_topic_name, 1, &TaskSequencer::get_franka_state, this);
    ros::topic::waitForMessage<franka_msgs::FrankaState>("/" + this->robot_name + this->franka_state_topic_name, ros::Duration(2.0));

    // Initializing the tau_ext norm and franka recovery publishers
    this->pub_franka_recovery = this->nh.advertise<franka_msgs::ErrorRecoveryActionGoal>("/" + this->robot_name + "/franka_control/error_recovery/goal", 1);
    this->pub_tau_ext_norm = this->nh.advertise<std_msgs::Float64>("tau_ext_norm", 1);

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
    this->grasp_task_service_name = "grasp_task_service";
    this->complex_grasp_task_service_name = "complex_grasp_task_service";
    this->place_task_service_name = "place_task_service";
    this->home_task_service_name = "home_task_service";
    this->handover_task_service_name = "handover_task_service";
    this->grasp_task_handtool_service_name = "grasping_handtool_service";//
    this->throwing_task_service_name = "throwing_service";
    this->replace_task_service_name ="replacing_service";

    //
    this->set_object_service_name = "set_object_service";
    this->set_place_service_name = "set_place_service";

    // 
    this->set_pre_throwing_joint_name = "set_pre_throwing_service";
    this->set_throwing_joint_name = "set_throwing_service";
    this->set_vacuum_name = "set_vacuum_service";
    this->set_duty_cycle_name ="set_duty_cycle_service";

    // Advertising the services
    this->grasp_task_server = this->nh.advertiseService(this->grasp_task_service_name, &TaskSequencer::call_simple_grasp_task, this);
    this->complex_grasp_task_server = this->nh.advertiseService(this->complex_grasp_task_service_name, &TaskSequencer::call_complex_grasp_task, this);
    this->place_task_server = this->nh.advertiseService(this->place_task_service_name, &TaskSequencer::call_simple_place_task, this);
    this->home_task_server = this->nh.advertiseService(this->home_task_service_name, &TaskSequencer::call_simple_home_task, this);
    this->handover_task_server = this->nh.advertiseService(this->handover_task_service_name, &TaskSequencer::call_simple_handover_task, this);
    this->grasp_handtool_task_server = this->nh.advertiseService(this->grasp_task_handtool_service_name, &TaskSequencer::call_grasp_handtool_task, this); //
    this->throwing_task_server = this->nh.advertiseService(this->throwing_task_service_name,  &TaskSequencer::call_throwing_task, this);
    this->replace_task_server = this->nh.advertiseService(this->replace_task_service_name, &TaskSequencer::call_replace_task, this);

    this->set_object_server = this->nh.advertiseService(this->set_object_service_name, &TaskSequencer::call_set_object, this);
    this->set_place_server =  this->nh.advertiseService(this->set_place_service_name, &TaskSequencer::call_set_place, this);
    
    
    this->set_pre_throwing_server = this->nh.advertiseService(this->set_pre_throwing_joint_name, &TaskSequencer::call_set_prethrowing_joints_place, this);
    this->set_throwing_server = this->nh.advertiseService(this->set_throwing_joint_name, &TaskSequencer::call_set_throwing_joints_place, this);
    this->set_vacuum_place_server = this->nh.advertiseService(this->set_vacuum_name, &TaskSequencer::call_set_vacuum_place, this);
    this->set_duty_cycle_server = this->nh.advertiseService(this->set_duty_cycle_name, &TaskSequencer::call_set_duty_cycle, this); 

    // Initializing other control values 
    this->waiting_time = ros::Duration(10.0);
    this->null_joints.resize(7);
    std::fill(this->null_joints.begin(), this->null_joints.end(), 0.0);
    // trajectory_msgs::JointTrajectoryPoint empty_joints_point;
    // empty_joints_point.positions = this->null_joints;
    // this->tmp_traj.points.push_back(empty_joints_point);

    // Spinning once
    ros::spinOnce();
}

TaskSequencer::~TaskSequencer(){
    std::cout << "Destructor executed" << std::endl;
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
		this->object_topic_name = "/grasping/chosen_object";
		success = false;
	}

	if(!ros::param::get("/task_sequencer/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/pre_throwing_joints", this->pre_throwing_joints)){
		ROS_WARN("The param 'pre_throwing_joints' not found in param server! Using default.");
		this->pre_throwing_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/throwing_joints", this->throwing_joints)){
		ROS_WARN("The param 'throwing_joints' not found in param server! Using default.");
		this->throwing_joints = {-0.035, -0.109, -0.048, -1.888, 0.075, 1.797, -0.110};
		success = false;
	}

    /*duty cycle*/
    
    if(!ros::param::get("/task_sequencer/duty_cycle", this->duty_cycle)){
		ROS_WARN("The param 'duty cycle' not found in param server!");	
	}

    /*Parsing the first nd second syn value*/

    if(!ros::param::get("/task_sequencer/first_syn_value", this->first_syn_value.data)){
		ROS_WARN("The param 'first_syn_value' not found in param server!");	
	}

    if(!ros::param::get("/task_sequencer/second_syn_value", this->second_syn_value.data)){
		ROS_WARN("The param 'second_syn_value' not found in param server!");	
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
    

    /*-----*/

    if(!ros::param::get("/task_sequencer/pre_vacuum_transform", this->pre_vacuum_transform)){
		ROS_WARN("The param 'pre_vacuum_transform' not found in param server! Using default.");
		this->pre_vacuum_transform.resize(6);
        std::fill(this->pre_vacuum_transform.begin(), this->pre_vacuum_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_vacuum_transform vector to geometry_msgs Pose
    this->pre_vacuum_T = this->convert_vector_to_pose(this->pre_vacuum_transform);


    if(!ros::param::get("/task_sequencer/vacuum_transform", this->vacuum_transform)){
		ROS_WARN("The param 'vacuum_transform' not found in param server! Using default.");
		this->vacuum_transform.resize(6);
        std::fill(this->vacuum_transform.begin(), this->vacuum_transform.end(), 0.0);
		success = false;
	}

    // Converting the vacuum_transform vector to geometry_msgs Pose
    this->vacuum_T = this->convert_vector_to_pose(this->vacuum_transform);

    /*-----*/
    
    /*-----*/

    if(!ros::param::get("/task_sequencer/pre_throwing_transform", this->pre_throwing_transform)){
		ROS_WARN("The param 'pre_throwing_transform' not found in param server! Using default.");
		this->pre_throwing_transform.resize(6);
        std::fill(this->pre_throwing_transform.begin(), this->pre_throwing_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_vacuum_transform vector to geometry_msgs Pose
    this->pre_throwing_T = this->convert_vector_to_pose(this->pre_throwing_transform);


    if(!ros::param::get("/task_sequencer/throwing_transform", this->throwing_transform)){
		ROS_WARN("The param 'throwing_transform' not found in param server! Using default.");
		this->throwing_transform.resize(6);
        std::fill(this->throwing_transform.begin(), this->throwing_transform.end(), 0.0);
		success = false;
	}

    // Converting the vacuum_transform vector to geometry_msgs Pose
    this->throwing_T = this->convert_vector_to_pose(this->throwing_transform);

    /*-----*/

    if(!ros::param::get("/task_sequencer/pre_replace_hand_tool", this->pre_replace_hand_tool)){
		ROS_WARN("The param 'pre_replace_hand_tool' not found in param server! Using default.");
		this->pre_replace_hand_tool.resize(6);
        std::fill(this->pre_replace_hand_tool.begin(), this->pre_replace_hand_tool.end(), 0.0);
		success = false;
	}

    // Converting the pre_vacuum_transform vector to geometry_msgs Pose
    this->pre_replace_hand_tool_T = this->convert_vector_to_pose(this->pre_replace_hand_tool);

    if(!ros::param::get("/task_sequencer/replace_hand_tool", this->replace_hand_tool)){
		ROS_WARN("The param 'replace_hand_tool' not found in param server! Using default.");
		this->replace_hand_tool.resize(6);
        std::fill(this->replace_hand_tool.begin(), this->replace_hand_tool.end(), 0.0);
		success = false;
	}

    // Converting the vacuum_transform vector to geometry_msgs Pose
    this->replace_hand_tool_T = this->convert_vector_to_pose(this->replace_hand_tool);

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
            std::cout << it.first << " : [ ";
            for(auto vec_it : it.second){
                std::cout << vec_it << " ";
            } 
            std::cout << "]" << std::endl;     
        }
    }

    /* Parsing the vacuum_poses_map */

    if(!parseParameter(this->task_seq_params, this->vacuum_pose_map, "vacuum_pose_map")){
        ROS_ERROR("Could not parse the vacuum pose map.");
        success = false;
    }

    if(DEBUG){
        ROS_INFO_STREAM("The vacuum_pose_map is");
        for(auto it : this->vacuum_pose_map){
            std::cout << it.first << " : [ ";
            for(auto vec_it : it.second){
                std::cout << vec_it << " ";
            } 
            std::cout << "]" << std::endl;     
        }
    }

    if(!parseParameter(this->task_seq_params, this->place_joints_map, "place_joints_map")){
        ROS_ERROR("Could not parse the poses map.");
        success = false;
    }

    if(DEBUG){
        ROS_INFO_STREAM("The place joins map is");
        for(auto it : this->place_joints_map){
            std::cout << it.first << " : [ ";
            for(auto vec_it : it.second){
                std::cout << vec_it << " ";
            } 
            std::cout << "]" << std::endl;     
        }
    }
    
    /* Parsing the pre_throwing_joints_map*/

    if(!parseParameter(this->task_seq_params, this->pre_throwing_joints_map, "pre_throwing_joints_map")){
        ROS_ERROR("Could not parse the poses map.");
        success = false;
    }

    if(DEBUG){
        ROS_INFO_STREAM("The pre_throwing_joint map is");
        for(auto it : this->pre_throwing_joints_map){
            std::cout << it.first << " : [ ";
            for(auto vec_it : it.second){
                std::cout << vec_it << " ";
            } 
            std::cout << "]" << std::endl;     
        }
    }

    /* Parsing the throwing_joints_map*/
    
    if(!parseParameter(this->task_seq_params, this->throwing_joints_map, "throwing_joints_map")){
        ROS_ERROR("Could not parse the poses map.");
        success = false;
    }

    if(DEBUG){
        ROS_INFO_STREAM("The throwing_joint map is");
        for(auto it : this->throwing_joints_map){
            std::cout << it.first << " : [ ";
            for(auto vec_it : it.second){
                std::cout << vec_it << " ";
            } 
            std::cout << "]" << std::endl;     
        }
    }

    /* Parsing duty_cycle_map */
    
    if(!parseParameter(this->task_seq_params, this->duty_cycle_map, "duty_cycle_map")){
        ROS_ERROR("Could not parse the duty_cycle map.");
        success = false;
        
    }

    if(DEBUG){
        ROS_INFO_STREAM("The duty_cycle map is");
        for(auto it : this->duty_cycle_map){
            std::cout << it.first << " : [ ";
            std::cout << it.second << " ";
            std::cout << "]" << std::endl;     
        }
    }

    /*Parsing the first synergy map */

    if(!parseParameter(this->task_seq_params, this->first_synergy_map, "first_synergy_map")){
        ROS_ERROR("Could not parse the first_synergy_map.");
        success = false;    
    }

    if(DEBUG){
        ROS_INFO_STREAM("The first_synergy_map is");
        for(auto it : this->first_synergy_map){
            std::cout << it.first << " : [ ";
            std::cout << it.second << " ";
            std::cout << "]" << std::endl;     
        }
    }

    /* Parsing the second synergy map */

    if(!parseParameter(this->task_seq_params, this->second_synergy_map, "second_synergy_map")){
        ROS_ERROR("Could not parse the second_sinergy_map.");
        success = false;
    }

    if(DEBUG){
        ROS_INFO_STREAM("The second_synergy_map is");
        for(auto it : this->second_synergy_map){
            std::cout << it.first << " : [ ";
            std::cout << it.second << " ";
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

    // Computing the grasp and pregrasp pose and converting to geometry_msgs Pose

    // Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(grasp_transform_aff, grasp_pose);
    
    // Couting object pose for debugging
    // std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;
    // std::cout << "Object orientation is \n" << object_pose_aff.rotation() << std::endl;
    // Setting zero pose as starting from present
    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;


    /*PLAN 0: Planning to PreGraspPose*/

    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, present_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }
    
    /*EXEC 0*/

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    /*PLAN 1: Planning to GraspPose*/

    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, pre_grasp_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }
    
    /*WAIT 0:*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }
    
    /*EXEC 1: Going to GraspPose*/

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    /*PLAN 2: Plan the closing of the hand*/

    if(!this->panda_softhand_client.call_hand_plan_service(this->first_syn_value.data, this->second_syn_value.data, 1.5, this->tmp_traj_hand) || !this->franka_ok){
        ROS_ERROR("Could not plan the simple open.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }
    
    /*WAIT 1: waiting for the arm*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    /*EXEC 2*/

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj_hand) || !this->franka_ok){
        ROS_ERROR("Could not perform the call hand control service.");
        res.success = false;
        res.message = "The service call hand control service was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    /*PLAN 3: Planning to PreGraspPose*/

    if(!this->panda_softhand_client.call_slerp_service(pre_grasp_pose, grasp_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    }
    
    /*WAIT 2*/
    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }
    
    /*EXEC 3*/
    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    /*WAIT 3*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    /*PLAN 4*/

    if(!this->panda_softhand_client.call_joint_service(this->home_joints, null_joints, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to home joints position.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false;
    };

    
    /*EXEC 4: Going to Home Joints*/

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to Home Joints.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    /*PLAN 5: Plan the opening of the hand*/

    if(!this->panda_softhand_client.call_hand_plan_service(0.0,0.0,1.5,this->tmp_traj_hand) || !this->franka_ok){
        ROS_ERROR("Could not plan the simple open.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    /*WAIT 4*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to home joints position");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    /*EXEC 5: Open the hand*/

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj_hand) || !this->franka_ok){
        ROS_ERROR("Could not perform the call hand control service.");
        res.success = false;
        res.message = "The service call hand control service was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    /*WAIT 5*/

    if(!this->panda_softhand_client.call_hand_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for opening the hand");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }






    // // 3) Going to grasp pose

    // if(!this->panda_softhand_client.call_slerp_service(grasp_pose, pre_grasp_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not plan to the specified grasp pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
    //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
    //     return false;
    // }

    // if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not go to grasp pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
    //     return false;
    // }

    // // 4) Performing simple grasp with planning, executing and waiting

    // if(!this->panda_softhand_client.call_hand_plan_service(1.0, 1.0, 2.0, this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not plan the simple grasp.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand plan.";
    //     return false;
    // }

    // if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
    //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
    //     return false;
    // }

    // if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not perform the grasping.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand control.";
    //     return false;
    // }

    // // Getting current joints
    // std::vector<double> now_joints;
    // double timeout = 3.0;
    // bool found_ik_now = this->performIK(grasp_pose, timeout, now_joints);
    // if (!found_ik_now){
    //     ROS_ERROR("Could not perform IK.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
    //     return false;
    // }

    // // 4) Lifting the grasped? object
    // if(!this->panda_softhand_client.call_joint_service(this->home_joints, now_joints, this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not lift to the specified pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // if(!this->panda_softhand_client.call_hand_wait_service(ros::Duration(3.0)) || !this->franka_ok){
    //     ROS_ERROR("Could not perform the simple grasp.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand wait.";
    //     return false;
    // }

    // if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not go to grasp pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
    //     return false;
    // }

    // if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
    //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
    //     return false;
    // }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_grasp_task was correctly performed!";
    return true;
    
}

// Callback for complex grasp task service (goes to specified pose)
bool TaskSequencer::call_complex_grasp_task(panda_softhand_msgs::complex_grasp::Request &req, panda_softhand_msgs::complex_grasp::Response &res){

    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the complex grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_complex_grasp_task done correctly with false request!";
        return true;
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

    // Setting zero pose as starting from present
    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;

    // 2) Going to pregrasp pose
    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, present_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    // 3) Going to grasp pose
    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, pre_grasp_pose, false, this->tmp_traj, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    // 4) Performing complex grasp with planning, executing and waiting
    if(!this->panda_softhand_client.call_hand_plan_service(1.0, 1.0, 2.0, this->tmp_traj) || this->franka_ok){
        ROS_ERROR("Could not plan the grasping.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }
    
    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not perform the grasping.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    // Getting current joints
    std::vector<double> now_joints;
    double timeout = 3.0;
    bool found_ik_now = this->performIK(grasp_pose, timeout, now_joints);
    if (!found_ik_now){
        ROS_ERROR("Could not perform IK.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    // 4) Lifting the grasped? object
    if(!this->panda_softhand_client.call_joint_service(this->home_joints, now_joints, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan lift to the specified pose.");
        res.success = false;
        res.message = "The service call_complex_grasp_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_hand_wait_service(ros::Duration(3.0)) || !this->franka_ok){
        ROS_ERROR("Could not wait for the grasping.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error plan in hand wait.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to lift pose");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly! Error wait in arm control.";
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
    if(!this->panda_softhand_client.call_joint_service(this->place_joints, this->null_joints, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified place joint config.");
        res.success = false;
        res.message = "The service call_simple_place_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to place joint config.");
        res.success = false;
        res.message = "The service call_simple_place_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    // // 2) Opening hand
    // if(!this->panda_softhand_client.call_hand_plan_service(1.0, 1.0, 2.0, this->tmp_traj)this->franka_ok){
    //     ROS_ERROR("Could not plan the simple open.");
    //     res.success = false;
    //     res.message = "The service call_simple_place_task was NOT performed correctly! Error plan in hand plan.";
    //     return false;
    // }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to place joint config");
        res.success = false;
        res.message = "The service call_simple_place_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    // if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not perform the simple open.");
    //     res.success = false;
    //     res.message = "The service call_simple_place_task was NOT performed correctly! Error plan in hand control.";
    //     return false;
    // }

    // if(!this->panda_softhand_client.call_hand_wait_service(this->waiting_time) || !this->franka_ok){
    //     ROS_ERROR("Could not wait for the simple open.");
    //     res.success = false;
    //     res.message = "The service call_simple_place_task was NOT performed correctly! Error plan in hand wait.";
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
    if(!this->panda_softhand_client.call_joint_service(this->home_joints, this->null_joints, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified home joint config.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to home joint config.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    // 2) Opening hand
    if(!this->panda_softhand_client.call_hand_plan_service(1.0, 1.0, 2.0, this->tmp_traj) || this->franka_ok){
        ROS_ERROR("Could plan the simple open.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to home joint config");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not perform the simple open.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    if(!this->panda_softhand_client.call_hand_wait_service(ros::Duration(3.0)) || !this->franka_ok){
        ROS_ERROR("Could not wait the simple open.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error plan in hand wait.";
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
    if(!this->panda_softhand_client.call_joint_service(this->handover_joints, this->null_joints, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to the specified handover joint config.");
        res.success = false;
        res.message = "The service call_simple_handover_task was NOT performed correctly!";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not go to handover joint config.");
        res.success = false;
        res.message = "The service call_simple_handover_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to handover joint config");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error wait in arm control.";
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
    if(!this->panda_softhand_client.call_hand_plan_service(1.0, 1.0, 2.0, this->tmp_traj) || this->franka_ok){
        ROS_ERROR("Could not plan the simple open.");
        res.success = false;
        res.message = "The service call_simple_handover_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not perform the simple open.");
        res.success = false;
        res.message = "The service call_simple_handover_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    if(!this->panda_softhand_client.call_hand_wait_service(ros::Duration(3.0)) || !this->franka_ok){
        ROS_ERROR("Could not wait the simple open.");
        res.success = false;
        res.message = "The service call_simple_handover_task was NOT performed correctly! Error plan in hand wait.";
        return false;
    }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_handover_task was correctly performed!";
    return true;

}

// Callback for grasping hand-tool 
bool TaskSequencer::call_grasp_handtool_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  
    // // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_throwing_task done correctly with false request!";
        return true;
    }
    
    /* Computing the grasp and pregrasp pose and converting to geometry_msgs Pose */

    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(object_pose_aff * grasp_transform_aff, grasp_pose);

    // Couting object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;
    std::cout << "Object orientation is \n" << object_pose_aff.rotation() << std::endl;

    // Setting zero pose as starting from present (it starts from home position)
    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;
    
    // 1) Going to pregrasp pose
    
    /* PLAN 1*/

    if(!this->panda_softhand_client.call_pose_service(pre_grasp_pose, present_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly!";
        return false;
    };

    /* EXEC 1*/

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error in arm control.";
        return false;
    };
   
    // 2) Going to grasp pose
    
    /* PLAN 2*/

    if(!this->panda_softhand_client.call_slerp_service(grasp_pose, pre_grasp_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly!";
        return false;
    };

    /* WAIT 1*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error wait in arm control.";
        return false;
    };

    /* EXEC 2*/

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error in arm control.";
        return false;
    };
    
    // 3) Performing simple grasp with planning, executing and waiting

    /* PLAN 3*/

    if(!this->panda_softhand_client.call_hand_plan_service(1.0, 1.0, 2.0, this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not plan the simple grasp.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error plan in hand plan.";
        return false;
    };

    /* WAIT 2*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to grasp pose");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error wait in arm control.";
        return false;
    };

    /* EXEC 3*/

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj_hand) || !this->franka_ok){
        ROS_ERROR("Could not perform the grasping.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error plan in hand control.";
        return false;
    };

    /* PLAN 4*/

    if(!this->panda_softhand_client.call_slerp_service(pre_grasp_pose, grasp_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly!";
        return false;
    };
    
    /* WAIT 3 */

    if(!this->panda_softhand_client.call_hand_wait_service(ros::Duration(3.0)) || !this->franka_ok){
        ROS_ERROR("Could not perform the simple grasp.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error plan in hand wait.";
        return false;
    };

    /* EXEC 4 */

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error in arm control.";
        return false;
    };

    return true;
};

// Callback for throwing several objects listed in the YAML file
bool TaskSequencer::call_throwing_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    /* PLAN 2: PLanning to pre_throwing pose */

    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d vacuum_transform_aff; tf::poseMsgToEigen(this->vacuum_T, vacuum_transform_aff);
    Eigen::Affine3d pre_vacuum_transform_aff; tf::poseMsgToEigen(this->pre_vacuum_T, pre_vacuum_transform_aff);

    geometry_msgs::Pose pre_vacuum_pose; geometry_msgs::Pose vacuum_pose;
    tf::poseEigenToMsg(object_pose_aff * vacuum_transform_aff * pre_vacuum_transform_aff, pre_vacuum_pose);
    tf::poseEigenToMsg(object_pose_aff * vacuum_transform_aff, vacuum_pose);


    // Couting object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;
    std::cout << "Object orientation is \n" << object_pose_aff.rotation() << std::endl;

    // Setting zero pose as starting from present
    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;
    
    if(!this->panda_softhand_client.call_pose_service(pre_vacuum_pose, present_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false; 
    }
    
       
    /* EXEC 2: Going to prevacuuming pose */
    
    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre vacuuming pose.");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    /* PLAN 3: Planning to vacuuming pose */

    if(!this->panda_softhand_client.call_slerp_service(vacuum_pose, pre_vacuum_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified vacuum pose.");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly!";
        return false;
    }

    /* WAIT 2*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre vacuuming configuration from prethrowing configuration");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    /* EXEC 3: Going to vacuuming configuration */

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to vacuum pose.");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly! Error in arm control.";
        return false;
    }

    /*PLAN 4: Planning to prevacuuming pose*/

    if(!this->panda_softhand_client.call_pose_service(pre_vacuum_pose, vacuum_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_simple_grasp_task was NOT performed correctly!";
        return false; 
    }
    
    /* WAIT 3*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre vacuuming configuration from prethrowing configuration");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    std_msgs::Empty msg;
    pub_suction.publish(msg);
    
      
    /* EXEC 4: Going to Vacuuming configuration */
    
    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre vacuuming pose.");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly! Error in arm control.";
        return false;
    }
   
    /* PLAN 5: Planning to throwing configuration (joint plan) */
        
    std::vector<trajectory_msgs::JointTrajectoryPoint> traj_arm_points3 = (this->tmp_traj_arm).points;
    trajectory_msgs::JointTrajectoryPoint last_point3 = traj_arm_points3.back();
    
    int n = 7;
    std::vector<double> now_joints_end(n,0.0);

    // for(int i=0; i < now_joints_end.size(); i++){
    //     now_joints_end.at(i) = last_point3.positions[i];
    // }

    if(!this->panda_softhand_client.call_joint_service(this->throwing_joints, now_joints_end, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not lift to the specified pose.");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly!";
        return false;
    };
    
    /* WAIT 4 */

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre vacuuming configuration from prethrowing configuration");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }


    /* EXEC 5 */

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to prethrowing position");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly! Error in arm control.";
        return false;
    };
    
    /* WAIT 5*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre vacuuming configuration from prethrowing configuration");
        res.success = false;
        res.message = "The service call_throwing_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }
    
   
    // Getting the current ee throwing transform 
    std::cout << "Getting the current righ_hand_ee_link trasnform w.r.t the world" << std::endl;
    try {
		this->tf_listener_throwing.waitForTransform("/world", this->end_effector_name, ros::Time(0), ros::Duration(10.0) );
		this->tf_listener_throwing.lookupTransform("/world", this->end_effector_name, ros::Time(0), this->stamp_ee_transform_throwing);
        
        double yaw, pitch, roll;
        this->stamp_ee_transform_throwing.getBasis().getRPY(roll, pitch, yaw);
        tf::Quaternion q = this->stamp_ee_transform_throwing.getRotation();
        tf::Vector3 v = this->stamp_ee_transform_throwing.getOrigin();
        std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
        std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
                  << q.getZ() << ", " << q.getW() << "]" << std::endl
                  << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
                  << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;
  
        //print transform

    } catch (tf::TransformException ex){
      	ROS_ERROR("%s", ex.what());
      	ros::Duration(1.0).sleep();
        return false;
    }
    
    tf::Transform ee_transform(this->stamp_ee_transform_throwing.getRotation(), this->stamp_ee_transform_throwing.getOrigin());
    tf::transformTFToEigen(ee_transform, this->end_effector_state_throwing);
    
    // Print the current end-effector pose
	ROS_INFO_STREAM("Endeffector current throwing Translation: \n" << this->end_effector_state_throwing.translation());
	ROS_INFO_STREAM("Endeffector current throwing Rotation: \n" << this->end_effector_state_throwing.rotation());
    

    Eigen::Matrix3d eigen_rot_matrix = this->end_effector_state_throwing.rotation();
    Eigen::Matrix<double, 3, 3> rotation_matrix = eigen_rot_matrix;

    // Select the 2nd column

    Eigen::Matrix<double, 3, 1> dummy_vec(0.0,1.0,0.0);
    Eigen::Matrix<double, 3, 1> result = rotation_matrix*dummy_vec;
    
    // Assign the previous result to a vector of double

    std::vector<double> versor(3,0.0);
    
    for(int i=0; i < versor.size(); ++i){
        versor[i] = -result[i];
        std::cout << "the element of versor are: " << versor[i] << std::endl;
    }

    std::cout << "den" << sqrt(std::pow(versor[0],2)+std::pow(versor[1],2)) << std::endl;

    double den = sqrt(std::pow(versor[0],2)+std::pow(versor[1],2));
    double throwing_angle_rad = atan2(versor[2],den);
    
    std::cout << "The throwing angle [deg] is : " << throwing_angle_rad*(180.0/M_PI) << std::endl;

    // Activate the blowing-off function (FESTO), and stop the Venturi pump for suctioning (COVAL)
    
    std_msgs::Empty msg2;
    pub_blow.publish(msg2);

    /* PLAN 6*/

    // if(!this->panda_softhand_client.call_joint_service(this->place_joints, this->null_joints, this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not plan to the specified place joint config.");
    //     res.success = false;
    //     res.message = "The service call_simple_place_task was NOT performed correctly!";
    //     return false;
    // }
    
    // /*WAIT 5 */

    // if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){ // WAITING FOR END EXEC
    //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre vacuuming configuration from prethrowing configuration");
    //     res.success = false;
    //     res.message = "The service call_throwing_task was NOT performed correctly! Error wait in arm control.";
    //     return false;
    // }
    
    // /* EXEC 6*/

    // if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj) || !this->franka_ok){
    //     ROS_ERROR("Could not go to place joint config.");
    //     res.success = false;
    //     res.message = "The service call_simple_place_task was NOT performed correctly! Error in arm control.";
    //     return false;
    // }

    /* WAIT 6 */

    // if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
    //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to place joint config");
    //     res.success = false;
    //     res.message = "The service call_simple_place_task was NOT performed correctly! Error wait in arm control.";
    //     return false;
    // }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_place_task was correctly performed!";
 
    return true;
};

// Callback for set object task service
bool TaskSequencer::call_set_object(panda_softhand_msgs::set_object::Request &req, panda_softhand_msgs::set_object::Response &res){

    // Checking if the parsed map contains the requested object
    auto search = this->poses_map.find(req.object_name);
    if(search == this->poses_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my poses_map memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
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

// Callback for set place joints service
bool TaskSequencer::call_set_place(panda_softhand_msgs::set_object::Request &req, panda_softhand_msgs::set_object::Response &res){

    // Checking if the parsed map contains the requested object
    auto search = this->place_joints_map.find(req.object_name);
    if(search == this->place_joints_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my place_joints_map memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }

    // Setting the place joints as requested
    this->place_joints = this->place_joints_map.at(req.object_name);
    
    // Now, everything is ok
    ROS_INFO_STREAM("Place joints changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;

}

/**/
bool TaskSequencer::call_set_prethrowing_joints_place(panda_softhand_msgs::set_object::Request &req, panda_softhand_msgs::set_object::Response &res){
    
    // Checking if the parsed map contains the requested object

    auto search = this->pre_throwing_joints_map.find(req.object_name);
    if(search == this->pre_throwing_joints_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my pre_throwing_joints_map memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }

    // Setting the place joints as requested
    this->pre_throwing_joints = this->pre_throwing_joints_map.at(req.object_name);

    // Now, everything is ok
    ROS_INFO_STREAM("Place prethrowing joints changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;

};

bool TaskSequencer::call_set_vacuum_place(panda_softhand_msgs::set_object::Request &req, panda_softhand_msgs::set_object::Response &res){
    
    // Checking if the parsed map contains the requested object

    auto search = this->vacuum_pose_map.find(req.object_name);
    if(search == this->vacuum_pose_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my poses_map memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }

    // Setting the vacuum pose as requested
    this->vacuum_transform = this->vacuum_pose_map.at(req.object_name);

    // Converting the vacuum_transform vector to geometry_msgs Pose
    this->vacuum_T = this->convert_vector_to_pose(this->vacuum_transform);

    // Now, everything is ok
    
    ROS_INFO_STREAM("Vacuum pose changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;
    
};

bool TaskSequencer::call_set_throwing_joints_place(panda_softhand_msgs::set_object::Request &req, panda_softhand_msgs::set_object::Response &res){

    // Checking if the parsed map contains the requested object

    auto search = this->throwing_joints_map.find(req.object_name);
    if(search == this->throwing_joints_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my throwing_joints_map memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }

    // Setting the place joints as requested
    this->throwing_joints = this->throwing_joints_map.at(req.object_name);

    // Now, everything is ok
    ROS_INFO_STREAM("Place joints changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;
};

bool TaskSequencer::call_set_duty_cycle(panda_softhand_msgs::set_object::Request &req, panda_softhand_msgs::set_object::Response &res){
   
    auto search = this->duty_cycle_map.find(req.object_name);
    if(search == this->duty_cycle_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my duty_cycle_map memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }

    // Setting the place joints as requested
    this->duty_cycle = this->duty_cycle_map.at(req.object_name);
    std::cout<< "the duty cycle is: " <<  duty_cycle << std::endl;
    std::cout << "Publishing the duty cycle" << std::endl;

    std_msgs::UInt8 msg;
    msg.data = this->duty_cycle;
    pub_duty.publish(msg);

    // Now, everything is ok
    ROS_INFO_STREAM("Duty cycle changed. Duty set to " << req.object_name << ".");
    res.result = true;
    return res.result;

};

bool TaskSequencer::call_replace_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
       
  
    /* Computing the grasp and pregrasp pose and converting to geometry_msgs Pose */

    Eigen::Affine3d object_pose_aff; tf::poseMsgToEigen(this->object_pose_T, object_pose_aff);
    Eigen::Affine3d replace_transform_aff; tf::poseMsgToEigen(this->replace_hand_tool_T, replace_transform_aff);
    Eigen::Affine3d pre_replace_transform_aff; tf::poseMsgToEigen(this->pre_replace_hand_tool_T, pre_replace_transform_aff);

    geometry_msgs::Pose pre_replace_pose; geometry_msgs::Pose replace_pose;
    tf::poseEigenToMsg(object_pose_aff * replace_transform_aff * pre_replace_transform_aff, pre_replace_pose);
    tf::poseEigenToMsg(object_pose_aff * replace_transform_aff, replace_pose);

    // Couting object pose for debugging
    std::cout << "Object position is \n" << object_pose_aff.translation() << std::endl;
    std::cout << "Object orientation is \n" << object_pose_aff.rotation() << std::endl;

    // Setting zero pose as starting from present (it starts from home position)
    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;
    
    // 1) Going to pregrasp pose
    
    /* PLAN 1*/

    if(!this->panda_softhand_client.call_pose_service(pre_replace_pose, present_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly!";
        return false;
    };

    /* EXEC 1*/

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error in arm control.";
        return false;
    };
   
    // 2) Going to grasp pose
    
    /* PLAN 2*/

    if(!this->panda_softhand_client.call_slerp_service(replace_pose, pre_replace_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly!";
        return false;
    };

    /* WAIT 1*/

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to pre grasp from home joints");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error wait in arm control.";
        return false;
    };

    /* EXEC 2*/

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error in arm control.";
        return false;
    };
    
    // 3) Opening hand
    
    /*PLAN 3 */

    if(!this->panda_softhand_client.call_hand_plan_service(1.0, 1.0, 2.0, this->tmp_traj) || this->franka_ok){
        ROS_ERROR("Could plan the simple open.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    /* WAIT 2 */

    if(!this->panda_softhand_client.call_arm_wait_service(this->waiting_time) || !this->franka_ok){        // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to home joint config");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error wait in arm control.";
        return false;
    }

    /* EXEC 3 */

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj) || !this->franka_ok){
        ROS_ERROR("Could not perform the simple open.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error plan in hand control.";
        return false;
    }

    /* PLAN 4*/

    if(!this->panda_softhand_client.call_slerp_service(pre_replace_pose, replace_pose, false, this->tmp_traj_arm, this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not plan to the specified grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly!";
        return false;
    };
    
    /* WAIT 3 */

    if(!this->panda_softhand_client.call_hand_wait_service(ros::Duration(3.0)) || !this->franka_ok){
        ROS_ERROR("Could not perform the simple grasp.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error plan in hand wait.";
        return false;
    };

    /* EXEC 4 */

    if(!this->panda_softhand_client.call_arm_control_service(this->tmp_traj_arm) || !this->franka_ok){
        ROS_ERROR("Could not go to pre grasp pose.");
        res.success = false;
        res.message = "The service call_grasp_handtool was NOT performed correctly! Error in arm control.";
        return false;
    };

    return true;        
}

bool TaskSequencer::call_set_first_synergy(panda_softhand_msgs::set_object::Request &req, panda_softhand_msgs::set_object::Response &res){

    // Checking if the parsed map contains the requested object

    auto search = this->first_synergy_map.find(req.object_name);
    if(search == this->first_synergy_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my first synergy map memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }
    
    // Setting the first syn value as requested
    this->first_syn_value.data = this->first_synergy_map.at(req.object_name);

    // Now, everything is ok
    ROS_INFO_STREAM("First synergy value changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;
};

bool TaskSequencer::call_set_second_synergy(panda_softhand_msgs::set_object::Request &req, panda_softhand_msgs::set_object::Response &res){

    // Checking if the parsed map contains the requested object

    auto search = this->second_synergy_map.find(req.object_name);
    if(search == this->second_synergy_map.end()){
        ROS_WARN_STREAM("The object " << req.object_name << " is not present in my second synergy map memory; using the previously used one or default... Did you spell it correctly? Is it in the yaml?");
        res.result = false;
        return res.result;
    }
    
    // Setting the first syn value as requested
    this->second_syn_value.data = this->second_synergy_map.at(req.object_name);

    // Now, everything is ok
    ROS_INFO_STREAM("Second synergy value changed. Object set to " << req.object_name << ".");
    res.result = true;
    return res.result;

};

bool TaskSequencer::call_test_hand(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple test hand  task service with data = false?");
        res.success = true;
        res.message = "The service call test hand done correctly with false request!";
        return true;
    }

    /*PLAN 1*/

    if(!this->panda_softhand_client.call_hand_plan_service(this->first_syn_value.data, this->second_syn_value.data, 1.0, this->tmp_traj_hand) || !this->franka_ok){
        ROS_ERROR("Could not plan the simple open.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly! Error plan in hand plan.";
        return false;
    }

    /*EXEC 1*/

    if(!this->panda_softhand_client.call_hand_control_service(this->tmp_traj_hand) || !this->franka_ok){
        ROS_ERROR("Could not perform the call hand control service.");
        res.success = false;
        res.message = "The service call hand control service was NOT performed correctly! Error plan in hand control.";
        return false;
    }
    
    /*WAIT 1*/
    if(!this->panda_softhand_client.call_hand_wait_service(ros::Duration(3.0)) || !this->franka_ok){
        ROS_ERROR("Could not perform the hand wait service.");
        res.success = false;
        res.message = "The service call_hand_wait_service was NOT performed correctly! Error plan in hand wait.";
        return false;
    };
    
    return true;

}
// FK and IK Functions which makes use of MoveIt
geometry_msgs::Pose TaskSequencer::performFK(std::vector<double> joints_in){
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_model->getJointModelGroup(this->group_name);
    this->kinematic_state->setJointGroupPositions(joint_model_group, joints_in);
    const Eigen::Affine3d& end_effector_eigen = this->kinematic_state->getGlobalLinkTransform(this->end_effector_name);
    geometry_msgs::Pose end_effector_pose;
    tf::poseEigenToMsg(end_effector_eigen, end_effector_pose);
    return end_effector_pose;
}

bool TaskSequencer::performIK(geometry_msgs::Pose pose_in, double timeout, std::vector<double>& joints_out){
    Eigen::Isometry3d end_effector_state;
    tf::poseMsgToEigen(pose_in, end_effector_state);
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_model->getJointModelGroup(this->group_name);
    bool found_ik = this->kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if (!found_ik){
        ROS_ERROR("Could not find IK solution in TaskSequencer...");
        return false;
    }

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);
    this->kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);

    if (DEBUG){
        ROS_INFO("Found an IK solution in TaskSequencer: ");
        for (std::size_t i = 0; i < joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints_out[i]);
        }
    }

    return true;
}