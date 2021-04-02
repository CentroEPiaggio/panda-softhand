/* CONTROL SERVER - Creates all necessary service servers to command Panda and SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic Includes
#include "ros/ros.h"

// Object Includes
#include "panda_softhand_control/HandPlan_2.h"
#include "panda_softhand_control/HandControl_2.h"
#include "panda_softhand_control/ArmControl_2.h"
#include "panda_softhand_control/SlerpPlan_2.h"
#include "panda_softhand_control/PosePlan_2.h"
#include "panda_softhand_control/JointPlan_2.h"

/**********************************************
ROS NODE MAIN SERVICE SERVERS 
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_softhand_control_server_2");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the arm client pointer");

    std::string arm_jt_topic_2 = "/panda_arm_2/position_joint_trajectory_controller_2/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_2(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_jt_topic_2, true));

    ROS_INFO("Creating the hand client pointer");

    std::string hand_jt_topic_2 = "/gripper/joint_trajectory_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr_2(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(hand_jt_topic_2, true));

    ROS_INFO("Creating the hand plan and control objects");
    HandPlan_2 hand_plan_obj_2(nh_, 20, "gripper_synergy_joint");
    HandControl_2 hand_control_obj_2(nh_, hand_client_ptr_2);

    ROS_INFO("Creating the arm control object");
    ArmControl_2 arm_control_obj_2(nh_, arm_client_ptr_2);

    ROS_INFO("Creating the slerp plan object");
    SlerpPlan_2 slerp_plan_obj_2(nh_, "panda_arm_2", "panda_arm_2_EE", 60);

    ROS_INFO("Creating the pose plan object");
    PosePlan_2 pose_plan_obj_2(nh_, "panda_arm_2", "panda_arm_2_EE");

    ROS_INFO("Creating the joint plan object");
    JointPlan_2 joint_plan_obj_2(nh_, "panda_arm_2");
    
    ROS_INFO("Advertising the services");

    
    ros::ServiceServer hand_plan_service_2 = nh_.advertiseService("hand_plan_service_2", &HandPlan_2::call_hand_plan, &hand_plan_obj_2);
    ros::ServiceServer hand_wait_service_2 = nh_.advertiseService("hand_wait_service_2", &HandControl_2::call_hand_wait, &hand_control_obj_2);
    ros::ServiceServer hand_service_2 = nh_.advertiseService("hand_control_service_2", &HandControl_2::call_hand_control, &hand_control_obj_2);

    ros::ServiceServer arm_service_2 = nh_.advertiseService("arm_control_service_2", &ArmControl_2::call_arm_control, &arm_control_obj_2);
    ros::ServiceServer arm_wait_service_2 = nh_.advertiseService("arm_wait_service_2", &ArmControl_2::call_arm_wait, &arm_control_obj_2);

    ros::ServiceServer slerp_service_2 = nh_.advertiseService("slerp_plan_service_2", &SlerpPlan_2::call_slerp_plan, &slerp_plan_obj_2);
    ros::ServiceServer pose_service_2 = nh_.advertiseService("pose_plan_service_2", &PosePlan_2::call_pose_plan, &pose_plan_obj_2);
    ros::ServiceServer joint_service_2 = nh_.advertiseService("joint_plan_service_2", &JointPlan_2::call_joint_plan, &joint_plan_obj_2);

    ROS_INFO("The main service server is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while(ros::ok()){
        // Nothing to do here
    }

    spinner.stop();

    return 0;
}