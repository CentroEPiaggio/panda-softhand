/* CONTROL SERVER - Creates all necessary service servers to command Panda and SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

// Basic Includes
#include "ros/ros.h"

// Object Includes
#include "panda_softhand_control/HandPlan_1.h"
#include "panda_softhand_control/HandControl_1.h"
#include "panda_softhand_control/ArmControl_1.h"
#include "panda_softhand_control/SlerpPlan_1.h"
#include "panda_softhand_control/PosePlan_1.h"
#include "panda_softhand_control/JointPlan_1.h"

/**********************************************
ROS NODE MAIN SERVICE SERVERS 
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_softhand_control_server");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the arm client pointer");

    std::string arm_jt_topic_1 = "/panda_arm_1/position_joint_trajectory_controller_1/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_1(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_jt_topic_1, true));

    ROS_INFO("Creating the hand client pointer");

    std::string hand_jt_topic_1 = "/right_hand/joint_trajectory_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_ptr_1(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(hand_jt_topic_1, true));

    ROS_INFO("Creating the hand plan and control objects");
    HandPlan_1 hand_plan_obj_1(nh_, 20, "right_hand_synergy_joint");
    HandControl_1 hand_control_obj_1(nh_, hand_client_ptr_1);

    ROS_INFO("Creating the arm control object");
    ArmControl_1 arm_control_obj_1(nh_, arm_client_ptr_1);

    ROS_INFO("Creating the slerp plan object");
    SlerpPlan_1 slerp_plan_obj_1(nh_, "panda_arm_1", "panda_arm_1_EE", 60);

    ROS_INFO("Creating the pose plan object");
    PosePlan_1 pose_plan_obj_1(nh_, "panda_arm_1", "panda_arm_1_EE");

    ROS_INFO("Creating the joint plan object");
    JointPlan_1 joint_plan_obj_1(nh_, "panda_arm_1");
    
    ROS_INFO("Advertising the services");

    
    ros::ServiceServer hand_plan_service = nh_.advertiseService("hand_plan_service", &HandPlan_1::call_hand_plan, &hand_plan_obj_1);
    ros::ServiceServer hand_wait_service = nh_.advertiseService("hand_wait_service", &HandControl_1::call_hand_wait, &hand_control_obj_1);
    ros::ServiceServer hand_service = nh_.advertiseService("hand_control_service", &HandControl_1::call_hand_control, &hand_control_obj_1);

    ros::ServiceServer arm_service = nh_.advertiseService("arm_control_service", &ArmControl_1::call_arm_control, &arm_control_obj_1);
    ros::ServiceServer arm_wait_service = nh_.advertiseService("arm_wait_service", &ArmControl_1::call_arm_wait, &arm_control_obj_1);

    ros::ServiceServer slerp_service = nh_.advertiseService("slerp_plan_service", &SlerpPlan_1::call_slerp_plan, &slerp_plan_obj_1);
    ros::ServiceServer pose_service = nh_.advertiseService("pose_plan_service", &PosePlan_1::call_pose_plan, &pose_plan_obj_1);
    ros::ServiceServer joint_service = nh_.advertiseService("joint_plan_service", &JointPlan_1::call_joint_plan, &joint_plan_obj_1);

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