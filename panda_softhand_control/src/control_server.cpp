/* CONTROL SERVER - Creates all necessary service servers to command Panda and SoftHand
Authors: George Jose Pollayil - Mathew Jose Pollayil -  Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com , stefano.angeli@ing.unipi.it */

// Basic Includes
#include "ros/ros.h"

// Object Includes
#include "panda_softhand_control/HandPlanFirstSyn.h"
#include "panda_softhand_control/HandPlanSecondSyn.h"
#include "panda_softhand_control/HandControlFirstSyn.h"
#include "panda_softhand_control/HandControlSecondSyn.h"
#include "panda_softhand_control/ArmControl.h"
#include "panda_softhand_control/SlerpPlan.h"
#include "panda_softhand_control/PosePlan.h"
#include "panda_softhand_control/JointPlan.h"

/**********************************************
ROS NODE MAIN SERVICE SERVERS 
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_softhand_control_server");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the arm client pointer");

    std::string arm_jt_topic = "/panda_arm/position_joint_trajectory_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> arm_client_ptr_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(arm_jt_topic, true));

    ROS_INFO("Creating the hand client pointer");

    std::string hand_first_syn_jt_topic = "/right_hand/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_first_syn_ptr_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(hand_first_syn_jt_topic, true));
    
    std::string hand_second_syn_jt_topic = "/right_hand/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/follow_joint_trajectory/";
    boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> hand_client_second_syn_ptr_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(hand_second_syn_jt_topic, true));

    ROS_INFO("Creating the hand plan First Syn and Second Syn and control objects");
    HandPlanFirstSyn hand_plan_first_syn_obj(nh_, 20, "qbhand2m1_synergy_joint");
    HandControlFirstSyn hand_control_first_syn_obj(nh_, hand_client_first_syn_ptr_);
    
    //
    HandPlanSecondSyn hand_plan_second_syn_obj(nh_, 20, "qbhand2m1_manipulation_joint");
    HandControlSecondSyn hand_control_second_syn_obj(nh_, hand_client_first_syn_ptr_);

    ROS_INFO("Creating the arm control object");
    ArmControl arm_control_obj(nh_, arm_client_ptr_);

    ROS_INFO("Creating the slerp plan object");
    SlerpPlan slerp_plan_obj(nh_, "panda_arm", "right_hand_ee_link", 60);

    ROS_INFO("Creating the pose plan object");
    PosePlan pose_plan_obj(nh_, "panda_arm", "right_hand_ee_link");

    ROS_INFO("Creating the joint plan object");
    JointPlan joint_plan_obj(nh_, "panda_arm");
    
    ROS_INFO("Advertising the services");

    ros::ServiceServer hand_plan_first_syn_service = nh_.advertiseService("hand_plan_service_first_syn", &HandPlanFirstSyn::call_hand_plan, &hand_plan_first_syn_obj);
    ros::ServiceServer hand_wait_first_syn_service = nh_.advertiseService("hand_wait_service_first_syn", &HandControlFirstSyn::call_hand_wait, &hand_control_first_syn_obj);
    ros::ServiceServer hand_control_first_syn_service = nh_.advertiseService("hand_control_service_first_syn", &HandControlFirstSyn::call_hand_control, &hand_control_first_syn_obj);
    
    ros::ServiceServer hand_plan_second_syn_service = nh_.advertiseService("hand_plan_service_second_syn", &HandPlanSecondSyn::call_hand_plan, &hand_plan_second_syn_obj);

    ros::ServiceServer arm_service = nh_.advertiseService("arm_control_service", &ArmControl::call_arm_control, &arm_control_obj);
    ros::ServiceServer arm_wait_service = nh_.advertiseService("arm_wait_service", &ArmControl::call_arm_wait, &arm_control_obj);

    ros::ServiceServer slerp_service = nh_.advertiseService("slerp_plan_service", &SlerpPlan::call_slerp_plan, &slerp_plan_obj);
    ros::ServiceServer pose_service = nh_.advertiseService("pose_plan_service", &PosePlan::call_pose_plan, &pose_plan_obj);
    ros::ServiceServer joint_service = nh_.advertiseService("joint_plan_service", &JointPlan::call_joint_plan, &joint_plan_obj);

    ROS_INFO("The main service server is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while(ros::ok()){
        // Nothing to do here
    }

    spinner.stop();

    return 0;
}