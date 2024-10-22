/* TASK SERVER THROWING to call demo task service (DARKO-Stefano)*/
#include "ros/ros.h"
#include <iostream>
#include "ros/service_client.h"

// Object Includes
#include "panda_softhand_control/TaskSequencer.h"

// Ros messages
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int64.h"
#include "utils/utilities.h"

#include <thread>
#include "ros/callback_queue.h"
#include "actionlib/client/simple_action_client.h"

#include <franka_msgs/ErrorRecoveryActionGoal.h>

boost::shared_ptr<ros::AsyncSpinner> classical_spinner; // Spinner for the global queue
boost::shared_ptr<ros::AsyncSpinner> spinner_state;     // Spinner for the custom queue

// Global callback queue for robot state
ros::CallbackQueue robot_state_queue;

bool need_recovery = false;

std_msgs::Int64 finished_task;

// Function to handle error recovery using action client
void performRecovery(actionlib::SimpleActionClient<franka_msgs::ErrorRecoveryAction> &client)
{
   franka_msgs::ErrorRecoveryGoal recovery_goal;

   ROS_INFO("Sending recovery goal to Franka arm...");
   client.sendGoal(recovery_goal);

   bool finished_before_timeout = client.waitForResult(ros::Duration(20.0));

   if (finished_before_timeout)
   {
      actionlib::SimpleClientGoalState state = client.getState();
      ROS_INFO("Franka recovery finished: %s", state.toString().c_str());
   }
   else
   {
      ROS_ERROR("Franka recovery did not finish before the timeout.");
   }
}

void ackCallback(const std_msgs::Int64 &msg);

void robotStateCallback(const franka_msgs::FrankaState::ConstPtr &msg)
{
   if (msg->robot_mode != 2 && msg->robot_mode != 5) // Check if not in Idle or Automatic mode
   {
      need_recovery = true;
      ROS_WARN_THROTTLE(5.0, "Robot is in error state. Recovery needed.");
   }
   else
   {
      need_recovery = false;
      ROS_WARN_THROTTLE(5.0, "Robot is in a valid mode. No recovery needed.");
   }
}

/**********************************************
ROS NODE MAIN TASK SEQUENCE SERVER
**********************************************/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "task_server_demo_only_pickandthrow");

   ros::NodeHandle nh_;
   ros::NodeHandle nh_state_;

   // Set callback queue
   ros::CallbackQueue queue;
   nh_state_.setCallbackQueue(&queue);

   std::vector<double> first_object_vec;
   geometry_msgs::Pose first_object_pose;

   std::vector<double> second_object_vec;
   geometry_msgs::Pose second_object_pose;

   // Initialize action client for recovery
   actionlib::SimpleActionClient<franka_msgs::ErrorRecoveryAction> franka_recovery_client("/panda_arm/franka_control/error_recovery", true);

   // Initialize finished task value

   finished_task.data = 0;
   //
   ros::Publisher pub_pose_Giorgio = nh_.advertise<geometry_msgs::PoseStamped>("/throw_node/throw_pos_des", 1);
   ros::Publisher pub_int_Giorgio = nh_.advertise<std_msgs::Int64>("/throw_node/throw_state", 1);
   ros::Subscriber robot_state_sub = nh_state_.subscribe("/panda_arm/franka_state_controller/franka_states", 1, robotStateCallback);

   // Listening to the completion of the task
   ros::Subscriber action_task_completion = nh_.subscribe("/throw_node/throw_ack", 1, &ackCallback);

   ROS_INFO("Creating the TaskSequencer object");

   TaskSequencer task_sequencer_obj(nh_);

   ROS_INFO("The main task sequence client is running. Running as fast as possible!");

   // Parsing the object position for throwing
   if (!ros::param::get("/task_sequencer/first_object_pose", first_object_vec))
   {
      ROS_WARN("The param 'first_object_pose' not found in param server! ");
   }

   // Converting the grasp_transform vector to geometry_msgs Pose
   first_object_pose = task_sequencer_obj.convert_vector_to_pose(first_object_vec);

   if (!ros::param::get("/task_sequencer/second_object_pose", second_object_vec))
   {
      ROS_WARN("The param 'second_object_pose' not found in param server! ");
   }

   // Converting the grasp_transform vector to geometry_msgs Pose
   second_object_pose = task_sequencer_obj.convert_vector_to_pose(second_object_vec);

   // Spinner for the custom queue, using a single thread
   spinner_state.reset(new ros::AsyncSpinner(1, &queue)); // Spin only the robotStateCallback in a separate thread
   spinner_state->start();

   // ROS Async spinner (necessary for processing callbacks inside the service callbacks)

   classical_spinner.reset(new ros::AsyncSpinner(3)); // Spin other callbacks in the global queue
   classical_spinner->start();

   // Start spinning this in a separate thread
   // Conditionally perform recovery if needed
   if (need_recovery)
   {
      performRecovery(franka_recovery_client);
   }
   else
   {
      ROS_INFO("Recovery not needed. Robot is in a valid mode.");
   }
   sleep(1.0);
   bool switch_done = task_sequencer_obj.switch_controllers("panda_arm","computed_torque_controller");

   // ############################# GO HOME ############################################

   // std_srvs::SetBool::Request req_home;
   // req_home.data = true;
   // std_srvs::SetBool::Response resp_home;

   // ROS_INFO("Call the simple home task!");

   // // bool success_home = task_sequencer_obj.call_simple_home_task(req_home, resp_home);

   // // if (success_home)
   // // {
   // //    ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   // // }
   // // else
   // // {
   // //    ROS_INFO_STREAM("Failed to completed the service");
   // // }

   // /* 1) Call simple grasp task for OBJECT1*/
   // // Create the request and response object

   // darko_manipulation_msgs::grasp::Request req;
   // darko_manipulation_msgs::grasp::Response res;

   // req.grasp_pose.position.x = 0.438;
   // req.grasp_pose.position.y = 0.700;
   // req.grasp_pose.position.z = 0.317;
   // req.grasp_pose.orientation.x = 0.581;
   // req.grasp_pose.orientation.y = 0.739;
   // req.grasp_pose.orientation.z = 0.236;
   // req.grasp_pose.orientation.w = -0.247;
   // req.data = true;

   // ROS_INFO("Call the simple grasp task for OBJECT1!");

   // bool success = task_sequencer_obj.call_simple_grasp_task(req, res);

   // // Check the success and use of the response

   // if (success)
   // {
   //    ROS_INFO_STREAM("Test service completed correctly: " << res.success);
   // }
   // else
   // {
   //    ROS_INFO_STREAM("Failed to completed the service");
   // }

   // success = task_sequencer_obj.call_simple_grasp_task(req, res);

   // // Check the success and use of the response

   // if (success)
   // {
   //    ROS_INFO_STREAM("Test service completed correctly: " << res.success);
   // }
   // else
   // {
   //    ROS_INFO_STREAM("Failed to completed the service");
   // }

   // bool switch_done = task_sequencer_obj.switch_controllers("panda_arm", "computed_torque_controller");

   // // //
   // /*THROW 2*/
   geometry_msgs::PoseStamped first_object_pose_stamped;
   first_object_pose_stamped.pose.position.x = first_object_pose.position.x;
   first_object_pose_stamped.pose.position.y = first_object_pose.position.y;
   first_object_pose_stamped.pose.position.z = first_object_pose.position.z;

   first_object_pose_stamped.pose.orientation.x = first_object_pose.orientation.x;
   first_object_pose_stamped.pose.orientation.y = first_object_pose.orientation.y;
   first_object_pose_stamped.pose.orientation.z = first_object_pose.orientation.z;
   first_object_pose_stamped.pose.orientation.w = first_object_pose.orientation.w;

   // pub_pose_Giorgio.publish(first_object_pose_stamped);

   // sleep(0.1);

   // std_msgs::Int64 giorgio_int;
   // giorgio_int.data = 1;
   // pub_int_Giorgio.publish(giorgio_int);

   // //
   // ros::Rate rate(5);
   // while(finished_task.data != 1){
   //    ROS_INFO("I haven't finished to perform the throw phase yet!");
   //    rate.sleep();
   // }
   // sleep(1.0);

   // switch_done = task_sequencer_obj.switch_controllers("panda_arm","position_joint_trajectory_controller");
   // sleep(1.0);
   // franka_msgs::ErrorRecoveryActionGoal recovery;
   // pub_franka_recovery.publish(recovery);
   // sleep(3.0);

   // // // // // // ##############################################################################

   // // // // // // ##################### OBJECT2 ###################################

   // /* Update the value for the first synergy*/

   // panda_softhand_msgs::set_object::Request req_first_syn2;
   // req_first_syn2.object_name = "oven_gel";
   // panda_softhand_msgs::set_object::Response resp_first_syn2;

   // ROS_INFO("Call the call_set_first_synergy");

   // bool success_call_first_syn2 = task_sequencer_obj.call_set_first_synergy(req_first_syn2,resp_first_syn2);

   // if(success_call_first_syn2){
   //    ROS_INFO_STREAM("Call_set_first synergy service completed correctly: " << resp_first_syn2.result);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   // }

   // /* Update the value for the second synergy*/

   // panda_softhand_msgs::set_object::Request req_second_syn2;
   // req_second_syn2.object_name = "oven_gel";
   // panda_softhand_msgs::set_object::Response resp_second_syn2;

   // ROS_INFO("Call the call_set_first_synergy");

   // bool success_call_second_syn2 = task_sequencer_obj.call_set_second_synergy(req_second_syn2,resp_second_syn2);

   // if(success_call_second_syn2){
   //    ROS_INFO_STREAM("Call_set second synergy service completed correctly: " << resp_second_syn2.result);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the call set second synergy service");
   // }

   // /* Update the pose map for the OBJECT2*/

   // panda_softhand_msgs::set_object::Request req_object2;
   // req_object2.object_name = "oven_gel";
   // panda_softhand_msgs::set_object::Response resp_object2;

   // bool success_call_set_object2 = task_sequencer_obj.call_set_object(req_object2,resp_object2);

   // if(success_call_set_object2){
   //    ROS_INFO_STREAM("Call_set_object service completed correctly: " << resp_object2.result);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the call set object service");
   // }

   // // // ##################### GO HOME ####################################

   // ROS_INFO("Call the simple home task!");

   // success_home = task_sequencer_obj.call_simple_home_task(req_home,resp_home);

   // if(success_home){
   //    ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the service");
   // }

   // // /* 2) Call simple grasp task for OBJECT2*/
   // //Create the request and response object

   // std_srvs::SetBool::Request req2;
   // req2.data = true;
   // std_srvs::SetBool::Response resp2;

   // ROS_INFO("Call the simple grasp task for OBJECT2");

   // bool success2 = task_sequencer_obj.call_simple_grasp_task(req2,resp2);

   // //Check the success and use of the response

   // if(success2){
   //    ROS_INFO_STREAM("Test service completed correctly: " << resp2.success);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the service");
   // }

   // switch_done = task_sequencer_obj.switch_controllers("panda_arm","computed_torque_controller");
   // sleep(1.0);

   // finished_task.data = 0;

   // /*THROW 2*/

   // geometry_msgs::PoseStamped second_object_pose_stamped;
   // second_object_pose_stamped.pose.position.x = second_object_pose.position.x;
   // second_object_pose_stamped.pose.position.y = second_object_pose.position.y;
   // second_object_pose_stamped.pose.position.z = second_object_pose.position.z;

   // second_object_pose_stamped.pose.orientation.x = second_object_pose.orientation.x;
   // second_object_pose_stamped.pose.orientation.y = second_object_pose.orientation.y;
   // second_object_pose_stamped.pose.orientation.z = second_object_pose.orientation.z;
   // second_object_pose_stamped.pose.orientation.w = second_object_pose.orientation.w;

   // pub_pose_Giorgio.publish(second_object_pose_stamped);

   // sleep(0.1);

   // giorgio_int.data = 1;
   // pub_int_Giorgio.publish(giorgio_int);

   // while(finished_task.data != 1){
   //    ROS_INFO("I haven't finished to perform the throw phase yet!");
   //    rate.sleep();
   // }
   // sleep(1.0);

   // switch_done = task_sequencer_obj.switch_controllers("panda_arm","position_joint_trajectory_controller");
   // sleep(1.0);
   // // #############################GO HOME ################################################

   // pub_franka_recovery.publish(recovery);
   // sleep(3.0);

   // ROS_INFO("Call the simple home task!");

   // success_home = task_sequencer_obj.call_simple_home_task(req_home,resp_home);

   // if(success_home){
   //    ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the service");
   // }

   // ##############################################################################
   ros::waitForShutdown();
   classical_spinner->stop();
   spinner_state->stop();
   return 0;
}

void ackCallback(const std_msgs::Int64 &msg)
{

   finished_task.data = msg.data;
}
