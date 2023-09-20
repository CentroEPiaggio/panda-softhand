/* TASK SERVER THROWING to call demo task service (DARKO-Stefano)*/
#include "ros/ros.h"
#include <iostream>
#include "ros/service_client.h"

// Object Includes
#include "panda_softhand_control/TaskSequencer.h"

// Ros messages
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Int64.h"

std_msgs::Int64 finished_task;

void ackCallback(const std_msgs::Int64& msg);

/**********************************************
ROS NODE MAIN TASK SEQUENCE SERVER 
**********************************************/

int main(int argc, char **argv)
{
   ros::init(argc, argv, "task_server_demo");

   ros::NodeHandle nh_;
   
   // Initialize finished task value

   finished_task.data = 0;
   //
   ros::Publisher pub_pose_Giorgio = nh_.advertise<geometry_msgs::PoseStamped>("/throw_node/throw_pos_des", 1);
   ros::Publisher pub_int_Giorgio = nh_.advertise<std_msgs::Int64>("/throw_node/throw_state", 1);
   
   // Listening to the completion of the task
   ros::Subscriber action_task_completion = nh_.subscribe("/throw_node/throw_ack", 1, &ackCallback);

   ROS_INFO("Creating the TaskSequencer object");

   TaskSequencer task_sequencer_obj(nh_);

   ROS_INFO("The main task sequence client is running. Running as fast as possible!");

   // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
   ros::AsyncSpinner spinner(2);
   spinner.start();
   
   // bool switch_done = task_sequencer_obj.switch_controllers("panda_arm","computed_torque_controller");

   // ##################### OBJECT1 ###################################

   /* Update the value for the first synergy*/
   
   panda_softhand_msgs::set_object::Request req_first_syn;
   req_first_syn.object_name = "object1";
   panda_softhand_msgs::set_object::Response resp_first_syn;

   ROS_INFO("Call the call_set_first_synergy");

   bool success_call_first_syn = task_sequencer_obj.call_set_first_synergy(req_first_syn,resp_first_syn);
   
   if(success_call_first_syn){
      ROS_INFO_STREAM("Call_set_first synergy service completed correctly: " << resp_first_syn.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }

   /* Update the value for the second synergy*/
   
   panda_softhand_msgs::set_object::Request req_second_syn;
   req_second_syn.object_name = "object1";
   panda_softhand_msgs::set_object::Response resp_second_syn;

   ROS_INFO("Call the call_set_first_synergy");

   bool success_call_second_syn = task_sequencer_obj.call_set_second_synergy(req_second_syn,resp_second_syn);
   
   if(success_call_second_syn){
      ROS_INFO_STREAM("Call_set second synergy service completed correctly: " << resp_second_syn.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set second synergy service");
   }
   
   /* Update the pose map for the OBJECT1*/
   
   panda_softhand_msgs::set_object::Request req_object1;
   req_object1.object_name = "object1";
   panda_softhand_msgs::set_object::Response resp_object1;
   
   bool success_call_set_object = task_sequencer_obj.call_set_object(req_object1,resp_object1);
   
   if(success_call_set_object){
      ROS_INFO_STREAM("Call_set_object service completed correctly: " << resp_object1.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }
   // Switch controller 
   // switch_done = task_sequencer_obj.switch_controllers("panda_arm","position_joint_trajectory_controller");
   
   // ############################# GO HOME ############################################

   std_srvs::SetBool::Request req_home;
   req_home.data = true;
   std_srvs::SetBool::Response resp_home;

   ROS_INFO("Call the simple home task!");
   
   bool success_home = task_sequencer_obj.call_simple_home_task(req_home,resp_home);

   if(success_home){
      ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }

   /* 1) Call simple grasp task for OBJECT1*/
   //Create the request and response object
   
   std_srvs::SetBool::Request req;
   req.data = true;
   std_srvs::SetBool::Response resp;

   ROS_INFO("Call the simple grasp task for OBJECT1!");
   
   bool success = task_sequencer_obj.call_simple_grasp_task(req,resp);
    
   //Check the success and use of the response

   if(success){
      ROS_INFO_STREAM("Test service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }

   // bool switch_done = task_sequencer_obj.switch_controllers("panda_arm","computed_torque_controller");
   
   // sleep(0.3);

   //
   /*THROW 2*/
   // geometry_msgs::PoseStamped giorgio_msg;
   // giorgio_msg.pose.position.x = 1.3;
   // giorgio_msg.pose.position.y = 0.0;
   // giorgio_msg.pose.position.z = 0.0;

   // pub_pose_Giorgio.publish(giorgio_msg);

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
   // // // // ##############################################################################


   // // // // ##################### OBJECT2 ###################################

   /* Update the value for the first synergy*/
   
   panda_softhand_msgs::set_object::Request req_first_syn2;
   req_first_syn2.object_name = "object2";
   panda_softhand_msgs::set_object::Response resp_first_syn2;

   ROS_INFO("Call the call_set_first_synergy");

   bool success_call_first_syn2 = task_sequencer_obj.call_set_first_synergy(req_first_syn2,resp_first_syn2);
   
   if(success_call_first_syn2){
      ROS_INFO_STREAM("Call_set_first synergy service completed correctly: " << resp_first_syn2.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }

   /* Update the value for the second synergy*/
   
   panda_softhand_msgs::set_object::Request req_second_syn2;
   req_second_syn2.object_name = "object2";
   panda_softhand_msgs::set_object::Response resp_second_syn2;

   ROS_INFO("Call the call_set_first_synergy");

   bool success_call_second_syn2 = task_sequencer_obj.call_set_second_synergy(req_second_syn2,resp_second_syn2);
   
   if(success_call_second_syn2){
      ROS_INFO_STREAM("Call_set second synergy service completed correctly: " << resp_second_syn2.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set second synergy service");
   }
   
   /* Update the pose map for the OBJECT2*/
   
   panda_softhand_msgs::set_object::Request req_object2;
   req_object2.object_name = "object2";
   panda_softhand_msgs::set_object::Response resp_object2;
   
   bool success_call_set_object2 = task_sequencer_obj.call_set_object(req_object2,resp_object2);
   
   if(success_call_set_object2){
      ROS_INFO_STREAM("Call_set_object service completed correctly: " << resp_object2.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }
   
   // ##################### GO HOME ####################################

   ROS_INFO("Call the simple home task!");
   
   success_home = task_sequencer_obj.call_simple_home_task(req_home,resp_home);

   if(success_home){
      ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }

   // /* 2) Call simple grasp task for OBJECT2*/
   //Create the request and response object

   std_srvs::SetBool::Request req2;
   req2.data = true;
   std_srvs::SetBool::Response resp2;

   ROS_INFO("Call the simple grasp task for OBJECT2");
   
   bool success2 = task_sequencer_obj.call_simple_grasp_task(req2,resp2);
    
   //Check the success and use of the response

   if(success2){
      ROS_INFO_STREAM("Test service completed correctly: " << resp2.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }

   // #############################GO HOME ################################################

   ROS_INFO("Call the simple home task!");
   
   success_home = task_sequencer_obj.call_simple_home_task(req_home,resp_home);

   if(success_home){
      ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }
   

   // ##################### OBJECT3 ###################################

   /* Update the value for the first synergy*/
   
   panda_softhand_msgs::set_object::Request req_first_syn3;
   req_first_syn3.object_name = "object3";
   panda_softhand_msgs::set_object::Response resp_first_syn3;

   ROS_INFO("Call the call_set_first_synergy");

   bool success_call_first_syn3 = task_sequencer_obj.call_set_first_synergy(req_first_syn3,resp_first_syn3);
   
   if(success_call_first_syn3){
      ROS_INFO_STREAM("Call_set_first synergy service completed correctly: " << resp_first_syn3.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }
   
   /* Update the value for the second synergy*/
   
   panda_softhand_msgs::set_object::Request req_second_syn3;
   req_second_syn3.object_name = "object3";
   panda_softhand_msgs::set_object::Response resp_second_syn3;

   ROS_INFO("Call the call_set_first_synergy");

   bool success_call_second_syn3 = task_sequencer_obj.call_set_second_synergy(req_second_syn3,resp_second_syn3);
   
   if(success_call_second_syn3){
      ROS_INFO_STREAM("Call_set second synergy service completed correctly: " << resp_second_syn3.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set second synergy service");
   }
   
   /* Update the pose map for the OBJECT3*/
   
   panda_softhand_msgs::set_object::Request req_object3;
   req_object3.object_name = "object3";
   panda_softhand_msgs::set_object::Response resp_object3;
   
   bool success_call_set_object3 = task_sequencer_obj.call_set_object(req_object3,resp_object3);
   
   if(success_call_set_object3){
      ROS_INFO_STREAM("Call_set_object service completed correctly: " << resp_object3.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }

   /* Update the place pose map for the OBJECT3*/
   
   bool success_call_set_place_object3 = task_sequencer_obj.call_set_pose_place(req_object3,resp_object3);
   
   if(success_call_set_place_object3){
      ROS_INFO_STREAM("Call_set_pose place service completed correctly: " << resp_object3.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }

   /* 2) Call simple grasp task for OBJECT3*/
   //Create the request and response object

   std_srvs::SetBool::Request req3;
   req3.data = true;
   std_srvs::SetBool::Response resp3;

   ROS_INFO("Call the simple place task for OBJECT3");
   
   bool success3 = task_sequencer_obj.call_simple_pick_and_place_task(req3,resp3);
    
   //Check the success and use of the response

   if(success3){
      ROS_INFO_STREAM("Test service completed correctly: " << resp3.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }

   // // ############################# GO HOME ##############################################

   ROS_INFO("Call the simple home task!");
   
   success_home = task_sequencer_obj.call_simple_home_task(req_home,resp_home);

   if(success_home){
      ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }

   // ##############################################################################

   spinner.stop();
   return 0;
}

void ackCallback(const std_msgs::Int64& msg){

   finished_task.data = msg.data;
}
