/* TASK SERVER THROWING to call demo task service (DARKO-Stefano)*/
#include "ros/ros.h"
#include <iostream>
#include "ros/service_client.h"

// Object Includes
#include "panda_softhand_control/TaskSequencer.h"

/**********************************************
ROS NODE MAIN TASK SEQUENCE SERVER 
**********************************************/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "task_server_demo_only_pickandplace");

   ros::NodeHandle nh_;

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
   req_first_syn.object_name = "white_box";
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
   req_second_syn.object_name = "white_box";
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
   req_object1.object_name = "white_box";
   panda_softhand_msgs::set_object::Response resp_object1;
   
   bool success_call_set_object = task_sequencer_obj.call_set_object(req_object1,resp_object1);
   
   if(success_call_set_object){
      ROS_INFO_STREAM("Call_set_object service completed correctly: " << resp_object1.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }
 
   /* Update the place pose map for the OBJECT1*/

   panda_softhand_msgs::set_object::Request req_pose_place;
   req_pose_place.object_name = "white_box";
   panda_softhand_msgs::set_object::Response resp_pose_place;

   ROS_INFO("Call the simple place task for OBJECT3");
   
   bool success_call_set_pose_place_object1 = task_sequencer_obj.call_set_pose_place(req_pose_place,resp_pose_place);
   
   if(success_call_set_pose_place_object1){
      ROS_INFO_STREAM("Call_set_pose place service completed correctly: " << resp_pose_place.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }
   
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

   // ROS_INFO("Call the simple grasp task for OBJECT1!");
   
   // bool success = task_sequencer_obj.call_simple_pick_and_place_task(req,resp);
    
   // //Check the success and use of the response

   // if(success){
   //    ROS_INFO_STREAM("Test service completed correctly: " << resp.success);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the service");
   // }

   // #################### GO HOME #####################################

   // ROS_INFO("Call the simple home task!");
   
   // success_home = task_sequencer_obj.call_simple_home_task(req_home,resp_home);

   // if(success_home){
   //    ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the service");
   // }


   // ########################### CUP ##################################

  
   req_first_syn.object_name = "cup";

   ROS_INFO("Call the call_set_first_synergy");

   success_call_first_syn = task_sequencer_obj.call_set_first_synergy(req_first_syn,resp_first_syn);
   
   if(success_call_first_syn){
      ROS_INFO_STREAM("Call_set_first synergy service completed correctly: " << resp_first_syn.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }

   /* Update the value for the second synergy*/
   
   req_second_syn.object_name = "cup";

   ROS_INFO("Call the call_set_second_synergy");

   success_call_second_syn = task_sequencer_obj.call_set_second_synergy(req_second_syn,resp_second_syn);
   
   if(success_call_second_syn){
      ROS_INFO_STREAM("Call_set second synergy service completed correctly: " << resp_second_syn.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set second synergy service");
   }
   
   /* Update the pose map for the "CUP"*/
   
   req_object1.object_name = "cup";
   
   success_call_set_object = task_sequencer_obj.call_set_object(req_object1,resp_object1);
   
   if(success_call_set_object){
      ROS_INFO_STREAM("Call_set_object service completed correctly: " << resp_object1.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }

   ROS_INFO("Call the simple_grasp_move_cup!");
   
   bool success = task_sequencer_obj.call_simple_grasp_move_cup(req,resp);
    
   //Check the success and use of the response

   if(success){
      ROS_INFO_STREAM("Test service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }

   // #################### GO HOME #####################################

   ROS_INFO("Call the simple home task!");
   
   success_home = task_sequencer_obj.call_simple_home_task(req_home,resp_home);

   if(success_home){
      ROS_INFO_STREAM("Test service completed correctly: " << resp_home.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }


   // ######################### OVEN GEL #########################################
   
     /* Update the value for the first synergy*/
   
   panda_softhand_msgs::set_object::Request req_first_syn3;
   req_first_syn3.object_name = "oven_gel";
   panda_softhand_msgs::set_object::Response resp_first_syn3;

   ROS_INFO("Call the call_set_first_synergy");

   bool success_call_first_syn3 = task_sequencer_obj.call_set_first_synergy(req_first_syn3,resp_first_syn3);
   
   if(success_call_first_syn3){
      ROS_INFO_STREAM("Call_set_first synergy service completed correctly: " << resp_first_syn.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }

   /* Update the value for the second synergy*/
   
   panda_softhand_msgs::set_object::Request req_second_syn3;
   req_second_syn3.object_name = "oven_gel";
   panda_softhand_msgs::set_object::Response resp_second_syn3;

   ROS_INFO("Call the call_set_second_synergy");

   bool success_call_second_syn3 = task_sequencer_obj.call_set_second_synergy(req_second_syn3,resp_second_syn3);
   
   if(success_call_second_syn3){
      ROS_INFO_STREAM("Call_set second synergy service completed correctly: " << resp_second_syn.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set second synergy service");
   }
   
   /* Update the pose map for the OBJECT1*/
   
   panda_softhand_msgs::set_object::Request req_object3;
   req_object3.object_name = "oven_gel";
   panda_softhand_msgs::set_object::Response resp_object3;
   
   bool success_call_set_object3 = task_sequencer_obj.call_set_object(req_object3,resp_object3);
   
   if(success_call_set_object3){
      ROS_INFO_STREAM("Call_set_object service completed correctly: " << resp_object3.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }
 
   /* Update the place pose map for the OBJECT1*/

   panda_softhand_msgs::set_object::Request req_pose_place3;
   req_pose_place3.object_name = "oven_gel";
   panda_softhand_msgs::set_object::Response resp_pose_place3;

   ROS_INFO("Call the simple place task for OBJECT3");
   
   bool success_call_set_pose_place_object3 = task_sequencer_obj.call_set_pose_place(req_pose_place3,resp_pose_place3);
   
   if(success_call_set_pose_place_object3){
      ROS_INFO_STREAM("Call_set_pose place service completed correctly: " << resp_pose_place3.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call set object service");
   }


   /* 1) Call simple grasp task for OVEN GEL*/
   //Create the request and response object
   
   std_srvs::SetBool::Request req3;
   req3.data = true;
   std_srvs::SetBool::Response resp3;

   ROS_INFO("Call the simple grasp task for OVEN GEL!");
   
   success = task_sequencer_obj.call_simple_pick_and_place_task(req3,resp3);
    
   //Check the success and use of the response

   if(success){
      ROS_INFO_STREAM("Test service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }

   //

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
