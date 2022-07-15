/* TASK SERVER THROWING to call throwing task service (DARKO-Stefano)*/
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
    ros::init(argc, argv, "task_server_throwing");

    ros::NodeHandle nh_;
    

    ROS_INFO("Creating the TaskSequencer object");

    TaskSequencer task_sequencer_obj(nh_);

    ROS_INFO("The main task sequence client is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
   /* 1) Going to home position */
   //Create the request and response object
    
    std_srvs::SetBool::Request req;
    req.data = true;
    std_srvs::SetBool::Response resp;

    ROS_INFO("Going to home position");
   
   bool success_home_position = task_sequencer_obj.call_simple_home_task(req,resp);
    
   //Check the success and use of the response

   if(success_home_position){
      ROS_INFO_STREAM("Home service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }
    
   /* 2) Going to grasping position */

   ROS_INFO("Going to grasping position for the handtool");
   
   //TO DO
   bool success_grasping = task_sequencer_obj.call_grasp_handtool_task(req,resp);
    
   if(success_grasping){
      ROS_INFO_STREAM("Grasping service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the grasping service");
   }
     
   // // /* 3) Going to throwing position (1) */
   
   ROS_INFO("Going to throwing position");

   bool success_throwing = task_sequencer_obj.call_throwing_task(req,resp);
   if(success_throwing){
      ROS_INFO_STREAM("Throwing service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the throwing service");
   }
   

   
   // /* 4) Set new prethrowing, throwing joint configurations and vacuum transform before calling another throwing task service */
   
   
   panda_softhand_control::set_object::Request req_prethrowing;
   req_prethrowing.object_name = "object2";
   panda_softhand_control::set_object::Response resp_prethrowing;
   
   ROS_INFO("Call the call_set_prethrowing_joints_place ");

   bool success_call_prethrowing = task_sequencer_obj.call_set_prethrowing_joints_place(req_prethrowing,resp_prethrowing);
   
   if(success_call_prethrowing){
      ROS_INFO_STREAM("Call_set_prethrowing_joints_place service completed correctly: " << resp_prethrowing.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_prethrowing_joints_place service");
   }
   
   //

   panda_softhand_control::set_object::Request req_throwing;
   req_throwing.object_name = "object2";
   panda_softhand_control::set_object::Response resp_throwing;

   ROS_INFO("Call the call_set_throwing_joints_place ");

   bool success_call_throwing = task_sequencer_obj.call_set_throwing_joints_place(req_throwing,resp_throwing);
   
   if(success_call_throwing){
      ROS_INFO_STREAM("Call_set_throwing_joints_place service completed correctly: " << resp_throwing.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_throwing_joints_place service");
   }

   

   panda_softhand_control::set_object::Request req_vacuum;
   req_vacuum.object_name = "object2";
   panda_softhand_control::set_object::Response resp_vacuum;

   ROS_INFO("Call the call_set_vacuum_place ");

   bool success_call_vacuum = task_sequencer_obj.call_set_vacuum_place(req_vacuum,resp_vacuum);
   
   if(success_call_vacuum){
      ROS_INFO_STREAM("Call_set_vacuum_place  service completed correctly: " << resp_vacuum.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_vacuum_place service");
   }
   
   std::cout << "I would like to throw object2" << std::endl;

   /* 5) Going to throwing position ("object2") */
   
   // ROS_INFO("Going to throwing position");

   // std_srvs::SetBool::Request req2;
   // req2.data = true;
   // std_srvs::SetBool::Response resp2;

   // bool success_throwing2 = task_sequencer_obj.call_throwing_task(req2,resp2);
   // if(success_throwing2){
   //    ROS_INFO_STREAM("Throwing service for object2 completed correctly: " << resp2.success);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the throwing service for object2");
   // }


   while(ros::ok()){
         // Nothing to do here
   }

   spinner.stop();

   return 0;
}