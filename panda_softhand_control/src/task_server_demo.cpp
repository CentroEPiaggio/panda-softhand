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
   ros::init(argc, argv, "task_server_demo");

   ros::NodeHandle nh_;
    
   ROS_INFO("Creating the TaskSequencer object");

   TaskSequencer task_sequencer_obj(nh_);

   ROS_INFO("The main task sequence client is running. Running as fast as possible!");

   // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
   ros::AsyncSpinner spinner(2);
   spinner.start();
    
   // /* Update the value for the first synergy*/
   
   // panda_softhand_msgs::set_object::Request req_first_syn;
   // req_first_syn.object_name = "object1";
   // panda_softhand_msgs::set_object::Response resp_first_syn;

   // ROS_INFO("Call the call_set_first_synergy");

   // bool success_call_first_syn = task_sequencer_obj.call_set_first_synergy(req_first_syn,resp_first_syn);
   
   // if(success_call_first_syn){
   //    ROS_INFO_STREAM("Call_set_first synergy service completed correctly: " << resp_first_syn.result);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   // }

   // /* Update the value for the second synergy*/
   
   // panda_softhand_msgs::set_object::Request req_second_syn;
   // req_second_syn.object_name = "object1";
   // panda_softhand_msgs::set_object::Response resp_second_syn;

   // ROS_INFO("Call the call_set_first_synergy");

   // bool success_call_second_syn = task_sequencer_obj.call_set_second_synergy(req_second_syn,resp_second_syn);
   
   // if(success_call_second_syn){
   //    ROS_INFO_STREAM("Call_set second synergy service completed correctly: " << resp_second_syn.result);
   // } else {
   //    ROS_INFO_STREAM("Failed to completed the call set second synergy service");
   // }

   /* 1) Test Grasp*/
   //Create the request and response object
    
   std_srvs::SetBool::Request req;
   req.data = true;
   std_srvs::SetBool::Response resp;

   ROS_INFO("Test softhand2");
   
   bool success = task_sequencer_obj.call_simple_grasp_task(req,resp);
    
   //Check the success and use of the response

   if(success){
      ROS_INFO_STREAM("Test service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }





   spinner.stop();
   return 0;
}