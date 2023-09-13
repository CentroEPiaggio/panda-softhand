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
   ros::init(argc, argv, "task_server_demo");

   ros::NodeHandle nh_;
    

   ROS_INFO("Creating the TaskSequencer object");

   TaskSequencer task_sequencer_obj(nh_);

   ROS_INFO("The main task sequence client is running. Running as fast as possible!");

   // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
   ros::AsyncSpinner spinner(2);
   spinner.start();
    
   /* 1) Going to home position */
   //Create the request and response object
    
   std_srvs::SetBool::Request req;
   req.data = true;
   std_srvs::SetBool::Response resp;

   ROS_INFO("Test softhand2");
   
   bool success = task_sequencer_obj.call_test_hand(req,resp);
    
   //Check the success and use of the response

   if(success){
      ROS_INFO_STREAM("Test service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the service");
   }
    
   while(ros::ok()){
         // Nothing to do here
   }

   spinner.stop();
   return 0;
}