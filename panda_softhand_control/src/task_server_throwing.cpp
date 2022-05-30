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
      ROS_INFO_STREAM("Graspingg service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the grasping service");
   }
     
   /* 3) Going to throwing position (1) */
   
   ROS_INFO("Going to throwing position");
   
   //TO DO
   bool success_throwing = task_sequencer_obj.call_throwing_task(req,resp);
   if(success_throwing){
      ROS_INFO_STREAM("Throwing service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the throwing service");
   }



   






   while(ros::ok()){
         // Nothing to do here
   }

   spinner.stop();

   return 0;
}