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
   

   /*** Update the map for throwing OBJECT 1***/ 

   panda_softhand_control::set_object::Request req_throwing;
   req_throwing.object_name = "object1";
   panda_softhand_control::set_object::Response resp_throwing;

   ROS_INFO("Call the call_set_throwing_joints_place ");

   bool success_call_throwing = task_sequencer_obj.call_set_throwing_joints_place(req_throwing,resp_throwing);
   
   if(success_call_throwing){
      ROS_INFO_STREAM("Call_set_throwing_joints_place service completed correctly: " << resp_throwing.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_throwing_joints_place service");
   }

   

   panda_softhand_control::set_object::Request req_vacuum;
   req_vacuum.object_name = "object1";
   panda_softhand_control::set_object::Response resp_vacuum;

   ROS_INFO("Call the call_set_vacuum_place ");

   bool success_call_vacuum = task_sequencer_obj.call_set_vacuum_place(req_vacuum,resp_vacuum);
   
   if(success_call_vacuum){
      ROS_INFO_STREAM("Call_set_vacuum_place  service completed correctly: " << resp_vacuum.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_vacuum_place service");
   }


   panda_softhand_control::set_object::Request req_duty;
   req_duty.object_name = "object1";
   panda_softhand_control::set_object::Response resp_duty;

   ROS_INFO("Call the call_set_duty_cycle service");

   bool success_call_duty = task_sequencer_obj.call_set_duty_cycle(req_duty,resp_duty);
   
   if(success_call_duty){
      ROS_INFO_STREAM("Call_set_duty_cycle service completed correctly: " << resp_duty.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }

 
   /* 3) Going to throwing position (1) */
   
   ROS_INFO("Going to throwing position");

   bool success_throwing = task_sequencer_obj.call_throwing_task(req,resp);
   if(success_throwing){
      ROS_INFO_STREAM("Throwing service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the throwing service");
   }
   
   // Calling the place joint service 
   
   panda_softhand_control::set_object::Request req_joint_place;
   req_joint_place.object_name = "middle12";
   panda_softhand_control::set_object::Response resp_joint_place;

   ROS_INFO("Call the call_set_joint_place service");

   bool success_joint_place = task_sequencer_obj.call_set_place(req_joint_place,resp_joint_place);
   
   if(success_joint_place){
      ROS_INFO_STREAM("Call_set_joint_place service completed correctly: " << resp_joint_place.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }

   /* Going to middle12 joint configuration */


   ROS_INFO("Going to place joint configuration");
   
   bool success_place_joint = task_sequencer_obj.call_simple_place_task(req,resp);
   

   /*** Update the map for throwing OBJECT 2***/ 

   panda_softhand_control::set_object::Request req_throwing2;
   req_throwing2.object_name = "object2";
   panda_softhand_control::set_object::Response resp_throwing2;

   ROS_INFO("Call the call_set_throwing_joints_place ");

   bool success_call_throwing2 = task_sequencer_obj.call_set_throwing_joints_place(req_throwing2,resp_throwing2);
   
   if(success_call_throwing2){
      ROS_INFO_STREAM("Call_set_throwing_joints_place service completed correctly: " << resp_throwing2.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_throwing_joints_place service");
   }

   

   panda_softhand_control::set_object::Request req_vacuum2;
   req_vacuum2.object_name = "object2";
   panda_softhand_control::set_object::Response resp_vacuum2;

   ROS_INFO("Call the call_set_vacuum_place ");

   bool success_call_vacuum2 = task_sequencer_obj.call_set_vacuum_place(req_vacuum2,resp_vacuum2);
   
   if(success_call_vacuum2){
      ROS_INFO_STREAM("Call_set_vacuum_place  service completed correctly: " << resp_vacuum2.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_vacuum_place service");
   }


   panda_softhand_control::set_object::Request req_duty2;
   req_duty2.object_name = "object2";
   panda_softhand_control::set_object::Response resp_duty2;

   ROS_INFO("Call the call_set_duty_cycle service");

   bool success_call_duty2 = task_sequencer_obj.call_set_duty_cycle(req_duty2,resp_duty2);
   
   if(success_call_duty2){
      ROS_INFO_STREAM("Call_set_duty_cycle service completed correctly: " << resp_duty2.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }


   /* Going to throwing position ("object2") */
   
   ROS_INFO("Going to throwing position");

   std_srvs::SetBool::Request req2;
   req2.data = true;
   std_srvs::SetBool::Response resp2;

   bool success_throwing2 = task_sequencer_obj.call_throwing_task(req2,resp2);
   if(success_throwing2){
      ROS_INFO_STREAM("Throwing service for object2 completed correctly: " << resp2.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the throwing service for object2");
   }


   /* Going to middle12 joint configuration */


   ROS_INFO("Going to place joint configuration");
   
   bool success_place_joint2 = task_sequencer_obj.call_simple_place_task(req,resp);
   
   
   /*** Update the map for throwing OBJECT 3 ***/ 

   panda_softhand_control::set_object::Request req_throwing3;
   req_throwing3.object_name = "object3";
   panda_softhand_control::set_object::Response resp_throwing3;

   ROS_INFO("Call the call_set_throwing_joints_place ");

   bool success_call_throwing3 = task_sequencer_obj.call_set_throwing_joints_place(req_throwing3,resp_throwing3);
   
   if(success_call_throwing3){
      ROS_INFO_STREAM("Call_set_throwing_joints_place service completed correctly: " << resp_throwing2.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_throwing_joints_place service");
   }

   

   panda_softhand_control::set_object::Request req_vacuum3;
   req_vacuum3.object_name = "object3";
   panda_softhand_control::set_object::Response resp_vacuum3;

   ROS_INFO("Call the call_set_vacuum_place ");

   bool success_call_vacuum3 = task_sequencer_obj.call_set_vacuum_place(req_vacuum3,resp_vacuum3);
   
   if(success_call_vacuum3){
      ROS_INFO_STREAM("Call_set_vacuum_place  service completed correctly: " << resp_vacuum2.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_vacuum_place service");
   }


   panda_softhand_control::set_object::Request req_duty3;
   req_duty3.object_name = "object3";
   panda_softhand_control::set_object::Response resp_duty3;

   ROS_INFO("Call the call_set_duty_cycle service");

   bool success_call_duty3 = task_sequencer_obj.call_set_duty_cycle(req_duty3,resp_duty3);
   
   if(success_call_duty3){
      ROS_INFO_STREAM("Call_set_duty_cycle service completed correctly: " << resp_duty3.result);
   } else {
      ROS_INFO_STREAM("Failed to completed the call_set_duty_cycle service");
   }


  

   /* Going to throwing position ("object3") */
   
   ROS_INFO("Going to throwing position");

   std_srvs::SetBool::Request req3;
   req3.data = true;
   std_srvs::SetBool::Response resp3;

   bool success_throwing3 = task_sequencer_obj.call_throwing_task(req3,resp3);
   if(success_throwing3){
      ROS_INFO_STREAM("Throwing service for object2 completed correctly: " << resp3.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the throwing service for object2");
   }

   

   /* Going to middle12 joint configuration */

   ROS_INFO("Going to place joint configuration");
   
   bool success_place_joint3 = task_sequencer_obj.call_simple_place_task(req,resp);
  
   /**/

   ROS_INFO("Going to replacing position for the handtool");
   
   //TO DO
   bool success_replace = task_sequencer_obj.call_replace_task(req,resp);
    
   if(success_replace){
      ROS_INFO_STREAM("Replacing service completed correctly: " << resp.success);
   } else {
      ROS_INFO_STREAM("Failed to completed the replacing service");
   }

   ROS_INFO("Going to home position");
   
   bool success_home_position2 = task_sequencer_obj.call_simple_home_task(req,resp);

   while(ros::ok()){
         // Nothing to do here
   }

   spinner.stop();

   return 0;
}