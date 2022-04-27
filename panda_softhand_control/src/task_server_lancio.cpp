/* TASK SERVER - Creates the task sequence client to perform some tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com  */

//

// Basic Includes
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
    ros::init(argc, argv, "panda_softhand_task_server_lancio");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the TaskSequencer object");

    TaskSequencer task_sequencer_obj(nh_);

    ROS_INFO("The main task sequence client is running. Running as fast as possible!");

    // ROS Async spinner (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    std::cout << std::boolalpha;
    /* 1) Going to home position*/
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
    
    /* 2) Going to grasping position for handtool*/
    
    ROS_INFO("Going to grasping position");
    bool success_handtool = task_sequencer_obj.call_simple_grasp_task(req,resp);
    
    // Check the success_handtool and use of the response

    if(success_handtool){
       ROS_INFO_STREAM("Handtool service completed correctly: " << resp.success);
    } else {
       ROS_INFO_STREAM("Failed to completed the handtool service");
    }
    

    /*3) Going to vacuuming position */
    ROS_INFO("Going to vacuuming position");
    bool success_vacuuming = task_sequencer_obj.call_simple_vacuum_task(req,resp);
    
    // Check the success_handtool and use of the response

    if(success_vacuuming){
       ROS_INFO_STREAM("Vacuuming service completed correctly: " << resp.success);
    } else {
       ROS_INFO_STREAM("Failed to completed the Vacuuming service");
    }
   
   /*4) Going to prethrowing position */

    ROS_INFO("Going to prethrowing position");
    bool success_prethrowing = task_sequencer_obj.call_simple_prethrowing_task(req,resp);
    
    // Check the success_handtool and use of the response

    if(success_prethrowing){
       ROS_INFO_STREAM("Prethrowing service completed correctly: " << resp.success);
    } else {
       ROS_INFO_STREAM("Failed to completed the Prethrowing service");
    }


    /*5) Going to throwing position */
    ROS_INFO("Going to throwing position");
    bool success_throwing = task_sequencer_obj.call_simple_place_task(req,resp);
    
    // Check the success_throwing and use of the response

    if(success_throwing){
       ROS_INFO_STREAM("Throwing service completed correctly: " << resp.success);
    } else {
       ROS_INFO_STREAM("Failed to completed the Throwing service");
    }
    

    while(ros::ok()){
         // Nothing to do here
     }

    spinner.stop();

    return 0;
}