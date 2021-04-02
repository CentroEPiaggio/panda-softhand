


// Basic Includes
#include "ros/ros.h"
#include "panda_softhand_control/GraspFailure.h"


/**********************************************
ROS NODE MAIN GRASP
**********************************************/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "panda_softhand_grasp");

    ros::NodeHandle nh_;

    ROS_INFO("Creating the GraspFailure object");

    GraspFailure grasp_failure_obj(nh_);
    
    ROS_INFO("The main task sequence client is running. Running as fast as possible!");

    // ROS Async spinnedr (necessary for processing callbacks inside the service callbacks)
    ros::AsyncSpinner spinner(4);
    spinner.start();

    while(ros::ok()){
        // Nothing to do here
    }

    spinner.stop();

    return 0;
}