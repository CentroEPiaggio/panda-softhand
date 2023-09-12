#ifndef ARM_CONTROL_TRIAL_H
#define ARM_CONTROL_TRIAL_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/RobotTrajectory.h>

class ArmControlTrial
{
public:
    ArmControlTrial();
    ~ArmControlTrial();
    void sendTrajectory(const trajectory_msgs::JointTrajectory trajectory);
    bool waitForCompletion();
 

private:
    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> ac_;
    ros::Time start_; 
};

#endif  // ARM_CONTROL_TRIAL_H
