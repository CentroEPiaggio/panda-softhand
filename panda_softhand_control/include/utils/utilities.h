#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

/**
* @brief This h file contains utilities for parsing single parameters from 
* the ROS parameter server
*
*/

geometry_msgs::Pose applyRotationOffset(const geometry_msgs::Pose &pose, double angle_degrees, const Eigen::Vector3d &axis);

geometry_msgs::Pose applyDisplacementOffset(const geometry_msgs::Pose &pose, const Eigen::Vector3d &displacement);

geometry_msgs::Pose getPosemsg(std::string parent, std::string child);

geometry_msgs::Pose posemsgMultiplication(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);


#endif //UTILITIES_H
