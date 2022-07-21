#include "ros/ros.h"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <cmath>

int main(int argc, char** argv){

  ros::init(argc, argv, "my_throwing_angle_listener");
  ros::NodeHandle node;
  tf::TransformListener tf_listener_throwing;
  tf::StampedTransform stamp_ee_transform_throwing;
  Eigen::Affine3d end_effector_state_throwing;


  while (node.ok()){

    // Getting the current ee throwing transform 
    
    try {
		tf_listener_throwing.waitForTransform("/world", "/right_hand_ee_link", ros::Time(0), ros::Duration(10.0));
		tf_listener_throwing.lookupTransform("/world", "/right_hand_ee_link", ros::Time(0), stamp_ee_transform_throwing);
        
        double yaw, pitch, roll;
        stamp_ee_transform_throwing.getBasis().getRPY(roll, pitch, yaw);     
     
    } catch (tf::TransformException ex){
      	ROS_ERROR("%s", ex.what());
      	ros::Duration(1.0).sleep();
        return false;
    }
    
    tf::Transform ee_transform(stamp_ee_transform_throwing.getRotation(),stamp_ee_transform_throwing.getOrigin());
    tf::transformTFToEigen(ee_transform, end_effector_state_throwing);
    

    Eigen::Matrix3d eigen_rot_matrix = end_effector_state_throwing.rotation();
    Eigen::Matrix<double, 3, 3> rotation_matrix = eigen_rot_matrix;
    

    // Select the 2nd column

    Eigen::Matrix<double, 3, 1> dummy_vec(0.0,1.0,0.0);
    Eigen::Matrix<double, 3, 1> result = rotation_matrix*dummy_vec;
    
    // Assign the previous result to a vector of double

    std::vector<double> versor(3,0.0);
    
    for(int i=0; i < versor.size(); ++i){
        versor[i] = -result[i];
    }
    
    double den = sqrt(std::pow(versor[0],2)+std::pow(versor[1],2));
    double throwing_angle_rad = atan2(versor[2],den);
   
    std::cout << "The throwing angle in deg is: " << throwing_angle_rad*(180.0/M_PI) << "\n";    
  }
  return 0;
};