#include "utils/utilities.h"

geometry_msgs::Pose applyRotationOffset(const geometry_msgs::Pose &pose, double angle_degrees, const Eigen::Vector3d &axis)
{
   double angle_radians = angle_degrees * M_PI / 180.0;
   Eigen::AngleAxisd rotation(angle_radians, axis); //   axis = Eigen::Vector3d::UnitX() or Eigen::Vector3d::UnitY() or Eigen::Vector3d::UnitZ()
   Eigen::Affine3d offset_eigen = Eigen::Affine3d::Identity();
   offset_eigen.rotate(rotation);

   // Convert the geometry_msgs::Pose into Eigen
   Eigen::Affine3d pose_transform_aff;
   tf::poseMsgToEigen(pose, pose_transform_aff);

   // Post multiplied the input pose for the RotationOffset
   Eigen::Affine3d offset_pose = pose_transform_aff * offset_eigen;

   // Convert the Eigen into geometry_msgs::Pose
   geometry_msgs::Pose offset_pose_msg;
   tf::poseEigenToMsg(offset_pose, offset_pose_msg);

   return offset_pose_msg;
}

geometry_msgs::Pose applyDisplacementOffset(const geometry_msgs::Pose &pose, const Eigen::Vector3d &displacement)
{
   // Create an Eigen Affine3d transformation for the displacement
   Eigen::Affine3d offset_eigen = Eigen::Affine3d::Identity();
   offset_eigen.translate(displacement);

   // Convert the geometry_msgs::Pose into Eigen
   Eigen::Affine3d pose_transform_aff;
   tf::poseMsgToEigen(pose, pose_transform_aff);

   // Post multiply the input pose for the displacement offset
   Eigen::Affine3d offset_pose = pose_transform_aff * offset_eigen;

   // Convert the Eigen into geometry_msgs::Pose
   geometry_msgs::Pose offset_pose_msg;
   tf::poseEigenToMsg(offset_pose, offset_pose_msg);

   return offset_pose_msg;
}

geometry_msgs::Pose getPosemsg(std::string parent, std::string child)
{  
   tf::StampedTransform pose_stamped_transform;
   Eigen::Affine3d pose_eigen;
   tf::TransformListener tf_listener;
   try
   {
      tf_listener.waitForTransform(parent, child, ros::Time(0), ros::Duration(10.0));
      tf_listener.lookupTransform(parent, child, ros::Time(0), pose_stamped_transform);
   }
   catch (tf::TransformException ex)
   {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
   }
   // Convert TF to Eigen
   tf::Transform pose_tf(pose_stamped_transform.getRotation(), pose_stamped_transform.getOrigin());
   tf::transformTFToEigen(pose_tf, pose_eigen);

   //  Get the Eigen to Msg
   geometry_msgs::Pose pose_msg;
   tf::poseEigenToMsg(pose_eigen, pose_msg);
   return pose_msg;
}

geometry_msgs::Pose posemsgMultiplication(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2){
   
   geometry_msgs::Pose output_pose;

   Eigen::Affine3d pose1_aff;
   tf::poseMsgToEigen(pose1, pose1_aff);

   Eigen::Affine3d pose2_aff;
   tf::poseMsgToEigen(pose2, pose2_aff);
   
   Eigen::Affine3d output_eigen = pose1_aff * pose2_aff;
   Eigen::Quaterniond pippo;
   pippo = output_eigen.linear();
   pippo.normalized();
   output_eigen.linear().normalized();

   std::cout << "norm is " << output_eigen.linear().norm() << std::endl;
   tf::poseEigenToMsg(output_eigen, output_pose);
   
   return output_pose;
}
