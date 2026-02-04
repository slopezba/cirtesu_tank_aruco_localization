#pragma once

#include <ros/ros.h>

#include <aruco_opencv_msgs/ArucoDetection.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>
#include <map>
#include <vector>

class ArucoMapLocalization
{
public:
  ArucoMapLocalization(ros::NodeHandle& nh);

private:

  // ===== ROS =====

  ros::NodeHandle nh_;
  ros::Subscriber aruco_sub_;
  ros::Publisher marker_pub_;
  ros::Publisher pose_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ===== Frames & topics =====

  std::string world_frame_;
  std::string base_frame_;
  std::string camera_frame_;

  std::string aruco_topic_;
  std::string marker_topic_;
  std::string pose_topic_;

  // ===== Parameters =====

  double alpha_pos_;
  double alpha_yaw_;

  double sigma_dist_;

  double aruco_yaw_offset_;

  std::string mesh_path_;
  std::vector<double> mesh_scale_;
  std::vector<double> mesh_pos_;
  double mesh_yaw_;

  // ===== Aruco map =====

  std::map<int, Eigen::Vector3d> aruco_map_;

  // ===== Filter memory =====

  bool first_measurement_;
  Eigen::Vector3d prev_pos_;
  double prev_yaw_;

  // ===== Methods =====

  void arucoCallback(const aruco_opencv_msgs::ArucoDetection::ConstPtr& msg);

  double normalizeAngle(double a);
};

