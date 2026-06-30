#pragma once

#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Dense>

#include <map>
#include <memory>
#include <string>
#include <vector>

class ArucoMapLocalization : public rclcpp::Node
{
public:
  explicit ArucoMapLocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  using ArucoDetection = aruco_opencv_msgs::msg::ArucoDetection;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  struct ArucoMarker
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    double yaw{0.0};
  };

  void loadParameters();
  void loadArucoMap();
  void publishMapMarkers(const std::vector<int>& visible_ids = {});
  void arucoCallback(const ArucoDetection::SharedPtr msg);
  Eigen::Vector3d transformMapPosition(
      const Eigen::Vector3d& position,
      const geometry_msgs::msg::TransformStamped& transform) const;
  double transformMapYaw(
      double yaw,
      const geometry_msgs::msg::TransformStamped& transform) const;
  Eigen::Vector3d baseVectorToWorld(const Eigen::Vector3d& vector, double yaw) const;
  std::string namespacedFrame(const std::string& frame_name) const;
  double normalizeAngle(double angle) const;

  rclcpp::Subscription<ArucoDetection>::SharedPtr aruco_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::CreateTimerROS> tf_timer_interface_;

  std::string world_frame_;
  std::string marker_frame_;
  std::string base_frame_;
  std::string camera_frame_;

  std::string aruco_topic_;
  std::string marker_topic_;
  std::string pose_topic_;

  double alpha_pos_;
  double alpha_yaw_;
  double sigma_dist_;
  double aruco_yaw_offset_;
  double multi_marker_covariance_gain_;
  double pose_covariance_xy_;
  double pose_covariance_z_;
  double pose_covariance_yaw_;

  std::string mesh_path_;
  std::vector<double> mesh_scale_;
  std::vector<double> mesh_pos_;
  double mesh_yaw_;

  std::map<int, ArucoMarker> aruco_map_;

  bool first_measurement_;
  Eigen::Vector3d prev_pos_;
  double prev_yaw_;
};
