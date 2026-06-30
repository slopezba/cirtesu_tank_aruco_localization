#include "cirtesu_tank_aruco_localization/aruco_map_localization.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>

ArucoMapLocalization::ArucoMapLocalization(const rclcpp::NodeOptions& options)
: rclcpp::Node("aruco_map_localization", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  first_measurement_(true),
  prev_pos_(Eigen::Vector3d::Zero()),
  prev_yaw_(0.0)
{
  tf_timer_interface_ = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(),
      get_node_timers_interface());
  tf_buffer_.setCreateTimerInterface(tf_timer_interface_);

  loadParameters();
  loadArucoMap();

  aruco_sub_ = create_subscription<ArucoDetection>(
      aruco_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ArucoMapLocalization::arucoCallback, this, std::placeholders::_1));

  marker_pub_ = create_publisher<MarkerArray>(marker_topic_, 1);
  pose_pub_ = create_publisher<PoseWithCovarianceStamped>(pose_topic_, 1);

  RCLCPP_INFO(get_logger(), "Aruco map localization C++ node started");
}

void ArucoMapLocalization::loadParameters()
{
  const auto get_double = [this](const std::string& name, double default_value) {
    if (!has_parameter(name)) {
      return declare_parameter<double>(name, default_value);
    }

    return get_parameter(name).as_double();
  };

  const auto get_string = [this](const std::string& name, const std::string& default_value) {
    if (!has_parameter(name)) {
      return declare_parameter<std::string>(name, default_value);
    }

    return get_parameter(name).as_string();
  };

  const auto get_double_array =
      [this](const std::string& name, const std::vector<double>& default_value) {
        if (!has_parameter(name)) {
          return declare_parameter<std::vector<double>>(name, default_value);
        }

        return get_parameter(name).as_double_array();
      };

  alpha_pos_ = get_double("alpha_pos", 0.4);
  alpha_yaw_ = get_double("alpha_yaw", 0.3);
  sigma_dist_ = get_double("sigma_dist", 1.0);
  aruco_yaw_offset_ = get_double("aruco_yaw_offset", 0.0);
  multi_marker_covariance_gain_ = get_double("multi_marker_covariance_gain", 0.25);
  pose_covariance_xy_ = get_double("pose_covariance_xy", 0.13);
  pose_covariance_z_ = get_double("pose_covariance_z", 0.005);
  pose_covariance_yaw_ = get_double("pose_covariance_yaw", 0.25);

  world_frame_ = namespacedFrame(get_string("world_frame", "world_ned"));
  marker_frame_ = get_string("marker_frame", world_frame_);
  base_frame_ = namespacedFrame(get_string("base_frame", "base_link"));
  camera_frame_ = namespacedFrame(get_string("camera_frame", "down_camera/camera"));

  aruco_topic_ = get_string("aruco_topic", "/girona500/down_camera/aruco_detections");
  marker_topic_ = get_string("marker_topic", "/tandem_girona/aruco_map_markers");
  pose_topic_ = get_string("pose_topic", "/girona500/navigator/aruco_pose");

  mesh_path_ = get_string("mesh_path", "package://girona500_description/meshes/cirtesu.dae");
  mesh_scale_ = get_double_array("mesh_scale", {1.0, 1.0, 1.0});
  mesh_pos_ = get_double_array("mesh_pos", {0.0, 0.0, 0.2});
  mesh_yaw_ = get_double("mesh_yaw", 0.0);

  if (mesh_scale_.size() != 3) {
    RCLCPP_WARN(get_logger(), "mesh_scale must contain 3 values. Using default scale.");
    mesh_scale_ = {1.0, 1.0, 1.0};
  }

  if (mesh_pos_.size() != 3) {
    RCLCPP_WARN(get_logger(), "mesh_pos must contain 3 values. Using default position.");
    mesh_pos_ = {0.0, 0.0, 0.2};
  }
}

void ArucoMapLocalization::loadArucoMap()
{
  const auto aruco_map_parameters = list_parameters({"aruco_map"}, 2);

  for (const auto& parameter_name : aruco_map_parameters.names) {
    rclcpp::Parameter marker_position;
    if (!get_parameter(parameter_name, marker_position)) {
      continue;
    }

    const auto marker_id = parameter_name.substr(std::string("aruco_map.").size());

    if (marker_position.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
      RCLCPP_WARN(
          get_logger(),
          "Ignoring aruco_map.%s because it is not a double array.",
          marker_id.c_str());
      continue;
    }

    const auto values = marker_position.as_double_array();
    if (values.size() != 3 && values.size() != 4) {
      RCLCPP_WARN(
          get_logger(),
          "Ignoring aruco_map.%s because it does not contain 3 or 4 values.",
          marker_id.c_str());
      continue;
    }

    ArucoMarker marker;
    marker.position = Eigen::Vector3d(values[0], values[1], values[2]);
    marker.yaw = values.size() == 4 ? values[3] : 0.0;
    aruco_map_[std::stoi(marker_id)] = marker;
  }

  RCLCPP_INFO(get_logger(), "Loaded %zu ArUco map markers.", aruco_map_.size());
}

double ArucoMapLocalization::normalizeAngle(double angle) const
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d ArucoMapLocalization::transformMapPosition(
    const Eigen::Vector3d& position,
    const geometry_msgs::msg::TransformStamped& transform) const
{
  geometry_msgs::msg::PointStamped map_point;
  map_point.header.frame_id = marker_frame_;
  map_point.header.stamp = transform.header.stamp;
  map_point.point.x = position.x();
  map_point.point.y = position.y();
  map_point.point.z = position.z();

  geometry_msgs::msg::PointStamped world_point;
  tf2::doTransform(map_point, world_point, transform);

  return Eigen::Vector3d(world_point.point.x, world_point.point.y, world_point.point.z);
}

double ArucoMapLocalization::transformMapYaw(
    double yaw,
    const geometry_msgs::msg::TransformStamped& transform) const
{
  geometry_msgs::msg::PoseStamped map_pose;
  map_pose.header.frame_id = marker_frame_;
  map_pose.header.stamp = transform.header.stamp;

  tf2::Quaternion q_map;
  q_map.setRPY(0.0, 0.0, yaw);
  map_pose.pose.orientation = tf2::toMsg(q_map);

  geometry_msgs::msg::PoseStamped world_pose;
  tf2::doTransform(map_pose, world_pose, transform);

  tf2::Quaternion q_world;
  tf2::fromMsg(world_pose.pose.orientation, q_world);

  double roll = 0.0;
  double pitch = 0.0;
  double world_yaw = 0.0;
  tf2::Matrix3x3(q_world).getRPY(roll, pitch, world_yaw);

  return normalizeAngle(world_yaw);
}

Eigen::Vector3d ArucoMapLocalization::baseVectorToWorld(
    const Eigen::Vector3d& vector,
    double yaw) const
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  return Eigen::Vector3d(
      c * vector.x() - s * vector.y(),
      s * vector.x() + c * vector.y(),
      vector.z());
}

std::string ArucoMapLocalization::namespacedFrame(const std::string& frame_name) const
{
  if (frame_name.empty()) {
    return frame_name;
  }

  const bool absolute_frame = frame_name.front() == '/';
  auto frame = frame_name;
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  while (!frame.empty() && frame.back() == '/') {
    frame.pop_back();
  }

  if (absolute_frame || frame.empty()) {
    return frame;
  }

  auto node_namespace = std::string(get_namespace());
  while (!node_namespace.empty() && node_namespace.front() == '/') {
    node_namespace.erase(node_namespace.begin());
  }
  while (!node_namespace.empty() && node_namespace.back() == '/') {
    node_namespace.pop_back();
  }

  if (node_namespace.empty() || frame.rfind(node_namespace + "/", 0) == 0) {
    return frame;
  }

  return node_namespace + "/" + frame;
}

void ArucoMapLocalization::publishMapMarkers(const std::vector<int>& visible_ids)
{
  MarkerArray array;
  const auto marker_stamp = get_clock()->now();

  visualization_msgs::msg::Marker mesh;
  mesh.header.frame_id = marker_frame_;
  mesh.header.stamp = marker_stamp;
  mesh.ns = "cirtesu_mesh";
  mesh.id = 1000;
  mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  mesh.mesh_resource = mesh_path_;
  mesh.mesh_use_embedded_materials = true;
  mesh.scale.x = mesh_scale_[0];
  mesh.scale.y = mesh_scale_[1];
  mesh.scale.z = mesh_scale_[2];
  mesh.pose.position.x = mesh_pos_[0];
  mesh.pose.position.y = mesh_pos_[1];
  mesh.pose.position.z = mesh_pos_[2];

  tf2::Quaternion q_mesh;
  q_mesh.setRPY(M_PI, 0.0, mesh_yaw_);
  mesh.pose.orientation = tf2::toMsg(q_mesh);
  mesh.color.a = 1.0;

  array.markers.push_back(mesh);

  for (const auto& aruco : aruco_map_) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = marker_frame_;
    marker.header.stamp = marker_stamp;
    marker.ns = "aruco_map";
    marker.id = aruco.first;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.pose.position.x = aruco.second.position.x();
    marker.pose.position.y = aruco.second.position.y();
    marker.pose.position.z = aruco.second.position.z();
    tf2::Quaternion q_marker;
    q_marker.setRPY(0.0, 0.0, aruco.second.yaw);
    marker.pose.orientation = tf2::toMsg(q_marker);
    marker.scale.x = 0.30;
    marker.scale.y = 0.30;
    marker.scale.z = 0.08;
    marker.color.g = 1.0;
    marker.color.a = 0.8;

    if (std::find(visible_ids.begin(), visible_ids.end(), aruco.first) != visible_ids.end()) {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
    }

    array.markers.push_back(marker);
  }

  marker_pub_->publish(array);
}

void ArucoMapLocalization::arucoCallback(const ArucoDetection::SharedPtr msg)
{
  if (msg->markers.empty()) {
    publishMapMarkers({});
    return;
  }

  std::vector<int> visible_ids;

  for (const auto& marker : msg->markers) {
    visible_ids.push_back(marker.marker_id);
  }

  publishMapMarkers(visible_ids);

  geometry_msgs::msg::TransformStamped tf_base_cam;
  geometry_msgs::msg::TransformStamped tf_world_map;

  try {
    tf_base_cam = tf_buffer_.lookupTransform(
        base_frame_,
        camera_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "TF base->camera not available: %s",
        ex.what());
    return;
  }

  try {
    tf_world_map = tf_buffer_.lookupTransform(
        world_frame_,
        marker_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.2));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "TF world->aruco map not available: %s",
        ex.what());
    return;
  }

  std::vector<Eigen::Vector3d> estimates;
  std::vector<double> weights;
  std::vector<double> yaw_estimates;
  std::vector<double> yaw_weights;

  for (const auto& marker : msg->markers) {
    const int id = marker.marker_id;

    if (!aruco_map_.count(id)) {
      continue;
    }

    geometry_msgs::msg::PoseStamped cam_marker;
    cam_marker.header = msg->header;
    cam_marker.pose = marker.pose;

    geometry_msgs::msg::PoseStamped base_marker;

    try {
      tf2::doTransform(cam_marker, base_marker, tf_base_cam);
    } catch (const tf2::TransformException&) {
      continue;
    }

    const auto& map_marker = aruco_map_[id];
    const Eigen::Vector3d world_marker = transformMapPosition(map_marker.position, tf_world_map);
    const Eigen::Vector3d base_pos(
        base_marker.pose.position.x,
        base_marker.pose.position.y,
        base_marker.pose.position.z);
    const double dist = base_pos.norm();
    const double weight = std::exp(-(dist * dist) / (2.0 * sigma_dist_ * sigma_dist_));

    tf2::Quaternion q;
    tf2::fromMsg(base_marker.pose.orientation, q);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw_marker = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_marker);

    const double marker_yaw_world = transformMapYaw(map_marker.yaw, tf_world_map);
    const double yaw_robot = normalizeAngle(marker_yaw_world - yaw_marker + aruco_yaw_offset_);

    const Eigen::Vector3d marker_offset_world = baseVectorToWorld(base_pos, yaw_robot);
    const Eigen::Vector3d robot_est = world_marker - marker_offset_world;

    estimates.push_back(robot_est);
    weights.push_back(weight);
    yaw_estimates.push_back(yaw_robot);
    yaw_weights.push_back(weight);
  }

  if (estimates.empty()) {
    return;
  }

  Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
  double sum_w = 0.0;

  for (size_t index = 0; index < estimates.size(); ++index) {
    mean_pos += weights[index] * estimates[index];
    sum_w += weights[index];
  }

  mean_pos /= sum_w;

  double sin_sum = 0.0;
  double cos_sum = 0.0;

  for (size_t index = 0; index < yaw_estimates.size(); ++index) {
    sin_sum += std::sin(yaw_estimates[index]) * yaw_weights[index];
    cos_sum += std::cos(yaw_estimates[index]) * yaw_weights[index];
  }

  const double mean_yaw = std::atan2(sin_sum, cos_sum);
  Eigen::Vector3d filt_pos;
  double filt_yaw = 0.0;

  if (first_measurement_) {
    filt_pos = mean_pos;
    filt_yaw = mean_yaw;
    first_measurement_ = false;
  } else {
    filt_pos = alpha_pos_ * mean_pos + (1.0 - alpha_pos_) * prev_pos_;
    const double dyaw = normalizeAngle(mean_yaw - prev_yaw_);
    filt_yaw = prev_yaw_ + alpha_yaw_ * dyaw;
  }

  prev_pos_ = filt_pos;
  prev_yaw_ = filt_yaw;

  PoseWithCovarianceStamped out;

  out.header.stamp = get_clock()->now();
  out.header.frame_id = world_frame_;
  out.pose.pose.position.x = filt_pos.x();
  out.pose.pose.position.y = filt_pos.y();
  out.pose.pose.position.z = filt_pos.z();

  tf2::Quaternion q_out;
  q_out.setRPY(0.0, 0.0, filt_yaw);
  out.pose.pose.orientation = tf2::toMsg(q_out);

  const double marker_count = static_cast<double>(estimates.size());
  const double covariance_weight =
      sum_w + multi_marker_covariance_gain_ * std::max(0.0, marker_count - 1.0);
  const double sigma_xy = pose_covariance_xy_ / std::sqrt(covariance_weight);
  const double sigma_z = pose_covariance_z_ / std::sqrt(covariance_weight);
  const double sigma_yaw = pose_covariance_yaw_ / std::sqrt(covariance_weight);

  out.pose.covariance[0] = sigma_xy;
  out.pose.covariance[7] = sigma_xy;
  out.pose.covariance[14] = sigma_z;
  out.pose.covariance[35] = sigma_yaw;

  pose_pub_->publish(out);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  rclcpp::spin(std::make_shared<ArucoMapLocalization>(options));
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
