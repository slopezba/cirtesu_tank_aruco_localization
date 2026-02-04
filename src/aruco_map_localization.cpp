#include "cirtesu_tank_aruco_localization/aruco_map_localization.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <algorithm>

// =====================================================

ArucoMapLocalization::ArucoMapLocalization(ros::NodeHandle& nh)
: nh_(nh),
  tf_listener_(tf_buffer_),
  first_measurement_(true)
{
  // =============================
  // Load parameters
  // =============================

  nh_.param("alpha_pos", alpha_pos_, 0.4);
  nh_.param("alpha_yaw", alpha_yaw_, 0.3);

  nh_.param("sigma_dist", sigma_dist_, 1.0);
  nh_.param("aruco_yaw_offset", aruco_yaw_offset_, -M_PI/2.0);

  nh_.param("world_frame", world_frame_, std::string("world_ned"));
  nh_.param("base_frame", base_frame_, std::string("girona500/base_link"));
  nh_.param("camera_frame", camera_frame_, std::string("girona500/down_camera/camera"));

  nh_.param("aruco_topic", aruco_topic_, std::string("/girona500/down_camera/aruco_detections"));
  nh_.param("marker_topic", marker_topic_, std::string("/tandem_girona/aruco_map_markers"));
  nh_.param("pose_topic", pose_topic_, std::string("/girona500/navigator/aruco_pose"));

  nh_.param("mesh_path", mesh_path_, std::string("package://girona500_description/meshes/cirtesu.dae"));
  nh_.getParam("mesh_scale", mesh_scale_);
  nh_.getParam("mesh_pos", mesh_pos_);
  nh_.param("mesh_yaw", mesh_yaw_, 0.0);

  // =============================
  // Load ArUco map
  // =============================

  XmlRpc::XmlRpcValue map_param;

  if (nh_.getParam("aruco_map", map_param))
  {
    for (auto it = map_param.begin(); it != map_param.end(); ++it)
    {
      int id = std::stoi(it->first);
      XmlRpc::XmlRpcValue val = it->second;

      Eigen::Vector3d p;
      p << double(val[0]), double(val[1]), double(val[2]);

      aruco_map_[id] = p;
    }
  }

  // =============================
  // ROS interfaces
  // =============================

  aruco_sub_ = nh_.subscribe(
      aruco_topic_, 1,
      &ArucoMapLocalization::arucoCallback, this);

  marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      marker_topic_, 1);

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      pose_topic_, 1);

  ROS_INFO("Aruco map localization C++ node started");
}

// =====================================================

double ArucoMapLocalization::normalizeAngle(double a)
{
  return atan2(sin(a), cos(a));
}

// =====================================================

void ArucoMapLocalization::arucoCallback(
    const aruco_opencv_msgs::ArucoDetection::ConstPtr& msg)
{
  std::vector<int> visible_ids;

  for (auto& m : msg->markers)
    visible_ids.push_back(m.marker_id);

  // =====================================================
  // 1) Publish MarkerArray (BASE FRAME)
  // =====================================================

  visualization_msgs::MarkerArray array;

  visualization_msgs::Marker mesh;

  mesh.header.frame_id = "cirtesu_base_link";
  mesh.header.stamp = ros::Time::now();

  mesh.ns = "cirtesu_mesh";
  mesh.id = 1000;
  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;

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

  for (auto& kv : aruco_map_)
  {
    visualization_msgs::Marker m;

    m.header = mesh.header;
    m.ns = "aruco_map";
    m.id = kv.first;

    m.type = visualization_msgs::Marker::CUBE;

    m.pose.position.x = kv.second.x();
    m.pose.position.y = kv.second.y();
    m.pose.position.z = kv.second.z();
    m.pose.orientation.w = 1.0;

    m.scale.x = 0.30;
    m.scale.y = 0.30;
    m.scale.z = 0.08;

    // GREEN default
    m.color.g = 1.0;
    m.color.a = 0.8;

    // YELLOW if visible
    if (std::find(visible_ids.begin(),
                  visible_ids.end(),
                  kv.first) != visible_ids.end())
    {
      m.color.r = 1.0;
      m.color.g = 1.0;
    }

    array.markers.push_back(m);
  }

  marker_pub_.publish(array);

  // =====================================================
  // 2) Robot localization
  // =====================================================

  if (msg->markers.empty())
    return;

  geometry_msgs::TransformStamped tf_base_cam;

  try
  {
    tf_base_cam = tf_buffer_.lookupTransform(
        base_frame_,
        camera_frame_,
        ros::Time(0),
        ros::Duration(0.2));
  }
  catch (...)
  {
    ROS_WARN_THROTTLE(1.0, "TF base->camera not available");
    return;
  }

  std::vector<Eigen::Vector3d> estimates;
  std::vector<double> weights;

  std::vector<double> yaw_estimates;
  std::vector<double> yaw_weights;

  for (auto& marker : msg->markers)
  {
    int id = marker.marker_id;

    if (!aruco_map_.count(id))
      continue;

    geometry_msgs::PoseStamped cam_marker;
    cam_marker.header = msg->header;
    cam_marker.pose = marker.pose;

    geometry_msgs::PoseStamped base_marker;

    try
    {
      tf2::doTransform(cam_marker, base_marker, tf_base_cam);
    }
    catch (...)
    {
      continue;
    }

    Eigen::Vector3d world_marker = aruco_map_[id];

    Eigen::Vector3d base_pos(
        base_marker.pose.position.x,
        base_marker.pose.position.y,
        base_marker.pose.position.z);

    Eigen::Vector3d robot_est = world_marker - base_pos;

    double dist = base_pos.norm();
    double weight = exp(-(dist * dist) / (2.0 * sigma_dist_ * sigma_dist_));

    estimates.push_back(robot_est);
    weights.push_back(weight);

    tf2::Quaternion q;
    tf2::fromMsg(base_marker.pose.orientation, q);

    double roll, pitch, yaw_marker;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_marker);

    double yaw_robot = normalizeAngle(yaw_marker + aruco_yaw_offset_);

    yaw_estimates.push_back(yaw_robot);
    yaw_weights.push_back(weight);
  }

  if (estimates.empty())
    return;

  Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
  double sum_w = 0.0;

  for (size_t i = 0; i < estimates.size(); ++i)
  {
    mean_pos += weights[i] * estimates[i];
    sum_w += weights[i];
  }

  mean_pos /= sum_w;

  double sin_sum = 0.0, cos_sum = 0.0;

  for (size_t i = 0; i < yaw_estimates.size(); ++i)
  {
    sin_sum += sin(yaw_estimates[i]) * yaw_weights[i];
    cos_sum += cos(yaw_estimates[i]) * yaw_weights[i];
  }

  double mean_yaw = atan2(sin_sum, cos_sum);

  // =====================================================
  // Temporal filter (EMA)
  // =====================================================

  Eigen::Vector3d filt_pos;
  double filt_yaw;

  if (first_measurement_)
  {
    filt_pos = mean_pos;
    filt_yaw = mean_yaw;
    first_measurement_ = false;
  }
  else
  {
    filt_pos = alpha_pos_ * mean_pos +
               (1.0 - alpha_pos_) * prev_pos_;

    double dyaw = normalizeAngle(mean_yaw - prev_yaw_);
    filt_yaw = prev_yaw_ + alpha_yaw_ * dyaw;
  }

  prev_pos_ = filt_pos;
  prev_yaw_ = filt_yaw;

  // =====================================================
  // Build local pose (base_link)
  // =====================================================

  geometry_msgs::PoseWithCovarianceStamped out;

  out.header.stamp = ros::Time::now();
  out.header.frame_id = "cirtesu_base_link";

  out.pose.pose.position.x = filt_pos.x();
  out.pose.pose.position.y = filt_pos.y();
  out.pose.pose.position.z = filt_pos.z();

  tf2::Quaternion q_out;
  q_out.setRPY(0, 0, -filt_yaw);

  out.pose.pose.orientation = tf2::toMsg(q_out);

  double sigma_xy  = 0.13  / sqrt(sum_w);
  double sigma_z   = 0.005 / sqrt(sum_w);
  double sigma_yaw = 1.0   / sqrt(sum_w);

  out.pose.covariance[0]  = sigma_xy;
  out.pose.covariance[7]  = sigma_xy;
  out.pose.covariance[14] = sigma_z;
  out.pose.covariance[35] = sigma_yaw;

  // =====================================================
  // Transform pose to world_ned (IDENTICAL to Python)
  // =====================================================

  geometry_msgs::PoseStamped pose_local;
  pose_local.header.stamp = out.header.stamp;
  pose_local.header.frame_id = "cirtesu_base_link";
  pose_local.pose = out.pose.pose;

  geometry_msgs::TransformStamped tf_world_base;

  try
  {
    tf_world_base = tf_buffer_.lookupTransform(
        world_frame_,
        "cirtesu_base_link",
        ros::Time(0),
        ros::Duration(0.2));
  }
  catch (...)
  {
    ROS_WARN_THROTTLE(1.0, "TF world_ned -> base not available");
    return;
  }

  geometry_msgs::PoseStamped pose_world;

  try
  {
    tf2::doTransform(pose_local, pose_world, tf_world_base);
  }
  catch (...)
  {
    ROS_WARN("Pose transform failed");
    return;
  }

  // =====================================================
  // Publish FINAL pose in world_ned
  // =====================================================

  out.header.frame_id = "world_ned";
  out.pose.pose = pose_world.pose;

  pose_pub_.publish(out);
}

// =====================================================

int main(int argc, char** argv)
{
  ros::init(argc, argv, "aruco_map_localization");

  ros::NodeHandle nh("~");

  ArucoMapLocalization node(nh);

  ros::spin();

  return 0;
}

