# cirtesu_tank_aruco_localization

**ROS 2 Humble C++ package for ArUco-based localization in CIRTESU tank environments**

This repository provides a ROS 2 Humble package that implements a marker-map-based localization system using ArUco detections and TF2 transformations. It is designed for the Girona500 underwater robot and for use in CIRTESU tank experiments and Stonefish simulation.

---

## Features

- Fully implemented in C++ (rclcpp)
- Compatible with ROS 2 Humble (Ubuntu 22.04)
- TF2-based frame transformations
- Publishes RViz visualization markers
- Publishes robot pose with covariance
- Multi-marker weighted fusion
- Temporal EMA filtering for stability
- YAML configurable
- Designed for Girona500 + CIRTESU tank setup

---

## Coordinate Frames

TF chain used by the node:

camera_frame -> base_frame -> world_ned

The final pose output is published in the world_ned frame.

Default frames:

- girona500/down_camera/camera
- girona500/base_link
- world_ned

---

## Published Topics

### Robot Pose

Topic:
/girona500/navigator/aruco_pose

Type:
geometry_msgs/PoseWithCovarianceStamped

Frame:
world_ned

---

### Marker Visualization

Topic:
/tandem_girona/aruco_map_markers

Type:
visualization_msgs/MarkerArray

Includes:

- CIRTESU tank mesh
- Fixed ArUco map markers
- Highlighted visible markers (yellow)

---

## Subscribed Topics

### ArUco Detections

Topic:
/girona500/down_camera/aruco_detections

Type:
aruco_opencv_msgs/msg/ArucoDetection

---

## Parameters (YAML)

Configuration file:

config/aruco_map.yaml

Main parameters:

alpha_pos: EMA position filter coefficient  
alpha_yaw: EMA yaw filter coefficient  
sigma_dist: Gaussian distance weighting  
aruco_yaw_offset: Camera-to-robot yaw correction  

---

## Dependencies

Required ROS 2 Humble packages:

- rclcpp
- tf2
- tf2_ros
- tf2_geometry_msgs
- geometry_msgs
- visualization_msgs
- aruco_opencv_msgs
- eigen3

Install dependencies:

sudo apt install ros-humble-tf2-geometry-msgs ros-humble-aruco-opencv-msgs

---

## Build Instructions

From your ROS 2 Humble workspace root:

cd ~/cirtesub_ws
colcon build --packages-select cirtesu_tank_aruco_localization
source install/setup.bash

---

## Run the Node

ros2 launch cirtesu_tank_aruco_localization aruco_map_localization.launch.py

Expected output:

[INFO] Aruco map localization C++ node started

---

## Debugging

Check pose output:

ros2 topic echo /girona500/navigator/aruco_pose

Check TF:

ros2 run tf2_ros tf2_echo world_ned girona500/base_link

Visualize markers:

Open RViz and add MarkerArray display:

Topic: /tandem_girona/aruco_map_markers

---

## Tested With

- Girona500 (Stonefish simulation)
- CIRTESU tank ArUco map
- ROS 2 Humble (Ubuntu 22.04)
- aruco_opencv

---

## Typical Use Case

This package is intended for:

- Absolute localization in tank experiments
- Navigation ground truth correction
- Input for robot_localization EKF
- RViz visualization
- Benchmarking perception pipelines

---

## Author

Salvador López-Barajas  
CIRTESU – Universitat Jaume I  
Underwater Robotics & Intervention Systems

---

## License

MIT License (recommended). Modify according to project requirements.
