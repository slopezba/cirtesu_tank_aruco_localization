# cirtesu_tank_aruco_localization

**ROS Noetic C++ package for ArUco-based localization in CIRTESU tank environments**

This repository provides a ROS Noetic package that implements a marker-map-based localization system using ArUco detections and TF2 transformations. It is designed for the Girona500 underwater robot and for use in CIRTESU tank experiments and Stonefish simulation.

---

## Features

- Fully implemented in C++ (roscpp)
- Compatible with ROS Noetic (Ubuntu 20.04)
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
aruco_opencv_msgs/ArucoDetection

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

Required ROS Noetic packages:

- roscpp
- tf2
- tf2_ros
- tf2_geometry_msgs
- geometry_msgs
- visualization_msgs
- aruco_opencv_msgs
- eigen3

Install dependencies:

sudo apt install ros-noetic-tf2-geometry-msgs ros-noetic-aruco-opencv

---

## Build Instructions

From your ROS Noetic workspace:

cd ~/catkin_ws_stonefish_cirtesu
catkin build cirtesu_tank_aruco_localization
source devel/setup.bash

Or using catkin_make:

catkin_make
source devel/setup.bash

---

## Run the Node

roslaunch cirtesu_tank_aruco_localization aruco_map_localization.launch

Expected output:

[INFO] Aruco map localization C++ node started

---

## Debugging

Check pose output:

rostopic echo /girona500/navigator/aruco_pose

Check TF:

rosrun tf tf_echo world_ned girona500/base_link

Visualize markers:

Open RViz and add MarkerArray display:

Topic: /tandem_girona/aruco_map_markers

---

## Tested With

- Girona500 (Stonefish simulation)
- CIRTESU tank ArUco map
- ROS Noetic (Ubuntu 20.04)
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
