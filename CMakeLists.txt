cmake_minimum_required(VERSION 3.8)
project(cirtesu_tank_aruco_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(aruco_map_localization_node
  src/aruco_map_localization.cpp
)

target_include_directories(aruco_map_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(aruco_map_localization_node
  Eigen3::Eigen
)

ament_target_dependencies(aruco_map_localization_node
  aruco_opencv_msgs
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

install(
  TARGETS aruco_map_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
