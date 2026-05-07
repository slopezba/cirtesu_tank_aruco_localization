from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("cirtesu_tank_aruco_localization"),
            "config",
            "aruco_map.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="cirtesu_tank_aruco_localization",
                executable="aruco_map_localization_node",
                name="aruco_map_localization",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
