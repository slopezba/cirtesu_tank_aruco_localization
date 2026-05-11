import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def namespaced_config(config_file, robot_namespace):
    text = Path(config_file).read_text(encoding="utf-8")
    text = text.replace("/cirtesub/", f"/{robot_namespace}/")
    text = text.replace("cirtesub/", f"{robot_namespace}/")
    text = text.replace("cirtesub_", f"{robot_namespace}_")

    safe_namespace = robot_namespace.replace("/", "_")
    output_file = f"/tmp/aruco_map_localization_{safe_namespace}_{Path(config_file).name}"
    Path(output_file).write_text(text, encoding="utf-8")
    return output_file


def launch_setup(context, *args, **kwargs):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    if not robot_namespace:
        raise RuntimeError("Launch argument 'robot_namespace' cannot be empty.")

    aruco_share = get_package_share_directory("cirtesu_tank_aruco_localization")
    config_file = namespaced_config(
        os.path.join(aruco_share, "config", "aruco_map.yaml"),
        robot_namespace,
    )

    return [
        Node(
            package="cirtesu_tank_aruco_localization",
            executable="aruco_map_localization_node",
            name="aruco_map_localization",
            output="screen",
            parameters=[config_file],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="bluerov"),
            OpaqueFunction(function=launch_setup),
        ]
    )
