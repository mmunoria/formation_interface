"""Launch the formation node with the drone configuration file.

    ros2 launch formation_interface formation.launch.py
    ros2 launch formation_interface formation.launch.py backend:=sim

Run the interface node separately (it is interactive):

    ros2 run formation_interface interface_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("formation_interface"), "config", "drones.yaml")

    backend = LaunchConfiguration("backend")

    return LaunchDescription([
        DeclareLaunchArgument(
            "backend", default_value="px4",
            description="Command backend: 'px4' (real) or 'sim' (no hardware)."),
        Node(
            package="formation_interface",
            executable="formation_node",
            name="formation_node",
            output="screen",
            parameters=[config, {"backend": backend}],
        ),
    ])
