"""Launch the live multi-drone monitor + path planner GUI (host PC).

    ros2 launch formation_interface monitor.launch.py

Dry run with no OptiTrack hardware (also starts a node that publishes
simulated DronePose/DroneActive traffic for a few wandering drones):

    ros2 launch formation_interface monitor.launch.py use_mock:=true

Loads config/monitor.yaml for the flight-area bounds and planner settings.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("formation_interface"), "config", "monitor.yaml")

    use_mock = LaunchConfiguration("use_mock")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock", default_value="false",
            description="Also launch mock_optitrack_node to simulate drones "
                        "with no OptiTrack hardware."),
        Node(
            package="formation_interface",
            executable="monitor_gui",
            name="formation_monitor",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="formation_interface",
            executable="mock_optitrack_node",
            name="mock_optitrack",
            output="screen",
            parameters=[config],
            condition=IfCondition(use_mock),
        ),
    ])
