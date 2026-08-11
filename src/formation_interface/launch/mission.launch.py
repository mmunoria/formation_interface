"""Launch the mission manager node.

    ros2 launch formation_interface mission.launch.py
    ros2 launch formation_interface mission.launch.py backend:=ssh

Loads config/mission_manager.yaml, plus the installed config/drone_profiles/
and flight_profiles/ directories (so this works from an installed package,
not just a source checkout). 'backend' overrides every drone profile's
default backend for this run - leave it at 'mock_ssh' (the default, no
hardware required) unless real companion computers are reachable.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("formation_interface")
    config = os.path.join(pkg_share, "config", "mission_manager.yaml")
    drone_profiles_dir = os.path.join(pkg_share, "config", "drone_profiles")
    flight_profiles_dir = os.path.join(pkg_share, "flight_profiles")

    backend = LaunchConfiguration("backend")

    return LaunchDescription([
        DeclareLaunchArgument(
            "backend", default_value="mock_ssh",
            description="Default deploy backend: 'local' | 'ssh' | "
                        "'mock_ssh' (no hardware, the safe default)."),
        Node(
            package="formation_interface",
            executable="mission_node",
            name="mission_node",
            output="screen",
            parameters=[config, {
                "default_backend": backend,
                "drone_profiles_dir": drone_profiles_dir,
                "flight_profiles_dir": flight_profiles_dir,
            }],
        ),
    ])
