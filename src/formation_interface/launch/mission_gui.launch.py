"""Launch the mission control panel (host PC).

    ros2 launch formation_interface mission_gui.launch.py

Loads config/mission_manager.yaml plus the installed config/drone_profiles/
and flight_profiles/ directories, same as mission.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("formation_interface")
    config = os.path.join(pkg_share, "config", "mission_manager.yaml")
    drone_profiles_dir = os.path.join(pkg_share, "config", "drone_profiles")
    flight_profiles_dir = os.path.join(pkg_share, "flight_profiles")

    return LaunchDescription([
        Node(
            package="formation_interface",
            executable="mission_gui",
            name="mission_gui",
            output="screen",
            parameters=[config, {
                "drone_profiles_dir": drone_profiles_dir,
                "flight_profiles_dir": flight_profiles_dir,
            }],
        ),
    ])
