"""Launch the graphical control panel (host PC).

    ros2 launch formation_interface gui.launch.py

Loads config/drones.yaml so the map subscribes to the same pose topics the
formation node uses.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("formation_interface"), "config", "drones.yaml")

    return LaunchDescription([
        Node(
            package="formation_interface",
            executable="gui_node",
            name="formation_gui",
            output="screen",
            parameters=[config],
        ),
    ])
