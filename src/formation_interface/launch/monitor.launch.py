"""Launch the live multi-drone monitor + path planner GUI (host PC).

    ros2 launch formation_interface monitor.launch.py

Dry run with no OptiTrack hardware (also starts a node that publishes
simulated DronePose/DroneActive traffic for a few wandering drones):

    ros2 launch formation_interface monitor.launch.py use_mock:=true

Real single-drone testing (no fleet simulation): bridges drone 1's real
OptiTrack pose onto DronePose, and publishes its DroneActive heartbeat.
Each is its own node so a real multi-drone deployment just launches one
instance of each per drone (with a distinct drone_id):

    ros2 launch formation_interface monitor.launch.py use_drone_bridge:=true

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
    use_drone_bridge = LaunchConfiguration("use_drone_bridge")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock", default_value="false",
            description="Also launch mock_optitrack_node to simulate a whole "
                        "fleet with no OptiTrack hardware."),
        DeclareLaunchArgument(
            "use_drone_bridge", default_value="false",
            description="Also launch drone_active_publisher + "
                        "optitrack_pose_bridge for drone 1, bridging its real "
                        "OptiTrack pose instead of simulating one."),
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
        Node(
            package="formation_interface",
            executable="drone_active_publisher",
            name="drone_active_publisher",
            output="screen",
            parameters=[config],
            condition=IfCondition(use_drone_bridge),
        ),
        Node(
            package="formation_interface",
            executable="optitrack_pose_bridge",
            name="optitrack_pose_bridge",
            output="screen",
            parameters=[config],
            condition=IfCondition(use_drone_bridge),
        ),
    ])
