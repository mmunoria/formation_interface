"""Companion-computer-side launch file: run ONE drone's rendered flight
params through flight_executor_node.py.

    ros2 launch formation_interface single_drone_flight.launch.py params_file:=/path/to/droneN_params.yaml

This is the static, checked-in launch file deploy/backends.py's SSHDeployer
starts remotely over ssh (see its docstring); it is not regenerated per
mission - only the params file (rendered by deploy/launch_gen.py, in the
same drones.yaml-style `/**: ros__parameters:` wildcard shape) changes
between flights.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            description="Path to a single drone's rendered params YAML "
                        "(see deploy/launch_gen.py:write_params_yaml)."),
        Node(
            package="formation_interface",
            executable="flight_executor_node",
            name="flight_executor_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
