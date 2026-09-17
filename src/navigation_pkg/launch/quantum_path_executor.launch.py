from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="navigation_pkg",
                executable="quantum_path_executor",
                name="quantum_path_executor",
                output="screen",
            )
        ]
    )