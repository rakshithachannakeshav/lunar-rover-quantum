#!/usr/bin/env python3
"""Launch the quantum planner and quantum path executor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "graph_path",
            default_value="results/graphs/latest.graphml",
        ),
        DeclareLaunchArgument("goal_x", default_value="8.0"),
        DeclareLaunchArgument("goal_y", default_value="0.0"),
    ]

    quantum_optimizer = Node(
        package="planning_pkg",
        executable="quantum_optimizer_node",
        name="quantum_optimizer",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "graph_path": LaunchConfiguration("graph_path"),
                "goal_x": LaunchConfiguration("goal_x"),
                "goal_y": LaunchConfiguration("goal_y"),
            }
        ],
    )

    quantum_executor = Node(
        package="navigation_pkg",
        executable="quantum_path_executor",
        name="quantum_path_executor",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription(
        args + [
            quantum_optimizer,
            quantum_executor,
        ]
    )