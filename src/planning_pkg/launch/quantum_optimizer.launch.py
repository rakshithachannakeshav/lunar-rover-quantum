#!/usr/bin/env python3
"""Launch the Phase-8 quantum optimizer."""

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
        DeclareLaunchArgument("frame_id", default_value="map"),
        DeclareLaunchArgument("out_dir", default_value="results/paths"),
        DeclareLaunchArgument("max_edges", default_value="10"),
        DeclareLaunchArgument("reps", default_value="1"),
        DeclareLaunchArgument("shots", default_value="2048"),
        DeclareLaunchArgument("restarts", default_value="4"),
        DeclareLaunchArgument("seed", default_value="7"),
    ]

    node = Node(
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
                "frame_id": LaunchConfiguration("frame_id"),
                "out_dir": LaunchConfiguration("out_dir"),
                "max_edges": LaunchConfiguration("max_edges"),
                "reps": LaunchConfiguration("reps"),
                "shots": LaunchConfiguration("shots"),
                "restarts": LaunchConfiguration("restarts"),
                "seed": LaunchConfiguration("seed"),
            }
        ],
    )

    return LaunchDescription(args + [node])