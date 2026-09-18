#!/usr/bin/env python3
"""Launch description for the Phase-9 path executor node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("path_topic", default_value="/path/quantum"),
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("goal_tolerance", default_value="0.5"),
        DeclareLaunchArgument("waypoint_tolerance", default_value="0.35"),
        DeclareLaunchArgument("max_linear", default_value="0.32"),
        DeclareLaunchArgument("max_angular", default_value="0.9"),
    ]

    node = Node(
        package="navigation_pkg",
        executable="path_executor",
        name="path_executor",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "path_topic": LaunchConfiguration("path_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
                "goal_tolerance": LaunchConfiguration("goal_tolerance"),
                "waypoint_tolerance": LaunchConfiguration("waypoint_tolerance"),
                "max_linear": LaunchConfiguration("max_linear"),
                "max_angular": LaunchConfiguration("max_angular"),
            }
        ],
    )

    return LaunchDescription(args + [node])

