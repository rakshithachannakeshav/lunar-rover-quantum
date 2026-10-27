#!/usr/bin/env python3
"""Launch the hybrid multi-site mission planner and the path executor.

Example:
  ros2 launch planning_pkg multi_site_mission.launch.py \
      sites:="[10.0, 10.0, -12.0, 8.0, 15.0, -10.0, -8.0, -14.0]" solver:=qaoa
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "heightmap_path",
            default_value="src/rover_simulation/worlds/heightmap.png",
        ),
        DeclareLaunchArgument(
            "sites",
            default_value="[10.0, 10.0, -12.0, 8.0, 15.0, -10.0, -8.0, -14.0]",
        ),
        DeclareLaunchArgument("solver", default_value="qaoa"),
        DeclareLaunchArgument("reps", default_value="1"),
        DeclareLaunchArgument("path_topic", default_value="/path/mission"),
    ]

    planner = Node(
        package="planning_pkg",
        executable="multi_site_mission_node",
        name="multi_site_mission",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "heightmap_path": LaunchConfiguration("heightmap_path"),
                # Substituted strings are YAML-parsed by launch_ros, so these
                # arrive as a double array and an int.
                "sites": LaunchConfiguration("sites"),
                "solver": LaunchConfiguration("solver"),
                "reps": LaunchConfiguration("reps"),
                "path_topic": LaunchConfiguration("path_topic"),
            }
        ],
    )

    executor = Node(
        package="navigation_pkg",
        executable="path_executor",
        name="mission_path_executor",
        output="screen",
        emulate_tty=True,
        parameters=[{"path_topic": LaunchConfiguration("path_topic")}],
    )

    return LaunchDescription(args + [planner, executor])
