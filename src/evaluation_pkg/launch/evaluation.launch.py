#!/usr/bin/env python3
"""
Phase 10: evaluation launch file.

Starts the simulated battery monitor and the evaluator. Run it alongside the
Phase 9 navigation pipeline (Gazebo + planner + path executor); it only listens to
/odom and /cmd_vel and publishes /battery/status and /metrics.

    ros2 launch evaluation_pkg evaluation.launch.py planner:=qaoa
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'planner', default_value='astar',
            description='Which planned path to evaluate: dijkstra | astar | qaoa. '
                        'Match it to the planner driving the rover.'),
        DeclareLaunchArgument(
            'comparison_path', default_value='results/energy_comparison/latest_comparison.json',
            description='Comparison file written by `python -m evaluation_pkg.metrics`.'),
        DeclareLaunchArgument(
            'goal_tolerance', default_value='0.35',
            description='Distance to the final waypoint (m) that counts as arrived.'),
        DeclareLaunchArgument(
            'capacity_wh', default_value='40.0',
            description='Simulated battery capacity in Wh.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Gazebo clock (/clock).'),

        Node(
            package='evaluation_pkg',
            executable='battery_monitor_node',
            name='battery_monitor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'capacity_wh': LaunchConfiguration('capacity_wh'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='evaluation_pkg',
            executable='evaluator_node',
            name='evaluator',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'planner': LaunchConfiguration('planner'),
                'comparison_path': LaunchConfiguration('comparison_path'),
                'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
