#!/usr/bin/env python3

# Copyright 2026 Rakshitha Channakeshav
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Launch the classical path planner node (Dijkstra and A*)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the classical planner node."""
    graph_path_arg = DeclareLaunchArgument(
        'graph_path',
        default_value='results/graphs/latest.graphml',
        description='Path to the energy-weighted GraphML file.',
    )

    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Start world X position in meters.',
    )

    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Start world Y position in meters.',
    )

    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='10.0',
        description='Goal world X position in meters.',
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='10.0',
        description='Goal world Y position in meters.',
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Reference TF frame for path poses.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from Gazebo /clock.',
    )

    out_dir_arg = DeclareLaunchArgument(
        'out_dir',
        default_value='results/paths',
        description='Directory to save path planning JSON artifacts.',
    )

    planner_node = Node(
        package='planning_pkg',
        executable='classical_planner_node',
        name='classical_planner',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'graph_path': LaunchConfiguration('graph_path'),
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'frame_id': LaunchConfiguration('frame_id'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'out_dir': LaunchConfiguration('out_dir'),
        }]
    )

    return LaunchDescription([
        graph_path_arg,
        start_x_arg,
        start_y_arg,
        goal_x_arg,
        goal_y_arg,
        frame_id_arg,
        use_sim_time_arg,
        out_dir_arg,
        planner_node,
    ])
