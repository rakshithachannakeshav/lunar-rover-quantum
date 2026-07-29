#!/usr/bin/env python3
"""Launch simulation with keyboard teleop or autonomous patrol."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rover_simulation')
    sim_launch = os.path.join(pkg, 'launch', 'simulation_launch.py')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='patrol',
        description='Drive mode: patrol | creep | idle',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='RViz2 (off by default — avoids TF time warnings)',
    )

    mode = LaunchConfiguration('mode')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items(),
    )

    patrol_node = Node(
        package='rover_simulation',
        executable='rover_patrol.py',
        name='rover_patrol',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'patrol'"])),
    )

    creep_node = Node(
        package='rover_simulation',
        executable='rover_creep.py',
        name='rover_creep',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'creep'"])),
    )

    return LaunchDescription([
        mode_arg,
        use_rviz_arg,
        simulation,
        TimerAction(period=12.0, actions=[patrol_node]),
        TimerAction(period=12.0, actions=[creep_node]),
    ])
