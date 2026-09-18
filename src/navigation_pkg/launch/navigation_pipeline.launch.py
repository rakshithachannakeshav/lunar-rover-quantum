#!/usr/bin/env python3
"""
Phase 9: Complete Navigation Pipeline Launch File.

Launches the selected planner node (classical A* or QAOA quantum optimizer)
together with the path executor node to drive the lunar rover to the goal.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    planner_mode_arg = DeclareLaunchArgument(
        "planner_mode",
        default_value="quantum",
        description="Planner type: quantum | classical",
    )
    graph_path_arg = DeclareLaunchArgument(
        "graph_path",
        default_value="results/graphs/latest.graphml",
        description="Path to NetworkX weighted graphml file",
    )
    goal_x_arg = DeclareLaunchArgument("goal_x", default_value="8.0")
    goal_y_arg = DeclareLaunchArgument("goal_y", default_value="0.0")

    planner_mode = LaunchConfiguration("planner_mode")

    is_quantum = PythonExpression(["'", planner_mode, "' == 'quantum'"])
    is_classical = PythonExpression(["'", planner_mode, "' == 'classical'"])

    quantum_planner = Node(
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
        condition=IfCondition(is_quantum),
    )

    classical_planner = Node(
        package="planning_pkg",
        executable="classical_planner_node",
        name="classical_planner",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "graph_path": LaunchConfiguration("graph_path"),
                "goal_x": LaunchConfiguration("goal_x"),
                "goal_y": LaunchConfiguration("goal_y"),
            }
        ],
        condition=IfCondition(is_classical),
    )

    quantum_executor = Node(
        package="navigation_pkg",
        executable="path_executor",
        name="path_executor",
        output="screen",
        emulate_tty=True,
        parameters=[{"path_topic": "/path/quantum"}],
        condition=IfCondition(is_quantum),
    )

    classical_executor = Node(
        package="navigation_pkg",
        executable="path_executor",
        name="path_executor",
        output="screen",
        emulate_tty=True,
        parameters=[{"path_topic": "/path/classical"}],
        condition=IfCondition(is_classical),
    )

    return LaunchDescription(
        [
            planner_mode_arg,
            graph_path_arg,
            goal_x_arg,
            goal_y_arg,
            quantum_planner,
            classical_planner,
            quantum_executor,
            classical_executor,
        ]
    )
