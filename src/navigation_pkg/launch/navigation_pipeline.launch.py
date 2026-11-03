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
    goal_x_arg = DeclareLaunchArgument("goal_x", default_value="35.0")
    goal_y_arg = DeclareLaunchArgument("goal_y", default_value="5.0")
    # Start endpoint. Left as .nan the quantum planner starts from the rover's
    # /odom pose; set it and the rover drives to the start first, then to the goal.
    start_x_arg = DeclareLaunchArgument("start_x", default_value=".nan")
    start_y_arg = DeclareLaunchArgument("start_y", default_value=".nan")
    executor_args = [
        DeclareLaunchArgument("goal_tolerance", default_value="0.15",
                              description="metres from the goal that counts as arrived"),
        DeclareLaunchArgument("avoid_obstacles", default_value="true",
                              description="use /scan to steer round obstacles"),
        DeclareLaunchArgument("stop_distance", default_value="0.7",
                              description="obstacle this close ahead triggers avoidance"),
        DeclareLaunchArgument("side_clearance", default_value="0.6",
                              description="veer away from anything this close on a flank"),
        DeclareLaunchArgument("avoid_timeout", default_value="30.0",
                              description="stop after avoiding continuously this many seconds"),
    ]
    executor_params = {
        "goal_tolerance": LaunchConfiguration("goal_tolerance"),
        "avoid_obstacles": LaunchConfiguration("avoid_obstacles"),
        "stop_distance": LaunchConfiguration("stop_distance"),
        "side_clearance": LaunchConfiguration("side_clearance"),
        "avoid_timeout": LaunchConfiguration("avoid_timeout"),
    }
    # QAOA knobs (quantum mode only). selection:=most_likely or top judges QAOA
    # on its own output distribution instead of best-of-N sampling.
    qaoa_args = [
        DeclareLaunchArgument("selection", default_value="best",
                              description="best | most_likely | top"),
        DeclareLaunchArgument("shots", default_value="2048"),
        DeclareLaunchArgument("reps", default_value="1"),
        DeclareLaunchArgument("restarts", default_value="4"),
        DeclareLaunchArgument("max_edges", default_value="12"),
        DeclareLaunchArgument("seed", default_value="7"),
        DeclareLaunchArgument("device", default_value="auto"),
    ]

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
                "start_x": LaunchConfiguration("start_x"),
                "start_y": LaunchConfiguration("start_y"),
                "goal_x": LaunchConfiguration("goal_x"),
                "goal_y": LaunchConfiguration("goal_y"),
                "selection": LaunchConfiguration("selection"),
                "shots": LaunchConfiguration("shots"),
                "reps": LaunchConfiguration("reps"),
                "restarts": LaunchConfiguration("restarts"),
                "max_edges": LaunchConfiguration("max_edges"),
                "seed": LaunchConfiguration("seed"),
                "device": LaunchConfiguration("device"),
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
        parameters=[{"path_topic": "/path/quantum", **executor_params}],
        condition=IfCondition(is_quantum),
    )

    classical_executor = Node(
        package="navigation_pkg",
        executable="path_executor",
        name="path_executor",
        output="screen",
        emulate_tty=True,
        parameters=[{"path_topic": "/path/classical", **executor_params}],
        condition=IfCondition(is_classical),
    )

    return LaunchDescription(
        [
            planner_mode_arg,
            graph_path_arg,
            goal_x_arg,
            goal_y_arg,
            start_x_arg,
            start_y_arg,
            *executor_args,
            *qaoa_args,
            quantum_planner,
            classical_planner,
            quantum_executor,
            classical_executor,
        ]
    )
