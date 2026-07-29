#!/usr/bin/env python3
"""
Gazebo Harmonic + ROS 2 Jazzy — lunar rover simulation.

- Server always runs headless (`gz sim -s`).
- GUI is optional (`headless:=false`) and attaches with `gz sim -g` only.
  Do not pass the world file to the GUI (avoids duplicate sun/goal_marker crash).
- Default headless=true: WSL2 often cannot open the Ogre/EGL window reliably.
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare('rover_simulation').find('rover_simulation')
    urdf_file = os.path.join(pkg_share, 'urdf', 'rover.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'lunar_terrain.world')
    bridge_file = os.path.join(pkg_share, 'config', 'bridge.yaml')
    rviz_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='true = no Gazebo GUI (recommended on WSL). false = attach GUI client.',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2 (optional)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock',
    )

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    gz_env = {
        'GZ_SIM_RESOURCE_PATH': os.pathsep.join([
            os.path.join(pkg_share, 'worlds'),
            os.path.join(pkg_share, 'models'),
        ]),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': '/opt/ros/jazzy/opt/gz_sim_vendor/lib',
    }

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-v3', '-r', world_file],
        output='screen',
        additional_env=gz_env,
    )

    gz_gui_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v3'],
        output='screen',
        additional_env=gz_env,
        condition=IfCondition(PythonExpression(["'", headless, "' == 'false'"])),
    )

    ignore_gui_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_gui_client,
            on_exit=[LogInfo(msg='Gazebo GUI closed; physics server still running.')],
        ),
        condition=IfCondition(PythonExpression(["'", headless, "' == 'false'"])),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': bridge_file,
        }],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    headless_notice = LogInfo(
        msg=(
            '\n'
            '=== Gazebo HEADLESS (no window) — sim is running in the background ===\n'
            '  See the rover:  bash ~/lunar-rover-quantum/scripts/open_gazebo_gui.sh\n'
            '  Or use RViz:    ros2 launch rover_simulation demo.launch.py mode:=creep use_rviz:=true\n'
            '  Or check odom:  ros2 topic echo /odom --field pose.pose.position\n'
            '====================================================================\n'
        ),
        condition=IfCondition(PythonExpression(["'", headless, "' == 'true'"])),
    )

    gui_notice = LogInfo(
        msg='=== Gazebo GUI will open in ~4 s (headless:=false) ===',
        condition=IfCondition(PythonExpression(["'", headless, "' == 'false'"])),
    )

    return LaunchDescription([
        headless_arg,
        use_rviz_arg,
        use_sim_time_arg,
        headless_notice,
        gui_notice,
        robot_state_publisher,
        gz_server,
        TimerAction(period=4.0, actions=[gz_gui_client]),
        ignore_gui_exit,
        TimerAction(period=6.0, actions=[gz_bridge]),
        TimerAction(period=10.0, actions=[rviz2]),
    ])
