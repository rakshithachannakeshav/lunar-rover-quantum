from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_pkg',
            executable='lidar_processor.py',
            name='lidar_processor',
            output='screen'
        ),
        Node(
            package='sensor_pkg',
            executable='imu_processor.py',
            name='imu_processor',
            output='screen'
        ),
        Node(
            package='sensor_pkg',
            executable='encoder_processor.py',
            name='encoder_processor',
            output='screen'
        ),
    ])
