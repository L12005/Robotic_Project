from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hmi_test',
            executable='experiment_logger',
            name='experiment_logger',
            output='screen',
        ),
    ])
