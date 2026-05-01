from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hmi_behavior',
            executable='behavior_node',
            name='hmi_behavior',
            output='screen',
        ),
    ])
