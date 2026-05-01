from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hmi_test',
            executable='robot_state_monitor',
            name='robot_state_monitor',
            output='screen',
        ),
        Node(
            package='hmi_test',
            executable='human_state_monitor',
            name='human_state_monitor',
            output='screen',
        ),
        Node(
            package='hmi_test',
            executable='obstacle_state_monitor',
            name='obstacle_state_monitor',
            output='screen',
        ),
        Node(
            package='hmi_test',
            executable='map_state_monitor',
            name='map_state_monitor',
            output='screen',
        ),
    ])
