from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='elevator_yield_human_control',
            description='Gazebo world name that exposes /world/<name>/visual_config.',
        ),
        Node(
            package='hmi_feedback',
            executable='led_strip_controller',
            name='led_strip_controller',
            parameters=[
                {
                    'world_name': world_name,
                }
            ],
            output='screen',
        ),
    ])
