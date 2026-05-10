from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_sound_path(filename: str) -> str:
    package_share_candidate = Path(__file__).resolve().parents[1] / 'sound' / filename
    workspace_candidate = Path(__file__).resolve().parents[3] / 'sound' / filename
    if package_share_candidate.is_file():
        return str(package_share_candidate)
    return str(workspace_candidate)


def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    enable_audio_cues = LaunchConfiguration('enable_audio_cues')
    soft_sound_path = LaunchConfiguration('soft_sound_path')
    warning_sound_path = LaunchConfiguration('warning_sound_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='elevator_yield_human_control',
            description='Gazebo world name that exposes /world/<name>/visual_config.',
        ),
        DeclareLaunchArgument(
            'enable_audio_cues',
            default_value='true',
            description='Whether to play sound cues for yielding and hard-stop events.',
        ),
        DeclareLaunchArgument(
            'soft_sound_path',
            default_value=_default_sound_path('softsound.mp3'),
            description='Path to the soft cue sound file.',
        ),
        DeclareLaunchArgument(
            'warning_sound_path',
            default_value=_default_sound_path('warning.mp3'),
            description='Path to the hard-stop warning sound file.',
        ),
        Node(
            package='hmi_feedback',
            executable='led_strip_controller',
            name='led_strip_controller',
            parameters=[
                {
                    'world_name': world_name,
                    'enable_audio_cues': enable_audio_cues,
                    'soft_sound_path': soft_sound_path,
                    'warning_sound_path': warning_sound_path,
                }
            ],
            output='screen',
        ),
    ])
