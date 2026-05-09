from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    world_file = Path(get_package_share_directory('hmi_world')) / 'human_xboxcontrol.world'
    scene_config = Path(get_package_share_directory('hmi_scene')) / 'human_xboxcontrol_scene.yaml'

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                str(world_file),
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='human_xboxcontrol_set_pose_bridge',
            arguments=[
                '/world/human_xboxcontrol/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='human_xboxcontrol_joint_bridge',
            arguments=[
                '/model/human_xbox/left_arm_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/human_xbox/right_arm_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/human_xbox/left_leg_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/human_xbox/right_leg_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            ],
            output='screen',
        ),
        Node(
            package='hmi_scene',
            executable='human_remote_cmd_receiver',
            name='human_remote_cmd_receiver',
            parameters=[
                {
                    'cmd_vel_topic': '/hmi/human/cmd_vel',
                    'listen_host': '0.0.0.0',
                    'listen_port': 8765,
                    'fixed_speed': 1.0,
                    'deadzone': 0.25,
                    'publish_rate_hz': 30.0,
                }
            ],
            output='screen',
        ),
        Node(
            package='hmi_scene',
            executable='human_xboxcontrol_controller',
            name='human_xboxcontrol_controller',
            parameters=[
                {
                    'scene_config_path': str(scene_config),
                    'world_name': 'human_xboxcontrol',
                    'visual_entity_name': 'human_xbox',
                    'collision_entity_name': 'human_xbox_collision_proxy',
                    'joint_topic_prefix': '/model/human_xbox',
                    'cmd_vel_topic': '/hmi/human/cmd_vel',
                    'update_rate_hz': 60.0,
                    'turn_speed_rad_s': 4.0,
                    'cmd_timeout_sec': 0.30,
                    'stride_length': 0.62,
                    'arm_swing_amplitude_rad': 0.72,
                    'leg_swing_amplitude_rad': 0.48,
                }
            ],
            output='screen',
        ),
    ])
