from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    workspace_root = Path(get_package_prefix('hmi_scene')).parents[1]
    source_packages_dir = workspace_root / 'src'
    world_file = Path(get_package_share_directory('hmi_world')) / 'open_area_human_control.world'
    scene_config = Path(get_package_share_directory('hmi_scene')) / 'open_area_human_control_scene.yaml'
    model_path = source_packages_dir / 'hmi_elements'

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=str(model_path),
        ),
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
            name='open_area_human_control_set_pose_bridge',
            arguments=[
                '/world/open_area_human_control/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='open_area_human_control_joint_bridge',
            arguments=[
                '/model/pedestrian_1/left_arm_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/pedestrian_1/right_arm_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/pedestrian_1/left_leg_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/pedestrian_1/right_leg_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
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
                    'input_rotation_rad': -1.570796,
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
                    'world_name': 'open_area_human_control',
                    'visual_entity_name': 'pedestrian_1',
                    'collision_entity_name': 'pedestrian_1_collision_proxy',
                    'joint_topic_prefix': '/model/pedestrian_1',
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
        Node(
            package='hmi_scene',
            executable='robot_cmd_vel_controller',
            name='robot_cmd_vel_controller',
            parameters=[
                {
                    'scene_config_path': str(scene_config),
                    'world_name': 'open_area_human_control',
                    'use_robot_state_feedback': False,
                    'update_rate_hz': 20.0,
                }
            ],
            output='screen',
        ),
        Node(
            package='hmi_scene',
            executable='goal_pose_publisher',
            name='goal_pose_publisher',
            parameters=[
                {
                    'goal_x': 0.0,
                    'goal_y': 9.0,
                    'goal_yaw': 1.570796,
                }
            ],
            output='screen',
        ),
        Node(
            package='hmi_behavior',
            executable='behavior_node',
            name='hmi_behavior',
            parameters=[
                {
                    'control_rate_hz': 20.0,
                }
            ],
            output='screen',
        ),
        Node(
            package='hmi_feedback',
            executable='led_strip_controller',
            name='led_strip_controller',
            parameters=[
                {
                    'world_name': 'open_area_human_control',
                    'segment_count': 24,
                }
            ],
            output='screen',
        ),
        Node(
            package='hmi_scene',
            executable='scene_state_publisher',
            name='scene_state_publisher',
            parameters=[
                {
                    'scene_config_path': str(scene_config),
                    'gazebo_pose_topic': '/world/open_area_human_control/pose/info',
                    'gazebo_stats_topic': '/world/open_area_human_control/stats',
                    'publish_rate_hz': 20.0,
                }
            ],
            output='screen',
        ),
    ])
