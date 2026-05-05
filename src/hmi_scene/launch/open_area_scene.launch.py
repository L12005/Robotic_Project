from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    workspace_root = Path(get_package_prefix('hmi_scene')).parents[1]
    source_packages_dir = workspace_root / 'src'
    world_file = Path(get_package_share_directory('hmi_world')) / 'open_area.world'
    scene_config = Path(get_package_share_directory('hmi_scene')) / 'open_area_scene.yaml'
    model_path = source_packages_dir / 'hmi_elements'

    joint_topics = [
        '/model/pedestrian_1/left_arm_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/pedestrian_1/right_arm_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/pedestrian_1/left_leg_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
        '/model/pedestrian_1/right_leg_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
    ]

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=str(model_path),
        ),
        ExecuteProcess(
            cmd=[
                'gz',
                'sim',
                '-r',
                str(world_file),
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='open_area_set_pose_bridge',
            arguments=[
                '/world/open_area/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='open_area_pedestrian_joint_bridge',
            arguments=joint_topics,
            output='screen',
        ),
        Node(
            package='hmi_scene',
            executable='open_area_pedestrian_controller',
            name='open_area_pedestrian_controller',
            parameters=[
                {
                    'scene_config_path': str(scene_config),
                    'world_name': 'open_area',
                    'update_rate_hz': 30.0,
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
                    'world_name': 'open_area',
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
            package='hmi_scene',
            executable='scene_state_publisher',
            name='scene_state_publisher',
            parameters=[
                {
                    'scene_config_path': str(scene_config),
                    'gazebo_pose_topic': '/world/open_area/pose/info',
                    'gazebo_stats_topic': '/world/open_area/stats',
                    'publish_rate_hz': 20.0,
                }
            ],
            output='screen',
        ),
    ])
