from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    workspace_root = Path(get_package_prefix('hmi_scene')).parents[1]
    source_packages_dir = workspace_root / 'src'
    world_file = Path(get_package_share_directory('hmi_world')) / 'elevator_yield.world'
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
            name='set_pose_bridge',
            arguments=[
                '/world/elevator_yield/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='human_joint_bridge',
            arguments=[
                '/human_in_elevator/left_arm_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/human_in_elevator/right_arm_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/human_in_elevator/left_leg_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
                '/human_in_elevator/right_leg_joint_cmd@std_msgs/msg/Float64@gz.msgs.Double',
            ],
            output='screen',
        ),
        Node(
            package='hmi_scene',
            executable='human_motion_controller',
            name='human_motion_controller',
            parameters=[
                {
                    'start_delay_sec': 6.0,
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
                    'use_robot_state_feedback': False,
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
                    'goal_y': 2.95,
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
                    'publish_rate_hz': 20.0,
                    'robot_odometry_topic': '/hmi/scene/robot_odometry',
                }
            ],
            output='screen',
        ),
    ])
