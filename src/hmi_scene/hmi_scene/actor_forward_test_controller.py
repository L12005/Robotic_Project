from __future__ import annotations

import math
import subprocess
import time
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class ActorForwardTestController(Node):
    def __init__(self) -> None:
        super().__init__('actor_forward_test_controller')

        default_scene_config = str(
            Path(get_package_share_directory('hmi_scene')) / 'elevator_yield_scene.yaml'
        )
        scene_config_path = Path(
            self.declare_parameter(
                'scene_config_path',
                default_scene_config,
                ParameterDescriptor(description='Path to the YAML scene config used for the human actor start pose.'),
            ).value
        )
        self._world_name = str(
            self.declare_parameter(
                'world_name',
                'elevator_yield',
                ParameterDescriptor(description='Gazebo world name used for set_pose requests.'),
            ).value
        )
        self._actor_entity_name = str(
            self.declare_parameter(
                'actor_entity_name',
                'human_in_elevator',
                ParameterDescriptor(description='Gazebo actor entity to move forward for the minimal motion test.'),
            ).value
        )
        self._linear_speed = float(
            self.declare_parameter(
                'linear_speed',
                0.30,
                ParameterDescriptor(description='Forward speed in meters per second for the minimal actor motion test.'),
            ).value
        )
        self._travel_distance = float(
            self.declare_parameter(
                'travel_distance',
                6.20,
                ParameterDescriptor(description='Total forward travel distance in meters for the minimal actor motion test.'),
            ).value
        )
        self._start_delay_sec = float(
            self.declare_parameter(
                'start_delay_sec',
                1.0,
                ParameterDescriptor(description='Delay before the actor starts moving so Gazebo has time to finish loading.'),
            ).value
        )
        update_rate_hz = float(
            self.declare_parameter(
                'update_rate_hz',
                10.0,
                ParameterDescriptor(description='Update rate for sending actor set_pose commands.'),
            ).value
        )

        self._start_x, self._start_y, self._start_z, self._start_yaw = self._load_human_pose(scene_config_path)
        self._set_pose_service = f'/world/{self._world_name}/set_pose'
        self._launch_wall_time = time.monotonic()
        self._last_motion_wall_time: float | None = None
        self._travelled_distance = 0.0
        self._completed = False

        self.create_timer(1.0 / update_rate_hz, self._update_motion)
        self.get_logger().info(
            'Actor forward test controller is ready. '
            f'Actor: {self._actor_entity_name}, speed: {self._linear_speed:.2f} m/s, '
            f'distance: {self._travel_distance:.2f} m.'
        )

    def _load_human_pose(self, scene_config_path: Path) -> tuple[float, float, float, float]:
        if not scene_config_path.is_file():
            raise FileNotFoundError(f'Scene config not found: {scene_config_path}')

        with scene_config_path.open('r', encoding='utf-8') as file_handle:
            data = yaml.safe_load(file_handle)

        models = data.get('models', {})
        if not isinstance(models, dict):
            raise ValueError('models must be a mapping in the scene config.')
        human = models.get('human', {})
        if not isinstance(human, dict):
            raise ValueError('models.human must be a mapping in the scene config.')
        pose = human.get('pose', [])
        if not isinstance(pose, list) or len(pose) < 6:
            raise ValueError('models.human.pose must contain [x, y, z, roll, pitch, yaw].')
        return float(pose[0]), float(pose[1]), float(pose[2]), float(pose[5])

    def _update_motion(self) -> None:
        if self._completed:
            return

        now_wall = time.monotonic()
        if now_wall - self._launch_wall_time < self._start_delay_sec:
            return

        if self._last_motion_wall_time is None:
            self._last_motion_wall_time = now_wall
            return

        dt = now_wall - self._last_motion_wall_time
        self._last_motion_wall_time = now_wall
        if dt <= 0.0:
            return

        remaining = self._travel_distance - self._travelled_distance
        if remaining <= 1e-6:
            self._completed = True
            self.get_logger().info('Minimal actor forward test completed.')
            return

        step = min(self._linear_speed * dt, remaining)
        self._travelled_distance += step

        x = self._start_x + math.cos(self._start_yaw) * self._travelled_distance
        y = self._start_y + math.sin(self._start_yaw) * self._travelled_distance
        self._set_actor_pose(x, y, self._start_z, self._start_yaw)

    def _set_actor_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        half_yaw = yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        request = (
            f'name: "{self._actor_entity_name}" '
            f'position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}} '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz:.9f}, w: {qw:.9f}}}'
        )
        result = subprocess.run(
            [
                'gz',
                'service',
                '-s',
                self._set_pose_service,
                '--reqtype',
                'gz.msgs.Pose',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                '1000',
                '--req',
                request,
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            error_text = result.stderr.strip() or result.stdout.strip() or 'unknown error'
            self.get_logger().warning(f'Failed to set actor pose: {error_text}')


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActorForwardTestController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
