from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from hmi_interfaces.msg import ActorState, ObstacleState
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


@dataclass(frozen=True)
class ActorConfig:
    entity_name: str
    actor_id: str
    actor_type: str
    fallback_pose: tuple[float, float, float] | None


@dataclass(frozen=True)
class ObstacleConfig:
    obstacle_id: str
    entity_name: str
    width: float
    length: float
    is_static: bool
    fallback_pose: tuple[float, float, float] | None


@dataclass(frozen=True)
class MapConfig:
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    static_inflation_radius: float
    human_inflation_radius: float


@dataclass(frozen=True)
class BoxConfig:
    box_id: str
    entity_name: str
    size_x: float
    size_y: float
    fallback_pose: tuple[float, float, float] | None
    inflate_radius: float


class SceneStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('scene_state_publisher')

        default_scene_config = str(
            Path(get_package_share_directory('hmi_scene')) / 'elevator_yield_scene.yaml'
        )
        scene_config_path = Path(
            self.declare_parameter(
                'scene_config_path',
                default_scene_config,
                ParameterDescriptor(
                    description='Path to the YAML scene config used for static metadata such as entity names and obstacle sizes.'
                ),
            ).value
        )
        pose_topic = self.declare_parameter(
            'gazebo_pose_topic',
            '/world/elevator_yield/pose/info',
            ParameterDescriptor(description='Gazebo pose topic used as the direct source of scene state.'),
        ).value
        frame_id = self.declare_parameter(
            'frame_id',
            'world',
            ParameterDescriptor(description='Frame id to stamp on published scene states.'),
        ).value
        publish_rate_hz = float(
            self.declare_parameter(
                'publish_rate_hz',
                10.0,
                ParameterDescriptor(description='Publishing rate for scene state topics in Hz.'),
            ).value
        )

        (
            self._robot_config,
            self._human_config,
            self._obstacle_configs,
            self._map_config,
            self._static_boxes,
            self._dynamic_boxes,
        ) = self._load_scene_config(scene_config_path)

        robot_topic = self.declare_parameter('robot_state_topic', '/hmi/scene/robot_state').value
        human_topic = self.declare_parameter('human_state_topic', '/hmi/scene/human_state').value
        obstacle_topic = self.declare_parameter('obstacle_state_topic', '/hmi/scene/obstacle_state').value
        map_topic = self.declare_parameter('map_state_topic', '/hmi/scene/map_state').value
        static_map_topic = self.declare_parameter('static_map_topic', '/hmi/scene/static_map').value

        self._frame_id = str(frame_id)
        self._gazebo_pose_topic = str(pose_topic)
        self._latest_poses: dict[str, tuple[float, float, float]] = {}
        self._pose_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._pose_reader_process: subprocess.Popen[str] | None = None
        self._static_map_data = self._build_static_map_data()

        self._robot_publisher = self.create_publisher(ActorState, robot_topic, 10)
        self._human_publisher = self.create_publisher(ActorState, human_topic, 10)
        self._obstacle_publisher = self.create_publisher(ObstacleState, obstacle_topic, 10)
        self._map_publisher = self.create_publisher(OccupancyGrid, map_topic, 10)
        self._static_map_publisher = self.create_publisher(OccupancyGrid, static_map_topic, 10)

        self._pose_reader_thread = threading.Thread(target=self._run_pose_reader, daemon=True)
        self._pose_reader_thread.start()
        self.create_timer(1.0 / publish_rate_hz, self._publish_scene_state)
        self.get_logger().info(
            'Publishing scene state from Gazebo topic '
            f'{pose_topic} to {robot_topic}, {human_topic}, {obstacle_topic}, {map_topic}, and {static_map_topic}.'
        )
        self.get_logger().info(f'Static scene metadata is loaded from {scene_config_path}.')
        self.get_logger().info(
            'Map grid configured as '
            f'{self._map_config.width}x{self._map_config.height} cells at '
            f'{self._map_config.resolution:.3f} m/cell from origin '
            f'({self._map_config.origin_x:.2f}, {self._map_config.origin_y:.2f}).'
        )

    def _load_scene_config(
        self,
        scene_config_path: Path,
    ) -> tuple[ActorConfig, ActorConfig, list[ObstacleConfig], MapConfig, list[BoxConfig], list[BoxConfig]]:
        if not scene_config_path.is_file():
            raise FileNotFoundError(f'Scene config not found: {scene_config_path}')

        with scene_config_path.open('r', encoding='utf-8') as file_handle:
            data = yaml.safe_load(file_handle)

        models = data.get('models', {})
        robot = self._expect_mapping(models, 'robot')
        human = self._expect_mapping(models, 'human')
        walls = models.get('walls', [])
        obstacles = models.get('obstacles', [])
        if not isinstance(walls, list):
            raise ValueError('models.walls must be a list in the scene config.')
        if not isinstance(obstacles, list):
            raise ValueError('models.obstacles must be a list in the scene config.')

        robot_config = ActorConfig(
            entity_name=str(robot.get('entity_name', robot.get('name', 'turtlebot3_burger_ir'))),
            actor_id=str(robot.get('name', 'turtlebot3_burger_ir')),
            actor_type=str(robot.get('actor_type', 'robot')),
            fallback_pose=self._parse_pose_entry(robot),
        )
        human_config = ActorConfig(
            entity_name=str(human.get('entity_name', human.get('name', 'human_in_elevator'))),
            actor_id=str(human.get('name', 'human_in_elevator')),
            actor_type=str(human.get('actor_type', 'human')),
            fallback_pose=self._parse_pose_entry(human),
        )

        map_config = self._parse_map_config(data.get('map', {}))

        obstacle_configs = [self._parse_obstacle(item) for item in obstacles]

        static_boxes = [
            self._parse_box(
                item,
                default_name='wall',
                inflate_radius=map_config.static_inflation_radius,
            )
            for item in walls
        ]
        dynamic_boxes: list[BoxConfig] = []
        for item in obstacles:
            obstacle = self._parse_obstacle(item)
            box = self._parse_box(
                item,
                default_name=obstacle.obstacle_id,
                inflate_radius=(
                    map_config.static_inflation_radius if obstacle.is_static else map_config.human_inflation_radius
                ),
            )
            if obstacle.is_static:
                static_boxes.append(box)
            else:
                dynamic_boxes.append(box)

        return robot_config, human_config, obstacle_configs, map_config, static_boxes, dynamic_boxes

    def _parse_map_config(self, data: Any) -> MapConfig:
        if not isinstance(data, dict):
            raise ValueError('map must be a mapping in the scene config.')

        origin = data.get('origin', [])
        if not isinstance(origin, list) or len(origin) < 2:
            raise ValueError('map.origin must be [x, y].')

        return MapConfig(
            resolution=float(data.get('resolution', 0.05)),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            width=int(data.get('width', 200)),
            height=int(data.get('height', 200)),
            static_inflation_radius=float(data.get('static_inflation_radius', 0.18)),
            human_inflation_radius=float(data.get('human_inflation_radius', 0.0)),
        )

    def _expect_mapping(self, parent: dict[str, Any], key: str) -> dict[str, Any]:
        value = parent.get(key, {})
        if not isinstance(value, dict):
            raise ValueError(f'models.{key} must be a mapping in the scene config.')
        return value

    def _parse_obstacle(self, data: Any) -> ObstacleConfig:
        if not isinstance(data, dict):
            raise ValueError('Each obstacle entry must be a mapping in the scene config.')

        size = data.get('size', [])
        if not isinstance(size, list) or len(size) < 2:
            raise ValueError('Each obstacle entry must provide size: [width, length, ...].')

        obstacle_id = str(data.get('name', 'obstacle'))
        entity_name = str(data.get('entity_name', obstacle_id))
        return ObstacleConfig(
            obstacle_id=obstacle_id,
            entity_name=entity_name,
            width=float(size[0]),
            length=float(size[1]),
            is_static=bool(data.get('is_static', True)),
            fallback_pose=self._parse_pose_entry(data),
        )

    def _parse_box(self, data: Any, default_name: str, inflate_radius: float) -> BoxConfig:
        if not isinstance(data, dict):
            raise ValueError('Each map box entry must be a mapping in the scene config.')

        size = data.get('size', [])
        if not isinstance(size, list) or len(size) < 2:
            raise ValueError('Each map box entry must provide size: [size_x, size_y, ...].')

        box_id = str(data.get('name', default_name))
        entity_name = str(data.get('entity_name', box_id))
        return BoxConfig(
            box_id=box_id,
            entity_name=entity_name,
            size_x=float(size[0]),
            size_y=float(size[1]),
            fallback_pose=self._parse_pose_entry(data),
            inflate_radius=inflate_radius,
        )

    def _parse_pose_entry(self, data: dict[str, Any]) -> tuple[float, float, float] | None:
        pose = data.get('pose', [])
        if not isinstance(pose, list) or len(pose) < 6:
            return None
        return float(pose[0]), float(pose[1]), float(pose[5])

    def _run_pose_reader(self) -> None:
        command = [
            'gz',
            'topic',
            '-e',
            '-t',
            self._gazebo_pose_topic,
            '--json-output',
        ]
        while not self._stop_event.is_set():
            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1,
                )
            except FileNotFoundError:
                self.get_logger().error('Could not find the `gz` executable needed to read Gazebo poses.')
                return

            self._pose_reader_process = process
            self.get_logger().info(f'Started Gazebo pose reader: {" ".join(command)}')

            stderr_output = ''
            try:
                if process.stdout is None:
                    raise RuntimeError('Gazebo pose reader has no stdout stream.')

                for line in process.stdout:
                    if self._stop_event.is_set():
                        break
                    line = line.strip()
                    if not line:
                        continue
                    self._on_pose_json_line(line)
            except Exception as error:
                self.get_logger().warning(f'Gazebo pose reader hit an error: {error}')
            finally:
                if process.stdout is not None:
                    process.stdout.close()
                if process.stderr is not None:
                    stderr_output = process.stderr.read().strip()
                    process.stderr.close()
                self._stop_process(process)
                if self._pose_reader_process is process:
                    self._pose_reader_process = None

            if self._stop_event.is_set():
                break
            if stderr_output:
                self.get_logger().warning(f'Gazebo pose reader exited and will retry: {stderr_output}')
            else:
                self.get_logger().warning('Gazebo pose reader exited and will retry.')
            time.sleep(1.0)

    def _stop_process(self, process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def _on_pose_json_line(self, line: str) -> None:
        data = json.loads(line)
        poses = data.get('pose', [])
        if not isinstance(poses, list):
            return

        latest_poses: dict[str, tuple[float, float, float]] = {}
        for pose in poses:
            if not isinstance(pose, dict):
                continue

            entity_name = str(pose.get('name', '')).strip()
            position = pose.get('position', {})
            if not entity_name or not isinstance(position, dict):
                continue
            if not any(axis in position for axis in ('x', 'y', 'z')):
                continue

            orientation = pose.get('orientation', {})
            if not isinstance(orientation, dict):
                orientation = {}

            yaw = math.atan2(
                2.0 * (
                    float(orientation.get('w', 1.0)) * float(orientation.get('z', 0.0)) +
                    float(orientation.get('x', 0.0)) * float(orientation.get('y', 0.0))
                ),
                1.0 - 2.0 * (
                    float(orientation.get('y', 0.0)) * float(orientation.get('y', 0.0)) +
                    float(orientation.get('z', 0.0)) * float(orientation.get('z', 0.0))
                ),
            )
            latest_poses[entity_name] = (
                float(position.get('x', 0.0)),
                float(position.get('y', 0.0)),
                float(yaw),
            )

        with self._pose_lock:
            self._latest_poses = latest_poses

    def _find_entity_pose(
        self,
        entity_name: str,
        fallback_pose: tuple[float, float, float] | None,
    ) -> tuple[float, float, float] | None:
        with self._pose_lock:
            exact_pose = self._latest_poses.get(entity_name)
        if exact_pose is not None:
            return exact_pose

        prefix = entity_name + '::'
        with self._pose_lock:
            latest_poses = dict(self._latest_poses)
        for pose_name, pose in latest_poses.items():
            if pose_name.startswith(prefix):
                return pose
        return fallback_pose

    def _build_static_map_data(self) -> list[int]:
        grid = [0] * (self._map_config.width * self._map_config.height)
        for box in self._static_boxes:
            pose = self._find_entity_pose(box.entity_name, box.fallback_pose) or box.fallback_pose
            if pose is None:
                continue
            self._mark_box_on_grid(
                grid,
                center_x=pose[0],
                center_y=pose[1],
                yaw=pose[2],
                size_x=box.size_x,
                size_y=box.size_y,
                inflate_radius=box.inflate_radius,
            )
        return grid

    def _build_dynamic_map_data(self) -> list[int]:
        grid = list(self._static_map_data)
        for box in self._dynamic_boxes:
            pose = self._find_entity_pose(box.entity_name, box.fallback_pose)
            if pose is None:
                continue
            self._mark_box_on_grid(
                grid,
                center_x=pose[0],
                center_y=pose[1],
                yaw=pose[2],
                size_x=box.size_x,
                size_y=box.size_y,
                inflate_radius=box.inflate_radius,
            )
        return grid

    def _mark_box_on_grid(
        self,
        grid: list[int],
        center_x: float,
        center_y: float,
        yaw: float,
        size_x: float,
        size_y: float,
        inflate_radius: float,
    ) -> None:
        half_x = size_x * 0.5 + inflate_radius
        half_y = size_y * 0.5 + inflate_radius
        abs_cos = abs(math.cos(yaw))
        abs_sin = abs(math.sin(yaw))
        bound_x = abs_cos * half_x + abs_sin * half_y
        bound_y = abs_sin * half_x + abs_cos * half_y

        min_grid_x = max(
            0,
            int(math.floor((center_x - bound_x - self._map_config.origin_x) / self._map_config.resolution)),
        )
        max_grid_x = min(
            self._map_config.width - 1,
            int(math.floor((center_x + bound_x - self._map_config.origin_x) / self._map_config.resolution)),
        )
        min_grid_y = max(
            0,
            int(math.floor((center_y - bound_y - self._map_config.origin_y) / self._map_config.resolution)),
        )
        max_grid_y = min(
            self._map_config.height - 1,
            int(math.floor((center_y + bound_y - self._map_config.origin_y) / self._map_config.resolution)),
        )

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        for grid_y in range(min_grid_y, max_grid_y + 1):
            world_y = self._map_config.origin_y + (grid_y + 0.5) * self._map_config.resolution
            for grid_x in range(min_grid_x, max_grid_x + 1):
                world_x = self._map_config.origin_x + (grid_x + 0.5) * self._map_config.resolution
                dx = world_x - center_x
                dy = world_y - center_y
                local_x = cos_yaw * dx + sin_yaw * dy
                local_y = -sin_yaw * dx + cos_yaw * dy
                if abs(local_x) <= half_x and abs(local_y) <= half_y:
                    grid[grid_y * self._map_config.width + grid_x] = 100

    def _create_occupancy_grid_msg(self, now, data: list[int]) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.stamp = now
        msg.header.frame_id = self._frame_id
        msg.info.resolution = self._map_config.resolution
        msg.info.width = self._map_config.width
        msg.info.height = self._map_config.height
        msg.info.origin = Pose()
        msg.info.origin.position.x = self._map_config.origin_x
        msg.info.origin.position.y = self._map_config.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    def _publish_scene_state(self) -> None:
        now = self.get_clock().now().to_msg()

        robot_pose = self._find_entity_pose(self._robot_config.entity_name, self._robot_config.fallback_pose)
        if robot_pose is not None:
            robot_msg = ActorState()
            robot_msg.header.stamp = now
            robot_msg.header.frame_id = self._frame_id
            robot_msg.actor_id = self._robot_config.actor_id
            robot_msg.actor_type = self._robot_config.actor_type
            robot_msg.x = robot_pose[0]
            robot_msg.y = robot_pose[1]
            robot_msg.yaw = robot_pose[2]
            robot_msg.linear_x = 0.0
            robot_msg.angular_z = 0.0
            robot_msg.is_moving = False
            self._robot_publisher.publish(robot_msg)

        human_pose = self._find_entity_pose(self._human_config.entity_name, self._human_config.fallback_pose)
        if human_pose is not None:
            human_msg = ActorState()
            human_msg.header.stamp = now
            human_msg.header.frame_id = self._frame_id
            human_msg.actor_id = self._human_config.actor_id
            human_msg.actor_type = self._human_config.actor_type
            human_msg.x = human_pose[0]
            human_msg.y = human_pose[1]
            human_msg.yaw = human_pose[2]
            human_msg.linear_x = 0.0
            human_msg.angular_z = 0.0
            human_msg.is_moving = False
            self._human_publisher.publish(human_msg)

        for obstacle in self._obstacle_configs:
            obstacle_pose = self._find_entity_pose(obstacle.entity_name, obstacle.fallback_pose)
            if obstacle_pose is None:
                continue

            obstacle_msg = ObstacleState()
            obstacle_msg.header.stamp = now
            obstacle_msg.header.frame_id = self._frame_id
            obstacle_msg.obstacle_id = obstacle.obstacle_id
            obstacle_msg.x = obstacle_pose[0]
            obstacle_msg.y = obstacle_pose[1]
            obstacle_msg.width = obstacle.width
            obstacle_msg.length = obstacle.length
            obstacle_msg.is_static = obstacle.is_static
            self._obstacle_publisher.publish(obstacle_msg)

        self._static_map_publisher.publish(self._create_occupancy_grid_msg(now, list(self._static_map_data)))
        self._map_publisher.publish(self._create_occupancy_grid_msg(now, self._build_dynamic_map_data()))

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._pose_reader_process is not None:
            self._stop_process(self._pose_reader_process)
        if self._pose_reader_thread.is_alive():
            self._pose_reader_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SceneStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
