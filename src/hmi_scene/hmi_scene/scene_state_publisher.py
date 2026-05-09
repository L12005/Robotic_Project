from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from dataclasses import replace
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from hmi_scene.pedestrian_path_profiles import load_pedestrian_path_profile
from hmi_interfaces.msg import ActorState, ObstacleState
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


@dataclass(frozen=True)
class ActorConfig:
    entity_name: str
    actor_id: str
    actor_type: str
    fallback_pose: tuple[float, float, float] | None
    fallback_linear_speed: float
    pose_source_entity_name: str
    pose_source_offset: tuple[float, float, float]


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


@dataclass(frozen=True)
class PriorityHumanConfig:
    actor: ActorConfig
    start_delay_sec: float


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
        pedestrian_path_config_value = str(
            self.declare_parameter(
                'pedestrian_path_config_path',
                '',
                ParameterDescriptor(
                    description='Optional YAML file containing named pedestrian path profiles.'
                ),
            ).value
        ).strip()
        pedestrian_path_config_path = Path(pedestrian_path_config_value) if pedestrian_path_config_value else None
        pedestrian_path_name = str(
            self.declare_parameter(
                'pedestrian_path_name',
                '',
                ParameterDescriptor(
                    description='Optional pedestrian path profile name to load from pedestrian_path_config_path.'
                ),
            ).value
        ).strip()
        pose_topic = self.declare_parameter(
            'gazebo_pose_topic',
            '/world/elevator_yield/pose/info',
            ParameterDescriptor(description='Gazebo pose topic used as the direct source of scene state.'),
        ).value
        stats_topic = self.declare_parameter(
            'gazebo_stats_topic',
            '/world/elevator_yield/stats',
            ParameterDescriptor(description='Gazebo stats topic used to detect whether the simulation is paused.'),
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
        motion_stale_timeout_sec = float(
            self.declare_parameter(
                'motion_stale_timeout_sec',
                0.35,
                ParameterDescriptor(
                    description='If no actual pose change is detected for this long, treat the actor as stopped.'
                ),
            ).value
        )
        motion_position_epsilon_m = float(
            self.declare_parameter(
                'motion_position_epsilon_m',
                0.002,
                ParameterDescriptor(
                    description='Minimum xy pose change in meters needed to consider an actor moving.'
                ),
            ).value
        )
        motion_yaw_epsilon_rad = float(
            self.declare_parameter(
                'motion_yaw_epsilon_rad',
                0.01,
                ParameterDescriptor(
                    description='Minimum yaw change in radians needed to consider an actor moving.'
                ),
            ).value
        )

        (
            self._robot_config,
            self._human_config,
            self._priority_human_candidates,
            self._obstacle_configs,
            self._map_config,
            self._static_boxes,
            self._dynamic_boxes,
        ) = self._load_scene_config(
            scene_config_path,
            pedestrian_path_config_path,
            pedestrian_path_name,
        )

        robot_topic = self.declare_parameter('robot_state_topic', '/hmi/scene/robot_state').value
        robot_odometry_topic = self.declare_parameter(
            'robot_odometry_topic',
            f'/model/{self._robot_config.entity_name}/odometry',
            ParameterDescriptor(description='Robot odometry topic bridged from Gazebo DiffDrive.'),
        ).value
        human_topic = self.declare_parameter('human_state_topic', '/hmi/scene/human_state').value
        obstacle_topic = self.declare_parameter('obstacle_state_topic', '/hmi/scene/obstacle_state').value
        map_topic = self.declare_parameter('map_state_topic', '/hmi/scene/map_state').value
        static_map_topic = self.declare_parameter('static_map_topic', '/hmi/scene/static_map').value

        self._frame_id = str(frame_id)
        self._gazebo_pose_topic = str(pose_topic)
        self._gazebo_stats_topic = str(stats_topic)
        self._motion_stale_timeout_sec = max(float(motion_stale_timeout_sec), 0.05)
        self._motion_position_epsilon_m = max(float(motion_position_epsilon_m), 0.0)
        self._motion_yaw_epsilon_rad = max(float(motion_yaw_epsilon_rad), 0.0)
        self._latest_poses: dict[str, tuple[float, float, float]] = {}
        self._latest_pose_stamps: dict[str, float] = {}
        self._robot_odom_lock = threading.Lock()
        self._latest_robot_odom_pose: tuple[float, float, float] | None = None
        self._latest_robot_odom_twist: tuple[float, float] | None = None
        self._latest_robot_odom_stamp: float | None = None
        self._last_actor_pose_samples: dict[str, tuple[float, float, float, float]] = {}
        self._last_actor_motion_times: dict[str, float] = {}
        self._selected_priority_human_id: str | None = None
        self._use_priority_human_selector = len(self._priority_human_candidates) > 1
        self._pose_lock = threading.Lock()
        self._stats_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._pose_reader_process: subprocess.Popen[str] | None = None
        self._stats_reader_process: subprocess.Popen[str] | None = None
        self._gazebo_is_paused = True
        self._latest_sim_time_sec: float | None = None
        self._static_map_data = self._build_static_map_data()

        self._robot_publisher = self.create_publisher(ActorState, robot_topic, 10)
        self._human_publisher = self.create_publisher(ActorState, human_topic, 10)
        self._obstacle_publisher = self.create_publisher(ObstacleState, obstacle_topic, 10)
        self._map_publisher = self.create_publisher(OccupancyGrid, map_topic, 10)
        self._static_map_publisher = self.create_publisher(OccupancyGrid, static_map_topic, 10)
        self.create_subscription(Odometry, robot_odometry_topic, self._on_robot_odometry, 20)

        self._pose_reader_thread = threading.Thread(target=self._run_pose_reader, daemon=True)
        self._stats_reader_thread = threading.Thread(target=self._run_stats_reader, daemon=True)
        self._pose_reader_thread.start()
        self._stats_reader_thread.start()
        self.create_timer(1.0 / publish_rate_hz, self._publish_scene_state)
        self.get_logger().info(
            'Publishing scene state from Gazebo topic '
            f'{pose_topic} to {robot_topic}, {human_topic}, {obstacle_topic}, {map_topic}, and {static_map_topic}.'
        )
        self.get_logger().info(f'Robot odometry is consumed from {robot_odometry_topic} when available.')
        self.get_logger().info(f'Gazebo play/pause state is read from {stats_topic}.')
        self.get_logger().info(f'Static scene metadata is loaded from {scene_config_path}.')
        self.get_logger().info(
            'Map grid configured as '
            f'{self._map_config.width}x{self._map_config.height} cells at '
            f'{self._map_config.resolution:.3f} m/cell from origin '
            f'({self._map_config.origin_x:.2f}, {self._map_config.origin_y:.2f}).'
        )
        self.get_logger().info(
            'Actor motion publishes fixed configured speed only while actual pose changes '
            f'continue within {self._motion_stale_timeout_sec:.2f} s.'
        )
        self.get_logger().info(
            'Actor motion is detected with thresholds '
            f'{self._motion_position_epsilon_m:.4f} m and {self._motion_yaw_epsilon_rad:.4f} rad.'
        )
        self.get_logger().info(
            'Actor motion is forced to static when no pose change is detected for more than '
            f'{self._motion_stale_timeout_sec:.2f} s.'
        )

    def _load_scene_config(
        self,
        scene_config_path: Path,
        pedestrian_path_config_path: Path | None,
        pedestrian_path_name: str,
    ) -> tuple[
        ActorConfig,
        ActorConfig,
        list[PriorityHumanConfig],
        list[ObstacleConfig],
        MapConfig,
        list[BoxConfig],
        list[BoxConfig],
    ]:
        if not scene_config_path.is_file():
            raise FileNotFoundError(f'Scene config not found: {scene_config_path}')

        with scene_config_path.open('r', encoding='utf-8') as file_handle:
            data = yaml.safe_load(file_handle)

        models = data.get('models', {})
        robot = self._expect_mapping(models, 'robot')
        human = self._expect_mapping(models, 'human')
        pedestrians = data.get('pedestrians', [])
        walls = models.get('walls', [])
        obstacles = models.get('obstacles', [])
        map_static_boxes = models.get('map_static_boxes', [])
        if pedestrians is None:
            pedestrians = []
        if not isinstance(pedestrians, list):
            raise ValueError('pedestrians must be a list in the scene config.')
        if not isinstance(walls, list):
            raise ValueError('models.walls must be a list in the scene config.')
        if not isinstance(obstacles, list):
            raise ValueError('models.obstacles must be a list in the scene config.')
        if not isinstance(map_static_boxes, list):
            raise ValueError('models.map_static_boxes must be a list in the scene config.')

        robot_config = ActorConfig(
            entity_name=str(robot.get('entity_name', robot.get('name', 'starship_delivery_robot_model'))),
            actor_id=str(robot.get('name', 'starship_delivery_robot_model')),
            actor_type=str(robot.get('actor_type', 'robot')),
            fallback_pose=self._parse_pose_entry(robot),
            fallback_linear_speed=float(robot.get('linear_speed', 0.0)),
            pose_source_entity_name=str(
                robot.get(
                    'state_source_entity_name',
                    robot.get(
                        'command_entity_name',
                        robot.get('entity_name', robot.get('name', 'starship_delivery_robot_model')),
                    ),
                )
            ),
            pose_source_offset=self._parse_xy_yaw_offset(
                robot.get(
                    'state_pose_offset',
                    robot.get('command_offset', [0.0, 0.0, 0.0]),
                )
            ),
        )
        human_config = ActorConfig(
            entity_name=str(human.get('entity_name', human.get('name', 'human_in_elevator'))),
            actor_id=str(human.get('name', 'human_in_elevator')),
            actor_type=str(human.get('actor_type', 'human')),
            fallback_pose=self._parse_pose_entry(human),
            fallback_linear_speed=float(human.get('linear_speed', 0.0)),
            pose_source_entity_name=str(human.get('entity_name', human.get('name', 'human_in_elevator'))),
            pose_source_offset=(0.0, 0.0, 0.0),
        )
        priority_human_candidates = self._parse_priority_humans(pedestrians)

        map_config = self._parse_map_config(data.get('map', {}))

        obstacle_configs = [self._parse_obstacle(item) for item in obstacles]

        profile = load_pedestrian_path_profile(pedestrian_path_config_path, pedestrian_path_name)
        if profile is not None and profile.pose is not None:
            pose = (profile.pose[0], profile.pose[1], profile.pose[5])
            human_config = replace(human_config, fallback_pose=pose)

            collision_entity_name = ''
            if pedestrians:
                first_pedestrian = pedestrians[0]
                if isinstance(first_pedestrian, dict):
                    collision_entity_name = str(first_pedestrian.get('collision_entity_name', '')).strip()
            if collision_entity_name:
                obstacle_configs = [
                    replace(obstacle, fallback_pose=pose)
                    if obstacle.entity_name == collision_entity_name
                    else obstacle
                    for obstacle in obstacle_configs
                ]
            self.get_logger().info(
                f'Loaded pedestrian path profile `{pedestrian_path_name}` from {pedestrian_path_config_path}.'
            )

        static_boxes = [
            self._parse_box(
                item,
                default_name='wall',
                inflate_radius=map_config.static_inflation_radius,
            )
            for item in walls
        ]
        static_boxes.extend(
            [
                self._parse_box(
                    item,
                    default_name='map_static_box',
                    inflate_radius=map_config.static_inflation_radius,
                )
                for item in map_static_boxes
            ]
        )
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

        return (
            robot_config,
            human_config,
            priority_human_candidates,
            obstacle_configs,
            map_config,
            static_boxes,
            dynamic_boxes,
        )

    def _parse_priority_humans(self, pedestrians: list[Any]) -> list[PriorityHumanConfig]:
        candidates: list[PriorityHumanConfig] = []
        for item in pedestrians:
            if not isinstance(item, dict):
                raise ValueError('Each pedestrian entry must be a mapping in the scene config.')
            pose = item.get('pose', [])
            if not isinstance(pose, list) or len(pose) < 6:
                raise ValueError('Each pedestrian pose must contain [x, y, z, roll, pitch, yaw].')
            actor = ActorConfig(
                entity_name=str(item.get('entity_name', item.get('name', 'human'))),
                actor_id=str(item.get('name', item.get('entity_name', 'human'))),
                actor_type='human',
                fallback_pose=self._parse_pose_entry(item),
                fallback_linear_speed=float(item.get('linear_speed', 0.0)),
                pose_source_entity_name=str(item.get('entity_name', item.get('name', 'human'))),
                pose_source_offset=(0.0, 0.0, 0.0),
            )
            candidates.append(
                PriorityHumanConfig(
                    actor=actor,
                    start_delay_sec=float(item.get('start_delay_sec', 0.0)),
                )
            )
        return candidates

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

    def _parse_xy_yaw_offset(self, data: Any) -> tuple[float, float, float]:
        if not isinstance(data, list) or len(data) < 3:
            raise ValueError('Actor pose offset must be [x, y, yaw].')
        return (float(data[0]), float(data[1]), float(data[2]))

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
            inflate_radius=float(data.get('inflate_radius', inflate_radius)),
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

    def _run_stats_reader(self) -> None:
        command = [
            'gz',
            'topic',
            '-e',
            '-t',
            self._gazebo_stats_topic,
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
                self.get_logger().error('Could not find the `gz` executable needed to read Gazebo stats.')
                return

            self._stats_reader_process = process
            self.get_logger().info(f'Started Gazebo stats reader: {" ".join(command)}')

            stderr_output = ''
            try:
                if process.stdout is None:
                    raise RuntimeError('Gazebo stats reader has no stdout stream.')

                for line in process.stdout:
                    if self._stop_event.is_set():
                        break
                    line = line.strip()
                    if not line:
                        continue
                    self._on_stats_json_line(line)
            except Exception as error:
                self.get_logger().warning(f'Gazebo stats reader hit an error: {error}')
            finally:
                if process.stdout is not None:
                    process.stdout.close()
                if process.stderr is not None:
                    stderr_output = process.stderr.read().strip()
                    process.stderr.close()
                self._stop_process(process)
                if self._stats_reader_process is process:
                    self._stats_reader_process = None

            if self._stop_event.is_set():
                break
            if stderr_output:
                self.get_logger().warning(f'Gazebo stats reader exited and will retry: {stderr_output}')
            else:
                self.get_logger().warning('Gazebo stats reader exited and will retry.')
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
        latest_pose_stamps: dict[str, float] = {}
        receive_time_sec = time.monotonic()
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
            latest_pose_stamps[entity_name] = receive_time_sec

        with self._pose_lock:
            self._latest_poses = latest_poses
            self._latest_pose_stamps = latest_pose_stamps

    def _on_stats_json_line(self, line: str) -> None:
        data = json.loads(line)
        sim_time = data.get('simTime', {})
        if not isinstance(sim_time, dict):
            return
        sec = float(sim_time.get('sec', 0.0))
        nsec = float(sim_time.get('nsec', 0.0))
        sim_time_sec = sec + nsec * 1e-9
        with self._stats_lock:
            if self._latest_sim_time_sec is None:
                self._gazebo_is_paused = sim_time_sec <= 0.0
            else:
                self._gazebo_is_paused = abs(sim_time_sec - self._latest_sim_time_sec) <= 1e-9
            self._latest_sim_time_sec = sim_time_sec

    def _on_robot_odometry(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        odom_yaw = math.atan2(
            2.0 * (
                float(orientation.w) * float(orientation.z) +
                float(orientation.x) * float(orientation.y)
            ),
            1.0 - 2.0 * (
                float(orientation.y) * float(orientation.y) +
                float(orientation.z) * float(orientation.z)
            ),
        )
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        odom_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(odom_yaw),
        )
        if self._robot_config.fallback_pose is not None:
            world_pose = self._apply_pose_offset(self._robot_config.fallback_pose, odom_pose)
        else:
            world_pose = odom_pose
        with self._robot_odom_lock:
            self._latest_robot_odom_pose = world_pose
            self._latest_robot_odom_twist = (
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
            )
            self._latest_robot_odom_stamp = stamp_sec

    def _get_robot_odometry(self) -> tuple[tuple[float, float, float] | None, tuple[float, float] | None, float | None]:
        with self._robot_odom_lock:
            return (
                self._latest_robot_odom_pose,
                self._latest_robot_odom_twist,
                self._latest_robot_odom_stamp,
            )

    def _find_entity_pose(
        self,
        entity_name: str,
        fallback_pose: tuple[float, float, float] | None,
    ) -> tuple[tuple[float, float, float] | None, float | None]:
        with self._pose_lock:
            exact_pose = self._latest_poses.get(entity_name)
            exact_stamp = self._latest_pose_stamps.get(entity_name)
        if exact_pose is not None:
            return exact_pose, exact_stamp

        prefix = entity_name + '::'
        with self._pose_lock:
            latest_poses = dict(self._latest_poses)
            latest_pose_stamps = dict(self._latest_pose_stamps)
        for pose_name, pose in latest_poses.items():
            if pose_name.startswith(prefix):
                return pose, latest_pose_stamps.get(pose_name)
        return fallback_pose, None

    def _apply_pose_offset(
        self,
        base_pose: tuple[float, float, float],
        offset: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        base_x, base_y, base_yaw = base_pose
        offset_x, offset_y, offset_yaw = offset
        world_x = base_x + math.cos(base_yaw) * offset_x - math.sin(base_yaw) * offset_y
        world_y = base_y + math.sin(base_yaw) * offset_x + math.cos(base_yaw) * offset_y
        world_yaw = self._normalize_angle(base_yaw + offset_yaw)
        return (world_x, world_y, world_yaw)

    def _resolve_actor_pose(self, actor: ActorConfig) -> tuple[tuple[float, float, float] | None, float | None]:
        source_pose, source_stamp = self._find_entity_pose(actor.pose_source_entity_name, None)
        if source_pose is not None:
            return self._apply_pose_offset(source_pose, actor.pose_source_offset), source_stamp
        return actor.fallback_pose, None

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _is_simulation_paused(self) -> bool:
        with self._stats_lock:
            return self._gazebo_is_paused

    def _estimate_actor_motion(
        self,
        actor_key: str,
        pose: tuple[float, float, float],
        stamp_sec: float | None,
        fallback_linear_speed: float,
    ) -> tuple[float, float, bool]:
        if stamp_sec is None:
            return 0.0, 0.0, False

        if self._is_simulation_paused():
            return 0.0, 0.0, False

        previous_sample = self._last_actor_pose_samples.get(actor_key)
        self._last_actor_pose_samples[actor_key] = (pose[0], pose[1], pose[2], stamp_sec)

        if previous_sample is None:
            self._last_actor_motion_times.setdefault(actor_key, time.monotonic())

        if previous_sample is not None:
            dx = pose[0] - previous_sample[0]
            dy = pose[1] - previous_sample[1]
            dyaw = self._normalize_angle(pose[2] - previous_sample[2])
            moved = (
                math.hypot(dx, dy) >= self._motion_position_epsilon_m or
                abs(dyaw) >= self._motion_yaw_epsilon_rad
            )
            if moved:
                self._last_actor_motion_times[actor_key] = time.monotonic()

        last_motion_time = self._last_actor_motion_times.get(actor_key)
        if last_motion_time is None:
            return 0.0, 0.0, False
        if time.monotonic() - last_motion_time > self._motion_stale_timeout_sec:
            return 0.0, 0.0, False

        linear_speed = abs(float(fallback_linear_speed))
        is_moving = linear_speed > 1e-3
        return linear_speed, 0.0, is_moving

    def _select_priority_human(
        self,
        robot_pose: tuple[float, float, float] | None,
    ) -> tuple[ActorConfig, tuple[float, float, float], float | None] | None:
        if not self._priority_human_candidates:
            return None

        best_candidate: tuple[float, ActorConfig, tuple[float, float, float], float | None] | None = None
        current_candidate: tuple[float, ActorConfig, tuple[float, float, float], float | None] | None = None

        for candidate in self._priority_human_candidates:
            pose, stamp = self._resolve_actor_pose(candidate.actor)
            if pose is None:
                continue
            score = self._score_priority_human(robot_pose, pose)
            payload = (score, candidate.actor, pose, stamp)
            if candidate.actor.actor_id == self._selected_priority_human_id:
                current_candidate = payload
            if best_candidate is None or score > best_candidate[0]:
                best_candidate = payload

        if best_candidate is None:
            return None

        selected = best_candidate
        if current_candidate is not None and current_candidate[0] >= best_candidate[0] - 0.75:
            selected = current_candidate

        self._selected_priority_human_id = selected[1].actor_id
        return selected[1], selected[2], selected[3]

    def _score_priority_human(
        self,
        robot_pose: tuple[float, float, float] | None,
        human_pose: tuple[float, float, float],
    ) -> float:
        if robot_pose is None:
            return 0.0

        dx = human_pose[0] - robot_pose[0]
        dy = human_pose[1] - robot_pose[1]
        distance = math.hypot(dx, dy)
        if distance > 12.0:
            return -distance

        local_x, local_y = self._transform_world_to_local_pose(robot_pose, human_pose[0], human_pose[1])
        score = -distance
        if local_x >= -0.5:
            score += 3.0
        if local_x >= 0.0:
            score += 2.0
        if abs(local_y) <= 0.9:
            score += 2.5
        elif abs(local_y) <= 1.8:
            score += 1.0
        if 0.0 <= local_x <= 4.0:
            score += 1.5
        elif -1.0 <= local_x < 0.0:
            score += 0.5
        return score

    def _transform_world_to_local_pose(
        self,
        robot_pose: tuple[float, float, float],
        x: float,
        y: float,
    ) -> tuple[float, float]:
        dx = x - robot_pose[0]
        dy = y - robot_pose[1]
        cos_yaw = math.cos(robot_pose[2])
        sin_yaw = math.sin(robot_pose[2])
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
        )

    def _build_static_map_data(self) -> list[int]:
        grid = [0] * (self._map_config.width * self._map_config.height)
        for box in self._static_boxes:
            pose, _ = self._find_entity_pose(box.entity_name, box.fallback_pose)
            pose = pose or box.fallback_pose
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
            pose, _ = self._find_entity_pose(box.entity_name, box.fallback_pose)
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

        robot_pose, robot_twist, robot_stamp = self._get_robot_odometry()
        if robot_pose is None:
            fallback_pose, fallback_stamp = self._resolve_actor_pose(self._robot_config)
            robot_pose = fallback_pose
            robot_stamp = fallback_stamp
        if robot_pose is not None:
            if robot_twist is not None:
                robot_linear_x = float(robot_twist[0])
                robot_angular_z = float(robot_twist[1])
                robot_is_moving = abs(robot_linear_x) >= 1e-3 or abs(robot_angular_z) >= 1e-3
            else:
                robot_linear_x, robot_angular_z, robot_is_moving = self._estimate_actor_motion(
                    self._robot_config.actor_id,
                    robot_pose,
                    robot_stamp,
                    self._robot_config.fallback_linear_speed,
                )
            robot_msg = ActorState()
            robot_msg.header.stamp = now
            robot_msg.header.frame_id = self._frame_id
            robot_msg.actor_id = self._robot_config.actor_id
            robot_msg.actor_type = self._robot_config.actor_type
            robot_msg.x = robot_pose[0]
            robot_msg.y = robot_pose[1]
            robot_msg.yaw = robot_pose[2]
            robot_msg.linear_x = float(robot_linear_x)
            robot_msg.nominal_linear_x = float(self._robot_config.fallback_linear_speed)
            robot_msg.angular_z = float(robot_angular_z)
            robot_msg.is_moving = bool(robot_is_moving)
            self._robot_publisher.publish(robot_msg)

        selected_human: tuple[ActorConfig, tuple[float, float, float], float | None] | None = None
        if self._use_priority_human_selector and self._priority_human_candidates:
            selected_human = self._select_priority_human(robot_pose)
        elif (fallback_human_pose := self._resolve_actor_pose(self._human_config))[0] is not None:
            selected_human = (self._human_config, fallback_human_pose[0], fallback_human_pose[1])

        if selected_human is not None:
            human_actor, human_pose, human_stamp = selected_human
            human_linear_x, human_angular_z, human_is_moving = self._estimate_actor_motion(
                human_actor.actor_id,
                human_pose,
                human_stamp,
                human_actor.fallback_linear_speed,
            )
            human_msg = ActorState()
            human_msg.header.stamp = now
            human_msg.header.frame_id = self._frame_id
            human_msg.actor_id = human_actor.actor_id
            human_msg.actor_type = human_actor.actor_type
            human_msg.x = human_pose[0]
            human_msg.y = human_pose[1]
            human_msg.yaw = human_pose[2]
            human_msg.linear_x = float(human_linear_x)
            human_msg.nominal_linear_x = float(human_actor.fallback_linear_speed)
            human_msg.angular_z = float(human_angular_z)
            human_msg.is_moving = bool(human_is_moving)
            self._human_publisher.publish(human_msg)

        for obstacle in self._obstacle_configs:
            obstacle_pose, _ = self._find_entity_pose(obstacle.entity_name, obstacle.fallback_pose)
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
        if self._stats_reader_process is not None:
            self._stop_process(self._stats_reader_process)
        if self._pose_reader_thread.is_alive():
            self._pose_reader_thread.join(timeout=2.0)
        if self._stats_reader_thread.is_alive():
            self._stats_reader_thread.join(timeout=2.0)
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
