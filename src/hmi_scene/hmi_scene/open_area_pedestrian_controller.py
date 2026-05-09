from __future__ import annotations

import json
import math
import subprocess
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Float64


@dataclass
class PedestrianRuntime:
    name: str
    visual_entity_name: str
    collision_entity_name: str
    visual_z: float
    collision_z: float
    linear_speed: float
    start_delay_sec: float
    current_x: float
    current_y: float
    current_yaw: float
    waypoints: list[tuple[float, float]]
    travelled_distance: float = 0.0
    waypoint_index: int = 0
    motion_phase: str = 'turn'
    completed: bool = False


class OpenAreaPedestrianController(Node):
    def __init__(self) -> None:
        super().__init__('open_area_pedestrian_controller')

        default_scene_config = str(
            Path(get_package_share_directory('hmi_scene')) / 'open_area_scene.yaml'
        )
        scene_config_path = Path(
            self.declare_parameter(
                'scene_config_path',
                default_scene_config,
                ParameterDescriptor(description='Path to the YAML scene config for the open-area pedestrians.'),
            ).value
        )
        self._world_name = str(
            self.declare_parameter(
                'world_name',
                'open_area',
                ParameterDescriptor(description='Gazebo world name used for set_pose requests.'),
            ).value
        )
        self._turn_speed = float(
            self.declare_parameter(
                'turn_speed_rad_s',
                1.2,
                ParameterDescriptor(description='In-place turn speed toward the next waypoint.'),
            ).value
        )
        self._position_tolerance = float(
            self.declare_parameter(
                'position_tolerance_m',
                0.04,
                ParameterDescriptor(description='Distance threshold for considering a waypoint reached.'),
            ).value
        )
        self._yaw_tolerance = float(
            self.declare_parameter(
                'yaw_tolerance_rad',
                0.04,
                ParameterDescriptor(description='Yaw threshold for considering a turn complete.'),
            ).value
        )
        self._stride_length = float(
            self.declare_parameter(
                'stride_length',
                0.72,
                ParameterDescriptor(description='Stride length used to drive the limb swing phase.'),
            ).value
        )
        self._arm_swing_amplitude = float(
            self.declare_parameter(
                'arm_swing_amplitude_rad',
                0.55,
                ParameterDescriptor(description='Peak shoulder swing angle in radians.'),
            ).value
        )
        self._leg_swing_amplitude = float(
            self.declare_parameter(
                'leg_swing_amplitude_rad',
                0.35,
                ParameterDescriptor(description='Peak hip swing angle in radians.'),
            ).value
        )
        self._start_delay_sec = float(
            self.declare_parameter(
                'start_delay_sec',
                0.0,
                ParameterDescriptor(description='Delay in simulation seconds after pressing play before pedestrians start.'),
            ).value
        )
        update_rate_hz = float(
            self.declare_parameter(
                'update_rate_hz',
                30.0,
                ParameterDescriptor(description='Update rate for sending pedestrian set_pose commands.'),
            ).value
        )

        self._pedestrians = self._load_scene_config(scene_config_path)
        self._set_pose_service = f'/world/{self._world_name}/set_pose'
        self._stats_topic = f'/world/{self._world_name}/stats'
        self._set_pose_client = self.create_client(SetEntityPose, self._set_pose_service)
        self._pending_pose_futures: list[Any] = []
        self._warned_service_unavailable = False

        self._joint_publishers: dict[str, dict[str, Any]] = {}
        for pedestrian in self._pedestrians:
            self._joint_publishers[pedestrian.name] = {
                'left_arm': self.create_publisher(Float64, f'/model/{pedestrian.name}/left_arm_joint_cmd', 10),
                'right_arm': self.create_publisher(Float64, f'/model/{pedestrian.name}/right_arm_joint_cmd', 10),
                'left_leg': self.create_publisher(Float64, f'/model/{pedestrian.name}/left_leg_joint_cmd', 10),
                'right_leg': self.create_publisher(Float64, f'/model/{pedestrian.name}/right_leg_joint_cmd', 10),
            }

        self._stats_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._stats_reader_process: subprocess.Popen[str] | None = None
        self._stats_reader_thread = threading.Thread(target=self._run_stats_reader, daemon=True)
        self._stats_reader_thread.start()

        self._simulation_started = False
        self._motion_start_sim_time: float | None = None
        self._latest_sim_time_sec: float | None = None
        self._last_motion_sim_time: float | None = None

        self.create_timer(1.0 / max(update_rate_hz, 1.0), self._update_motion)
        self.get_logger().info(
            f'Open-area pedestrian controller is ready for {len(self._pedestrians)} pedestrians in world '
            f'{self._world_name}.'
        )

    def _load_scene_config(self, scene_config_path: Path) -> list[PedestrianRuntime]:
        if not scene_config_path.is_file():
            raise FileNotFoundError(f'Scene config not found: {scene_config_path}')

        with scene_config_path.open('r', encoding='utf-8') as file_handle:
            data = yaml.safe_load(file_handle)

        pedestrians = data.get('pedestrians', [])
        if not isinstance(pedestrians, list) or not pedestrians:
            raise ValueError('pedestrians must be a non-empty list in the scene config.')

        models = data.get('models', {})
        obstacles = models.get('obstacles', []) if isinstance(models, dict) else []
        if not isinstance(obstacles, list):
            raise ValueError('models.obstacles must be a list in the scene config.')

        obstacle_pose_by_entity: dict[str, list[Any]] = {}
        for obstacle in obstacles:
            if not isinstance(obstacle, dict):
                continue
            entity_name = str(obstacle.get('entity_name', obstacle.get('name', ''))).strip()
            pose = obstacle.get('pose', [])
            if entity_name and isinstance(pose, list) and len(pose) >= 6:
                obstacle_pose_by_entity[entity_name] = pose

        runtimes: list[PedestrianRuntime] = []
        for pedestrian in pedestrians:
            if not isinstance(pedestrian, dict):
                raise ValueError('Each pedestrian entry must be a mapping.')

            pose = pedestrian.get('pose', [])
            if not isinstance(pose, list) or len(pose) < 6:
                raise ValueError('Each pedestrian pose must contain [x, y, z, roll, pitch, yaw].')

            collision_entity_name = str(pedestrian.get('collision_entity_name', '')).strip()
            collision_pose = obstacle_pose_by_entity.get(collision_entity_name)
            if collision_pose is None:
                raise ValueError(
                    f'Could not find pose for collision entity {collision_entity_name} in models.obstacles.'
                )

            waypoints = pedestrian.get('waypoints', [])
            if not isinstance(waypoints, list):
                raise ValueError('Each pedestrian waypoints entry must be a list.')

            parsed_waypoints: list[tuple[float, float]] = []
            for item in waypoints:
                if not isinstance(item, list) or len(item) < 2:
                    raise ValueError('Each pedestrian waypoint must be [x, y].')
                parsed_waypoints.append((float(item[0]), float(item[1])))

            runtimes.append(
                PedestrianRuntime(
                    name=str(pedestrian.get('name', pedestrian.get('entity_name', 'pedestrian'))),
                    visual_entity_name=str(pedestrian.get('entity_name', pedestrian.get('name', 'pedestrian'))),
                    collision_entity_name=collision_entity_name,
                    visual_z=float(pose[2]),
                    collision_z=float(collision_pose[2]),
                    linear_speed=float(pedestrian.get('linear_speed', 1.0)),
                    start_delay_sec=float(pedestrian.get('start_delay_sec', 0.0)),
                    current_x=float(pose[0]),
                    current_y=float(pose[1]),
                    current_yaw=float(pose[5]),
                    waypoints=parsed_waypoints,
                    completed=(len(parsed_waypoints) == 0),
                )
            )

        return runtimes

    def _run_stats_reader(self) -> None:
        command = [
            'gz',
            'topic',
            '-e',
            '-t',
            self._stats_topic,
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

    def _on_stats_json_line(self, line: str) -> None:
        data = json.loads(line)
        sim_time = data.get('simTime', {})
        if not isinstance(sim_time, dict):
            return

        sec = float(sim_time.get('sec', 0.0))
        nsec = float(sim_time.get('nsec', 0.0))
        sim_time_sec = sec + nsec * 1e-9

        with self._stats_lock:
            self._latest_sim_time_sec = sim_time_sec

    def _get_sim_time(self) -> float | None:
        with self._stats_lock:
            return self._latest_sim_time_sec

    def _update_motion(self) -> None:
        sim_time_sec = self._get_sim_time()
        if sim_time_sec is None:
            return

        if not self._simulation_started:
            if sim_time_sec <= 0.0:
                return
            self._simulation_started = True
            self._motion_start_sim_time = sim_time_sec
            self._last_motion_sim_time = sim_time_sec
            for pedestrian in self._pedestrians:
                self._set_entity_poses(pedestrian)
                self._publish_joint_commands(pedestrian, 0.0)
            return

        if self._motion_start_sim_time is None:
            self._motion_start_sim_time = sim_time_sec
        if sim_time_sec - self._motion_start_sim_time < self._start_delay_sec:
            self._last_motion_sim_time = sim_time_sec
            for pedestrian in self._pedestrians:
                self._publish_joint_commands(pedestrian, 0.0)
            return

        if self._last_motion_sim_time is None:
            self._last_motion_sim_time = sim_time_sec
            return

        dt = sim_time_sec - self._last_motion_sim_time
        self._last_motion_sim_time = sim_time_sec
        if dt <= 0.0:
            return

        self._pending_pose_futures = [future for future in self._pending_pose_futures if not future.done()]
        if len(self._pending_pose_futures) >= max(8, len(self._pedestrians) * 4):
            return

        for pedestrian in self._pedestrians:
            self._update_one_pedestrian(pedestrian, dt)

    def _update_one_pedestrian(self, pedestrian: PedestrianRuntime, dt: float) -> None:
        if self._motion_start_sim_time is not None:
            if self._last_motion_sim_time is not None:
                elapsed_sim_time = self._last_motion_sim_time - self._motion_start_sim_time
            else:
                elapsed_sim_time = 0.0
            if elapsed_sim_time < pedestrian.start_delay_sec:
                self._set_entity_poses(pedestrian)
                self._publish_joint_commands(pedestrian, 0.0)
                return

        if pedestrian.completed:
            self._publish_joint_commands(pedestrian, 0.0)
            return

        if pedestrian.waypoint_index >= len(pedestrian.waypoints):
            pedestrian.completed = True
            self._set_entity_poses(pedestrian)
            self._publish_joint_commands(pedestrian, 0.0)
            return

        target_x, target_y = pedestrian.waypoints[pedestrian.waypoint_index]
        dx = target_x - pedestrian.current_x
        dy = target_y - pedestrian.current_y
        distance_to_target = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx) if distance_to_target > 1e-9 else pedestrian.current_yaw

        if pedestrian.motion_phase == 'turn':
            yaw_error = self._normalize_angle(target_yaw - pedestrian.current_yaw)
            if abs(yaw_error) <= self._yaw_tolerance:
                pedestrian.current_yaw = target_yaw
                pedestrian.motion_phase = 'walk'
            else:
                yaw_step = min(self._turn_speed * dt, abs(yaw_error))
                pedestrian.current_yaw = self._normalize_angle(
                    pedestrian.current_yaw + math.copysign(yaw_step, yaw_error)
                )
            self._set_entity_poses(pedestrian)
            self._publish_joint_commands(pedestrian, 0.0)
            return

        if distance_to_target <= self._position_tolerance:
            pedestrian.current_x = target_x
            pedestrian.current_y = target_y
            pedestrian.current_yaw = target_yaw
            pedestrian.waypoint_index += 1
            pedestrian.motion_phase = 'turn'
            if pedestrian.waypoint_index >= len(pedestrian.waypoints):
                pedestrian.completed = True
            self._set_entity_poses(pedestrian)
            self._publish_joint_commands(pedestrian, 0.0)
            return

        step = min(abs(pedestrian.linear_speed) * dt, distance_to_target)
        direction_x = dx / max(distance_to_target, 1e-9)
        direction_y = dy / max(distance_to_target, 1e-9)
        pedestrian.current_x += direction_x * step
        pedestrian.current_y += direction_y * step
        pedestrian.current_yaw = target_yaw
        pedestrian.travelled_distance += step

        self._set_entity_poses(pedestrian)
        self._publish_joint_commands(pedestrian, pedestrian.travelled_distance)

    def _set_entity_poses(self, pedestrian: PedestrianRuntime) -> None:
        if not self._set_pose_client.service_is_ready():
            if not self._warned_service_unavailable:
                self.get_logger().info(f'Waiting for ROS bridge service {self._set_pose_service}...')
                self._warned_service_unavailable = True
            return
        self._warned_service_unavailable = False

        quaternion = self._quaternion_from_rpy(0.0, 0.0, pedestrian.current_yaw)
        visual_request = self._make_pose_request(
            pedestrian.visual_entity_name,
            pedestrian.current_x,
            pedestrian.current_y,
            pedestrian.visual_z,
            quaternion,
        )
        collision_request = self._make_pose_request(
            pedestrian.collision_entity_name,
            pedestrian.current_x,
            pedestrian.current_y,
            pedestrian.collision_z,
            quaternion,
        )
        self._pending_pose_futures.append(self._set_pose_client.call_async(visual_request))
        self._pending_pose_futures.append(self._set_pose_client.call_async(collision_request))

    def _publish_joint_commands(self, pedestrian: PedestrianRuntime, travelled_distance: float) -> None:
        publishers = self._joint_publishers[pedestrian.name]
        phase = 2.0 * math.pi * travelled_distance / max(self._stride_length, 1e-6)
        arm_angle = self._arm_swing_amplitude * math.sin(phase)
        leg_angle = self._leg_swing_amplitude * math.sin(phase)

        publishers['left_arm'].publish(Float64(data=arm_angle))
        publishers['right_arm'].publish(Float64(data=-arm_angle))
        publishers['left_leg'].publish(Float64(data=-leg_angle))
        publishers['right_leg'].publish(Float64(data=leg_angle))

    def _make_pose_request(
        self,
        entity_name: str,
        x: float,
        y: float,
        z: float,
        quaternion: tuple[float, float, float, float],
    ) -> SetEntityPose.Request:
        request = SetEntityPose.Request()
        request.entity = Entity(name=entity_name, type=Entity.MODEL)
        request.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3],
            ),
        )
        return request

    def _quaternion_from_rpy(self, roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)

        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _stop_process(self, process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._stats_reader_process is not None:
            self._stop_process(self._stats_reader_process)
        if self._stats_reader_thread.is_alive():
            self._stats_reader_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OpenAreaPedestrianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
