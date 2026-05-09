from __future__ import annotations

import json
import math
import subprocess
import threading
from pathlib import Path
from typing import Any

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from hmi_scene.pedestrian_path_profiles import load_pedestrian_path_profile
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float64


class ActorForwardTestController(Node):
    def __init__(self) -> None:
        super().__init__('human_motion_controller')

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
        self._world_name = str(
            self.declare_parameter(
                'world_name',
                'elevator_yield',
                ParameterDescriptor(description='Gazebo world name used for set_pose requests.'),
            ).value
        )
        self._visual_entity_name = str(
            self.declare_parameter(
                'visual_entity_name',
                'human_in_elevator',
                ParameterDescriptor(description='Visible human model name.'),
            ).value
        )
        self._collision_entity_name = str(
            self.declare_parameter(
                'collision_entity_name',
                'human_collision_proxy',
                ParameterDescriptor(description='Gazebo collision proxy entity that should stay aligned with the visible human.'),
            ).value
        )
        self._joint_topic_prefix = str(
            self.declare_parameter(
                'joint_topic_prefix',
                '/human_in_elevator',
                ParameterDescriptor(
                    description='ROS topic prefix used for the arm / leg joint command topics.'
                ),
            ).value
        ).rstrip('/')
        self._linear_speed = float(
            self.declare_parameter(
                'linear_speed',
                1.00,
                ParameterDescriptor(description='Forward speed in meters per second for the moving human test.'),
            ).value
        )
        self._travel_distance = float(
            self.declare_parameter(
                'travel_distance',
                6.20,
                ParameterDescriptor(description='Fallback forward travel distance in meters when no waypoints are configured.'),
            ).value
        )
        self._turn_speed = float(
            self.declare_parameter(
                'turn_speed_rad_s',
                1.0,
                ParameterDescriptor(description='In-place turning speed in radians per second toward the next waypoint.'),
            ).value
        )
        self._position_tolerance = float(
            self.declare_parameter(
                'position_tolerance_m',
                0.03,
                ParameterDescriptor(description='Distance tolerance in meters for considering a waypoint reached.'),
            ).value
        )
        self._yaw_tolerance = float(
            self.declare_parameter(
                'yaw_tolerance_rad',
                0.03,
                ParameterDescriptor(description='Yaw tolerance in radians for considering a turn complete.'),
            ).value
        )
        self._stride_length = float(
            self.declare_parameter(
                'stride_length',
                0.72,
                ParameterDescriptor(description='Walking stride length in meters used to drive limb swing phase.'),
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
                ParameterDescriptor(description='Delay in simulation seconds after pressing play before the human starts moving.'),
            ).value
        )
        update_rate_hz = float(
            self.declare_parameter(
                'update_rate_hz',
                60.0,
                ParameterDescriptor(description='Update rate for sending human set_pose commands.'),
            ).value
        )

        (
            self._start_x,
            self._start_y,
            self._visual_z,
            self._start_yaw,
            self._collision_z,
            self._waypoints,
        ) = self._load_scene_poses(
            scene_config_path,
            pedestrian_path_config_path,
            pedestrian_path_name,
        )
        self._set_pose_service = f'/world/{self._world_name}/set_pose'
        self._stats_topic = f'/world/{self._world_name}/stats'
        self._set_pose_client = self.create_client(SetEntityPose, self._set_pose_service)
        self._left_arm_publisher = self.create_publisher(
            Float64, f'{self._joint_topic_prefix}/left_arm_joint_cmd', 10
        )
        self._right_arm_publisher = self.create_publisher(
            Float64, f'{self._joint_topic_prefix}/right_arm_joint_cmd', 10
        )
        self._left_leg_publisher = self.create_publisher(
            Float64, f'{self._joint_topic_prefix}/left_leg_joint_cmd', 10
        )
        self._right_leg_publisher = self.create_publisher(
            Float64, f'{self._joint_topic_prefix}/right_leg_joint_cmd', 10
        )
        self._pending_pose_futures: list[Any] = []
        self._warned_service_unavailable = False
        self._stats_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._stats_reader_process: subprocess.Popen[str] | None = None
        self._stats_reader_thread = threading.Thread(target=self._run_stats_reader, daemon=True)
        self._stats_reader_thread.start()
        self._simulation_started = False
        self._motion_start_sim_time: float | None = None
        self._latest_sim_time_sec: float | None = None
        self._last_motion_sim_time: float | None = None
        self._current_x = self._start_x
        self._current_y = self._start_y
        self._current_yaw = self._start_yaw
        self._waypoint_index = 0
        self._motion_phase = 'turn'
        self._travelled_distance = 0.0
        self._completed = False

        self.create_timer(1.0 / update_rate_hz, self._update_motion)
        self.get_logger().info(
            'Human motion controller is ready. '
            f'Initial visual entity: {self._visual_entity_name}, collision entity: {self._collision_entity_name}, '
            f'joint topic prefix: {self._joint_topic_prefix}, speed: {self._linear_speed:.2f} m/s, '
            f'waypoints: {len(self._waypoints)}.'
        )

    def _load_scene_poses(
        self,
        scene_config_path: Path,
        pedestrian_path_config_path: Path | None,
        pedestrian_path_name: str,
    ) -> tuple[float, float, float, float, float, list[tuple[float, float]]]:
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
        human_pose = human.get('pose', [])
        if not isinstance(human_pose, list) or len(human_pose) < 6:
            raise ValueError('models.human.pose must contain [x, y, z, roll, pitch, yaw].')
        waypoints = self._parse_waypoints(human, human_pose)

        profile = load_pedestrian_path_profile(pedestrian_path_config_path, pedestrian_path_name)
        if profile is not None:
            if profile.pose is not None:
                human_pose = list(profile.pose)
            if profile.waypoints is not None:
                waypoints = profile.waypoints
            self.get_logger().info(
                f'Loaded pedestrian path profile `{pedestrian_path_name}` from {pedestrian_path_config_path}.'
            )

        obstacles = models.get('obstacles', [])
        if not isinstance(obstacles, list):
            raise ValueError('models.obstacles must be a list in the scene config.')

        collision_pose: list[Any] | None = None
        for obstacle in obstacles:
            if not isinstance(obstacle, dict):
                continue
            entity_name = str(obstacle.get('entity_name', obstacle.get('name', ''))).strip()
            if entity_name == self._collision_entity_name:
                raw_pose = obstacle.get('pose', [])
                if isinstance(raw_pose, list) and len(raw_pose) >= 6:
                    collision_pose = raw_pose
                break

        if collision_pose is None:
            raise ValueError(f'Could not find pose for collision entity {self._collision_entity_name} in models.obstacles.')

        return (
            float(human_pose[0]),
            float(human_pose[1]),
            float(human_pose[2]),
            float(human_pose[5]),
            float(collision_pose[2]),
            waypoints,
        )

    def _parse_waypoints(self, human: dict[str, Any], human_pose: list[Any]) -> list[tuple[float, float]]:
        raw_waypoints = human.get('waypoints', [])
        if raw_waypoints is None:
            raw_waypoints = []
        if not isinstance(raw_waypoints, list):
            raise ValueError('models.human.waypoints must be a list of [x, y] points.')

        waypoints: list[tuple[float, float]] = []
        for item in raw_waypoints:
            if not isinstance(item, list) or len(item) < 2:
                raise ValueError('Each human waypoint must be [x, y].')
            waypoints.append((float(item[0]), float(item[1])))

        if waypoints:
            return waypoints

        start_x = float(human_pose[0])
        start_y = float(human_pose[1])
        start_yaw = float(human_pose[5])
        fallback_x = start_x + math.cos(start_yaw) * self._travel_distance
        fallback_y = start_y + math.sin(start_yaw) * self._travel_distance
        return [(fallback_x, fallback_y)]

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

    def _get_stats_snapshot(self) -> float | None:
        with self._stats_lock:
            return self._latest_sim_time_sec

    def _update_motion(self) -> None:
        if self._completed:
            return

        sim_time_sec = self._get_stats_snapshot()
        if sim_time_sec is None:
            return

        if not self._simulation_started:
            if sim_time_sec <= 0.0:
                return
            self._simulation_started = True
            self._motion_start_sim_time = sim_time_sec
            self._last_motion_sim_time = sim_time_sec
            self._set_entity_poses(self._current_x, self._current_y, self._current_yaw)
            self._publish_joint_commands(0.0)
            return

        if self._motion_start_sim_time is None:
            self._motion_start_sim_time = sim_time_sec
        if sim_time_sec - self._motion_start_sim_time < self._start_delay_sec:
            self._last_motion_sim_time = sim_time_sec
            self._publish_joint_commands(0.0)
            return

        if self._last_motion_sim_time is None:
            self._last_motion_sim_time = sim_time_sec
            return

        dt = sim_time_sec - self._last_motion_sim_time
        self._last_motion_sim_time = sim_time_sec
        if dt <= 0.0:
            return

        if self._waypoint_index >= len(self._waypoints):
            self._set_entity_poses(self._current_x, self._current_y, self._current_yaw)
            self._publish_joint_commands(0.0)
            self._completed = True
            self.get_logger().info('Human motion test completed.')
            return

        self._pending_pose_futures = [future for future in self._pending_pose_futures if not future.done()]
        if len(self._pending_pose_futures) >= 4:
            return

        target_x, target_y = self._waypoints[self._waypoint_index]
        dx = target_x - self._current_x
        dy = target_y - self._current_y
        target_yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else self._current_yaw

        if self._motion_phase == 'turn':
            yaw_error = self._normalize_angle(target_yaw - self._current_yaw)
            if abs(yaw_error) <= self._yaw_tolerance:
                self._current_yaw = target_yaw
                self._motion_phase = 'walk'
            else:
                yaw_step = min(self._turn_speed * dt, abs(yaw_error))
                self._current_yaw = self._normalize_angle(self._current_yaw + math.copysign(yaw_step, yaw_error))
            self._set_entity_poses(self._current_x, self._current_y, self._current_yaw)
            self._publish_joint_commands(0.0)
            return

        distance_to_target = math.hypot(dx, dy)
        if distance_to_target <= self._position_tolerance:
            self._current_x = target_x
            self._current_y = target_y
            self._current_yaw = target_yaw
            self._waypoint_index += 1
            self._motion_phase = 'turn'
            self._set_entity_poses(self._current_x, self._current_y, self._current_yaw)
            self._publish_joint_commands(0.0)
            return

        step = min(self._linear_speed * dt, distance_to_target)
        direction_x = dx / max(distance_to_target, 1e-9)
        direction_y = dy / max(distance_to_target, 1e-9)
        self._current_x += direction_x * step
        self._current_y += direction_y * step
        self._current_yaw = target_yaw
        self._travelled_distance += step

        self._set_entity_poses(self._current_x, self._current_y, self._current_yaw)
        self._publish_joint_commands(self._travelled_distance)

    def _set_entity_poses(self, x: float, y: float, yaw: float) -> None:
        if not self._set_pose_client.service_is_ready():
            if not self._warned_service_unavailable:
                self.get_logger().info(f'Waiting for ROS bridge service {self._set_pose_service}...')
                self._warned_service_unavailable = True
            return
        self._warned_service_unavailable = False

        visual_quaternion = self._quaternion_from_rpy(0.0, 0.0, yaw)
        collision_quaternion = self._quaternion_from_rpy(0.0, 0.0, yaw)

        visual_request = self._make_pose_request(
            self._visual_entity_name,
            x,
            y,
            self._visual_z,
            visual_quaternion,
        )
        collision_request = self._make_pose_request(
            self._collision_entity_name,
            x,
            y,
            self._collision_z,
            collision_quaternion,
        )
        self._pending_pose_futures.append(self._set_pose_client.call_async(visual_request))
        self._pending_pose_futures.append(self._set_pose_client.call_async(collision_request))

    def _publish_joint_commands(self, travelled_distance: float) -> None:
        phase = 2.0 * math.pi * travelled_distance / max(self._stride_length, 1e-6)
        arm_angle = self._arm_swing_amplitude * math.sin(phase)
        leg_angle = self._leg_swing_amplitude * math.sin(phase)

        self._left_arm_publisher.publish(Float64(data=arm_angle))
        self._right_arm_publisher.publish(Float64(data=-arm_angle))
        self._left_leg_publisher.publish(Float64(data=-leg_angle))
        self._right_leg_publisher.publish(Float64(data=leg_angle))

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
    node = ActorForwardTestController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
