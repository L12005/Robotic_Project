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
from geometry_msgs.msg import Twist
from hmi_interfaces.msg import ActorState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class RobotCmdVelController(Node):
    def __init__(self) -> None:
        super().__init__('robot_cmd_vel_controller')

        default_scene_config = str(
            Path(get_package_share_directory('hmi_scene')) / 'elevator_yield_scene.yaml'
        )
        scene_config_path = Path(
            self.declare_parameter(
                'scene_config_path',
                default_scene_config,
                ParameterDescriptor(description='Path to the YAML scene config used for robot metadata and initial pose.'),
            ).value
        )
        self._world_name = str(
            self.declare_parameter(
                'world_name',
                'elevator_yield',
                ParameterDescriptor(description='Gazebo world name used for set_pose requests and stats.'),
            ).value
        )
        cmd_vel_topic = self.declare_parameter(
            'cmd_vel_topic',
            '/hmi/control/cmd_vel',
            ParameterDescriptor(description='Twist command topic published by the control-side behavior node.'),
        ).value
        robot_state_topic = self.declare_parameter(
            'robot_state_topic',
            '/hmi/scene/robot_state',
            ParameterDescriptor(description='Robot state topic used as pose feedback for the scene-side controller.'),
        ).value
        update_rate_hz = float(
            self.declare_parameter(
                'update_rate_hz',
                30.0,
                ParameterDescriptor(description='Update rate for applying robot pose integration in Hz.'),
            ).value
        )
        cmd_timeout_sec = float(
            self.declare_parameter(
                'cmd_timeout_sec',
                0.5,
                ParameterDescriptor(description='If no cmd_vel arrives within this time, the robot is stopped.'),
            ).value
        )

        (
            self._command_entity_name,
            self._command_offset_x,
            self._command_offset_y,
            self._command_offset_yaw,
            self._fallback_x,
            self._fallback_y,
            self._fallback_z,
            self._fallback_yaw,
        ) = self._load_robot_config(scene_config_path)
        self._stats_topic = f'/world/{self._world_name}/stats'
        self._set_pose_service = f'/world/{self._world_name}/set_pose'

        self._stats_lock = threading.Lock()
        self._pose_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._stats_reader_process: subprocess.Popen[str] | None = None
        self._stats_reader_thread = threading.Thread(target=self._run_stats_reader, daemon=True)
        self._stats_reader_thread.start()
        self._last_sent_command_pose: tuple[float, float, float, float] | None = None
        self._last_cmd_debug_log_sec = 0.0
        self._last_cmd_debug_signature: tuple[float, float] | None = None
        self._last_pose_sync_log_sec = 0.0

        self._latest_sim_time_sec: float | None = None
        self._current_x = self._fallback_x
        self._current_y = self._fallback_y
        self._current_yaw = self._fallback_yaw
        self._robot_pose_received = False
        self._cmd_linear_x = 0.0
        self._cmd_angular_z = 0.0
        self._last_cmd_wall_time_sec = 0.0
        self._last_applied_sim_time_sec: float | None = None
        self._cmd_timeout_sec = max(cmd_timeout_sec, 0.0)

        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 20)
        self.create_subscription(ActorState, robot_state_topic, self._on_robot_state, 20)
        self.create_timer(1.0 / update_rate_hz, self._update_motion)

        self.get_logger().info(
            'Robot cmd_vel controller is ready. '
            f'Entity: {self._command_entity_name}, cmd topic: {cmd_vel_topic}, state topic: {robot_state_topic}.'
        )

    def _load_robot_config(self, scene_config_path: Path) -> tuple[str, float, float, float, float, float, float, float]:
        if not scene_config_path.is_file():
            raise FileNotFoundError(f'Scene config not found: {scene_config_path}')

        with scene_config_path.open('r', encoding='utf-8') as file_handle:
            data = yaml.safe_load(file_handle)

        models = data.get('models', {})
        if not isinstance(models, dict):
            raise ValueError('models must be a mapping in the scene config.')

        robot = models.get('robot', {})
        if not isinstance(robot, dict):
            raise ValueError('models.robot must be a mapping in the scene config.')

        pose = robot.get('pose', [])
        if not isinstance(pose, list) or len(pose) < 6:
            raise ValueError('models.robot.pose must contain [x, y, z, roll, pitch, yaw].')

        state_entity_name = str(robot.get('entity_name') or robot.get('name') or 'turtlebot3_burger_ir')
        command_entity_name = str(
            robot.get('command_entity_name') or
            (f'{state_entity_name}_1' if state_entity_name == 'turtlebot3_burger_ir' else state_entity_name)
        )
        command_offset = robot.get('command_offset', [0.0, 0.15, 1.570796])
        if not isinstance(command_offset, list) or len(command_offset) < 3:
            raise ValueError('models.robot.command_offset must be [x, y, yaw].')
        return (
            command_entity_name,
            float(command_offset[0]),
            float(command_offset[1]),
            float(command_offset[2]),
            float(pose[0]),
            float(pose[1]),
            float(pose[2]),
            float(pose[5]),
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_linear_x = float(msg.linear.x)
        self._cmd_angular_z = float(msg.angular.z)
        self._last_cmd_wall_time_sec = self.get_clock().now().nanoseconds * 1e-9
        self._maybe_log_cmd_debug()

    def _maybe_log_cmd_debug(self) -> None:
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        signature = (round(self._cmd_linear_x, 3), round(self._cmd_angular_z, 3))
        if signature == self._last_cmd_debug_signature and now_sec - self._last_cmd_debug_log_sec < 1.0:
            return
        self.get_logger().info(
            'scene_cmd_vel '
            f'received linear_x={self._cmd_linear_x:.3f} angular_z={self._cmd_angular_z:.3f}'
        )
        self._last_cmd_debug_log_sec = now_sec
        self._last_cmd_debug_signature = signature

    def _on_robot_state(self, msg: ActorState) -> None:
        if self._should_ignore_robot_state_feedback():
            return
        with self._pose_lock:
            self._current_x = float(msg.x)
            self._current_y = float(msg.y)
            self._current_yaw = float(msg.yaw)
            self._robot_pose_received = True
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self._last_pose_sync_log_sec >= 1.0:
            self.get_logger().info(
                'scene_pose_sync '
                f'robot_state x={self._current_x:.3f} y={self._current_y:.3f} yaw={self._current_yaw:.3f}'
            )
            self._last_pose_sync_log_sec = now_sec

    def _should_ignore_robot_state_feedback(self) -> bool:
        now_wall_sec = self.get_clock().now().nanoseconds * 1e-9
        if self._last_cmd_wall_time_sec <= 0.0:
            return False
        if now_wall_sec - self._last_cmd_wall_time_sec > self._cmd_timeout_sec:
            return False
        return abs(self._cmd_linear_x) > 1e-4 or abs(self._cmd_angular_z) > 1e-4

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

        if self._last_applied_sim_time_sec is None:
            self._last_applied_sim_time_sec = sim_time_sec
            return

        dt = sim_time_sec - self._last_applied_sim_time_sec
        self._last_applied_sim_time_sec = sim_time_sec
        if dt <= 0.0:
            return

        now_wall_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_wall_sec - self._last_cmd_wall_time_sec > self._cmd_timeout_sec:
            linear_x = 0.0
            angular_z = 0.0
        else:
            linear_x = self._cmd_linear_x
            angular_z = self._cmd_angular_z

        with self._pose_lock:
            if not self._robot_pose_received:
                current_x = self._current_x
                current_y = self._current_y
                current_yaw = self._current_yaw
            else:
                current_x = self._current_x
                current_y = self._current_y
                current_yaw = self._current_yaw

        new_yaw = self._normalize_angle(current_yaw + angular_z * dt)
        new_x = current_x + linear_x * math.cos(new_yaw) * dt
        new_y = current_y + linear_x * math.sin(new_yaw) * dt

        with self._pose_lock:
            self._current_x = new_x
            self._current_y = new_y
            self._current_yaw = new_yaw

        self._set_entity_pose(new_x, new_y, self._fallback_z, new_yaw)

    def _set_entity_pose(self, x: float, y: float, z: float, yaw: float) -> None:
        command_yaw = self._normalize_angle(yaw - self._command_offset_yaw)
        offset_world_x = (
            math.cos(command_yaw) * self._command_offset_x -
            math.sin(command_yaw) * self._command_offset_y
        )
        offset_world_y = (
            math.sin(command_yaw) * self._command_offset_x +
            math.cos(command_yaw) * self._command_offset_y
        )
        command_x = x - offset_world_x
        command_y = y - offset_world_y

        command_pose = (command_x, command_y, z, command_yaw)
        if self._last_sent_command_pose is not None:
            dx = command_pose[0] - self._last_sent_command_pose[0]
            dy = command_pose[1] - self._last_sent_command_pose[1]
            dyaw = self._normalize_angle(command_pose[3] - self._last_sent_command_pose[3])
            if math.hypot(dx, dy) < 1e-4 and abs(dyaw) < 1e-4:
                return

        quaternion = self._quaternion_from_rpy(0.0, 0.0, command_yaw)
        request_text = (
            f'name: "{self._command_entity_name}" '
            f'position {{ x: {command_x:.9f} y: {command_y:.9f} z: {z:.9f} }} '
            f'orientation {{ x: {quaternion[0]:.9f} y: {quaternion[1]:.9f} '
            f'z: {quaternion[2]:.9f} w: {quaternion[3]:.9f} }}'
        )
        command = [
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
            request_text,
        ]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                check=False,
                timeout=1.2,
            )
        except FileNotFoundError:
            self.get_logger().error('Could not find the `gz` executable needed to move the robot.')
            return
        except subprocess.TimeoutExpired:
            self.get_logger().warning('Timed out while sending a Gazebo set_pose request for the robot.')
            return

        if result.returncode != 0 or 'data: true' not in result.stdout:
            stderr_text = result.stderr.strip()
            stdout_text = result.stdout.strip()
            self.get_logger().warning(
                'Robot set_pose request failed: '
                f'code={result.returncode}, stdout="{stdout_text}", stderr="{stderr_text}"'
            )
            return

        self._last_sent_command_pose = command_pose

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
    node = RobotCmdVelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
