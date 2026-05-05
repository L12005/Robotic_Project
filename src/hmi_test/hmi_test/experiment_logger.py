from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from hmi_interfaces.msg import ActorState, BehaviorState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


@dataclass
class PoseSample:
    stamp_sec: float
    x: float
    y: float
    yaw: float


@dataclass
class GoalSample:
    stamp_sec: float
    x: float
    y: float
    yaw: float


class ExperimentLogger(Node):
    def __init__(self) -> None:
        super().__init__('experiment_logger')

        robot_state_topic = self.declare_parameter(
            'robot_state_topic',
            '/hmi/scene/robot_state',
            ParameterDescriptor(description='Robot state topic to log as CSV.'),
        ).value
        human_state_topic = self.declare_parameter(
            'human_state_topic',
            '/hmi/scene/human_state',
            ParameterDescriptor(description='Human state topic to log as CSV.'),
        ).value
        behavior_state_topic = self.declare_parameter(
            'behavior_state_topic',
            '/hmi/control/behavior_state',
            ParameterDescriptor(description='Behavior state topic to log as CSV and summarize.'),
        ).value
        goal_topic = self.declare_parameter(
            'goal_topic',
            '/hmi/control/goal_pose',
            ParameterDescriptor(description='Goal pose topic used for success and arrival-time metrics.'),
        ).value
        goal_tolerance = float(
            self.declare_parameter(
                'goal_tolerance',
                0.20,
                ParameterDescriptor(description='Distance threshold for considering the goal reached.'),
            ).value
        )
        output_dir_param = str(
            self.declare_parameter(
                'output_dir',
                '',
                ParameterDescriptor(
                    description='Optional output directory for CSV logs. Defaults to <workspace>/experiment_log.'
                ),
            ).value
        )

        self._goal_tolerance = max(goal_tolerance, 0.0)
        self._run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self._output_dir = self._resolve_output_dir(output_dir_param)
        self._output_dir.mkdir(parents=True, exist_ok=True)

        self._robot_file = self._open_writer(
            self._output_dir / f'{self._run_id}_robot_state.csv',
            [
                'stamp_sec',
                'frame_id',
                'actor_id',
                'actor_type',
                'x',
                'y',
                'yaw',
                'linear_x',
                'angular_z',
                'is_moving',
            ],
        )
        self._human_file = self._open_writer(
            self._output_dir / f'{self._run_id}_human_state.csv',
            [
                'stamp_sec',
                'frame_id',
                'actor_id',
                'actor_type',
                'x',
                'y',
                'yaw',
                'linear_x',
                'angular_z',
                'is_moving',
            ],
        )
        self._behavior_file = self._open_writer(
            self._output_dir / f'{self._run_id}_behavior_state.csv',
            [
                'stamp_sec',
                'current_state',
                'internal_state',
                'reason',
                'target_linear_x',
                'target_angular_z',
            ],
        )
        self._summary_file = self._open_writer(
            self._output_dir / f'{self._run_id}_summary.csv',
            [
                'run_id',
                'start_time_sec',
                'end_time_sec',
                'duration_sec',
                'robot_samples',
                'human_samples',
                'behavior_samples',
                'goal_samples',
                'path_length_m',
                'min_human_distance_m',
                'waiting_duration_sec',
                'replanning_count',
                'yield_event_count',
                'goal_reached',
                'arrival_time_sec',
                'final_goal_distance_m',
                'goal_tolerance_m',
            ],
        )

        self._start_time_sec: Optional[float] = None
        self._end_time_sec: Optional[float] = None
        self._robot_count = 0
        self._human_count = 0
        self._behavior_count = 0
        self._goal_count = 0
        self._path_length_m = 0.0
        self._min_human_distance_m = math.inf
        self._waiting_duration_sec = 0.0
        self._replanning_count = 0
        self._yield_event_count = 0
        self._goal_reached = False
        self._arrival_time_sec: Optional[float] = None

        self._last_robot_sample: Optional[PoseSample] = None
        self._last_human_sample: Optional[PoseSample] = None
        self._last_goal: Optional[GoalSample] = None
        self._last_behavior_stamp_sec: Optional[float] = None
        self._last_behavior_internal_state = ''
        self._shutdown_done = False

        self.create_subscription(ActorState, robot_state_topic, self._on_robot_state, 20)
        self.create_subscription(ActorState, human_state_topic, self._on_human_state, 20)
        self.create_subscription(BehaviorState, behavior_state_topic, self._on_behavior_state, 20)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal_pose, 10)

        self.get_logger().info(
            'Experiment logger is ready. '
            f'Run id: {self._run_id}, output directory: {self._output_dir}'
        )

    def _resolve_output_dir(self, output_dir_param: str) -> Path:
        if output_dir_param.strip():
            return Path(output_dir_param).expanduser().resolve()

        workspace_root = Path(get_package_prefix('hmi_test')).parents[1]
        return workspace_root / 'experiment_log'

    def _open_writer(self, path: Path, fieldnames: list[str]) -> tuple[Path, object, csv.DictWriter]:
        file_handle = path.open('w', newline='', encoding='utf-8')
        writer = csv.DictWriter(file_handle, fieldnames=fieldnames)
        writer.writeheader()
        file_handle.flush()
        return path, file_handle, writer

    def _stamp_to_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _pose_stamped_yaw(self, msg: PoseStamped) -> float:
        q = msg.pose.orientation
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _note_start_time(self, stamp_sec: float) -> None:
        if self._start_time_sec is None:
            self._start_time_sec = stamp_sec

    def _flush_writer(self, file_tuple: tuple[Path, object, csv.DictWriter]) -> None:
        _, file_handle, _ = file_tuple
        file_handle.flush()

    def _writer(self, file_tuple: tuple[Path, object, csv.DictWriter]) -> csv.DictWriter:
        return file_tuple[2]

    def _on_robot_state(self, msg: ActorState) -> None:
        stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self._note_start_time(stamp_sec)
        self._end_time_sec = stamp_sec
        self._robot_count += 1

        writer = self._writer(self._robot_file)
        writer.writerow(
            {
                'stamp_sec': f'{stamp_sec:.9f}',
                'frame_id': msg.header.frame_id,
                'actor_id': msg.actor_id,
                'actor_type': msg.actor_type,
                'x': f'{msg.x:.6f}',
                'y': f'{msg.y:.6f}',
                'yaw': f'{msg.yaw:.6f}',
                'linear_x': f'{msg.linear_x:.6f}',
                'angular_z': f'{msg.angular_z:.6f}',
                'is_moving': int(bool(msg.is_moving)),
            }
        )
        self._flush_writer(self._robot_file)

        sample = PoseSample(stamp_sec=stamp_sec, x=float(msg.x), y=float(msg.y), yaw=float(msg.yaw))
        if self._last_robot_sample is not None:
            self._path_length_m += math.hypot(sample.x - self._last_robot_sample.x, sample.y - self._last_robot_sample.y)
        self._last_robot_sample = sample
        self._update_min_human_distance()
        self._update_goal_progress(sample)

    def _on_human_state(self, msg: ActorState) -> None:
        stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self._note_start_time(stamp_sec)
        self._end_time_sec = stamp_sec
        self._human_count += 1

        writer = self._writer(self._human_file)
        writer.writerow(
            {
                'stamp_sec': f'{stamp_sec:.9f}',
                'frame_id': msg.header.frame_id,
                'actor_id': msg.actor_id,
                'actor_type': msg.actor_type,
                'x': f'{msg.x:.6f}',
                'y': f'{msg.y:.6f}',
                'yaw': f'{msg.yaw:.6f}',
                'linear_x': f'{msg.linear_x:.6f}',
                'angular_z': f'{msg.angular_z:.6f}',
                'is_moving': int(bool(msg.is_moving)),
            }
        )
        self._flush_writer(self._human_file)

        self._last_human_sample = PoseSample(
            stamp_sec=stamp_sec,
            x=float(msg.x),
            y=float(msg.y),
            yaw=float(msg.yaw),
        )
        self._update_min_human_distance()

    def _on_behavior_state(self, msg: BehaviorState) -> None:
        stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self._note_start_time(stamp_sec)
        self._end_time_sec = stamp_sec
        self._behavior_count += 1

        writer = self._writer(self._behavior_file)
        writer.writerow(
            {
                'stamp_sec': f'{stamp_sec:.9f}',
                'current_state': msg.current_state,
                'internal_state': msg.internal_state,
                'reason': msg.reason,
                'target_linear_x': f'{msg.target_linear_x:.6f}',
                'target_angular_z': f'{msg.target_angular_z:.6f}',
            }
        )
        self._flush_writer(self._behavior_file)

        if self._last_behavior_stamp_sec is not None and self._last_behavior_internal_state == 'Wait':
            self._waiting_duration_sec += max(0.0, stamp_sec - self._last_behavior_stamp_sec)

        if msg.internal_state in ('Navigate', 'ConflictAvoidingNavigate') and msg.internal_state != self._last_behavior_internal_state:
            self._replanning_count += 1
        if msg.internal_state == 'ConflictAvoid' and msg.internal_state != self._last_behavior_internal_state:
            self._yield_event_count += 1

        self._last_behavior_stamp_sec = stamp_sec
        self._last_behavior_internal_state = msg.internal_state

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        stamp_sec = self._stamp_to_sec(msg.header.stamp)
        self._note_start_time(stamp_sec)
        self._end_time_sec = stamp_sec
        self._goal_count += 1
        self._last_goal = GoalSample(
            stamp_sec=stamp_sec,
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            yaw=self._pose_stamped_yaw(msg),
        )
        if self._last_robot_sample is not None:
            self._update_goal_progress(self._last_robot_sample)

    def _update_min_human_distance(self) -> None:
        if self._last_robot_sample is None or self._last_human_sample is None:
            return
        distance = math.hypot(
            self._last_robot_sample.x - self._last_human_sample.x,
            self._last_robot_sample.y - self._last_human_sample.y,
        )
        self._min_human_distance_m = min(self._min_human_distance_m, distance)

    def _update_goal_progress(self, robot_sample: PoseSample) -> None:
        if self._last_goal is None:
            return
        distance = math.hypot(robot_sample.x - self._last_goal.x, robot_sample.y - self._last_goal.y)
        if distance <= self._goal_tolerance and not self._goal_reached:
            self._goal_reached = True
            if self._start_time_sec is not None:
                self._arrival_time_sec = max(0.0, robot_sample.stamp_sec - self._start_time_sec)

    def _final_goal_distance(self) -> float:
        if self._last_robot_sample is None or self._last_goal is None:
            return math.inf
        return math.hypot(
            self._last_robot_sample.x - self._last_goal.x,
            self._last_robot_sample.y - self._last_goal.y,
        )

    def _write_summary(self) -> None:
        if self._start_time_sec is None:
            self.get_logger().warning('No messages were received; experiment summary will not be written.')
            return

        end_time_sec = self._end_time_sec if self._end_time_sec is not None else self._start_time_sec
        duration_sec = max(0.0, end_time_sec - self._start_time_sec)
        final_goal_distance = self._final_goal_distance()
        waiting_duration_sec = self._waiting_duration_sec
        if self._last_behavior_stamp_sec is not None and self._last_behavior_internal_state == 'Wait':
            waiting_duration_sec += max(0.0, end_time_sec - self._last_behavior_stamp_sec)

        writer = self._writer(self._summary_file)
        writer.writerow(
            {
                'run_id': self._run_id,
                'start_time_sec': f'{self._start_time_sec:.9f}',
                'end_time_sec': f'{end_time_sec:.9f}',
                'duration_sec': f'{duration_sec:.6f}',
                'robot_samples': self._robot_count,
                'human_samples': self._human_count,
                'behavior_samples': self._behavior_count,
                'goal_samples': self._goal_count,
                'path_length_m': f'{self._path_length_m:.6f}',
                'min_human_distance_m': '' if math.isinf(self._min_human_distance_m) else f'{self._min_human_distance_m:.6f}',
                'waiting_duration_sec': f'{waiting_duration_sec:.6f}',
                'replanning_count': self._replanning_count,
                'yield_event_count': self._yield_event_count,
                'goal_reached': int(self._goal_reached),
                'arrival_time_sec': '' if self._arrival_time_sec is None else f'{self._arrival_time_sec:.6f}',
                'final_goal_distance_m': '' if math.isinf(final_goal_distance) else f'{final_goal_distance:.6f}',
                'goal_tolerance_m': f'{self._goal_tolerance:.6f}',
            }
        )
        self._flush_writer(self._summary_file)

        self.get_logger().info(
            'Experiment summary written. '
            f'goal_reached={self._goal_reached} path_length={self._path_length_m:.3f}m '
            f'waiting_duration={waiting_duration_sec:.3f}s output_dir={self._output_dir}'
        )

    def _close_file(self, file_tuple: tuple[Path, object, csv.DictWriter]) -> None:
        _, file_handle, _ = file_tuple
        file_handle.close()

    def destroy_node(self) -> bool:
        if self._shutdown_done:
            return super().destroy_node()
        self._shutdown_done = True
        self._write_summary()
        self._close_file(self._robot_file)
        self._close_file(self._human_file)
        self._close_file(self._behavior_file)
        self._close_file(self._summary_file)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
