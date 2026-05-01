from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from hmi_behavior.conflict_detector import ConflictAssessment, assess_conflict
from hmi_behavior.elevator_policy import plan_yield_goal as plan_elevator_yield_goal
from hmi_behavior.open_area_policy import plan_yield_goal as plan_open_area_yield_goal
from hmi_behavior.robot_state import (
    AggregatedState,
    GoalSnapshot,
    MotionTarget,
    normalize_angle,
    segment_is_free,
    target_distance,
    transform_world_to_local,
)
from hmi_behavior.scene_classifier import classify_scene


class InternalState(str, Enum):
    IDLE = 'Idle'
    NAVIGATE = 'Navigate'
    FORWARD = 'Forward'
    CONFLICT_AVOIDING_NAVIGATE = 'ConflictAvoidingNavigate'
    CONFLICT_AVOID = 'ConflictAvoid'
    WAIT = 'Wait'


@dataclass
class ControllerConfig:
    goal_tolerance: float
    max_forward_speed: float
    max_reverse_speed: float
    max_angular_speed: float
    angular_gain: float
    linear_gain: float
    wait_duration: float
    safety_radius: float
    human_zone_time: float
    human_exit_time: float
    human_zone_half_width: float
    reverse_obstacle_distance: float
    reverse_obstacle_half_width: float
    reverse_distance_elevator: float
    reverse_distance_open_area: float
    lateral_offset_elevator: float
    lateral_offset_open_area: float
    side_probe_distance: float
    narrow_width_threshold: float
    heading_slow_threshold: float
    heading_stop_threshold: float
    resume_duration: float


@dataclass
class StateMachineOutput:
    internal_state: InternalState
    behavior_state: str
    reason: str
    target_linear_x: float
    target_angular_z: float
    scene_label: str
    active_target: Optional[MotionTarget]


class BehaviorStateMachine:
    def __init__(self, config: ControllerConfig) -> None:
        self._config = config
        self._state = InternalState.IDLE
        self._yield_target: Optional[MotionTarget] = None
        self._wait_started_at: Optional[float] = None
        self._resume_until: float = 0.0

    def step(self, context: AggregatedState, now_sec: float) -> StateMachineOutput:
        if self._resume_until > 0.0 and now_sec >= self._resume_until:
            self._resume_until = 0.0

        scene_label = classify_scene(
            context,
            narrow_width_threshold=self._config.narrow_width_threshold,
            side_probe_distance=self._config.side_probe_distance,
        )
        conflict = assess_conflict(
            context,
            safety_radius=self._config.safety_radius,
            human_zone_time=self._config.human_zone_time,
            human_exit_time=self._config.human_exit_time,
            human_zone_half_width=self._config.human_zone_half_width,
            reverse_obstacle_distance=self._config.reverse_obstacle_distance,
            reverse_obstacle_half_width=self._config.reverse_obstacle_half_width,
        )

        if context.robot is None or context.goal is None:
            self._state = InternalState.IDLE
            self._yield_target = None
            return self._output(scene_label, '', 0.0, 0.0, None, now_sec)

        if target_distance(context.robot, context.goal) <= self._config.goal_tolerance:
            self._state = InternalState.IDLE
            self._yield_target = None
            self._resume_until = 0.0
            return self._output(scene_label, '', 0.0, 0.0, None, now_sec)

        if self._state == InternalState.WAIT and self._wait_started_at is not None:
            if now_sec - self._wait_started_at >= self._config.wait_duration and not conflict.hard_stop:
                self._state = InternalState.IDLE
                self._wait_started_at = None

        if conflict.hard_stop:
            self._state = InternalState.WAIT
            self._wait_started_at = now_sec
            self._resume_until = 0.0
            return self._output(scene_label, conflict.reason, 0.0, 0.0, self._yield_target, now_sec)

        if self._state in (InternalState.IDLE, InternalState.NAVIGATE, InternalState.FORWARD):
            self._state = InternalState.NAVIGATE
            if conflict.conflict:
                self._state = InternalState.CONFLICT_AVOIDING_NAVIGATE

        if self._state == InternalState.NAVIGATE:
            if self._goal_path_is_available(context, context.goal):
                self._state = InternalState.FORWARD
            else:
                self._state = InternalState.WAIT
                self._wait_started_at = now_sec
                return self._output(scene_label, 'obstacle_back', 0.0, 0.0, None, now_sec)

        if self._state == InternalState.CONFLICT_AVOIDING_NAVIGATE:
            self._yield_target = self._plan_yield_target(context, scene_label)
            if self._yield_target is None:
                self._state = InternalState.WAIT
                self._wait_started_at = now_sec
                return self._output(scene_label, conflict.reason or 'human_close', 0.0, 0.0, None, now_sec)
            self._state = InternalState.CONFLICT_AVOID

        if self._state == InternalState.CONFLICT_AVOID and self._yield_target is not None:
            if target_distance(context.robot, self._yield_target) <= self._config.goal_tolerance:
                if conflict.exit_zone:
                    self._state = InternalState.CONFLICT_AVOIDING_NAVIGATE
                    return self.step(context, now_sec)
                self._yield_target = None
                self._state = InternalState.NAVIGATE
                self._resume_until = now_sec + self._config.resume_duration
                return self.step(context, now_sec)

            if conflict.obstacle_behind:
                self._state = InternalState.WAIT
                self._wait_started_at = now_sec
                return self._output(scene_label, 'obstacle_back', 0.0, 0.0, self._yield_target, now_sec)

            target_linear_x, target_angular_z = self._command_to_target(context, self._yield_target)
            return self._output(
                scene_label,
                conflict.reason or 'human_close',
                target_linear_x,
                target_angular_z,
                self._yield_target,
                now_sec,
            )

        if self._state == InternalState.FORWARD:
            if conflict.conflict:
                self._state = InternalState.CONFLICT_AVOIDING_NAVIGATE
                return self.step(context, now_sec)

            target_linear_x, target_angular_z = self._command_to_target(
                context,
                MotionTarget(
                    x=context.goal.x,
                    y=context.goal.y,
                    reverse_ok=False,
                    source='goal',
                ),
            )
            return self._output(scene_label, conflict.reason, target_linear_x, target_angular_z, None, now_sec)

        return self._output(scene_label, conflict.reason, 0.0, 0.0, self._yield_target, now_sec)

    def _goal_path_is_available(self, context: AggregatedState, goal: GoalSnapshot) -> bool:
        if context.robot is None:
            return False
        return segment_is_free(context.map_state, context.robot.x, context.robot.y, goal.x, goal.y)

    def _plan_yield_target(self, context: AggregatedState, scene_label: str) -> Optional[MotionTarget]:
        if scene_label == 'elevator':
            return plan_elevator_yield_goal(
                context,
                reverse_distance=self._config.reverse_distance_elevator,
                lateral_offset=self._config.lateral_offset_elevator,
                side_probe_distance=self._config.side_probe_distance,
            )
        return plan_open_area_yield_goal(
            context,
            reverse_distance=self._config.reverse_distance_open_area,
            lateral_offset=self._config.lateral_offset_open_area,
            side_probe_distance=self._config.side_probe_distance,
        )

    def _command_to_target(self, context: AggregatedState, target: MotionTarget) -> tuple[float, float]:
        if context.robot is None:
            return 0.0, 0.0

        local_x, local_y = transform_world_to_local(context.robot, target.x, target.y)
        distance = math.hypot(local_x, local_y)

        if target.reverse_ok and local_x < -0.05:
            heading_error = math.atan2(local_y, abs(local_x))
            angular_z = max(
                -self._config.max_angular_speed,
                min(self._config.max_angular_speed, self._config.angular_gain * heading_error),
            )
            linear_x = -min(self._config.max_reverse_speed, self._config.linear_gain * distance)
            if abs(heading_error) > self._config.heading_slow_threshold:
                linear_x *= 0.5
            return linear_x, angular_z

        heading_error = normalize_angle(math.atan2(local_y, local_x))
        angular_z = max(
            -self._config.max_angular_speed,
            min(self._config.max_angular_speed, self._config.angular_gain * heading_error),
        )
        linear_x = min(self._config.max_forward_speed, self._config.linear_gain * distance)
        if abs(heading_error) > self._config.heading_stop_threshold:
            linear_x = 0.0
        elif abs(heading_error) > self._config.heading_slow_threshold:
            linear_x *= 0.4
        return linear_x, angular_z

    def _output(
        self,
        scene_label: str,
        reason: str,
        target_linear_x: float,
        target_angular_z: float,
        active_target: Optional[MotionTarget],
        now_sec: float,
    ) -> StateMachineOutput:
        behavior_state = self._behavior_state_label(now_sec)
        if behavior_state == 'Resume':
            reason = 'human_passed'

        return StateMachineOutput(
            internal_state=self._state,
            behavior_state=behavior_state,
            reason=reason,
            target_linear_x=target_linear_x,
            target_angular_z=target_angular_z,
            scene_label=scene_label,
            active_target=active_target,
        )

    def _behavior_state_label(self, now_sec: float) -> str:
        if self._resume_until > now_sec:
            return 'Resume'
        if self._state == InternalState.CONFLICT_AVOIDING_NAVIGATE:
            return 'HumanDetected'
        if self._state == InternalState.CONFLICT_AVOID:
            return 'YieldBackward'
        if self._state in (InternalState.IDLE, InternalState.WAIT):
            return 'Waiting'
        return 'NormalMove'
