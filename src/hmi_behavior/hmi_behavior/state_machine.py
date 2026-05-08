from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional

from nav_msgs.msg import OccupancyGrid

from hmi_behavior.conflict_detector import assess_conflict
from hmi_behavior.elevator_policy import generate_yield_candidates as generate_elevator_candidates
from hmi_behavior.grid_planner import (
    GridPlan,
    clone_grid_data,
    make_grid_spec,
    paint_disc,
    paint_oriented_box,
    plan_a_star,
)
from hmi_behavior.open_area_policy import plan_yield_goal as plan_open_area_yield_goal
from hmi_behavior.robot_state import (
    AggregatedState,
    MotionTarget,
    distance_xy,
    normalize_angle,
    select_base_map,
    target_distance,
    transform_world_to_local,
)
from hmi_behavior.scene_classifier import classify_scene


class InternalState(str, Enum):
    IDLE = "Idle"
    NAVIGATE = "Navigate"
    FORWARD = "Forward"
    CONFLICT_AVOIDING_NAVIGATE = "ConflictAvoidingNavigate"
    CONFLICT_AVOID = "ConflictAvoid"
    WAIT = "Wait"
    HARD_STOP = "HardStop"


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
    path_lookahead_distance: float
    dynamic_obstacle_inflation: float
    human_body_radius: float
    human_forward_min_depth: float
    yield_sample_count: int
    planner_occupancy_threshold: int


@dataclass
class StateMachineOutput:
    internal_state: InternalState
    behavior_state: str
    reason: str
    target_linear_x: float
    target_angular_z: float
    scene_label: str
    active_target: Optional[MotionTarget]
    motion_direction: str
    is_resuming: bool
    avoidance_started_event: bool


@dataclass
class YieldPlan:
    target: MotionTarget
    path: GridPlan


# Threshold for angular velocity to distinguish turning from straight motion
_TURN_ANGULAR_THRESHOLD = 0.15


def _determine_motion_direction(target_linear_x: float, target_angular_z: float) -> str:
    """Determine motion direction from velocity commands.

    Returns one of: forward, backward, left_turn, right_turn, none.
    """
    if abs(target_linear_x) < 0.01 and abs(target_angular_z) < 0.01:
        return "none"
    if target_linear_x < -0.01:
        return "backward"
    if target_angular_z > _TURN_ANGULAR_THRESHOLD:
        return "left_turn"
    if target_angular_z < -_TURN_ANGULAR_THRESHOLD:
        return "right_turn"
    if target_linear_x > 0.01:
        return "forward"
    return "none"


class BehaviorStateMachine:
    def __init__(self, config: ControllerConfig) -> None:
        self._config = config
        self._state = InternalState.IDLE
        self._active_target: Optional[MotionTarget] = None
        self._wait_started_at: Optional[float] = None
        self._hard_stop_started_at: Optional[float] = None
        self._resume_until = 0.0
        # Avoidance session de-duplication: tracks whether we have already
        # fired the avoidance-started event for the current pedestrian encounter.
        self._avoidance_session_active: bool = False

    def step(self, context: AggregatedState, now_sec: float) -> StateMachineOutput:
        if self._resume_until > 0.0 and now_sec >= self._resume_until:
            self._resume_until = 0.0

        base_map = select_base_map(context)
        scene_label = classify_scene(
            context,
            narrow_width_threshold=self._config.narrow_width_threshold,
            side_probe_distance=self._config.side_probe_distance,
        )

        if context.robot is None or context.goal is None or base_map is None:
            self._state = InternalState.IDLE
            self._active_target = None
            return self._output(scene_label, "", 0.0, 0.0, None, now_sec)

        conflict = assess_conflict(
            context,
            safety_radius=self._config.safety_radius,
            human_zone_time=self._config.human_zone_time,
            human_exit_time=self._config.human_exit_time,
            human_zone_half_width=self._config.human_zone_half_width,
            reverse_obstacle_distance=self._config.reverse_obstacle_distance,
            reverse_obstacle_half_width=self._config.reverse_obstacle_half_width,
        )

        # --- HardStop exit: 1s elapsed AND circle cleared ---
        if self._state == InternalState.HARD_STOP and self._hard_stop_started_at is not None:
            if now_sec - self._hard_stop_started_at >= self._config.wait_duration and not conflict.hard_stop:
                self._state = InternalState.IDLE
                self._hard_stop_started_at = None

        # --- HardStop: highest priority once scene inputs are available ---
        if conflict.hard_stop:
            if self._state != InternalState.HARD_STOP:
                self._state = InternalState.HARD_STOP
                self._hard_stop_started_at = now_sec
            self._active_target = None
            return self._output(scene_label, conflict.reason or "human_close", 0.0, 0.0, None, now_sec)

        # --- Wait exit: 1s elapsed (no hard_stop check, it is a separate state now) ---
        if self._state == InternalState.WAIT and self._wait_started_at is not None:
            if now_sec - self._wait_started_at >= self._config.wait_duration:
                self._state = InternalState.IDLE
                self._wait_started_at = None

        # --- Goal reached ---
        if target_distance(context.robot, context.goal) <= self._config.goal_tolerance:
            self._state = InternalState.IDLE
            self._active_target = None
            self._resume_until = 0.0
            self._avoidance_session_active = False
            return self._output(scene_label, "", 0.0, 0.0, None, now_sec)

        # --- ConflictAvoid arrival: re-evaluate ---
        if self._state == InternalState.CONFLICT_AVOID and self._active_target is not None:
            if target_distance(context.robot, self._active_target) <= self._config.goal_tolerance:
                if conflict.exit_zone:
                    self._state = InternalState.CONFLICT_AVOIDING_NAVIGATE
                else:
                    self._state = InternalState.NAVIGATE
                    self._resume_until = now_sec + self._config.resume_duration
                    # Pedestrian left exit zone -> reset avoidance session
                    self._avoidance_session_active = False
                self._active_target = None

        planning_map, planning_data = self._build_planning_map(context, base_map)

        # --- A* to goal ---
        goal_plan = plan_a_star(
            planning_map,
            planning_data,
            start_xy=(context.robot.x, context.robot.y),
            goal_xy=(context.goal.x, context.goal.y),
            threshold=self._config.planner_occupancy_threshold,
        )

        # --- Path unreachable + human conflict -> yield or Wait ---
        # Issue 9 fix: only use conflict.conflict, removed _human_is_relevant() heuristic
        if goal_plan is None and conflict.conflict:
            yield_plan = self._plan_yield_path(context, planning_map, planning_data, scene_label)
            if yield_plan is not None:
                avoidance_event = self._enter_conflict_avoid(now_sec)
                self._active_target = yield_plan.target
                target_linear_x, target_angular_z = self._command_for_path(
                    context,
                    yield_plan.path,
                    reverse_ok=True,
                )
                return self._output(
                    scene_label,
                    conflict.reason or "human_close",
                    target_linear_x,
                    target_angular_z,
                    self._active_target,
                    now_sec,
                    avoidance_started_override=avoidance_event,
                )

            # All candidates unreachable -> Wait
            self._state = InternalState.WAIT
            self._wait_started_at = now_sec
            return self._output(scene_label, conflict.reason or "human_close", 0.0, 0.0, None, now_sec)

        # --- Forward no-go zone conflict OR still in ConflictAvoidingNavigate ---
        if conflict.conflict or self._state == InternalState.CONFLICT_AVOIDING_NAVIGATE:
            yield_plan = self._plan_yield_path(context, planning_map, planning_data, scene_label)
            if yield_plan is not None:
                avoidance_event = self._enter_conflict_avoid(now_sec)
                self._active_target = yield_plan.target
                target_linear_x, target_angular_z = self._command_for_path(
                    context,
                    yield_plan.path,
                    reverse_ok=True,
                )
                return self._output(
                    scene_label,
                    conflict.reason or "human_close",
                    target_linear_x,
                    target_angular_z,
                    self._active_target,
                    now_sec,
                    avoidance_started_override=avoidance_event,
                )

            # All candidates unreachable -> Wait
            self._state = InternalState.WAIT
            self._wait_started_at = now_sec
            return self._output(scene_label, conflict.reason or "human_close", 0.0, 0.0, None, now_sec)

        # --- Normal forward navigation ---
        if goal_plan is not None:
            self._state = InternalState.FORWARD
            self._active_target = None
            target_linear_x, target_angular_z = self._command_for_path(
                context,
                goal_plan,
                reverse_ok=False,
            )
            return self._output(scene_label, conflict.reason, target_linear_x, target_angular_z, None, now_sec)

        # --- Path unreachable, no human conflict -> Wait ---
        self._state = InternalState.WAIT
        self._wait_started_at = now_sec
        self._active_target = None
        return self._output(
            scene_label,
            "obstacle_back" if conflict.obstacle_behind else "",
            0.0,
            0.0,
            None,
            now_sec,
        )

    def _enter_conflict_avoid(self, now_sec: float) -> bool:
        """Transition into ConflictAvoid and return whether this is the first
        avoidance event for the current pedestrian encounter."""
        avoidance_event = False
        if not self._avoidance_session_active:
            self._avoidance_session_active = True
            avoidance_event = True
        self._state = InternalState.CONFLICT_AVOID
        return avoidance_event

    def _build_planning_map(
        self,
        context: AggregatedState,
        base_map: OccupancyGrid,
    ) -> tuple[OccupancyGrid, list[int]]:
        data = clone_grid_data(base_map)
        spec = make_grid_spec(base_map)

        for obstacle in context.obstacles.values():
            if obstacle.is_static:
                continue
            paint_oriented_box(
                data,
                spec,
                center_x=obstacle.x,
                center_y=obstacle.y,
                yaw=0.0,
                size_x=obstacle.width,
                size_y=obstacle.length,
                inflate_radius=self._config.dynamic_obstacle_inflation,
            )

        if context.human is not None:
            paint_disc(
                data,
                spec,
                center_x=context.human.x,
                center_y=context.human.y,
                radius=self._config.human_body_radius,
            )

            # Planning blocks the human body plus the forward no-go zone;
            # exit_zone only gates resume.
            forward_zone_depth = abs(context.human.linear_x) * self._config.human_zone_time
            if context.human.is_moving:
                forward_zone_depth = max(forward_zone_depth, self._config.human_forward_min_depth)

            if forward_zone_depth > 0.0:
                center_x = context.human.x + math.cos(context.human.yaw) * forward_zone_depth * 0.5
                center_y = context.human.y + math.sin(context.human.yaw) * forward_zone_depth * 0.5
                paint_oriented_box(
                    data,
                    spec,
                    center_x=center_x,
                    center_y=center_y,
                    yaw=context.human.yaw,
                    size_x=forward_zone_depth,
                    size_y=self._config.human_zone_half_width * 2.0,
                )

        planning_map = OccupancyGrid()
        planning_map.header = base_map.header
        planning_map.info = base_map.info
        planning_map.data = list(data)
        return planning_map, data

    def _plan_yield_path(
        self,
        context: AggregatedState,
        planning_map: OccupancyGrid,
        planning_data: list[int],
        scene_label: str,
    ) -> YieldPlan | None:
        if context.robot is None:
            return None

        candidates: list[MotionTarget]
        if scene_label == "elevator":
            candidates = generate_elevator_candidates(
                context,
                reverse_distance=self._config.reverse_distance_elevator,
                lateral_offset=self._config.lateral_offset_elevator,
                sample_count=self._config.yield_sample_count,
            )
        else:
            candidates = self._generate_open_area_candidates(context)

        for candidate in candidates:
            plan = plan_a_star(
                planning_map,
                planning_data,
                start_xy=(context.robot.x, context.robot.y),
                goal_xy=(candidate.x, candidate.y),
                threshold=self._config.planner_occupancy_threshold,
            )
            if plan is not None:
                return YieldPlan(target=candidate, path=plan)
        return None

    def _generate_open_area_candidates(self, context: AggregatedState) -> list[MotionTarget]:
        seed = plan_open_area_yield_goal(
            context,
            reverse_distance=self._config.reverse_distance_open_area,
            lateral_offset=self._config.lateral_offset_open_area,
            side_probe_distance=self._config.side_probe_distance,
        )
        candidates = generate_elevator_candidates(
            context,
            reverse_distance=self._config.reverse_distance_open_area,
            lateral_offset=self._config.lateral_offset_open_area,
            sample_count=max(self._config.yield_sample_count, 12),
        )
        if seed is not None:
            candidates = [seed] + [
                candidate
                for candidate in candidates
                if distance_xy(candidate.x, candidate.y, seed.x, seed.y) > 0.05
            ]
        return candidates

    def _command_for_path(
        self,
        context: AggregatedState,
        plan: GridPlan,
        reverse_ok: bool,
    ) -> tuple[float, float]:
        if context.robot is None or not plan.world_points:
            return 0.0, 0.0

        lookahead_x, lookahead_y = plan.world_points[-1]
        for point_x, point_y in plan.world_points:
            if distance_xy(context.robot.x, context.robot.y, point_x, point_y) >= self._config.path_lookahead_distance:
                lookahead_x, lookahead_y = point_x, point_y
                break

        return self._command_to_target(
            context,
            MotionTarget(
                x=lookahead_x,
                y=lookahead_y,
                reverse_ok=reverse_ok,
                source="planned_path",
            ),
        )

    def _command_to_target(self, context: AggregatedState, target: MotionTarget) -> tuple[float, float]:
        if context.robot is None:
            return 0.0, 0.0

        local_x, local_y = transform_world_to_local(context.robot, target.x, target.y)
        distance = math.hypot(local_x, local_y)

        if target.reverse_ok and local_x < -0.03:
            heading_error = math.atan2(local_y, max(abs(local_x), 1e-6))
            angular_z = max(
                -self._config.max_angular_speed,
                min(self._config.max_angular_speed, self._config.angular_gain * heading_error),
            )
            linear_x = -min(self._config.max_reverse_speed, self._config.linear_gain * distance)
            if abs(heading_error) > self._config.heading_slow_threshold:
                linear_x *= 0.5
            return linear_x, angular_z

        heading_error = normalize_angle(math.atan2(local_y, max(local_x, 1e-6)))
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
        avoidance_started_override: bool = False,
    ) -> StateMachineOutput:
        is_resuming = self._resume_until > now_sec
        motion_direction = _determine_motion_direction(target_linear_x, target_angular_z)

        return StateMachineOutput(
            internal_state=self._state,
            behavior_state=self._state.value,
            reason=reason,
            target_linear_x=target_linear_x,
            target_angular_z=target_angular_z,
            scene_label=scene_label,
            active_target=active_target,
            motion_direction=motion_direction,
            is_resuming=is_resuming,
            avoidance_started_event=avoidance_started_override,
        )
