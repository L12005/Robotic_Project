from __future__ import annotations

from nav_msgs.msg import OccupancyGrid

from hmi_behavior.robot_state import ActorSnapshot, AggregatedState, GoalSnapshot, MotionTarget, cell_is_blocked
from hmi_behavior.state_machine import BehaviorStateMachine, ControllerConfig, InternalState


def _actor(
    x: float,
    y: float,
    yaw: float,
    *,
    linear_x: float,
    nominal_linear_x: float,
    is_moving: bool,
    actor_type: str,
) -> ActorSnapshot:
    return ActorSnapshot(
        actor_id=actor_type,
        actor_type=actor_type,
        x=x,
        y=y,
        yaw=yaw,
        linear_x=linear_x,
        nominal_linear_x=nominal_linear_x,
        angular_z=0.0,
        is_moving=is_moving,
        stamp_sec=0.0,
        frame_id="map",
    )


def _goal(x: float, y: float) -> GoalSnapshot:
    return GoalSnapshot(x=x, y=y, yaw=0.0, frame_id="map", stamp_sec=0.0)


def _empty_map() -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.info.resolution = 0.1
    grid.info.width = 200
    grid.info.height = 200
    grid.info.origin.position.x = -10.0
    grid.info.origin.position.y = -10.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * (grid.info.width * grid.info.height)
    return grid


def _corridor_map(half_width: float = 0.45) -> OccupancyGrid:
    grid = _empty_map()
    resolution = grid.info.resolution
    origin_y = grid.info.origin.position.y
    width = grid.info.width
    height = grid.info.height

    for grid_y in range(height):
        world_y = origin_y + (grid_y + 0.5) * resolution
        if abs(world_y) <= half_width:
            continue
        for grid_x in range(width):
            grid.data[grid_y * width + grid_x] = 100
    return grid


def _config() -> ControllerConfig:
    return ControllerConfig(
        goal_tolerance=0.2,
        max_forward_speed=1.2,
        max_reverse_speed=1.1,
        max_angular_speed=1.2,
        angular_gain=1.4,
        linear_gain=1.5,
        wait_duration=0.2,
        safety_radius=0.05,
        human_zone_time=1.8,
        human_exit_time=2.4,
        human_zone_half_width=0.5,
        human_relevance_distance=2.5,
        robot_body_size_x=0.62,
        robot_body_size_y=0.68,
        human_body_size_x=0.95,
        human_body_size_y=0.60,
        reverse_distance_elevator=1.2,
        reverse_distance_open_area=0.75,
        lateral_offset_elevator=0.45,
        lateral_offset_open_area=0.8,
        side_probe_distance=1.0,
        narrow_width_threshold=1.6,
        heading_slow_threshold=0.5,
        heading_stop_threshold=1.2,
        resume_duration=1.2,
        path_lookahead_distance=0.35,
        dynamic_obstacle_inflation=0.18,
        human_body_radius=0.4,
        human_forward_min_depth=0.35,
        yield_sample_count=16,
        planner_occupancy_threshold=50,
    )


def test_wait_enters_conflict_avoiding_navigate_when_conflict_arrives() -> None:
    machine = BehaviorStateMachine(_config())
    machine._state = InternalState.WAIT
    machine._wait_started_at = 1.0

    context = AggregatedState(
        robot=_actor(
            x=0.3,
            y=0.1,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type="robot",
        ),
        human=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.3,
            nominal_linear_x=0.5,
            is_moving=True,
            actor_type="human",
        ),
        goal=_goal(5.0, 0.1),
        static_map=_empty_map(),
    )

    output = machine.step(context, now_sec=1.1)

    assert output.internal_state == InternalState.CONFLICT_AVOIDING_NAVIGATE
    assert output.behavior_state == InternalState.CONFLICT_AVOIDING_NAVIGATE.value
    assert output.reason == "human_close"
    assert output.target_linear_x == 0.0
    assert output.target_angular_z == 0.0


def test_conflict_avoiding_planning_does_not_paint_forward_no_go_zone() -> None:
    machine = BehaviorStateMachine(_config())
    context = AggregatedState(
        robot=_actor(
            x=0.3,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type="robot",
        ),
        human=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.5,
            nominal_linear_x=0.5,
            is_moving=True,
            actor_type="human",
        ),
        goal=_goal(5.0, 0.0),
        static_map=_empty_map(),
    )

    machine._state = InternalState.FORWARD
    blocked_map, _ = machine._build_planning_map(context, context.static_map)
    assert cell_is_blocked(blocked_map, 0.6, 0.0)

    machine._state = InternalState.CONFLICT_AVOIDING_NAVIGATE
    unblocked_map, _ = machine._build_planning_map(context, context.static_map)
    assert not cell_is_blocked(unblocked_map, 0.6, 0.0)

    machine._state = InternalState.CONFLICT_AVOID
    retreat_map, _ = machine._build_planning_map(context, context.static_map)
    assert not cell_is_blocked(retreat_map, 0.6, 0.0)


def test_human_beyond_relevance_distance_only_hides_forward_zone() -> None:
    machine = BehaviorStateMachine(_config())
    context = AggregatedState(
        robot=_actor(
            x=3.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type="robot",
        ),
        human=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=1.0,
            nominal_linear_x=1.0,
            is_moving=True,
            actor_type="human",
        ),
        goal=_goal(5.0, 0.0),
        static_map=_empty_map(),
    )

    machine._state = InternalState.FORWARD
    planning_map, _ = machine._build_planning_map(context, context.static_map)
    assert cell_is_blocked(planning_map, 0.0, 0.0)
    assert not cell_is_blocked(planning_map, 1.0, 0.0)


def test_relevant_human_blocking_goal_prefers_yield_over_wait() -> None:
    machine = BehaviorStateMachine(_config())
    context = AggregatedState(
        robot=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type="robot",
        ),
        human=_actor(
            x=1.0,
            y=0.0,
            yaw=0.0,
            linear_x=1.0,
            nominal_linear_x=1.0,
            is_moving=True,
            actor_type="human",
        ),
        goal=_goal(5.0, 0.0),
        static_map=_corridor_map(),
    )

    output = machine.step(context, now_sec=1.0)

    assert output.internal_state == InternalState.CONFLICT_AVOID
    assert output.behavior_state == InternalState.CONFLICT_AVOID.value
    assert output.reason == "human_close"
    assert output.active_target is not None


def test_conflict_avoid_state_clears_when_human_leaves_relevance_distance() -> None:
    machine = BehaviorStateMachine(_config())
    machine._state = InternalState.CONFLICT_AVOID
    machine._avoidance_session_active = True
    machine._active_target = MotionTarget(
        x=-0.9,
        y=0.0,
        reverse_ok=True,
        source="test",
    )

    context = AggregatedState(
        robot=_actor(
            x=3.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type="robot",
        ),
        human=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=1.0,
            nominal_linear_x=1.0,
            is_moving=True,
            actor_type="human",
        ),
        goal=_goal(5.0, 0.0),
        static_map=_empty_map(),
    )

    output = machine.step(context, now_sec=1.0)

    assert output.internal_state == InternalState.FORWARD
    assert output.reason == ""
    assert output.active_target is None
    assert output.target_linear_x > 0.0


def test_conflict_is_not_cleared_until_active_yield_target_is_reached() -> None:
    machine = BehaviorStateMachine(_config())
    machine._state = InternalState.CONFLICT_AVOID
    machine._avoidance_session_active = True
    machine._active_target = MotionTarget(
        x=-0.9,
        y=0.0,
        reverse_ok=True,
        source="test",
    )

    context = AggregatedState(
        robot=_actor(
            x=0.45,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type="robot",
        ),
        human=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.2,
            nominal_linear_x=0.5,
            is_moving=True,
            actor_type="human",
        ),
        goal=_goal(5.0, 0.0),
        static_map=_empty_map(),
    )

    output = machine.step(context, now_sec=1.0)

    assert output.internal_state == InternalState.CONFLICT_AVOID
    assert output.behavior_state == InternalState.CONFLICT_AVOID.value
    assert output.reason == "human_close"
    assert output.active_target == machine._active_target
    assert output.target_linear_x < 0.0


def test_reverse_retreat_rotates_in_place_before_backing_when_target_is_off_axis() -> None:
    machine = BehaviorStateMachine(_config())
    context = AggregatedState(
        robot=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type="robot",
        ),
        human=_actor(
            x=1.0,
            y=1.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=1.0,
            is_moving=False,
            actor_type="human",
        ),
    )

    linear_x, angular_z = machine._command_to_target(
        context,
        MotionTarget(
            x=-1.0,
            y=0.8,
            reverse_ok=True,
            source="test",
        ),
    )

    assert linear_x == 0.0
    assert angular_z > 0.0
