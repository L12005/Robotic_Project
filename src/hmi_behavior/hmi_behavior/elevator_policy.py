from __future__ import annotations

import math
from typing import Optional

from hmi_behavior.robot_state import (
    AggregatedState,
    MotionTarget,
    lateral_clearance,
    segment_is_free,
    transform_world_to_local,
)


def plan_yield_goal(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    side_probe_distance: float,
) -> Optional[MotionTarget]:
    return plan_named_yield_goal(
        context,
        reverse_distance=reverse_distance,
        lateral_offset=lateral_offset,
        side_probe_distance=side_probe_distance,
        source='elevator_policy',
    )


def plan_named_yield_goal(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    side_probe_distance: float,
    source: str,
) -> Optional[MotionTarget]:
    return _plan_common_yield_goal(
        context,
        reverse_distance=reverse_distance,
        lateral_offset=lateral_offset,
        side_probe_distance=side_probe_distance,
        source=source,
    )


def _plan_common_yield_goal(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    side_probe_distance: float,
    source: str,
) -> Optional[MotionTarget]:
    if context.robot is None:
        return None

    direction = _choose_safer_side(context, side_probe_distance)
    backward_x = context.robot.x - math.cos(context.robot.yaw) * reverse_distance
    backward_y = context.robot.y - math.sin(context.robot.yaw) * reverse_distance

    target_x = backward_x - math.sin(context.robot.yaw) * direction * lateral_offset
    target_y = backward_y + math.cos(context.robot.yaw) * direction * lateral_offset

    if not segment_is_free(context.map_state, context.robot.x, context.robot.y, target_x, target_y):
        target_x = backward_x
        target_y = backward_y
        if not segment_is_free(context.map_state, context.robot.x, context.robot.y, target_x, target_y):
            return None

    return MotionTarget(x=target_x, y=target_y, reverse_ok=True, source=source)


def _choose_safer_side(context: AggregatedState, side_probe_distance: float) -> int:
    if context.robot is None:
        return 1

    if context.human is not None:
        _, human_local_y = transform_world_to_local(context.robot, context.human.x, context.human.y)
        preferred = -1 if human_local_y >= 0.0 else 1
    else:
        preferred = 1

    left_clearance = lateral_clearance(
        context.map_state,
        context.robot.x,
        context.robot.y,
        context.robot.yaw,
        side_sign=1,
        max_distance=side_probe_distance,
    )
    right_clearance = lateral_clearance(
        context.map_state,
        context.robot.x,
        context.robot.y,
        context.robot.yaw,
        side_sign=-1,
        max_distance=side_probe_distance,
    )
    if preferred == 1 and left_clearance >= 0.3:
        return 1
    if preferred == -1 and right_clearance >= 0.3:
        return -1
    return 1 if left_clearance >= right_clearance else -1
