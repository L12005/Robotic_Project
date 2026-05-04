from __future__ import annotations

import math
from typing import Optional

from hmi_behavior.robot_state import AggregatedState, MotionTarget, transform_world_to_local


def generate_yield_candidates(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    sample_count: int,
) -> list[MotionTarget]:
    if context.robot is None:
        return []

    direction = _choose_safer_side(context)
    base_radius = max(0.25, reverse_distance)
    candidates: list[MotionTarget] = []
    preferred_angles = [
        math.pi,
        math.pi + direction * 0.45,
        math.pi - direction * 0.45,
        math.pi + direction * 0.85,
        math.pi - direction * 0.85,
    ]

    for angle in preferred_angles:
        candidates.append(
            MotionTarget(
                x=context.robot.x + math.cos(context.robot.yaw + angle) * base_radius
                - math.sin(context.robot.yaw + angle) * (direction * lateral_offset * 0.5),
                y=context.robot.y + math.sin(context.robot.yaw + angle) * base_radius
                + math.cos(context.robot.yaw + angle) * (direction * lateral_offset * 0.5),
                reverse_ok=True,
                source='elevator_policy',
            )
        )

    for index in range(sample_count):
        angle = math.pi - (2.0 * math.pi * index / max(1, sample_count))
        candidates.append(
            MotionTarget(
                x=context.robot.x + math.cos(context.robot.yaw + angle) * base_radius,
                y=context.robot.y + math.sin(context.robot.yaw + angle) * base_radius,
                reverse_ok=True,
                source='elevator_policy',
            )
        )

    # Remove duplicates while preserving order.
    unique: list[MotionTarget] = []
    seen: set[tuple[int, int]] = set()
    for candidate in candidates:
        key = (round(candidate.x * 100.0), round(candidate.y * 100.0))
        if key in seen:
            continue
        seen.add(key)
        unique.append(candidate)
    return unique


def plan_yield_goal(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    side_probe_distance: float,
) -> Optional[MotionTarget]:
    candidates = generate_yield_candidates(
        context,
        reverse_distance=reverse_distance,
        lateral_offset=lateral_offset,
        sample_count=max(8, int(side_probe_distance * 8.0)),
    )
    return candidates[0] if candidates else None


def _choose_safer_side(context: AggregatedState) -> int:
    if context.robot is None:
        return 1

    if context.human is not None:
        _, human_local_y = transform_world_to_local(context.robot, context.human.x, context.human.y)
        preferred = -1 if human_local_y >= 0.0 else 1
    else:
        preferred = 1

    left_clearance = _side_clearance(context, side_sign=1)
    right_clearance = _side_clearance(context, side_sign=-1)
    if preferred == 1 and left_clearance >= 0.3:
        return 1
    if preferred == -1 and right_clearance >= 0.3:
        return -1
    return 1 if left_clearance >= right_clearance else -1


def _side_clearance(context: AggregatedState, side_sign: int) -> float:
    if context.robot is None:
        return 0.0
    if context.static_map is None and context.map_state is None:
        return 1.0

    map_state = context.static_map if context.static_map is not None else context.map_state
    assert map_state is not None
    from hmi_behavior.robot_state import lateral_clearance

    return lateral_clearance(
        map_state,
        context.robot.x,
        context.robot.y,
        context.robot.yaw,
        side_sign=side_sign,
        max_distance=1.0,
    )
