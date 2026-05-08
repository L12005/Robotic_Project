from __future__ import annotations

import math
from typing import Optional

from hmi_behavior.robot_state import AggregatedState, MotionTarget


def generate_yield_candidates(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    sample_count: int,
    center_first: bool = True,
    source: str = 'elevator_policy',
) -> list[MotionTarget]:
    if context.robot is None:
        return []

    axis_yaw = _yield_axis_yaw(context)
    direction = _choose_safer_side(context, axis_yaw)
    base_radius = max(0.25, reverse_distance)
    candidates: list[MotionTarget] = []
    preferred_angles = _preferred_angles(direction, center_first)

    for angle, lateral_sign in preferred_angles:
        candidates.append(
            MotionTarget(
                x=context.robot.x + math.cos(axis_yaw + angle) * base_radius
                - math.sin(axis_yaw) * (lateral_sign * lateral_offset * 0.5),
                y=context.robot.y + math.sin(axis_yaw + angle) * base_radius
                + math.cos(axis_yaw) * (lateral_sign * lateral_offset * 0.5),
                reverse_ok=True,
                source=source,
            )
        )

    for index in range(sample_count):
        angle = -math.pi + (2.0 * math.pi * index / max(1, sample_count))
        candidates.append(
            MotionTarget(
                x=context.robot.x + math.cos(axis_yaw + angle) * base_radius,
                y=context.robot.y + math.sin(axis_yaw + angle) * base_radius,
                reverse_ok=True,
                source=source,
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


def _yield_axis_yaw(context: AggregatedState) -> float:
    assert context.robot is not None

    if context.human is None:
        return context.robot.yaw + math.pi

    away_x = context.robot.x - context.human.x
    away_y = context.robot.y - context.human.y
    away_distance = math.hypot(away_x, away_y)
    if context.human.is_moving:
        axis_yaw = context.human.yaw
        if away_distance > 1e-6:
            heading_x = math.cos(axis_yaw)
            heading_y = math.sin(axis_yaw)
            if heading_x * away_x + heading_y * away_y < 0.0:
                axis_yaw += math.pi
        return axis_yaw

    if away_distance > 1e-6:
        return math.atan2(away_y, away_x)
    return context.human.yaw + math.pi


def _preferred_angles(direction: int, center_first: bool) -> list[tuple[float, int]]:
    side_angles = [
        (direction * 0.45, direction),
        (-direction * 0.45, -direction),
        (direction * 0.85, direction),
        (-direction * 0.85, -direction),
    ]
    center = (0.0, 0)
    if center_first:
        return [center, *side_angles]
    return [*side_angles, center]


def _choose_safer_side(context: AggregatedState, axis_yaw: float) -> int:
    if context.robot is None:
        return 1

    if context.human is not None:
        human_local_y = _axis_local_y(
            context.robot.x,
            context.robot.y,
            axis_yaw,
            context.human.x,
            context.human.y,
        )
        preferred = -1 if human_local_y >= 0.0 else 1
    else:
        preferred = 1

    left_clearance = _side_clearance(context, axis_yaw=axis_yaw, side_sign=1)
    right_clearance = _side_clearance(context, axis_yaw=axis_yaw, side_sign=-1)
    if preferred == 1 and left_clearance >= 0.3:
        return 1
    if preferred == -1 and right_clearance >= 0.3:
        return -1
    return 1 if left_clearance >= right_clearance else -1


def _axis_local_y(origin_x: float, origin_y: float, axis_yaw: float, x: float, y: float) -> float:
    dx = x - origin_x
    dy = y - origin_y
    return -math.sin(axis_yaw) * dx + math.cos(axis_yaw) * dy


def _side_clearance(context: AggregatedState, axis_yaw: float, side_sign: int) -> float:
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
        axis_yaw,
        side_sign=side_sign,
        max_distance=1.0,
    )
