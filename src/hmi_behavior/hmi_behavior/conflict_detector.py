from __future__ import annotations

import math
from dataclasses import dataclass

from hmi_behavior.robot_state import AggregatedState, effective_human_zone_speed, transform_world_to_local


@dataclass
class ConflictAssessment:
    human_relevant: bool
    conflict: bool
    hard_stop: bool
    exit_zone: bool
    distance_to_human: float
    reason: str


def assess_conflict(
    context: AggregatedState,
    safety_radius: float,
    human_zone_time: float,
    human_exit_time: float,
    human_zone_half_width: float,
    human_relevance_distance: float,
    robot_body_size_x: float,
    robot_body_size_y: float,
    human_body_size_x: float,
    human_body_size_y: float,
) -> ConflictAssessment:
    if context.robot is None or context.human is None:
        return ConflictAssessment(
            human_relevant=False,
            conflict=False,
            hard_stop=False,
            exit_zone=False,
            distance_to_human=math.inf,
            reason='',
        )

    distance_to_human = math.hypot(context.robot.x - context.human.x, context.robot.y - context.human.y)
    human_relevant = distance_to_human <= human_relevance_distance

    human_speed = effective_human_zone_speed(context.human)
    conflict_depth = human_speed * human_zone_time
    exit_depth = human_speed * human_exit_time
    robot_contour_radius = _contour_radius(robot_body_size_x, robot_body_size_y)
    human_contour_radius = _human_contour_radius(
        context,
        default_size_x=human_body_size_x,
        default_size_y=human_body_size_y,
    )

    robot_local_x, robot_local_y = transform_world_to_local(context.human, context.robot.x, context.robot.y)
    inside_forward_zone = (
        human_relevant and 0.0 <= robot_local_x <= conflict_depth and abs(robot_local_y) <= human_zone_half_width
    )
    inside_exit_zone = (
        human_relevant and 0.0 <= robot_local_x <= exit_depth and abs(robot_local_y) <= human_zone_half_width
    )
    hard_stop = distance_to_human <= robot_contour_radius + human_contour_radius + safety_radius
    conflict = hard_stop or inside_forward_zone

    reason = 'human_close' if conflict else ''

    return ConflictAssessment(
        human_relevant=human_relevant,
        conflict=conflict,
        hard_stop=hard_stop,
        exit_zone=inside_exit_zone,
        distance_to_human=distance_to_human,
        reason=reason,
    )


def _contour_radius(size_x: float, size_y: float) -> float:
    half_x = max(0.0, float(size_x)) * 0.5
    half_y = max(0.0, float(size_y)) * 0.5
    return math.hypot(half_x, half_y)


def _human_contour_radius(
    context: AggregatedState,
    default_size_x: float,
    default_size_y: float,
) -> float:
    if context.human is None:
        return _contour_radius(default_size_x, default_size_y)

    nearest_dynamic_obstacle = None
    nearest_distance = math.inf
    for obstacle in context.obstacles.values():
        if obstacle.is_static:
            continue
        distance = math.hypot(obstacle.x - context.human.x, obstacle.y - context.human.y)
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_dynamic_obstacle = obstacle

    if nearest_dynamic_obstacle is not None and nearest_distance <= 1.5:
        return _contour_radius(nearest_dynamic_obstacle.width, nearest_dynamic_obstacle.length)
    return _contour_radius(default_size_x, default_size_y)
