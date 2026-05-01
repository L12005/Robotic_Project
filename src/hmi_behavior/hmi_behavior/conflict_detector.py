from __future__ import annotations

import math
from dataclasses import dataclass

from hmi_behavior.robot_state import AggregatedState, transform_world_to_local


@dataclass
class ConflictAssessment:
    conflict: bool
    hard_stop: bool
    exit_zone: bool
    obstacle_behind: bool
    distance_to_human: float
    reason: str


def assess_conflict(
    context: AggregatedState,
    safety_radius: float,
    human_zone_time: float,
    human_exit_time: float,
    human_zone_half_width: float,
    reverse_obstacle_distance: float,
    reverse_obstacle_half_width: float,
) -> ConflictAssessment:
    obstacle_behind = _obstacle_blocks_reverse(
        context,
        reverse_obstacle_distance=reverse_obstacle_distance,
        reverse_obstacle_half_width=reverse_obstacle_half_width,
    )

    if context.robot is None or context.human is None:
        return ConflictAssessment(
            conflict=False,
            hard_stop=False,
            exit_zone=False,
            obstacle_behind=obstacle_behind,
            distance_to_human=math.inf,
            reason='obstacle_back' if obstacle_behind else '',
        )

    distance_to_human = math.hypot(context.robot.x - context.human.x, context.robot.y - context.human.y)
    human_speed = max(abs(context.human.linear_x), 0.1 if context.human.is_moving else 0.0)
    conflict_depth = human_speed * human_zone_time
    exit_depth = human_speed * human_exit_time

    robot_local_x, robot_local_y = transform_world_to_local(context.human, context.robot.x, context.robot.y)
    inside_forward_zone = 0.0 <= robot_local_x <= conflict_depth and abs(robot_local_y) <= human_zone_half_width
    inside_exit_zone = 0.0 <= robot_local_x <= exit_depth and abs(robot_local_y) <= human_zone_half_width
    hard_stop = distance_to_human <= safety_radius
    conflict = hard_stop or inside_forward_zone

    if obstacle_behind:
        reason = 'obstacle_back'
    elif conflict:
        reason = 'human_close'
    else:
        reason = ''

    return ConflictAssessment(
        conflict=conflict,
        hard_stop=hard_stop,
        exit_zone=inside_exit_zone,
        obstacle_behind=obstacle_behind,
        distance_to_human=distance_to_human,
        reason=reason,
    )


def _obstacle_blocks_reverse(
    context: AggregatedState,
    reverse_obstacle_distance: float,
    reverse_obstacle_half_width: float,
) -> bool:
    if context.robot is None:
        return False

    for obstacle in context.obstacles.values():
        local_x, local_y = transform_world_to_local(context.robot, obstacle.x, obstacle.y)
        half_width = reverse_obstacle_half_width + obstacle.width * 0.5
        half_length = reverse_obstacle_distance + obstacle.length * 0.5
        if -half_length <= local_x <= 0.1 and abs(local_y) <= half_width:
            return True
    return False
