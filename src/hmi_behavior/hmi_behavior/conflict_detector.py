from __future__ import annotations

import math
from dataclasses import dataclass

from hmi_behavior.robot_state import AggregatedState, effective_human_zone_speed, transform_world_to_local


@dataclass
class ConflictAssessment:
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
) -> ConflictAssessment:
    if context.robot is None or context.human is None:
        return ConflictAssessment(
            conflict=False,
            hard_stop=False,
            exit_zone=False,
            distance_to_human=math.inf,
            reason='',
        )

    distance_to_human = math.hypot(context.robot.x - context.human.x, context.robot.y - context.human.y)
    human_speed = effective_human_zone_speed(context.human)
    conflict_depth = human_speed * human_zone_time
    exit_depth = human_speed * human_exit_time

    robot_local_x, robot_local_y = transform_world_to_local(context.human, context.robot.x, context.robot.y)
    inside_forward_zone = 0.0 <= robot_local_x <= conflict_depth and abs(robot_local_y) <= human_zone_half_width
    inside_exit_zone = 0.0 <= robot_local_x <= exit_depth and abs(robot_local_y) <= human_zone_half_width
    hard_stop = distance_to_human <= safety_radius
    conflict = hard_stop or inside_forward_zone

    reason = 'human_close' if conflict else ''

    return ConflictAssessment(
        conflict=conflict,
        hard_stop=hard_stop,
        exit_zone=inside_exit_zone,
        distance_to_human=distance_to_human,
        reason=reason,
    )
