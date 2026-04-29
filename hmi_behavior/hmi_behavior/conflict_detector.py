import math
from typing import Iterable, Tuple

from .robot_state import ActorSnapshot, ObstacleSnapshot


def relative_components(
    robot: ActorSnapshot,
    target_x: float,
    target_y: float,
) -> Tuple[float, float]:
    dx = target_x - robot.x
    dy = target_y - robot.y
    cos_yaw = math.cos(robot.yaw)
    sin_yaw = math.sin(robot.yaw)
    forward = dx * cos_yaw + dy * sin_yaw
    lateral = -dx * sin_yaw + dy * cos_yaw
    return forward, lateral


def distance_between(actor_a: ActorSnapshot, actor_b: ActorSnapshot) -> float:
    return math.hypot(actor_a.x - actor_b.x, actor_a.y - actor_b.y)


def has_path_conflict(
    robot: ActorSnapshot,
    human: ActorSnapshot,
    conflict_distance: float,
    lateral_tolerance: float,
) -> bool:
    if distance_between(robot, human) > conflict_distance:
        return False
    forward, lateral = relative_components(robot, human.x, human.y)
    return forward >= 0.0 and abs(lateral) <= lateral_tolerance


def has_human_passed(
    robot: ActorSnapshot,
    human: ActorSnapshot,
    pass_margin: float,
) -> bool:
    forward, _ = relative_components(robot, human.x, human.y)
    return forward < -pass_margin


def has_static_obstacle_behind(
    robot: ActorSnapshot,
    obstacles: Iterable[ObstacleSnapshot],
    stop_distance: float,
    lateral_tolerance: float,
) -> bool:
    for obstacle in obstacles:
        if not obstacle.is_static:
            continue
        forward, lateral = relative_components(robot, obstacle.x, obstacle.y)
        longitudinal_clearance = abs(forward) - obstacle.length * 0.5
        lateral_clearance = abs(lateral) - obstacle.width * 0.5
        if (
            forward < 0.0
            and longitudinal_clearance <= stop_distance
            and lateral_clearance <= lateral_tolerance
        ):
            return True
    return False
