from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Optional

from geometry_msgs.msg import PoseStamped
from hmi_interfaces.msg import ActorState, ObstacleState
from nav_msgs.msg import OccupancyGrid


@dataclass
class ActorSnapshot:
    actor_id: str
    actor_type: str
    x: float
    y: float
    yaw: float
    linear_x: float
    angular_z: float
    is_moving: bool
    stamp_sec: float
    frame_id: str

    @classmethod
    def from_msg(cls, msg: ActorState) -> 'ActorSnapshot':
        return cls(
            actor_id=msg.actor_id,
            actor_type=msg.actor_type,
            x=msg.x,
            y=msg.y,
            yaw=msg.yaw,
            linear_x=msg.linear_x,
            angular_z=msg.angular_z,
            is_moving=msg.is_moving,
            stamp_sec=stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec),
            frame_id=msg.header.frame_id,
        )


@dataclass
class ObstacleSnapshot:
    obstacle_id: str
    x: float
    y: float
    width: float
    length: float
    is_static: bool
    stamp_sec: float

    @classmethod
    def from_msg(cls, msg: ObstacleState) -> 'ObstacleSnapshot':
        return cls(
            obstacle_id=msg.obstacle_id,
            x=msg.x,
            y=msg.y,
            width=msg.width,
            length=msg.length,
            is_static=msg.is_static,
            stamp_sec=stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec),
        )


@dataclass
class GoalSnapshot:
    x: float
    y: float
    yaw: float
    frame_id: str
    stamp_sec: float

    @classmethod
    def from_msg(cls, msg: PoseStamped) -> 'GoalSnapshot':
        orientation = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        return cls(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            yaw=yaw,
            frame_id=msg.header.frame_id,
            stamp_sec=stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec),
        )


@dataclass
class MotionTarget:
    x: float
    y: float
    reverse_ok: bool
    source: str


@dataclass
class AggregatedState:
    robot: Optional[ActorSnapshot] = None
    human: Optional[ActorSnapshot] = None
    map_state: Optional[OccupancyGrid] = None
    goal: Optional[GoalSnapshot] = None
    obstacles: Dict[str, ObstacleSnapshot] = field(default_factory=dict)


def stamp_to_sec(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1e-9


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def distance_xy(ax: float, ay: float, bx: float, by: float) -> float:
    return math.hypot(ax - bx, ay - by)


def target_distance(robot: ActorSnapshot, target: MotionTarget | GoalSnapshot) -> float:
    return distance_xy(robot.x, robot.y, target.x, target.y)


def transform_world_to_local(robot: ActorSnapshot, x: float, y: float) -> tuple[float, float]:
    dx = x - robot.x
    dy = y - robot.y
    cos_yaw = math.cos(robot.yaw)
    sin_yaw = math.sin(robot.yaw)
    return (
        cos_yaw * dx + sin_yaw * dy,
        -sin_yaw * dx + cos_yaw * dy,
    )


def cell_is_blocked(map_state: Optional[OccupancyGrid], x: float, y: float, occupancy_threshold: int = 50) -> bool:
    if map_state is None:
        return False

    resolution = map_state.info.resolution
    origin_x = map_state.info.origin.position.x
    origin_y = map_state.info.origin.position.y
    width = map_state.info.width
    height = map_state.info.height

    grid_x = int((x - origin_x) / resolution)
    grid_y = int((y - origin_y) / resolution)
    if grid_x < 0 or grid_y < 0 or grid_x >= width or grid_y >= height:
        return True

    index = grid_y * width + grid_x
    if index < 0 or index >= len(map_state.data):
        return True
    value = map_state.data[index]
    return value < 0 or value >= occupancy_threshold


def segment_is_free(
    map_state: Optional[OccupancyGrid],
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    step: float = 0.05,
) -> bool:
    if map_state is None:
        return True

    distance = distance_xy(start_x, start_y, end_x, end_y)
    if distance < 1e-6:
        return not cell_is_blocked(map_state, end_x, end_y)

    samples = max(2, int(distance / step) + 1)
    for index in range(samples + 1):
        ratio = index / samples
        probe_x = start_x + (end_x - start_x) * ratio
        probe_y = start_y + (end_y - start_y) * ratio
        if cell_is_blocked(map_state, probe_x, probe_y):
            return False
    return True


def lateral_clearance(
    map_state: Optional[OccupancyGrid],
    x: float,
    y: float,
    yaw: float,
    side_sign: int,
    max_distance: float,
    step: float = 0.05,
) -> float:
    if map_state is None:
        return max_distance

    distance = 0.0
    while distance <= max_distance:
        probe_x = x - math.sin(yaw) * side_sign * distance
        probe_y = y + math.cos(yaw) * side_sign * distance
        if cell_is_blocked(map_state, probe_x, probe_y):
            return max(0.0, distance - step)
        distance += step
    return max_distance
