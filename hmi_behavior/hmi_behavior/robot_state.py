from dataclasses import dataclass, field
from typing import Dict, Optional

from hmi_interfaces.msg import ActorState, ObstacleState


def stamp_to_seconds(msg_stamp) -> float:
    return float(msg_stamp.sec) + float(msg_stamp.nanosec) / 1_000_000_000.0


@dataclass
class ActorSnapshot:
    actor_id: str = ""
    actor_type: str = ""
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_x: float = 0.0
    angular_z: float = 0.0
    is_moving: bool = False
    stamp_sec: float = 0.0

    @classmethod
    def from_msg(cls, msg: ActorState) -> "ActorSnapshot":
        return cls(
            actor_id=msg.actor_id,
            actor_type=msg.actor_type,
            x=msg.x,
            y=msg.y,
            yaw=msg.yaw,
            linear_x=msg.linear_x,
            angular_z=msg.angular_z,
            is_moving=msg.is_moving,
            stamp_sec=stamp_to_seconds(msg.header.stamp),
        )


@dataclass
class ObstacleSnapshot:
    obstacle_id: str = ""
    x: float = 0.0
    y: float = 0.0
    width: float = 0.0
    length: float = 0.0
    is_static: bool = True
    stamp_sec: float = 0.0

    @classmethod
    def from_msg(cls, msg: ObstacleState) -> "ObstacleSnapshot":
        return cls(
            obstacle_id=msg.obstacle_id,
            x=msg.x,
            y=msg.y,
            width=msg.width,
            length=msg.length,
            is_static=msg.is_static,
            stamp_sec=stamp_to_seconds(msg.header.stamp),
        )


@dataclass
class SceneState:
    robot: Optional[ActorSnapshot] = None
    human: Optional[ActorSnapshot] = None
    obstacles: Dict[str, ObstacleSnapshot] = field(default_factory=dict)

    def update_robot(self, msg: ActorState) -> None:
        self.robot = ActorSnapshot.from_msg(msg)

    def update_human(self, msg: ActorState) -> None:
        self.human = ActorSnapshot.from_msg(msg)

    def update_obstacle(self, msg: ObstacleState) -> None:
        obstacle = ObstacleSnapshot.from_msg(msg)
        obstacle_id = obstacle.obstacle_id or "default_obstacle"
        self.obstacles[obstacle_id] = obstacle
