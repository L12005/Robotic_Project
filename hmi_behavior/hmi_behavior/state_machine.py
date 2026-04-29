from dataclasses import dataclass

from .conflict_detector import (
    has_human_passed,
    has_path_conflict,
    has_static_obstacle_behind,
)
from .robot_state import SceneState


@dataclass
class StateMachineConfig:
    forward_speed: float = 0.25
    backward_speed: float = -0.20
    conflict_distance: float = 1.5
    conflict_lateral_tolerance: float = 0.8
    obstacle_stop_distance: float = 0.5
    obstacle_lateral_tolerance: float = 0.8
    human_pass_margin: float = 0.4
    resume_duration_sec: float = 1.0


@dataclass
class BehaviorDecision:
    current_state: str
    reason: str
    target_linear_x: float
    target_angular_z: float
    internal_state: str
    scene_type: str


class BehaviorStateMachine:
    IDLE = "Idle"
    NAVIGATE = "Navigate"
    CONFLICT_ASSESS = "ConflictAssess"
    YIELD_EXECUTE = "YieldExecute"
    YIELD_WAIT = "YieldWait"
    RESUME = "Resume"

    def __init__(self, config: StateMachineConfig) -> None:
        self._config = config
        self._state = self.IDLE
        self._state_since_sec = 0.0

    @property
    def state(self) -> str:
        return self._state

    def _transition(self, new_state: str, now_sec: float) -> None:
        if new_state != self._state:
            self._state = new_state
            self._state_since_sec = now_sec

    def _decision(
        self,
        current_state: str,
        reason: str,
        linear_x: float,
        angular_z: float,
        scene_type: str,
    ) -> BehaviorDecision:
        return BehaviorDecision(
            current_state=current_state,
            reason=reason,
            target_linear_x=linear_x,
            target_angular_z=angular_z,
            internal_state=self._state,
            scene_type=scene_type,
        )

    def step(self, scene_state: SceneState, now_sec: float, scene_type: str) -> BehaviorDecision:
        robot = scene_state.robot
        human = scene_state.human

        if robot is None:
            self._transition(self.IDLE, now_sec)
            return self._decision("Waiting", "missing_robot_state", 0.0, 0.0, scene_type)

        if human is None:
            self._transition(self.NAVIGATE, now_sec)
            return self._decision(
                "NormalMove",
                "no_human_detected",
                self._config.forward_speed,
                0.0,
                scene_type,
            )

        conflict = has_path_conflict(
            robot,
            human,
            self._config.conflict_distance,
            self._config.conflict_lateral_tolerance,
        )
        obstacle_back = has_static_obstacle_behind(
            robot,
            scene_state.obstacles.values(),
            self._config.obstacle_stop_distance,
            self._config.obstacle_lateral_tolerance,
        )
        human_passed = has_human_passed(robot, human, self._config.human_pass_margin)

        if self._state == self.IDLE:
            self._transition(self.NAVIGATE, now_sec)

        if self._state == self.NAVIGATE:
            if conflict:
                self._transition(self.CONFLICT_ASSESS, now_sec)
                return self._decision("HumanDetected", "human_close", 0.0, 0.0, scene_type)
            return self._decision(
                "NormalMove",
                "path_clear",
                self._config.forward_speed,
                0.0,
                scene_type,
            )

        if self._state == self.CONFLICT_ASSESS:
            if not conflict:
                self._transition(self.NAVIGATE, now_sec)
                return self._decision(
                    "NormalMove",
                    "path_clear",
                    self._config.forward_speed,
                    0.0,
                    scene_type,
                )
            if obstacle_back:
                self._transition(self.YIELD_WAIT, now_sec)
                return self._decision("Waiting", "obstacle_back", 0.0, 0.0, scene_type)
            self._transition(self.YIELD_EXECUTE, now_sec)
            return self._decision(
                "YieldBackward",
                "human_close",
                self._config.backward_speed,
                0.0,
                scene_type,
            )

        if self._state == self.YIELD_EXECUTE:
            if obstacle_back:
                self._transition(self.YIELD_WAIT, now_sec)
                return self._decision("Waiting", "obstacle_back", 0.0, 0.0, scene_type)
            if human_passed or not conflict:
                self._transition(self.YIELD_WAIT, now_sec)
                return self._decision("Waiting", "human_passed", 0.0, 0.0, scene_type)
            return self._decision(
                "YieldBackward",
                "human_close",
                self._config.backward_speed,
                0.0,
                scene_type,
            )

        if self._state == self.YIELD_WAIT:
            if human_passed or not conflict:
                self._transition(self.RESUME, now_sec)
                return self._decision(
                    "Resume",
                    "human_passed",
                    self._config.forward_speed,
                    0.0,
                    scene_type,
                )
            return self._decision("Waiting", "human_close", 0.0, 0.0, scene_type)

        if self._state == self.RESUME:
            if conflict:
                self._transition(self.CONFLICT_ASSESS, now_sec)
                return self._decision("HumanDetected", "human_close", 0.0, 0.0, scene_type)
            if now_sec - self._state_since_sec >= self._config.resume_duration_sec:
                self._transition(self.NAVIGATE, now_sec)
            return self._decision(
                "Resume",
                "human_passed",
                self._config.forward_speed,
                0.0,
                scene_type,
            )

        self._transition(self.NAVIGATE, now_sec)
        return self._decision("NormalMove", "path_clear", self._config.forward_speed, 0.0, scene_type)
