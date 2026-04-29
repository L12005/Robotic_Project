from geometry_msgs.msg import Twist
from hmi_interfaces.msg import ActorState, BehaviorState, ObstacleState
import rclpy
from rclpy.node import Node

from .robot_state import SceneState
from .scene_classifier import SceneClassifier
from .state_machine import BehaviorStateMachine, StateMachineConfig


class BehaviorNode(Node):
    def __init__(self) -> None:
        super().__init__("hmi_behavior")

        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("default_scene_type", "open_area")
        self.declare_parameter("forward_speed", 0.25)
        self.declare_parameter("backward_speed", -0.20)
        self.declare_parameter("conflict_distance", 1.5)
        self.declare_parameter("conflict_lateral_tolerance", 0.8)
        self.declare_parameter("obstacle_stop_distance", 0.5)
        self.declare_parameter("obstacle_lateral_tolerance", 0.8)
        self.declare_parameter("human_pass_margin", 0.4)
        self.declare_parameter("resume_duration_sec", 1.0)

        config = StateMachineConfig(
            forward_speed=float(self.get_parameter("forward_speed").value),
            backward_speed=float(self.get_parameter("backward_speed").value),
            conflict_distance=float(self.get_parameter("conflict_distance").value),
            conflict_lateral_tolerance=float(
                self.get_parameter("conflict_lateral_tolerance").value
            ),
            obstacle_stop_distance=float(self.get_parameter("obstacle_stop_distance").value),
            obstacle_lateral_tolerance=float(
                self.get_parameter("obstacle_lateral_tolerance").value
            ),
            human_pass_margin=float(self.get_parameter("human_pass_margin").value),
            resume_duration_sec=float(self.get_parameter("resume_duration_sec").value),
        )

        self._scene_state = SceneState()
        self._scene_classifier = SceneClassifier(
            default_scene_type=str(self.get_parameter("default_scene_type").value)
        )
        self._state_machine = BehaviorStateMachine(config)
        self._last_reported_key = None

        self._cmd_vel_pub = self.create_publisher(Twist, "/hmi/control/cmd_vel", 10)
        self._behavior_state_pub = self.create_publisher(
            BehaviorState,
            "/hmi/control/behavior_state",
            10,
        )

        self.create_subscription(
            ActorState,
            "/hmi/scene/robot_state",
            self._on_robot_state,
            10,
        )
        self.create_subscription(
            ActorState,
            "/hmi/scene/human_state",
            self._on_human_state,
            10,
        )
        self.create_subscription(
            ObstacleState,
            "/hmi/scene/obstacle_state",
            self._on_obstacle_state,
            10,
        )

        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.create_timer(1.0 / control_rate_hz, self._on_control_timer)

    def _on_robot_state(self, msg: ActorState) -> None:
        self._scene_state.update_robot(msg)

    def _on_human_state(self, msg: ActorState) -> None:
        self._scene_state.update_human(msg)

    def _on_obstacle_state(self, msg: ObstacleState) -> None:
        self._scene_state.update_obstacle(msg)

    def _on_control_timer(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1_000_000_000.0
        scene_type = self._scene_classifier.classify(self._scene_state)
        decision = self._state_machine.step(self._scene_state, now_sec, scene_type)

        twist = Twist()
        twist.linear.x = decision.target_linear_x
        twist.angular.z = decision.target_angular_z
        self._cmd_vel_pub.publish(twist)

        behavior_msg = BehaviorState()
        behavior_msg.header.stamp = now.to_msg()
        behavior_msg.current_state = decision.current_state
        behavior_msg.reason = decision.reason
        behavior_msg.target_linear_x = decision.target_linear_x
        behavior_msg.target_angular_z = decision.target_angular_z
        self._behavior_state_pub.publish(behavior_msg)

        report_key = (decision.internal_state, decision.current_state, decision.reason, scene_type)
        if report_key != self._last_reported_key:
            self.get_logger().info(
                "state=%s behavior=%s reason=%s scene=%s"
                % (
                    decision.internal_state,
                    decision.current_state,
                    decision.reason,
                    decision.scene_type,
                )
            )
            self._last_reported_key = report_key


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
