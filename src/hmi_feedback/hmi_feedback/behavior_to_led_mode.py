from __future__ import annotations

from typing import Optional

import rclpy
from hmi_interfaces.msg import BehaviorState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import String


class BehaviorToLedModeNode(Node):
    def __init__(self) -> None:
        super().__init__('behavior_to_led_mode')

        behavior_state_topic = self.declare_parameter(
            'behavior_state_topic',
            '/hmi/control/behavior_state',
            ParameterDescriptor(description='Input BehaviorState topic.'),
        ).value
        led_mode_topic = self.declare_parameter(
            'led_mode_topic',
            '/hmi/visual/led_mode',
            ParameterDescriptor(description='Output String LED mode topic for ros_gz_bridge.'),
        ).value

        self._publisher = self.create_publisher(String, led_mode_topic, 10)
        self.create_subscription(BehaviorState, behavior_state_topic, self._on_behavior_state, 20)
        self._last_mode: Optional[str] = None

        self.get_logger().info(
            f'behavior_to_led_mode listening on {behavior_state_topic} and publishing {led_mode_topic}.'
        )

    def _on_behavior_state(self, msg: BehaviorState) -> None:
        mode = self._map_led_mode(msg)
        if mode == self._last_mode:
            return

        output = String()
        output.data = mode
        self._publisher.publish(output)
        self._last_mode = mode
        self.get_logger().info(f'led_mode={mode}')

    def _map_led_mode(self, msg: BehaviorState) -> str:
        if msg.internal_state == 'HardStop':
            return 'red_fast_blink'

        if msg.is_resuming or msg.current_state == 'Resume':
            return 'green_fade_to_white'

        if msg.internal_state == 'ConflictAvoid':
            if msg.motion_direction == 'backward' or msg.target_linear_x < -0.01:
                return 'green_backward_flow'
            if msg.motion_direction == 'left_turn' or msg.target_angular_z > 0.15:
                return 'green_ccw_flow'
            if msg.motion_direction == 'right_turn' or msg.target_angular_z < -0.15:
                return 'green_cw_flow'
            return 'green_steady'

        if msg.internal_state == 'Wait':
            return 'green_steady'

        if msg.internal_state == 'Idle':
            return 'white_dim'

        return 'white_steady'


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = BehaviorToLedModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
