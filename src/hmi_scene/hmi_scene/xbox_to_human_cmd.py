from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import Joy


class XboxToHumanCmd(Node):
    def __init__(self) -> None:
        super().__init__('xbox_to_human_cmd')

        joy_topic = str(
            self.declare_parameter(
                'joy_topic',
                '/joy',
                ParameterDescriptor(description='Joystick topic published by joy_node.'),
            ).value
        )
        cmd_vel_topic = str(
            self.declare_parameter(
                'cmd_vel_topic',
                '/hmi/human/cmd_vel',
                ParameterDescriptor(description='Human velocity topic consumed by the human scene controller.'),
            ).value
        )
        self._publish_rate_hz = float(
            self.declare_parameter(
                'publish_rate_hz',
                30.0,
                ParameterDescriptor(description='How often to republish the last human cmd_vel command.'),
            ).value
        )
        self._fixed_speed = float(
            self.declare_parameter(
                'fixed_speed',
                1.0,
                ParameterDescriptor(description='Fixed walking speed used whenever the left stick leaves the deadzone.'),
            ).value
        )
        self._deadzone = float(
            self.declare_parameter(
                'deadzone',
                0.25,
                ParameterDescriptor(description='Minimum left-stick magnitude required to trigger walking.'),
            ).value
        )
        self._left_x_axis = int(
            self.declare_parameter(
                'left_x_axis',
                0,
                ParameterDescriptor(description='Axis index for the Xbox left stick horizontal motion.'),
            ).value
        )
        self._left_y_axis = int(
            self.declare_parameter(
                'left_y_axis',
                1,
                ParameterDescriptor(description='Axis index for the Xbox left stick vertical motion.'),
            ).value
        )
        self._invert_left_y = bool(
            self.declare_parameter(
                'invert_left_y',
                True,
                ParameterDescriptor(description='Whether to invert the left stick vertical axis before mapping.'),
            ).value
        )
        self._a_button_index = int(
            self.declare_parameter(
                'a_button_index',
                0,
                ParameterDescriptor(description='Button index for the Xbox A button; when pressed the human stops.'),
            ).value
        )

        self._publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 20)
        self.create_timer(1.0 / max(self._publish_rate_hz, 1.0), self._publish_latest_command)

        self._last_axes: list[float] = []
        self._last_buttons: list[int] = []
        self._last_command_signature: tuple[float, float] | None = None

        self.get_logger().info(
            'Xbox-to-human cmd node is ready. '
            f'joy_topic={joy_topic}, cmd_vel_topic={cmd_vel_topic}, fixed_speed={self._fixed_speed:.2f} m/s.'
        )

    def _on_joy(self, msg: Joy) -> None:
        self._last_axes = list(msg.axes)
        self._last_buttons = list(msg.buttons)

    def _publish_latest_command(self) -> None:
        twist = Twist()

        left_x = self._read_axis(self._last_axes, self._left_x_axis)
        left_y = self._read_axis(self._last_axes, self._left_y_axis)
        if self._invert_left_y:
            left_y = -left_y

        stop_pressed = self._read_button(self._last_buttons, self._a_button_index)
        magnitude = math.hypot(left_x, left_y)

        if not stop_pressed and magnitude >= self._deadzone:
            direction_x = left_x / max(magnitude, 1e-6)
            direction_y = left_y / max(magnitude, 1e-6)
            twist.linear.x = direction_x * self._fixed_speed
            twist.linear.y = direction_y * self._fixed_speed

        self._publisher.publish(twist)
        self._maybe_log_command(twist, stop_pressed)

    def _maybe_log_command(self, twist: Twist, stop_pressed: bool) -> None:
        signature = (round(float(twist.linear.x), 3), round(float(twist.linear.y), 3))
        if signature == self._last_command_signature:
            return
        state = 'stopped' if stop_pressed or signature == (0.0, 0.0) else 'moving'
        self.get_logger().info(
            'human_cmd '
            f'state={state} vx={twist.linear.x:.3f} vy={twist.linear.y:.3f}'
        )
        self._last_command_signature = signature

    def _read_axis(self, values: list[float], index: int) -> float:
        if index < 0 or index >= len(values):
            return 0.0
        return float(values[index])

    def _read_button(self, values: list[int], index: int) -> bool:
        if index < 0 or index >= len(values):
            return False
        return int(values[index]) != 0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = XboxToHumanCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
