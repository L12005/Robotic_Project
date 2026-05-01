import rclpy
from hmi_interfaces.msg import ActorState
from rclpy.node import Node

from hmi_test.formatters import format_float, format_stamp


class HumanStateMonitor(Node):
    def __init__(self) -> None:
        super().__init__('human_state_monitor')
        self.create_subscription(
            ActorState,
            '/hmi/scene/human_state',
            self._callback,
            10,
        )
        self.get_logger().info(
            'Monitoring /hmi/scene/human_state as ActorState for conflict detection inputs.'
        )

    def _callback(self, msg: ActorState) -> None:
        motion_hint = 'moving' if msg.is_moving else 'static'
        self.get_logger().info(
            'human_state '
            f'frame={msg.header.frame_id or "<empty>"} stamp={format_stamp(msg.header.stamp)} '
            f'id={msg.actor_id or "<empty>"} type={msg.actor_type or "<empty>"} '
            f'pose=({format_float(msg.x)}, {format_float(msg.y)}, yaw={format_float(msg.yaw)}) '
            f'vel=(linear_x={format_float(msg.linear_x)}, angular_z={format_float(msg.angular_z)}) '
            f'is_moving={msg.is_moving} motion_hint={motion_hint}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HumanStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
