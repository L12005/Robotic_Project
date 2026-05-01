import rclpy
from hmi_interfaces.msg import ObstacleState
from rclpy.node import Node

from hmi_test.formatters import format_float, format_stamp


class ObstacleStateMonitor(Node):
    def __init__(self) -> None:
        super().__init__('obstacle_state_monitor')
        self.create_subscription(
            ObstacleState,
            '/hmi/scene/obstacle_state',
            self._callback,
            10,
        )
        self.get_logger().info(
            'Monitoring /hmi/scene/obstacle_state as ObstacleState for runtime obstacle overlays.'
        )

    def _callback(self, msg: ObstacleState) -> None:
        self.get_logger().info(
            'obstacle_state '
            f'stamp={format_stamp(msg.header.stamp)} '
            f'id={msg.obstacle_id or "<empty>"} '
            f'center=({format_float(msg.x)}, {format_float(msg.y)}) '
            f'size=(width={format_float(msg.width)}, length={format_float(msg.length)}) '
            f'is_static={msg.is_static}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
