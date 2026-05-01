import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from hmi_test.formatters import summarize_occupancy_grid


class MapStateMonitor(Node):
    def __init__(self) -> None:
        super().__init__('map_state_monitor')
        self.create_subscription(
            OccupancyGrid,
            '/hmi/scene/map_state',
            self._callback,
            10,
        )
        self.get_logger().info(
            'Monitoring /hmi/scene/map_state as OccupancyGrid for planning map validation.'
        )

    def _callback(self, msg: OccupancyGrid) -> None:
        self.get_logger().info(f'map_state {summarize_occupancy_grid(msg)}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapStateMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
