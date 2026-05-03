from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from hmi_test.formatters import format_binary_grid_rows, occupancy_grid_to_binary_rows, summarize_occupancy_grid


class MapGridDump(Node):
    def __init__(self) -> None:
        super().__init__('map_grid_dump')
        self._output_txt = Path('/tmp/hmi_scene_map_state_grid.txt')
        self._output_csv = Path('/tmp/hmi_scene_map_state_grid.csv')
        self.create_subscription(
            OccupancyGrid,
            '/hmi/scene/map_state',
            self._map_state_callback,
            10,
        )
        self.get_logger().info(
            'Waiting for one /hmi/scene/map_state message, then dumping the binary 2D grid to terminal and /tmp.'
        )

    def _map_state_callback(self, msg: OccupancyGrid) -> None:
        rows = occupancy_grid_to_binary_rows(msg)
        formatted_rows = format_binary_grid_rows(rows)
        csv_rows = '\n'.join(','.join(str(value) for value in row) for row in rows)

        self._output_txt.write_text(formatted_rows + '\n', encoding='utf-8')
        self._output_csv.write_text(csv_rows + '\n', encoding='utf-8')

        self.get_logger().info(summarize_occupancy_grid(msg))
        self.get_logger().info(f'Binary grid written to {self._output_txt} and {self._output_csv}')
        print(formatted_rows, flush=True)
        raise SystemExit(0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapGridDump()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
