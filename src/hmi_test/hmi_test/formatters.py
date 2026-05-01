from nav_msgs.msg import OccupancyGrid


def format_stamp(stamp) -> str:
    return f'{stamp.sec}.{stamp.nanosec:09d}'


def format_float(value: float) -> str:
    return f'{value:.3f}'


def summarize_occupancy_grid(msg: OccupancyGrid) -> str:
    total_cells = len(msg.data)
    unknown_cells = sum(1 for value in msg.data if value < 0)
    occupied_cells = sum(1 for value in msg.data if value > 0)
    free_cells = total_cells - unknown_cells - occupied_cells
    occupied_ratio = (occupied_cells / total_cells * 100.0) if total_cells else 0.0

    return (
        f'frame={msg.header.frame_id or "<empty>"} stamp={format_stamp(msg.header.stamp)} '
        f'size={msg.info.width}x{msg.info.height} resolution={format_float(msg.info.resolution)}m/cell '
        f'origin=({format_float(msg.info.origin.position.x)}, '
        f'{format_float(msg.info.origin.position.y)}) total={total_cells} '
        f'free={free_cells} occupied={occupied_cells} unknown={unknown_cells} '
        f'occupied_ratio={occupied_ratio:.1f}%'
    )
