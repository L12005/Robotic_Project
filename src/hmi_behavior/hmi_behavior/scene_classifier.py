from __future__ import annotations

from hmi_behavior.robot_state import AggregatedState, lateral_clearance, select_base_map


def classify_scene(
    context: AggregatedState,
    narrow_width_threshold: float,
    side_probe_distance: float,
) -> str:
    if context.robot is None:
        return 'unknown'

    map_state = select_base_map(context)
    left_clearance = lateral_clearance(
        map_state,
        context.robot.x,
        context.robot.y,
        context.robot.yaw,
        side_sign=1,
        max_distance=side_probe_distance,
    )
    right_clearance = lateral_clearance(
        map_state,
        context.robot.x,
        context.robot.y,
        context.robot.yaw,
        side_sign=-1,
        max_distance=side_probe_distance,
    )
    local_width = left_clearance + right_clearance
    if local_width <= narrow_width_threshold:
        return 'elevator'
    return 'open_area'
