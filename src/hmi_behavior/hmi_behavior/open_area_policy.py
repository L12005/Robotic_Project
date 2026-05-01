from __future__ import annotations

from typing import Optional

from hmi_behavior.elevator_policy import plan_named_yield_goal
from hmi_behavior.robot_state import AggregatedState, MotionTarget


def plan_yield_goal(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    side_probe_distance: float,
) -> Optional[MotionTarget]:
    return plan_named_yield_goal(
        context,
        reverse_distance=reverse_distance,
        lateral_offset=lateral_offset,
        side_probe_distance=side_probe_distance,
        source='open_area_policy',
    )
