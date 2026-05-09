from __future__ import annotations

from typing import Optional

from hmi_behavior.elevator_policy import generate_yield_candidates
from hmi_behavior.robot_state import AggregatedState, MotionTarget


def generate_open_area_candidates(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    sample_count: int,
) -> list[MotionTarget]:
    return generate_yield_candidates(
        context,
        reverse_distance=reverse_distance,
        lateral_offset=lateral_offset,
        sample_count=sample_count,
        center_first=False,
        source='open_area_policy',
    )


def plan_yield_goal(
    context: AggregatedState,
    reverse_distance: float,
    lateral_offset: float,
    side_probe_distance: float,
) -> Optional[MotionTarget]:
    candidates = generate_open_area_candidates(
        context,
        reverse_distance=reverse_distance,
        lateral_offset=lateral_offset,
        sample_count=max(12, int(side_probe_distance * 12.0)),
    )
    return candidates[0] if candidates else None
