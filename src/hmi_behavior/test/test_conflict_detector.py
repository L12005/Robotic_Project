import math

from hmi_behavior.conflict_detector import assess_conflict
from hmi_behavior.robot_state import ActorSnapshot, AggregatedState


def _actor(
    x: float,
    y: float,
    yaw: float,
    *,
    linear_x: float,
    nominal_linear_x: float,
    is_moving: bool,
    actor_type: str,
) -> ActorSnapshot:
    return ActorSnapshot(
        actor_id=actor_type,
        actor_type=actor_type,
        x=x,
        y=y,
        yaw=yaw,
        linear_x=linear_x,
        nominal_linear_x=nominal_linear_x,
        angular_z=0.0,
        is_moving=is_moving,
        stamp_sec=0.0,
        frame_id='map',
    )


def test_static_human_uses_nominal_speed_for_conflict_and_exit_zones() -> None:
    context = AggregatedState(
        robot=_actor(
            x=0.0,
            y=0.20,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type='robot',
        ),
        human=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=1.0,
            is_moving=False,
            actor_type='human',
        ),
    )

    assessment = assess_conflict(
        context,
        safety_radius=0.05,
        human_zone_time=1.2,
        human_exit_time=2.0,
        human_zone_half_width=0.5,
    )

    assert assessment.conflict is True
    assert assessment.exit_zone is True


def test_moving_human_uses_actual_speed_not_nominal_speed() -> None:
    context = AggregatedState(
        robot=_actor(
            x=0.24,
            y=0.20,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type='robot',
        ),
        human=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.2,
            nominal_linear_x=1.0,
            is_moving=True,
            actor_type='human',
        ),
    )

    assessment = assess_conflict(
        context,
        safety_radius=0.05,
        human_zone_time=1.2,
        human_exit_time=2.0,
        human_zone_half_width=0.5,
    )

    assert assessment.conflict is False
    assert math.isclose(assessment.distance_to_human, math.hypot(0.24, 0.20))
