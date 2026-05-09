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
        human_relevance_distance=2.5,
        robot_body_size_x=0.01,
        robot_body_size_y=0.01,
        human_body_size_x=0.01,
        human_body_size_y=0.01,
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
        human_relevance_distance=2.5,
        robot_body_size_x=0.01,
        robot_body_size_y=0.01,
        human_body_size_x=0.01,
        human_body_size_y=0.01,
    )

    assert assessment.conflict is False
    assert math.isclose(assessment.distance_to_human, math.hypot(0.24, 0.20))


def test_human_beyond_relevance_distance_hides_zones_only() -> None:
    context = AggregatedState(
        robot=_actor(
            x=3.1,
            y=0.0,
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
            linear_x=1.0,
            nominal_linear_x=1.0,
            is_moving=True,
            actor_type='human',
        ),
    )

    assessment = assess_conflict(
        context,
        safety_radius=0.1,
        human_zone_time=4.0,
        human_exit_time=5.0,
        human_zone_half_width=1.0,
        human_relevance_distance=2.5,
        robot_body_size_x=0.62,
        robot_body_size_y=0.68,
        human_body_size_x=0.95,
        human_body_size_y=0.60,
    )

    assert assessment.human_relevant is False
    assert assessment.conflict is False
    assert assessment.exit_zone is False


def test_hard_stop_uses_outer_contour_not_center_distance() -> None:
    context = AggregatedState(
        robot=_actor(
            x=0.0,
            y=0.0,
            yaw=0.0,
            linear_x=0.0,
            nominal_linear_x=0.0,
            is_moving=False,
            actor_type='robot',
        ),
        human=_actor(
            x=1.05,
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
        human_zone_time=0.2,
        human_exit_time=0.2,
        human_zone_half_width=0.1,
        human_relevance_distance=2.5,
        robot_body_size_x=0.62,
        robot_body_size_y=0.68,
        human_body_size_x=0.95,
        human_body_size_y=0.60,
    )

    assert assessment.hard_stop is True
