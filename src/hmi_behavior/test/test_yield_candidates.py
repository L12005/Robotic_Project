import math

from hmi_behavior.elevator_policy import generate_yield_candidates
from hmi_behavior.open_area_policy import generate_open_area_candidates
from hmi_behavior.robot_state import ActorSnapshot, AggregatedState


def _actor(x: float, y: float, yaw: float, moving: bool = False) -> ActorSnapshot:
    return ActorSnapshot(
        actor_id='actor',
        actor_type='human' if moving else 'robot',
        x=x,
        y=y,
        yaw=yaw,
        linear_x=0.5 if moving else 0.0,
        angular_z=0.0,
        is_moving=moving,
        stamp_sec=0.0,
        frame_id='map',
    )


def _context(robot: ActorSnapshot, human: ActorSnapshot | None) -> AggregatedState:
    return AggregatedState(robot=robot, human=human)


def test_moving_human_axis_points_away_from_human() -> None:
    context = _context(
        robot=_actor(1.0, 0.0, math.pi),
        human=_actor(0.0, 0.0, 0.0, moving=True),
    )

    candidates = generate_yield_candidates(
        context,
        reverse_distance=0.7,
        lateral_offset=0.45,
        sample_count=16,
    )

    assert candidates
    assert candidates[0].x > context.robot.x


def test_moving_human_axis_flips_when_heading_faces_robot() -> None:
    context = _context(
        robot=_actor(1.0, 0.0, 0.0),
        human=_actor(0.0, 0.0, math.pi, moving=True),
    )

    candidates = generate_yield_candidates(
        context,
        reverse_distance=0.7,
        lateral_offset=0.45,
        sample_count=16,
    )

    assert candidates
    assert candidates[0].x > context.robot.x


def test_open_area_prefers_side_candidate_before_centerline() -> None:
    context = _context(
        robot=_actor(1.0, 0.0, 0.0),
        human=_actor(0.0, 0.5, 0.0, moving=True),
    )

    candidates = generate_open_area_candidates(
        context,
        reverse_distance=0.55,
        lateral_offset=0.80,
        sample_count=16,
    )

    assert candidates
    assert abs(candidates[0].y - context.robot.y) > 0.05


def test_elevator_keeps_centerline_first() -> None:
    context = _context(
        robot=_actor(1.0, 0.0, 0.0),
        human=_actor(0.0, 0.5, 0.0, moving=True),
    )

    candidates = generate_yield_candidates(
        context,
        reverse_distance=0.7,
        lateral_offset=0.45,
        sample_count=16,
    )
    expected_y = context.robot.y

    assert candidates
    assert abs(candidates[0].y - expected_y) < 1e-6


def test_static_human_falls_back_to_direct_away_axis() -> None:
    context = _context(
        robot=_actor(0.0, 2.0, 0.0),
        human=_actor(0.0, 0.0, math.pi / 2.0, moving=False),
    )

    candidates = generate_yield_candidates(
        context,
        reverse_distance=0.7,
        lateral_offset=0.45,
        sample_count=16,
    )

    assert candidates
    assert candidates[0].y > context.robot.y
