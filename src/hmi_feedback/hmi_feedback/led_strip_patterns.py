from __future__ import annotations

import math
from dataclasses import dataclass


WHITE = (1.0, 1.0, 1.0)
GREEN = (0.0, 1.0, 0.25)
RED = (1.0, 0.0, 0.0)


@dataclass(frozen=True)
class SegmentColor:
    red: float
    green: float
    blue: float
    intensity: float


@dataclass(frozen=True)
class LedFrame:
    mode: str
    direction: str
    segments: tuple[SegmentColor, ...]


def build_led_frame(
    *,
    internal_state: str,
    motion_direction: str,
    is_resuming: bool,
    now_sec: float,
    segment_count: int,
    flow_speed_segments_per_sec: float,
    hard_stop_fast_blink_hz: float,
    resume_duration: float,
    resume_start_sec: float | None,
) -> LedFrame | None:
    if segment_count <= 0:
        raise ValueError('segment_count must be positive')

    if internal_state == 'ConflictAvoidingNavigate':
        return None
    if internal_state == 'HardStop':
        enabled = _square_wave(now_sec, hard_stop_fast_blink_hz)
        return _steady_frame('hard_stop', 'none', RED, 1.0 if enabled else 0.08, segment_count)
    if internal_state == 'Wait':
        return _steady_frame('wait', 'none', GREEN, 0.85, segment_count)
    if internal_state == 'ConflictAvoid':
        if motion_direction == 'backward':
            return _flow_frame('yield_flow', 'backward', now_sec, segment_count, flow_speed_segments_per_sec)
        if motion_direction == 'left_turn':
            return _flow_frame('yield_flow', 'counter_clockwise', now_sec, segment_count, flow_speed_segments_per_sec)
        if motion_direction == 'right_turn':
            return _flow_frame('yield_flow', 'clockwise', now_sec, segment_count, flow_speed_segments_per_sec)
        return _steady_frame('yield_hold', 'none', GREEN, 0.85, segment_count)
    if internal_state == 'Forward' and is_resuming:
        return _resume_frame(now_sec, resume_start_sec, resume_duration, segment_count)
    if internal_state == 'Idle':
        return _steady_frame('idle', 'none', WHITE, 0.25, segment_count)
    return _steady_frame('normal', 'none', WHITE, 0.70, segment_count)


def _steady_frame(mode: str, direction: str, color: tuple[float, float, float], intensity: float, count: int) -> LedFrame:
    segment = SegmentColor(color[0], color[1], color[2], _clamp01(intensity))
    return LedFrame(mode=mode, direction=direction, segments=tuple(segment for _ in range(count)))


def _resume_frame(now_sec: float, start_sec: float | None, duration: float, count: int) -> LedFrame:
    if start_sec is None:
        progress = 0.0
    else:
        progress = _clamp01((now_sec - start_sec) / max(duration, 1e-6))
    color = _blend(GREEN, WHITE, progress)
    return _steady_frame('resume', 'none', color, 0.75, count)


def _flow_frame(
    mode: str,
    direction: str,
    now_sec: float,
    count: int,
    speed_segments_per_sec: float,
) -> LedFrame:
    speed = max(speed_segments_per_sec, 0.0)
    phase = (now_sec * speed) % float(count)
    if direction == 'clockwise':
        heads = [(-phase) % count]
    elif direction == 'counter_clockwise':
        heads = [phase]
    else:
        half_count = max(count / 2.0, 1.0)
        backward_phase = (now_sec * speed) % half_count
        heads = [backward_phase, (-backward_phase) % count]

    segments = []
    for index in range(count):
        peak = max(_circular_peak(index, head, count, tail_width=3.0) for head in heads)
        intensity = 0.18 + 0.82 * peak
        segments.append(SegmentColor(GREEN[0], GREEN[1], GREEN[2], _clamp01(intensity)))
    return LedFrame(mode=mode, direction=direction, segments=tuple(segments))


def _circular_peak(index: int, head: float, count: int, tail_width: float) -> float:
    distance = abs((index - head + count / 2.0) % count - count / 2.0)
    return _clamp01(1.0 - distance / max(tail_width, 1e-6))


def _square_wave(now_sec: float, hz: float) -> bool:
    if hz <= 0.0:
        return True
    return math.sin(now_sec * hz * math.tau) >= 0.0


def _blend(a: tuple[float, float, float], b: tuple[float, float, float], t: float) -> tuple[float, float, float]:
    t = _clamp01(t)
    return (
        a[0] * (1.0 - t) + b[0] * t,
        a[1] * (1.0 - t) + b[1] * t,
        a[2] * (1.0 - t) + b[2] * t,
    )


def _clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))
