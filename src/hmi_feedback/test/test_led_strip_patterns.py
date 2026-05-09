from hmi_feedback.led_strip_patterns import build_led_frame, select_display_frame


def test_conflict_avoid_backward_uses_green_flow() -> None:
    frame = build_led_frame(
        internal_state='ConflictAvoid',
        motion_direction='backward',
        is_resuming=False,
        now_sec=0.0,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
        resume_start_sec=None,
    )

    assert frame is not None
    assert frame.mode == 'yield_flow'
    assert frame.direction == 'backward'
    assert max(segment.intensity for segment in frame.segments) > min(segment.intensity for segment in frame.segments)
    assert all(segment.green > 0.0 and segment.red == 0.0 for segment in frame.segments)


def test_conflict_avoiding_navigate_holds_previous_light() -> None:
    frame = build_led_frame(
        internal_state='ConflictAvoidingNavigate',
        motion_direction='none',
        is_resuming=False,
        now_sec=0.0,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
        resume_start_sec=None,
    )

    assert frame is None


def test_select_display_frame_uses_previous_when_candidate_holds() -> None:
    previous = build_led_frame(
        internal_state='Wait',
        motion_direction='none',
        is_resuming=False,
        now_sec=0.0,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
        resume_start_sec=None,
    )

    assert previous is not None
    frame = select_display_frame(
        None,
        previous,
        now_sec=1.0,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
    )

    assert frame is previous


def test_select_display_frame_uses_green_startup_fallback_without_previous() -> None:
    frame = select_display_frame(
        None,
        None,
        now_sec=0.0,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
    )

    assert frame.mode == 'yield_hold'
    assert frame.direction == 'none'
    assert all(segment.red == 0.0 for segment in frame.segments)
    assert all(segment.green == 1.0 for segment in frame.segments)
    assert all(segment.blue == 0.25 for segment in frame.segments)


def test_resume_blends_green_toward_white() -> None:
    start = build_led_frame(
        internal_state='Forward',
        motion_direction='forward',
        is_resuming=True,
        now_sec=10.0,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
        resume_start_sec=10.0,
    )
    end = build_led_frame(
        internal_state='Forward',
        motion_direction='forward',
        is_resuming=True,
        now_sec=11.2,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
        resume_start_sec=10.0,
    )

    assert start is not None and end is not None
    assert start.segments[0].red < end.segments[0].red
    assert start.segments[0].blue < end.segments[0].blue
    assert end.segments[0].red > 0.999
    assert end.segments[0].green > 0.999
    assert end.segments[0].blue > 0.999


def test_wait_uses_full_brightness_green_steady() -> None:
    frame = build_led_frame(
        internal_state='Wait',
        motion_direction='none',
        is_resuming=False,
        now_sec=0.0,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
        resume_start_sec=None,
    )

    assert frame is not None
    assert frame.mode == 'wait'
    assert all(segment.red == 0.0 for segment in frame.segments)
    assert all(segment.green == 1.0 for segment in frame.segments)
    assert all(segment.blue == 0.25 for segment in frame.segments)
    assert all(segment.intensity == 1.0 for segment in frame.segments)


def test_forward_uses_white_steady_without_flow() -> None:
    frame = build_led_frame(
        internal_state='Forward',
        motion_direction='forward',
        is_resuming=False,
        now_sec=0.0,
        segment_count=24,
        flow_speed_segments_per_sec=8.0,
        hard_stop_fast_blink_hz=3.0,
        resume_duration=1.2,
        resume_start_sec=None,
    )

    assert frame is not None
    assert frame.mode == 'normal'
    assert frame.direction == 'none'
    assert all(segment.red == 1.0 for segment in frame.segments)
    assert all(segment.green == 1.0 for segment in frame.segments)
    assert all(segment.blue == 1.0 for segment in frame.segments)
