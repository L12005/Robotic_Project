from hmi_feedback.material_refresh import should_force_material_refresh
from hmi_feedback.gazebo_scene_info import parse_visual_ids_from_scene_text
from hmi_feedback.sound_cues import SoundCueTracker, default_sound_asset_path


def test_material_refresh_forces_before_initial_sync() -> None:
    assert should_force_material_refresh(
        now_sec=10.0,
        initial_sync_complete=False,
        force_refresh_until_sec=0.0,
        last_full_refresh_sec=9.9,
        steady_refresh_sec=1.0,
    )


def test_material_refresh_forces_during_startup_window() -> None:
    assert should_force_material_refresh(
        now_sec=10.0,
        initial_sync_complete=True,
        force_refresh_until_sec=11.0,
        last_full_refresh_sec=9.9,
        steady_refresh_sec=1.0,
    )


def test_material_refresh_forces_after_steady_interval() -> None:
    assert should_force_material_refresh(
        now_sec=10.0,
        initial_sync_complete=True,
        force_refresh_until_sec=0.0,
        last_full_refresh_sec=8.9,
        steady_refresh_sec=1.0,
    )


def test_material_refresh_skips_before_steady_interval() -> None:
    assert not should_force_material_refresh(
        now_sec=10.0,
        initial_sync_complete=True,
        force_refresh_until_sec=0.0,
        last_full_refresh_sec=9.5,
        steady_refresh_sec=1.0,
    )


def test_material_refresh_can_disable_steady_interval() -> None:
    assert not should_force_material_refresh(
        now_sec=10.0,
        initial_sync_complete=True,
        force_refresh_until_sec=0.0,
        last_full_refresh_sec=0.0,
        steady_refresh_sec=0.0,
    )


def test_parse_visual_ids_from_scene_text_filters_robot_base_link() -> None:
    scene_text = '''
model {
  name: "other_model"
  link {
    name: "base_link"
    visual {
      name: "led_segment_00"
      id: 99
    }
  }
}
model {
  name: "starship_delivery_robot_model"
  link {
    name: "base_link"
    visual {
      name: "led_segment_00"
      id: 11
    }
    visual {
      name: "led_segment_01"
      id: 12
    }
  }
}
'''
    assert parse_visual_ids_from_scene_text(
        scene_text,
        model_name='starship_delivery_robot_model',
        link_name='base_link',
    ) == {
        'led_segment_00': 11,
        'led_segment_01': 12,
    }


def test_sound_tracker_only_plays_soft_chime_once_per_actor() -> None:
    tracker = SoundCueTracker()

    first = tracker.evaluate(
        internal_state='ConflictAvoidingNavigate',
        avoidance_started_event=False,
        human_actor_id='pedestrian_alpha',
    )
    second = tracker.evaluate(
        internal_state='Forward',
        avoidance_started_event=False,
        human_actor_id='pedestrian_alpha',
    )
    third = tracker.evaluate(
        internal_state='ConflictAvoidingNavigate',
        avoidance_started_event=False,
        human_actor_id='pedestrian_alpha',
    )

    assert first.play_soft_chime is True
    assert first.soft_chime_actor_id == 'pedestrian_alpha'
    assert second.play_soft_chime is False
    assert third.play_soft_chime is False


def test_sound_tracker_allows_soft_chime_for_new_actor() -> None:
    tracker = SoundCueTracker()
    tracker.evaluate(
        internal_state='ConflictAvoidingNavigate',
        avoidance_started_event=False,
        human_actor_id='pedestrian_alpha',
    )

    decision = tracker.evaluate(
        internal_state='ConflictAvoid',
        avoidance_started_event=True,
        human_actor_id='pedestrian_beta',
    )

    assert decision.play_soft_chime is True
    assert decision.soft_chime_actor_id == 'pedestrian_beta'


def test_sound_tracker_only_plays_hard_stop_warning_on_entry() -> None:
    tracker = SoundCueTracker()

    first = tracker.evaluate(
        internal_state='HardStop',
        avoidance_started_event=False,
        human_actor_id='pedestrian_alpha',
    )
    second = tracker.evaluate(
        internal_state='HardStop',
        avoidance_started_event=False,
        human_actor_id='pedestrian_alpha',
    )

    assert first.play_hard_stop_warning is True
    assert second.play_hard_stop_warning is False


def test_default_sound_asset_path_points_to_workspace_sound_directory() -> None:
    sound_path = default_sound_asset_path('softsound.mp3')

    assert sound_path.name == 'softsound.mp3'
    assert sound_path.parent.name == 'sound'
