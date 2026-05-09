from hmi_feedback.material_refresh import should_force_material_refresh
from hmi_feedback.gazebo_scene_info import parse_visual_ids_from_scene_text


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
