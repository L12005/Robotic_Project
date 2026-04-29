from .robot_state import SceneState


class SceneClassifier:
    def __init__(self, default_scene_type: str = "open_area") -> None:
        self._default_scene_type = default_scene_type

    def classify(self, scene_state: SceneState) -> str:
        del scene_state
        return self._default_scene_type
